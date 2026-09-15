//! The multi-cylinder block: one master cylinder, a 720-cell phase ring, and
//! the manifolds and friction model that turn its trace into shaft torque.
//!
//! # Why one cylinder
//!
//! Every cylinder in an inline or vee engine sees the *same* physics on the
//! *same* geometry — it is only displaced in phase by its firing offset. So
//! rather than running eight independent solvers, the block integrates a single
//! master cylinder at full fidelity and writes its state into a 720-cell
//! historical ring indexed by crank degree. Cylinder `i` is then read back out
//! of that ring at its own phase:
//!
//! ```text
//! Index_i = floor( deg(theta_master - dtheta_firing_i) mod 720 ) + 1
//! ```
//!
//! The cost of a V8 is one solver plus eight array reads. The trade is that all
//! cylinders share a cycle: per-cylinder scatter (a weak injector, one hot
//! exhaust valve) is not represented, which is the right compromise for a
//! real-time audio/telemetry model and the wrong one for a design tool.
//!
//! # What comes out
//!
//! Indicated torque is summed over the cylinders at their own phases, and net
//! brake torque follows from the Chen-Flynn friction correlation. The exhaust
//! side switches between a lumped plenum and a segmented 1D acoustic pipe
//! depending on whether the pipe's reflection time lines up with the firing
//! interval — see [`ManifoldTransit`].

use std::f64::consts::PI;

use crate::environment::Environment;
use crate::physics::control::{CylinderHealth, EngineControlUnit, LimiterCut};
use crate::physics::cylinder::{deg, wrap_cycle, CylinderGeometry, GasProperties, CYCLE_ANGLE};
use crate::physics::plumbing::{ExhaustSystem, IntakeSystem};
use crate::physics::thermal::{EngineThermal, OilViscosity};
use crate::physics::thermodynamics::{
    CylinderModel, HeatRelease, PortConditions, Rk4Solver, StepReport, ThermoState,
};

/// One cell per crank degree over the full four-stroke cycle.
pub const PHASE_CELLS: usize = 720;

/// Points in the downsampled cycle handed to the audio thread.
///
/// The audio thread plays the solver's own curves back at crank rate, so the
/// table has to resolve the fastest thing in the cycle: the exhaust valve
/// cracking open, which takes some tens of crank degrees. 128 points is 5.625
/// degrees apiece — a handful of samples across that edge, which is enough to
/// carry its slope — and the three tables together are 1.5 kB per frame, which
/// is what keeps [`crate::audio::dsp::EngineSnapshot`] small enough to keep
/// shovelling through a lock-free queue.
pub const CYCLE_TABLE: usize = 128;

// ---------------------------------------------------------------------------
// Phase ring buffer
// ---------------------------------------------------------------------------

/// One crank degree of logged master-cylinder state.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PhaseSample {
    /// Cylinder pressure [Pa].
    pub pressure: f64,
    /// Bulk gas temperature [K].
    pub temperature: f64,
    /// Trapped mass [kg].
    pub mass: f64,
    /// Cylinder volume [m^3].
    pub volume: f64,
    /// `dV/dtheta` at this angle [m^3/rad].
    pub dvolume_dtheta: f64,
    /// Burned mass fraction [-].
    pub burned_fraction: f64,
    /// Livengood-Wu integral [-].
    pub knock_integral: f64,
    /// Intake port flux, positive into the cylinder [kg/s].
    pub intake_flow: f64,
    /// Exhaust port flux, positive into the cylinder [kg/s].
    pub exhaust_flow: f64,
    /// Combustion heat release rate [W].
    pub heat_release: f64,
    /// Wall heat loss rate [W].
    pub wall_loss: f64,
}

impl PhaseSample {
    /// Indicated torque this cylinder contributes at this angle [N m].
    ///
    /// `tau = (P_cyl - P_crankcase) * dV/dtheta`. The crankcase term matters:
    /// during the intake stroke the underside of the piston is what makes
    /// pumping loss show up as negative torque instead of nothing at all.
    pub fn indicated_torque(&self, crankcase_pressure: f64) -> f64 {
        (self.pressure - crankcase_pressure) * self.dvolume_dtheta
    }
}

/// A 720-cell historical ring of master-cylinder states, one per crank degree.
///
/// Writes go in at the master's phase; reads come out at any phase, which is
/// what lets a virtual cylinder look up "what the master looked like 270 crank
/// degrees ago" without storing anything of its own.
#[derive(Debug, Clone, PartialEq)]
pub struct PhaseRing {
    cells: Vec<PhaseSample>,
    written: Vec<bool>,
    filled: usize,
}

impl Default for PhaseRing {
    fn default() -> Self {
        Self::new()
    }
}

impl PhaseRing {
    /// An empty ring; every cell reads back as zero until it is first written.
    pub fn new() -> Self {
        Self {
            cells: vec![PhaseSample::default(); PHASE_CELLS],
            written: vec![false; PHASE_CELLS],
            filled: 0,
        }
    }

    /// The 1-based phase index of a crank angle, as the block indexing rule
    /// defines it: `floor(deg(theta) mod 720) + 1`, so `1 ..= 720`.
    pub fn phase_index(theta: f64) -> usize {
        let degrees = wrap_cycle(theta).to_degrees();
        // Angles are carried in radians, so a whole-degree angle comes back from
        // the conversion a few ULP either side of the integer. Left alone, an
        // exact 4-degree stride would land on 3.9999999997 and floor into the
        // previous cell, drifting the ring index by one every few hundred steps
        // and leaving holes behind. Snapping the boundary makes the cell a
        // cylinder occupies a function of its angle alone.
        let snapped = if (degrees - degrees.round()).abs() < 1e-9 {
            degrees.round()
        } else {
            degrees
        };
        // wrap_cycle guarantees [0, 720), but the snap can push 719.9999 up to
        // a full 720, which belongs at the start of the next cycle. Wrapping is
        // the only correct answer here: clamping would strand cell 0.
        (snapped.floor().max(0.0) as usize) % PHASE_CELLS + 1
    }

    /// Zero-based storage slot for a crank angle.
    pub fn cell_of(theta: f64) -> usize {
        Self::phase_index(theta) - 1
    }

    /// Whether every cell has been written at least once (a full cycle logged).
    pub fn is_primed(&self) -> bool {
        self.filled == PHASE_CELLS
    }

    /// How many distinct cells hold real data.
    pub fn filled_cells(&self) -> usize {
        self.filled
    }

    /// Writes a single cell.
    pub fn record(&mut self, theta: f64, sample: PhaseSample) {
        let cell = Self::cell_of(theta);
        if !self.written[cell] {
            self.written[cell] = true;
            self.filled += 1;
        }
        self.cells[cell] = sample;
    }

    /// Writes every cell a substep passed through.
    ///
    /// With the adaptive budget stretching `delta_theta` to as much as 4
    /// degrees, a substep can jump several cells at once. Filling the whole span
    /// keeps the ring hole-free: a virtual cylinder reading a skipped cell would
    /// otherwise see a stale sample from the previous cycle and contribute a
    /// torque spike out of nowhere.
    pub fn record_span(&mut self, previous_theta: f64, theta: f64, sample: PhaseSample) {
        let from = Self::cell_of(previous_theta);
        let to = Self::cell_of(theta);
        let span = (to + PHASE_CELLS - from) % PHASE_CELLS;

        if span == 0 {
            // Substep finished inside the cell it started in.
            self.record(theta, sample);
            return;
        }
        for k in 1..=span {
            let cell = (from + k) % PHASE_CELLS;
            if !self.written[cell] {
                self.written[cell] = true;
                self.filled += 1;
            }
            self.cells[cell] = sample;
        }
    }

    /// Reads the cell covering a crank angle.
    pub fn sample(&self, theta: f64) -> PhaseSample {
        self.cells[Self::cell_of(theta)]
    }

    /// Reads a cell by its zero-based slot.
    pub fn cell(&self, index: usize) -> PhaseSample {
        self.cells[index % PHASE_CELLS]
    }

    /// The whole ring, cell 0 first (0 to 1 crank degree).
    pub fn cells(&self) -> &[PhaseSample] {
        &self.cells
    }

    /// Downsamples one field of the logged cycle onto a fixed-size table.
    ///
    /// Cell zero of the result covers `origin`, so the caller chooses where the
    /// cycle is cut. The audio thread cuts it at exhaust valve opening, which
    /// makes the table's own index the phase a cylinder is at relative to its
    /// firing event and saves the playback path a rotation per read.
    ///
    /// Each output point is the *mean* of the ring cells it spans, not the cell
    /// that happens to land on it. 720 into 128 is 5.625 cells per point, so
    /// plain decimation would step between taking five cells and six and would
    /// fold everything above the new Nyquist back down — on a blowdown edge that
    /// is precisely the content being carried. Box-averaging is the cheapest
    /// anti-alias filter that is honest about it.
    ///
    /// Point `k` is therefore the cycle's mean over `[k, k+1) / CYCLE_TABLE` of
    /// a cycle, and *stands at the centre of that span*. A player reading the
    /// table at cycle phase `p` wants `p * CYCLE_TABLE - 0.5` as its fractional
    /// index; reading it at `p * CYCLE_TABLE` would advance the whole cycle by
    /// half a point, which at 128 points is 2.8 crank degrees of latency on
    /// every event in it.
    pub fn downsample_from(
        &self,
        origin: f64,
        field: impl Fn(&PhaseSample) -> f64,
    ) -> [f32; CYCLE_TABLE] {
        let first = Self::cell_of(origin);
        let mut table = [0.0f32; CYCLE_TABLE];
        for (k, out) in table.iter_mut().enumerate() {
            let from = k * PHASE_CELLS / CYCLE_TABLE;
            let to = (k + 1) * PHASE_CELLS / CYCLE_TABLE;
            let mut sum = 0.0;
            for cell in from..to {
                sum += field(&self.cells[(first + cell) % PHASE_CELLS]);
            }
            *out = (sum / (to - from) as f64) as f32;
        }
        table
    }

    /// Downsamples a function of crank angle onto the same table.
    ///
    /// The companion to [`PhaseRing::downsample_from`] for quantities that are
    /// a pure function of angle and therefore have nothing to log — valve lift
    /// above all. Sharing the origin, the box average and the half-cell
    /// convention is the point: a valve area table read at the phase a pressure
    /// table is read at has to describe the same instant, or the port opens at
    /// a different moment from the pulse that comes out of it.
    pub fn downsample_angles_from(origin: f64, field: impl Fn(f64) -> f64) -> [f32; CYCLE_TABLE] {
        let first = Self::cell_of(origin);
        let mut table = [0.0f32; CYCLE_TABLE];
        for (k, out) in table.iter_mut().enumerate() {
            let from = k * PHASE_CELLS / CYCLE_TABLE;
            let to = (k + 1) * PHASE_CELLS / CYCLE_TABLE;
            let mut sum = 0.0;
            for cell in from..to {
                // The centre of the cell, because a cell stands for the degree
                // it spans and the function being sampled is continuous.
                let theta = (((first + cell) % PHASE_CELLS) as f64 + 0.5) * CYCLE_ANGLE
                    / PHASE_CELLS as f64;
                sum += field(theta);
            }
            *out = (sum / (to - from) as f64) as f32;
        }
        table
    }

    /// Cycle mean of one field of the logged cycle.
    ///
    /// The cells are uniform in crank angle and a cycle is uniform in time at a
    /// fixed speed, so for a rate this is the mean rate over the cycle — which
    /// is what a thermal mass a hundred seconds long wants to be driven by,
    /// rather than by whatever the flux happens to be at this instant.
    pub fn cycle_mean(&self, field: impl Fn(&PhaseSample) -> f64) -> f64 {
        self.cells.iter().map(field).sum::<f64>() / PHASE_CELLS as f64
    }

    /// Highest pressure anywhere in the logged cycle [Pa].
    pub fn peak_pressure(&self) -> f64 {
        self.cells.iter().fold(0.0f64, |a, c| a.max(c.pressure))
    }

    /// Crank angle at which cylinder pressure peaks [deg, 0..720].
    pub fn peak_pressure_angle(&self) -> f64 {
        let mut best_i = 0;
        let mut best_p = 0.0f64;
        for (i, cell) in self.cells.iter().enumerate() {
            if cell.pressure > best_p {
                best_p = cell.pressure;
                best_i = i;
            }
        }
        best_i as f64 + 0.5
    }

    /// Net indicated work over the logged cycle, `integral P dV` [J].
    ///
    /// Trapezoidal over the 720 cells with `dV = (dV/dtheta) dtheta`, which is
    /// exact enough at 1-degree resolution for an IMEP that agrees with the
    /// solver's own work integral to well under a percent.
    pub fn indicated_work(&self, crankcase_pressure: f64) -> f64 {
        let dtheta = CYCLE_ANGLE / PHASE_CELLS as f64;
        let mut work = 0.0;
        for i in 0..PHASE_CELLS {
            let a = self.cells[i].indicated_torque(crankcase_pressure);
            let b = self.cells[(i + 1) % PHASE_CELLS].indicated_torque(crankcase_pressure);
            work += 0.5 * (a + b) * dtheta;
        }
        work
    }
}

// ---------------------------------------------------------------------------
// Firing order
// ---------------------------------------------------------------------------

/// One cylinder's identity and phase offset within the block.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CylinderIndex {
    /// Cylinder number as cast into the block, 1-based.
    pub number: u8,
    /// Which bank it sits on; 0 and 1 for a vee, all 0 for an inline.
    pub bank: u8,
    /// How far this cylinder lags the master, `dtheta_firing_i` [rad].
    pub firing_offset: f64,
}

/// Bank angle from the block's own reference axis, one side of a vee [rad].
///
/// A single-bank engine is upright: every cylinder's bore axis coincides with
/// the block's own reference axis, angle zero. Every vee this crate builds is
/// a 90-degree vee — nothing here records a shallower or wider one — so a
/// second bank splits symmetrically at plus or minus 45 degrees. This is the
/// one geometric fact [`FiringOrder::shaking_force`] rests on beyond firing
/// order and phase, and the reason a crossplane and a flatplane V8, which
/// share both bank angle and bank split, can still differ in the force they
/// put into the block: the two banks' phase relationship, not the angle, is
/// what changes between them.
pub fn bank_angle(bank: u8, bank_count: usize) -> f64 {
    if bank_count <= 1 {
        return 0.0;
    }
    let half_vee = std::f64::consts::FRAC_PI_4;
    if bank == 0 {
        -half_vee
    } else {
        half_vee
    }
}

/// The firing arrangement of a whole block.
#[derive(Debug, Clone, PartialEq)]
pub struct FiringOrder {
    /// Every cylinder, in cast order (cylinder 1 first).
    pub cylinders: Vec<CylinderIndex>,
    /// Nominal crank angle between successive firings [rad].
    pub interval: f64,
    /// Human-readable order, e.g. `[1, 8, 7, 2, 6, 5, 4, 3]`.
    pub sequence: Vec<u8>,
}

impl FiringOrder {
    /// Builds a firing order from a sequence of cylinder numbers.
    ///
    /// Cylinder `k` in the sequence fires `k * 720 / n` degrees after the first,
    /// which is the even-firing crank spacing. Bank assignment is supplied by
    /// the caller because it is a property of the block casting, not the crank.
    pub fn new(sequence: &[u8], banks: &dyn Fn(u8) -> u8) -> Self {
        let n = sequence.len().max(1);
        let interval = CYCLE_ANGLE / n as f64;

        let mut cylinders: Vec<CylinderIndex> = sequence
            .iter()
            .enumerate()
            .map(|(slot, &number)| CylinderIndex {
                number,
                bank: banks(number),
                firing_offset: interval * slot as f64,
            })
            .collect();
        cylinders.sort_by_key(|c| c.number);

        Self {
            cylinders,
            interval,
            sequence: sequence.to_vec(),
        }
    }

    /// Number of cylinders.
    pub fn len(&self) -> usize {
        self.cylinders.len()
    }

    /// Whether the block has no cylinders (only reachable by constructing one).
    pub fn is_empty(&self) -> bool {
        self.cylinders.is_empty()
    }

    /// How many distinct banks the block has.
    pub fn bank_count(&self) -> usize {
        self.cylinders
            .iter()
            .map(|c| c.bank as usize)
            .max()
            .map_or(0, |m| m + 1)
    }

    /// Indices of the cylinders on one bank, in cylinder order.
    pub fn cylinders_on_bank(&self, bank: u8) -> Vec<usize> {
        self.cylinders
            .iter()
            .enumerate()
            .filter(|(_, c)| c.bank == bank)
            .map(|(i, _)| i)
            .collect()
    }

    /// Firing offsets on one bank, sorted into cycle order [rad].
    pub fn bank_offsets(&self, bank: u8) -> Vec<f64> {
        let mut offsets: Vec<f64> = self
            .cylinders
            .iter()
            .filter(|c| c.bank == bank)
            .map(|c| c.firing_offset)
            .collect();
        offsets.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        offsets
    }

    /// Mean crank angle between exhaust pulses arriving in one bank's collector
    /// [rad].
    ///
    /// For a cross-plane V8 the *instantaneous* gaps on a bank are 90-180-270-180
    /// degrees, not a constant 180 — that unevenness is the entire reason the
    /// engine burbles instead of humming. The mean is what the manifold tuning
    /// rule is calibrated against; [`FiringOrder::bank_gaps`] exposes the real
    /// spacing for anyone who needs it.
    pub fn mean_bank_interval(&self, bank: u8) -> f64 {
        let count = self.bank_offsets(bank).len();
        if count == 0 {
            return CYCLE_ANGLE;
        }
        CYCLE_ANGLE / count as f64
    }

    /// The actual crank-angle gaps between successive firings on one bank [rad].
    pub fn bank_gaps(&self, bank: u8) -> Vec<f64> {
        let offsets = self.bank_offsets(bank);
        if offsets.is_empty() {
            return Vec::new();
        }
        (0..offsets.len())
            .map(|i| {
                let next = offsets[(i + 1) % offsets.len()];
                let gap = next - offsets[i];
                if gap <= 0.0 {
                    gap + CYCLE_ANGLE
                } else {
                    gap
                }
            })
            .collect()
    }

    /// Reciprocating shaking-force resultant the whole block feels this
    /// instant, resolved onto the block's own in-plane axes [N]: `x` across
    /// the vee, `y` along the axis a single-bank engine's cylinders all share.
    ///
    /// Every cylinder's own inertia force acts along its own bore axis only;
    /// this reads each one out of `geometry` at its own crank phase — the same
    /// `theta_master - dtheta_firing_i` convention the module doc above uses to
    /// index the phase ring — and projects it through [`bank_angle`] before
    /// summing, the same way [`crate::audio`]'s per-sample synthesis sums
    /// `pressure_slope_at` over the cylinders. A single bank's cylinders all
    /// share one axis, so this collapses to a plain sum along `y`; a vee's two
    /// banks point apart, and it is that projection — not the firing order —
    /// that lets a crossplane and a flatplane V8 differ here.
    pub fn shaking_force(&self, geometry: &CylinderGeometry, theta: f64, omega: f64) -> (f64, f64) {
        let bank_count = self.bank_count();
        let (mut x, mut y) = (0.0, 0.0);
        for cylinder in &self.cylinders {
            let force = geometry.inertia_force(theta - cylinder.firing_offset, omega);
            let angle = bank_angle(cylinder.bank, bank_count);
            x += force * angle.sin();
            y += force * angle.cos();
        }
        (x, y)
    }

    /// Cross-plane V8, firing order 1-8-7-2-6-5-4-3.
    ///
    /// The 90-degree crank throws give even 90-degree firing *overall*, but the
    /// odd/even bank split (1-3-5-7 left, 2-4-6-8 right) leaves each bank with
    /// the uneven 90-180-270-180 pattern that produces the characteristic
    /// American V8 burble.
    pub fn cross_plane_v8() -> Self {
        Self::new(&[1, 8, 7, 2, 6, 5, 4, 3], &|n| u8::from(n % 2 == 0))
    }

    /// Flat-plane V8, firing order 1-5-3-7-4-8-2-6: each bank fires evenly at
    /// 180 degrees, which is why it sounds like two inline-fours sharing a crank.
    ///
    /// Note the numbering convention differs from the cross-plane block above:
    /// flat-plane vees are conventionally numbered 1-4 down one bank and 5-8
    /// down the other, not odd/even across the vee. Pairing this firing order
    /// with an odd/even bank split would produce uneven banks and quietly turn a
    /// flat-plane engine back into a cross-plane one.
    pub fn flat_plane_v8() -> Self {
        Self::new(&[1, 5, 3, 7, 4, 8, 2, 6], &|n| u8::from(n > 4))
    }

    /// Inline four, firing order 1-3-4-2, single bank.
    pub fn inline_four() -> Self {
        Self::new(&[1, 3, 4, 2], &|_| 0)
    }

    /// Inline six, firing order 1-5-3-6-2-4, single bank.
    ///
    /// Six throws at 120 degrees fire every 120 degrees of crank into one
    /// collector, evenly, with no gap anywhere in the cycle — the reason a
    /// straight six sounds continuous where a four sounds like four separate
    /// events. There is only one bank, so unlike a vee there is no second
    /// pulse train to beat against.
    pub fn inline_six() -> Self {
        Self::new(&[1, 5, 3, 6, 2, 4], &|_| 0)
    }

    /// Boxer six (flat six), firing order 1-6-2-4-3-5 across two opposing banks.
    ///
    /// Cylinders 1-3 are bank 0 (left) and 4-6 are bank 1 (right). Over 720 degrees
    /// of four-stroke cycle, six cylinders fire every 120 degrees overall, and
    /// firings alternate strictly between banks: 1 (bank 0), 6 (bank 1), 2 (bank 0),
    /// 4 (bank 1), 3 (bank 0), 5 (bank 1). Each bank therefore experiences an even
    /// 240-degree pulse interval into its collector — the geometric reason a
    /// Porsche flat-six sings in clean harmonic triads rather than burbling.
    pub fn boxer_six() -> Self {
        Self::new(&[1, 6, 2, 4, 3, 5], &|n| u8::from(n > 3))
    }

    /// 90-degree V10, firing order 1-10-9-4-3-6-5-8-7-2.
    ///
    /// Ten cylinders over 720 degrees is a 72-degree firing interval, which a
    /// 90-degree vee cannot deliver from evenly spaced throws — real V10s split
    /// each crankpin by 18 degrees to get there. The 0D block has no crankpin,
    /// only phase offsets, so the offsets are simply the even 72 degrees the
    /// split is designed to produce. Cylinders 1-5 are the left bank, 6-10 the
    /// right, which leaves each bank firing at an uneven 144-72-144-144-216
    /// pattern — the reason a V10 has a harder, less symmetrical edge than a
    /// V12.
    pub fn v10() -> Self {
        Self::new(&[1, 10, 9, 4, 3, 6, 5, 8, 7, 2], &|n| u8::from(n > 5))
    }

    /// 60-degree V12, firing order 1-12-4-9-2-11-6-7-3-10-5-8.
    ///
    /// A 60-degree vee with six throws fires every 60 degrees overall and,
    /// unusually, evenly on each bank too: bank 0 gets a clean 120-degree
    /// spacing, as does bank 1. That is why a V12 sounds like two smooth
    /// inline-sixes rather than like anything that burbles.
    pub fn v12() -> Self {
        Self::new(&[1, 12, 4, 9, 2, 11, 6, 7, 3, 10, 5, 8], &|n| {
            u8::from(n > 6)
        })
    }

    /// Two-rotor Wankel, mapped onto four evenly spaced firings.
    ///
    /// A rotary has no cylinders to order, so this is an approximation, and it
    /// is worth being precise about which one. Each rotor completes one power
    /// stroke per turn of the eccentric shaft. The block's cycle is 720 degrees
    /// of shaft rotation, so in one cycle each rotor fires twice and the engine
    /// fires four times, evenly, at 180-degree intervals. Four phase slots
    /// reproduce that firing pattern exactly: chambers 1 and 2 are the front
    /// rotor at 0 and 360 degrees, 3 and 4 the rear rotor at 180 and 540.
    ///
    /// What it does *not* reproduce is the chamber itself. A Wankel's volume
    /// curve comes from an epitrochoid, not a slider-crank, and its combustion
    /// chamber is long, thin and moving, which is why rotaries burn slowly and
    /// run cool-headed but hot-exhausted. Those differences live in the
    /// [`CylinderModel`] a caller pairs with this order — a long rod ratio to
    /// flatten the volume curve, a stretched Wiebe duration, a low compression
    /// ratio — not in the firing order.
    ///
    /// [`CylinderModel`]: crate::physics::thermodynamics::CylinderModel
    pub fn two_rotor_wankel() -> Self {
        Self::new(&[1, 3, 2, 4], &|_| 0)
    }

    /// A single cylinder: one firing per two revolutions, and nothing else.
    ///
    /// The degenerate case, and the interesting one. Every other order in this
    /// catalogue spreads its firings around the cycle so that the crank is
    /// being pushed somewhere in it at almost all times; a single is pushed
    /// once, hard, over about sixty degrees, and then dragged round the
    /// remaining six hundred and sixty by whatever the flywheel kept. That is
    /// what makes the intra-cycle speed ripple of Stage 1c audible as the
    /// engine's *character* rather than as a subtlety — there is nothing else
    /// in the cycle to mask it.
    pub fn single() -> Self {
        Self::new(&[1], &|_| 0)
    }
}

// ---------------------------------------------------------------------------
// Chen-Flynn friction
// ---------------------------------------------------------------------------

/// Chen-Flynn friction mean effective pressure correlation.
///
/// ```text
/// FMEP = A + B * P_max + C * S_p + D * S_p^2      [bar]
/// ```
///
/// The four terms map onto four physical mechanisms: `A` is the speed- and
/// load-independent drag of the seals and accessories, `B * P_max` is the ring
/// and bearing load that scales with peak cylinder pressure, `C * S_p` is
/// hydrodynamic shear in the bearings, and `D * S_p^2` is the windage and
/// pumping that grows with the square of piston speed.
///
/// The coefficients are quoted for an engine at operating temperature, which is
/// the only condition anybody measures them at and is why a simulator built on
/// them alone has no cold engine. The two piston-speed terms are the shear ones:
/// they are the ones the oil's viscosity is in, so they are the ones
/// [`OilViscosity`] scales. `A` is accessories and seal rub and `B * P_max` is
/// ring load, and neither is shearing a full film.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChenFlynn {
    /// Constant term `A` [bar].
    pub constant: f64,
    /// Peak-pressure coefficient `B` [-].
    pub peak_pressure_factor: f64,
    /// Mean-piston-speed coefficient `C` [bar s/m].
    pub piston_speed_factor: f64,
    /// Squared-piston-speed coefficient `D` [bar s^2/m^2].
    pub piston_speed_squared_factor: f64,
    /// The oil the shear terms are shearing.
    pub oil: OilViscosity,
}

impl Default for ChenFlynn {
    /// Coefficients in the usual range for a warm production gasoline V8.
    fn default() -> Self {
        Self {
            constant: 0.35,
            peak_pressure_factor: 0.005,
            piston_speed_factor: 0.09,
            piston_speed_squared_factor: 0.0009,
            oil: OilViscosity::default(),
        }
    }
}

impl ChenFlynn {
    /// Friction mean effective pressure at an oil temperature [Pa].
    pub fn fmep(&self, peak_pressure: f64, mean_piston_speed: f64, oil_temperature: f64) -> f64 {
        let p_max_bar = (peak_pressure / 1e5).max(0.0);
        let sp = mean_piston_speed.abs();
        let viscous = self.oil.friction_multiplier(oil_temperature);
        let bar = self.constant
            + self.peak_pressure_factor * p_max_bar
            + viscous
                * (self.piston_speed_factor * sp + self.piston_speed_squared_factor * sp * sp);
        bar.max(0.0) * 1e5
    }

    /// Friction torque for a four-stroke engine [N m].
    ///
    /// An MEP is work per unit displaced volume per cycle, and a four-stroke
    /// cycle is `4 pi` radians of crank, so `tau = FMEP * V_total / (4 pi)`.
    pub fn torque(
        &self,
        peak_pressure: f64,
        mean_piston_speed: f64,
        total_displacement: f64,
        oil_temperature: f64,
    ) -> f64 {
        self.fmep(peak_pressure, mean_piston_speed, oil_temperature) * total_displacement
            / CYCLE_ANGLE
    }
}

// ---------------------------------------------------------------------------
// Manifolds: lumped plenum and 1D segmented acoustic pipe
// ---------------------------------------------------------------------------

/// A fixed-volume, well-stirred plenum.
///
/// Mass conservation plus the open-system first law at constant volume:
///
/// ```text
/// dm/dt      = sum_j m_dot_j
/// m c_v dT/dt = sum_j m_dot_j (h_j - u)
/// P          = m R T / V
/// ```
///
/// Zero-dimensional by construction: it has no length, so it cannot carry a
/// wave. That is exactly what [`ManifoldTransit`] switches away from when the
/// pipe starts to matter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Plenum {
    /// Plenum volume [m^3].
    pub volume: f64,
    /// Trapped mass [kg].
    pub mass: f64,
    /// Bulk temperature [K].
    pub temperature: f64,
    /// Specific gas constant of the contents [J/(kg K)].
    pub gas_constant: f64,
    /// Ratio of specific heats of the contents [-].
    pub gamma: f64,
}

impl Plenum {
    /// A plenum filled to a given pressure and temperature.
    pub fn new(
        volume: f64,
        pressure: f64,
        temperature: f64,
        gas_constant: f64,
        gamma: f64,
    ) -> Self {
        let volume = volume.max(1e-6);
        Self {
            volume,
            mass: pressure * volume / (gas_constant * temperature),
            temperature,
            gas_constant,
            gamma,
        }
    }

    /// Static pressure [Pa].
    pub fn pressure(&self) -> f64 {
        self.mass * self.gas_constant * self.temperature / self.volume
    }

    /// Constant-volume specific heat [J/(kg K)].
    fn cv(&self) -> f64 {
        self.gas_constant / (self.gamma - 1.0)
    }

    /// Mass that would move the plenum to `target_pressure` at fixed volume [kg].
    ///
    /// Positive means it has room to take that much more in.
    pub fn mass_to_reach(&self, target_pressure: f64) -> f64 {
        (target_pressure - self.pressure()) * self.volume
            / (self.gas_constant * self.temperature.max(1.0))
    }

    /// Largest flow that will not overshoot `target_pressure` in one step [kg/s].
    ///
    /// Both manifolds are driven by quasi-steady orifice equations integrated
    /// explicitly, and a plenum this small refills in well under a frame. Left
    /// uncapped, one Euler step can blow straight past the pressure it is
    /// chasing and leave a naturally aspirated engine sitting above ambient —
    /// silently supercharging itself, and by enough to matter: an 11 % intake
    /// overshoot is an 11 % torque error. The cap makes the equilibrium a
    /// property of the physics rather than of the frame rate.
    pub fn rate_limit(&self, target_pressure: f64, dt: f64) -> f64 {
        if !(dt.is_finite() && dt > 0.0) {
            return 0.0;
        }
        (self.mass_to_reach(target_pressure).abs() / dt).max(0.0)
    }

    /// Advances the plenum by `dt` under a net inflow.
    ///
    /// `inflow` is positive *into* the plenum and arrives at `inflow_temperature`;
    /// `outflow` leaves at the plenum's own temperature.
    pub fn integrate(&mut self, dt: f64, inflow: f64, inflow_temperature: f64, outflow: f64) {
        if !(dt.is_finite() && dt > 0.0) {
            return;
        }
        let cv = self.cv();
        let cp = self.gamma * cv;
        let mass = self.mass.max(1e-9);
        let u = cv * self.temperature;

        // Only the incoming stream carries a foreign enthalpy; what leaves is
        // already at the plenum state, so its (h - u) term is just R*T.
        let energy = inflow * (cp * inflow_temperature - u) - outflow * (cp * self.temperature - u);

        self.temperature = (self.temperature + dt * energy / (mass * cv)).clamp(1.0, 4000.0);
        self.mass = (self.mass + dt * (inflow - outflow)).max(1e-9);
    }
}

/// A 1D segmented acoustic pipe: the model that knows a runner has a length.
///
/// Pressure is split into a right-running and a left-running wave on a uniform
/// grid of `segments` cells. The grid is stepped at exactly unit CFL — one cell
/// per internal step of `tau_seg = L / (N c)` — which is the special case where
/// the discrete update is *exact* for the linear wave equation, no numerical
/// dispersion at all. Frame deltas that are not whole multiples of `tau_seg`
/// leave a remainder in the accumulator and are picked up next call.
///
/// A round trip is `2 N` internal steps, i.e.
///
/// ```text
/// tau_pulse = 2 N * tau_seg = 2 L / c_exhaust
/// ```
///
/// which is the transit time the switching rule is written against.
#[derive(Debug, Clone, PartialEq)]
pub struct AcousticPipe {
    /// Pipe length from valve to open end [m].
    pub length: f64,
    /// Cross-sectional area [m^2].
    pub area: f64,
    /// Per-segment amplitude retention, modelling wall friction [-].
    pub damping: f64,
    /// Magnitude of the open-end reflection [-]; the sign inversion is implicit.
    pub open_end_reflection: f64,
    /// Reflection at the valve end when the valve is shut (a closed end) [-].
    pub closed_end_reflection: f64,
    forward: Vec<f64>,
    backward: Vec<f64>,
    accumulator: f64,
}

impl AcousticPipe {
    /// A pipe discretised into `segments` cells, initially at rest.
    pub fn new(length: f64, area: f64, segments: usize) -> Self {
        let segments = segments.clamp(2, 512);
        Self {
            length: length.max(1e-3),
            area: area.max(1e-6),
            damping: 0.995,
            open_end_reflection: 0.8,
            closed_end_reflection: 0.95,
            forward: vec![0.0; segments],
            backward: vec![0.0; segments],
            accumulator: 0.0,
        }
    }

    /// Number of segments.
    pub fn segments(&self) -> usize {
        self.forward.len()
    }

    /// Round-trip reflection time `tau_pulse = 2 L / c` [s].
    pub fn transit_time(&self, speed_of_sound: f64) -> f64 {
        2.0 * self.length / speed_of_sound.max(1.0)
    }

    /// Fundamental resonance of the pipe, `1 / tau_pulse` [Hz].
    pub fn fundamental(&self, speed_of_sound: f64) -> f64 {
        1.0 / self.transit_time(speed_of_sound)
    }

    /// Pressure perturbation at the valve end [Pa].
    pub fn valve_end_perturbation(&self) -> f64 {
        self.forward[0] + self.backward[0]
    }

    /// Clears the wave field; used when switching modes so the pipe does not
    /// resume with a stale standing wave from the last time it was live.
    pub fn reset(&mut self) {
        self.forward.iter_mut().for_each(|v| *v = 0.0);
        self.backward.iter_mut().for_each(|v| *v = 0.0);
        self.accumulator = 0.0;
    }

    /// Advances the wave field by `dt`.
    ///
    /// `mass_flux` is the flow leaving the cylinder into the pipe [kg/s]; it is
    /// injected as a linear-acoustic source `p' = rho c u = c m_dot / A`.
    /// `valve_open_fraction` blends the valve-end boundary between a closed end
    /// (total reflection) and an open valve (mostly transmitting).
    pub fn integrate(
        &mut self,
        dt: f64,
        speed_of_sound: f64,
        mass_flux: f64,
        valve_open_fraction: f64,
    ) {
        if !(dt.is_finite() && dt > 0.0) {
            return;
        }
        let c = speed_of_sound.max(1.0);
        let tau_seg = self.length / (self.segments() as f64 * c);
        if !tau_seg.is_finite() || tau_seg <= 0.0 {
            return;
        }

        let source = (c * mass_flux.max(0.0) / self.area).clamp(0.0, 5e5);
        let open = valve_open_fraction.clamp(0.0, 1.0);
        let valve_reflection = self.closed_end_reflection * (1.0 - open) + 0.1 * open;

        self.accumulator += dt;
        // A very long frame could otherwise ask for thousands of internal steps;
        // the pipe has already reached steady state long before that.
        let budget = (self.segments() * 8) as f64 * tau_seg;
        if self.accumulator > budget {
            self.accumulator = budget;
        }

        while self.accumulator >= tau_seg {
            self.accumulator -= tau_seg;
            self.advance(source * open, valve_reflection);
        }
    }

    /// One unit-CFL grid step.
    fn advance(&mut self, source: f64, valve_reflection: f64) {
        let n = self.segments();
        let d = self.damping.clamp(0.0, 1.0);

        // Boundaries are evaluated against the pre-shift field.
        // Open end: an expansion wave comes back, hence the sign inversion. The
        // missing amplitude is what radiates out of the tailpipe as sound.
        let reflected_at_open = -self.open_end_reflection * self.forward[n - 1];
        let reflected_at_valve = valve_reflection * self.backward[0];

        for i in (1..n).rev() {
            self.forward[i] = self.forward[i - 1] * d;
        }
        self.forward[0] = source + reflected_at_valve;

        for i in 0..n - 1 {
            self.backward[i] = self.backward[i + 1] * d;
        }
        self.backward[n - 1] = reflected_at_open;
    }
}

/// Which manifold model is live.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ManifoldMode {
    /// Lumped, zero-dimensional plenum: the pipe is acoustically short.
    Plenum,
    /// Segmented 1D acoustic pipe: the pipe is tuned to the firing rate.
    Acoustic,
}

/// The manifold transit rule: when does a runner stop being a plenum?
///
/// A lumped plenum is only valid while the pipe is acoustically compact — while
/// a pressure disturbance crosses it and back long before the next pulse
/// arrives. It stops being valid when the round-trip reflection time
///
/// ```text
/// tau_pulse = (2 * L_pipe) / c_exhaust
/// ```
///
/// divides into the firing interval a whole number of times, because then each
/// reflected wave returns to the valve *in phase* with the next exhaust pulse
/// and the runner starts to scavenge (or choke) the cylinder. The rule compares
///
/// ```text
/// ratio = T_pulse / tau_pulse,      T_pulse = dtheta_bank / (6 * rpm)   [s]
/// ```
///
/// against the nearest integer and switches to the segmented pipe inside a
/// tolerance band. The band is wider on the way out than on the way in
/// (hysteresis), so an engine sitting exactly on the boundary cannot chatter
/// between two models once a frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ManifoldTransit {
    /// Half-width of the band that switches *into* the acoustic model [-].
    pub engage_tolerance: f64,
    /// Half-width of the band that keeps it engaged [-].
    pub release_tolerance: f64,
    /// Highest harmonic considered a resonance [-].
    pub max_harmonic: u32,
}

impl Default for ManifoldTransit {
    fn default() -> Self {
        Self {
            engage_tolerance: 0.15,
            release_tolerance: 0.25,
            max_harmonic: 6,
        }
    }
}

impl ManifoldTransit {
    /// Time between exhaust pulses arriving at one bank's collector [s].
    ///
    /// `T_pulse = dtheta_bank / (6 * rpm)`: `dtheta/360` of a revolution, times
    /// `60/rpm` seconds per revolution.
    pub fn pulse_period(bank_interval: f64, rpm: f64) -> f64 {
        let degrees = bank_interval.to_degrees();
        degrees / (6.0 * rpm.abs().max(1e-3))
    }

    /// `T_pulse / tau_pulse`: how many round trips fit between pulses [-].
    pub fn tuning_ratio(&self, bank_interval: f64, rpm: f64, pipe: &AcousticPipe, c: f64) -> f64 {
        let period = Self::pulse_period(bank_interval, rpm);
        let transit = pipe.transit_time(c);
        if transit <= 0.0 {
            return 0.0;
        }
        period / transit
    }

    /// Distance from the tuning ratio to the nearest resonant harmonic [-].
    pub fn detuning(&self, ratio: f64) -> f64 {
        if !ratio.is_finite() || ratio <= 0.0 {
            return f64::INFINITY;
        }
        let nearest = ratio.round().clamp(1.0, self.max_harmonic as f64);
        (ratio - nearest).abs()
    }

    /// Decides the mode for this frame, given the mode currently running.
    pub fn evaluate(&self, current: ManifoldMode, ratio: f64) -> ManifoldMode {
        let detune = self.detuning(ratio);
        match current {
            ManifoldMode::Plenum if detune <= self.engage_tolerance => ManifoldMode::Acoustic,
            ManifoldMode::Acoustic if detune > self.release_tolerance => ManifoldMode::Plenum,
            other => other,
        }
    }
}

/// One bank's exhaust: a plenum, a pipe, and the rule that picks between them.
#[derive(Debug, Clone, PartialEq)]
pub struct ExhaustManifold {
    /// Lumped collector volume.
    pub plenum: Plenum,
    /// Segmented primary runner.
    pub pipe: AcousticPipe,
    /// Switching rule.
    pub transit: ManifoldTransit,
    /// Which model is currently live.
    pub mode: ManifoldMode,
    /// Ambient back pressure the pipe's open end discharges into [Pa].
    pub ambient_pressure: f64,
    /// Most recent tuning ratio, kept for telemetry [-].
    pub tuning_ratio: f64,
}

impl ExhaustManifold {
    /// A bank manifold sized from a runner length, area and collector volume.
    pub fn new(
        runner_length: f64,
        runner_area: f64,
        segments: usize,
        collector_volume: f64,
        ambient_pressure: f64,
        temperature: f64,
        gas: &GasProperties,
    ) -> Self {
        Self {
            plenum: Plenum::new(
                collector_volume,
                ambient_pressure * 1.03,
                temperature,
                gas.r_burned,
                gas.gamma_burned,
            ),
            pipe: AcousticPipe::new(runner_length, runner_area, segments),
            transit: ManifoldTransit::default(),
            mode: ManifoldMode::Plenum,
            ambient_pressure,
            tuning_ratio: 0.0,
        }
    }

    /// Speed of sound in the collector gas [m/s].
    pub fn speed_of_sound(&self) -> f64 {
        (self.plenum.gamma * self.plenum.gas_constant * self.plenum.temperature).sqrt()
    }

    /// Static pressure the exhaust valve sees [Pa].
    ///
    /// In plenum mode this is just the collector pressure. In acoustic mode the
    /// pipe's wave field rides on top of it, which is what lets a returning
    /// expansion wave pull below the collector pressure and scavenge the
    /// cylinder during overlap.
    pub fn port_pressure(&self) -> f64 {
        let base = self.plenum.pressure();
        match self.mode {
            ManifoldMode::Plenum => base,
            ManifoldMode::Acoustic => (base + self.pipe.valve_end_perturbation()).max(1e3),
        }
    }

    /// Advances the manifold by one frame.
    ///
    /// `bank_flux` is the total mass leaving all of this bank's cylinders
    /// [kg/s], `bank_temperature` the enthalpy-carrying temperature of that
    /// stream, and `valve_open_fraction` how much of the bank's exhaust valve
    /// area is currently uncovered.
    #[allow(clippy::too_many_arguments)]
    pub fn integrate(
        &mut self,
        dt: f64,
        rpm: f64,
        bank_interval: f64,
        bank_flux: f64,
        bank_temperature: f64,
        valve_open_fraction: f64,
    ) {
        let c = self.speed_of_sound();
        self.tuning_ratio = self.transit.tuning_ratio(bank_interval, rpm, &self.pipe, c);

        let next = self.transit.evaluate(self.mode, self.tuning_ratio);
        if next != self.mode {
            // Starting the pipe from a stale standing wave would inject a
            // discontinuity into the port pressure; start it at rest instead.
            if next == ManifoldMode::Acoustic {
                self.pipe.reset();
            }
            self.mode = next;
        }

        // The collector always runs: it sets the mean back pressure that the
        // acoustic model perturbs around, so it cannot be skipped in either mode.
        let outflow = self.tailpipe_flow(dt);
        self.plenum
            .integrate(dt, bank_flux.max(0.0), bank_temperature, outflow);

        if self.mode == ManifoldMode::Acoustic {
            self.pipe.integrate(dt, c, bank_flux, valve_open_fraction);
        }
    }

    /// Quasi-steady discharge from the collector out of the tailpipe [kg/s].
    ///
    /// A linearised orifice: enough to give the collector a realistic blowdown
    /// time constant without a second full compressible solve.
    fn tailpipe_flow(&self, dt: f64) -> f64 {
        let excess = self.plenum.pressure() - self.ambient_pressure;
        if excess <= 0.0 {
            return 0.0;
        }
        let density = self.plenum.mass / self.plenum.volume;
        // Bernoulli through the tailpipe area, taken as the runner area, rate
        // limited so the collector cannot be pumped below ambient in one step.
        let bernoulli = self.pipe.area * 0.7 * (2.0 * excess * density).sqrt();
        bernoulli.min(self.plenum.rate_limit(self.ambient_pressure, dt))
    }
}

// ---------------------------------------------------------------------------
// The block
// ---------------------------------------------------------------------------

/// Everything the block reports after one frame.
#[derive(Debug, Clone, PartialEq)]
pub struct BlockOutput {
    /// Summed indicated torque over all cylinders at this instant [N m].
    pub indicated_torque: f64,
    /// Chen-Flynn friction torque [N m].
    pub friction_torque: f64,
    /// Net brake torque, `indicated - friction` [N m].
    pub brake_torque: f64,
    /// Brake power `tau * omega` [W].
    pub brake_power: f64,
    /// Net indicated mean effective pressure of the logged cycle [Pa].
    pub imep: f64,
    /// Brake mean effective pressure [Pa].
    pub bmep: f64,
    /// Peak cylinder pressure over the logged cycle [Pa].
    pub peak_pressure: f64,
    /// Livengood-Wu integral of the master cylinder [-].
    pub knock_integral: f64,
    /// Whether the master cylinder autoignited this frame.
    pub knocking: bool,
    /// Per-cylinder instantaneous pressure, in cast order [Pa].
    pub cylinder_pressures: Vec<f64>,
    /// Per-cylinder instantaneous indicated torque, in cast order [N m].
    pub cylinder_torques: Vec<f64>,
    /// Per-cylinder blowdown pressure at exhaust valve opening [Pa].
    pub cylinder_evo_pressures: Vec<f64>,
    /// Manifold mode per bank.
    pub manifold_modes: Vec<ManifoldMode>,
    /// Tuning ratio per bank [-].
    pub tuning_ratios: Vec<f64>,
    /// What the solver did this frame.
    pub step: StepReport,
}

/// A complete multi-cylinder engine: master cylinder, phase ring, manifolds.
#[derive(Debug, Clone)]
pub struct EngineBlock {
    /// Per-cylinder physics.
    pub model: CylinderModel,
    /// Firing arrangement.
    pub firing: FiringOrder,
    /// RK4 solver with its watchdog and angular budget.
    pub solver: Rk4Solver,
    /// The one cylinder that is actually integrated.
    pub master: ThermoState,
    /// 720-cell historical trace of the master.
    pub ring: PhaseRing,
    /// Intake plenum shared by every cylinder.
    pub intake: Plenum,
    /// One exhaust manifold per bank.
    pub exhaust_banks: Vec<ExhaustManifold>,
    /// Friction correlation.
    pub friction: ChenFlynn,
    /// Ambient conditions.
    pub environment: Environment,
    /// Crankcase pressure acting on the piston underside [Pa].
    pub crankcase_pressure: f64,
    /// Shaft speed [rad/s].
    pub omega: f64,
    /// Optional overrides for per-cylinder EVO pressure, for testing cylinder scatter [Pa].
    pub evo_overrides: Vec<Option<f64>>,
    /// Exhaust system geometry: primaries, collector, crossover, and silencers.
    pub exhaust: ExhaustSystem,
    /// Intake system geometry: runners, plenum, throttle, airbox, and snorkel.
    pub intake_system: IntakeSystem,
    /// Dressed mass of the engine block [kg].
    pub block_mass: f64,
    /// Block temperature, and the chamber wall the solver runs against.
    pub thermal: EngineThermal,
    /// Driver throttle demand, `0..=1` [-].
    pub throttle: f64,
    /// Engine control unit: fuelling, timing, knock retard, limiters, and cylinder health.
    pub ecu: EngineControlUnit,
}

impl EngineBlock {
    /// Builds a block from a cylinder model and a firing order.
    pub fn new(model: CylinderModel, firing: FiringOrder, environment: Environment) -> Self {
        let banks = firing.bank_count().max(1);
        let per_bank = (firing.len() / banks).max(1);
        // Collector sized to roughly one bank's worth of displacement: big
        // enough to smooth the pulses, small enough to still breathe.
        let collector = model.geometry.displacement() * per_bank as f64 * 1.5;
        let exhaust = ExhaustSystem::default_for_cylinders(firing.len(), banks);
        let intake_system = IntakeSystem::default_for_cylinders(firing.len());
        let runner_area = exhaust.primary_area();

        let exhaust_banks = (0..banks)
            .map(|bank_idx| {
                let runner_length =
                    exhaust.primary_length_for_cylinders(&firing.cylinders_on_bank(bank_idx as u8));
                ExhaustManifold::new(
                    runner_length,
                    runner_area,
                    24,
                    collector,
                    environment.pressure,
                    900.0,
                    &model.gas,
                )
            })
            .collect();

        let intake = Plenum::new(
            model.geometry.displacement() * firing.len() as f64 * 0.8,
            environment.pressure,
            environment.temperature,
            model.gas.r_unburned,
            model.gas.gamma_unburned,
        );

        let master = ThermoState::at_ambient(&model.geometry, &model.gas, &environment);

        let thermal = EngineThermal::soaked(180.0, &exhaust, environment.temperature);
        let ecu = EngineControlUnit {
            base_wiebe_duration: model.combustion.duration(),
            // There is no coil on a compression-ignition engine, so there is
            // nothing for a spark cut to cut. The only way to stop a diesel
            // firing is to stop fuelling it, which is also why a diesel on its
            // limiter simply goes quiet instead of banging: a cylinder that got
            // no fuel has none to send out of the exhaust unburnt.
            limiter_cut_type: match model.combustion {
                HeatRelease::Spark(_) => LimiterCut::Spark,
                HeatRelease::Compression(_) => LimiterCut::Fuel,
            },
            ..EngineControlUnit::default()
        };

        Self {
            model,
            firing,
            solver: Rk4Solver::default(),
            master,
            ring: PhaseRing::new(),
            intake,
            exhaust_banks,
            friction: ChenFlynn::default(),
            environment,
            crankcase_pressure: environment.pressure,
            omega: 0.0,
            evo_overrides: Vec::new(),
            exhaust,
            intake_system,
            block_mass: 180.0,
            thermal,
            throttle: 1.0,
            ecu,
        }
    }

    /// Re-sizes the exhaust manifolds from the block's current exhaust geometry.
    pub fn rebuild_exhaust_banks(&mut self) {
        let banks = self.firing.bank_count().max(1);
        let per_bank = (self.firing.len() / banks).max(1);
        let collector = self.model.geometry.displacement() * per_bank as f64 * 1.5;
        let runner_area = self.exhaust.primary_area();
        self.exhaust_banks = (0..banks)
            .map(|bank_idx| {
                let runner_length = self
                    .exhaust
                    .primary_length_for_cylinders(&self.firing.cylinders_on_bank(bank_idx as u8));
                ExhaustManifold::new(
                    runner_length,
                    runner_area,
                    24,
                    collector,
                    self.environment.pressure,
                    900.0,
                    &self.model.gas,
                )
            })
            .collect();
        self.thermal.rebuild_exhaust(&self.exhaust);
    }

    /// A 4.0 litre cross-plane V8 on the default cylinder model.
    pub fn cross_plane_v8(environment: Environment) -> Self {
        Self::new(
            CylinderModel::default(),
            FiringOrder::cross_plane_v8(),
            environment,
        )
    }

    /// Total swept volume of the block [m^3].
    pub fn total_displacement(&self) -> f64 {
        self.model.geometry.displacement() * self.firing.len() as f64
    }

    /// Cylinder geometry, for callers that only need the kinematics.
    pub fn geometry(&self) -> &CylinderGeometry {
        &self.model.geometry
    }

    /// The 1-based phase index cylinder `i` reads this frame.
    ///
    /// ```text
    /// Index_i = floor( deg(theta_master - dtheta_firing_i) mod 720 ) + 1
    /// ```
    pub fn phase_index_of(&self, cylinder: usize) -> usize {
        let offset = self
            .firing
            .cylinders
            .get(cylinder)
            .map_or(0.0, |c| c.firing_offset);
        PhaseRing::phase_index(self.master.cylinder.theta - offset)
    }

    /// The master's current phase sample, read straight out of the ring.
    pub fn sample_of(&self, cylinder: usize) -> PhaseSample {
        self.ring.cell(self.phase_index_of(cylinder) - 1)
    }

    /// Returns the operational health of cylinder `i`.
    pub fn cylinder_health(&self, cylinder: usize) -> CylinderHealth {
        self.ecu.cylinder_health(cylinder)
    }

    /// Sets the operational health of cylinder `i`.
    pub fn set_cylinder_health(&mut self, cylinder: usize, health: CylinderHealth) {
        self.ecu.set_cylinder_health(cylinder, health);
    }

    /// Exhaust valve opening cylinder pressure for cylinder `i` [Pa].
    ///
    /// Reads the master cylinder trace at the exhaust valve opening angle, or
    /// returns an override if one has been set for testing cylinder-to-cylinder
    /// scatter. When a cylinder is dead (no spark or no fuel), it produces
    /// manifold pressure at EVO rather than a combustion blowdown pulse.
    pub fn cylinder_evo_pressure(&self, cylinder: usize) -> f64 {
        if let Some(Some(p)) = self.evo_overrides.get(cylinder) {
            return *p;
        }
        let factor = self.ecu.cylinder_combustion_factor(cylinder) as f64;
        let fired_p = self
            .ring
            .sample(self.model.valves.exhaust.open_angle)
            .pressure;
        let bank_idx = self
            .firing
            .cylinders
            .get(cylinder)
            .map_or(0, |c| (c.bank as usize) % self.exhaust_banks.len().max(1));
        let manifold_p = self
            .exhaust_banks
            .get(bank_idx)
            .map_or(self.environment.pressure, |b| b.port_pressure());
        manifold_p + factor * (fired_p - manifold_p)
    }

    /// Alias for [`cylinder_evo_pressure`].
    pub fn evo_pressure_of(&self, cylinder: usize) -> f64 {
        self.cylinder_evo_pressure(cylinder)
    }

    /// Exhaust valve opening pressure for every cylinder in cast order [Pa].
    pub fn evo_pressures(&self) -> Vec<f64> {
        (0..self.firing.len())
            .map(|i| self.cylinder_evo_pressure(i))
            .collect()
    }

    /// Sets an EVO pressure override for cylinder `i` [Pa].
    pub fn set_cylinder_evo_pressure(&mut self, cylinder: usize, pressure: f64) {
        if self.evo_overrides.len() <= cylinder {
            self.evo_overrides.resize(cylinder + 1, None);
        }
        self.evo_overrides[cylinder] = Some(pressure);
    }

    /// Port boundary conditions currently seen by the cylinders.
    pub fn port_conditions(&self) -> PortConditions {
        let mut ports = PortConditions::from_environment(&self.environment, &self.model.gas);
        ports.intake.pressure = self.intake.pressure();
        ports.intake.temperature = self.intake.temperature;
        // The master is cylinder 1, which lives on bank 0.
        if let Some(bank) = self.exhaust_banks.first() {
            ports.exhaust.pressure = bank.port_pressure();
            ports.exhaust.temperature = bank.plenum.temperature;
        }
        ports
    }

    /// Advances the whole block by one wall-clock frame.
    ///
    /// Order of operations: integrate the master and log the ring, feed the
    /// manifolds from the summed bank fluxes read back out of the ring, then sum
    /// torque over the cylinders at their own phases.
    pub fn update(&mut self, frame_dt: f64, rpm: f64) -> BlockOutput {
        self.omega = rpm * 2.0 * PI / 60.0;
        let load = self.load_fraction();
        // A compression-ignition engine has no throttle plate and no lambda
        // target. It draws a full cylinder of air on every stroke whatever the
        // load and meters fuel into that, so it is lean everywhere and never
        // anything else — and the AFR schedule, which is a spark engine's map
        // of how rich to run and when, has nothing to say about it. Its mixture
        // is its own.
        let afr = match self.model.combustion {
            HeatRelease::Spark(_) => {
                let scheduled = self.ecu.schedule_afr(load, rpm, self.throttle, frame_dt);
                self.model.air_fuel_ratio = scheduled;
                scheduled
            }
            HeatRelease::Compression(_) => self.model.air_fuel_ratio,
        };
        self.model.gas = GasProperties::for_afr(afr);
        let limiter = self.ecu.evaluate_limiter(rpm);
        let dfco = self.ecu.update_dfco(self.throttle, rpm);
        self.model.fuel_cut = dfco || limiter == LimiterCut::Fuel || self.ecu.cranking;
        // Spark timing is the ECU's on an engine that has a coil. A diesel has
        // none: its heat release starts where the Arrhenius integral says, and
        // the latch solves that per cycle. See [`HeatRelease`].
        let spark_angle = self.ecu.spark_angle_with_throttle(load, rpm, self.throttle);
        let wiebe_duration = self.ecu.wiebe_duration(afr);
        if let Some(wiebe) = self.model.combustion.spark_mut() {
            wiebe.spark_angle = spark_angle;
            wiebe.duration = wiebe_duration;
        }
        for bank in &mut self.exhaust_banks {
            bank.plenum.gamma = self.model.gas.gamma_burned;
            bank.plenum.gas_constant = self.model.gas.r_burned;
        }
        let ports = self.port_conditions();

        // Destructured so the observer can borrow the ring while the solver
        // borrows the master; they are disjoint fields of the same struct.
        let EngineBlock {
            model,
            solver,
            master,
            ring,
            environment,
            ..
        } = self;
        let omega = rpm * 2.0 * PI / 60.0;

        let step = solver.step_frame(
            model,
            master,
            omega,
            frame_dt,
            &ports,
            environment,
            |st, previous_theta| {
                let flows = model.flows(st, omega, &ports);
                let theta = st.cylinder.theta;
                let sample = PhaseSample {
                    pressure: st.cylinder.pressure(&model.geometry, &model.gas),
                    temperature: st.cylinder.temperature,
                    mass: st.cylinder.mass,
                    volume: model.geometry.safe_volume(theta),
                    dvolume_dtheta: model.geometry.dvolume_dtheta(theta),
                    burned_fraction: st.cylinder.burned_fraction,
                    knock_integral: st.knock_integral,
                    intake_flow: flows.intake_flow,
                    exhaust_flow: flows.exhaust_flow,
                    heat_release: flows.heat_release,
                    wall_loss: flows.wall_loss,
                };
                ring.record_span(previous_theta, theta, sample);
            },
        );

        self.update_manifolds(step.plan.dt, rpm);
        self.update_thermal(step.plan.dt, rpm);
        let knocked = step.knocked || self.master.knock_integral >= 1.0;
        self.ecu.update_knock_retard(knocked, step.plan.dt);
        self.assemble_output(rpm, step)
    }

    /// Advances the block's temperature and re-derives the chamber wall.
    ///
    /// The solver has just run a frame against last frame's wall temperature.
    /// That is a lag of one frame against a time constant of minutes, which is
    /// nothing; solving the wall implicitly with the charge would cost a second
    /// Newton iteration inside every RK4 stage to move a temperature by
    /// millikelvin.
    fn update_thermal(&mut self, dt: f64, rpm: f64) {
        // A stopped engine's phase ring is a frozen photograph of the last cycle
        // it turned: the fluxes in it are real rates, but nothing is happening at
        // them any more. Only a turning crank puts heat into the block.
        let turning = self.omega != 0.0;
        let chamber_heat = if turning {
            self.ring.cycle_mean(|s| s.wall_loss).max(0.0)
        } else {
            0.0
        };
        // Every watt the crankshaft spends on friction is a watt rubbed into the
        // bearings, the bores and the oil, and all of it lands in the block. It
        // is a third of the warm-up heat at idle, and it is what makes a cold
        // engine warm itself faster than a warm one would.
        let friction_heat = if turning {
            self.friction.torque(
                self.ring.peak_pressure(),
                self.model.geometry.mean_piston_speed(rpm.abs()),
                self.total_displacement(),
                self.thermal.oil_temperature(),
            ) * self.omega.abs()
        } else {
            0.0
        };

        let cylinders = self.firing.len().max(1);
        // What one cylinder is actually pushing into its own primary: the mean
        // outward port flux over the logged cycle, and the temperature it leaves
        // the port at. Both are the solver's, not a nominal.
        let port_flow = if turning {
            self.ring.cycle_mean(|s| (-s.exhaust_flow).max(0.0))
        } else {
            0.0
        };
        let port_temperature = self
            .exhaust_banks
            .first()
            .map_or(self.environment.temperature, |bank| bank.plenum.temperature);

        let banks = self.firing.bank_count().max(1);
        self.thermal.integrate(
            dt,
            chamber_heat * cylinders as f64,
            friction_heat,
            port_temperature,
            port_flow,
            cylinders / banks,
        );

        let area = self.model.heat.mean_surface_area(&self.model.geometry);
        self.model.heat.wall_temperature = self.thermal.wall_temperature(chamber_heat, area);
    }

    /// Sets the dressed block mass, resizing the thermal mass that follows it.
    ///
    /// The two cannot drift apart: how much metal there is decides both where
    /// the structural modes sit and how long the engine takes to warm up.
    pub fn set_block_mass(&mut self, mass: f64) {
        self.block_mass = mass.max(1.0);
        self.thermal.set_block_mass(self.block_mass);
    }

    /// Fills the phase ring by motoring the engine through two cycles.
    ///
    /// An unwritten ring cell is not an engine with no pressure in it, it is an
    /// engine nothing has solved yet, and
    /// [`Self::instantaneous_indicated_torque`] reports nothing at all until
    /// every cell holds a real sample. That is the right answer to give and the
    /// wrong state to start a crank in: a shaft at rest cannot turn far enough
    /// to fill the ring before a starter has already dragged it up to speed
    /// against no compression whatsoever, which is a silent start. So the cycle
    /// is solved here, once, before the key is turned.
    ///
    /// Motored, whatever the ECU would otherwise have metered, because the
    /// charge this seeds is exactly the one a starter has to work against. Two
    /// cycles rather than one: the first fills the ring from a chamber that
    /// began at rest, the second replaces it with one whose trapped mass came
    /// out of a cycle that had actually happened.
    ///
    /// Seeding is bookkeeping, not time passing, so the thermal state is put
    /// back exactly as it was found: an engine that stood overnight has stood
    /// overnight whether or not anybody worked out what was in its cylinders.
    pub fn prime_ring(&mut self, rpm: f64) {
        let rpm = rpm.max(1.0);
        let was_cranking = self.ecu.cranking;
        let thermal = self.thermal.clone();
        self.ecu.cranking = true;
        let dt = 1.0 / 480.0;
        let frames = (2.0 * 120.0 / rpm / dt).ceil() as usize;
        for _ in 0..frames {
            self.update(dt, rpm);
        }
        self.ecu.cranking = was_cranking;
        self.thermal = thermal;
    }

    /// Puts every thermal mass back to ambient: an engine that stood overnight.
    pub fn cold_start(&mut self) {
        self.thermal =
            EngineThermal::cold(self.block_mass, &self.exhaust, self.environment.temperature);
    }

    /// Pushes the summed per-bank fluxes into the manifolds.
    fn update_manifolds(&mut self, dt: f64, rpm: f64) {
        let mut intake_draw = 0.0;

        for bank in 0..self.exhaust_banks.len() as u8 {
            let mut flux = 0.0;
            let mut enthalpy_flux = 0.0;
            let mut open_area = 0.0;

            for (i, cyl) in self.firing.cylinders.iter().enumerate() {
                let sample = self.sample_of(i);
                if cyl.bank == bank {
                    // Ring flux is signed into the cylinder, so leaving the
                    // cylinder is a negative exhaust_flow.
                    let out = (-sample.exhaust_flow).max(0.0);
                    flux += out;
                    enthalpy_flux += out * sample.temperature;
                    let theta = wrap_cycle(self.master.cylinder.theta - cyl.firing_offset);
                    open_area += self.model.valves.exhaust.effective_area(theta);
                }
                if bank == 0 {
                    // The intake plenum is shared, so accumulate it once.
                    intake_draw += sample.intake_flow.max(0.0);
                }
            }

            let temperature = if flux > 1e-12 {
                enthalpy_flux / flux
            } else {
                self.exhaust_banks[bank as usize].plenum.temperature
            };
            let reference_area = PI * self.model.valves.exhaust.diameter.powi(2) / 4.0;
            let open_fraction = (open_area / reference_area.max(1e-12)).clamp(0.0, 1.0);
            let interval = self.firing.mean_bank_interval(bank);

            self.exhaust_banks[bank as usize].integrate(
                dt,
                rpm,
                interval,
                flux,
                temperature,
                open_fraction,
            );
        }

        // The intake plenum refills from ambient through the throttle and is
        // drawn down by whatever the cylinders swallow.
        let throttle_flow = self.intake_makeup_flow(dt);
        self.intake
            .integrate(dt, throttle_flow, self.environment.temperature, intake_draw);
    }

    /// Quasi-steady flow through the throttle into the intake plenum [kg/s].
    ///
    /// Gated by `self.throttle`, which is never anything but its `1.0`
    /// default unless a caller — the vehicle-load driving path in
    /// [`crate::bench::Driveline::update`] is the only one today — sets it
    /// from the pedal. At `1.0` the gate is wide open and this is bit-for-bit
    /// the ungated valve-area restriction every existing preset and recorded
    /// fingerprint was measured against; below it, the plate itself becomes
    /// the bottleneck rather than the valves, which is what lets
    /// [`EngineBlock::load_fraction`] respond to load at all instead of being
    /// a function of rpm alone.
    fn intake_makeup_flow(&self, dt: f64) -> f64 {
        let deficit = self.environment.pressure - self.intake.pressure();
        if deficit <= 0.0 {
            return 0.0;
        }
        let density = self.environment.air_density();
        let valve_area =
            PI * self.model.valves.intake.diameter.powi(2) / 4.0 * self.firing.len() as f64 * 0.5;
        // A shut plate is never quite sealed: idle bypass and plate
        // clearance leave a small leak, the way a real one does.
        let throttle_fraction = (0.02 + 0.98 * self.throttle.clamp(0.0, 1.0)).min(1.0);
        let area = valve_area * throttle_fraction;
        let bernoulli = area * 0.8 * (2.0 * deficit * density).sqrt();
        // Never past ambient: a throttle cannot supercharge the engine.
        bernoulli.min(self.intake.rate_limit(self.environment.pressure, dt))
    }

    /// Sums torque over the cylinders and packages the frame's telemetry.
    fn assemble_output(&self, rpm: f64, step: StepReport) -> BlockOutput {
        let n = self.firing.len();
        let mut cylinder_pressures = Vec::with_capacity(n);
        let mut cylinder_torques = Vec::with_capacity(n);
        let mut indicated_torque = 0.0;

        for i in 0..n {
            let sample = self.sample_of(i);
            let factor = self.ecu.cylinder_combustion_factor(i) as f64;
            let torque = self.cylinder_indicated_torque(i);
            indicated_torque += torque;
            cylinder_pressures
                .push(sample.pressure * factor + self.environment.pressure * (1.0 - factor));
            cylinder_torques.push(torque);
        }

        let peak_pressure = self.ring.peak_pressure();
        let mean_piston_speed = self.model.geometry.mean_piston_speed(rpm.abs());
        let displacement = self.total_displacement();
        let friction_torque = self.friction.torque(
            peak_pressure,
            mean_piston_speed,
            displacement,
            self.thermal.oil_temperature(),
        );
        let brake_torque = indicated_torque - friction_torque;

        // Cycle-averaged quantities come from the ring, scaled by cylinder health.
        let healthy_fraction = (0..n)
            .map(|i| self.ecu.cylinder_combustion_factor(i) as f64)
            .sum::<f64>()
            / n.max(1) as f64;
        let cycle_work =
            self.ring.indicated_work(self.crankcase_pressure) * (n as f64 * healthy_fraction);
        let imep = cycle_work / displacement.max(1e-12);
        let mean_indicated_torque = cycle_work / CYCLE_ANGLE;
        let mean_brake_torque = mean_indicated_torque - friction_torque;
        let bmep = mean_brake_torque * CYCLE_ANGLE / displacement.max(1e-12);

        BlockOutput {
            indicated_torque,
            friction_torque,
            brake_torque,
            brake_power: mean_brake_torque * self.omega,
            imep,
            bmep,
            peak_pressure,
            knock_integral: self.master.knock_integral,
            knocking: step.knocked,
            cylinder_pressures,
            cylinder_torques,
            cylinder_evo_pressures: self.evo_pressures(),
            manifold_modes: self.exhaust_banks.iter().map(|b| b.mode).collect(),
            tuning_ratios: self.exhaust_banks.iter().map(|b| b.tuning_ratio).collect(),
            step,
        }
    }

    /// Indicated torque one cylinder is making at the crank's current angle [N m].
    pub fn cylinder_indicated_torque(&self, cylinder: usize) -> f64 {
        let factor = self.ecu.cylinder_combustion_factor(cylinder) as f64;
        self.sample_of(cylinder)
            .indicated_torque(self.crankcase_pressure)
            * factor
    }

    /// Indicated torque summed over the cylinders at the crank's current angle [N m].
    ///
    /// Not the cycle mean that [`Self::mean_brake_torque`] reports, and the
    /// difference between the two is the whole of why a cranking engine chugs.
    /// A piston on its way up a closed bore is a gas spring being wound: tens
    /// of newton metres against the direction of rotation, most of which comes
    /// back over the top. Averaged over a cycle all of that cancels into the
    /// pumping loss and says nothing about what the crank is doing; sampled at
    /// the angle the crank has actually reached, it *is* the compression stroke
    /// a starter has to drag the engine over, and a starter of finite torque
    /// dragged over it slows down and speeds up on its own.
    ///
    /// Zero until the ring holds a whole cycle. An unwritten cell is not a
    /// measurement of zero pressure, it is no measurement at all, and billing
    /// it as a torque would put a full atmosphere of crankcase pressure under a
    /// piston nothing has solved yet.
    pub fn instantaneous_indicated_torque(&self) -> f64 {
        self.indicated_torque_at(self.master.cylinder.theta)
    }

    /// Indicated torque the cylinders would sum to with the crank at `theta` [N m].
    ///
    /// The same quantity [`Self::instantaneous_indicated_torque`] reports, asked
    /// about an angle the crank is not at — which is what lets the whole cycle
    /// be scanned for its worst moment without turning the engine to find it.
    pub fn indicated_torque_at(&self, theta: f64) -> f64 {
        if !self.ring.is_primed() {
            return 0.0;
        }
        self.firing
            .cylinders
            .iter()
            .enumerate()
            .map(|(i, cyl)| {
                self.ring
                    .sample(theta - cyl.firing_offset)
                    .indicated_torque(self.crankcase_pressure)
                    * self.ecu.cylinder_combustion_factor(i) as f64
            })
            .sum()
    }

    /// The compression hump a starter has to drag this engine over [N m].
    ///
    /// The deepest the summed gas torque goes anywhere in the logged cycle,
    /// reported positive. On a multi-cylinder engine the compressions overlap
    /// with somebody else's expansion and partly fill each other in, so this is
    /// a long way below the sum of the individual peaks; on a single it is one
    /// cylinder's whole compression with nothing on the other side of it, which
    /// is why a single is the hard engine to crank and why real ones are fitted
    /// with a decompressor.
    pub fn peak_motored_resistance(&self) -> f64 {
        (0..PHASE_CELLS)
            .map(|cell| -self.indicated_torque_at(deg(cell as f64 + 0.5)))
            .fold(0.0, f64::max)
    }

    /// Mean brake torque over the logged cycle [N m].
    pub fn mean_brake_torque(&self, rpm: f64) -> f64 {
        let n = self.firing.len() as f64;
        let cycle_work = self.ring.indicated_work(self.crankcase_pressure) * n;
        let mean_indicated = cycle_work / CYCLE_ANGLE;
        mean_indicated
            - self.friction.torque(
                self.ring.peak_pressure(),
                self.model.geometry.mean_piston_speed(rpm.abs()),
                self.total_displacement(),
                self.thermal.oil_temperature(),
            )
    }

    /// Forces the per-cycle latch, for tests and for seeding a warm start.
    pub fn latch_now(&mut self) {
        self.master.latch = self.model.latch(&self.master.cylinder, self.omega);
    }

    /// Location of peak cylinder pressure relative to compression TDC (360 deg) [deg ATDC].
    pub fn lpp_deg_atdc(&self) -> f64 {
        let angle = self.ring.peak_pressure_angle();
        if angle >= 360.0 {
            angle - 360.0
        } else {
            angle + 360.0
        }
    }

    /// Trapped mass against the atmospheric reference at the master
    /// cylinder's current displacement [-]: `m_trapped / (rho_amb * V_disp)`,
    /// where `rho_amb` is what the ideal gas law gives for ambient pressure
    /// and temperature. Unclamped — see [`EngineBlock::load_fraction`] and
    /// [`EngineBlock::volumetric_efficiency`], which are this ratio clamped to
    /// the range each of them is actually used over.
    fn trapped_mass_ratio(&self) -> f64 {
        let r_air = 287.058; // specific gas constant [J/(kg K)]
        let t_amb = self.environment.temperature.max(200.0);
        let p_amb = self.environment.pressure.max(50_000.0);
        let rho_amb = p_amb / (r_air * t_amb);
        let cyl_disp = self.model.geometry.displacement().max(1e-6);
        let trapped = self.master.cylinder.mass.max(0.0);
        trapped / (rho_amb * cyl_disp)
    }

    /// Load fraction: normalised trapped mass against what the cylinder would
    /// trap at atmospheric pressure and ambient temperature, at this speed.
    ///
    /// This, not throttle position, is what the ECU schedules on. A throttle
    /// is a valve, and how far it is open only matters through what it lets
    /// the cylinder actually trap — which is also shaped by rpm (breathing
    /// dynamics), back-pressure and reversion, none of which a throttle angle
    /// alone can see. Load fraction reads the trapped charge directly instead,
    /// so `schedule_spark_advance` and `schedule_afr` are scheduling against
    /// what the cylinder actually got, not against where the pedal happens to
    /// be. It coincides with [`EngineBlock::volumetric_efficiency`] for a
    /// naturally aspirated engine — both ask "how much of a full atmospheric
    /// charge did the cylinder trap" — and the two will diverge once a
    /// compressor can push this past 1 while a restrictive port still caps
    /// volumetric efficiency; see `TURBO_PLAN.md`.
    pub fn load_fraction(&self) -> f64 {
        self.trapped_mass_ratio().clamp(0.0, 1.5)
    }

    /// Volumetric efficiency of the master cylinder [-].
    ///
    /// Ratio of trapped air mass to the theoretical air mass occupying the cylinder displacement
    /// at ambient intake conditions: eta_v = m_trapped / (rho_amb * V_disp_cyl).
    pub fn volumetric_efficiency(&self) -> f64 {
        self.trapped_mass_ratio().clamp(0.0, 3.0)
    }

    /// Brake specific fuel consumption [g / (kW h)].
    pub fn bsfc_g_kwh(&self, rpm: f64, brake_power_kw: f64) -> f64 {
        if brake_power_kw <= 0.1 || rpm <= 100.0 {
            return 0.0;
        }
        let afr = match self.model.combustion {
            HeatRelease::Spark(_) => self.ecu.target_afr(0.8, rpm, 1.0),
            HeatRelease::Compression(_) => 18.0, // lean diesel combustion
        }
        .max(8.0);
        let trapped_fuel = self.master.cylinder.mass / afr;
        let n_cyl = self.firing.len() as f64;
        let fuel_kg_s = trapped_fuel * (rpm / 120.0) * n_cyl;
        let bsfc = (fuel_kg_s * 3_600.0 * 1_000.0) / brake_power_kw;
        bsfc.clamp(100.0, 2_000.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::cylinder::deg;

    fn approx(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() <= tol, "expected {b}, got {a} (tol {tol})");
    }

    // -- phase ring ---------------------------------------------------------

    #[test]
    fn phase_index_is_one_based_over_the_whole_cycle() {
        assert_eq!(PhaseRing::phase_index(deg(0.0)), 1);
        assert_eq!(PhaseRing::phase_index(deg(0.5)), 1);
        assert_eq!(PhaseRing::phase_index(deg(1.0)), 2);
        assert_eq!(PhaseRing::phase_index(deg(719.9)), 720);
        // Wraps rather than overflowing.
        assert_eq!(PhaseRing::phase_index(deg(720.0)), 1);
        // Whole-degree angles must snap to the cell they open, not the one
        // below, however the radian round-trip rounds.
        assert_eq!(PhaseRing::phase_index(deg(725.0)), 6);
        assert_eq!(PhaseRing::phase_index(deg(5.0)), 6);
        assert_eq!(PhaseRing::phase_index(deg(-90.0)), 631);
        for d in [-3600.0, 3600.0, 12345.6, -0.000_001] {
            let idx = PhaseRing::phase_index(deg(d));
            assert!((1..=PHASE_CELLS).contains(&idx), "index {idx} out of range");
        }
    }

    /// Plays a downsampled table back the way the audio thread does: linear
    /// interpolation between the two points either side of a cycle phase.
    fn playback(table: &[f32; CYCLE_TABLE], phase: f64) -> f64 {
        let x = phase.rem_euclid(1.0) * CYCLE_TABLE as f64 - 0.5;
        let i = x.floor().rem_euclid(CYCLE_TABLE as f64) as usize;
        let frac = x - x.floor();
        let a = table[i] as f64;
        let b = table[(i + 1) % CYCLE_TABLE] as f64;
        a + (b - a) * frac
    }

    #[test]
    fn downsampled_cycle_plays_back_the_curve_it_came_from() {
        // A two-cycle-per-revolution pressure curve — four periods over 720
        // degrees, which is far faster than anything the solver's own pressure
        // trace does and therefore a pessimistic test of the resampling.
        let curve = |degrees: f64| 20.0e5 + 15.0e5 * (4.0 * 2.0 * PI * degrees / 720.0).sin();
        let mut ring = PhaseRing::new();
        for cell in 0..PHASE_CELLS {
            ring.record(
                deg(cell as f64 + 0.5),
                PhaseSample {
                    pressure: curve(cell as f64 + 0.5),
                    ..PhaseSample::default()
                },
            );
        }

        let table = ring.downsample_from(0.0, |s| s.pressure);
        // Read back at the original resolution. Box-averaging over 5.625 cells
        // and interpolating between the results costs a little amplitude at
        // this rate; 3 % of the swing is the whole of the error.
        let mut worst = 0.0f64;
        for cell in 0..PHASE_CELLS {
            let degrees = cell as f64 + 0.5;
            let played = playback(&table, degrees / 720.0);
            worst = worst.max((played - curve(degrees)).abs());
        }
        assert!(worst < 0.03 * 15.0e5, "playback error {worst:.0} Pa");
    }

    #[test]
    fn downsampling_cuts_the_cycle_at_the_origin_it_is_given() {
        let mut ring = PhaseRing::new();
        for cell in 0..PHASE_CELLS {
            ring.record(
                deg(cell as f64 + 0.5),
                PhaseSample {
                    pressure: cell as f64,
                    ..PhaseSample::default()
                },
            );
        }
        // Cell zero of a table cut at 360 degrees is the mean of cells 360..365.
        let table = ring.downsample_from(deg(360.0), |s| s.pressure);
        approx(table[0] as f64, 362.0, 1e-3);
        // And the table still spans the whole cycle, wrapping at the end.
        let last = table[CYCLE_TABLE - 1] as f64;
        approx(last, 356.5, 1.0);
    }

    #[test]
    fn downsampling_averages_rather_than_decimates() {
        // A cell-to-cell alternation is pure Nyquist for the ring and must not
        // survive into a table running at a fifth of its rate.
        let mut ring = PhaseRing::new();
        for cell in 0..PHASE_CELLS {
            ring.record(
                deg(cell as f64 + 0.5),
                PhaseSample {
                    pressure: if cell % 2 == 0 { 1.0 } else { -1.0 },
                    ..PhaseSample::default()
                },
            );
        }
        let table = ring.downsample_from(0.0, |s| s.pressure);
        let worst = table.iter().fold(0.0f32, |m, &v| m.max(v.abs()));
        assert!(worst <= 0.2, "alias survived downsampling: {worst}");
    }

    #[test]
    fn ring_round_trips_a_sample_at_its_own_angle() {
        let mut ring = PhaseRing::new();
        let sample = PhaseSample {
            pressure: 42e5,
            ..PhaseSample::default()
        };
        ring.record(deg(123.4), sample);
        approx(ring.sample(deg(123.0)).pressure, 42e5, 0.0);
        approx(ring.sample(deg(123.99)).pressure, 42e5, 0.0);
        // The neighbouring cell was not touched.
        approx(ring.sample(deg(124.0)).pressure, 0.0, 0.0);
        assert_eq!(ring.filled_cells(), 1);
        assert!(!ring.is_primed());
    }

    #[test]
    fn record_span_leaves_no_holes_when_dtheta_is_stretched() {
        let mut ring = PhaseRing::new();
        // Walk the whole cycle in 4-degree strides, the coarsest the budget allows.
        let mut theta = 0.0;
        for i in 0..180 {
            let next = wrap_cycle(theta + deg(4.0));
            ring.record_span(
                theta,
                next,
                PhaseSample {
                    pressure: i as f64 + 1.0,
                    ..PhaseSample::default()
                },
            );
            theta = next;
        }
        assert!(
            ring.is_primed(),
            "only {} cells filled",
            ring.filled_cells()
        );
        for cell in ring.cells() {
            assert!(cell.pressure > 0.0, "hole left in the ring");
        }
    }

    #[test]
    fn record_span_handles_the_720_degree_wrap() {
        let mut ring = PhaseRing::new();
        let sample = PhaseSample {
            pressure: 7.0,
            ..PhaseSample::default()
        };
        ring.record_span(deg(718.0), deg(2.0), sample);
        // The span covers the cells *entered*: 719, 0, 1 and 2. The cell the
        // step departed from was written by the step before it, which is what
        // makes consecutive spans tile the ring exactly once.
        for d in [719.5, 0.5, 1.5, 2.5] {
            approx(ring.sample(deg(d)).pressure, 7.0, 0.0);
        }
        for d in [717.5, 718.5, 3.5] {
            approx(ring.sample(deg(d)).pressure, 0.0, 0.0);
        }
    }

    #[test]
    fn indicated_work_integrates_a_synthetic_pv_loop() {
        // Put a constant gauge pressure in every cell: net work over a closed
        // cycle must be zero, because the volume returns to where it started.
        let geometry = CylinderGeometry::default();
        let mut ring = PhaseRing::new();
        for i in 0..PHASE_CELLS {
            let theta = deg(i as f64 + 0.5);
            ring.record(
                theta,
                PhaseSample {
                    pressure: 3e5,
                    dvolume_dtheta: geometry.dvolume_dtheta(theta),
                    ..PhaseSample::default()
                },
            );
        }
        approx(ring.indicated_work(0.0), 0.0, 1e-6);

        // Now pressurise only the expansion strokes: the loop must do net
        // positive work.
        for i in 0..PHASE_CELLS {
            let theta = deg(i as f64 + 0.5);
            let expanding = geometry.dvolume_dtheta(theta) > 0.0;
            ring.record(
                theta,
                PhaseSample {
                    pressure: if expanding { 30e5 } else { 1e5 },
                    dvolume_dtheta: geometry.dvolume_dtheta(theta),
                    ..PhaseSample::default()
                },
            );
        }
        assert!(ring.indicated_work(0.0) > 0.0);
    }

    // -- firing order -------------------------------------------------------

    #[test]
    fn cross_plane_v8_offsets_follow_the_firing_order() {
        let order = FiringOrder::cross_plane_v8();
        assert_eq!(order.len(), 8);
        approx(order.interval.to_degrees(), 90.0, 1e-12);

        // Cylinders come out sorted by number; 1-8-7-2-6-5-4-3 at 90 degrees.
        let expected = [0.0, 270.0, 630.0, 540.0, 450.0, 360.0, 180.0, 90.0];
        for (cyl, want) in order.cylinders.iter().zip(expected) {
            approx(cyl.firing_offset.to_degrees(), want, 1e-9);
        }
        assert_eq!(order.cylinders[0].number, 1);
        assert_eq!(order.bank_count(), 2);
    }

    /// The defining acoustic signature of a cross-plane V8: each bank fires
    /// unevenly. This is the test that would catch a firing order typo.
    #[test]
    fn cross_plane_banks_fire_unevenly_and_flat_plane_banks_do_not() {
        let cross = FiringOrder::cross_plane_v8();
        let mut left = cross.bank_gaps(0);
        left.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let left_deg: Vec<f64> = left.iter().map(|g| g.to_degrees()).collect();
        for (got, want) in left_deg.iter().zip([90.0, 180.0, 180.0, 270.0]) {
            approx(*got, want, 1e-9);
        }
        // Gaps must still add up to a full cycle.
        approx(left_deg.iter().sum::<f64>(), 720.0, 1e-9);

        let flat = FiringOrder::flat_plane_v8();
        for gap in flat.bank_gaps(0) {
            approx(gap.to_degrees(), 180.0, 1e-9);
        }
        for gap in flat.bank_gaps(1) {
            approx(gap.to_degrees(), 180.0, 1e-9);
        }
    }

    #[test]
    fn boxer_six_offsets_alternate_banks_evenly() {
        let boxer = FiringOrder::boxer_six();
        assert_eq!(boxer.len(), 6);
        approx(boxer.interval.to_degrees(), 120.0, 1e-12);
        assert_eq!(boxer.bank_count(), 2);

        // Sequence is 1-6-2-4-3-5.
        // Firing times [deg]:
        // Cyl 1 (bank 0): 0 deg
        // Cyl 6 (bank 1): 120 deg
        // Cyl 2 (bank 0): 240 deg
        // Cyl 4 (bank 1): 360 deg
        // Cyl 3 (bank 0): 480 deg
        // Cyl 5 (bank 1): 600 deg
        // Sorted by cylinder number 1..=6:
        // [0.0, 240.0, 480.0, 360.0, 600.0, 120.0]
        let expected = [0.0, 240.0, 480.0, 360.0, 600.0, 120.0];
        for (cyl, want) in boxer.cylinders.iter().zip(expected) {
            approx(cyl.firing_offset.to_degrees(), want, 1e-9);
        }

        // Bank 0 (1, 2, 3) and Bank 1 (4, 5, 6) each have three cylinders
        // firing at exact 240-degree intervals.
        for gap in boxer.bank_gaps(0) {
            approx(gap.to_degrees(), 240.0, 1e-9);
        }
        for gap in boxer.bank_gaps(1) {
            approx(gap.to_degrees(), 240.0, 1e-9);
        }

        // Alternating bank sequence: 0, 1, 0, 1, 0, 1.
        let bank_pattern: Vec<u8> = boxer
            .sequence
            .iter()
            .map(|&n| boxer.cylinders.iter().find(|c| c.number == n).unwrap().bank)
            .collect();
        assert_eq!(bank_pattern, vec![0, 1, 0, 1, 0, 1]);
    }

    #[test]
    fn every_cylinder_lands_on_a_distinct_phase_cell() {
        let env = Environment::default();
        let block = EngineBlock::cross_plane_v8(env);
        let mut seen: Vec<usize> = (0..block.firing.len())
            .map(|i| block.phase_index_of(i))
            .collect();
        seen.sort_unstable();
        seen.dedup();
        assert_eq!(seen.len(), 8, "two cylinders collided on one phase cell");
        for idx in seen {
            assert!((1..=PHASE_CELLS).contains(&idx));
        }
    }

    /// Amplitude of a `2*pi`-periodic function's second harmonic, by direct
    /// quadrature against `cos(2 theta)` and `sin(2 theta)` over one crank
    /// revolution — the piston kinematics repeat every revolution, not every
    /// 720-degree master cycle, so that is the period the harmonic lives in.
    fn second_harmonic_amplitude(f: impl Fn(f64) -> f64) -> f64 {
        let steps = 3600;
        let dtheta = CYCLE_ANGLE / 2.0 / steps as f64;
        let (mut re, mut im) = (0.0, 0.0);
        for i in 0..steps {
            let theta = dtheta * i as f64;
            let v = f(theta);
            let (s, c) = (2.0 * theta).sin_cos();
            re += v * c;
            im += v * s;
        }
        (re * re + im * im).sqrt()
    }

    /// Second-harmonic magnitude of a firing order's shaking-force resultant,
    /// combining both in-plane axes.
    fn resultant_second_harmonic(
        order: &FiringOrder,
        geometry: &CylinderGeometry,
        omega: f64,
    ) -> f64 {
        let x = second_harmonic_amplitude(|theta| order.shaking_force(geometry, theta, omega).0);
        let y = second_harmonic_amplitude(|theta| order.shaking_force(geometry, theta, omega).1);
        (x * x + y * y).sqrt()
    }

    #[test]
    fn inline_four_secondary_does_not_cancel_and_v8_layouts_differ() {
        // This is the test that proves the block layout is actually being
        // read: an inline-four has nothing to cancel against (one bank, every
        // cylinder's inertia force on the same axis), a crossplane V8's four
        // throws per bank already span a full quadrant and cancel on their
        // own, and a flatplane V8 shares the crossplane's bank angle and bank
        // split population but not its phase pairing, so it does not.
        let geometry = CylinderGeometry::default().with_reciprocating_mass(0.5);
        let omega = 500.0; // rad/s, shared so only layout differs

        let inline_four = resultant_second_harmonic(&FiringOrder::inline_four(), &geometry, omega);
        let cross_plane =
            resultant_second_harmonic(&FiringOrder::cross_plane_v8(), &geometry, omega);
        let flat_plane = resultant_second_harmonic(&FiringOrder::flat_plane_v8(), &geometry, omega);

        assert!(
            inline_four > 1.0,
            "an inline-four's secondary should not cancel: {inline_four}"
        );
        assert!(
            cross_plane < 0.01 * inline_four,
            "a crossplane V8's secondary should cancel against an inline-four's: \
             {cross_plane} vs {inline_four}"
        );
        assert!(
            (flat_plane - cross_plane).abs() > 0.1 * inline_four,
            "a flatplane and a crossplane V8 should differ in resultant: \
             {flat_plane} vs {cross_plane}"
        );
    }

    // -- friction -----------------------------------------------------------

    /// The temperature the Chen-Flynn coefficients were measured at [K].
    fn warm_oil() -> f64 {
        OilViscosity::default().reference_temperature
    }

    #[test]
    fn chen_flynn_grows_with_load_and_speed() {
        let cf = ChenFlynn::default();
        let t = warm_oil();
        let idle = cf.fmep(20e5, 3.0, t);
        let loaded = cf.fmep(80e5, 3.0, t);
        let fast = cf.fmep(20e5, 15.0, t);
        assert!(loaded > idle, "peak pressure must raise FMEP");
        assert!(fast > idle, "piston speed must raise FMEP");
        // A warm V8 at 3000 rpm should land in the usual 0.5-2.5 bar band.
        let cruise = cf.fmep(60e5, 8.6, t);
        assert!(
            (0.5e5..2.5e5).contains(&cruise),
            "FMEP {cruise} Pa is outside the plausible band"
        );
    }

    #[test]
    fn cold_oil_raises_fmep_and_warm_oil_leaves_it_alone() {
        let cf = ChenFlynn::default();
        let warm = cf.fmep(20e5, 3.0, warm_oil());
        let cold = cf.fmep(20e5, 3.0, 293.15);
        assert!(
            cold > warm,
            "thick oil must cost more: {cold} Pa cold against {warm} Pa warm"
        );
        // The shear terms roughly double; the constant and the ring load do not
        // move at all, so the whole FMEP goes up by something under a half.
        let rise = cold / warm - 1.0;
        assert!(
            (0.15..0.60).contains(&rise),
            "a cold idle's FMEP rose by {:.0} %, which is not what a cold engine does",
            rise * 100.0
        );
    }

    #[test]
    fn friction_torque_matches_the_fmep_definition() {
        let cf = ChenFlynn::default();
        let displacement = 4.0e-3; // 4.0 litre
        let t = warm_oil();
        let fmep = cf.fmep(60e5, 8.6, t);
        approx(
            cf.torque(60e5, 8.6, displacement, t),
            fmep * displacement / (4.0 * PI),
            1e-9,
        );
    }

    // -- manifolds ----------------------------------------------------------

    #[test]
    fn plenum_conserves_mass_and_tracks_the_ideal_gas_law() {
        let gas = GasProperties::default();
        let mut p = Plenum::new(2e-3, 101_325.0, 300.0, gas.r_unburned, gas.gamma_unburned);
        approx(p.pressure(), 101_325.0, 1e-6);

        let m0 = p.mass;
        p.integrate(1e-3, 0.01, 300.0, 0.0);
        approx(p.mass, m0 + 1e-5, 1e-15);
        assert!(p.pressure() > 101_325.0, "filling must raise pressure");

        // Hot inflow must raise the plenum temperature.
        let t0 = p.temperature;
        p.integrate(1e-3, 0.05, 900.0, 0.0);
        assert!(p.temperature > t0);

        // Balanced flow at the plenum's own temperature is a no-op on mass.
        let m1 = p.mass;
        p.integrate(1e-3, 0.02, p.temperature, 0.02);
        approx(p.mass, m1, 1e-15);
    }

    #[test]
    fn pipe_round_trip_matches_two_l_over_c() {
        let mut pipe = AcousticPipe::new(0.75, 1e-3, 16);
        let c = 550.0;
        approx(pipe.transit_time(c), 2.0 * 0.75 / c, 1e-15);
        approx(pipe.fundamental(c), c / (2.0 * 0.75), 1e-9);

        // A single-step impulse must come back inverted after one round trip.
        pipe.damping = 1.0;
        pipe.open_end_reflection = 1.0;
        pipe.closed_end_reflection = 0.0;
        let tau_seg = 0.75 / (16.0 * c);
        pipe.integrate(tau_seg, c, 1.0 / (c / pipe.area), 1.0); // unit source, one step
        let injected = pipe.forward[0];
        assert!(injected > 0.0);

        // Propagate the rest of the round trip with no further source.
        for _ in 0..(2 * 16 - 1) {
            pipe.integrate(tau_seg, c, 0.0, 1.0);
        }
        assert!(
            pipe.valve_end_perturbation() < 0.0,
            "open end must return an expansion wave, got {}",
            pipe.valve_end_perturbation()
        );
    }

    #[test]
    fn pipe_stays_bounded_under_continuous_excitation() {
        let mut pipe = AcousticPipe::new(0.8, 1.2e-3, 24);
        for _ in 0..20_000 {
            pipe.integrate(1e-5, 560.0, 0.08, 1.0);
            assert!(pipe.valve_end_perturbation().is_finite());
        }
        assert!(
            pipe.valve_end_perturbation().abs() < 5e5,
            "wave field ran away: {}",
            pipe.valve_end_perturbation()
        );
    }

    #[test]
    fn transit_rule_engages_on_resonance_and_releases_off_it() {
        let rule = ManifoldTransit::default();
        let pipe = AcousticPipe::new(0.75, 1e-3, 24);
        let c = 560.0;
        let interval = deg(180.0);

        // tau_pulse = 2*0.75/560 = 2.679 ms. Find the rpm where the 180-degree
        // bank interval is exactly two round trips.
        let tau = pipe.transit_time(c);
        // T_pulse = 180/(6 rpm) = 30/rpm, so rpm = 30/(k tau).
        let rpm_resonant = 30.0 / (2.0 * tau);
        let ratio = rule.tuning_ratio(interval, rpm_resonant, &pipe, c);
        approx(ratio, 2.0, 1e-9);
        approx(rule.detuning(ratio), 0.0, 1e-9);
        assert_eq!(
            rule.evaluate(ManifoldMode::Plenum, ratio),
            ManifoldMode::Acoustic
        );

        // Halfway between two harmonics is maximally detuned: stay lumped.
        let off = rule.tuning_ratio(interval, 30.0 / (2.5 * tau), &pipe, c);
        approx(off, 2.5, 1e-9);
        assert_eq!(
            rule.evaluate(ManifoldMode::Plenum, off),
            ManifoldMode::Plenum
        );
        assert_eq!(
            rule.evaluate(ManifoldMode::Acoustic, off),
            ManifoldMode::Plenum
        );
    }

    #[test]
    fn transit_rule_has_hysteresis_so_it_cannot_chatter() {
        let rule = ManifoldTransit::default();
        // A ratio in the dead band between the two tolerances holds whichever
        // mode is already running.
        let between = 2.0 + 0.5 * (rule.engage_tolerance + rule.release_tolerance);
        assert!(between > rule.engage_tolerance + 2.0);
        assert!(between - 2.0 <= rule.release_tolerance);
        assert_eq!(
            rule.evaluate(ManifoldMode::Plenum, between),
            ManifoldMode::Plenum
        );
        assert_eq!(
            rule.evaluate(ManifoldMode::Acoustic, between),
            ManifoldMode::Acoustic
        );
    }

    #[test]
    fn detuning_rejects_garbage_ratios() {
        let rule = ManifoldTransit::default();
        assert!(rule.detuning(f64::NAN).is_infinite());
        assert!(rule.detuning(0.0).is_infinite());
        assert!(rule.detuning(-3.0).is_infinite());
        // Beyond the highest harmonic the rule stops finding resonances.
        assert!(rule.detuning(50.0) > rule.engage_tolerance);
    }

    // -- the block ----------------------------------------------------------

    #[test]
    fn v8_runs_a_bounded_cycle_and_produces_positive_brake_torque() {
        let env = Environment::default();
        let mut block = EngineBlock::cross_plane_v8(env);
        let rpm = 3000.0;

        // Twelve cycles at 3000 rpm to settle the trapped mass and manifolds.
        let cycle_seconds = 120.0 / rpm;
        let frame = 1.0 / 600.0;
        let frames = (12.0 * cycle_seconds / frame) as usize;

        let mut last = block.update(frame, rpm);
        for _ in 1..frames {
            last = block.update(frame, rpm);
            assert!(last.indicated_torque.is_finite());
            assert!(last.peak_pressure.is_finite());
        }

        assert!(block.ring.is_primed(), "ring never filled");
        assert!(
            last.brake_torque > 0.0,
            "V8 made no net torque: {} N m",
            last.brake_torque
        );
        assert!(
            (20e5..200e5).contains(&last.peak_pressure),
            "peak pressure {} Pa is implausible",
            last.peak_pressure
        );
        assert!(
            (2e5..20e5).contains(&last.imep),
            "IMEP {} Pa is implausible",
            last.imep
        );
        assert_eq!(last.cylinder_pressures.len(), 8);
        assert_eq!(
            block.solver.watchdog.resets, 0,
            "watchdog should never fire"
        );
    }

    #[test]
    fn a_lag_spike_does_not_destabilise_the_block() {
        let env = Environment::default();
        let mut block = EngineBlock::cross_plane_v8(env);
        for _ in 0..600 {
            block.update(1.0 / 600.0, 3000.0);
        }
        let healthy = block.update(1.0 / 600.0, 3000.0).peak_pressure;

        // A two-second stall, then business as usual.
        let spike = block.update(2.0, 3000.0);
        assert!(spike.step.plan.clamped);
        approx(spike.step.plan.dt, 1.0 / 30.0, 1e-15);
        assert!(spike.step.plan.substeps <= block.solver.budget.max_substeps);

        for _ in 0..600 {
            let out = block.update(1.0 / 600.0, 3000.0);
            assert!(out.brake_torque.is_finite());
        }
        let recovered = block.update(1.0 / 600.0, 3000.0).peak_pressure;
        assert!(
            (recovered - healthy).abs() < healthy * 0.5,
            "block did not recover from the lag spike: {healthy} -> {recovered}"
        );
    }

    #[test]
    fn torque_sums_over_all_eight_cylinders() {
        let env = Environment::default();
        let mut block = EngineBlock::cross_plane_v8(env);
        for _ in 0..2000 {
            block.update(1.0 / 600.0, 3000.0);
        }
        let out = block.update(1.0 / 600.0, 3000.0);
        let summed: f64 = out.cylinder_torques.iter().sum();
        approx(out.indicated_torque, summed, 1e-9);
        approx(
            out.brake_torque,
            out.indicated_torque - out.friction_torque,
            1e-9,
        );
    }

    /// A cross-plane V8's torque pulses must be evenly spaced overall (90
    /// degrees), even though each bank is uneven — that is the crank's whole job.
    #[test]
    fn firing_pulses_are_evenly_spaced_across_the_whole_engine() {
        let order = FiringOrder::cross_plane_v8();
        let mut offsets: Vec<f64> = order
            .cylinders
            .iter()
            .map(|c| c.firing_offset.to_degrees())
            .collect();
        offsets.sort_by(|a, b| a.partial_cmp(b).unwrap());
        for (i, off) in offsets.iter().enumerate() {
            approx(*off, i as f64 * 90.0, 1e-9);
        }
    }

    #[test]
    fn block_exposes_per_cylinder_evo_pressure() {
        let env = Environment::default();
        let mut block = EngineBlock::cross_plane_v8(env);
        let evo = block.model.valves.exhaust.open_angle;
        block.ring.record(
            evo,
            PhaseSample {
                pressure: 3.8e5,
                ..PhaseSample::default()
            },
        );

        // Every cylinder defaults to reading the master trace at EVO.
        for i in 0..8 {
            approx(block.cylinder_evo_pressure(i), 3.8e5, 1e-6);
            approx(block.evo_pressure_of(i), 3.8e5, 1e-6);
        }
        let pressures = block.evo_pressures();
        assert_eq!(pressures.len(), 8);
        assert!(pressures.iter().all(|&p| (p - 3.8e5).abs() < 1e-6));

        // Setting an override changes that cylinder's EVO pressure.
        block.set_cylinder_evo_pressure(1, 2.5e5);
        approx(block.cylinder_evo_pressure(0), 3.8e5, 1e-6);
        approx(block.cylinder_evo_pressure(1), 2.5e5, 1e-6);
        assert_eq!(block.evo_pressures()[1], 2.5e5);
    }

    // -- thermal state ------------------------------------------------------

    /// The solver clamps a frame to `1/30 s` and ages the thermal state on the
    /// same clock, so every warm-up here is stepped inside that.
    const THERMAL_DT: f64 = 1.0 / 120.0;

    /// A cold cross-plane V8, ready to be warmed up.
    fn cold_v8() -> EngineBlock {
        let mut block = EngineBlock::cross_plane_v8(Environment::default());
        block.cold_start();
        block
    }

    /// Runs `block` at a fixed speed, reading `probe` every `interval` seconds.
    fn warm_up(
        block: &mut EngineBlock,
        rpm: f64,
        interval: f64,
        samples: usize,
        probe: impl Fn(&EngineBlock) -> f64,
    ) -> Vec<f64> {
        let steps = (interval / THERMAL_DT).round() as usize;
        let mut trace = Vec::with_capacity(samples + 1);
        trace.push(probe(block));
        for _ in 0..samples {
            for _ in 0..steps {
                block.update(THERMAL_DT, rpm);
            }
            trace.push(probe(block));
        }
        trace
    }

    fn assert_rising(trace: &[f64], what: &str) {
        for pair in trace.windows(2) {
            assert!(
                pair[1] > pair[0],
                "{what} fell from {:.2} to {:.2} during warm-up: {trace:.1?}",
                pair[0],
                pair[1]
            );
        }
    }

    #[test]
    fn from_cold_every_pipe_warms_monotonically_to_a_plateau() {
        let mut block = cold_v8();
        let ambient = block.environment.temperature;
        let trace = warm_up(&mut block, 3_000.0, 10.0, 20, |b| {
            b.thermal.exhaust.primaries[0].wall.temperature
        });

        assert!(
            (trace[0] - ambient).abs() < 1e-9,
            "a cold start must begin at ambient, not {:.1} K",
            trace[0]
        );
        assert_rising(&trace, "the primary wall");

        // A plateau, not a ramp that ran out of test: the last ten seconds have
        // to move the wall by a small fraction of what the first ten did.
        let first = trace[1] - trace[0];
        let last = trace[trace.len() - 1] - trace[trace.len() - 2];
        assert!(
            last < 0.05 * first,
            "still climbing at {last:.1} K per ten seconds against {first:.1} K at the start"
        );
        // And it plateaued below the gas driving it, which is the only place a
        // wall heated by that gas can settle.
        assert!(
            *trace.last().unwrap() < block.thermal.exhaust.primary_gas(0),
            "the wall passed the gas heating it"
        );
    }

    #[test]
    fn the_exhaust_keeps_a_gradient_down_its_length() {
        let mut block = cold_v8();
        for _ in 0..(120.0 / THERMAL_DT) as usize {
            block.update(THERMAL_DT, 3_000.0);
        }
        let port = block.exhaust_banks[0].plenum.temperature;
        let primary = block.thermal.exhaust.primary_gas(0);
        let tailpipe = block.thermal.exhaust.tailpipe_gas();
        assert!(
            port > primary && primary > tailpipe,
            "no gradient: port {port:.0} K, primary {primary:.0} K, tailpipe {tailpipe:.0} K"
        );
    }

    #[test]
    fn a_stopped_hot_engine_cools_at_the_modelled_time_constant() {
        let mut block = EngineBlock::cross_plane_v8(Environment::default());
        let ambient = block.environment.temperature;
        // Started just under the thermostat's rating, where the valve is shut
        // and the only path out is the bypass — so the conductance, and with it
        // the time constant, is a constant over the whole decay.
        let start = block.thermal.thermostat.open_temperature - 6.0;
        block.thermal.block.temperature = start;
        block.thermal.block.conductance = block.thermal.thermostat.conductance(start);
        let tau = block.thermal.block.time_constant();
        assert!(
            (600.0..3_600.0).contains(&tau),
            "an engine cooling in still air takes {tau:.0} s, which is not an engine"
        );

        for _ in 0..(tau / THERMAL_DT) as usize {
            block.update(THERMAL_DT, 0.0);
        }

        // One time constant leaves `1/e` of the excess over ambient.
        let ratio = (block.thermal.block_temperature() - ambient) / (start - ambient);
        approx(ratio, 1.0 / std::f64::consts::E, 1e-3);
    }

    #[test]
    fn fmep_falls_monotonically_as_the_block_warms() {
        let mut block = cold_v8();
        let open = block.thermal.thermostat.open_temperature;
        let oil = warm_up(&mut block, 2_000.0, 5.0, 16, |b| {
            b.thermal.oil_temperature()
        });

        // Read at one fixed load and speed throughout, so what moves is the oil
        // and nothing else: the claim is about viscosity, not about the engine
        // making a different peak pressure when it is cold.
        let fmep: Vec<f64> = oil
            .iter()
            .map(|&t| block.friction.fmep(60e5, 8.6, t))
            .collect();

        // Only while the engine is actually warming. Once the thermostat has it
        // the temperature is flat to the last bit, and so is the friction; a
        // strict inequality there would be asserting on rounding.
        let warming = oil.iter().take_while(|&&t| t < open).count();
        assert!(
            warming > 4,
            "the block reached its thermostat too fast to test"
        );
        assert_rising(&oil[..warming], "the oil temperature");
        // Settled means flat, not "close to where it crossed the threshold":
        // comparing a single sample right at the crossing against the last one
        // is sensitive to exactly which frame the trace happened to sample it
        // on. The tail spread is not.
        let tail = &oil[oil.len() - 4..];
        let tail_spread = tail.iter().cloned().fold(f64::MIN, f64::max)
            - tail.iter().cloned().fold(f64::MAX, f64::min);
        assert!(
            tail_spread < oil[1] - oil[0],
            "the block never settled on its thermostat: {oil:.1?}"
        );
        for pair in fmep[..warming].windows(2) {
            assert!(
                pair[1] < pair[0],
                "FMEP rose from {:.0} to {:.0} Pa while the block was warming",
                pair[0],
                pair[1]
            );
        }
        assert!(
            fmep[0] > 1.15 * fmep[warming - 1],
            "a cold engine's FMEP is {:.0} Pa against {:.0} Pa warm, which is no change at all",
            fmep[0],
            fmep[warming - 1]
        );
    }

    #[test]
    fn dead_cylinder_zeroes_blowdown_and_reduces_indicated_torque() {
        let mut block = EngineBlock::cross_plane_v8(Environment::default());
        for _ in 0..600 {
            block.update(1.0 / 240.0, 3_000.0);
        }
        let healthy_evo = block.cylinder_evo_pressure(1);
        assert!(healthy_evo > block.environment.pressure * 1.5);

        block.set_cylinder_health(1, CylinderHealth::dead_plug());
        let dead_evo = block.cylinder_evo_pressure(1);
        let bank_idx = (block.firing.cylinders[1].bank as usize) % block.exhaust_banks.len().max(1);
        let manifold_p = block.exhaust_banks[bank_idx].port_pressure();
        assert!(
            (dead_evo - manifold_p).abs() < 1e-3,
            "dead cylinder EVO pressure ({dead_evo:.1}) must equal manifold ({manifold_p:.1})"
        );
    }

    #[test]
    fn lpp_and_volumetric_efficiency_diagnostics() {
        let mut block = EngineBlock::cross_plane_v8(Environment::default());
        for _ in 0..600 {
            block.update(1.0 / 240.0, 3_000.0);
        }

        let lpp = block.lpp_deg_atdc();
        assert!(
            (5.0..36.0).contains(&lpp),
            "LPP must sit safely after compression TDC (360 deg) in expansion: got {lpp:.1} deg ATDC"
        );

        let eta_v = block.volumetric_efficiency();
        assert!(
            (0.5..1.5).contains(&eta_v),
            "naturally aspirated volumetric efficiency must be plausible: got {eta_v:.2}"
        );
    }

    #[test]
    fn load_fraction_coincides_with_volumetric_efficiency_na() {
        // Both ask "how much of a full atmospheric charge did the cylinder
        // trap"; for a naturally aspirated engine within load_fraction's
        // tighter clamp they must agree exactly.
        let mut block = EngineBlock::cross_plane_v8(Environment::default());
        for _ in 0..600 {
            block.update(1.0 / 240.0, 3_000.0);
        }
        assert_eq!(block.load_fraction(), block.volumetric_efficiency());
    }

    #[test]
    fn wide_open_throttle_leaves_intake_flow_ungated() {
        let mut block = EngineBlock::cross_plane_v8(Environment::default());
        block.throttle = 1.0;
        // A real deficit below ambient for the gate to have something to restrict.
        block.intake.mass *= 0.9;
        let dt = 1.0 / 480.0;
        let gated = block.intake_makeup_flow(dt);

        let valve_area =
            PI * block.model.valves.intake.diameter.powi(2) / 4.0 * block.firing.len() as f64 * 0.5;
        let deficit = block.environment.pressure - block.intake.pressure();
        let density = block.environment.air_density();
        let expected = (valve_area * 0.8 * (2.0 * deficit * density).sqrt())
            .min(block.intake.rate_limit(block.environment.pressure, dt));

        assert_eq!(gated, expected, "throttle = 1.0 must not gate flow at all");
    }

    #[test]
    fn closing_the_throttle_reduces_trapped_mass_and_load_fraction() {
        let rpm = 3_000.0;
        let frame = 1.0 / 240.0;
        let frames = 600;

        let mut open = EngineBlock::cross_plane_v8(Environment::default());
        open.throttle = 1.0;
        for _ in 0..frames {
            open.update(frame, rpm);
        }

        let mut closed = EngineBlock::cross_plane_v8(Environment::default());
        closed.throttle = 0.25;
        for _ in 0..frames {
            closed.update(frame, rpm);
        }

        assert!(
            closed.load_fraction() < open.load_fraction(),
            "a more closed throttle must trap less: closed={} open={}",
            closed.load_fraction(),
            open.load_fraction()
        );
    }

    /// Settles a fresh V8 at `rpm` on a fixed throttle for long enough for
    /// trapped mass, exhaust temperature and the phase ring to stop moving.
    fn settled_at_throttle(rpm: f64, throttle: f64) -> EngineBlock {
        let mut block = EngineBlock::cross_plane_v8(Environment::default());
        block.throttle = throttle;
        for _ in 0..600 {
            block.update(1.0 / 240.0, rpm);
        }
        block
    }

    #[test]
    fn a_gear_change_that_raises_required_torque_raises_load_fraction() {
        // `RoadLoad`/`Gearbox` already prove (see `physics::vehicle::tests`)
        // that two gears at the same rpm imply different required crank
        // torque; a driver meets more required torque with more pedal. This
        // is the other half: more pedal really does trap more mass, so the
        // schedules downstream see a different load, not the same one.
        let light = settled_at_throttle(3_000.0, 0.08);
        let heavy = settled_at_throttle(3_000.0, 0.15);
        assert!(
            heavy.load_fraction() > light.load_fraction(),
            "more pedal must trap more mass: light={:.3} heavy={:.3}",
            light.load_fraction(),
            heavy.load_fraction()
        );
    }

    #[test]
    fn grade_raises_load_fraction_and_enriches_afr() {
        use crate::physics::vehicle::RoadLoad;

        // Climbing a grade at the same road speed raises the torque the
        // engine must produce — pure algebra, no engine involved yet.
        let level = RoadLoad::generic_road_car();
        let uphill = RoadLoad {
            grade: 0.08, // steep, so the effect is unmistakable
            ..RoadLoad::generic_road_car()
        };
        let speed_mps = 25.0;
        let overall_ratio = 4.0;
        let air_density = 1.2041;
        assert!(
            uphill.crank_torque(speed_mps, air_density, overall_ratio)
                > level.crank_torque(speed_mps, air_density, overall_ratio),
            "a grade must raise the torque required to hold the same speed"
        );

        // Meeting that extra torque takes more pedal, and more pedal really
        // does raise load fraction — leaving the lean-cruise band the ECU
        // uses to save fuel at light, steady load, and enriching toward
        // stoichiometric the way a real ECU does once cruise gives way to
        // sustained pull.
        let cruising = settled_at_throttle(3_000.0, 0.02);
        let climbing = settled_at_throttle(3_000.0, 0.1);
        let cruising_load = cruising.load_fraction();
        let climbing_load = climbing.load_fraction();
        assert!(
            climbing_load > cruising_load,
            "more load must follow more pedal: cruising={cruising_load:.3} climbing={climbing_load:.3}"
        );

        let cruising_afr = cruising.ecu.target_afr(cruising_load, 3_000.0, 0.02);
        let climbing_afr = climbing.ecu.target_afr(climbing_load, 3_000.0, 0.1);
        assert!(
            climbing_afr < cruising_afr,
            "the AFR schedule must enrich (lower number) under more load: \
             cruising={cruising_afr:.2} climbing={climbing_afr:.2}"
        );
    }

    #[test]
    fn higher_load_raises_exhaust_temperature_and_moves_primary_tuning() {
        let light = settled_at_throttle(3_000.0, 0.08);
        let heavy = settled_at_throttle(3_000.0, 0.15);
        assert!(heavy.load_fraction() > light.load_fraction());

        let light_temp = light.exhaust_banks[0].plenum.temperature;
        let heavy_temp = heavy.exhaust_banks[0].plenum.temperature;
        assert!(
            heavy_temp > light_temp,
            "higher load must run a hotter exhaust: light={light_temp:.1} heavy={heavy_temp:.1}"
        );

        let light_tuning = light.exhaust_banks[0].tuning_ratio;
        let heavy_tuning = heavy.exhaust_banks[0].tuning_ratio;
        assert!(
            (heavy_tuning - light_tuning).abs() > 1e-3,
            "the primaries' tuning ratio must move with exhaust temperature: \
             light={light_tuning:.4} heavy={heavy_tuning:.4}"
        );
    }

    #[test]
    fn two_gears_at_the_same_rpm_schedule_different_spark_advance() {
        use crate::physics::vehicle::{Gear, Gearbox, RoadLoad};

        // The plan's own example: cruising in sixth against pulling in
        // second, both at the same rpm — the gears alone already put a
        // different torque demand on the crank.
        let mut gearbox = Gearbox::generic_six_speed();
        gearbox.gear = Gear::Engaged(2);
        let low_gear_ratio = gearbox.overall_ratio().unwrap();
        gearbox.gear = Gear::Engaged(6);
        let high_gear_ratio = gearbox.overall_ratio().unwrap();

        let road = RoadLoad::generic_road_car();
        let speed_mps = 25.0;
        let air_density = 1.2041;
        assert!(
            road.crank_torque(speed_mps, air_density, low_gear_ratio)
                != road.crank_torque(speed_mps, air_density, high_gear_ratio),
            "second and sixth must not ask the crank for the same torque"
        );

        // That difference in demanded torque is met with a different pedal
        // position, which traps a different mass at the same rpm.
        let rpm = 3_000.0;
        let pulling = settled_at_throttle(rpm, 0.15);
        let cruising = settled_at_throttle(rpm, 0.08);
        let pulling_load = pulling.load_fraction();
        let cruising_load = cruising.load_fraction();
        assert!(
            (pulling_load - cruising_load).abs() > 1e-3,
            "the same rpm in two gears must trap different mass: \
             2nd={pulling_load:.3} 6th={cruising_load:.3}"
        );

        let pulling_spark = pulling.ecu.schedule_spark_advance(pulling_load, rpm);
        let cruising_spark = cruising.ecu.schedule_spark_advance(cruising_load, rpm);
        assert!(
            (pulling_spark - cruising_spark).abs() > 1e-3,
            "different load fractions at the same rpm must schedule different \
             spark advance: 2nd={pulling_spark:.2} 6th={cruising_spark:.2}"
        );
    }
}
