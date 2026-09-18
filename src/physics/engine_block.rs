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
use crate::physics::control::{CylinderHealth, EngineControlUnit, LimiterCut, SequentialValve};
use crate::physics::cylinder::{deg, wrap_cycle, CylinderGeometry, GasProperties, CYCLE_ANGLE};
use crate::physics::intake::{ForcedInduction, IntakePlenum, ThrottleBody, ValveDraw};
use crate::physics::plumbing::{ExhaustSystem, IntakeSystem, ThrottleLayout};
use crate::physics::thermal::{EngineThermal, OilViscosity};
use crate::physics::thermodynamics::{
    CylinderModel, HeatRelease, PortConditions, PortState, Rk4Solver, StepReport, ThermoState,
    STOICH_AFR,
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

/// Leanest mixture the solver will carry as a number [-].
///
/// Past the lean limit the charge does not burn at all, so how lean it is stops
/// mattering to anything except the arithmetic — and a delivered fraction on
/// its way to zero would otherwise divide into an air-fuel ratio on its way to
/// infinity and take the gas properties with it.
pub const MAX_TRACKED_AFR: f64 = 60.0;

/// Air path left round a shut throttle plate, as a fraction of the intake
/// valves' reference area [-].
///
/// Plate clearance and nothing else. It used to be nearly seven times this,
/// because it was standing in for the idle bypass as well; the bypass is now
/// its own term with its own actuator, so this is back to being the leak it is
/// named after. On its own it will not idle an engine, which is correct: shut
/// the bypass on a warm engine with your foot off the pedal and it stalls.
pub const PLATE_LEAK_FRACTION: f64 = 0.005;

/// Air the idle bypass can pass at full travel, on the same scale [-].
///
/// Sized so an engine idles with the bypass only part way open, which is what
/// gives the governor authority in both directions: it can starve the engine
/// towards a stall as well as feed it. A governor that can only add air
/// cannot overshoot, and an idle that cannot overshoot cannot lope.
pub const IDLE_BYPASS_AUTHORITY: f64 = 0.050;

/// Typical intake valve diameter as a fraction of cylinder bore [-], the
/// usual two-valve-per-cylinder rule of thumb.
///
/// The reference [`EngineBlock::update_manifolds`] sizes the idle bypass and
/// plate leak against, chosen instead of the live valve or port diameter so
/// an exotic profile's own curtain area — a WOT and reversion claim — cannot
/// move the idle authority a stock valve would have given.
pub const INTAKE_VALVE_TO_BORE_RATIO: f64 = 0.44;

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

    /// Splits one bank's own cylinders into the two scroll groups a
    /// twin-scroll turbine housing would feed, alternating through firing
    /// order rather than by cylinder number.
    ///
    /// Two cylinders that fire back to back are exactly the pair a divided
    /// manifold exists to keep apart — one's blowdown pulse would otherwise
    /// rob the other's scavenging window. Walking the bank's own firing
    /// order and dealing cylinders alternately into the two groups puts
    /// every adjacent pair on opposite scrolls, whatever the physical
    /// cylinder numbering happens to be. See `docs/TURBO_PLAN.md`'s TB5.
    pub fn twin_scroll_groups(&self, bank: u8) -> [Vec<usize>; 2] {
        let mut on_bank: Vec<usize> = self
            .cylinders
            .iter()
            .enumerate()
            .filter(|(_, c)| c.bank == bank)
            .map(|(i, _)| i)
            .collect();
        on_bank.sort_by(|&a, &b| {
            self.cylinders[a]
                .firing_offset
                .partial_cmp(&self.cylinders[b].firing_offset)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let mut groups = [Vec::new(), Vec::new()];
        for (slot, index) in on_bank.into_iter().enumerate() {
            groups[slot % 2].push(index);
        }
        groups
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
    ///
    /// `turbine_outflow`, when a turbocharger is fitted, is this bank's share
    /// of the turbine's actual mass flow — see
    /// [`crate::physics::intake::ForcedInduction::advance_exhaust`] — and
    /// replaces [`Self::tailpipe_flow`]'s plain vent to atmosphere: a fitted
    /// wheel is what the collector empties through now, not open air.
    #[allow(clippy::too_many_arguments)]
    pub fn integrate(
        &mut self,
        dt: f64,
        rpm: f64,
        bank_interval: f64,
        bank_flux: f64,
        bank_temperature: f64,
        valve_open_fraction: f64,
        turbine_outflow: Option<f64>,
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
        let outflow = turbine_outflow.unwrap_or_else(|| self.tailpipe_flow(dt));
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

/// One turbocharger and the exhaust banks whose flux drives it.
///
/// Most boosted engines carry exactly one of these, its `banks` covering
/// every bank in the block — the single shared shaft TB3 and TB4 built and
/// tested against. Giving a V-engine one turbo per bank
/// (`docs/TURBO_PLAN.md`'s TB5) is a second, independent unit with its own
/// single-bank `banks` instead of a change to this one's shape.
#[derive(Debug, Clone)]
pub struct ForcedInductionUnit {
    /// The compressor, turbine, shaft and any wastegate/BOV/boost control.
    pub hardware: ForcedInduction,
    /// Which banks feed this unit's turbine.
    pub banks: Vec<u8>,
    /// Whether those banks feed the turbine through two separate,
    /// unmerged inlets rather than one flux-weighted one.
    ///
    /// With two banks each bank is already its own inlet, so this only
    /// changes how they are combined. With one bank it also determines how
    /// that bank's own cylinders were split when the unit was fitted — see
    /// [`EngineBlock::fit_forced_induction`].
    pub twin_scroll: bool,
    /// A sequential changeover valve gating how much of this unit's shared
    /// bank actually reaches its turbine, `None` meaning always fully open.
    ///
    /// A sequential or small-feeds-a-large-one layout is two units on the
    /// same `banks`: a primary with `activation: None` and a secondary whose
    /// valve brings it online past a threshold — see `docs/TURBO_PLAN.md`'s
    /// TB5. [`EngineBlock::update_manifolds`] feeds the valve's own open
    /// fraction to [`crate::physics::intake::ForcedInduction::advance_exhaust`]
    /// as its turbine's effective nozzle area fraction, so a shut valve
    /// really is a zero-area nozzle — no flow, no power — rather than one
    /// that merely sees a discounted pressure and still, through the map's
    /// own low-flow floor, draws a little of both.
    pub activation: Option<SequentialValve>,
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
    pub intake: IntakePlenum,
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
    /// Idle air bypass position, from shut to the governor's full travel [-].
    ///
    /// A second, small hole past the throttle plate, and the only air an
    /// idling engine gets that the driver is not asking for. It is the idle
    /// governor's actuator: see
    /// [`IdleGovernor`](crate::physics::control::IdleGovernor), which owns the
    /// controller, its lag and its authority limit, and
    /// [`EngineBlock::update_manifolds`], which is where the position becomes
    /// the throttle body's leak area. Zero is a shut bypass, which is a stall
    /// on any engine whose plate is shut as well.
    pub idle_bypass: f64,
    /// Engine control unit: fuelling, timing, knock retard, limiters, and cylinder health.
    pub ecu: EngineControlUnit,
    /// Real turbocharger hardware, if this engine is boosted — see
    /// `docs/TURBO_PLAN.md`'s TB3. Empty leaves the intake and exhaust paths
    /// exactly as an atmospheric engine's. More than one unit is a per-bank
    /// turbo layout — see [`ForcedInductionUnit`] and TB5.
    pub forced_induction: Vec<ForcedInductionUnit>,
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

        // Sized generously past the total intake valve curtain area, so the
        // valves stay the true bottleneck at wide-open throttle and the plate
        // only restricts when the driver actually lifts. A choked compressible
        // orifice cannot be sized against the old incompressible-Bernoulli
        // model's numbers — that model had no sonic ceiling at all, so a plate
        // sized to only match it chokes well before the old model ever did.
        let intake_valve_area =
            PI * model.valves.intake.diameter.powi(2) / 4.0 * firing.len() as f64;
        let throttle = ThrottleBody::new(
            (4.0 * intake_valve_area / PI).sqrt(),
            0.9,
            PLATE_LEAK_FRACTION,
        );
        let intake = IntakePlenum::at_ambient(
            model.geometry.displacement() * firing.len() as f64 * 0.8,
            throttle,
            &environment,
            &model.gas,
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
                HeatRelease::Spark(_) | HeatRelease::TwoPlug(_) => LimiterCut::Spark,
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
            idle_bypass: 0.0,
            ecu,
            forced_induction: Vec::new(),
        }
    }

    /// Fits a turbocharger driven by the listed banks' exhaust.
    ///
    /// `twin_scroll` with a single bank splits that bank's own cylinders
    /// into the two groups [`FiringOrder::twin_scroll_groups`] picks and
    /// rebuilds its collector to the smaller, divided volume one group's
    /// own cylinders would fill alone — the same
    /// `displacement * cylinders * 1.5` rule [`EngineBlock::new`] already
    /// sizes a whole bank's collector by, applied to half as many
    /// cylinders. A twin-scroll manifold is physically two smaller
    /// collectors merging at the wheel, not one bank's full collector
    /// wearing a different label, and a smaller lumped volume genuinely
    /// resonates differently — see `docs/TURBO_PLAN.md`'s TB5. With two
    /// banks each bank's collector already is that divided size, so
    /// nothing is rebuilt; the flag only changes how
    /// [`EngineBlock::update_manifolds`] feeds the turbine.
    pub fn fit_forced_induction(
        &mut self,
        hardware: ForcedInduction,
        banks: Vec<u8>,
        twin_scroll: bool,
    ) {
        if twin_scroll {
            if let [bank] = banks[..] {
                let groups = self.firing.twin_scroll_groups(bank);
                let scroll_cylinders = groups[0].len().max(1) as f64;
                let bank_cylinders = self.firing.cylinders_on_bank(bank).len().max(1) as f64;
                let manifold = &self.exhaust_banks[bank as usize];
                let scroll_volume = manifold.plenum.volume * scroll_cylinders / bank_cylinders;
                let pressure = manifold.plenum.pressure();
                let temperature = manifold.plenum.temperature;
                self.exhaust_banks[bank as usize].plenum = Plenum::new(
                    scroll_volume,
                    pressure,
                    temperature,
                    self.model.gas.r_burned,
                    self.model.gas.gamma_burned,
                );
            }
        }
        self.forced_induction.push(ForcedInductionUnit {
            hardware,
            banks,
            twin_scroll,
            activation: None,
        });
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

    /// Re-sizes the intake throttle bore from the block's own declared
    /// intake system geometry.
    ///
    /// [`EngineBlock::new`] seeds the throttle off total intake valve
    /// curtain area, a reasonable default before any preset-specific intake
    /// is known. A real preset's [`IntakeSystem::throttle`] is the actual
    /// number — individual throttle bodies add their bores in parallel,
    /// since each one is its own hole into the shared plenum, where a single
    /// central body is just the one bore.
    pub fn rebuild_intake_throttle(&mut self) {
        let bore_area = match self.intake_system.throttle {
            ThrottleLayout::Single { bore } => PI * bore * bore / 4.0,
            ThrottleLayout::IndividualBodies { bore } => {
                self.firing.len() as f64 * PI * bore * bore / 4.0
            }
        };
        self.intake.throttle.bore = (4.0 * bore_area / PI).sqrt().max(1e-4);
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
        let limiter = self.ecu.evaluate_limiter(rpm);
        let dfco = self.ecu.update_dfco(self.throttle, rpm);
        // A cranking engine is fuelled as soon as the ECU has a crank signal to
        // fuel against, and not before: below that the cylinders are gas springs
        // and there is nothing in them to light. It is not the starter letting
        // go that starts the engine — the engine catches under the starter and
        // the starter notices afterwards.
        let no_sync = self.ecu.cranking && rpm.abs() < crate::physics::control::CRANK_SYNC_RPM;
        self.model.fuel_cut = dfco || limiter == LimiterCut::Fuel || no_sync || self.ecu.motoring;
        let cold = self.thermal.cold_fraction();
        // Both spark topologies wet a port and both can misfire lean; a
        // compression-ignition engine injects straight into the cylinder and
        // has no port to wet, so `is_spark_ignited` is the switch here rather
        // than a match on the variant — a two-plug engine takes the same path
        // a single-plug one does.
        let afr = if self.model.combustion.is_spark_ignited() {
            // What the injector is told, and then what actually arrives. A
            // cold engine is commanded rich; a cold port swallows the
            // difference and gives it back a second later, so the mixture the
            // cylinder sees on a start is leaner than anything the schedule
            // ever asks for. Both terms are identities at `cold_fraction ==
            // 0`, so a warm engine is metered exactly what it was metered
            // before any of this existed.
            let scheduled = self.ecu.schedule_afr(load, rpm, self.throttle, frame_dt);
            let commanded = self.ecu.cold_enriched_afr(scheduled, cold);
            let metered = if self.model.fuel_cut { 0.0 } else { 1.0 };
            let delivery = self.ecu.update_wall_film(metered, cold, frame_dt);
            // A cut engine has no mixture, so it has no mixture strength
            // either: the schedule stands and the film quietly drains behind
            // it. Dividing a commanded ratio by a delivery of zero would hand
            // the gas properties a number that means nothing and change what
            // a cut cylinder pumps.
            let effective = if self.model.fuel_cut {
                commanded
            } else if delivery > 1e-3 {
                (commanded / delivery).min(MAX_TRACKED_AFR)
            } else {
                MAX_TRACKED_AFR
            };
            // A charge past the lean limit makes a kernel and no flame. The
            // fuel stays in the cylinder and goes out of the exhaust valve
            // with the rest of the charge — see [`CylinderModel::misfire`].
            self.ecu.misfiring =
                !self.model.fuel_cut && effective > crate::physics::control::LEAN_MISFIRE_AFR;
            self.model.misfire = self.ecu.misfiring;
            self.model.air_fuel_ratio = effective;
            effective
        } else {
            // A diesel injects straight into the cylinder, so it has no port
            // to wet and no film to wait for, and it is lean everywhere by
            // construction rather than by schedule. Nothing above applies.
            self.ecu.misfiring = false;
            self.model.misfire = false;
            self.model.air_fuel_ratio
        };
        self.model.gas = GasProperties::for_afr(afr);
        // Spark timing is the ECU's on an engine that has a coil. A diesel has
        // none: its heat release starts where the Arrhenius integral says, and
        // the latch solves that per cycle. See [`HeatRelease`].
        let spark_angle = self.ecu.spark_angle_with_throttle(load, rpm, self.throttle);
        let wiebe_duration = self.ecu.wiebe_duration(afr);
        self.model
            .combustion
            .set_spark_timing(spark_angle, wiebe_duration);
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
        let thermal = self.thermal.clone();
        let ecu = self.ecu.clone();
        self.ecu.motoring = true;
        let dt = 1.0 / 480.0;
        let frames = (2.0 * 120.0 / rpm / dt).ceil() as usize;
        for _ in 0..frames {
            self.update(dt, rpm);
        }
        // Everything the ECU accumulated over the seed goes back too: the port
        // wall film above all, which would otherwise arrive at the first real
        // injection already full and hand the engine a start it has not had.
        self.ecu = ecu;
        self.thermal = thermal;
    }

    /// Puts every thermal mass back to ambient: an engine that stood overnight.
    pub fn cold_start(&mut self) {
        self.thermal =
            EngineThermal::cold(self.block_mass, &self.exhaust, self.environment.temperature);
    }

    /// Pushes the summed per-bank fluxes into the manifolds.
    fn update_manifolds(&mut self, dt: f64, rpm: f64) {
        // Bank flux is gathered in its own pass, ahead of `integrate`, so a
        // fitted turbine can be given the combined exhaust state below before
        // any bank's collector is advanced this frame.
        struct BankFlux {
            flux: f64,
            temperature: f64,
            open_fraction: f64,
            interval: f64,
        }
        // Anti-lag's overrun fuelling and retard are already real in the
        // ECU — see `is_anti_lag_active` — but until now nothing downstream
        // of it noticed: the extra unburned mixture that reaches the
        // manifold at a shut throttle is exactly what afterburns there in a
        // real anti-lag system, and that afterburn is a genuine heat source
        // this bank's own combustion never produced. Richer fuelling and
        // more retard both mean more unburned mixture surviving to the
        // manifold, so both raise how hot that afterburn can plausibly run.
        // A floor rather than a blend, because once it lights, afterburn
        // combustion burns to roughly its own characteristic temperature
        // rather than diluting down with whatever the baseline flow's own
        // enthalpy happens to be.
        let anti_lag_active = self.ecu.is_anti_lag_active(self.throttle, rpm);
        let bank_flux: Vec<BankFlux> = (0..self.exhaust_banks.len() as u8)
            .map(|bank| {
                let mut flux = 0.0;
                let mut enthalpy_flux = 0.0;
                let mut open_area = 0.0;

                for (i, cyl) in self.firing.cylinders.iter().enumerate() {
                    if cyl.bank != bank {
                        continue;
                    }
                    let sample = self.sample_of(i);
                    // Ring flux is signed into the cylinder, so leaving the
                    // cylinder is a negative exhaust_flow.
                    let out = (-sample.exhaust_flow).max(0.0);
                    flux += out;
                    enthalpy_flux += out * sample.temperature;
                    let theta = wrap_cycle(self.master.cylinder.theta - cyl.firing_offset);
                    open_area += self.model.valves.exhaust.effective_area(theta);
                }

                let temperature = if flux > 1e-12 {
                    enthalpy_flux / flux
                } else {
                    self.exhaust_banks[bank as usize].plenum.temperature
                };
                let temperature = if anti_lag_active {
                    let richness = (STOICH_AFR / self.ecu.anti_lag_afr.max(1.0)).max(1.0);
                    let retard_fraction = (self.ecu.anti_lag_retard / 40.0).clamp(0.0, 1.0);
                    let afterburn_temperature = 1_000.0 + 300.0 * retard_fraction * richness;
                    temperature.max(afterburn_temperature)
                } else {
                    temperature
                };
                let reference_area = PI * self.model.valves.exhaust.diameter.powi(2) / 4.0;
                let open_fraction = (open_area / reference_area.max(1e-12)).clamp(0.0, 1.0);
                let interval = self.firing.mean_bank_interval(bank);
                BankFlux {
                    flux,
                    temperature,
                    open_fraction,
                    interval,
                }
            })
            .collect();

        // The intake plenum is shared, so every cylinder's draw is collected
        // once, whatever bank it lives on. Reversion lasts a handful of crank
        // degrees round overlap — far narrower than a wall-clock frame is
        // wide in crank angle at real rpm — so it is read off the ring's own
        // cycle mean rather than one instantaneous per-cylinder sample: the
        // same smoothing the forward draw already gets for free by summing N
        // phase-offset cylinders, applied in time instead of space. Every
        // cylinder shares one waveform just phase-shifted, so one cylinder's
        // cycle mean is every cylinder's.
        let cylinders = self.firing.len() as f64;
        let forward_flow = cylinders * self.ring.cycle_mean(|s| s.intake_flow.max(0.0));
        let reversion_flow = cylinders * self.ring.cycle_mean(|s| s.intake_flow.min(0.0));
        let reversion_enthalpy = cylinders
            * self
                .ring
                .cycle_mean(|s| s.intake_flow.min(0.0) * s.temperature);
        let reversion_temperature = if reversion_flow.abs() > 1e-9 {
            reversion_enthalpy / reversion_flow
        } else {
            self.environment.temperature
        };
        let intake_draws = [
            ValveDraw::new(forward_flow, self.environment.temperature),
            ValveDraw::new(reversion_flow, reversion_temperature),
        ];

        // Plate clearance and the idle bypass are two holes in parallel round
        // the same plate, but only one of them is a property of the plate.
        // `PLATE_LEAK_FRACTION` is manufacturing clearance round a butterfly
        // that happens to be shut, which is [`ThrottleBody::leak_area_fraction`]'s
        // own native unit — a fraction of *this* plate's own bore — so it is
        // used exactly as declared, whatever bore a preset was cast with.
        //
        // The idle bypass is a separate, real actuator with its own sizing,
        // tracking an engine's idle air demand rather than its WOT throttle
        // body, so [`IDLE_BYPASS_AUTHORITY`] is a fraction of a reference area
        // sized off the cylinder bore instead. The reference is bore, not the
        // live intake valve or port diameter: a named port profile (see
        // `physics::rotor`) can legitimately size its own curtain area far
        // past a stock valve's on purpose, and that is a WOT and reversion
        // claim, not an idle-bypass one — coupling the two would move this
        // engine's idle authority every time a cam or port profile changed,
        // for a reason that has nothing to do with idle.
        let bore_reference_area =
            PI * (self.model.geometry.bore * INTAKE_VALVE_TO_BORE_RATIO).powi(2) / 4.0
                * self.firing.len() as f64;
        let plate_bore_area = self.intake.throttle.bore_area().max(1e-9);
        let bypass_authority = IDLE_BYPASS_AUTHORITY * bore_reference_area / plate_bore_area;
        self.intake.throttle.leak_area_fraction = (PLATE_LEAK_FRACTION
            + bypass_authority * self.idle_bypass.clamp(0.0, 1.0))
        .clamp(0.0, 1.0);
        // Read before this frame's `advance_exhaust` below, so a fitted
        // compressor's shaft is loaded with *this* frame's power draw rather
        // than a frame-stale one. More than one unit's compressor discharges
        // into the same shared charge air ahead of the throttle plate, so
        // their outputs are combined here — weighted by each unit's own mass
        // flow rather than a plain mean, because a unit that is not actually
        // flowing (a spun-down sequential secondary) sits near ambient
        // pressure and would otherwise dilute an active unit's boost by its
        // full share regardless of how little air it is moving. An
        // always-on multi-unit fitment like a per-bank twin turbo has
        // near-equal flows on every unit, so this is a no-op there.
        // The estimate the throttle body hands back is its whole draw, as
        // if only one pipe fed it — correct with exactly one unit fitted,
        // but with more than one every pipe would otherwise be charged for
        // the *entire* draw rather than its own share, N-fold overcounting
        // the mass actually leaving. Split it by last frame's own recorded
        // flow, the same weighting the merge below uses, falling back to an
        // even split before any unit has flowed at all.
        let total_previous_flow: f64 = self
            .forced_induction
            .iter()
            .map(|u| u.hardware.last_mass_flow())
            .sum();
        let unit_count = self.forced_induction.len().max(1) as f64;
        let mut intake_states: Vec<(PortState, f64)> = Vec::new();
        for unit in self.forced_induction.iter_mut() {
            let previous = unit.hardware.upstream_port_state();
            let total_throttle_flow_estimate =
                self.intake.throttle_flow(self.throttle, &previous);
            let share = if total_previous_flow > 1e-9 {
                unit.hardware.last_mass_flow() / total_previous_flow
            } else {
                1.0 / unit_count
            };
            let throttle_flow_estimate = total_throttle_flow_estimate * share;
            let state = unit.hardware.advance_intake(
                dt,
                throttle_flow_estimate,
                self.intake.pressure(),
                &self.environment,
                &self.model.gas,
            );
            intake_states.push((state, unit.hardware.last_mass_flow()));
        }
        let upstream = if intake_states.is_empty() {
            IntakePlenum::ambient_upstream(&self.environment, &self.model.gas)
        } else {
            let total_flow: f64 = intake_states.iter().map(|(_, flow)| flow).sum();
            let n = intake_states.len() as f64;
            let (pressure, temperature) = if total_flow > 1e-9 {
                (
                    intake_states
                        .iter()
                        .map(|(s, flow)| s.pressure * flow)
                        .sum::<f64>()
                        / total_flow,
                    intake_states
                        .iter()
                        .map(|(s, flow)| s.temperature * flow)
                        .sum::<f64>()
                        / total_flow,
                )
            } else {
                (
                    intake_states.iter().map(|(s, _)| s.pressure).sum::<f64>() / n,
                    intake_states
                        .iter()
                        .map(|(s, _)| s.temperature)
                        .sum::<f64>()
                        / n,
                )
            };
            PortState {
                pressure,
                temperature,
                ..intake_states[0].0
            }
        };

        // Each fitted unit sits downstream of only the banks it lists — a
        // per-bank turbo never sees another bank's exhaust, and a
        // twin-scroll unit keeps its two inlets apart all the way to the
        // wheel rather than flux-weighting them into one first (see
        // [`ForcedInductionUnit::twin_scroll`]). Whatever flow a unit hands
        // back is split to its own banks in proportion to their own share
        // of what fed it; any bank no unit reaches vents straight to
        // atmosphere, same as an atmospheric engine's.
        let reference =
            PortConditions::from_environment(&self.environment, &self.model.gas).exhaust;
        let port_state = |pressure: f64, temperature: f64| PortState {
            pressure,
            temperature,
            ..reference
        };
        // Accumulated rather than merely set, because a sequential pair
        // shares one `banks` entry across two units — see
        // [`ForcedInductionUnit::activation`] — and both units' outflow
        // leaves the same collector this frame.
        let mut bank_outflow = vec![0.0_f64; self.exhaust_banks.len()];
        let mut bank_has_induction = vec![false; self.exhaust_banks.len()];
        for idx in 0..self.forced_induction.len() {
            let twin_scroll = self.forced_induction[idx].twin_scroll;
            let banks = self.forced_induction[idx].banks.clone();
            if twin_scroll {
                let inlets: [PortState; 2] = if let [bank] = banks[..] {
                    let groups = self.firing.twin_scroll_groups(bank);
                    let mut states = [port_state(0.0, 0.0); 2];
                    for (g, group) in groups.iter().enumerate() {
                        let mut flux = 0.0;
                        let mut enthalpy_flux = 0.0;
                        for &i in group {
                            let sample = self.sample_of(i);
                            let out = (-sample.exhaust_flow).max(0.0);
                            flux += out;
                            enthalpy_flux += out * sample.temperature;
                        }
                        let temperature = if flux > 1e-12 {
                            enthalpy_flux / flux
                        } else {
                            self.exhaust_banks[bank as usize].plenum.temperature
                        };
                        states[g] = port_state(
                            self.exhaust_banks[bank as usize].port_pressure(),
                            temperature,
                        );
                    }
                    states
                } else {
                    let mut states = [port_state(0.0, 0.0); 2];
                    for (g, &bank) in banks.iter().enumerate().take(2) {
                        states[g] = port_state(
                            self.exhaust_banks[bank as usize].port_pressure(),
                            self.exhaust_banks[bank as usize].plenum.temperature,
                        );
                    }
                    states
                };
                let outflows = self.forced_induction[idx].hardware.advance_exhaust_scrolls(
                    dt,
                    &inlets,
                    self.environment.pressure,
                );
                if let [bank] = banks[..] {
                    bank_outflow[bank as usize] += outflows[0] + outflows[1];
                    bank_has_induction[bank as usize] = true;
                } else {
                    for (g, &bank) in banks.iter().enumerate().take(2) {
                        bank_outflow[bank as usize] += outflows[g];
                        bank_has_induction[bank as usize] = true;
                    }
                }
            } else {
                // A sequential secondary's own valve doubles as its turbine's
                // effective nozzle area fraction — see [`TurboShaft::advance`]
                // — a shut valve is a zero-area nozzle, drawing no power and
                // passing no flow whatever the bank's own pressure is doing,
                // and an open one is a fixed-geometry fitment's ordinary 1.0.
                // `None` (no valve fitted) always reads back exactly 1.0, so
                // every existing fitment is unaffected.
                let activation = self.forced_induction[idx]
                    .activation
                    .as_mut()
                    .map(|valve| valve.update(dt, rpm))
                    .unwrap_or(1.0);
                let total: f64 = banks.iter().map(|&b| bank_flux[b as usize].flux).sum();
                let (pressure, temperature) = if total > 1e-12 {
                    let pressure = banks
                        .iter()
                        .map(|&b| {
                            self.exhaust_banks[b as usize].port_pressure()
                                * bank_flux[b as usize].flux
                        })
                        .sum::<f64>()
                        / total;
                    let temperature = banks
                        .iter()
                        .map(|&b| {
                            self.exhaust_banks[b as usize].plenum.temperature
                                * bank_flux[b as usize].flux
                        })
                        .sum::<f64>()
                        / total;
                    (pressure, temperature)
                } else {
                    let n = banks.len().max(1) as f64;
                    (
                        banks
                            .iter()
                            .map(|&b| self.exhaust_banks[b as usize].port_pressure())
                            .sum::<f64>()
                            / n,
                        banks
                            .iter()
                            .map(|&b| self.exhaust_banks[b as usize].plenum.temperature)
                            .sum::<f64>()
                            / n,
                    )
                };
                let turbine_upstream = port_state(pressure, temperature);
                let outflow = self.forced_induction[idx].hardware.advance_exhaust(
                    dt,
                    &turbine_upstream,
                    self.environment.pressure,
                    activation,
                );
                for &b in &banks {
                    let share = if total > 1e-12 {
                        bank_flux[b as usize].flux / total
                    } else {
                        1.0 / banks.len().max(1) as f64
                    };
                    bank_outflow[b as usize] += outflow * share;
                    bank_has_induction[b as usize] = true;
                }
            }
        }

        for (bank, flux) in bank_flux.iter().enumerate() {
            self.exhaust_banks[bank].integrate(
                dt,
                rpm,
                flux.interval,
                flux.flux,
                flux.temperature,
                flux.open_fraction,
                bank_has_induction[bank].then_some(bank_outflow[bank]),
            );
        }

        self.intake
            .advance(dt, self.throttle, &upstream, &intake_draws);
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
        let afr = if self.model.combustion.is_spark_ignited() {
            self.ecu.target_afr(0.8, rpm, 1.0)
        } else {
            18.0 // lean diesel combustion
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
mod tests;
