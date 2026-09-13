//! The structural path: what the block itself radiates.
//!
//! # Why an engine needs a second radiating path
//!
//! Everything else in [`crate::audio`] is air: gas leaves a port, travels a
//! pipe, and a mouth radiates it. That is not how most of a diesel reaches the
//! listener, and it is not how any engine's idle does. Cylinder pressure acts on
//! the piston crown and on the head face, and the difference between those two
//! forces is reacted by the block — so every combustion event is also a hammer
//! blow delivered to a large lump of metal, which answers at its own modes and
//! radiates from its own surfaces. In NVH that path has a name, *combustion
//! noise*, and its governing quantity is not peak pressure but `dP/dtheta`: a
//! charge that arrives all at once puts energy at the frequencies the structure
//! actually responds at, and one that arrives gently does not. Two engines can
//! reach the same 60 bar peak and sound nothing alike for that reason alone.
//! It is the whole difference between a diesel's clatter and a petrol engine's
//! thump.
//!
//! # What is modelled here
//!
//! A filterbank of six resonances standing for three physical members, driven
//! in parallel and summed:
//!
//! - **Global bending of the dressed block**, three modes. The block with heads,
//!   pan and accessories hung on it is a beam-like lump on soft mounts, and its
//!   lowest mode is the `f = (1/2 pi) sqrt(K/m)` law
//!   [`crate::audio::filters::block_resonance_hz`] already ships — which this
//!   bank keeps exactly, as its anchor. The two above it follow the free-free
//!   Euler-Bernoulli eigenvalues.
//! - **Oil pan drumming**, one mode. A thin stamped panel over the crankcase
//!   footprint, which is the largest single radiating surface on the engine and
//!   the reason a bare block on a stand sounds nothing like one in a car.
//! - **Bore wall ovalling**, two modes. The cylinder walls are thin rings driven
//!   from inside by the same pressure, and they ring in the 2-8 kHz band. This
//!   is the bright end of clatter.
//!
//! Above the highest of those the mode count of a real block runs into the
//! hundreds and the individual peaks merge into a smooth plateau. That is
//! carried here as a residual term rather than as more filters: the bank passes
//! its input through at unity and adds the resolved modes on top, so the
//! plateau is the reference every source level in the synth is already measured
//! against and the modes are a departure from it.
//!
//! # The shape the bank stands for, and why it is bandpasses
//!
//! What a listener hears is surface *acceleration*, and the acceleration a
//! single mode gives up per unit of applied force rises as `omega^2` below its
//! resonance, where the structure is a spring, and flattens above it, where the
//! structure is a mass. A constant-peak-gain bandpass is `omega` below its
//! centre and `1/omega` above it, so a bandpass fed the *rate* of a force
//! delivers exactly that shape: `omega * omega` on the way up and `omega / omega`
//! on the way down. That is why combustion enters as `dP/dtheta` and not as `P`,
//! and it is the same reason a block does not radiate the mean cylinder
//! pressure it spends its whole life holding.
//!
//! The other two sources arrive as forces rather than as rates — a lifter
//! landing is an impulse, not the derivative of one — so for them the flat
//! residual *is* the mass-controlled plateau, with the modes standing on top of
//! it. Each source is normalised against its own physical reference, which is
//! what makes three different quantities addable at the bank's input.
//!
//! # Where the mass law lives
//!
//! An alloy block and an iron one of the same displacement differ far more in
//! density than in section, which is why the light one rings *higher* rather
//! than merely quieter. Mass therefore enters twice and in the same direction:
//! directly in the bending anchor, and through the smeared density of the block
//! envelope in the bore-wall speed of sound. Nothing in this module is typed in
//! per engine.

use crate::audio::filters::{block_resonance_hz, Biquad, BiquadCoeffs};

// ---------------------------------------------------------------------------
// Material and shape constants
// ---------------------------------------------------------------------------

/// Effective bending modulus of a dressed engine block [Pa].
///
/// Held fixed across the catalogue on purpose. Cast iron is 110 GPa and
/// aluminium 70, but an alloy block is cast with thicker sections precisely so
/// that it is not floppier than the iron one it replaced — the two end up within
/// a few per cent of each other in *stiffness*, and a factor of two and a half
/// apart in density. Treating stiffness as a constant and letting mass carry the
/// difference is therefore the physical statement, not a simplification of one.
pub const BLOCK_MODULUS: f32 = 100.0e9;

/// Bore centre spacing as a multiple of bore diameter [-].
///
/// Production practice sits between 1.15 and 1.30: below that there is no room
/// for a head bolt or a water jacket between the bores, and above it the engine
/// is longer and heavier than it needs to be. Used when a preset does not state
/// its own spacing.
pub const BORE_SPACING_RATIO: f64 = 1.22;

/// Crankcase width at the pan rail, as a multiple of bore spacing [-].
///
/// The pan rail follows the bores outboard of the cylinder walls, and it does
/// so whether the engine is an inline or a vee — the vee's extra width is in the
/// heads, which are above the joint and not part of this footprint.
pub const PAN_WIDTH_RATIO: f32 = 1.7;

/// Block height from pan floor to cam cover, as a multiple of bore spacing [-].
pub const BLOCK_HEIGHT_RATIO: f32 = 4.0;

/// Thinnest cylinder wall the model will assume [m].
///
/// The wall between two bores is `(spacing - bore) / 2` and gets thin fast on a
/// short-deck engine. A high-output V10 on 90 mm centres with an 84.5 mm bore
/// is down to 2.75 mm of iron between one cylinder and the next, which is about
/// as far as a siamesed block goes; nothing in the catalogue is thinner and
/// neither is this.
pub const MIN_BORE_WALL: f32 = 0.0025;

/// Young's modulus of a stamped steel oil pan [Pa].
pub const PAN_MODULUS: f32 = 200.0e9;

/// Density of a stamped steel oil pan [kg/m^3].
pub const PAN_DENSITY: f32 = 7_850.0;

/// Poisson's ratio of steel [-].
pub const PAN_POISSON: f32 = 0.30;

/// Sheet thickness of a stamped oil pan [m].
pub const PAN_THICKNESS: f32 = 0.0020;

/// Free-free Euler-Bernoulli bending eigenvalues `beta_n * L` [-].
///
/// The roots of `cos(beta L) cosh(beta L) = 1`. Bending frequency goes as the
/// square of these, so the first three modes of any free-free beam stand in the
/// ratio 1 : 2.76 : 5.40 whatever it is made of or how long it is — which is why
/// they can be applied to a mode frequency that came from somewhere else.
pub const BEAM_EIGENVALUES: [f32; 3] = [4.730_041, 7.853_205, 10.995_608];

/// Structural loss factor of the dressed block in bending [-].
///
/// Cast iron's own material damping is nearer 0.001. What sets this is
/// everything bolted to it: the head and pan joints, the mounts, and the oil.
/// A loss factor of 0.08 is a Q of 12.5, which puts the 80 Hz mode's ring-down
/// at 50 ms — long enough to sustain between firings at idle, short enough that
/// the block reads as a lump of metal rather than a drum.
pub const BLOCK_LOSS_FACTOR: f32 = 0.08;

/// Structural loss factor of the oil pan [-].
///
/// A pan is a thin panel with several litres of oil hanging off the inside of
/// it, and the fluid loading is the damping. Broad and short-lived: it adds
/// weight to the note without adding a pitch.
pub const PAN_LOSS_FACTOR: f32 = 0.25;

/// Structural loss factor of a cylinder wall [-].
///
/// The stiffest, driest and best-supported member of the three, and the only one
/// that genuinely rings: a Q of 29 at 2.6 kHz decays in 3.5 ms, which is the
/// length of one tick of clatter.
pub const BORE_WALL_LOSS_FACTOR: f32 = 0.035;

// ---------------------------------------------------------------------------
// Bank constants
// ---------------------------------------------------------------------------

/// How far the strongest resolved mode stands above the unresolved plateau [-].
///
/// The bank's broadband transmission is unity by construction — that is the
/// plateau the hundreds of unmodelled high modes merge into — and the resolved
/// peaks rise above it by this factor. 2.5 is 8 dB, which is the order a block's
/// measured mobility shows between its low modes and its high-frequency floor
/// once the joints and mounts have done their damping.
pub const MODAL_PEAK_RATIO: f32 = 2.5;

/// Number of resolved modes in the bank.
pub const STRUCTURAL_MODES: usize = 6;

/// Band a derived mode is clamped into [Hz].
///
/// Outside it the formula is no longer describing the member it was written
/// for — a 10 mm bore's ring mode really is ultrasonic, and a block heavy
/// enough to bend below 20 Hz is not a block. Same convention as
/// [`crate::audio::filters::block_resonance_hz`]'s own clamp: refuse to
/// extrapolate rather than emit a frequency nothing can hear.
pub const STRUCTURAL_BAND_HZ: (f32, f32) = (20.0, 16_000.0);

// ---------------------------------------------------------------------------
// Modes
// ---------------------------------------------------------------------------

/// One resolved structural mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StructuralMode {
    /// Resonant frequency [Hz].
    pub frequency: f32,
    /// Sharpness, `1 / loss_factor` [-].
    pub q: f32,
    /// Share of the radiated output this mode carries [-].
    pub weight: f32,
}

/// The geometry and mass a block's modes are derived from.
///
/// Everything here is something a preset already knows about itself. There is no
/// frequency, no Q and no mix level in this struct, because those are results.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StructuralSpec {
    /// Dressed mass of the block: casting, heads, pan and accessories [kg].
    pub dressed_mass: f64,
    /// Cylinder bore diameter [m].
    pub bore: f64,
    /// Distance between adjacent bore centres [m].
    pub bore_spacing: f64,
    /// Cylinders on one bank — the number the block's length is set by [-].
    pub cylinders_per_bank: usize,
}

impl Default for StructuralSpec {
    /// The block [`crate::audio::filters::BLOCK_REFERENCE_MASS`] describes: a
    /// four-litre iron-decked V8, four cylinders to a bank.
    fn default() -> Self {
        Self::new(180.0, 0.094, 4)
    }
}

impl StructuralSpec {
    /// A spec with production bore spacing for the given bore.
    pub fn new(dressed_mass: f64, bore: f64, cylinders_per_bank: usize) -> Self {
        Self {
            dressed_mass,
            bore,
            bore_spacing: bore * BORE_SPACING_RATIO,
            cylinders_per_bank,
        }
    }

    /// Overrides the bore centre spacing [m].
    ///
    /// For the engines whose chambers are not on a production four-stroke
    /// pitch: a rotary's housings, or anything with a main bearing between
    /// every pair of bores.
    pub fn with_bore_spacing(mut self, spacing: f64) -> Self {
        self.bore_spacing = spacing;
        self
    }

    /// Bore centre spacing, clamped to something a block could be cast as [m].
    fn spacing(&self) -> f32 {
        let bore = self.bore.max(0.010) as f32;
        (self.bore_spacing as f32).clamp(bore + 2.0 * MIN_BORE_WALL, 3.0 * bore)
    }

    /// Deck length: one bore spacing per cylinder plus a half web at each end [m].
    fn deck_length(&self) -> f32 {
        (self.cylinders_per_bank.max(1) as f32 + 1.0) * self.spacing()
    }

    /// Crankcase width at the pan rail [m].
    fn pan_width(&self) -> f32 {
        PAN_WIDTH_RATIO * self.spacing()
    }

    /// Block height from pan floor to cam cover [m].
    fn height(&self) -> f32 {
        BLOCK_HEIGHT_RATIO * self.spacing()
    }

    /// Smeared density of the block envelope [kg/m^3].
    ///
    /// Dressed mass over the box the engine occupies, so it is well below the
    /// density of the metal — most of that box is air, water and oil. What
    /// matters is that it moves the right way: the same envelope in aluminium
    /// weighs a third of what it does in iron, and structure-borne sound crosses
    /// it at `sqrt(E / rho)`.
    fn envelope_density(&self) -> f32 {
        let volume = self.deck_length() * self.pan_width() * self.height();
        (self.dressed_mass as f32 / volume.max(1e-6)).clamp(500.0, 8_000.0)
    }

    /// Frequency of the `n`-th global bending mode, `n` from zero [Hz].
    ///
    /// The first is the anchor law itself — a lump of mass `m` on a structure of
    /// fixed stiffness `K`, at `(1/2 pi) sqrt(K/m)` — and the rest follow it by
    /// the square of the free-free eigenvalue ratio. So every mode in the family
    /// carries the `1 / sqrt(mass)` law, and a heavy block rings lower all the
    /// way up.
    pub fn bending_hz(&self, n: usize) -> f32 {
        let first = block_resonance_hz(self.dressed_mass as f32);
        let ratio = BEAM_EIGENVALUES[n.min(BEAM_EIGENVALUES.len() - 1)] / BEAM_EIGENVALUES[0];
        first * ratio * ratio
    }

    /// Fundamental of the oil pan, as a thin rectangular plate [Hz].
    ///
    /// ```text
    /// f = (pi h / 2) sqrt(E / (12 rho (1 - nu^2))) (1/a^2 + 1/b^2)
    /// ```
    ///
    /// `a` is the deck length and `b` the pan rail width, so a long engine's pan
    /// drums lower than a short one's for the same reason any long panel does.
    /// Steel and 2 mm whatever the block is made of: a pan is a pressing, and it
    /// is the same pressing on an alloy engine as on an iron one.
    pub fn pan_hz(&self) -> f32 {
        let a = self.deck_length();
        let b = self.pan_width();
        let stiffness =
            (PAN_MODULUS / (12.0 * PAN_DENSITY * (1.0 - PAN_POISSON * PAN_POISSON))).sqrt();
        0.5 * std::f32::consts::PI * PAN_THICKNESS * stiffness * (1.0 / (a * a) + 1.0 / (b * b))
    }

    /// Frequency of the `n`-nodal-diameter flexural mode of a bore wall [Hz].
    ///
    /// A cylinder wall is a thin ring of radius `a = bore / 2` and thickness
    /// `h = (spacing - bore) / 2` — the metal that is actually there between one
    /// bore and the next, which is why bore spacing and not bore alone sets the
    /// pitch. The free ring's flexural modes are
    ///
    /// ```text
    /// f_n = n (n^2 - 1) / sqrt(n^2 + 1) * h sqrt(E / rho) / (2 pi sqrt(12) a^2)
    /// ```
    ///
    /// a `1 / bore` law like the knock cavity modes, but in the metal rather
    /// than in the gas — so it moves with the block's density and not with the
    /// charge temperature.
    pub fn bore_wall_hz(&self, n: f32) -> f32 {
        let bore = self.bore.max(0.010) as f32;
        let a = 0.5 * bore;
        let h = (0.5 * (self.spacing() - bore)).max(MIN_BORE_WALL);
        let c = (BLOCK_MODULUS / self.envelope_density()).sqrt();
        let shape = n * (n * n - 1.0) / (n * n + 1.0).sqrt();
        shape * h * c / (2.0 * std::f32::consts::PI * 12.0f32.sqrt() * a * a)
    }

    /// The six modes, in families rather than in frequency order.
    ///
    /// # Where the weights come from
    ///
    /// A mode radiates in proportion to the surface it moves, and to how much of
    /// that surface survives its own cancellation. Both halves are geometry:
    ///
    /// - **Area.** The bending modes shake the four sides of the block, the pan
    ///   mode its floor, and the bore walls reach the outside through the deck.
    ///   Each family's weight starts as its share of the total radiating area.
    /// - **Cancellation.** A mode with `n` antinodes has `n - 1` internal nodes,
    ///   and adjacent antinodes move in antiphase — so in the far field all but
    ///   about one antinode's worth cancels, and the surviving fraction goes as
    ///   `1 / n`. That is why the higher member of each family is quieter
    ///   without anything being turned down.
    ///
    /// Normalised so the strongest mode has weight one; the bank turns that into
    /// a peak of [`MODAL_PEAK_RATIO`] over its own broadband plateau.
    pub fn modes(&self) -> [StructuralMode; STRUCTURAL_MODES] {
        let length = self.deck_length();
        let width = self.pan_width();
        let height = self.height();

        // The four vertical faces of the block, its floor, and its deck.
        let sides = 2.0 * length * height + 2.0 * width * height;
        let floor = length * width;
        let deck = floor;
        let total = (sides + floor + deck).max(1e-9);
        let (sides, floor, deck) = (sides / total, floor / total, deck / total);

        let block_q = 1.0 / BLOCK_LOSS_FACTOR;
        let wall_q = 1.0 / BORE_WALL_LOSS_FACTOR;
        let mut modes = [
            StructuralMode {
                frequency: self.bending_hz(0),
                q: block_q,
                weight: sides,
            },
            StructuralMode {
                frequency: self.bending_hz(1),
                q: block_q,
                weight: sides / 2.0,
            },
            StructuralMode {
                frequency: self.bending_hz(2),
                q: block_q,
                weight: sides / 3.0,
            },
            StructuralMode {
                frequency: self.pan_hz(),
                q: 1.0 / PAN_LOSS_FACTOR,
                weight: floor,
            },
            StructuralMode {
                frequency: self.bore_wall_hz(2.0),
                q: wall_q,
                weight: deck / 2.0,
            },
            StructuralMode {
                frequency: self.bore_wall_hz(3.0),
                q: wall_q,
                weight: deck / 3.0,
            },
        ];

        for mode in modes.iter_mut() {
            mode.frequency = mode
                .frequency
                .clamp(STRUCTURAL_BAND_HZ.0, STRUCTURAL_BAND_HZ.1);
        }

        let peak = modes
            .iter()
            .fold(0.0f32, |acc, mode| acc.max(mode.weight))
            .max(1e-9);
        for mode in modes.iter_mut() {
            mode.weight /= peak;
        }
        modes
    }
}

// ---------------------------------------------------------------------------
// Combustion drive
// ---------------------------------------------------------------------------

/// Cylinder pressure rise rate that maps to unit structural drive [Pa/s].
///
/// A petrol engine at normal load runs 2 to 4 bar per crank degree through the
/// pressure rise, which at 3000 rpm is 4 to 8 GPa/s; a direct-injection diesel
/// reaches several times that, and *that* is what its extra clatter is. 5 GPa/s
/// therefore sits inside the petrol range with the steeper cases left free to
/// run above unity, which is the right way round for a reference.
pub const REFERENCE_PRESSURE_RATE: f32 = 5.0e9;

/// Bore whose piston area normalises the combustion force rate [m].
///
/// What the block feels is `A dP/dt`, not `dP/dt`, so a big engine radiates more
/// structure-borne noise from the same rise rate. Normalised on the bore of the
/// block [`crate::audio::filters::BLOCK_REFERENCE_MASS`] describes, so the
/// shipped V8 sits at unity.
pub const REFERENCE_STRUCTURAL_BORE: f32 = 0.094;

/// Structural drive from a cylinder pressure rise rate [-].
///
/// `pressure_rate` in Pascals per second and `bore` in metres, against the two
/// references above. Linear in both, which is the claim the whole path rests on:
/// at the same peak pressure, the steeper rise drives the structure harder,
/// because what reaches the block is the rate and not the height.
#[inline]
pub fn combustion_drive(pressure_rate: f32, bore: f32) -> f32 {
    let area_ratio = (bore / REFERENCE_STRUCTURAL_BORE).powi(2);
    pressure_rate / REFERENCE_PRESSURE_RATE * area_ratio
}

// ---------------------------------------------------------------------------
// Reciprocating shaking-force drive
// ---------------------------------------------------------------------------

/// Reciprocating shaking-force resultant that maps to unit structural drive
/// [N].
///
/// The shipped cross-plane V8 — 94 mm bore, 0.51 kg reciprocating mass —
/// resolves a resultant of about 500 N at redline once its own near-complete
/// cancellation is accounted for, and a few newtons at idle: this reference
/// sits just above the redline figure, so that engine's own shake stays
/// clearly under unity even at the top of its rev range, in the same spirit
/// as [`REFERENCE_PRESSURE_RATE`] leaving the steeper cases free to run above
/// it. Unlike that reference this one needs no separate bore-area term: bore
/// already sets [`crate::physics::cylinder::default_reciprocating_mass`], so
/// a bigger cylinder already shakes harder before this constant is ever
/// applied. A flat-plane engine, whose secondary forces do not cancel the way
/// a crossplane crank's do, comes out well above unity here at the same
/// speed and mass — a real difference between the two layouts, not an
/// artefact of the reference.
pub const REFERENCE_SHAKING_FORCE: f32 = 10_000.0;

/// Structural drive from a reciprocating shaking-force resultant [-].
///
/// `force` in newtons, against the reference above. Linear, for the same
/// reason [`combustion_drive`] is: the block cannot radiate a shake
/// nonlinearly just because this crate normalised it.
#[inline]
pub fn shaking_drive(force: f32) -> f32 {
    force / REFERENCE_SHAKING_FORCE
}

// ---------------------------------------------------------------------------
// The filterbank
// ---------------------------------------------------------------------------

/// The block as a radiating structure: a modal filterbank with a residual.
///
/// Input is a normalised structural force — combustion, mechanical impacts and
/// knock all arrive here as one drive signal, because the block cannot tell them
/// apart either. Output is mono: the block is one object radiating a four-metre
/// wavelength at its lowest mode, so there is nothing to image.
///
/// Retuned by nobody. Every frequency in it is geometry and mass, and neither
/// changes while the stream is open, so the bank is designed once at
/// construction and the audio thread only ever runs the difference equations.
#[derive(Debug, Clone)]
pub struct StructuralPath {
    modes: [Biquad; STRUCTURAL_MODES],
    gains: [f32; STRUCTURAL_MODES],
    frequencies: [f32; STRUCTURAL_MODES],
    qs: [f32; STRUCTURAL_MODES],
}

impl StructuralPath {
    /// Builds the bank from a mode list.
    ///
    /// The weights are taken as given and scaled to [`MODAL_PEAK_RATIO`]
    /// relative to the residual; nothing here knows what a block is.
    pub fn new(sample_rate: f32, modes: &[StructuralMode; STRUCTURAL_MODES]) -> Self {
        let mut path = Self {
            modes: [Biquad::default(); STRUCTURAL_MODES],
            gains: [0.0; STRUCTURAL_MODES],
            frequencies: [0.0; STRUCTURAL_MODES],
            qs: [0.0; STRUCTURAL_MODES],
        };
        for (i, mode) in modes.iter().enumerate() {
            let frequency = mode.frequency.clamp(5.0, 0.45 * sample_rate);
            path.modes[i] = Biquad::new(BiquadCoeffs::bandpass(sample_rate, frequency, mode.q));
            path.gains[i] = mode.weight * (MODAL_PEAK_RATIO - 1.0);
            path.frequencies[i] = frequency;
            path.qs[i] = mode.q;
        }
        path
    }

    /// Mode frequencies as built [Hz].
    pub fn mode_frequencies(&self) -> [f32; STRUCTURAL_MODES] {
        self.frequencies
    }

    /// Mode sharpnesses as built [-].
    pub fn mode_qs(&self) -> [f32; STRUCTURAL_MODES] {
        self.qs
    }

    /// Radiates one sample of structural drive.
    ///
    /// The bare input term is the residual: the mode continuum above the
    /// resolved band, which is dense enough to be flat and is what makes this a
    /// transfer function with peaks rather than a set of six tuned tones.
    #[inline(always)]
    pub fn process(&mut self, drive: f32) -> f32 {
        let mut out = drive;
        for (mode, gain) in self.modes.iter_mut().zip(self.gains.iter()) {
            out += mode.process(drive) * gain;
        }
        out
    }

    /// Clears every mode's state.
    pub fn reset(&mut self) {
        self.modes.iter_mut().for_each(Biquad::reset);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::audio::filters::Noise;
    use std::f32::consts::TAU;

    const FS: f32 = 48_000.0;

    /// Six modes spread far enough apart to be measured one at a time.
    fn spread() -> [StructuralMode; STRUCTURAL_MODES] {
        let mut modes = [StructuralMode {
            frequency: 0.0,
            q: 10.0,
            weight: 0.0,
        }; STRUCTURAL_MODES];
        for (i, mode) in modes.iter_mut().enumerate() {
            mode.frequency = 80.0 * 3.0f32.powi(i as i32);
            mode.weight = 1.0 / (i as f32 + 1.0);
        }
        modes
    }

    /// Steady-state magnitude response at one frequency, by quadrature
    /// correlation after the transient has passed.
    fn magnitude_at(path: &mut StructuralPath, frequency: f32) -> f32 {
        let n = 32_768;
        let (mut re, mut im) = (0.0f32, 0.0f32);
        for i in 0..n {
            let t = i as f32 / FS;
            let phase = TAU * frequency * t;
            let y = path.process(phase.sin());
            if i >= n / 2 {
                re += y * phase.sin();
                im += y * phase.cos();
            }
        }
        let half = (n / 2) as f32;
        2.0 * (re * re + im * im).sqrt() / half
    }

    #[test]
    fn the_first_mode_is_the_block_resonance_law() {
        // The anchor: whatever else the bank grows, mode one is exactly the
        // rumble the synth has always had, so the existing calibration survives.
        for mass in [95.0, 110.0, 180.0, 240.0, 320.0] {
            let spec = StructuralSpec::new(mass, 0.094, 4);
            let law = block_resonance_hz(mass as f32);
            assert!(
                (spec.modes()[0].frequency - law).abs() < 1e-4,
                "mode one is {} Hz, the law says {law} Hz",
                spec.modes()[0].frequency
            );
        }
    }

    #[test]
    fn every_bending_mode_scales_as_one_over_root_mass() {
        // Four times the mass, half the frequency — in all three bending modes,
        // because they share one stiffness and one mass.
        let light = StructuralSpec::new(80.0, 0.094, 4);
        let heavy = StructuralSpec::new(320.0, 0.094, 4);
        for n in 0..BEAM_EIGENVALUES.len() {
            let ratio = light.bending_hz(n) / heavy.bending_hz(n);
            assert!(
                (ratio - 2.0).abs() < 0.01,
                "mode {n}: {ratio} instead of 2 for a quarter of the mass"
            );
        }
    }

    #[test]
    fn bending_modes_stand_in_the_free_free_beam_ratios() {
        // A free-free beam's modes go as (beta_n L)^2, whatever it is made of:
        // 1 : 2.76 : 5.40.
        let spec = StructuralSpec::default();
        let first = spec.bending_hz(0);
        for (n, beta) in BEAM_EIGENVALUES.iter().enumerate() {
            let expected = first * (beta / BEAM_EIGENVALUES[0]).powi(2);
            assert!(
                (spec.bending_hz(n) - expected).abs() < 1e-3,
                "mode {n} at {} Hz, beam theory says {expected} Hz",
                spec.bending_hz(n)
            );
        }
    }

    #[test]
    fn mass_moves_every_mode_the_block_owns() {
        // Five of the six modes are the block's own metal and all of them carry
        // the mass law, by two different routes: the bending family through the
        // `sqrt(K/m)` anchor, and the bore walls through the envelope density
        // under `sqrt(E/rho)`. Quarter the mass and both halve.
        let heavy = StructuralSpec::new(320.0, 0.094, 4);
        let light = StructuralSpec::new(80.0, 0.094, 4);
        let (heavy_modes, light_modes) = (heavy.modes(), light.modes());

        for i in [0, 1, 2, 4, 5] {
            let ratio = light_modes[i].frequency / heavy_modes[i].frequency;
            assert!(
                (ratio - 2.0).abs() < 0.02,
                "mode {i} moved by {ratio} instead of 2 for a quarter of the mass"
            );
        }

        // The sixth is the pan, which is a steel pressing bolted underneath and
        // does not know what the block is made of.
        assert!(
            (light_modes[3].frequency - heavy_modes[3].frequency).abs() < 1e-3,
            "the pan followed the block's mass: {} vs {}",
            light_modes[3].frequency,
            heavy_modes[3].frequency
        );

        // And the whole bank really is lower on the heavy block, which is the
        // audible form of the same statement.
        let mean = |modes: [StructuralMode; STRUCTURAL_MODES]| {
            modes.iter().map(|m| m.frequency.ln()).sum::<f32>() / STRUCTURAL_MODES as f32
        };
        assert!(mean(light_modes) > mean(heavy_modes));
    }

    #[test]
    fn an_alloy_block_rings_higher_than_an_iron_one_of_the_same_size() {
        // The asymmetry the whole module rests on: same castings, same bores,
        // two thirds of the mass. Stiffness barely moves, so every family goes
        // up — the bending modes through the mass law, the bore walls through
        // the envelope density.
        let iron = StructuralSpec::new(240.0, 0.094, 4);
        let alloy = StructuralSpec::new(150.0, 0.094, 4);
        assert!(
            alloy.bending_hz(0) > iron.bending_hz(0) * 1.2,
            "alloy bending {} vs iron {}",
            alloy.bending_hz(0),
            iron.bending_hz(0)
        );
        assert!(
            alloy.bore_wall_hz(2.0) > iron.bore_wall_hz(2.0) * 1.2,
            "alloy bore wall {} vs iron {}",
            alloy.bore_wall_hz(2.0),
            iron.bore_wall_hz(2.0)
        );
        // The pan is a steel pressing either way, and does not move.
        assert!((alloy.pan_hz() - iron.pan_hz()).abs() < 1e-3);
    }

    #[test]
    fn a_thicker_wall_between_the_bores_rings_higher() {
        // Bore spacing, not bore, sets the metal that is doing the ringing.
        let tight = StructuralSpec::new(180.0, 0.094, 4).with_bore_spacing(0.100);
        let generous = StructuralSpec::new(180.0, 0.094, 4).with_bore_spacing(0.120);
        assert!(
            generous.bore_wall_hz(2.0) > tight.bore_wall_hz(2.0),
            "{} Hz on 120 mm centres against {} Hz on 100",
            generous.bore_wall_hz(2.0),
            tight.bore_wall_hz(2.0)
        );
    }

    #[test]
    fn a_longer_block_drums_lower() {
        // A pan is a panel: stretch it and its fundamental falls.
        let four = StructuralSpec::new(180.0, 0.086, 4);
        let six = StructuralSpec::new(180.0, 0.086, 6);
        assert!(
            six.pan_hz() < four.pan_hz(),
            "six-cylinder pan at {} Hz, four at {}",
            six.pan_hz(),
            four.pan_hz()
        );
    }

    #[test]
    fn derived_modes_are_ordered_finite_and_audible() {
        for spec in [
            StructuralSpec::default(),
            StructuralSpec::new(95.0, 0.105, 2),
            StructuralSpec::new(260.0, 0.084, 6),
            // Degenerate: no cylinders, a zero bore, a nonsense spacing.
            StructuralSpec::new(0.0, 0.0, 0).with_bore_spacing(-1.0),
        ] {
            for mode in spec.modes() {
                assert!(
                    mode.frequency.is_finite() && (5.0..20_000.0).contains(&mode.frequency),
                    "{mode:?} is not an audible structural mode"
                );
                assert!(mode.q.is_finite() && mode.q > 0.5, "{mode:?} has no Q");
                assert!(
                    mode.weight.is_finite() && (0.0..=1.0).contains(&mode.weight),
                    "{mode:?} is not normalised"
                );
            }
        }
    }

    #[test]
    fn the_residual_passes_everything_at_unity() {
        // Far from every mode the bank is a wire. That is what makes a source
        // level mean the same thing before and after the structure.
        let mut path = StructuralPath::new(FS, &spread());
        let far = magnitude_at(&mut path, 12_000.0);
        assert!(
            (far - 1.0).abs() < 0.15,
            "residual is not unity off-mode: {far}"
        );
    }

    #[test]
    fn the_strongest_mode_peaks_at_the_modal_ratio() {
        let modes = spread();
        let mut path = StructuralPath::new(FS, &modes);
        let peak = magnitude_at(&mut path, modes[0].frequency);
        assert!(
            (peak - MODAL_PEAK_RATIO).abs() < 0.2,
            "strongest mode peaked at {peak}, expected {MODAL_PEAK_RATIO}"
        );
        // And a half-weight mode stands half as far above the plateau.
        let mut path = StructuralPath::new(FS, &modes);
        let second = magnitude_at(&mut path, modes[1].frequency);
        let expected = 1.0 + 0.5 * (MODAL_PEAK_RATIO - 1.0);
        assert!(
            (second - expected).abs() < 0.2,
            "second mode peaked at {second}, expected {expected}"
        );
    }

    #[test]
    fn modes_sit_where_they_were_asked_to() {
        let modes = spread();
        let path = StructuralPath::new(FS, &modes);
        for (built, asked) in path.mode_frequencies().iter().zip(modes.iter()) {
            assert!(
                (built - asked.frequency).abs() < 1e-3,
                "built {built} Hz for a requested {} Hz",
                asked.frequency
            );
        }
        assert_eq!(path.mode_qs()[0], modes[0].q);
    }

    #[test]
    fn a_mode_above_nyquist_does_not_go_unstable() {
        let mut modes = spread();
        modes[5].frequency = 4.0 * FS;
        let mut path = StructuralPath::new(FS, &modes);
        let mut noise = Noise::new(7);
        for _ in 0..FS as usize {
            let y = path.process(noise.next_bipolar());
            assert!(y.is_finite() && y.abs() < 100.0, "unstable: {y}");
        }
    }

    #[test]
    fn the_bank_is_bounded_and_dc_free_on_its_modes() {
        // A constant drive must not leave a constant offset on the modal part:
        // every mode is a bandpass, so only the residual survives DC.
        let mut path = StructuralPath::new(FS, &spread());
        let mut last = 0.0;
        for _ in 0..FS as usize {
            last = path.process(1.0);
        }
        assert!(
            (last - 1.0).abs() < 1e-3,
            "the modes left {last} of gain at DC, expected the residual's 1.0"
        );

        // And a bounded input stays bounded.
        let mut noise = Noise::new(11);
        let mut peak = 0.0f32;
        for _ in 0..(4 * FS as usize) {
            peak = peak.max(path.process(noise.next_bipolar()).abs());
        }
        assert!(
            peak.is_finite() && peak < 8.0,
            "bank ran away on unit noise: peak {peak}"
        );
    }

    #[test]
    fn reset_clears_the_ring() {
        let mut path = StructuralPath::new(FS, &spread());
        for _ in 0..64 {
            path.process(1.0);
        }
        path.reset();
        assert_eq!(path.process(0.0), 0.0);
    }
}
