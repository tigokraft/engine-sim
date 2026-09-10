//! The synthesis voice: physics state in, stereo samples out.
//!
//! # What the audio thread does and does not know
//!
//! The synth never sees the engine. It sees [`EngineSnapshot`] — a dozen scalars
//! describing the thermodynamic state at some recent instant — and it is
//! responsible for turning those into a continuous waveform no matter how
//! irregularly they arrive. That division matters:
//!
//! - **The physics thread owns amplitude.** How hard a cylinder hits is a
//!   thermodynamic fact (the blowdown pressure difference at EVO), and only the
//!   solver knows it.
//! - **The audio thread owns timing.** Firing events are reconstructed here, by
//!   integrating crank phase at the sample rate, rather than being sent as
//!   discrete events. A physics thread running at 60 Hz would otherwise collapse
//!   every pulse in a 16 ms window onto one instant, which at 3000 rpm is four
//!   cylinders arriving together — an engine that stutters in time with the
//!   frame rate. Integrating locally makes firing sample-accurate and makes the
//!   sound independent of the simulation's cadence.
//!
//! The consequence is that a starved snapshot queue is not a glitch. The synth
//! coasts on its last known state and keeps firing on schedule; the note simply
//! stops evolving until fresh physics arrives.
//!
//! # Signal flow
//!
//! ```text
//!                 blowdown pulses ──┐
//!   crank phase ──┤   (per cylinder)│   amplitude and theta jittered
//!         + CCV   │                 │   per cycle below 1500 rpm
//!                 backfire pops ────┤
//!                                   ▼
//!                     ┌──── exhaust bank (one per cylinder bank) ────┐
//!                     │  runner delay tau = L/sqrt(gamma R T)        │
//!                     │  feedback damped by tau_interval / tau_pulse │
//!                     │  Helmholtz muffler bandpass  ──> pan         │
//!                     └─────────────────────────────────────────────┘
//!   intake noise ── bandpass(m_dot, throttle) ───────────────> centre ──┐
//!   turbo whistle + surge flutter ──[if fitted]──────────────> centre ──┤
//!   valvetrain clicks + FMEP rumble ─────────────────────────> centre ──┤
//!                                                                       ▼
//!                            DC block ─> block resonance ─> soft clip ─> out
//!                                        (60-120 Hz, gain falls with rpm)
//! ```
//!
//! # Idle and the "robotic" failure mode
//!
//! Four of the stages above exist specifically because a naive version of this
//! synth sounds like a synthesiser at low speed, and each addresses a different
//! reason why:
//!
//! - A fixed firing table produces an impulse train periodic to the sample,
//!   which no machine is. [`CycleVariation`] jitters amplitude and phase.
//! - A lossless waveguide excited a handful of times a second flanges rather
//!   than resonates. [`crate::audio::filters::waveguide_damping`] closes it
//!   down when the pulses are far apart.
//! - Combustion at idle is a series of widely spaced events, and an engine
//!   built from combustion alone has audible *gaps*. [`MechanicalVoice`] fills
//!   them with the noise floor a real engine never stops making.
//! - The exhaust path is bandpass-like end to end and leaves out the block
//!   itself, which at idle is most of what a listener hears as size.
//!   [`crate::audio::filters::BlockResonator`] puts it back.
//!
//! All four are scheduled to recede with engine speed, because all four describe
//! things that stop mattering once combustion is loud and frequent.
//!
//! # Real-time discipline
//!
//! [`EngineSynth::render`] allocates nothing, locks nothing, and takes no branch
//! whose cost depends on the signal. Filter coefficients are redesigned once per
//! [`CONTROL_BLOCK`] samples rather than per sample, because the transcendental
//! functions in the design equations are the most expensive thing in the module
//! and the parameters driving them are already smoothed.
//!
//! The stochastic layers keep that contract too. Every random draw happens at a
//! firing event or a control boundary, never per sample, and the generator is
//! the same allocation-free xorshift the noise layers use — so the output stays
//! deterministic for a given seed and independent of the device's buffer size.

use std::f32::consts::TAU;

use crate::audio::filters::{
    firing_interval_seconds, soft_clip, waveguide_damping, Biquad, BiquadCoeffs, BlockResonator,
    DcBlocker, ExhaustRunner, ModalBank, Muffler, MufflerGeometry, Noise, OnePole, Smoothed,
};

/// Samples between control-rate updates.
///
/// 32 samples is 0.67 ms at 48 kHz — far below the ~10 ms it takes a listener to
/// resolve a timbral change, and 32x cheaper than redesigning coefficients every
/// sample.
pub const CONTROL_BLOCK: usize = 32;

/// Blowdown pressure difference that maps to full-scale pulse amplitude [Pa].
///
/// Measured against the block this crate ships: a cross-plane V8 opens its
/// exhaust valve on 2.2-4.2 bar above the manifold across its whole operating
/// range, the difference *falling* with engine speed as the manifold backs up
/// and the cylinder has less time to build pressure. 4.5 bar therefore sits just
/// above the loudest single pulse the engine produces, which is where a
/// normalising reference belongs: it uses the available headroom without ever
/// forcing the clipper to work.
///
/// Amplitude is strictly linear in the difference below this, per the excitation
/// model. Note that a quieter pulse does not mean a quieter engine — pulses
/// arrive 8x more often at 7000 rpm than at 850, so radiated power still climbs
/// steeply with speed.
pub const REFERENCE_BLOWDOWN: f32 = 4.5e5;

/// Instantaneous induction mass flow that maps to full intake noise [kg/s].
///
/// This is the *sum of instants* over the cylinders currently drawing, not a
/// cycle-averaged flow, and for the shipped V8 it runs from 0.025 kg/s at idle
/// to 0.19 kg/s at the limiter.
pub const REFERENCE_INTAKE_FLOW: f32 = 0.20;

/// Friction mean effective pressure that maps to a full mechanical noise floor [Pa].
///
/// The Chen-Flynn correlation the block ships with runs from about 0.7 bar of
/// FMEP at a warm idle to 2.3 bar near the limiter, so 2.5 bar is the top of the
/// range with a little headroom — the same convention as [`REFERENCE_BLOWDOWN`].
///
/// FMEP is the right driver for this layer because it is, definitionally, the
/// work the engine spends on itself per unit displacement: bearing shear, ring
/// drag, valvetrain, oil pump, windage. Every one of those is also a noise
/// source, and they scale together because they are the same losses.
pub const REFERENCE_FMEP: f32 = 2.5e5;

/// Speed below which cycle-to-cycle combustion variation is modelled [rev/min].
///
/// Above this the residual fraction is low, the charge motion is strong and
/// repeatable, and the COV of IMEP on a healthy engine falls under 1 % — which
/// is to say the variation is real but inaudible. Below it the same engine can
/// reach 8 %, and that *is* the lope.
pub const CCV_THRESHOLD_RPM: f32 = 1_500.0;

// ---------------------------------------------------------------------------
// Snapshot
// ---------------------------------------------------------------------------

/// Maximum number of cylinders supported by fixed-size snapshot arrays.
pub const MAX_CYLINDERS: usize = 16;

/// One frame of thermodynamic state, handed from the physics thread to the
/// audio callback.
///
/// Deliberately `Copy` and free of indirection: it crosses a lock-free queue
/// into a real-time thread, so it cannot own a heap allocation whose `Drop`
/// would run in the callback.
///
/// Fields are `f32` because that is the precision the synth works in. The
/// physics solves in `f64`; the narrowing happens once, here, at the boundary.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EngineSnapshot {
    /// Crankshaft speed [rev/min].
    pub rpm: f32,
    /// Per-cylinder `P_cylinder(theta_EVO) - P_exhaust_manifold(bank)` [Pa].
    pub blowdown_delta: [f32; MAX_CYLINDERS],
    /// Bulk exhaust runner gas temperature [K].
    pub exhaust_temperature: f32,
    /// Ratio of specific heats of the exhaust gas [-].
    pub exhaust_gamma: f32,
    /// Specific gas constant of the exhaust gas [J/(kg K)].
    pub exhaust_gas_constant: f32,
    /// Instantaneous induction mass flow, summed over cylinders [kg/s].
    pub intake_mass_flow: f32,
    /// Throttle position, `0..=1` [-].
    pub throttle: f32,
    /// Turbocharger shaft speed [rev/min].
    pub turbo_rpm: f32,
    /// Compressor surge severity, `0..=1` [-].
    pub turbo_surge: f32,
    /// Unburnt fuel mass being dumped into the exhaust per cycle [kg].
    pub unburnt_fuel_mass: f32,
    /// Chen-Flynn friction mean effective pressure [Pa].
    ///
    /// Drives the mechanical noise floor — the valvetrain and bearing racket
    /// that is inaudible under load and is most of what an idling engine
    /// actually radiates. See [`REFERENCE_FMEP`].
    pub friction_mep: f32,
    /// Whether ignition is currently cut (shift cut, launch control, overrun).
    pub spark_cut: bool,
    /// How far past 1.0 the Livengood-Wu knock integral went, `(I - 1).max(0)` [-].
    pub knock_intensity: f32,
    /// Cylinder bore diameter [m].
    pub bore: f32,
    /// Mean indicated torque over the cycle [N m].
    pub indicated_torque: f32,
    /// Rotating assembly inertia [kg m^2].
    pub inertia: f32,
}

impl Default for EngineSnapshot {
    /// A stopped engine at ambient: silent, but with sane gas properties so the
    /// filters tune to something physical before the first frame arrives.
    fn default() -> Self {
        Self {
            rpm: 0.0,
            blowdown_delta: [0.0; MAX_CYLINDERS],
            exhaust_temperature: 300.0,
            exhaust_gamma: 1.33,
            exhaust_gas_constant: 287.0,
            intake_mass_flow: 0.0,
            throttle: 0.0,
            turbo_rpm: 0.0,
            turbo_surge: 0.0,
            unburnt_fuel_mass: 0.0,
            friction_mep: 0.0,
            spark_cut: false,
            knock_intensity: 0.0,
            bore: 0.084,
            indicated_torque: 0.0,
            inertia: 0.25,
        }
    }
}

impl EngineSnapshot {
    /// Replaces any non-finite field with a safe value.
    ///
    /// The physics has a NaN watchdog, but it runs *after* a macro step, so a
    /// snapshot taken mid-corruption can still carry one through. A NaN reaching
    /// a filter's state is permanent — every subsequent sample is NaN, which the
    /// device renders as full-scale noise. Cheaper to check eleven scalars once
    /// per frame than to reset the whole synth afterwards.
    pub fn sanitized(mut self) -> Self {
        let fallback = Self::default();
        macro_rules! guard {
            ($field:ident, $lo:expr, $hi:expr) => {
                if !self.$field.is_finite() {
                    self.$field = fallback.$field;
                }
                self.$field = self.$field.clamp($lo, $hi);
            };
        }
        guard!(rpm, 0.0, 30_000.0);
        for p in self.blowdown_delta.iter_mut() {
            if !p.is_finite() {
                *p = 0.0;
            }
            *p = p.clamp(0.0, 5.0e6);
        }
        guard!(exhaust_temperature, 200.0, 2_500.0);
        guard!(exhaust_gamma, 1.05, 1.70);
        guard!(exhaust_gas_constant, 150.0, 600.0);
        guard!(intake_mass_flow, 0.0, 50.0);
        guard!(throttle, 0.0, 1.0);
        guard!(turbo_rpm, 0.0, 400_000.0);
        guard!(turbo_surge, 0.0, 1.0);
        guard!(unburnt_fuel_mass, 0.0, 1.0);
        guard!(friction_mep, 0.0, 2.0e6);
        guard!(knock_intensity, 0.0, 50.0);
        guard!(bore, 0.010, 0.500);
        guard!(indicated_torque, -500.0, 50_000.0);
        guard!(inertia, 0.010, 100.0);
        self
    }
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Where one cylinder sits in the cycle and which bank it exhausts into.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CylinderTap {
    /// Master-cycle phase of this cylinder's EVO, `0..1` over 720 degrees [-].
    pub evo_phase: f32,
    /// Index into the bank list.
    pub bank: usize,
}

/// Static description of the engine's acoustic plumbing.
/// What a fitted turbocharger sounds like.
///
/// This is the compressor's *voice*, kept apart from
/// [`crate::audio::TurboModel`], which is its shaft. The shaft says how fast
/// the wheel is turning; this says what a wheel turning that fast is heard as,
/// and it is what separates a small twin-scroll unit on a two-litre four from
/// the truck-sized single hung off a drag six.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TurboVoicing {
    /// Order of the turbo tone relative to shaft speed [-].
    ///
    /// True blade-pass frequency is shaft speed times blade count, which for a
    /// modern compressor wheel at full song is above 25 kHz — inaudible, and
    /// above Nyquist. The tone people actually hear is a lower circumferential
    /// order of the same wheel, so this is left as a tuning parameter rather
    /// than hard-wired to a blade count. A larger wheel turns slower for the
    /// same air, so a big single is voiced at a lower order than a small twin;
    /// order and speed together have to keep the tone inside the audible band
    /// and below the anti-alias ceiling `TurboVoice::tune` applies — a wheel
    /// voiced too high simply pins against that clamp and whistles at a fixed
    /// pitch no matter what the shaft is doing.
    pub order: f64,
    /// Shaft speed at which the turbo tone reaches full level [rev/min].
    ///
    /// Belongs a little under the shaft's own limit, so the whistle arrives
    /// before the compressor runs out of wheel rather than exactly with it.
    pub reference_rpm: f64,
    /// Level of the turbo layer in the mix [-].
    ///
    /// Measured against a unity-RMS source — see `TURBO_UNITY_RMS` — so it is
    /// directly comparable to [`SynthConfig::intake_level`] and
    /// [`SynthConfig::mechanical_level`]. A whistle is a near-pure tone
    /// concentrated in one critical band, so it carries much further than a
    /// broadband layer at the same RMS and belongs well below the exhaust.
    pub level: f64,
}

impl Default for TurboVoicing {
    /// A single mid-frame turbo: present, but not the thing you hear first.
    fn default() -> Self {
        Self {
            order: 2.5,
            reference_rpm: 130_000.0,
            level: 0.018,
        }
    }
}

///
/// Everything here is fixed for the life of the stream — it is geometry, not
/// state — so it is passed once at construction and never crosses the ring
/// buffer.
#[derive(Debug, Clone, PartialEq)]
pub struct SynthConfig {
    /// Output sample rate [Hz].
    pub sample_rate: f32,
    /// One entry per cylinder, in any order.
    pub cylinders: Vec<CylinderTap>,
    /// Number of exhaust banks; every [`CylinderTap::bank`] must be below this.
    pub bank_count: usize,
    /// Primary runner length from valve to collector [m].
    pub runner_length: f64,
    /// Magnitude of the collector reflection coefficient, `0..1` [-].
    pub runner_reflection: f64,
    /// Muffler cavity dimensions.
    pub muffler: MufflerGeometry,
    /// Blowdown duration in crank degrees, which sets the pulse decay [deg].
    pub blowdown_degrees: f64,
    /// The turbocharger, or `None` for a naturally aspirated engine.
    ///
    /// `None` is not a level of zero. With no turbo fitted the voice is never
    /// run at all, so an atmospheric engine cannot whistle or flutter however
    /// hard it is driven — which is the whole point, because half the engines
    /// in the catalogue never had a compressor bolted to them.
    ///
    /// Normally set from [`crate::audio::Induction`], which keeps this and the
    /// shaft that drives it together.
    pub turbo: Option<TurboVoicing>,
    /// Unburnt fuel per cycle above which backfires become possible [kg].
    pub backfire_fuel_threshold: f64,
    /// Runner temperature above which backfires become possible [K].
    pub backfire_temperature_threshold: f64,
    /// Dressed mass of the engine block [kg].
    ///
    /// Sets where the structural rumble sits, via
    /// [`crate::audio::filters::block_resonance_hz`]: a heavy iron block rings
    /// low and an alloy one rings high, because the stiffness barely changes and
    /// the mass does. Nothing else in the synth reads it.
    pub block_mass: f64,
    /// Sharpness of the block resonance [-].
    pub block_resonance_q: f64,
    /// Block rumble boost at idle, tapering to nothing with speed [dB].
    pub block_resonance_db: f64,
    /// Standard deviation of cycle-to-cycle variation at the threshold speed [-].
    ///
    /// See [`CCV_THRESHOLD_RPM`]. Applied to both the blowdown amplitude and the
    /// firing phase.
    pub combustion_variation_min: f64,
    /// Standard deviation of cycle-to-cycle variation at idle and below [-].
    pub combustion_variation_max: f64,
    /// Speed at which cycle-to-cycle variation reaches
    /// `combustion_variation_max` [rev/min].
    pub combustion_variation_idle_rpm: f64,
    /// How much of the Transit-Time Decision Rule's damping to apply, `0..=1` [-].
    ///
    /// One is the physical schedule; zero restores the undamped waveguide, which
    /// is useful for hearing what the rule is actually removing.
    pub waveguide_damping: f64,
    /// Level of each layer in the final mix [-].
    ///
    /// `backfire_level` is far above the others on purpose: a pop is an
    /// unmetered charge lighting off in open pipe, and it is genuinely several
    /// times the amplitude of an ordinary blowdown. The soft clipper is what
    /// keeps that from tearing.
    pub exhaust_level: f64,
    pub intake_level: f64,
    pub backfire_level: f64,
    /// Level of the valvetrain and bearing noise floor [-].
    pub mechanical_level: f64,
    /// Gain applied to the summed bus before the soft clipper [-].
    pub master_gain: f64,
}

impl Default for SynthConfig {
    /// A cross-plane V8: two banks of four, 90 degrees apart, firing 1-8-4-3-6-5-7-2.
    fn default() -> Self {
        Self::uniform_v8(48_000.0)
    }
}

impl SynthConfig {
    /// An evenly-fired engine with `cylinders` cylinders split across `banks`.
    ///
    /// Useful as a starting point and for tests; a real block should build the
    /// taps from its own firing order, which is what
    /// [`crate::audio::SnapshotSource`] does.
    pub fn uniform(sample_rate: f32, cylinders: usize, banks: usize) -> Self {
        let n = cylinders.max(1);
        let banks = banks.max(1);
        let taps = (0..n)
            .map(|i| CylinderTap {
                evo_phase: i as f32 / n as f32,
                // Alternate banks the way a vee engine alternates: consecutive
                // firings cross the vee.
                bank: i % banks,
            })
            .collect();
        Self {
            sample_rate,
            cylinders: taps,
            bank_count: banks,
            runner_length: 0.45,
            runner_reflection: 0.55,
            muffler: MufflerGeometry::default(),
            blowdown_degrees: 40.0,
            // Atmospheric by default: a turbo is something a preset fits, not
            // something every engine is born with.
            turbo: None,
            // A cut charge on the shipped V8 carries 24-36 mg of fuel to the
            // exhaust, so 12 mg puts a real spark cut at 2-3x the threshold —
            // enough to crackle hard, while a partial misfire stays below it.
            backfire_fuel_threshold: 12.0e-6,
            backfire_temperature_threshold: 900.0,
            // A 4-litre iron-decked V8 with pan and accessories: 80 Hz.
            block_mass: 180.0,
            // Low enough that the peak is felt as weight rather than heard as a
            // pitch. Past about 3 the block starts to sing a note of its own,
            // which reads as a resonating cabinet, not an engine.
            block_resonance_q: 1.1,
            // A V8's firing fundamental at idle is 57 Hz, which sits on this
            // peak's skirt — so the boost lands on the note itself, not just on
            // the noise floor, and a little of it goes a long way. Past about
            // 7 dB the idle stops being weighty and starts being louder than
            // the redline, which is the wrong way round.
            block_resonance_db: 5.5,
            // A healthy warm engine: COV of IMEP around 3 % where the lope
            // starts and 8 % at idle. A tired one would run higher, and this is
            // the knob that models it.
            combustion_variation_min: 0.03,
            combustion_variation_max: 0.08,
            combustion_variation_idle_rpm: 700.0,
            waveguide_damping: 1.0,
            exhaust_level: 1.0,
            intake_level: 0.35,
            backfire_level: 2.5,
            // Calibrated to sit about 8 dB under the exhaust at idle: audible
            // in the gaps between firings, which is its whole job, without
            // becoming the thing the engine sounds like. Level is measured
            // against a unity-RMS source, so this is directly comparable to
            // `intake_level` and [`TurboVoicing::level`].
            mechanical_level: 0.030,
            master_gain: 0.55,
        }
    }

    /// A cross-plane V8 with its characteristic uneven bank intervals.
    ///
    /// The firing order 1-8-4-3-6-5-7-2 on a 90-degree crank leaves each bank
    /// with gaps of 90-180-270-180 degrees instead of an even 180. That
    /// irregularity *is* the burble: routing the pulses through per-bank runners
    /// reproduces it for free, because each bank's delay line receives an
    /// unevenly spaced train.
    pub fn cross_plane_v8(sample_rate: f32) -> Self {
        // Firing offsets in crank degrees for cylinders 1..8, and their bank.
        const ORDER: [(f32, usize); 8] = [
            (0.0, 0),
            (90.0, 1),
            (180.0, 0),
            (270.0, 0),
            (360.0, 1),
            (450.0, 1),
            (540.0, 1),
            (630.0, 0),
        ];
        let taps = ORDER
            .iter()
            .map(|&(deg, bank)| CylinderTap {
                evo_phase: deg / 720.0,
                bank,
            })
            .collect();
        Self {
            cylinders: taps,
            bank_count: 2,
            ..Self::uniform(sample_rate, 8, 2)
        }
    }

    /// Alias for [`SynthConfig::cross_plane_v8`], kept for readability.
    pub fn uniform_v8(sample_rate: f32) -> Self {
        Self::cross_plane_v8(sample_rate)
    }

    /// Number of cylinders.
    pub fn cylinder_count(&self) -> usize {
        self.cylinders.len()
    }

    /// Firing frequency at a given speed, `f = (RPM / 120) * N_cyl` [Hz].
    pub fn firing_frequency(&self, rpm: f32) -> f32 {
        rpm / 120.0 * self.cylinder_count() as f32
    }
}

// ---------------------------------------------------------------------------
// Blowdown excitation
// ---------------------------------------------------------------------------

/// One in-flight exhaust blowdown pulse.
///
/// The envelope is the product of two exponentials — a fast rise as the valve
/// cracks and the flow chokes, and a slower fall as the cylinder empties —
/// evaluated recursively so a running pulse costs two multiplies and an add.
/// The waveform under it is a positive pressure step roughened by broadband
/// noise, which is what a choked orifice actually radiates: a step in mean
/// pressure plus the turbulence of the jet.
#[derive(Debug, Clone, Copy, Default)]
struct Blowdown {
    amplitude: f32,
    noise_depth: f32,
    attack_coeff: f32,
    decay_coeff: f32,
    attack_state: f32,
    decay_state: f32,
    /// Whole samples still to wait before the envelope starts running.
    ///
    /// Lets a pulse be scheduled *after* the crank angle that triggered it,
    /// which is how cycle-to-cycle firing jitter is applied. Moving the trigger
    /// angle itself would be the obvious alternative and is wrong: the angle is
    /// what the once-per-cycle crossing test is built on, and an angle that
    /// jumps forward after a cylinder has fired can be crossed a second time in
    /// the same cycle.
    pending: u32,
    active: bool,
}

impl Blowdown {
    /// Starts a pulse, `age` samples into its own envelope.
    ///
    /// The fractional `age` is what buys sample-accurate firing. Crank phase
    /// almost never crosses a cylinder's trigger exactly on a sample boundary,
    /// and quantising to the nearest sample adds up to half a sample of jitter
    /// to every pulse — at 48 kHz that is a 10 microsecond random walk on the
    /// firing instant, which is audible on a steady note as a faint rasp.
    /// Advancing the envelope analytically to its true starting point removes
    /// it.
    #[allow(clippy::too_many_arguments)]
    fn trigger(
        &mut self,
        sample_rate: f32,
        amplitude: f32,
        attack_seconds: f32,
        decay_seconds: f32,
        noise_depth: f32,
        age: f32,
        start_delay: f32,
    ) {
        let ta = attack_seconds.max(1.0 / sample_rate);
        let td = decay_seconds.max(2.0 / sample_rate);
        self.attack_coeff = 1.0 - (-1.0 / (ta * sample_rate)).exp();
        self.decay_coeff = (-1.0 / (td * sample_rate)).exp();

        // Peak of (1 - e^-t/ta) e^-t/td, so pulses of different lengths hit the
        // same level for the same pressure difference. Without this, a pulse at
        // 7000 rpm — where the decay is a fifth as long — would be quiet purely
        // because its envelope never has time to rise.
        let ratio = ta / (ta + td);
        let peak = (td / (ta + td)) * ratio.powf(ta / td);

        self.amplitude = amplitude / peak.max(1e-3);
        self.noise_depth = noise_depth.clamp(0.0, 1.0);

        // `age` counts forward from the pulse's own start; `start_delay` pushes
        // that start into the future. Their difference is where the envelope
        // stands right now, and when it is negative the pulse has not begun —
        // so the whole-sample part becomes a countdown and the remainder stays
        // as the sub-sample phase the envelope resumes from.
        let now = age.clamp(0.0, 1.0) - start_delay.max(0.0);
        let wait = (-now).max(0.0).ceil();
        let age = (now + wait).clamp(0.0, 1.0);
        self.pending = wait as u32;

        self.attack_state = 1.0 - (1.0 - self.attack_coeff).powf(age);
        self.decay_state = self.decay_coeff.powf(age);
        self.active = true;
    }

    /// Next sample of this pulse, or zero once it has died out.
    #[inline(always)]
    fn process(&mut self, noise: f32) -> f32 {
        if !self.active {
            return 0.0;
        }
        if self.pending > 0 {
            self.pending -= 1;
            return 0.0;
        }
        self.attack_state += (1.0 - self.attack_state) * self.attack_coeff;
        self.decay_state *= self.decay_coeff;
        // Below -100 dB the pulse is inaudible and only costs cycles; retiring
        // it also keeps the slot available for the next firing.
        if self.decay_state < 1e-5 {
            self.active = false;
            return 0.0;
        }
        let envelope = self.attack_state * self.decay_state;
        envelope * self.amplitude * (1.0 - self.noise_depth + self.noise_depth * noise)
    }
}

/// Fixed pool of overlapping pulses.
///
/// Pulses overlap whenever the decay outlasts the firing interval, which on a
/// four-cylinder bank happens above roughly 5000 rpm. Four slots covers that
/// with margin; allocating per pulse is not an option in a callback, and
/// stealing the oldest slot when the pool is exhausted degrades gracefully
/// (the pulse being stolen is by then the quietest one present).
#[derive(Debug, Clone, Copy, Default)]
struct PulsePool {
    slots: [Blowdown; 4],
    next: usize,
}

impl PulsePool {
    #[allow(clippy::too_many_arguments)]
    fn trigger(
        &mut self,
        sample_rate: f32,
        amplitude: f32,
        attack: f32,
        decay: f32,
        noise_depth: f32,
        age: f32,
        start_delay: f32,
    ) {
        // Prefer an idle slot; fall back to round-robin stealing.
        let slot = self
            .slots
            .iter()
            .position(|s| !s.active)
            .unwrap_or(self.next);
        self.next = (self.next + 1) % self.slots.len();
        self.slots[slot].trigger(
            sample_rate,
            amplitude,
            attack,
            decay,
            noise_depth,
            age,
            start_delay,
        );
    }

    #[inline(always)]
    fn process(&mut self, noise: &mut Noise) -> f32 {
        let mut sum = 0.0;
        for slot in self.slots.iter_mut() {
            if slot.active {
                sum += slot.process(noise.next_bipolar());
            }
        }
        sum
    }

    fn reset(&mut self) {
        self.slots = [Blowdown::default(); 4];
        self.next = 0;
    }
}

// ---------------------------------------------------------------------------
// Cycle-to-cycle combustion variation
// ---------------------------------------------------------------------------

/// One cylinder's draw for its next firing.
///
/// # The physics
///
/// No two cycles of a spark-ignition engine are the same. The flame kernel
/// starts in whatever turbulence happens to be at the plug gap at the instant of
/// the spark, and at idle that is a lottery: the charge is heavily diluted with
/// residual gas left over from the previous cycle, the intake velocity is low so
/// there is little organised tumble to speed the burn, and the early flame is
/// small enough that a single eddy moves it. The result is that the same nominal
/// charge produces a spread of peak pressures and a spread of burn *timings* —
/// the coefficient of variation of IMEP runs 3-8 % at idle on a healthy engine,
/// and far more on a tired or a big-cam one.
///
/// Both halves of that reach the microphone:
///
/// - A weaker cycle reaches EVO at a lower pressure, so `P_delta` is smaller and
///   the blowdown pulse is quieter.
/// - A slower burn peaks later and is still at higher pressure when the exhaust
///   valve cracks, but the *acoustic* event has effectively shifted in crank
///   angle — the pulse leaves late. That is `theta_offset`.
///
/// # Why this is what "robotic" means
///
/// An engine synthesised from a fixed firing table produces an impulse train
/// that is periodic to the sample. Periodic to the sample is not a sound any
/// physical object makes, and the ear is extremely good at hearing it: the
/// spectrum is a perfect harmonic comb with no skirt on any partial, which is
/// the signature of a synthesiser rather than a machine. Jittering the timing
/// broadens each partial; jittering the amplitude puts a slow wander on the
/// envelope. Together they are the lope.
///
/// The draw is bounded, not truly Gaussian — see [`Noise::next_gaussian`].
#[derive(Debug, Clone, Copy)]
struct CycleVariation {
    /// Multiplier on this cylinder's next blowdown amplitude [-].
    amplitude_scale: f32,
    /// Offset on this firing's retard, in cycle fraction [-].
    ///
    /// Zero-mean, and added to [`CCV_MEAN_RETARD_FRACTION`] to give the delay
    /// the pulse is actually scheduled with.
    phase_offset: f32,
}

impl Default for CycleVariation {
    /// The nominal cycle: full amplitude, dead on the firing table.
    fn default() -> Self {
        Self {
            amplitude_scale: 1.0,
            phase_offset: 0.0,
        }
    }
}

/// Widest phase offset allowed, as a fraction of the firing interval [-].
///
/// A pulse that wandered past half the gap to its neighbour would reorder the
/// firing sequence, which is a mechanical failure rather than a variation. At
/// the default depths the bounded Gaussian tops out at `3.46 * 0.08 = 0.28` of
/// an interval, so this never binds; it is here so that a caller who winds
/// `combustion_variation_max` up to model a sick engine gets a rough idle
/// rather than a scrambled one.
const CCV_MAX_PHASE_FRACTION: f32 = 0.3;

/// Mean retard of the blowdown pulse behind nominal EVO, as a fraction of the
/// firing interval [-].
///
/// The jitter has to be applied as a *delay* rather than as a move of the
/// trigger angle (see [`Blowdown::pending`]), and a delay cannot be negative —
/// so the nominal event is placed this far back and the draw varies either side
/// of it. Equal to [`CCV_MAX_PHASE_FRACTION`] so the sum is never negative.
///
/// This is not a fudge to make the arithmetic work. Nominal EVO is a valve
/// event; the blowdown that follows it is a *combustion* event, and the flame
/// takes a real and variable number of crank degrees to develop before the
/// cylinder is at the pressure that drives the pulse. A mean lag with variation
/// around it is what that looks like. The lag itself is common to every
/// cylinder, so acoustically it is latency and nothing else — only the
/// variation is audible.
const CCV_MEAN_RETARD_FRACTION: f32 = CCV_MAX_PHASE_FRACTION;

/// Widest amplitude excursion allowed, as a fraction of nominal [-].
const CCV_MAX_AMPLITUDE_EXCURSION: f32 = 0.75;

// ---------------------------------------------------------------------------
// Exhaust bank
// ---------------------------------------------------------------------------

/// One bank's exhaust path: excitation, runner, muffler, pan.
#[derive(Debug, Clone)]
struct ExhaustBank {
    pulses: PulsePool,
    runner: ExhaustRunner,
    muffler: Muffler,
    pan_left: f32,
    pan_right: f32,
    /// How many cylinders exhaust into this bank.
    ///
    /// Held here because it is the denominator of the Transit-Time Decision
    /// Rule's `tau_interval`: a bank's pulse train is what its runner sees, and
    /// on a cross-plane V8 the two banks do not even receive evenly spaced ones.
    cylinder_count: usize,
}

impl ExhaustBank {
    fn new(config: &SynthConfig, index: usize, snapshot: &EngineSnapshot) -> Self {
        let fs = config.sample_rate;
        // Spread the banks across the image. A real vee engine's banks reach the
        // listener along different paths, and separating them is what lets the
        // uneven bank intervals be heard as two interleaved rhythms rather than
        // one blurred train.
        let spread = if config.bank_count > 1 {
            let t = index as f32 / (config.bank_count - 1) as f32;
            (t - 0.5) * 0.7
        } else {
            0.0
        };
        // Equal-power pan, so the summed level is flat across the image.
        let angle = (spread * 0.5 + 0.5) * std::f32::consts::FRAC_PI_2;
        Self {
            pulses: PulsePool::default(),
            runner: ExhaustRunner::new(
                fs,
                config.runner_length as f32,
                config.runner_reflection as f32,
                snapshot.exhaust_gamma,
                snapshot.exhaust_gas_constant,
                snapshot.exhaust_temperature,
            ),
            muffler: Muffler::new(
                fs,
                config.muffler,
                snapshot.exhaust_gamma,
                snapshot.exhaust_gas_constant,
                snapshot.exhaust_temperature,
            ),
            pan_left: angle.cos(),
            pan_right: angle.sin(),
            cylinder_count: config
                .cylinders
                .iter()
                .filter(|tap| tap.bank == index)
                .count(),
        }
    }

    fn tune(&mut self, gamma: f32, gas_constant: f32, temperature: f32) {
        self.runner.tune(gamma, gas_constant, temperature);
        self.muffler.tune(gamma, gas_constant, temperature);
    }

    /// Applies the Transit-Time Decision Rule at the current speed.
    ///
    /// `strength` scales the whole schedule; see
    /// [`SynthConfig::waveguide_damping`].
    fn tune_damping(&mut self, rpm: f32, strength: f32) {
        let tau_interval = firing_interval_seconds(rpm, self.cylinder_count);
        let tau_pulse = self.runner.round_trip_seconds();
        self.runner
            .set_damping(waveguide_damping(tau_interval, tau_pulse) * strength.clamp(0.0, 1.0));
    }

    #[inline(always)]
    fn process(&mut self, noise: &mut Noise) -> f32 {
        let excitation = self.pulses.process(noise);
        let radiated = self.runner.process(excitation);
        self.muffler.process(radiated)
    }

    fn reset(&mut self) {
        self.pulses.reset();
        self.runner.reset();
        self.muffler.reset();
    }
}

// ---------------------------------------------------------------------------
// Intake
// ---------------------------------------------------------------------------

/// Induction noise: broadband turbulence gated by flow and throttle.
///
/// Air rushing past a throttle plate and down a runner is a jet, and a jet's
/// radiated sound is broadband with a spectral peak that climbs with velocity.
/// Both the level and that peak are driven from the same mass flow, which is
/// why an engine's intake gets not just louder but *brighter* as it breathes
/// harder — and why a closed throttle at high rpm goes quiet rather than merely
/// soft.
#[derive(Debug, Clone)]
struct IntakeVoice {
    band: Biquad,
    body: Biquad,
    gain: Smoothed,
    sample_rate: f32,
}

impl IntakeVoice {
    fn new(sample_rate: f32) -> Self {
        Self {
            band: Biquad::new(BiquadCoeffs::bandpass(sample_rate, 400.0, 0.7)),
            body: Biquad::new(BiquadCoeffs::lowpass(sample_rate, 2_000.0, 0.7)),
            gain: Smoothed::new(0.0, sample_rate, 0.020),
            sample_rate,
        }
    }

    /// Retunes from mass flow [kg/s] and throttle position.
    fn tune(&mut self, mass_flow: f32, throttle: f32) {
        let flow = (mass_flow / REFERENCE_INTAKE_FLOW).clamp(0.0, 1.6);

        // Peak velocity noise climbs roughly linearly with flow over the range
        // an engine actually uses.
        let centre = 260.0 + 1_500.0 * flow;
        self.band
            .set_coeffs(BiquadCoeffs::bandpass(self.sample_rate, centre, 0.65));
        self.body.set_coeffs(BiquadCoeffs::lowpass(
            self.sample_rate,
            (900.0 + 3_600.0 * flow).min(0.44 * self.sample_rate),
            0.7,
        ));

        // Radiated power grows faster than flow does; the 3/2 exponent keeps
        // idle from being buried while still opening up under load. The throttle
        // term is the plate itself: a shut throttle muffles the runner mouth
        // even while the engine is still pumping.
        let level = flow.powf(1.5) * (0.25 + 0.75 * throttle);
        self.gain.set_target(level.min(1.5));
    }

    #[inline(always)]
    fn process(&mut self, noise: &mut Noise) -> f32 {
        let g = self.gain.next_value();
        if g < 1e-6 {
            // Still run the filters so their state stays in step; only the
            // multiply is skipped. Bypassing them entirely would leave stale
            // state to thump when flow returns.
            let _ = self.body.process(self.band.process(0.0));
            return 0.0;
        }
        let excited = self.band.process(noise.next_bipolar());
        self.body.process(excited) * g
    }

    fn reset(&mut self) {
        self.band.reset();
        self.body.reset();
    }
}

// ---------------------------------------------------------------------------
// Mechanical noise floor
// ---------------------------------------------------------------------------

/// Valve seatings per cylinder per four-stroke cycle [-].
///
/// One intake valve closing and one exhaust valve closing. Those are the two
/// events that make the noise — a valve landing on its seat under full spring
/// load, transmitted up the stem into the head — while the openings are ramped
/// by the cam lobe and are comparatively silent.
const VALVE_EVENTS_PER_CYLINDER: f32 = 2.0;

/// Makeup gain for the twice-lowpassed rumble path [-].
///
/// Two one-poles at [`MECHANICAL_RUMBLE_HZ`] throw away most of the power in a
/// white source; this puts the survivor back at unity RMS so
/// [`SynthConfig::mechanical_level`] means the same thing as the other level
/// controls. Measured, not derived — see `mechanical_layers_sit_at_unity`.
const MECHANICAL_RUMBLE_MAKEUP: f32 = 23.7;

/// Corner of each rumble lowpass stage [Hz].
///
/// Deliberately well above the block's own modes. Mechanical noise is not
/// sub-bass: piston slap, gear mesh and bearing shear all radiate into the low
/// midrange, and a rumble filtered down to the bottom two octaves reads as an
/// air-conditioning duct rather than as an engine. It is also the wrong place
/// to put energy in a mix that already has a resonant peak at 80 Hz and a DC
/// blocker underneath it — headroom spent there is inaudible.
const MECHANICAL_RUMBLE_HZ: f32 = 380.0;

// ---------------------------------------------------------------------------
// Impulsive mechanical source primitive
// ---------------------------------------------------------------------------

/// How an impulsive mechanical source sets its recurrence rate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SourceRate {
    /// One event per cylinder per four-stroke engine cycle (crank order = cylinders * 0.5).
    PerCylinder,
    /// Explicit crankshaft order (events per crank revolution).
    Order(f32),
}

/// Control law governing how a mechanical source's gain scales with engine state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LevelLaw {
    /// Cam lash and spring force: constant base + moderate friction scaling.
    /// Does not vanish on spark cut.
    CamLash { base_ratio: f32 },
    /// Scales directly with friction mean effective pressure.
    FrictionMep,
    /// Scales with crankshaft speed: `rpm / reference_rpm`.
    Speed { reference_rpm: f32 },
    /// Constant level whenever the engine is turning.
    Constant,
}

impl LevelLaw {
    /// Evaluates the gain multiplier given friction MEP, engine speed, and cycle rate.
    pub fn compute(&self, friction_mep: f32, rpm: f32, cycle_hz: f32) -> f32 {
        if cycle_hz < 1e-3 || rpm < 1.0 {
            return 0.0;
        }
        match *self {
            LevelLaw::Constant => 1.0,
            LevelLaw::CamLash { base_ratio } => {
                let drag = (friction_mep / REFERENCE_FMEP).clamp(0.0, 2.0);
                base_ratio + (1.0 - base_ratio) * drag
            }
            LevelLaw::FrictionMep => (friction_mep / REFERENCE_FMEP).clamp(0.0, 3.0),
            LevelLaw::Speed { reference_rpm } => (rpm / reference_rpm.max(100.0)).clamp(0.0, 3.0),
        }
    }
}

/// One impulsive mechanical source: rate, jitter, envelope, body resonance, level law.
#[derive(Debug, Clone)]
pub struct ImpulsiveSource {
    pub rate: SourceRate,
    pub jitter: f32,
    pub decay: f32,
    pub envelope: f32,
    pub phase: f32,
    pub body: ModalBank,
    pub level_law: LevelLaw,
    pub base_level: f32,
    pub gain: Smoothed,
    pub event_hz: Smoothed,
    sample_rate: f32,
}

impl ImpulsiveSource {
    pub fn new(
        sample_rate: f32,
        rate: SourceRate,
        jitter: f32,
        decay_time: f32,
        body: ModalBank,
        level_law: LevelLaw,
        base_level: f32,
    ) -> Self {
        Self {
            rate,
            jitter: jitter.clamp(0.0, 1.0),
            decay: (-1.0 / (decay_time.max(1e-5) * sample_rate)).exp(),
            envelope: 0.0,
            phase: 0.0,
            body,
            level_law,
            base_level,
            gain: Smoothed::new(0.0, sample_rate, 0.040),
            event_hz: Smoothed::new(0.0, sample_rate, 0.030),
            sample_rate,
        }
    }

    /// Effective crank order (events per crank revolution).
    pub fn effective_order(&self, cylinders: usize) -> f32 {
        match self.rate {
            SourceRate::PerCylinder => cylinders.max(1) as f32 * 0.5,
            SourceRate::Order(o) => o,
        }
    }

    /// Effective recurrence frequency at a given cycle rate [Hz].
    pub fn effective_hz(&self, cycle_hz: f32, cylinders: usize) -> f32 {
        if cycle_hz < 1e-3 {
            0.0
        } else {
            let crank_hz = cycle_hz * 2.0;
            self.effective_order(cylinders) * crank_hz
        }
    }

    /// Retunes recurrence rate and target gain.
    pub fn tune(&mut self, friction_mep: f32, rpm: f32, cycle_hz: f32, cylinders: usize) {
        let hz = self.effective_hz(cycle_hz, cylinders);
        self.event_hz.set_target(hz);

        let law_gain = self.level_law.compute(friction_mep, rpm, cycle_hz);
        self.gain.set_target(self.base_level * law_gain);
    }

    /// Renders one audio sample of this impulsive source.
    #[inline(always)]
    pub fn process(&mut self, noise: &mut Noise) -> f32 {
        let gain = self.gain.next_value();
        let event_hz = self.event_hz.next_value();

        let increment = event_hz / self.sample_rate;
        if (1e-9..1.0).contains(&increment) {
            self.phase += increment;
            if self.phase >= 1.0 {
                self.phase -= 1.0;
                let jitter_factor = (1.0 - self.jitter) + self.jitter * noise.next_unit();
                self.envelope = jitter_factor;
            }
        }
        self.envelope *= self.decay;
        if self.envelope < 1e-5 {
            self.envelope = 0.0;
        }

        let raw = noise.next_bipolar() * self.envelope;
        self.body.process(raw) * gain
    }

    /// Resets filter state, phase accumulator, and envelope.
    pub fn reset(&mut self) {
        self.body.reset();
        self.envelope = 0.0;
        self.phase = 0.0;
    }
}

/// The racket an engine makes that has nothing to do with combustion.
///
/// # Why this layer exists
///
/// Strip the exhaust out of a recording of an idling engine and what is left is
/// not silence — it is most of the sound. A warm engine at 800 rpm is spending
/// roughly a third of its indicated work on itself, and all of that comes back
/// out as noise: valves seating, pistons slapping, gears meshing, the oil pump,
/// the accessory belt. Combustion at idle is a series of soft, widely spaced
/// events; the mechanical floor is continuous. An engine synthesised from
/// combustion alone therefore has *gaps* between its firings, and those gaps are
/// the single loudest tell that a sound is procedural.
///
/// # The two components
///
/// - **Lifter clicks.** Impulsive, bright, and locked to the cam — the "sewing
///   machine" tick. Their rate is [`VALVE_EVENTS_PER_CYLINDER`] times the
///   cylinder count times the cycle rate, so unlike everything else in this
///   module they are *not* driven by the firing order: they keep going on a cut
///   cylinder, because a cam does not care whether the plug fired.
/// - **Rumble.** Broadband, twice-lowpassed, and scaled by FMEP. This is
///   bearing shear, windage and ring drag — a continuum rather than events.
///
/// Both scale with [`EngineSnapshot::friction_mep`], but with different
/// exponents, because they are different mechanisms. Valve spring force is set
/// by the cam profile and is nearly independent of load, so the clicks stay
/// close to constant; hydrodynamic and pumping losses grow steeply with speed,
/// so the rumble follows FMEP directly.
#[derive(Debug, Clone)]
pub struct MechanicalVoice {
    click: ImpulsiveSource,
    rumble_a: OnePole,
    rumble_b: OnePole,
    pub click_gain: Smoothed,
    pub rumble_gain: Smoothed,
    pub event_hz: Smoothed,
    pub click_phase: f32,
}

pub type MechanicalRig = MechanicalVoice;

impl MechanicalVoice {
    fn new(sample_rate: f32) -> Self {
        let click = ImpulsiveSource::new(
            sample_rate,
            SourceRate::Order(0.0),
            0.45,
            0.0007,
            ModalBank::single(sample_rate, 3_200.0, 1.1),
            LevelLaw::CamLash { base_ratio: 0.45 },
            1.0,
        );
        Self {
            click,
            rumble_a: OnePole::new(sample_rate, MECHANICAL_RUMBLE_HZ),
            rumble_b: OnePole::new(sample_rate, MECHANICAL_RUMBLE_HZ),
            click_gain: Smoothed::new(0.0, sample_rate, 0.040),
            rumble_gain: Smoothed::new(0.0, sample_rate, 0.040),
            event_hz: Smoothed::new(0.0, sample_rate, 0.030),
            click_phase: 0.0,
        }
    }

    /// Retunes from FMEP [Pa], cycle rate [Hz] and cylinder count.
    fn tune(&mut self, friction_mep: f32, cycle_hz: f32, cylinders: usize) {
        let drag = (friction_mep / REFERENCE_FMEP).clamp(0.0, 1.5);
        let hz = cycle_hz * cylinders.max(1) as f32 * VALVE_EVENTS_PER_CYLINDER;
        self.click.rate =
            SourceRate::Order(cylinders.max(1) as f32 * (VALVE_EVENTS_PER_CYLINDER * 0.5));
        self.click
            .tune(friction_mep, cycle_hz * 120.0, cycle_hz, cylinders);
        self.event_hz.set_target(hz);

        self.click_gain.set_target(0.45 + 0.55 * drag);
        self.rumble_gain.set_target(drag);

        if cycle_hz < 1e-3 {
            self.click_gain.set_target(0.0);
            self.rumble_gain.set_target(0.0);
        }
    }

    #[inline(always)]
    fn process(&mut self, noise: &mut Noise) -> f32 {
        let click_gain = self.click_gain.next_value();
        let rumble_gain = self.rumble_gain.next_value();
        let event_hz = self.event_hz.next_value();

        self.click.event_hz.snap(event_hz);
        self.click.gain.snap(click_gain);

        let click = self.click.process(noise);
        self.click_phase = self.click.phase;

        let rumble = self
            .rumble_b
            .process(self.rumble_a.process(noise.next_bipolar()))
            * MECHANICAL_RUMBLE_MAKEUP
            * rumble_gain;

        click * 0.35 + rumble * 0.65
    }

    fn reset(&mut self) {
        self.click.reset();
        self.click_phase = 0.0;
        self.rumble_a.reset();
        self.rumble_b.reset();
    }
}

// ---------------------------------------------------------------------------
// Turbocharger
// ---------------------------------------------------------------------------

/// Scaling that puts [`TurboVoice`] at roughly unity RMS at its reference
/// speed.
///
/// The voice is a sum of sines with a fixed shape, so its natural amplitude is
/// an accident of how that shape was written down rather than anything
/// meaningful. Dividing it out is what makes [`TurboVoicing::level`] mean the
/// same thing as every other level in [`SynthConfig`] — and without it a level
/// chosen by ear on one engine is wrong on the next.
const TURBO_UNITY_RMS: f32 = 2.475;

/// Turbo whistle plus compressor surge flutter.
///
/// The whistle is a near-pure tone locked to shaft speed, with a second harmonic
/// and a trace of shaft-order imbalance under it. Level rises with the square of
/// shaft speed, following radiated acoustic power rather than amplitude, which
/// is why a turbo is inaudible at low boost and dominant near its limit.
///
/// Surge is the other half. When the compressor is pushed against a closed
/// throttle it stalls, the flow reverses, pressure collapses, and the cycle
/// repeats at 10-30 Hz. That is heard as chuffing — bursts of broadband noise at
/// the surge frequency, with the whistle warbling as the shaft loads and
/// unloads.
#[derive(Debug, Clone)]
struct TurboVoice {
    phase: f32,
    flutter_phase: f32,
    chuff: Biquad,
    gain: Smoothed,
    surge_gain: Smoothed,
    tone_hz: Smoothed,
    flutter_hz: Smoothed,
    sample_rate: f32,
}

impl TurboVoice {
    fn new(sample_rate: f32) -> Self {
        Self {
            phase: 0.0,
            flutter_phase: 0.0,
            chuff: Biquad::new(BiquadCoeffs::bandpass(sample_rate, 1_500.0, 0.9)),
            gain: Smoothed::new(0.0, sample_rate, 0.030),
            surge_gain: Smoothed::new(0.0, sample_rate, 0.015),
            tone_hz: Smoothed::new(0.0, sample_rate, 0.025),
            flutter_hz: Smoothed::new(18.0, sample_rate, 0.050),
            sample_rate,
        }
    }

    fn tune(&mut self, voicing: &TurboVoicing, turbo_rpm: f32, surge: f32) {
        let shaft_hz = turbo_rpm / 60.0;
        let tone = shaft_hz * voicing.order as f32;
        // Half of Nyquist, so the second harmonic stays below it too. Above this
        // the tone is inaudible anyway, and letting it wrap would fold a loud
        // partial back down into the middle of the exhaust note.
        self.tone_hz.set_target(tone.min(0.22 * self.sample_rate));

        let load = (turbo_rpm / voicing.reference_rpm as f32).clamp(0.0, 1.4);
        self.gain.set_target(load * load);
        self.surge_gain.set_target(surge.clamp(0.0, 1.0));

        // Surge speeds up as the compressor is driven further past the line.
        self.flutter_hz.set_target(11.0 + 17.0 * surge);
        // The chuff sits where a compressor's reversed flow makes its noise.
        self.chuff.set_coeffs(BiquadCoeffs::bandpass(
            self.sample_rate,
            900.0 + 1_400.0 * load,
            0.8,
        ));
    }

    #[inline(always)]
    fn process(&mut self, noise: &mut Noise) -> f32 {
        let gain = self.gain.next_value();
        let surge = self.surge_gain.next_value();
        let tone_hz = self.tone_hz.next_value();
        let flutter_hz = self.flutter_hz.next_value();

        self.flutter_phase += flutter_hz / self.sample_rate;
        if self.flutter_phase >= 1.0 {
            self.flutter_phase -= 1.0;
        }
        // Cubed raised cosine: a sharp chuff with a long gap, not a sine wobble.
        let cycle = 0.5 - 0.5 * (TAU * self.flutter_phase).cos();
        let burst = cycle * cycle * cycle;

        // Surge loads and unloads the shaft, so the tone warbles with it.
        let warble = 1.0 + 0.18 * surge * (2.0 * cycle - 1.0);
        self.phase += (tone_hz * warble) / self.sample_rate;
        if self.phase >= 1.0 {
            self.phase -= self.phase.floor();
        }
        let w = TAU * self.phase;
        let whistle = w.sin() + 0.28 * (2.0 * w).sin() + 0.06 * (0.25 * w).sin();

        // Surge chokes the steady whistle as it feeds the chuff.
        let whistle_level = gain * (1.0 - 0.55 * surge);
        let chuff = self.chuff.process(noise.next_bipolar()) * burst * surge * (0.35 + gain);

        (whistle * whistle_level * 0.55 + chuff * 1.4) * TURBO_UNITY_RMS
    }

    fn reset(&mut self) {
        self.phase = 0.0;
        self.flutter_phase = 0.0;
        self.chuff.reset();
    }
}

// ---------------------------------------------------------------------------
// Backfire
// ---------------------------------------------------------------------------

/// Decides when unburnt fuel in a hot runner lights off.
///
/// The physical story is specific: cut the spark while the throttle is still
/// open — an upshift cut, launch-control limiter, or a lifted overrun — and the
/// injectors keep delivering fuel that the cylinder pumps out unburnt. If the
/// runner is hot enough and there is enough oxygen behind it, it ignites in the
/// exhaust instead. Both conditions have to hold: a cold pipe swallows the fuel,
/// and a warm pipe with nothing in it does nothing.
///
/// The result is stochastic, because auto-ignition in a turbulent pipe is. This
/// generates a Poisson-like process whose rate rises with how far past both
/// thresholds the engine is, with a refractory period so a long cut becomes a
/// crackle rather than a buzz.
#[derive(Debug, Clone, Copy)]
struct BackfireVoice {
    severity: f32,
    cooldown: usize,
    sample_rate: f32,
}

impl BackfireVoice {
    fn new(sample_rate: f32) -> Self {
        Self {
            severity: 0.0,
            cooldown: 0,
            sample_rate,
        }
    }

    /// Recomputes how primed the exhaust is, `0` when either condition fails.
    fn tune(&mut self, config: &SynthConfig, snapshot: &EngineSnapshot) {
        if !snapshot.spark_cut {
            self.severity = 0.0;
            return;
        }
        let fuel = snapshot.unburnt_fuel_mass / config.backfire_fuel_threshold.max(1e-12) as f32;
        let heat =
            (snapshot.exhaust_temperature - config.backfire_temperature_threshold as f32) / 250.0;
        if fuel <= 1.0 || heat <= 0.0 {
            self.severity = 0.0;
            return;
        }
        // Saturating at twice the threshold: a genuine ignition cut dumps the
        // whole metered charge, which is well past that, so a real cut sits at
        // full severity rather than at a third of it. The ramp below still
        // separates a cut from a partial misfire.
        self.severity = (fuel - 1.0).min(1.0) * heat.min(1.0);
    }

    /// Polls once per control block; returns a pop's amplitude and decay [s].
    fn poll(&mut self, noise: &mut Noise, block: usize) -> Option<(f32, f32)> {
        self.cooldown = self.cooldown.saturating_sub(block);
        if self.severity <= 0.0 || self.cooldown > 0 {
            return None;
        }
        // Up to ~28 events per second when fully primed.
        let rate = 28.0 * self.severity;
        let probability = rate * block as f32 / self.sample_rate;
        if noise.next_unit() >= probability {
            return None;
        }
        // 35 ms refractory: fast enough to crackle, slow enough to stay discrete.
        self.cooldown = (0.035 * self.sample_rate) as usize;
        let scale = 0.55 + 0.75 * noise.next_unit();
        Some((
            self.severity * scale,
            // Pops in a pipe ring far longer than a blowdown crack does.
            0.006 + 0.014 * noise.next_unit(),
        ))
    }
}

// ---------------------------------------------------------------------------
// Knock resonance voice
// ---------------------------------------------------------------------------

/// First circumferential cavity acoustic mode constant [-].
pub const KNOCK_RHO_10: f32 = 1.8412;
/// Second circumferential cavity acoustic mode constant [-].
pub const KNOCK_RHO_20: f32 = 3.0542;
/// First radial cavity acoustic mode constant [-].
pub const KNOCK_RHO_01: f32 = 3.8317;

/// Sharpness of the cylinder cavity knock resonances [-].
pub const KNOCK_Q: f32 = 20.0;

/// Resonances of burned gas ringing inside the cylinder bore cavity.
///
/// Knock is auto-ignition of the unburnt end-gas ahead of the flame front:
/// an abrupt volumetric heat release that excites the acoustic modes of the
/// combustion chamber. The dominant modes are the first circumferential
/// (rho_10 = 1.8412), second circumferential (rho_20 = 3.0542), and first
/// radial (rho_01 = 3.8317).
///
/// The voice is excited on cylinder firing events by a noise burst whose amplitude
/// scales with how far past 1.0 the Livengood-Wu knock integral went, decaying
/// in 2-5 ms. It is routed into the structural path through the block resonator,
/// because knock is heard ringing through the engine block, not out the exhaust.
#[derive(Debug, Clone)]
pub struct KnockVoice {
    sample_rate: f32,
    mode_10: Biquad,
    mode_20: Biquad,
    mode_01: Biquad,
    mode_frequencies: [f32; 3],
    intensity: f32,
    envelope: f32,
    decay_coeff: f32,
}

impl KnockVoice {
    pub fn new(sample_rate: f32) -> Self {
        let f10 = 5500.0;
        let f20 = f10 * (KNOCK_RHO_20 / KNOCK_RHO_10);
        let f01 = f10 * (KNOCK_RHO_01 / KNOCK_RHO_10);

        // 3 ms exponential decay (within the 2-5 ms physical window).
        let decay_time = 0.003;
        let decay_coeff = (-1.0 / (decay_time * sample_rate)).exp();

        Self {
            sample_rate,
            mode_10: Biquad::new(BiquadCoeffs::bandpass(sample_rate, f10, KNOCK_Q)),
            mode_20: Biquad::new(BiquadCoeffs::bandpass(sample_rate, f20, KNOCK_Q)),
            mode_01: Biquad::new(BiquadCoeffs::bandpass(sample_rate, f01, KNOCK_Q)),
            mode_frequencies: [f10, f20, f01],
            intensity: 0.0,
            envelope: 0.0,
            decay_coeff,
        }
    }

    /// Mode frequencies `[f_10, f_20, f_01]` in Hz.
    pub fn mode_frequencies(&self) -> [f32; 3] {
        self.mode_frequencies
    }

    /// Audio sample rate [Hz].
    pub fn sample_rate(&self) -> f32 {
        self.sample_rate
    }

    /// Sets the knock intensity `(I - 1.0).max(0.0)`.
    pub fn set_intensity(&mut self, intensity: f32) {
        self.intensity = intensity.max(0.0);
    }

    /// Triggers a knock burst on a cylinder firing event.
    pub fn trigger(&mut self) {
        if self.intensity > 1e-4 {
            self.envelope = (self.intensity * 0.8).min(2.0);
        }
    }

    /// Clears filter state and envelope.
    pub fn reset(&mut self) {
        self.mode_10.reset();
        self.mode_20.reset();
        self.mode_01.reset();
        self.envelope = 0.0;
    }

    /// Retunes mode frequencies from cylinder bore diameter and gas state.
    ///
    /// Frequency of cavity mode m:
    /// `f_m = rho_m * c / (pi * bore)`
    /// where `c = sqrt(gamma * R * T)`.
    pub fn tune(&mut self, bore: f32, gamma: f32, gas_constant: f32, temperature: f32) {
        let b = bore.max(0.010);
        let c = (gamma * gas_constant * temperature.max(200.0)).sqrt();
        let base_f = c / (std::f32::consts::PI * b);

        let f10 = (KNOCK_RHO_10 * base_f).clamp(100.0, 0.48 * self.sample_rate);
        let f20 = (KNOCK_RHO_20 * base_f).clamp(100.0, 0.48 * self.sample_rate);
        let f01 = (KNOCK_RHO_01 * base_f).clamp(100.0, 0.48 * self.sample_rate);

        self.mode_frequencies = [f10, f20, f01];
        self.mode_10
            .set_coeffs(BiquadCoeffs::bandpass(self.sample_rate, f10, KNOCK_Q));
        self.mode_20
            .set_coeffs(BiquadCoeffs::bandpass(self.sample_rate, f20, KNOCK_Q));
        self.mode_01
            .set_coeffs(BiquadCoeffs::bandpass(self.sample_rate, f01, KNOCK_Q));
    }

    /// Renders one sample of knock ringing.
    #[inline(always)]
    pub fn process(&mut self, noise: &mut Noise) -> f32 {
        if self.envelope < 1e-5 {
            return 0.0;
        }
        let env = self.envelope;
        self.envelope *= self.decay_coeff;

        let excitation = noise.next_bipolar() * env;
        let m10 = self.mode_10.process(excitation);
        let m20 = self.mode_20.process(excitation) * 0.5;
        let m01 = self.mode_01.process(excitation) * 0.25;

        (m10 + m20 + m01) * 0.35
    }
}

// ---------------------------------------------------------------------------
// The synth
// ---------------------------------------------------------------------------

/// The complete engine voice.
///
/// Owns every buffer it will ever need. Construct on the physics thread, move
/// into the audio callback, and drive it with [`EngineSynth::set_snapshot`] and
/// [`EngineSynth::render`].
#[derive(Debug, Clone)]
pub struct EngineSynth {
    config: SynthConfig,
    banks: Vec<ExhaustBank>,
    intake: IntakeVoice,
    turbo: TurboVoice,
    backfire: BackfireVoice,
    mechanical: MechanicalVoice,
    knock: KnockVoice,

    /// Master-cycle phase, `0..1` over 720 crank degrees.
    cycle_phase: f32,
    /// Crank angular velocity perturbation from nominal speed [rad/s].
    crank_omega_delta: f32,
    /// Samples remaining before the next control-rate update.
    ///
    /// Persists across `render` calls so the control grid is absolute rather
    /// than aligned to buffer boundaries. Without this, a device that asks for
    /// 512 frames and one that asks for 30 would run the control code at
    /// different instants and produce measurably different audio from identical
    /// physics — and a device with a variable buffer size would modulate its own
    /// timbre.
    control_countdown: usize,

    // Control-rate parameters: advanced in blocks, read when retuning filters.
    exhaust_temperature: Smoothed,
    exhaust_gamma: Smoothed,
    exhaust_gas_constant: Smoothed,
    intake_flow: Smoothed,
    throttle: Smoothed,
    blowdown_pa: Vec<Smoothed>,

    /// Per-cylinder draw for the next firing; parallel to `config.cylinders`.
    ///
    /// Allocated once at construction — the only heap the synth owns besides the
    /// delay lines, and like them it is never resized.
    variation: Vec<CycleVariation>,
    /// Gaussian sigma currently applied to amplitude and phase [-].
    ///
    /// Recomputed at the control rate and read by the firing path, so a firing
    /// never has to work out the schedule for itself.
    variation_depth: f32,

    // Per-sample parameters.
    cycle_hz: Smoothed,
    exhaust_level: Smoothed,

    snapshot: EngineSnapshot,
    noise: Noise,
    dc: [DcBlocker; 2],
    /// Structural rumble of the block and pan, on the output bus.
    block: BlockResonator,
    /// Ramps 0 to 1 on the first samples so opening the stream is silent.
    fade_in: Smoothed,
}

impl EngineSynth {
    /// Builds a synth for the given plumbing.
    pub fn new(config: SynthConfig) -> Self {
        let fs = config.sample_rate.max(8_000.0);
        let mut config = config;
        config.sample_rate = fs;
        if config.cylinders.is_empty() {
            config.cylinders.push(CylinderTap {
                evo_phase: 0.0,
                bank: 0,
            });
        }
        config.bank_count = config.bank_count.max(1);
        // A tap pointing at a bank that does not exist would panic on the hot
        // path; fold it into a real one now instead.
        for tap in config.cylinders.iter_mut() {
            tap.bank %= config.bank_count;
            tap.evo_phase = tap.evo_phase.rem_euclid(1.0);
        }

        let snapshot = EngineSnapshot::default();
        let banks = (0..config.bank_count)
            .map(|i| ExhaustBank::new(&config, i, &snapshot))
            .collect();
        let blowdown_pa = (0..config.cylinders.len())
            .map(|_| Smoothed::new(0.0, fs, 0.005))
            .collect();

        let mut synth = Self {
            banks,
            intake: IntakeVoice::new(fs),
            turbo: TurboVoice::new(fs),
            backfire: BackfireVoice::new(fs),
            mechanical: MechanicalVoice::new(fs),
            knock: KnockVoice::new(fs),
            variation: vec![CycleVariation::default(); config.cylinders.len()],
            variation_depth: 0.0,
            cycle_phase: 0.0,
            crank_omega_delta: 0.0,
            control_countdown: 0,
            exhaust_temperature: Smoothed::new(snapshot.exhaust_temperature, fs, 0.080),
            exhaust_gamma: Smoothed::new(snapshot.exhaust_gamma, fs, 0.080),
            exhaust_gas_constant: Smoothed::new(snapshot.exhaust_gas_constant, fs, 0.080),
            intake_flow: Smoothed::new(0.0, fs, 0.020),
            throttle: Smoothed::new(0.0, fs, 0.020),
            blowdown_pa,
            cycle_hz: Smoothed::new(0.0, fs, 0.030),
            exhaust_level: Smoothed::new(config.exhaust_level as f32, fs, 0.050),
            snapshot,
            noise: Noise::new(0x9E37_79B9),
            dc: [DcBlocker::default(); 2],
            // The runner and muffler are both bandpass-like and between them
            // leave the bottom octave thin, where a large engine's felt weight
            // actually lives. This fills it in with the block's own mode rather
            // than a flat shelf, so the weight tracks the engine's mass and
            // recedes as the firing frequency climbs past it.
            block: BlockResonator::new(
                fs,
                config.block_mass as f32,
                config.block_resonance_q as f32,
                config.block_resonance_db as f32,
            ),
            fade_in: Smoothed::new(0.0, fs, 0.015),
            config,
        };
        synth.fade_in.set_target(1.0);
        synth
    }

    /// The plumbing this synth was built with.
    pub fn config(&self) -> &SynthConfig {
        &self.config
    }

    /// The most recent physics state the synth has seen.
    pub fn snapshot(&self) -> &EngineSnapshot {
        &self.snapshot
    }

    /// Knock resonance voice.
    pub fn knock(&self) -> &KnockVoice {
        &self.knock
    }

    /// Current fundamental firing frequency, `f = (RPM / 120) * N_cyl` [Hz].
    pub fn firing_frequency(&self) -> f32 {
        self.cycle_hz.value() * self.config.cylinder_count() as f32
    }

    /// Crank angular velocity perturbation from nominal speed [rad/s].
    pub fn crank_omega_delta(&self) -> f32 {
        self.crank_omega_delta
    }

    /// Instantaneous crank angular velocity `w = w_0 + delta_w` [rad/s].
    pub fn crank_omega(&self) -> f32 {
        4.0 * std::f32::consts::PI * self.cycle_hz.value() + self.crank_omega_delta
    }

    /// Accepts a new physics frame.
    ///
    /// Cheap by design — it stores targets and returns. Nothing is recomputed
    /// here, because this runs at the head of a callback where the deadline is
    /// tightest, and every derived quantity is needed at the control rate
    /// anyway.
    pub fn set_snapshot(&mut self, snapshot: &EngineSnapshot) {
        let snapshot = snapshot.sanitized();
        self.snapshot = snapshot;

        // RPM / 120 is the four-stroke cycle rate: two revolutions per cycle.
        self.cycle_hz.set_target(snapshot.rpm / 120.0);
        for (i, smoother) in self.blowdown_pa.iter_mut().enumerate() {
            smoother.set_target(snapshot.blowdown_delta[i]);
        }
        self.exhaust_temperature
            .set_target(snapshot.exhaust_temperature);
        self.exhaust_gamma.set_target(snapshot.exhaust_gamma);
        self.exhaust_gas_constant
            .set_target(snapshot.exhaust_gas_constant);
        self.intake_flow.set_target(snapshot.intake_mass_flow);
        self.throttle.set_target(snapshot.throttle);
        self.knock.set_intensity(snapshot.knock_intensity);
    }

    /// Clears every filter and delay line without changing parameters.
    pub fn reset(&mut self) {
        for bank in self.banks.iter_mut() {
            bank.reset();
        }
        for smoother in self.blowdown_pa.iter_mut() {
            smoother.snap(0.0);
        }
        self.intake.reset();
        self.turbo.reset();
        self.mechanical.reset();
        self.knock.reset();
        self.dc = [DcBlocker::default(); 2];
        self.block.reset();
        self.cycle_phase = 0.0;
        self.crank_omega_delta = 0.0;
        self.control_countdown = 0;
        self.variation
            .iter_mut()
            .for_each(|v| *v = CycleVariation::default());
    }

    /// Gaussian sigma for cycle-to-cycle variation at a given speed [-].
    ///
    /// Zero above [`CCV_THRESHOLD_RPM`], ramping to
    /// [`SynthConfig::combustion_variation_max`] at
    /// [`SynthConfig::combustion_variation_idle_rpm`] and below.
    ///
    /// The step at the threshold is deliberate and inaudible: this is the
    /// standard deviation of a distribution, not a signal. Crossing 1500 rpm
    /// changes how much the *next* pulse is allowed to differ from nominal, and
    /// since the depth there is only 3 % the pulse either side of the crossing is
    /// drawn from near-identical distributions. Smoothing it would mean carrying
    /// a glide on a parameter nothing can hear move.
    fn variation_depth_at(&self, rpm: f32) -> f32 {
        if rpm >= CCV_THRESHOLD_RPM {
            return 0.0;
        }
        let idle = self.config.combustion_variation_idle_rpm as f32;
        let span = (CCV_THRESHOLD_RPM - idle).max(1.0);
        let t = ((CCV_THRESHOLD_RPM - rpm) / span).clamp(0.0, 1.0);
        let lo = self.config.combustion_variation_min as f32;
        let hi = self.config.combustion_variation_max as f32;
        (lo + (hi - lo) * t).max(0.0)
    }

    /// Draws cylinder `index`'s amplitude and phase for its *next* firing.
    ///
    /// Called immediately after that cylinder fires, which is the one moment in
    /// the cycle when moving its trigger phase is provably safe: the crank is
    /// then a full firing interval away from this cylinder's trigger, so no
    /// offset within [`CCV_MAX_PHASE_FRACTION`] can make the crossing test fire
    /// it twice or skip it. Drawing at the trigger instead would let a phase that
    /// jumped backwards re-trigger on the following sample.
    fn reroll_variation(&mut self, index: usize, depth: f32) {
        if depth <= 0.0 {
            // Also the fast path at speed: no RNG is consumed above the
            // threshold, so the noise stream — and every layer downstream of it
            // — is bit-identical to an engine built without this model.
            self.variation[index] = CycleVariation::default();
            return;
        }
        let spacing = 1.0 / self.config.cylinders.len().max(1) as f32;
        // The thermodynamic solver carries per-cylinder blowdown pressure,
        // so deterministic cylinder differences are physical. The synthetic
        // amplitude variation is reduced to the genuinely stochastic remainder.
        let amplitude = 1.0 + (0.5 * depth) * self.noise.next_gaussian();
        let phase = depth * spacing * self.noise.next_gaussian();
        self.variation[index] = CycleVariation {
            amplitude_scale: amplitude.clamp(
                1.0 - CCV_MAX_AMPLITUDE_EXCURSION,
                1.0 + CCV_MAX_AMPLITUDE_EXCURSION,
            ),
            phase_offset: phase.clamp(
                -CCV_MAX_PHASE_FRACTION * spacing,
                CCV_MAX_PHASE_FRACTION * spacing,
            ),
        };
    }

    /// Control-rate work: retune every filter whose coefficients depend on state.
    fn update_control(&mut self) {
        let block = CONTROL_BLOCK;
        let temperature = self.exhaust_temperature.advance(block);
        let gamma = self.exhaust_gamma.advance(block);
        let gas_constant = self.exhaust_gas_constant.advance(block);
        let flow = self.intake_flow.advance(block);
        let throttle = self.throttle.advance(block);
        for smoother in self.blowdown_pa.iter_mut() {
            smoother.advance(block);
        }

        // The smoothed speed, not the snapshot's: every schedule below has to
        // move with the same glide the rest of the synth is on, or a throttle
        // stab would step the damping and the block gain while the note itself
        // was still sliding.
        let cycle_hz = self.cycle_hz.value();
        let rpm = cycle_hz * 120.0;

        let damping_strength = self.config.waveguide_damping as f32;
        for bank in self.banks.iter_mut() {
            bank.tune(gamma, gas_constant, temperature);
            bank.tune_damping(rpm, damping_strength);
        }
        self.variation_depth = self.variation_depth_at(rpm);
        self.block.tune(rpm);
        self.mechanical.tune(
            self.snapshot.friction_mep,
            cycle_hz,
            self.config.cylinders.len(),
        );
        self.intake.tune(flow, throttle);
        self.knock
            .tune(self.snapshot.bore, gamma, gas_constant, temperature);
        // Nothing to tune on an atmospheric engine: the voice is left cold and
        // never asked for a sample, rather than run at a level of zero.
        if let Some(voicing) = self.config.turbo {
            self.turbo
                .tune(&voicing, self.snapshot.turbo_rpm, self.snapshot.turbo_surge);
        }

        self.backfire.tune(&self.config, &self.snapshot);
        if let Some((severity, decay)) = self.backfire.poll(&mut self.noise, CONTROL_BLOCK) {
            let bank = (self.noise.next_u32() as usize) % self.banks.len();
            let amplitude = severity * self.config.backfire_level as f32;
            // Backfires are almost entirely broadband and much longer than a
            // blowdown crack; they share the runner and muffler so they pick up
            // the same pipe colour.
            self.banks[bank].pulses.trigger(
                self.config.sample_rate,
                amplitude,
                0.0004,
                decay,
                0.92,
                self.noise.next_unit(),
                0.0,
            );
        }
    }

    /// Closed-form antiderivative of the normalized cylinder torque profile:
    /// `G(phi) = integral_0^phi (g(u) - 1) du`.
    ///
    /// At phi = 0 and phi = 1, G = 0 by construction (integral of g is 1.0).
    /// Subtracting the mean value over the cycle (0.71787) yields zero DC work.
    #[inline(always)]
    fn cylinder_excess_work(phi: f32) -> f32 {
        let int_g = if phi < 0.25 {
            let x = phi * 4.0;
            let pi_x = std::f32::consts::PI * x;
            let (sin_pix, cos_pix) = pi_x.sin_cos();
            let pi = std::f32::consts::PI;
            let term = (1.0 - (1.0 - 0.5 * x) * cos_pix) / pi - 0.5 * sin_pix / (pi * pi);
            2.992 * term
        } else if phi < 0.75 {
            1.4286
        } else {
            let x = (phi - 0.75) * 4.0;
            let pi_x = std::f32::consts::PI * x;
            let (sin_pix, cos_pix) = pi_x.sin_cos();
            let pi = std::f32::consts::PI;
            let term = sin_pix / (pi * pi) - x * cos_pix / pi;
            1.4286 - 1.3464 * term
        };
        int_g - phi - 0.71787
    }

    /// Advances crank phase by one sample and fires any cylinder it passes.
    #[inline(always)]
    fn advance_crank(&mut self, cycle_hz: f32) {
        let nominal_omega = 4.0 * std::f32::consts::PI * cycle_hz;
        if nominal_omega <= 1e-6 {
            self.crank_omega_delta = 0.0;
            return;
        }

        // Integrate intra-cycle crank speed ripple:
        // dw/dt = (T_indicated(theta) - T_mean) / I.
        // In terms of excess indicated work:
        // delta_w(theta) = delta_E(theta) / (I * w_0).
        let n_cylinders = self.config.cylinders.len().max(1);
        let t_mean = self.snapshot.indicated_torque;
        if t_mean > 0.0 {
            let mut blowdown_sum = 0.0f32;
            for smoother in &self.blowdown_pa {
                blowdown_sum += smoother.value();
            }

            let mut delta_e = 0.0f32;
            for (index, tap) in self.config.cylinders.iter().enumerate() {
                let weight = if blowdown_sum > 1.0 {
                    (self.blowdown_pa[index].value() / blowdown_sum) * (n_cylinders as f32)
                } else {
                    1.0
                };
                let t_mean_cyl = (t_mean / n_cylinders as f32) * weight;

                // Cycle angle relative to cylinder combustion TDC (180 deg before EVO).
                let phi = (self.cycle_phase - (tap.evo_phase - 0.25)).rem_euclid(1.0);
                delta_e += t_mean_cyl * Self::cylinder_excess_work(phi);
            }

            let delta_work = 4.0 * std::f32::consts::PI * delta_e;
            let inertia = self.snapshot.inertia.max(0.010);
            self.crank_omega_delta = delta_work / (inertia * nominal_omega);
        } else {
            self.crank_omega_delta = 0.0;
        }

        let omega = (nominal_omega + self.crank_omega_delta)
            .clamp(0.1 * nominal_omega, 3.0 * nominal_omega);
        let cycle_hz_instant = omega / (4.0 * std::f32::consts::PI);
        let increment = cycle_hz_instant / self.config.sample_rate;
        // A stopped or impossibly fast engine fires nothing. The upper guard
        // matters: past one cycle per sample the crossing test below would miss
        // events, and the result would be a phantom subharmonic.
        if !(1e-9..1.0).contains(&increment) {
            return;
        }

        let previous = self.cycle_phase;
        // Blowdown lasts a roughly fixed number of crank degrees, so its
        // duration in seconds is inversely proportional to engine speed.
        // Holding it fixed in time instead would smear the pulses into each
        // other at high rpm and leave the note hollow at low rpm.
        let cycle_seconds = 1.0 / cycle_hz.max(1e-3);
        let decay =
            (self.config.blowdown_degrees as f32 / 720.0 * cycle_seconds).clamp(0.0004, 0.020);
        // Attack and noise content are deliberately *not* functions of
        // amplitude. Making the pulse shape vary with loudness would leave
        // the radiated level only roughly proportional to the pressure
        // difference, and strict proportionality is the excitation model's
        // one quantitative claim. Everything that shapes the pulse is a
        // function of engine speed, which is orthogonal to it.
        let attack = 0.00016;
        let noise_depth = 0.5;
        let depth = self.variation_depth;
        // Firing jitter is a delay in samples, so it needs the cycle in
        // samples. Zero above CCV_THRESHOLD_RPM, where the mean retard
        // switches off with the variation it exists to carry.
        let cycle_samples = cycle_seconds * self.config.sample_rate;
        let retard = if depth > 0.0 {
            CCV_MEAN_RETARD_FRACTION / self.config.cylinders.len() as f32 * cycle_samples
        } else {
            0.0
        };

        for index in 0..self.config.cylinders.len() {
            let tap = self.config.cylinders[index];
            let variation = self.variation[index];
            // Distance from the previous phase forward to this cylinder's
            // trigger, wrapped into [0, 1). The trigger is the firing table
            // and nothing else — jitter is applied to the pulse, not to the
            // angle, so this test still fires each cylinder exactly once per
            // cycle no matter what was drawn.
            let ahead = (tap.evo_phase - previous).rem_euclid(1.0);
            if ahead < increment {
                let blowdown = self.blowdown_pa[index].value();
                // Strictly linear in the pressure difference, per the excitation model.
                let amplitude = (blowdown / REFERENCE_BLOWDOWN).min(2.0);
                if amplitude > 1e-5 {
                    // Fraction of this sample that has elapsed since the pulse
                    // began, which is how far into its envelope it already is.
                    let age = 1.0 - ahead / increment;
                    self.banks[tap.bank].pulses.trigger(
                        self.config.sample_rate,
                        amplitude * variation.amplitude_scale,
                        attack,
                        decay,
                        noise_depth,
                        age,
                        retard + variation.phase_offset * cycle_samples,
                    );
                    self.knock.trigger();
                    // Draw this cylinder's next cycle now that this one has
                    // been committed to a slot.
                    self.reroll_variation(index, depth);
                }
            }
        }

        self.cycle_phase = (previous + increment).rem_euclid(1.0);
    }

    /// Produces one stereo frame.
    #[inline(always)]
    fn tick(&mut self) -> (f32, f32) {
        let cycle_hz = self.cycle_hz.next_value();
        self.advance_crank(cycle_hz);

        let exhaust_level = self.exhaust_level.next_value();
        let (mut left, mut right) = (0.0f32, 0.0f32);
        for bank in self.banks.iter_mut() {
            let out = bank.process(&mut self.noise) * exhaust_level;
            left += out * bank.pan_left;
            right += out * bank.pan_right;
        }

        // Intake and turbo are near the listener's centre line and share one
        // mono source; only the exhaust is imaged.
        // Intake, turbo, the mechanical floor, and cylinder bore knock are near
        // the listener's centre line and share one mono source; only the exhaust
        // is imaged. The mechanical and knock layers belong here because they
        // radiate from the block, which is a single structural object sitting
        // between the banks rather than something with two outlets.
        let turbo = match self.config.turbo {
            Some(voicing) => self.turbo.process(&mut self.noise) * voicing.level as f32,
            None => 0.0,
        };
        let centre = self.intake.process(&mut self.noise) * self.config.intake_level as f32
            + turbo
            + self.mechanical.process(&mut self.noise) * self.config.mechanical_level as f32
            + self.knock.process(&mut self.noise);
        left += centre;
        right += centre;

        let gain = self.config.master_gain as f32 * self.fade_in.next_value();
        let mut out = [left, right];
        for (i, sample) in out.iter_mut().enumerate() {
            // DC first: the block resonator is a peaking section with real gain
            // at 80 Hz, and the blowdown train's offset is exactly the kind of
            // near-DC energy it would amplify into the clipper.
            let blocked = self.dc[i].process(*sample);
            *sample = soft_clip(self.block.process(i, blocked) * gain);
        }
        (out[0], out[1])
    }

    /// Fills an interleaved output buffer.
    ///
    /// Every sample of `output` is written on every call, including when the
    /// engine is stopped and when no snapshot has ever arrived — an early return
    /// would leave the device playing whatever was in the buffer before, which
    /// is the classic source of a periodic tick at the callback rate.
    pub fn render(&mut self, output: &mut [f32], channels: usize) {
        let channels = channels.max(1);
        let frames = output.len() / channels;
        let mut done = 0;

        while done < frames {
            if self.control_countdown == 0 {
                self.update_control();
                self.control_countdown = CONTROL_BLOCK;
            }
            let block = self.control_countdown.min(frames - done);

            for frame in 0..block {
                let (left, right) = self.tick();
                let base = (done + frame) * channels;
                let slice = &mut output[base..base + channels];
                match channels {
                    1 => slice[0] = 0.5 * (left + right),
                    _ => {
                        slice[0] = left;
                        slice[1] = right;
                        // Surround and multi-channel devices: leave the extra
                        // channels silent rather than duplicating, which would
                        // sum to a comb filter on any downmix.
                        slice[2..].iter_mut().for_each(|s| *s = 0.0);
                    }
                }
            }
            self.control_countdown -= block;
            done += block;
        }

        // The tail of a buffer whose length is not a whole number of frames.
        output[frames * channels..]
            .iter_mut()
            .for_each(|s| *s = 0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f32 = 48_000.0;

    fn loaded_snapshot() -> EngineSnapshot {
        EngineSnapshot {
            rpm: 3_000.0,
            blowdown_delta: [4.0e5; MAX_CYLINDERS],
            exhaust_temperature: 950.0,
            exhaust_gamma: 1.33,
            exhaust_gas_constant: 287.0,
            intake_mass_flow: 0.20,
            throttle: 0.8,
            turbo_rpm: 90_000.0,
            turbo_surge: 0.0,
            unburnt_fuel_mass: 0.0,
            // Chen-Flynn on the shipped V8 at 3000 rpm under load.
            friction_mep: 1.5e5,
            spark_cut: false,
            knock_intensity: 0.0,
            bore: 0.084,
            indicated_torque: 250.0,
            inertia: 0.25,
        }
    }

    fn render(synth: &mut EngineSynth, frames: usize) -> Vec<f32> {
        let mut buffer = vec![0.0; frames * 2];
        synth.render(&mut buffer, 2);
        buffer
    }

    /// Renders in irregular chunks, the way a real device asks for samples.
    fn render_chunked(synth: &mut EngineSynth, frames: usize, sizes: &[usize]) -> Vec<f32> {
        let mut out = Vec::with_capacity(frames * 2);
        let mut i = 0;
        while out.len() < frames * 2 {
            let n = sizes[i % sizes.len()];
            let mut chunk = vec![0.0; n * 2];
            synth.render(&mut chunk, 2);
            out.extend_from_slice(&chunk);
            i += 1;
        }
        out.truncate(frames * 2);
        out
    }

    fn peak(samples: &[f32]) -> f32 {
        samples.iter().fold(0.0f32, |m, s| m.max(s.abs()))
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() <= 1e-4, "expected {b}, got {a}");
    }

    fn rms(samples: &[f32]) -> f32 {
        let sum: f64 = samples.iter().map(|s| (*s as f64) * (*s as f64)).sum();
        (sum / samples.len().max(1) as f64).sqrt() as f32
    }

    /// Largest step between consecutive samples — the thing a "pop" actually is.
    fn max_slew(samples: &[f32], channels: usize) -> f32 {
        samples
            .chunks(channels)
            .zip(samples.chunks(channels).skip(1))
            .fold(0.0f32, |m, (a, b)| m.max((b[0] - a[0]).abs()))
    }

    #[test]
    fn silent_before_any_snapshot() {
        let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
        let out = render(&mut synth, 4_800);
        assert_eq!(peak(&out), 0.0, "a stopped engine must be silent");
    }

    #[test]
    fn output_is_finite_and_bounded_under_load() {
        let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
        synth.set_snapshot(&loaded_snapshot());
        let out = render(&mut synth, 48_000);
        assert!(out.iter().all(|s| s.is_finite()), "non-finite sample");
        assert!(peak(&out) <= 1.0, "clipped past full scale: {}", peak(&out));
        assert!(rms(&out) > 1e-3, "engine produced no sound");
    }

    #[test]
    fn firing_frequency_tracks_the_speed_law() {
        // f = (RPM / 120) * N_cyl. Count blowdown triggers over a known span and
        // compare against the closed form.
        let config = SynthConfig::uniform(FS, 4, 1);
        let mut synth = EngineSynth::new(config);
        let mut snapshot = loaded_snapshot();
        snapshot.rpm = 6_000.0;
        synth.set_snapshot(&snapshot);
        // Let the smoothed speed settle before counting.
        render(&mut synth, 24_000);

        let expected = 6_000.0 / 120.0 * 4.0; // 200 Hz
        assert!((synth.firing_frequency() - expected).abs() < 1.0);

        // And the phase really advances at RPM / 120 cycles per second.
        let before = synth.cycle_phase;
        render(&mut synth, FS as usize); // exactly one second
        let cycles = 6_000.0 / 120.0;
        let expected_phase = (before + cycles).rem_euclid(1.0);
        assert!(
            (synth.cycle_phase - expected_phase).abs() < 1e-2,
            "phase drifted: {} vs {expected_phase}",
            synth.cycle_phase
        );
    }

    #[test]
    fn pulse_amplitude_is_linear_in_pressure_difference() {
        let level_at = |delta: f32| {
            // Isolate the exhaust: no intake and no mechanical floor (the V8
            // has no turbo to silence). The floor matters most here — it is the
            // only layer that is *continuous*, so leaving it in would put a
            // constant term under both measurements and pull any ratio toward
            // one.
            let mut config = SynthConfig::cross_plane_v8(FS);
            config.mechanical_level = 0.0;
            let mut synth = EngineSynth::new(config);
            let mut snapshot = loaded_snapshot();
            snapshot.blowdown_delta = [delta; MAX_CYLINDERS];
            snapshot.intake_mass_flow = 0.0;
            snapshot.turbo_rpm = 0.0;
            synth.set_snapshot(&snapshot);
            render(&mut synth, 24_000); // settle
            rms(&render(&mut synth, 48_000))
        };

        let single = level_at(1.0e5);
        let double = level_at(2.0e5);
        let ratio = double / single;
        assert!(
            (1.85..2.15).contains(&ratio),
            "amplitude is not linear in dP: ratio {ratio}"
        );
    }

    #[test]
    fn hotter_exhaust_raises_the_muffler_resonance() {
        let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
        let mut snapshot = loaded_snapshot();
        snapshot.exhaust_temperature = 400.0;
        synth.set_snapshot(&snapshot);
        render(&mut synth, 24_000);
        let cold = synth.banks[0].muffler.centre_frequency();
        let cold_delay = synth.banks[0].runner.delay_samples();

        snapshot.exhaust_temperature = 1_200.0;
        synth.set_snapshot(&snapshot);
        render(&mut synth, 48_000);
        let hot = synth.banks[0].muffler.centre_frequency();
        let hot_delay = synth.banks[0].runner.delay_samples();

        // Both scale with the speed of sound, so both move by sqrt(T ratio).
        let expected = (1_200.0f32 / 400.0).sqrt();
        assert!(
            (hot / cold - expected).abs() < 0.05,
            "muffler: {hot}/{cold}"
        );
        assert!(
            (cold_delay / hot_delay - expected).abs() < 0.05,
            "runner: {cold_delay}/{hot_delay}"
        );
    }

    #[test]
    fn buffer_size_does_not_change_the_output() {
        // The whole point of the control-block scheme is that it is invisible.
        // Rendering the same state in 64-frame and ragged chunks must agree.
        let make = || {
            let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
            synth.set_snapshot(&loaded_snapshot());
            synth
        };
        let frames = 20_000;
        let reference = render(&mut make(), frames);
        let ragged = render_chunked(&mut make(), frames, &[1, 7, 31, 32, 33, 128, 511]);

        assert_eq!(reference.len(), ragged.len());
        let worst = reference
            .iter()
            .zip(&ragged)
            .fold(0.0f32, |m, (a, b)| m.max((a - b).abs()));
        assert!(worst < 1e-6, "output depends on buffer size: {worst}");
    }

    #[test]
    fn no_discontinuity_when_state_jumps() {
        // A snapshot that steps hard: idle to full load in one frame. Nothing in
        // the output may step with it.
        let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
        let idle = EngineSnapshot {
            rpm: 800.0,
            blowdown_delta: [0.6e5; MAX_CYLINDERS],
            exhaust_temperature: 600.0,
            intake_mass_flow: 0.02,
            throttle: 0.05,
            ..loaded_snapshot()
        };
        synth.set_snapshot(&idle);
        let settle = render(&mut synth, 48_000);
        let quiet_slew = max_slew(&settle, 2);

        let mut hot = loaded_snapshot();
        hot.rpm = 7_000.0;
        hot.blowdown_delta = [6.0e5; MAX_CYLINDERS];
        hot.exhaust_temperature = 1_250.0;
        hot.intake_mass_flow = 0.45;
        hot.throttle = 1.0;
        hot.turbo_rpm = 160_000.0;
        synth.set_snapshot(&hot);
        let after = render(&mut synth, 48_000);

        assert!(after.iter().all(|s| s.is_finite()));
        // The loud state legitimately has faster slew than idle, but a parameter
        // step must not produce a full-scale jump.
        let jump = max_slew(&after, 2);
        assert!(jump < 0.9, "step discontinuity in output: {jump}");
        assert!(quiet_slew < 0.5);
    }

    #[test]
    fn starvation_keeps_the_engine_running() {
        // Render ten seconds without ever updating the snapshot; the note must
        // continue, not decay to silence or freeze on a DC value.
        let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
        synth.set_snapshot(&loaded_snapshot());
        render(&mut synth, 48_000);
        let early = rms(&render(&mut synth, 48_000));
        let late = rms(&render(&mut synth, 9 * 48_000));
        assert!(late > 0.5 * early, "coasting decayed: {early} -> {late}");
        assert!(late < 2.0 * early, "coasting grew: {early} -> {late}");
    }

    #[test]
    fn nan_snapshot_cannot_reach_the_output() {
        let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
        synth.set_snapshot(&EngineSnapshot {
            rpm: f32::NAN,
            blowdown_delta: [f32::INFINITY; MAX_CYLINDERS],
            exhaust_temperature: f32::NAN,
            exhaust_gamma: -1.0,
            exhaust_gas_constant: 0.0,
            intake_mass_flow: f32::NAN,
            throttle: f32::INFINITY,
            turbo_rpm: f32::NAN,
            turbo_surge: f32::NAN,
            unburnt_fuel_mass: f32::NAN,
            friction_mep: f32::NAN,
            spark_cut: true,
            knock_intensity: f32::NAN,
            bore: f32::NAN,
            indicated_torque: f32::NAN,
            inertia: f32::NAN,
        });
        let out = render(&mut synth, 48_000);
        assert!(out.iter().all(|s| s.is_finite()), "NaN reached the device");
        assert!(peak(&out) <= 1.0);
    }

    #[test]
    fn extreme_speeds_do_not_break_the_trigger() {
        for rpm in [0.0, 1.0, 500.0, 12_000.0, 30_000.0] {
            let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
            let mut snapshot = loaded_snapshot();
            snapshot.rpm = rpm;
            synth.set_snapshot(&snapshot);
            let out = render(&mut synth, 24_000);
            assert!(out.iter().all(|s| s.is_finite()), "broke at {rpm} rpm");
            assert!(peak(&out) <= 1.0, "clipped at {rpm} rpm");
        }
    }

    #[test]
    fn backfires_need_fuel_heat_and_a_spark_cut() {
        let count_pops = |snapshot: EngineSnapshot| {
            let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
            // Silence everything but the pops so they can be counted by level.
            // The turbo needs no silencing: the V8 is naturally aspirated.
            synth.config.exhaust_level = 0.0;
            synth.exhaust_level.snap(0.0);
            synth.config.intake_level = 0.0;
            synth.config.mechanical_level = 0.0;
            synth.set_snapshot(&snapshot);
            let out = render(&mut synth, 5 * 48_000);
            peak(&out)
        };

        let mut primed = loaded_snapshot();
        primed.spark_cut = true;
        primed.unburnt_fuel_mass = 30.0e-6;
        primed.exhaust_temperature = 1_150.0;

        // Exhaust level is zeroed, so anything audible came from a backfire
        // injected into the same bank chain... which is also silenced. Instead
        // check the decision logic directly.
        assert_eq!(count_pops(primed), 0.0);

        let mut voice = BackfireVoice::new(FS);
        let config = SynthConfig::cross_plane_v8(FS);
        let mut noise = Noise::new(7);

        voice.tune(&config, &primed);
        assert!(voice.severity > 0.0, "primed exhaust should be able to pop");

        let mut fired = 0;
        for _ in 0..(5 * 48_000 / CONTROL_BLOCK) {
            if voice.poll(&mut noise, CONTROL_BLOCK).is_some() {
                fired += 1;
            }
        }
        assert!((10..=150).contains(&fired), "implausible pop rate: {fired}");

        // Each condition on its own must produce nothing.
        for spoiler in [
            EngineSnapshot {
                spark_cut: false,
                ..primed
            },
            EngineSnapshot {
                unburnt_fuel_mass: 0.0,
                ..primed
            },
            EngineSnapshot {
                exhaust_temperature: 500.0,
                ..primed
            },
        ] {
            let mut voice = BackfireVoice::new(FS);
            voice.tune(&config, &spoiler);
            assert_eq!(voice.severity, 0.0);
            assert!(voice.poll(&mut noise, CONTROL_BLOCK).is_none());
        }
    }

    #[test]
    fn surge_flutter_modulates_at_the_surge_rate() {
        let mut config = SynthConfig::cross_plane_v8(FS);
        // The catalogue V8 is atmospheric, so this test has to fit the turbo it
        // means to measure.
        config.turbo = Some(TurboVoicing::default());
        let mut synth = EngineSynth::new(config);
        synth.config.exhaust_level = 0.0;
        synth.exhaust_level.snap(0.0);
        synth.config.intake_level = 0.0;
        // The mechanical floor is continuous by construction; it would fill in
        // the gaps between chuffs and hide exactly what this measures.
        synth.config.mechanical_level = 0.0;
        let mut snapshot = loaded_snapshot();
        snapshot.blowdown_delta = [0.0; MAX_CYLINDERS];
        snapshot.intake_mass_flow = 0.0;
        snapshot.turbo_rpm = 120_000.0;
        snapshot.turbo_surge = 1.0;
        synth.set_snapshot(&snapshot);
        render(&mut synth, 48_000);

        let out = render(&mut synth, 48_000);
        // The chuff envelope should make the level swing substantially within a
        // second, at a rate in the surge band.
        let window = (FS / 400.0) as usize; // 2.5 ms
        let levels: Vec<f32> = out
            .chunks(window * 2)
            .map(|c| c.iter().fold(0.0f32, |m, s| m.max(s.abs())))
            .collect();
        let hi = levels.iter().cloned().fold(0.0f32, f32::max);
        let lo = levels.iter().cloned().fold(f32::MAX, f32::min);
        assert!(hi > 1e-4, "no turbo output at all");
        assert!(lo < 0.6 * hi, "flutter did not modulate: {lo} vs {hi}");
    }

    #[test]
    fn only_a_fitted_turbo_makes_turbo_sound() {
        // Everything but the turbo silenced, and a shaft spinning hard enough
        // that a fitted one would be at full song.
        let level_with = |turbo: Option<TurboVoicing>| {
            let mut config = SynthConfig::cross_plane_v8(FS);
            config.turbo = turbo;
            config.exhaust_level = 0.0;
            config.intake_level = 0.0;
            config.mechanical_level = 0.0;
            let mut synth = EngineSynth::new(config);
            synth.exhaust_level.snap(0.0);
            let mut snapshot = loaded_snapshot();
            snapshot.blowdown_delta = [0.0; MAX_CYLINDERS];
            snapshot.intake_mass_flow = 0.0;
            snapshot.turbo_rpm = 150_000.0;
            snapshot.turbo_surge = 0.8;
            synth.set_snapshot(&snapshot);
            render(&mut synth, 48_000); // settle
            peak(&render(&mut synth, 48_000))
        };

        // Not "quiet": an engine with no compressor on it has nothing to make
        // the sound with, so the only correct level is exactly zero.
        assert_eq!(
            level_with(None),
            0.0,
            "an atmospheric engine whistled anyway"
        );
        assert!(
            level_with(Some(TurboVoicing::default())) > 1e-3,
            "a fitted turbo made no sound"
        );
    }

    /// An idle snapshot with a plausible warm FMEP behind it.
    fn idle_snapshot() -> EngineSnapshot {
        EngineSnapshot {
            rpm: 800.0,
            blowdown_delta: [1.0e5; MAX_CYLINDERS],
            exhaust_temperature: 700.0,
            intake_mass_flow: 0.025,
            throttle: 0.04,
            turbo_rpm: 0.0,
            // Chen-Flynn at a warm idle: ~0.73 bar (~29 N m friction torque).
            friction_mep: 0.73e5,
            indicated_torque: 30.0,
            ..loaded_snapshot()
        }
    }

    /// Peak level inside each successive window of `window` frames.
    fn window_peaks(samples: &[f32], window: usize) -> Vec<f32> {
        samples
            .chunks(window * 2)
            .map(|c| c.iter().fold(0.0f32, |m, s| m.max(s.abs())))
            .collect()
    }

    /// Coefficient of variation of a series, `sigma / mean`.
    fn coefficient_of_variation(values: &[f32]) -> f32 {
        let n = values.len().max(1) as f64;
        let mean = values.iter().map(|v| *v as f64).sum::<f64>() / n;
        if mean <= 1e-12 {
            return 0.0;
        }
        let var = values
            .iter()
            .map(|v| (*v as f64 - mean) * (*v as f64 - mean))
            .sum::<f64>()
            / n;
        (var.sqrt() / mean) as f32
    }

    // -- cycle-to-cycle combustion variation ---------------------------------

    #[test]
    fn combustion_variation_depth_follows_the_speed_schedule() {
        let synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
        let config = synth.config().clone();

        // Full depth at and below the idle speed.
        approx(
            synth.variation_depth_at(config.combustion_variation_idle_rpm as f32),
            config.combustion_variation_max as f32,
        );
        approx(
            synth.variation_depth_at(300.0),
            config.combustion_variation_max as f32,
        );
        // The floor depth as it reaches the threshold, and nothing above it.
        assert!(
            (synth.variation_depth_at(CCV_THRESHOLD_RPM - 1.0)
                - config.combustion_variation_min as f32)
                .abs()
                < 1e-3
        );
        assert_eq!(synth.variation_depth_at(CCV_THRESHOLD_RPM), 0.0);
        assert_eq!(synth.variation_depth_at(6_000.0), 0.0);

        // Monotone in between: an engine cannot get rougher as it slows down
        // past some interior speed.
        let mut previous = 0.0;
        let mut rpm = CCV_THRESHOLD_RPM - 1.0;
        while rpm >= 400.0 {
            let d = synth.variation_depth_at(rpm);
            assert!(d >= previous - 1e-6, "depth fell as speed fell at {rpm}");
            previous = d;
            rpm -= 10.0;
        }
    }

    #[test]
    fn combustion_variation_draws_match_the_requested_sigma() {
        let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
        let depth = 0.08;
        let spacing = 1.0 / 8.0;

        let n = 40_000;
        let mut amplitudes = Vec::with_capacity(n);
        let mut phases = Vec::with_capacity(n);
        for _ in 0..n {
            synth.reroll_variation(0, depth);
            amplitudes.push(synth.variation[0].amplitude_scale);
            phases.push(synth.variation[0].phase_offset);
        }

        let stats = |v: &[f32]| {
            let n = v.len() as f64;
            let mean = v.iter().map(|x| *x as f64).sum::<f64>() / n;
            let var = v.iter().map(|x| (*x as f64 - mean).powi(2)).sum::<f64>() / n;
            (mean as f32, var.sqrt() as f32)
        };

        // Amplitude: unbiased around nominal, with the requested spread (reduced
        // to the stochastic remainder, half the total depth). The tolerance on
        // the mean is three standard errors of the estimator itself, sigma/sqrt(n).
        let amplitude_sigma = 0.5 * depth;
        let standard_error = amplitude_sigma / (n as f32).sqrt();
        let (mean, sigma) = stats(&amplitudes);
        assert!(
            (mean - 1.0).abs() < 3.0 * standard_error,
            "amplitude draw is biased: {mean}"
        );
        assert!(
            (sigma - amplitude_sigma).abs() < 0.004,
            "amplitude sigma {sigma}, want {amplitude_sigma}"
        );

        // Phase: sigma is the depth as a fraction of the firing interval, so at
        // 8 % on a V8 it is 8 % of 45 crank degrees — about 3.6 degrees.
        let (mean, sigma) = stats(&phases);
        assert!(
            mean.abs() < 3.0 * standard_error * spacing,
            "phase draw is biased: {mean}"
        );
        assert!(
            (sigma - depth * spacing).abs() < 0.0006,
            "phase sigma {sigma}, want {}",
            depth * spacing
        );

        // Nothing ever escapes the guard rails.
        assert!(amplitudes.iter().all(|a| (1.0 - CCV_MAX_AMPLITUDE_EXCURSION
            ..=1.0 + CCV_MAX_AMPLITUDE_EXCURSION)
            .contains(a)));
        let bound = CCV_MAX_PHASE_FRACTION * spacing;
        assert!(phases.iter().all(|p| p.abs() <= bound));

        // Zero depth is the nominal cycle, exactly.
        synth.reroll_variation(0, 0.0);
        assert_eq!(synth.variation[0].amplitude_scale, 1.0);
        assert_eq!(synth.variation[0].phase_offset, 0.0);
    }

    #[test]
    fn every_cylinder_fires_exactly_once_per_cycle_however_it_is_jittered() {
        // The invariant firing jitter must not break. Applying the jitter to
        // the trigger *angle* violates it: a cylinder that has just fired and
        // then has its angle redrawn forwards is crossed a second time in the
        // same cycle, which doubles its pulses and lifts the idle by several dB
        // of pure artefact. Scheduling the pulse behind an unmoved trigger is
        // what makes this hold.
        let mut config = SynthConfig::cross_plane_v8(FS);
        // Well past anything the speed schedule would ask for, so the draw is
        // hitting its clamps rather than sitting near nominal.
        config.combustion_variation_max = 0.25;
        config.combustion_variation_min = 0.25;
        let mut synth = EngineSynth::new(config);
        synth.set_snapshot(&idle_snapshot());
        // Start exactly on a cycle boundary at a settled speed, so the count is
        // a whole number of cycles by construction.
        synth.cycle_hz.snap(800.0 / 120.0);
        for (i, smoother) in synth.blowdown_pa.iter_mut().enumerate() {
            smoother.snap(idle_snapshot().blowdown_delta[i]);
        }
        synth.cycle_phase = 0.0;

        let cycles = 5usize;
        let samples = (FS / (800.0 / 120.0)) as usize * cycles;
        let mut last: Vec<usize> = synth.banks.iter().map(|b| b.pulses.next).collect();
        let mut fires = 0usize;
        let mut buffer = [0.0f32; 2];
        for _ in 0..samples {
            synth.render(&mut buffer, 2);
            for (i, bank) in synth.banks.iter().enumerate() {
                let slots = bank.pulses.slots.len();
                let advanced = (bank.pulses.next + slots - last[i]) % slots;
                assert!(
                    advanced <= 1,
                    "bank {i} triggered {advanced} pulses in one sample"
                );
                fires += advanced;
                last[i] = bank.pulses.next;
            }
        }

        let expected = synth.config().cylinder_count() * cycles;
        assert_eq!(
            fires, expected,
            "{fires} firings over {cycles} cycles, expected {expected}"
        );
    }

    #[test]
    fn cylinders_fire_at_different_amplitudes() {
        let mut config = SynthConfig::cross_plane_v8(FS);
        // Turn off stochastic variation so differences are purely deterministic.
        config.combustion_variation_max = 0.0;
        config.combustion_variation_min = 0.0;
        let mut synth = EngineSynth::new(config);

        let mut snapshot = idle_snapshot();
        // Set distinct blowdown pressures for cylinder 0 and cylinder 2 (both on bank 0).
        snapshot.blowdown_delta[0] = 1.0e5;
        snapshot.blowdown_delta[2] = 2.0e5;
        synth.set_snapshot(&snapshot);

        synth.cycle_hz.snap(800.0 / 120.0);
        for (i, smoother) in synth.blowdown_pa.iter_mut().enumerate() {
            smoother.snap(snapshot.blowdown_delta[i]);
        }
        synth.cycle_phase = 0.0;

        // Render through 0.30 of a cycle (covering 0 deg and 180 deg, before 270 deg)
        // to capture cylinder 0 (at 0 deg) and cylinder 2 (at 180 deg).
        let quarter_cycle_samples = (FS / (800.0 / 120.0) * 0.30) as usize;
        let mut buffer = [0.0f32; 2];
        let mut bank0_amplitudes = Vec::new();
        let mut last_next = synth.banks[0].pulses.next;

        for _ in 0..quarter_cycle_samples {
            synth.render(&mut buffer, 2);
            let next = synth.banks[0].pulses.next;
            if next != last_next {
                let slots = synth.banks[0].pulses.slots.len();
                let slot = (next + slots - 1) % slots;
                bank0_amplitudes.push(synth.banks[0].pulses.slots[slot].amplitude);
                last_next = next;
            }
        }

        assert_eq!(
            bank0_amplitudes.len(),
            2,
            "expected two firings on bank 0 in the first half-cycle"
        );
        let ratio = bank0_amplitudes[1] / bank0_amplitudes[0];
        assert!(
            (ratio - 2.0).abs() < 1e-3,
            "amplitude ratio was {ratio}, expected 2.0 (proportional to blowdown delta)"
        );
    }

    #[test]
    fn knock_is_silent_when_integral_below_one() {
        let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
        let mut snapshot = loaded_snapshot();
        snapshot.knock_intensity = 0.0;
        synth.set_snapshot(&snapshot);

        // Render audio through multiple cycles.
        let mut buffer = [0.0f32; 2];
        for _ in 0..1000 {
            synth.render(&mut buffer, 2);
        }

        // When knock intensity is 0.0 (knock integral below 1.0), the voice is completely silent.
        assert_eq!(synth.knock.envelope, 0.0);
        let mut noise = Noise::new(42);
        assert_eq!(synth.knock.process(&mut noise), 0.0);
    }

    #[test]
    fn knock_pitch_scales_inversely_with_bore() {
        let mut voice_small = KnockVoice::new(FS);
        let mut voice_large = KnockVoice::new(FS);

        let bore_small = 0.078;
        let bore_large = 0.078 * 2.0;
        let gamma = 1.33;
        let gas_constant = 287.0;
        let temperature = 1150.0;

        voice_small.tune(bore_small, gamma, gas_constant, temperature);
        voice_large.tune(bore_large, gamma, gas_constant, temperature);

        let modes_small = voice_small.mode_frequencies();
        let modes_large = voice_large.mode_frequencies();

        for i in 0..3 {
            let ratio = modes_small[i] / modes_large[i];
            assert!(
                (ratio - 2.0).abs() < 1e-4,
                "mode {i} ratio {ratio} != 2.0: doubling bore must halve mode frequency"
            );
        }

        // Also verify through EngineSynth update_control.
        let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
        let mut snapshot1 = loaded_snapshot();
        snapshot1.bore = 0.078;
        synth
            .exhaust_temperature
            .snap(snapshot1.exhaust_temperature);
        synth.set_snapshot(&snapshot1);
        synth.update_control();
        let f_small = synth.knock().mode_frequencies()[0];

        let mut snapshot2 = loaded_snapshot();
        snapshot2.bore = 0.078 * 2.0;
        synth
            .exhaust_temperature
            .snap(snapshot2.exhaust_temperature);
        synth.set_snapshot(&snapshot2);
        synth.update_control();
        let f_large = synth.knock().mode_frequencies()[0];

        let ratio = f_small / f_large;
        assert!(
            (ratio - 2.0).abs() < 1e-4,
            "EngineSynth knock mode ratio {ratio} != 2.0"
        );
    }

    #[test]
    fn firing_intervals_ripple_at_idle_and_converge_at_limiter() {
        // Measures interval in samples between successive cylinder firings across the engine.
        let measure_intervals = |rpm: f32, torque: f32, cycles: usize| -> (Vec<f32>, usize) {
            let mut config = SynthConfig::cross_plane_v8(FS);
            // Disable stochastic CCV so interval variations are purely from crank dynamics.
            config.combustion_variation_max = 0.0;
            config.combustion_variation_min = 0.0;
            let mut synth = EngineSynth::new(config);

            let mut snapshot = idle_snapshot();
            snapshot.rpm = rpm;
            snapshot.indicated_torque = torque;
            // Distinct per-cylinder blowdowns (physical cylinder differences, Stage 1b).
            for i in 0..8 {
                snapshot.blowdown_delta[i] = 1.0e5 + (i as f32 * 0.2e5);
            }
            synth.set_snapshot(&snapshot);
            synth.cycle_hz.snap(rpm / 120.0);
            for (i, smoother) in synth.blowdown_pa.iter_mut().enumerate() {
                smoother.snap(snapshot.blowdown_delta[i]);
            }
            synth.cycle_phase = 0.001;

            let expected_fires = 8 * cycles;
            let mut last: Vec<usize> = synth.banks.iter().map(|b| b.pulses.next).collect();
            let mut firing_times = Vec::new();
            let mut buffer = [0.0f32; 2];
            let mut sample_idx = 0;

            while firing_times.len() < expected_fires && sample_idx < 100_000 {
                synth.render(&mut buffer, 2);
                for (i, bank) in synth.banks.iter().enumerate() {
                    let slots = bank.pulses.slots.len();
                    let advanced = (bank.pulses.next + slots - last[i]) % slots;
                    if advanced > 0 {
                        firing_times.push(sample_idx);
                    }
                    last[i] = bank.pulses.next;
                }
                sample_idx += 1;
            }

            let intervals = firing_times
                .windows(2)
                .map(|w| (w[1] - w[0]) as f32)
                .collect();
            (intervals, firing_times.len())
        };

        let cycles = 5;
        let (idle_intervals, idle_fires) = measure_intervals(800.0, 50.0, cycles);
        let (limiter_intervals, limiter_fires) = measure_intervals(7000.0, 50.0, cycles);

        // Every cylinder still fires exactly once per cycle (8 cylinders * 5 cycles = 40).
        let expected_fires = 8 * cycles;
        assert_eq!(
            idle_fires, expected_fires,
            "idle: {idle_fires} firings over {cycles} cycles, expected {expected_fires}"
        );
        assert_eq!(
            limiter_fires, expected_fires,
            "limiter: {limiter_fires} firings over {cycles} cycles, expected {expected_fires}"
        );

        // At idle, intra-cycle torque acceleration and compression deceleration
        // cause the sample interval between consecutive cylinder firings to ripple.
        let idle_cv = coefficient_of_variation(&idle_intervals);
        assert!(
            idle_cv > 0.01,
            "idle firing intervals must ripple: COV {idle_cv:.4}"
        );

        // At limiter, large rotating inertia (I * omega) dominates gas torque,
        // so firing intervals converge toward uniform.
        let limiter_cv = coefficient_of_variation(&limiter_intervals);
        assert!(
            limiter_cv < idle_cv * 0.25,
            "firing intervals must converge toward uniform at limiter: {limiter_cv:.4} vs {idle_cv:.4}"
        );
    }

    #[test]
    fn combustion_variation_is_inert_above_the_threshold() {
        // Above CCV_THRESHOLD_RPM no draw is taken, so the whole synth — every
        // layer downstream of the shared noise generator included — must be
        // bit-identical whether or not the model is configured on.
        let render_with = |max: f64| {
            let mut config = SynthConfig::cross_plane_v8(FS);
            config.combustion_variation_max = max;
            config.combustion_variation_min = max * 0.4;
            let mut synth = EngineSynth::new(config);
            let mut snapshot = loaded_snapshot();
            snapshot.rpm = 3_000.0;
            synth.set_snapshot(&snapshot);
            // Start *at* speed rather than gliding up to it. The smoothed speed
            // otherwise sweeps through the low-rpm band on its way, where the
            // model is legitimately live and does consume draws — a spin-up is
            // not the steady state this asserts about.
            synth.cycle_hz.snap(3_000.0 / 120.0);
            render(&mut synth, 48_000)
        };
        assert_eq!(
            render_with(0.0),
            render_with(0.08),
            "CCV leaked above 1500 rpm"
        );
    }

    #[test]
    fn combustion_variation_lopes_the_idle() {
        // At idle the same configuration must produce a *different* engine with
        // the model on, and specifically one whose firing-to-firing peak level
        // wanders. That wander is the lope.
        let peaks_with = |max: f64| {
            let mut config = SynthConfig::cross_plane_v8(FS);
            config.combustion_variation_max = max;
            config.combustion_variation_min = max * 0.4;
            // Only the exhaust: the noise layers have a spread of their own and
            // would mask the one being measured.
            config.intake_level = 0.0;
            config.mechanical_level = 0.0;
            let mut synth = EngineSynth::new(config);
            synth.set_snapshot(&idle_snapshot());
            render(&mut synth, 48_000); // settle
            let out = render(&mut synth, 4 * 48_000);
            // One firing interval on a V8 at 800 rpm: 720/8 degrees = 11.25 ms.
            window_peaks(&out, (FS * 0.01125) as usize)
        };

        let steady = coefficient_of_variation(&peaks_with(0.0));
        let loping = coefficient_of_variation(&peaks_with(0.08));
        // A fifth again as much spread. The measurement understates the effect:
        // the windows are a fixed grid while the firings now move about inside
        // it, so a jittered pulse that straddles a boundary is counted at its
        // full height in both windows.
        assert!(
            loping > 1.2 * steady,
            "no lope: COV {loping:.4} with variation vs {steady:.4} without"
        );
    }

    // -- block resonance -----------------------------------------------------

    #[test]
    fn block_rumble_is_strongest_at_idle_and_gone_at_speed() {
        let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
        synth.set_snapshot(&idle_snapshot());
        render(&mut synth, 48_000);
        let idle_db = synth.block.gain_db();

        let mut fast = loaded_snapshot();
        fast.rpm = 6_000.0;
        synth.set_snapshot(&fast);
        render(&mut synth, 4 * 48_000);
        let fast_db = synth.block.gain_db();

        let configured = SynthConfig::default().block_resonance_db as f32;
        assert!(
            idle_db > 0.9 * configured,
            "no rumble at idle: {idle_db} dB of a configured {configured}"
        );
        assert!(fast_db < 0.5, "still rumbling at 6000 rpm: {fast_db} dB");
    }

    #[test]
    fn a_heavier_block_rumbles_lower() {
        let centre_for = |mass: f64| {
            let mut config = SynthConfig::cross_plane_v8(FS);
            config.block_mass = mass;
            let mut synth = EngineSynth::new(config);
            synth.set_snapshot(&idle_snapshot());
            render(&mut synth, 4_800);
            synth.block.centre_frequency()
        };
        let alloy_four = centre_for(95.0);
        let iron_v8 = centre_for(240.0);
        assert!(
            alloy_four > iron_v8 + 15.0,
            "mass barely moved the mode: {alloy_four} vs {iron_v8} Hz"
        );
        // Both inside the band the model advertises.
        for f in [alloy_four, iron_v8] {
            assert!((60.0..=120.0).contains(&f), "outside 60-120 Hz: {f}");
        }
    }

    #[test]
    fn block_rumble_puts_weight_in_the_bottom_octave() {
        // The audible claim: at idle the engine has more low-frequency energy
        // with the resonator than without, and no more peak level than the
        // clipper allows.
        let low_energy = |db: f64| {
            let mut config = SynthConfig::cross_plane_v8(FS);
            config.block_resonance_db = db;
            let mut synth = EngineSynth::new(config);
            synth.set_snapshot(&idle_snapshot());
            render(&mut synth, 48_000);
            let out = render(&mut synth, 2 * 48_000);
            // Crude 80 Hz band energy by quadrature correlation on the left
            // channel — enough to compare two runs of the same signal.
            let (mut re, mut im) = (0.0f64, 0.0f64);
            for (i, frame) in out.chunks(2).enumerate() {
                let phase = TAU as f64 * 80.0 * i as f64 / FS as f64;
                re += frame[0] as f64 * phase.sin();
                im += frame[0] as f64 * phase.cos();
            }
            ((re * re + im * im).sqrt() / (out.len() / 2) as f64) as f32
        };

        let flat = low_energy(0.0);
        let resonant = low_energy(9.0);
        assert!(
            resonant > 1.5 * flat,
            "the resonator added no weight: {resonant:.5} vs {flat:.5}"
        );
    }

    // -- waveguide damping ---------------------------------------------------

    #[test]
    fn waveguide_is_damped_at_idle_and_open_at_speed() {
        let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
        synth.set_snapshot(&idle_snapshot());
        render(&mut synth, 2 * 48_000);
        let idle_damping = synth.banks[0].runner.damping();
        let idle_reflection = synth.banks[0].runner.reflection();

        let mut fast = loaded_snapshot();
        fast.rpm = 7_000.0;
        synth.set_snapshot(&fast);
        render(&mut synth, 4 * 48_000);
        let fast_damping = synth.banks[0].runner.damping();
        let fast_reflection = synth.banks[0].runner.reflection();

        assert!(idle_damping > 0.85, "idle left undamped: {idle_damping}");
        assert!(fast_damping < 0.15, "top end over-damped: {fast_damping}");
        assert!(
            idle_reflection < 0.6 * fast_reflection,
            "reflection did not follow the rule: {idle_reflection} vs {fast_reflection}"
        );
    }

    #[test]
    fn the_rule_flattens_the_comb_at_idle_and_leaves_it_at_speed() {
        // The end-to-end claim. Settle the synth at a speed, take the runner it
        // has actually arrived at, and measure the ripple in its magnitude
        // response — a comb filter *is* deep regular ripple, and how deep it is
        // is how audible the flange is.
        let ripple_db = |rpm: f32, strength: f64| {
            let mut config = SynthConfig::cross_plane_v8(FS);
            config.waveguide_damping = strength;
            let mut synth = EngineSynth::new(config);
            let mut snapshot = idle_snapshot();
            snapshot.rpm = rpm;
            synth.set_snapshot(&snapshot);
            render(&mut synth, 4 * 48_000);

            let (mut lo, mut hi) = (f32::MAX, 0.0f32);
            let mut f = 200.0f32;
            while f <= 2_000.0 {
                let mut runner = synth.banks[0].runner.clone();
                let (mut re, mut im) = (0.0f64, 0.0f64);
                for i in 0..12_000 {
                    let phase = TAU * f * i as f32 / FS;
                    let y = runner.process(phase.sin());
                    if i >= 6_000 {
                        re += y as f64 * phase.sin() as f64;
                        im += y as f64 * phase.cos() as f64;
                    }
                }
                let m = (2.0 * (re * re + im * im).sqrt() / 6_000.0) as f32;
                lo = lo.min(m);
                hi = hi.max(m);
                f += 20.0;
            }
            20.0 * (hi / lo.max(1e-9)).log10()
        };

        let idle = ripple_db(800.0, 1.0);
        let fast = ripple_db(7_000.0, 1.0);
        let idle_unruled = ripple_db(800.0, 0.0);

        assert!(
            idle < 0.5 * idle_unruled,
            "the rule left the idle comb standing: {idle:.1} dB vs {idle_unruled:.1} dB unruled"
        );
        assert!(
            fast > 1.8 * idle,
            "the rule took the pipe away at speed too: {fast:.1} dB at 7000 vs {idle:.1} dB at idle"
        );
    }

    // -- mechanical noise floor ----------------------------------------------

    #[test]
    fn mechanical_layers_sit_at_unity() {
        // MECHANICAL_RUMBLE_MAKEUP exists to make `mechanical_level` mean the
        // same thing as the other level controls: with FMEP at the reference,
        // the rumble path leaves the voice at unity RMS.
        let mut voice = MechanicalVoice::new(FS);
        let mut noise = Noise::new(19);
        voice.tune(REFERENCE_FMEP, 0.0, 8);
        voice.rumble_gain.snap(1.0);
        voice.click_gain.snap(0.0);
        voice.event_hz.snap(0.0);

        let mut sum_sq = 0.0f64;
        let n = 192_000;
        for _ in 0..n {
            let y = voice.process(&mut noise);
            sum_sq += (y as f64) * (y as f64);
        }
        let rms = (sum_sq / n as f64).sqrt() as f32;
        assert!(
            (rms - 1.0).abs() < 0.05,
            "rumble makeup is out of calibration: {rms} (adjust MECHANICAL_RUMBLE_MAKEUP)"
        );
    }

    #[test]
    fn mechanical_floor_scales_with_friction() {
        let level_at = |fmep: f32| {
            let mut config = SynthConfig::cross_plane_v8(FS);
            config.exhaust_level = 0.0;
            config.intake_level = 0.0;
            let mut synth = EngineSynth::new(config);
            synth.exhaust_level.snap(0.0);
            let mut snapshot = idle_snapshot();
            snapshot.friction_mep = fmep;
            synth.set_snapshot(&snapshot);
            render(&mut synth, 48_000);
            rms(&render(&mut synth, 48_000))
        };

        let light = level_at(0.7e5);
        let heavy = level_at(2.1e5);
        assert!(light > 1e-5, "no mechanical noise at idle friction");
        assert!(
            heavy > 1.5 * light,
            "friction did not drive the floor: {heavy:.6} vs {light:.6}"
        );

        // A stopped engine has no mechanical noise, whatever the correlation's
        // constant term says.
        let mut config = SynthConfig::cross_plane_v8(FS);
        config.exhaust_level = 0.0;
        let mut synth = EngineSynth::new(config);
        synth.exhaust_level.snap(0.0);
        synth.set_snapshot(&EngineSnapshot {
            rpm: 0.0,
            friction_mep: 0.35e5,
            ..EngineSnapshot::default()
        });
        render(&mut synth, 48_000);
        assert!(
            rms(&render(&mut synth, 24_000)) < 1e-5,
            "a stopped engine idled"
        );
    }

    #[test]
    fn mechanical_floor_fills_the_gaps_between_firings() {
        // The failure this guards against: an idle built from combustion alone
        // has audible silence between its pulses, which is the loudest tell
        // that a sound is synthesised.
        // Measured on a four rather than the V8: at 800 rpm a four fires every
        // 37 ms and its pulses are gone in 4, so the gaps are real. A V8 at the
        // same speed fires three times as often and partly fills its own.
        let gap_floor = |mechanical: f64| {
            let mut config = SynthConfig::uniform(FS, 4, 1);
            config.mechanical_level = mechanical;
            config.intake_level = 0.0;
            let mut synth = EngineSynth::new(config);
            synth.set_snapshot(&idle_snapshot());
            render(&mut synth, 48_000);
            let out = render(&mut synth, 2 * 48_000);
            let mut peaks = window_peaks(&out, (FS * 0.001) as usize);
            peaks.sort_by(|a, b| a.partial_cmp(b).unwrap());
            peaks[peaks.len() / 10]
        };

        let bare = gap_floor(0.0);
        let filled = gap_floor(SynthConfig::default().mechanical_level);
        assert!(
            filled > 1.5 * bare,
            "the floor did not fill the gaps: {filled:.6} vs {bare:.6}"
        );
    }

    #[test]
    fn valve_clicks_keep_time_with_the_camshaft() {
        // Two seatings per cylinder per cycle, and the cam does not care about
        // the firing order — so the rate is the cycle rate times 2 N_cyl.
        let mut voice = MechanicalVoice::new(FS);
        let mut noise = Noise::new(5);
        let cycle_hz = 800.0 / 120.0; // 800 rpm
        voice.tune(1.0e5, cycle_hz, 8);
        voice
            .event_hz
            .snap(cycle_hz * 8.0 * VALVE_EVENTS_PER_CYLINDER);
        voice.rumble_gain.snap(0.0);
        voice.click_gain.snap(1.0);

        let mut events = 0usize;
        let seconds = 4;
        for _ in 0..(seconds * FS as usize) {
            let before = voice.click_phase;
            voice.process(&mut noise);
            if voice.click_phase < before {
                events += 1;
            }
        }
        let expected = (cycle_hz * 8.0 * VALVE_EVENTS_PER_CYLINDER) as usize * seconds;
        assert!(
            events.abs_diff(expected) <= 2,
            "valve events: {events}, expected about {expected}"
        );
    }

    #[test]
    fn mono_and_multichannel_buffers_are_filled_completely() {
        for channels in [1usize, 2, 4] {
            let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
            synth.set_snapshot(&loaded_snapshot());
            let mut buffer = vec![f32::NAN; 512 * channels];
            synth.render(&mut buffer, channels);
            assert!(
                buffer.iter().all(|s| s.is_finite()),
                "{channels} channels: buffer left unwritten"
            );
        }
    }

    #[test]
    fn ragged_buffer_lengths_are_handled() {
        let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
        synth.set_snapshot(&loaded_snapshot());
        // A length that is not a whole number of stereo frames.
        let mut buffer = vec![f32::NAN; 101];
        synth.render(&mut buffer, 2);
        assert!(buffer.iter().all(|s| s.is_finite()));
        let mut empty: Vec<f32> = Vec::new();
        synth.render(&mut empty, 2);
    }

    #[test]
    fn degenerate_configs_do_not_panic() {
        let mut config = SynthConfig::uniform(FS, 1, 1);
        config.cylinders.clear();
        config.bank_count = 0;
        let mut synth = EngineSynth::new(config);
        synth.set_snapshot(&loaded_snapshot());
        assert!(render(&mut synth, 4_800).iter().all(|s| s.is_finite()));

        // A tap pointing outside the bank list must be folded, not panic.
        let mut config = SynthConfig::uniform(FS, 4, 2);
        config.cylinders[2].bank = 99;
        let mut synth = EngineSynth::new(config);
        synth.set_snapshot(&loaded_snapshot());
        assert!(render(&mut synth, 4_800).iter().all(|s| s.is_finite()));
    }
}
