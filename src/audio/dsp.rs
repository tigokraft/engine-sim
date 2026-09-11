//! The synthesis voice: physics state in, stereo samples out.
//!
//! # What the audio thread does and does not know
//!
//! The synth never sees the engine. It sees [`EngineSnapshot`] — a dozen scalars
//! describing the thermodynamic state at some recent instant — and it is
//! responsible for turning those into a continuous waveform no matter how
//! irregularly they arrive. That division matters:
//!
//! - **The physics thread owns amplitude, and the shape.** How hard a cylinder
//!   hits is a thermodynamic fact (the blowdown pressure difference at EVO), and
//!   only the solver knows it. So is what the pulse *looks* like: the snapshot
//!   carries the cylinder pressure and both port flows over the whole cycle, and
//!   [`CyclePlayer`] plays them back at crank rate. There is no pulse shape to
//!   choose here, and no constant that sets one.
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
//!   valvetrain clicks + FMEP rumble ──┐                                 │
//!   dP/dtheta per cylinder ───────────┴> modal block ────────> centre ──┤
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
//!   them with the noise floor a real engine never stops making — radiating,
//!   as it physically must, through the block rather than through the air.
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
    DcBlocker, ModalBank, Noise, OnePole, Smoothed,
};
use crate::audio::intake_voice::IntakeNetwork;
use crate::audio::structure::{combustion_drive, StructuralPath, StructuralSpec};
use crate::audio::waveguide::ExhaustNetwork;
use crate::physics::plumbing::{ExhaustSystem, IntakeSystem};

pub use crate::physics::engine_block::CYCLE_TABLE;

/// Samples between control-rate updates.
///
/// 32 samples is 0.67 ms at 48 kHz — far below the ~10 ms it takes a listener to
/// resolve a timbral change, and 32x cheaper than redesigning coefficients every
/// sample.
pub const CONTROL_BLOCK: usize = 32;

/// One whole master cycle, in the fixed-point units crank phase is kept in [-].
///
/// Crank phase is integrated one sample at a time for as long as the stream is
/// open — tens of millions of additions an hour — and a `f32` accumulator wrapped
/// into `0..1` cannot do that without walking. The increment at 3000 rpm is five
/// parts in ten thousand of a cycle, so adding it to a phase near one rounds away
/// a tenth of a per mille of it every time, in the same direction, and a quarter
/// of a cycle has gone missing inside a minute. A 32-bit fraction that simply
/// wraps has no such bias: the addition is exact, the wrap is free, and the only
/// error left is in quantising the increment once, which is a frequency offset of
/// well under a part per billion and does not accumulate at all.
const PHASE_ONE: f32 = 4_294_967_296.0;

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

/// Reference acoustic pressure scale for intake network excitations [Pa].
///
/// Induction rarefactions (-c * mdot / A) and valve-closing water hammer pulses
/// range from 10 to 40 kPa; 50 kPa normalises radiated mouth pressure to unity.
pub const REFERENCE_INTAKE_PRESSURE: f32 = 5.0e4;

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

/// Peak in-cylinder pressure that maps to reference piston slap amplitude [Pa].
///
/// Typical peak cylinder pressure at normal load sits around 50 to 70 bar, so
/// 60 bar (6.0e6 Pa) maps to unity scale for piston slap impacts.
pub const REFERENCE_PEAK_PRESSURE: f32 = 60.0e5;

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
    /// Instantaneous mass flow through each cylinder's intake port [kg/s].
    pub cylinder_intake_flow: [f32; MAX_CYLINDERS],
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
    /// Peak in-cylinder combustion pressure over the cycle [Pa].
    pub peak_cylinder_pressure: f32,
    /// Mean indicated torque over the cycle [N m].
    pub indicated_torque: f32,
    /// Rotating assembly inertia [kg m^2].
    pub inertia: f32,
    /// Master-cylinder pressure over the cycle, from exhaust valve opening [Pa].
    ///
    /// The solver's own curve, downsampled from the 720-cell phase ring by
    /// [`crate::physics::engine_block::PhaseRing::downsample_from`]. Point `k`
    /// is the mean over `[k, k+1) / CYCLE_TABLE` of the cycle and stands at the
    /// centre of that span, so a player reads it at `phase * CYCLE_TABLE - 0.5`.
    ///
    /// Cut at EVO because that is the event the audio thread schedules against:
    /// index zero is the instant a cylinder's exhaust valve opens, whichever
    /// cylinder it is and wherever its firing offset puts it.
    pub cylinder_pressure: [f32; CYCLE_TABLE],
    /// Mass flow through the exhaust port [kg/s], positive *out* of the cylinder.
    ///
    /// Signed, because reverse flow through an open valve is a real event with a
    /// real sound: late in overlap the pipe can be above the cylinder and push
    /// gas back through the port. What matters acoustically is that the valve is
    /// *off its seat* — which is when the magnitude of this is non-zero — while
    /// the direction the wave goes is set by the pressure difference driving it.
    /// Same phase convention as [`Self::cylinder_pressure`].
    pub exhaust_port_flow: [f32; CYCLE_TABLE],
    /// Mass flow entering the cylinder through the intake port [kg/s].
    ///
    /// Positive *in*, which is the direction that empties the runner and makes
    /// induction noise. Same phase convention as [`Self::cylinder_pressure`].
    pub intake_port_flow: [f32; CYCLE_TABLE],
    /// Mean exhaust manifold pressure across the banks [Pa].
    ///
    /// What [`Self::cylinder_pressure`] is measured against: the pipe is driven
    /// by the cylinder's excess over the gas already in it, and an absolute
    /// trace on its own would present the manifold's own standing pressure as a
    /// permanent offset for the delay lines to integrate.
    pub exhaust_manifold_pressure: f32,
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
            cylinder_intake_flow: [0.0; MAX_CYLINDERS],
            throttle: 0.0,
            turbo_rpm: 0.0,
            turbo_surge: 0.0,
            unburnt_fuel_mass: 0.0,
            friction_mep: 0.0,
            spark_cut: false,
            knock_intensity: 0.0,
            bore: 0.084,
            peak_cylinder_pressure: 0.0,
            indicated_torque: 0.0,
            inertia: 0.25,
            cylinder_pressure: [0.0; CYCLE_TABLE],
            exhaust_port_flow: [0.0; CYCLE_TABLE],
            intake_port_flow: [0.0; CYCLE_TABLE],
            exhaust_manifold_pressure: 101_325.0,
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
        for f in self.cylinder_intake_flow.iter_mut() {
            if !f.is_finite() {
                *f = 0.0;
            }
            *f = f.clamp(0.0, 10.0);
        }
        guard!(throttle, 0.0, 1.0);
        guard!(turbo_rpm, 0.0, 400_000.0);
        guard!(turbo_surge, 0.0, 1.0);
        guard!(unburnt_fuel_mass, 0.0, 1.0);
        guard!(friction_mep, 0.0, 2.0e6);
        guard!(knock_intensity, 0.0, 50.0);
        guard!(bore, 0.010, 0.500);
        guard!(peak_cylinder_pressure, 0.0, 50.0e6);
        guard!(indicated_torque, -500.0, 50_000.0);
        guard!(inertia, 0.010, 100.0);
        guard!(exhaust_manifold_pressure, 1_000.0, 2.0e6);
        for p in self.cylinder_pressure.iter_mut() {
            if !p.is_finite() {
                *p = 0.0;
            }
            *p = p.clamp(0.0, 50.0e6);
        }
        for f in self.exhaust_port_flow.iter_mut() {
            if !f.is_finite() {
                *f = 0.0;
            }
            *f = f.clamp(-50.0, 50.0);
        }
        for f in self.intake_port_flow.iter_mut() {
            if !f.is_finite() {
                *f = 0.0;
            }
            *f = f.clamp(0.0, 50.0);
        }
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

/// How an impulsive mechanical source sets its recurrence rate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SourceRate {
    /// One event per cylinder per four-stroke engine cycle (crank order = cylinders * 0.5).
    PerCylinder,
    /// Explicit crankshaft order (events per crank revolution).
    Order(f32),
}

/// Specification for one impulsive mechanical sound source.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImpulsiveSpec {
    /// Recurrence rate: per-cylinder or an explicit crankshaft order.
    pub rate: SourceRate,
    /// Level of the source in the mechanical mix.
    pub level: f32,
}

impl ImpulsiveSpec {
    /// An impulsive event occurring once per cylinder per four-stroke cycle.
    pub const fn per_cylinder(level: f32) -> Self {
        Self {
            rate: SourceRate::PerCylinder,
            level,
        }
    }

    /// An impulsive event occurring at an explicit crankshaft order.
    pub const fn order(order: f32, level: f32) -> Self {
        Self {
            rate: SourceRate::Order(order),
            level,
        }
    }
}

/// Configuration for the mechanical noise rig: which sources exist, their orders and levels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MechanicalSpec {
    /// Intake valve seating event: lighter valve, brighter ring.
    pub intake_valve: Option<ImpulsiveSpec>,
    /// Exhaust valve seating event: heavier valve, hotter, duller ring.
    pub exhaust_valve: Option<ImpulsiveSpec>,
    /// Piston slap: transverse skirt impact against the bore at TDC compression.
    pub piston_slap: Option<ImpulsiveSpec>,
    /// Injector tick: high-frequency click from solenoid or direct injector.
    pub injector: Option<ImpulsiveSpec>,
    /// Timing drive: chain pitch or belt tooth whirr.
    pub timing_chain: Option<ImpulsiveSpec>,
    /// Gear whine: tooth-mesh singing from oil pump, accessory or timing gears.
    pub gear_whine: Option<ImpulsiveSpec>,
    /// Accessory drive: non-integer order belt/pulley rumble.
    pub accessory: Option<ImpulsiveSpec>,
}

impl Default for MechanicalSpec {
    fn default() -> Self {
        Self {
            intake_valve: Some(ImpulsiveSpec::per_cylinder(0.50)),
            exhaust_valve: Some(ImpulsiveSpec::per_cylinder(0.50)),
            piston_slap: Some(ImpulsiveSpec::per_cylinder(0.40)),
            injector: Some(ImpulsiveSpec::per_cylinder(0.35)),
            timing_chain: Some(ImpulsiveSpec::order(19.0, 0.25)),
            gear_whine: Some(ImpulsiveSpec::order(31.0, 0.20)),
            accessory: Some(ImpulsiveSpec::order(1.37, 0.20)),
        }
    }
}

impl MechanicalSpec {
    /// Mechanical spec for a rotary engine: no valves, no reciprocating pistons,
    /// but phasing gears, eccentric shaft drive, oil pump gear whine and accessories.
    pub fn rotary() -> Self {
        Self {
            intake_valve: None,
            exhaust_valve: None,
            piston_slap: None,
            injector: Some(ImpulsiveSpec::per_cylinder(0.35)),
            timing_chain: None,
            gear_whine: Some(ImpulsiveSpec::order(3.0, 0.30)),
            accessory: Some(ImpulsiveSpec::order(1.37, 0.20)),
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
    /// Exhaust system geometry: primaries, collector, crossover, and silencers.
    pub exhaust: ExhaustSystem,
    /// Intake system geometry: runners, plenum, throttle, airbox, and snorkel.
    pub intake: IntakeSystem,
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
    /// Mass and geometry the block's modes are derived from.
    ///
    /// The structural path is a second radiator, not a filter on the exhaust:
    /// combustion, the mechanical rig and knock all reach the listener through
    /// it. See [`crate::audio::structure`].
    pub structure: StructuralSpec,
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
    /// `backfire_level` is above the others on purpose: a pop is an unmetered
    /// charge lighting off in open pipe, and it is genuinely larger than an
    /// ordinary blowdown. It is not as far above them as it once was, because
    /// a pop is broadband and the mouth radiates the top of that band far
    /// better than it radiates a firing fundamental — see
    /// [`radiation`](crate::audio::radiation). Against a tailpipe that tilts,
    /// the old figure put every overrun through the clipper hard enough to
    /// slew. Set where the whole catalogue renders clean with headroom to
    /// spare.
    pub exhaust_level: f64,
    pub intake_level: f64,
    pub backfire_level: f64,
    /// Level of the valvetrain and bearing noise floor [-].
    ///
    /// Measured at the *block*, not at the bus: the rig drives the structural
    /// path and reaches the listener only through it, so this number is the
    /// mechanical force against the combustion force the same structure is
    /// carrying, and it is not comparable to [`Self::intake_level`].
    pub mechanical_level: f64,
    /// Level of the structural path in the final mix [-].
    ///
    /// Scales what the block radiates against what the pipes do. The balance
    /// *within* the path is not set here — each source arrives at the bank
    /// already normalised against its own physical reference — so this is one
    /// number for how loud the engine's own casing is, and nothing else.
    pub structure_level: f64,
    /// Mechanical rig configuration: which impulsive sources exist, their orders and levels.
    pub mechanical: MechanicalSpec,
    /// Gain applied to the summed bus before the soft clipper [-].
    ///
    /// Lower than it was, because the bus it is scaling now has a far wider
    /// dynamic range to carry. A tailpipe radiates the rate of change of what
    /// reaches it, so an engine at its limiter is genuinely much louder than
    /// the same engine idling — 13 dB apart on a four-cylinder, against barely
    /// one when a flat lowpass stood in for the mouth. That spread is real and
    /// it has to fit under full scale, so the idle it leaves is quiet on
    /// purpose: the listener's volume control is the right place to answer
    /// that, and a bus gain that lets the redline slew is not.
    ///
    /// Set by measurement, not by ear: the largest value at which every preset
    /// in the catalogue renders every fixed profile clean — no clipping and no
    /// slew — with headroom in hand.
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
        let exhaust = ExhaustSystem::default_for_cylinders(n, banks);
        let intake = IntakeSystem::default_for_cylinders(n);
        Self {
            sample_rate,
            cylinders: taps,
            bank_count: banks,
            exhaust,
            intake,
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
            structure: StructuralSpec::default(),
            // A healthy warm engine: COV of IMEP around 3 % where the lope
            // starts and 8 % at idle. A tired one would run higher, and this is
            // the knob that models it.
            combustion_variation_min: 0.03,
            combustion_variation_max: 0.08,
            combustion_variation_idle_rpm: 700.0,
            waveguide_damping: 1.0,
            exhaust_level: 1.0,
            intake_level: 0.35,
            backfire_level: 0.4,
            // Calibrated to sit about 8 dB under the exhaust at idle: audible
            // in the gaps between firings, which is its whole job, without
            // becoming the thing the engine sounds like. The rig itself sits at
            // unity RMS, so this is the force it puts into the block relative to
            // the combustion drive alongside it.
            mechanical_level: 0.37,
            // Set by measurement against the catalogue: the largest value at
            // which the block is clearly present in the bottom octave at idle
            // without becoming the thing the engine sounds like.
            structure_level: 0.07,
            mechanical: MechanicalSpec::default(),
            master_gain: 0.20,
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

    /// Attaches a mechanical noise rig configuration.
    pub fn with_mechanical(mut self, mechanical: MechanicalSpec) -> Self {
        self.mechanical = mechanical;
        self
    }
}

// ---------------------------------------------------------------------------
// Backfire excitation
// ---------------------------------------------------------------------------

/// One in-flight backfire pop.
///
/// A pop is unburnt fuel finding enough heat to light in the pipework, and
/// unlike a blowdown it is not a cylinder event at all — there is no valve, no
/// port and no trace of it anywhere in the solver's cycle, so it is the one
/// exhaust excitation that still has to be synthesised. The envelope is the
/// product of two exponentials — a near-instant rise as the charge goes off,
/// and a fall set by how much of it there was — evaluated recursively so a
/// running pop costs two multiplies and an add. The waveform under it is almost
/// entirely broadband, which is what an unmetered charge burning in open pipe
/// radiates.
#[derive(Debug, Clone, Copy, Default)]
struct Pop {
    amplitude: f32,
    noise_depth: f32,
    attack_coeff: f32,
    decay_coeff: f32,
    attack_state: f32,
    decay_state: f32,
    active: bool,
}

impl Pop {
    /// Starts a pop, `age` samples into its own envelope.
    ///
    /// The fractional `age` is what buys sample-accurate onsets. A pop is polled
    /// for at the control rate but does not begin on a control boundary, and
    /// quantising to one would put every pop in the engine on a 32-sample grid —
    /// which a rapid string of them during a shift cut would make audible as a
    /// buzz at the control rate. Advancing the envelope analytically to its true
    /// starting point removes it.
    fn trigger(
        &mut self,
        sample_rate: f32,
        amplitude: f32,
        attack_seconds: f32,
        decay_seconds: f32,
        noise_depth: f32,
        age: f32,
    ) {
        let ta = attack_seconds.max(1.0 / sample_rate);
        let td = decay_seconds.max(2.0 / sample_rate);
        self.attack_coeff = 1.0 - (-1.0 / (ta * sample_rate)).exp();
        self.decay_coeff = (-1.0 / (td * sample_rate)).exp();

        // Peak of (1 - e^-t/ta) e^-t/td, so pops of different lengths hit the
        // same level for the same severity. Without this, a short pop would be
        // quiet purely because its envelope never has time to rise.
        let ratio = ta / (ta + td);
        let peak = (td / (ta + td)) * ratio.powf(ta / td);

        self.amplitude = amplitude / peak.max(1e-3);
        self.noise_depth = noise_depth.clamp(0.0, 1.0);

        // `age` counts forward from the pulse's own start, so it is how far into
        // its own envelope the pulse already is on the sample it first appears.
        let age = age.clamp(0.0, 1.0);
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
        self.attack_state += (1.0 - self.attack_state) * self.attack_coeff;
        self.decay_state *= self.decay_coeff;
        // Below -100 dB the pop is inaudible and only costs cycles; retiring it
        // also keeps the slot available for the next one.
        if self.decay_state < 1e-5 {
            self.active = false;
            return 0.0;
        }
        let envelope = self.attack_state * self.decay_state;
        envelope * self.amplitude * (1.0 - self.noise_depth + self.noise_depth * noise)
    }
}

/// Fixed pool of overlapping pops.
///
/// Pops overlap whenever one is still ringing as the next lights off, which a
/// long overrun on a hot pipe does readily. Four slots covers that with margin;
/// allocating per pop is not an option in a callback, and stealing the oldest
/// slot when the pool is exhausted degrades gracefully (the pop being stolen is
/// by then the quietest one present).
#[derive(Debug, Clone, Copy, Default)]
struct PopPool {
    slots: [Pop; 4],
    next: usize,
}

impl PopPool {
    fn trigger(
        &mut self,
        sample_rate: f32,
        amplitude: f32,
        attack: f32,
        decay: f32,
        noise_depth: f32,
        age: f32,
    ) {
        // Prefer an idle slot; fall back to round-robin stealing.
        let slot = self
            .slots
            .iter()
            .position(|s| !s.active)
            .unwrap_or(self.next);
        self.next = (self.next + 1) % self.slots.len();
        self.slots[slot].trigger(sample_rate, amplitude, attack, decay, noise_depth, age);
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
        self.slots = [Pop::default(); 4];
        self.next = 0;
    }
}

// ---------------------------------------------------------------------------
// Phase-resolved excitation
// ---------------------------------------------------------------------------

/// The solver's own cycle, ready to play back at crank rate.
///
/// # Why a table and not an envelope
///
/// A blowdown is not a shape anyone gets to choose. It is the cylinder emptying
/// through a valve that is itself opening, and how fast that happens depends on
/// port area, lift rate, bore and the pressure ratio across the seat. A
/// two-exponential envelope can be made to *look* like one, but only for the
/// engine its time constants were picked on: the same constants on a long-duration
/// cam or a peripheral-port rotary describe an event that engine never has. The
/// solver already computes the real curve 720 times a cycle, and this is it.
///
/// # What the pipe is driven with
///
/// ```text
/// shape(phi) = ( P_cyl(phi) - P_manifold )  *  |mdot_exh(phi)| / max |mdot_exh|
/// ```
///
/// normalised so the magnitude of its own peak is one. The pressure difference
/// is the driver — a pipe end is pushed by the gas behind it being at a higher
/// pressure than the gas in it, and pulled when it is lower — and the port flow
/// is the *window* that driver acts through: gas moves through the port exactly
/// when the valve is off its seat, so the magnitude of the flow is where the
/// window is open and its rise and fall are the valve's own ramps. The crack at
/// EVO is therefore as fast as the cam makes it and no faster.
///
/// Both signs matter. Late in overlap the pipe can stand above the cylinder and
/// push gas back through the port; that is a rarefaction leaving the valve, not
/// an absence of one, and clamping it away would leave a corner in the
/// excitation where a real engine has a smooth reversal.
///
/// Only the flow's *shape* is used, which is why it is normalised to its own
/// peak: its magnitude is already in the pressure difference, and counting it
/// twice would make the pulse grow as the square of the load.
///
/// The peak-one normalisation is what keeps the excitation model's one
/// quantitative claim intact. Amplitude stays strictly
/// `blowdown_delta / REFERENCE_BLOWDOWN`, exactly as it was when the envelope was
/// parametric; all that has changed is that the curve under it is the solver's
/// instead of an approximation of it.
#[derive(Debug, Clone, Copy)]
struct CycleTables {
    /// Exhaust excitation shape over the cycle from EVO, peak one [-].
    exhaust: [f32; CYCLE_TABLE],
    /// Induction mass flow through one cylinder's intake port [kg/s].
    ///
    /// Kept in its own units rather than normalised, because the intake layer's
    /// level law is a function of port *velocity* and therefore of the flow
    /// itself — see [`REFERENCE_INTAKE_FLOW`]. Summing this over the cylinders
    /// at their own phases is the induction gulp train, which is what makes an
    /// intake pitched rather than merely noisy.
    intake: [f32; CYCLE_TABLE],
    /// Rate of change of cylinder pressure per unit of cycle [Pa].
    ///
    /// `dP/dtheta` in the cycle's own units, so multiplying by the cycle rate
    /// gives `dP/dt` in Pa/s. This is the quantity the block hears: the piston
    /// and the head are pushed apart by `A P`, and a force that arrives quickly
    /// puts its energy where a stiff lump of iron will answer, while the same
    /// force arriving slowly does not. Combustion noise is this table.
    pressure_slope: [f32; CYCLE_TABLE],
}

impl Default for CycleTables {
    /// A stopped engine: nothing to play.
    fn default() -> Self {
        Self {
            exhaust: [0.0; CYCLE_TABLE],
            intake: [0.0; CYCLE_TABLE],
            pressure_slope: [0.0; CYCLE_TABLE],
        }
    }
}

impl CycleTables {
    /// Derives the playable curves from one physics frame.
    ///
    /// Runs at the head of the callback, once per snapshot — a few hundred
    /// multiplies at the physics frame rate, against the several thousand per
    /// *sample* the network costs. It allocates nothing and branches on nothing
    /// that depends on the signal.
    fn from_snapshot(snapshot: &EngineSnapshot) -> Self {
        let mut peak_flow = 0.0f32;
        for &flow in snapshot.exhaust_port_flow.iter() {
            peak_flow = peak_flow.max(flow.abs());
        }

        let mut exhaust = [0.0f32; CYCLE_TABLE];
        if peak_flow > 0.0 {
            let manifold = snapshot.exhaust_manifold_pressure;
            let mut peak = 0.0f32;
            for (k, out) in exhaust.iter_mut().enumerate() {
                let open = snapshot.exhaust_port_flow[k].abs() / peak_flow;
                let excess = snapshot.cylinder_pressure[k] - manifold;
                *out = open * excess;
                peak = peak.max(out.abs());
            }
            if peak > 0.0 {
                for out in exhaust.iter_mut() {
                    *out /= peak;
                }
            }
        }
        // Central difference on the master pressure trace. The table wraps, so
        // the derivative does too — the compression stroke's rise is continuous
        // with the previous cycle's expansion, and there is no seam at EVO.
        let mut pressure_slope = [0.0f32; CYCLE_TABLE];
        for (k, slope) in pressure_slope.iter_mut().enumerate() {
            let next = snapshot.cylinder_pressure[(k + 1) % CYCLE_TABLE];
            let previous = snapshot.cylinder_pressure[(k + CYCLE_TABLE - 1) % CYCLE_TABLE];
            *slope = 0.5 * (next - previous) * CYCLE_TABLE as f32;
        }

        Self {
            exhaust,
            intake: snapshot.intake_port_flow,
            pressure_slope,
        }
    }

    /// Reads a table at a cycle phase, `0..1` from EVO.
    ///
    /// Linearly interpolated between the two points either side, and offset by
    /// half a point because each point is the mean of the span it covers and
    /// therefore stands at that span's centre. Reading on the point boundaries
    /// instead would advance every event in the cycle by 2.8 crank degrees.
    #[inline(always)]
    fn read(table: &[f32; CYCLE_TABLE], phase: f32) -> f32 {
        let x = phase.rem_euclid(1.0) * CYCLE_TABLE as f32 - 0.5;
        let floor = x.floor();
        let frac = x - floor;
        let i = (floor as i32).rem_euclid(CYCLE_TABLE as i32) as usize;
        let a = table[i];
        let b = table[(i + 1) % CYCLE_TABLE];
        a + (b - a) * frac
    }

    /// Exhaust excitation at a cycle phase measured from EVO.
    #[inline(always)]
    fn exhaust_at(&self, phase: f32) -> f32 {
        Self::read(&self.exhaust, phase)
    }

    /// Intake port mass flow at a cycle phase measured from EVO [kg/s].
    #[inline(always)]
    fn intake_at(&self, phase: f32) -> f32 {
        Self::read(&self.intake, phase)
    }

    /// Cylinder pressure slope at a cycle phase measured from EVO [Pa/cycle].
    #[inline(always)]
    fn pressure_slope_at(&self, phase: f32) -> f32 {
        Self::read(&self.pressure_slope, phase)
    }
}

/// The last two cycles the physics sent, and the crossfade between them.
///
/// A snapshot is a *sample* of a cycle that is itself changing — the charge
/// warms, the manifold backs up, the cam gets swept past its tuned length — and
/// it arrives at whatever cadence the physics loop happens to run at. Swapping
/// the table outright on arrival would put a step into the excitation at that
/// cadence: at 240 Hz, a 240 Hz buzz on every pulse in the engine, modulated by
/// how hard the driver is working the throttle. That is precisely the
/// frame-rate artefact the firing path already goes to some trouble to avoid,
/// and it would arrive here by the back door.
///
/// So each new cycle is faded in rather than swapped in. The fade's starting
/// point is the curve actually being played at the instant the new frame lands,
/// baked down so an early arrival cannot step either — which makes the scheme
/// independent of the physics cadence rather than tuned to one.
#[derive(Debug, Clone)]
struct CyclePlayer {
    /// The curve being faded out of.
    previous: CycleTables,
    /// The curve most recently handed over.
    current: CycleTables,
    /// How far the fade has got, `0..1` [-].
    ///
    /// Ten milliseconds is two to three physics frames at the rate the sim
    /// runs, which is long enough to bridge the arrivals and short enough that
    /// the shape still follows a throttle stab. It is a glide on the pulse
    /// *shape*; amplitude has its own, faster one.
    blend: Smoothed,
}

impl CyclePlayer {
    fn new(sample_rate: f32) -> Self {
        Self {
            previous: CycleTables::default(),
            current: CycleTables::default(),
            blend: Smoothed::new(1.0, sample_rate, 0.010),
        }
    }

    /// Takes a new cycle, starting the fade from wherever the last one got to.
    fn accept(&mut self, snapshot: &EngineSnapshot) {
        let blend = self.blend.value();
        for k in 0..CYCLE_TABLE {
            let p = &mut self.previous;
            let c = &self.current;
            p.exhaust[k] += (c.exhaust[k] - p.exhaust[k]) * blend;
            p.intake[k] += (c.intake[k] - p.intake[k]) * blend;
            p.pressure_slope[k] += (c.pressure_slope[k] - p.pressure_slope[k]) * blend;
        }
        self.current = CycleTables::from_snapshot(snapshot);
        self.blend.snap(0.0);
        self.blend.set_target(1.0);
    }

    /// Advances the fade by one sample and returns where it stands.
    #[inline(always)]
    fn advance(&mut self) -> f32 {
        self.blend.next_value()
    }

    /// Exhaust excitation at a cycle phase measured from EVO.
    #[inline(always)]
    fn exhaust_at(&self, phase: f32, blend: f32) -> f32 {
        let a = self.previous.exhaust_at(phase);
        let b = self.current.exhaust_at(phase);
        a + (b - a) * blend
    }

    /// Intake port mass flow at a cycle phase measured from EVO [kg/s].
    #[inline(always)]
    fn intake_at(&self, phase: f32, blend: f32) -> f32 {
        let a = self.previous.intake_at(phase);
        let b = self.current.intake_at(phase);
        a + (b - a) * blend
    }

    /// Cylinder pressure slope at a cycle phase measured from EVO [Pa/cycle].
    #[inline(always)]
    fn pressure_slope_at(&self, phase: f32, blend: f32) -> f32 {
        let a = self.previous.pressure_slope_at(phase);
        let b = self.current.pressure_slope_at(phase);
        a + (b - a) * blend
    }

    /// Drops both cycles and stops mid-fade.
    fn reset(&mut self) {
        self.previous = CycleTables::default();
        self.current = CycleTables::default();
        self.blend.snap(1.0);
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
    /// Signed offset on the phase this cylinder reads the cycle at [-].
    ///
    /// Zero-mean, in cycle fraction. A slower burn peaks later and is still at
    /// higher pressure when the valve cracks, so the acoustic event has moved in
    /// crank angle; shifting the read phase is that, and nothing else is left of
    /// the timing model now that the shape comes from the solver.
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
///
/// The offset is signed, and applied to the phase a cylinder reads the cycle at.
/// It used to have to be a non-negative *delay* on a scheduled pulse, which
/// meant carrying a mean retard for it to vary either side of; there is no
/// schedule to delay any more, so the retard has gone and only the variation —
/// which is the audible half, and the only stochastic one — is left.
const CCV_MAX_PHASE_FRACTION: f32 = 0.3;

/// Widest amplitude excursion allowed, as a fraction of nominal [-].
const CCV_MAX_AMPLITUDE_EXCURSION: f32 = 0.75;

// ---------------------------------------------------------------------------
// Exhaust bank
// ---------------------------------------------------------------------------

/// One bank's place in the stereo image, and how many cylinders feed it.
///
/// The acoustics used to live here — a lumped runner and a muffler per bank.
/// They are now in [`ExhaustNetwork`], which holds one primary per cylinder and
/// a real collector, so what is left of a bank is where its tailpipe sits
/// relative to the listener.
#[derive(Debug, Clone, Copy)]
struct ExhaustBank {
    pan_left: f32,
    pan_right: f32,
    /// How many cylinders exhaust into this bank.
    ///
    /// Held here because it is the denominator of the Transit-Time Decision
    /// Rule's `tau_interval`: a bank's pulse train is what its pipes see, and
    /// on a cross-plane V8 the two banks do not even receive evenly spaced ones.
    cylinder_count: usize,
}

impl ExhaustBank {
    fn new(config: &SynthConfig, index: usize) -> Self {
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
            pan_left: angle.cos(),
            pan_right: angle.sin(),
            cylinder_count: config
                .cylinders
                .iter()
                .filter(|tap| tap.bank == index)
                .count(),
        }
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
/// [`SynthConfig::mechanical_level`] is a force the block is driven with rather
/// than an artefact of the filtering. Measured, not derived — see
/// `mechanical_layers_sit_at_unity`.
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

/// Control law governing how a mechanical source's gain scales with engine state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LevelLaw {
    /// Cam lash and spring force: constant base + moderate friction scaling.
    /// Does not vanish on spark cut.
    CamLash { base_ratio: f32 },
    /// Piston slap: scales directly with peak cylinder pressure, and vanishes on spark cut.
    PeakPressure { reference_pressure: f32 },
    /// Scales directly with friction mean effective pressure.
    FrictionMep,
    /// Scales with crankshaft speed: `rpm / reference_rpm`.
    Speed { reference_rpm: f32 },
    /// Constant level whenever the engine is turning.
    Constant,
}

impl LevelLaw {
    /// Evaluates the gain multiplier given engine snapshot and cycle rate.
    pub fn compute(&self, snapshot: &EngineSnapshot, cycle_hz: f32) -> f32 {
        if cycle_hz < 1e-3 || snapshot.rpm < 1.0 {
            return 0.0;
        }
        match *self {
            LevelLaw::Constant => 1.0,
            LevelLaw::CamLash { base_ratio } => {
                let drag = (snapshot.friction_mep / REFERENCE_FMEP).clamp(0.0, 2.0);
                base_ratio + (1.0 - base_ratio) * drag
            }
            LevelLaw::PeakPressure { reference_pressure } => {
                if snapshot.spark_cut || snapshot.peak_cylinder_pressure <= 0.0 {
                    0.0
                } else {
                    (snapshot.peak_cylinder_pressure / reference_pressure.max(1e3)).max(0.0)
                }
            }
            LevelLaw::FrictionMep => (snapshot.friction_mep / REFERENCE_FMEP).clamp(0.0, 3.0),
            LevelLaw::Speed { reference_rpm } => {
                (snapshot.rpm / reference_rpm.max(100.0)).clamp(0.0, 3.0)
            }
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
    pub fn tune(&mut self, snapshot: &EngineSnapshot, cycle_hz: f32, cylinders: usize) {
        let hz = self.effective_hz(cycle_hz, cylinders);
        self.event_hz.set_target(hz);

        let law_gain = self.level_law.compute(snapshot, cycle_hz);
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
    pub intake_valve: Option<ImpulsiveSource>,
    pub exhaust_valve: Option<ImpulsiveSource>,
    pub piston_slap: Option<ImpulsiveSource>,
    pub injector: Option<ImpulsiveSource>,
    pub timing_chain: Option<ImpulsiveSource>,
    pub gear_whine: Option<ImpulsiveSource>,
    pub accessory: Option<ImpulsiveSource>,
    rumble_a: OnePole,
    rumble_b: OnePole,
    pub click_gain: Smoothed,
    pub rumble_gain: Smoothed,
    pub event_hz: Smoothed,
    pub click_phase: f32,
    sample_rate: f32,
}

pub type MechanicalRig = MechanicalVoice;

impl MechanicalVoice {
    /// Builds a mechanical noise rig from a specification.
    pub fn from_spec(spec: &MechanicalSpec, sample_rate: f32) -> Self {
        let intake_valve = spec.intake_valve.map(|s| {
            let mut src = ImpulsiveSource::new(
                sample_rate,
                s.rate,
                0.40,
                0.0005,
                ModalBank::single(sample_rate, 3_800.0, 1.4),
                LevelLaw::CamLash { base_ratio: 0.45 },
                s.level,
            );
            src.phase = 0.0;
            src
        });

        let exhaust_valve = spec.exhaust_valve.map(|s| {
            let mut src = ImpulsiveSource::new(
                sample_rate,
                s.rate,
                0.40,
                0.0008,
                ModalBank::single(sample_rate, 2_600.0, 1.1),
                LevelLaw::CamLash { base_ratio: 0.45 },
                s.level,
            );
            src.phase = 0.5;
            src
        });

        let piston_slap = spec.piston_slap.map(|s| {
            let mut src = ImpulsiveSource::new(
                sample_rate,
                s.rate,
                0.35,
                0.0030,
                ModalBank::dual(sample_rate, (480.0, 1.6, 0.7), (950.0, 2.0, 0.3)),
                LevelLaw::PeakPressure {
                    reference_pressure: REFERENCE_PEAK_PRESSURE,
                },
                s.level,
            );
            src.phase = 0.25;
            src
        });

        let injector = spec.injector.map(|s| {
            let mut src = ImpulsiveSource::new(
                sample_rate,
                s.rate,
                0.25,
                0.0003,
                ModalBank::single(sample_rate, 4_200.0, 3.5),
                LevelLaw::Constant,
                s.level,
            );
            src.phase = 0.15;
            src
        });

        let timing_chain = spec.timing_chain.map(|s| {
            let mut src = ImpulsiveSource::new(
                sample_rate,
                s.rate,
                0.15,
                0.0010,
                ModalBank::single(sample_rate, 1_600.0, 2.2),
                LevelLaw::FrictionMep,
                s.level,
            );
            src.phase = 0.35;
            src
        });

        let gear_whine = spec.gear_whine.map(|s| {
            let mut src = ImpulsiveSource::new(
                sample_rate,
                s.rate,
                0.05,
                0.0015,
                ModalBank::dual(sample_rate, (2_200.0, 10.0, 0.8), (4_400.0, 12.0, 0.2)),
                LevelLaw::Speed {
                    reference_rpm: 3_000.0,
                },
                s.level,
            );
            src.phase = 0.45;
            src
        });

        let accessory = spec.accessory.map(|s| {
            let mut src = ImpulsiveSource::new(
                sample_rate,
                s.rate,
                0.20,
                0.0020,
                ModalBank::single(sample_rate, 1_100.0, 2.8),
                LevelLaw::Speed {
                    reference_rpm: 1_000.0,
                },
                s.level,
            );
            src.phase = 0.60;
            src
        });

        Self {
            intake_valve,
            exhaust_valve,
            piston_slap,
            injector,
            timing_chain,
            gear_whine,
            accessory,
            rumble_a: OnePole::new(sample_rate, MECHANICAL_RUMBLE_HZ),
            rumble_b: OnePole::new(sample_rate, MECHANICAL_RUMBLE_HZ),
            click_gain: Smoothed::new(0.0, sample_rate, 0.040),
            rumble_gain: Smoothed::new(0.0, sample_rate, 0.040),
            event_hz: Smoothed::new(0.0, sample_rate, 0.030),
            click_phase: 0.0,
            sample_rate,
        }
    }

    /// Builds a mechanical noise rig with default four-stroke sources.
    pub fn new(sample_rate: f32) -> Self {
        Self::from_spec(&MechanicalSpec::default(), sample_rate)
    }

    /// Retunes from engine snapshot, cycle rate [Hz] and cylinder count.
    fn tune(&mut self, snapshot: &EngineSnapshot, cycle_hz: f32, cylinders: usize) {
        let drag = (snapshot.friction_mep / REFERENCE_FMEP).clamp(0.0, 1.5);

        if let Some(s) = &mut self.intake_valve {
            s.tune(snapshot, cycle_hz, cylinders);
        }
        if let Some(s) = &mut self.exhaust_valve {
            s.tune(snapshot, cycle_hz, cylinders);
        }
        if let Some(s) = &mut self.piston_slap {
            s.tune(snapshot, cycle_hz, cylinders);
        }
        if let Some(s) = &mut self.injector {
            s.tune(snapshot, cycle_hz, cylinders);
        }
        if let Some(s) = &mut self.timing_chain {
            s.tune(snapshot, cycle_hz, cylinders);
        }
        if let Some(s) = &mut self.gear_whine {
            s.tune(snapshot, cycle_hz, cylinders);
        }
        if let Some(s) = &mut self.accessory {
            s.tune(snapshot, cycle_hz, cylinders);
        }

        let hz = cycle_hz * cylinders.max(1) as f32 * VALVE_EVENTS_PER_CYLINDER;
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

        let increment = event_hz / self.sample_rate;
        if (1e-9..1.0).contains(&increment) {
            self.click_phase += increment;
            if self.click_phase >= 1.0 {
                self.click_phase -= 1.0;
            }
        }

        let mut clicks = 0.0f32;
        if let Some(s) = &mut self.intake_valve {
            clicks += s.process(noise);
        }
        if let Some(s) = &mut self.exhaust_valve {
            clicks += s.process(noise);
        }
        if let Some(s) = &mut self.piston_slap {
            clicks += s.process(noise);
        }
        if let Some(s) = &mut self.injector {
            clicks += s.process(noise);
        }
        if let Some(s) = &mut self.timing_chain {
            clicks += s.process(noise);
        }
        if let Some(s) = &mut self.gear_whine {
            clicks += s.process(noise);
        }
        if let Some(s) = &mut self.accessory {
            clicks += s.process(noise);
        }

        let rumble = self
            .rumble_b
            .process(self.rumble_a.process(noise.next_bipolar()))
            * MECHANICAL_RUMBLE_MAKEUP
            * rumble_gain;

        clicks * click_gain * 0.35 + rumble * 0.65
    }

    fn reset(&mut self) {
        if let Some(s) = &mut self.intake_valve {
            s.reset();
        }
        if let Some(s) = &mut self.exhaust_valve {
            s.reset();
        }
        if let Some(s) = &mut self.piston_slap {
            s.reset();
        }
        if let Some(s) = &mut self.injector {
            s.reset();
        }
        if let Some(s) = &mut self.timing_chain {
            s.reset();
        }
        if let Some(s) = &mut self.gear_whine {
            s.reset();
        }
        if let Some(s) = &mut self.accessory {
            s.reset();
        }
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
    /// The pipes themselves: one primary per cylinder, a collector per bank,
    /// the crossover, the silencer chain, the tailpipes and their mouths.
    network: ExhaustNetwork,
    /// The solver's cycle, played back at crank rate.
    ///
    /// One set for the whole engine, because the block runs one master cylinder
    /// and every other cylinder is that same cycle displaced in phase — which is
    /// exactly how each of them reads it here.
    cycle: CyclePlayer,
    /// Excitation presented to the network this sample, one per cylinder.
    excitations: Vec<f32>,
    /// One backfire pool per bank. A backfire is unburnt fuel lighting off in
    /// the pipework, so it is a bank event and fires into the collector rather
    /// than down any one cylinder's primary.
    backfire_pulses: Vec<PopPool>,
    /// Excitation presented to each bank's collector this sample.
    bank_excitations: Vec<f32>,
    /// Pressure radiated from each bank's mouth this sample.
    radiated: Vec<f32>,
    intake_network: IntakeNetwork,
    intake_excitations: Vec<f32>,
    prev_cylinder_intake_flow: Vec<f32>,
    turbo: TurboVoice,
    backfire: BackfireVoice,
    mechanical: MechanicalVoice,
    knock: KnockVoice,

    /// Master-cycle phase over 720 crank degrees, as a fraction of [`PHASE_ONE`].
    ///
    /// Held fixed-point and wrapping rather than as a `0..1` float; see the
    /// constant for why.
    phase_fixed: u32,
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
    /// The block as a radiating body: modes of the casting, pan and bore walls.
    structure: StructuralPath,
    /// `drive = scale * dP/dt`, with the bore area folded in [s/Pa].
    ///
    /// Fixed geometry, so the area ratio and both references are collapsed into
    /// one multiply at construction rather than run per sample.
    combustion_scale: f32,
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
        let banks: Vec<ExhaustBank> = (0..config.bank_count)
            .map(|i| ExhaustBank::new(&config, i))
            .collect();
        let network = ExhaustNetwork::new(
            &config.exhaust,
            &config.cylinders,
            config.bank_count,
            fs,
            &snapshot,
        );
        let n_cyl = config.cylinders.len();
        let blowdown_pa = (0..config.cylinders.len())
            .map(|_| Smoothed::new(0.0, fs, 0.005))
            .collect();

        let mut synth = Self {
            banks,
            network,
            cycle: CyclePlayer::new(fs),
            excitations: vec![0.0; n_cyl],
            backfire_pulses: vec![PopPool::default(); config.bank_count],
            bank_excitations: vec![0.0; config.bank_count],
            radiated: vec![0.0; config.bank_count],
            intake_network: IntakeNetwork::new(&config.intake, config.cylinders.len(), fs),
            intake_excitations: vec![0.0; n_cyl],
            prev_cylinder_intake_flow: vec![0.0; n_cyl],
            turbo: TurboVoice::new(fs),
            backfire: BackfireVoice::new(fs),
            mechanical: MechanicalVoice::from_spec(&config.mechanical, fs),
            knock: KnockVoice::new(fs),
            variation: vec![CycleVariation::default(); config.cylinders.len()],
            variation_depth: 0.0,
            phase_fixed: 0,
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
            structure: StructuralPath::new(fs, &config.structure.modes()),
            combustion_scale: combustion_drive(1.0, config.structure.bore as f32),
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

    /// Mean acoustic round-trip time of a bank's primaries [s].
    pub fn runner_round_trip_seconds(&self, bank_idx: usize) -> f32 {
        self.network.bank_mean_round_trip_seconds(bank_idx)
    }

    /// Reference to the intake waveguide network.
    pub fn intake_network(&self) -> &IntakeNetwork {
        &self.intake_network
    }

    /// Mutable reference to the intake waveguide network.
    pub fn intake_network_mut(&mut self) -> &mut IntakeNetwork {
        &mut self.intake_network
    }

    /// Excitation presented to the intake network this sample, per cylinder [Pa].
    pub fn intake_excitations(&self) -> &[f32] {
        &self.intake_excitations
    }

    /// Master-cycle phase, `0..1` over 720 crank degrees.
    #[inline(always)]
    fn cycle_phase(&self) -> f32 {
        self.phase_fixed as f32 / PHASE_ONE
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
        self.intake_network.set_throttle(snapshot.throttle);
        self.knock.set_intensity(snapshot.knock_intensity);
        self.cycle.accept(&snapshot);
    }

    /// Clears every filter and delay line without changing parameters.
    pub fn reset(&mut self) {
        self.network.reset();
        self.intake_network.reset();
        for pool in self.backfire_pulses.iter_mut() {
            pool.reset();
        }
        self.cycle.reset();
        self.excitations.fill(0.0);
        self.intake_excitations.fill(0.0);
        self.prev_cylinder_intake_flow.fill(0.0);
        self.bank_excitations.fill(0.0);
        self.radiated.fill(0.0);
        for smoother in self.blowdown_pa.iter_mut() {
            smoother.snap(0.0);
        }
        self.turbo.reset();
        self.mechanical.reset();
        self.knock.reset();
        self.dc = [DcBlocker::default(); 2];
        self.block.reset();
        self.structure.reset();
        self.phase_fixed = 0;
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
        self.intake_flow.advance(block);
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

        self.network.tune(gamma, gas_constant, temperature);
        // The Transit-Time Decision Rule, applied where the reflection it is
        // talking about actually happens: the closed valve at the head of each
        // primary. Every cylinder has its own primary now, so each one gets the
        // rule computed against the pulse train its own bank receives.
        let damping_strength = self.config.waveguide_damping as f32;
        for (cylinder, tap) in self.config.cylinders.iter().enumerate() {
            let tau_interval = firing_interval_seconds(rpm, self.banks[tap.bank].cylinder_count);
            let tau_pulse = self.network.primary_round_trip_seconds(cylinder);
            let damping =
                waveguide_damping(tau_interval, tau_pulse) * damping_strength.clamp(0.0, 1.0);
            self.network.set_valve_damping(cylinder, damping);
        }
        self.variation_depth = self.variation_depth_at(rpm);
        self.block.tune(rpm);
        self.mechanical
            .tune(&self.snapshot, cycle_hz, self.config.cylinders.len());
        self.intake_network.set_throttle(throttle);
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
            let bank = (self.noise.next_u32() as usize) % self.backfire_pulses.len();
            let amplitude = severity * self.config.backfire_level as f32;
            // Backfires are almost entirely broadband and much longer than a
            // blowdown crack; they share the runner and muffler so they pick up
            // the same pipe colour.
            self.backfire_pulses[bank].trigger(
                self.config.sample_rate,
                amplitude,
                0.0004,
                decay,
                0.92,
                self.noise.next_unit(),
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
    ///
    /// Returns whether the crank actually turned this sample. A stopped engine
    /// has no phase to read the cycle at, and holding the last one would present
    /// the pipes with a frozen slice of the blowdown as a DC offset.
    #[inline(always)]
    fn advance_crank(&mut self, cycle_hz: f32) -> bool {
        let nominal_omega = 4.0 * std::f32::consts::PI * cycle_hz;
        if nominal_omega <= 1e-6 {
            self.crank_omega_delta = 0.0;
            return false;
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
                let phi = (self.cycle_phase() - (tap.evo_phase - 0.25)).rem_euclid(1.0);
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
            return false;
        }

        let previous = self.cycle_phase();
        let depth = self.variation_depth;

        for index in 0..self.config.cylinders.len() {
            let tap = self.config.cylinders[index];
            let alive = self.blowdown_pa[index].value() > 1.0;
            // Distance from the previous phase forward to this cylinder's EVO,
            // wrapped into [0, 1). Crossing it is no longer the start of a
            // synthesised pulse — the excitation plays continuously now — but it
            // is still the once-per-cycle instant this cylinder's combustion
            // happens at, and so it is when the knock voice is struck.
            let ahead = (tap.evo_phase - previous).rem_euclid(1.0);
            if ahead < increment && alive {
                self.knock.trigger();
            }
            // The next cycle's draw is taken half a cycle *away* from the event
            // it governs, where the curve is flat. Drawing it at the event
            // instead would move the phase a cylinder is reading its own
            // blowdown at while that blowdown is happening, which can walk the
            // edge back over a threshold it has already passed — the same
            // double-firing hazard the old delay-scheduled jitter was built to
            // avoid, in its new form.
            let quiet = (tap.evo_phase + 0.5).rem_euclid(1.0);
            if (quiet - previous).rem_euclid(1.0) < increment && alive {
                self.reroll_variation(index, depth);
            }
        }

        self.phase_fixed = self
            .phase_fixed
            .wrapping_add((increment * PHASE_ONE) as u32);
        true
    }

    /// Reads each cylinder's excitation out of the cycle at its own phase.
    ///
    /// Every cylinder plays the same curve, displaced by where its EVO sits in
    /// the master cycle — which is the audio-side statement of the same thing
    /// the phase ring does on the physics side.
    ///
    /// Returns the structural drive the whole engine is delivering this sample:
    /// `dP/dtheta` summed over the cylinders at their own phases, in the cycle's
    /// own units. Every cylinder hammers the same block, so unlike the exhaust —
    /// which has one primary per cylinder — this is a single scalar, and the
    /// firing order is in it by construction.
    #[inline(always)]
    fn fill_excitations(&mut self, turning: bool) -> f32 {
        // Advanced whether or not the crank is, so a fade cannot be left
        // half-finished by a stopped engine and resume when it restarts.
        let blend = self.cycle.advance();
        if !turning {
            self.excitations.fill(0.0);
            self.intake_excitations.fill(0.0);
            return 0.0;
        }
        let phase = self.cycle_phase();
        let mut rise = 0.0;
        let c_intake = crate::audio::filters::speed_of_sound(
            crate::audio::intake_voice::INTAKE_AIR_GAMMA,
            crate::audio::intake_voice::INTAKE_GAS_CONSTANT,
            crate::audio::intake_voice::INTAKE_AMBIENT_TEMPERATURE_K,
        );
        for index in 0..self.config.cylinders.len() {
            let tap = self.config.cylinders[index];
            let variation = self.variation[index];
            // Strictly linear in the pressure difference, per the excitation
            // model; the curve it multiplies is normalised to a peak of one.
            let amplitude = (self.blowdown_pa[index].value() / REFERENCE_BLOWDOWN).min(2.0)
                * variation.amplitude_scale;
            let cylinder = phase - tap.evo_phase;
            let combustion = cylinder - variation.phase_offset;
            self.excitations[index] = amplitude * self.cycle.exhaust_at(combustion, blend);
            // The same retard applies to the structural path, and for the same
            // reason: a cycle whose flame took longer to develop reaches the
            // block late as well as reaching the port late.
            rise += self.cycle.pressure_slope_at(combustion, blend) * variation.amplitude_scale;
            // Induction is read at the bare cylinder phase. The retard and the
            // jitter are properties of *combustion* — how long the flame takes
            // to develop, and how much that varies — and a valve opening on the
            // intake side does not wait for a flame.
            let flow = self.cycle.intake_at(cylinder, blend);

            let prev_flow = self.prev_cylinder_intake_flow[index];
            let d_flow_dt = (flow - prev_flow) * self.config.sample_rate;
            self.prev_cylinder_intake_flow[index] = flow;

            let (runner_len, runner_area) = if !self.config.intake.runners.is_empty() {
                let r = &self.config.intake.runners[index % self.config.intake.runners.len()];
                (r.length as f32, r.area as f32)
            } else {
                (
                    self.config.intake.runner_length() as f32,
                    self.config.intake.runner_area() as f32,
                )
            };
            let p_rarefaction =
                crate::audio::intake_voice::induction_rarefaction_pa(flow, runner_area, c_intake);
            let p_slam =
                crate::audio::intake_voice::valve_slam_pa(d_flow_dt, runner_len, runner_area);
            let flow_ratio = (flow / REFERENCE_INTAKE_FLOW).clamp(0.0, 1.6);
            let p_orifice = 1_000.0
                * crate::audio::intake_voice::dipole_orifice_amplitude(flow_ratio)
                * self.noise.next_bipolar();
            self.intake_excitations[index] = p_rarefaction + p_slam + p_orifice;
        }
        rise
    }

    /// Produces one stereo frame.
    #[inline(always)]
    fn tick(&mut self) -> (f32, f32) {
        let cycle_hz = self.cycle_hz.next_value();
        let turning = self.advance_crank(cycle_hz);
        // dP/dtheta in cycles, times cycles per second, is dP/dt — so the same
        // pressure curve drives the block harder at speed, which is why
        // combustion noise climbs with rpm on every engine ever measured.
        let rise = self.fill_excitations(turning) * cycle_hz;

        let exhaust_level = self.exhaust_level.next_value();
        for (excitation, pool) in self
            .bank_excitations
            .iter_mut()
            .zip(self.backfire_pulses.iter_mut())
        {
            *excitation = pool.process(&mut self.noise);
        }
        self.network.step(
            &self.excitations,
            &self.bank_excitations,
            &mut self.radiated,
        );
        let (mut left, mut right) = (0.0f32, 0.0f32);
        for (bank, &out) in self.banks.iter().zip(self.radiated.iter()) {
            let out = out * exhaust_level;
            left += out * bank.pan_left;
            right += out * bank.pan_right;
        }

        // Intake, turbo, cylinder bore knock and the block itself are near the
        // listener's centre line and share one mono source; only the exhaust is
        // imaged, because only the exhaust has two outlets. The block is a
        // single structural object sitting between the banks and radiating a
        // four-metre wavelength at its lowest mode: there is nothing to image.
        let turbo = match self.config.turbo {
            Some(voicing) => self.turbo.process(&mut self.noise) * voicing.level as f32,
            None => 0.0,
        };
        let intake_rad =
            self.intake_network.step(&self.intake_excitations) / REFERENCE_INTAKE_PRESSURE;
        // Combustion and the mechanical rig arrive at the block as one force,
        // because the block cannot tell them apart: a lifter landing on a valve
        // and a flame front arriving at the piston crown are both metal being
        // hit, and both reach the listener only by shaking the casing. Summing
        // them before the bank rather than after is not an optimisation — it is
        // the statement that there is one structure, not two.
        let drive = rise * self.combustion_scale
            + self.mechanical.process(&mut self.noise) * self.config.mechanical_level as f32;
        let centre = intake_rad * self.config.intake_level as f32
            + turbo
            + self.knock.process(&mut self.noise)
            + self.structure.process(drive) * self.config.structure_level as f32;
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

    /// Exhaust manifold pressure the synthetic cycles below sit on top of [Pa].
    const TEST_MANIFOLD_PA: f32 = 1.2e5;

    /// A cycle in the shape the solver produces, for tests that are not about
    /// the solver.
    ///
    /// Cut at EVO like the real thing: blowdown decaying towards the manifold,
    /// the exhaust valve open for `exhaust_duration` of the cycle with a
    /// raised-cosine flow under it, induction on the following stroke, and the
    /// combustion pressure rise just before the cycle comes back round to EVO.
    ///
    /// The trace joins up across the seam, because the seam is not an event: the
    /// cylinder is at its combustion pressure when the valve cracks, and blows
    /// down from *there*. A curve that stepped 55 bar in one cell would be a
    /// cycle no engine runs, and the structural path differentiates this.
    fn synthetic_cycle(
        peak: f32,
        exhaust_duration: f32,
    ) -> ([f32; CYCLE_TABLE], [f32; CYCLE_TABLE], [f32; CYCLE_TABLE]) {
        let mut pressure = [TEST_MANIFOLD_PA; CYCLE_TABLE];
        let mut exhaust = [0.0f32; CYCLE_TABLE];
        let mut intake = [0.0f32; CYCLE_TABLE];
        // IVO 200 degrees after EVO, IVC 440 after, on the default cam.
        let (ivo, ivc) = (200.0 / 720.0, 440.0 / 720.0);
        for k in 0..CYCLE_TABLE {
            let phi = (k as f32 + 0.5) / CYCLE_TABLE as f32;
            if phi < exhaust_duration {
                let u = phi / exhaust_duration;
                pressure[k] = TEST_MANIFOLD_PA + 15.0 * peak * (-phi / 0.04).exp();
                exhaust[k] = 0.5 * (1.0 - (TAU * u).cos());
            } else if phi < ivc {
                let u = ((phi - ivo) / (ivc - ivo)).clamp(0.0, 1.0);
                intake[k] = 0.05 * 0.5 * (1.0 - (TAU * u).cos());
            } else {
                // Compression and burn, peaking a little before the valve opens.
                let u = (phi - ivc) / (1.0 - ivc);
                pressure[k] = TEST_MANIFOLD_PA + 15.0 * peak * u.powi(6);
            }
        }
        (pressure, exhaust, intake)
    }

    fn loaded_snapshot() -> EngineSnapshot {
        let (cylinder_pressure, exhaust_port_flow, intake_port_flow) =
            synthetic_cycle(4.0e5, 240.0 / 720.0);
        EngineSnapshot {
            rpm: 3_000.0,
            blowdown_delta: [4.0e5; MAX_CYLINDERS],
            exhaust_temperature: 950.0,
            exhaust_gamma: 1.33,
            exhaust_gas_constant: 287.0,
            intake_mass_flow: 0.20,
            cylinder_intake_flow: [0.20 / 8.0; MAX_CYLINDERS],
            throttle: 0.8,
            turbo_rpm: 90_000.0,
            turbo_surge: 0.0,
            unburnt_fuel_mass: 0.0,
            // Chen-Flynn on the shipped V8 at 3000 rpm under load.
            friction_mep: 1.5e5,
            spark_cut: false,
            knock_intensity: 0.0,
            bore: 0.084,
            peak_cylinder_pressure: 60.0e5,
            indicated_torque: 250.0,
            inertia: 0.25,
            cylinder_pressure,
            exhaust_port_flow,
            intake_port_flow,
            exhaust_manifold_pressure: TEST_MANIFOLD_PA,
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
    /// Amplitude of one frequency in an interleaved stereo buffer.
    fn magnitude_at(samples: &[f32], hz: f32, sample_rate: f32) -> f32 {
        let mono: Vec<f32> = samples.chunks(2).map(|f| 0.5 * (f[0] + f[1])).collect();
        let w = TAU * hz / sample_rate;
        let (mut re, mut im) = (0.0f32, 0.0f32);
        for (n, &x) in mono.iter().enumerate() {
            let phase = w * n as f32;
            re += x * phase.cos();
            im += x * phase.sin();
        }
        2.0 * (re * re + im * im).sqrt() / mono.len() as f32
    }

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
        let before = synth.cycle_phase();
        render(&mut synth, FS as usize); // exactly one second
        let cycles = 6_000.0 / 120.0;
        let expected_phase = (before + cycles).rem_euclid(1.0);
        assert!(
            (synth.cycle_phase() - expected_phase).abs() < 1e-2,
            "phase drifted: {} vs {expected_phase}",
            synth.cycle_phase()
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
            // The intake plays the cycle's own port flow now, so zeroing the
            // snapshot's mass flow no longer silences it; the level is what
            // takes it out of the mix. The structure has to go for the same
            // reason and one more: it is driven by the pressure *curve*, which
            // this test does not vary, so it would sit under both measurements
            // as a constant.
            config.intake_level = 0.0;
            config.structure_level = 0.0;
            let mut synth = EngineSynth::new(config);
            let mut snapshot = loaded_snapshot();
            snapshot.blowdown_delta = [delta; MAX_CYLINDERS];
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
    fn induction_is_a_train_of_gulps_at_the_firing_order() {
        // The intake layer used to be noise whose loudness moved with a scalar,
        // which has no rate in it at all. Reading the port flow curve at each
        // cylinder's own phase gives the layer the engine's firing order for
        // free — the gulps *are* the modulation.
        let mut config = SynthConfig::cross_plane_v8(FS);
        config.exhaust_level = 0.0;
        config.mechanical_level = 0.0;
        let mut synth = EngineSynth::new(config);
        synth.exhaust_level.snap(0.0);
        synth.set_snapshot(&loaded_snapshot());
        render(&mut synth, 24_000);
        let out = render(&mut synth, 48_000);

        // A V8 at 3000 rpm draws 200 times a second.
        let firing = 3_000.0 / 120.0 * 8.0;
        let at_firing = magnitude_at(&out, firing, FS);
        // Two frequencies either side that are not orders of anything.
        let off =
            0.5 * (magnitude_at(&out, firing * 0.63, FS) + magnitude_at(&out, firing * 1.47, FS));
        assert!(
            at_firing > 4.0 * off,
            "induction is not pitched: {at_firing:.2e} at the firing order against {off:.2e} beside it"
        );
    }

    #[test]
    fn hotter_exhaust_raises_every_resonance_by_sqrt_of_the_ratio() {
        let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
        let mut snapshot = loaded_snapshot();
        snapshot.exhaust_temperature = 400.0;
        synth.set_snapshot(&snapshot);
        render(&mut synth, 24_000);
        let cold_delay = synth.network.primary_delay_samples(0);

        snapshot.exhaust_temperature = 1_200.0;
        synth.set_snapshot(&snapshot);
        render(&mut synth, 48_000);
        let hot_delay = synth.network.primary_delay_samples(0);

        // Every transit time in the network is L / c, so tripling the absolute
        // temperature shortens all of them — and lifts every resonance built on
        // them — by sqrt(T2 / T1).
        let expected = (1_200.0f32 / 400.0).sqrt();
        assert!(
            (cold_delay / hot_delay - expected).abs() < 0.05,
            "primary: {cold_delay}/{hot_delay}, expected a factor of {expected}"
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

    /// A synth running at a fixed speed with no crank ripple and no jitter, so
    /// the only thing moving the playback phase is the integrator.
    fn steady_synth(rpm: f32) -> EngineSynth {
        let mut config = SynthConfig::cross_plane_v8(FS);
        config.combustion_variation_max = 0.0;
        config.combustion_variation_min = 0.0;
        let mut synth = EngineSynth::new(config);
        let mut snapshot = loaded_snapshot();
        snapshot.rpm = rpm;
        // No mean indicated torque means no intra-cycle speed ripple, so the
        // crank turns at exactly the nominal rate.
        snapshot.indicated_torque = 0.0;
        synth.set_snapshot(&snapshot);
        synth.cycle_hz.snap(rpm / 120.0);
        for (i, smoother) in synth.blowdown_pa.iter_mut().enumerate() {
            smoother.snap(snapshot.blowdown_delta[i]);
        }
        synth.cycle.blend.snap(1.0);
        synth.phase_fixed = 0;
        synth
    }

    #[test]
    fn excitation_playback_does_not_drift_against_the_crank() {
        // The excitation is read at a phase the synth integrates itself, one
        // sample at a time, for as long as the stream is open. A part-per-
        // million bias in that integrator is inaudible for a second and a
        // quarter of a cycle out after twenty — which is how a synth that
        // sounded right in a test ends up with the banks of a vee engine
        // walking apart on a long drive.
        let rpm = 3_000.0f32;
        let mut synth = steady_synth(rpm);

        const SAMPLES: usize = 1_000_000;
        let mut buffer = vec![0.0f32; 2 * 1_000];
        for _ in 0..SAMPLES / 1_000 {
            synth.render(&mut buffer, 2);
        }

        // Cycles turned is time times the cycle rate, exactly.
        let cycles = SAMPLES as f64 * (rpm as f64 / 120.0) / FS as f64;
        let expected = cycles.rem_euclid(1.0) as f32;
        let drift = (synth.cycle_phase() - expected).abs();
        let drift = drift.min(1.0 - drift);
        assert!(
            drift < 1e-3,
            "playback phase drifted {drift} of a cycle over {SAMPLES} samples"
        );
    }

    #[test]
    fn excitation_playback_tracks_crank_speed() {
        // One blowdown per cylinder per cycle, at whatever rate the crank is
        // turning. Nothing in the playback path sets a rate of its own.
        for rpm in [800.0f32, 3_000.0, 7_000.0] {
            let mut synth = steady_synth(rpm);
            let threshold = 0.5 * loaded_snapshot().blowdown_delta[0] / REFERENCE_BLOWDOWN;
            let seconds = 2.0;
            let samples = (seconds * FS) as usize;

            let mut buffer = [0.0f32; 2];
            synth.render(&mut buffer, 2);
            let mut above = synth.excitations[0] > threshold;
            let mut events = 0usize;
            for _ in 1..samples {
                synth.render(&mut buffer, 2);
                let now = synth.excitations[0] > threshold;
                if now && !above {
                    events += 1;
                }
                above = now;
            }

            // A four-stroke cylinder fires once per two revolutions.
            let expected = seconds * rpm / 120.0;
            assert!(
                (events as f32 - expected).abs() <= 1.0,
                "{events} blowdowns in {seconds} s at {rpm} rpm, expected {expected}"
            );
        }
    }

    #[test]
    fn a_new_cycle_fades_in_rather_than_stepping() {
        // The excitation is a *table* now, and the physics hands over a new one
        // at whatever rate its loop runs. Swapping it outright would put a step
        // into every cylinder's pulse at the physics frame rate, which is the
        // artefact the firing path exists to avoid.
        let mut config = SynthConfig::cross_plane_v8(FS);
        config.combustion_variation_max = 0.0;
        config.combustion_variation_min = 0.0;
        let mut synth = EngineSynth::new(config);
        let calm = loaded_snapshot();
        synth.set_snapshot(&calm);
        synth.cycle_hz.snap(calm.rpm / 120.0);
        for (i, smoother) in synth.blowdown_pa.iter_mut().enumerate() {
            smoother.snap(calm.blowdown_delta[i]);
        }
        synth.cycle.blend.snap(1.0);

        let cycle_samples = (FS / (calm.rpm / 120.0)) as usize;
        let mut buffer = [0.0f32; 2];
        let mut slew_of = |synth: &mut EngineSynth, samples: usize| {
            let mut worst = 0.0f32;
            let mut previous = synth.excitations[0];
            for _ in 0..samples {
                synth.render(&mut buffer, 2);
                worst = worst.max((synth.excitations[0] - previous).abs());
                previous = synth.excitations[0];
            }
            worst
        };
        // The steepest the blowdown edge itself gets: the bar every other step
        // in the excitation has to stay under.
        let steady = slew_of(&mut synth, 2 * cycle_samples);

        // A cycle off a different engine — the same pressure difference through
        // a valve event half as long.
        let (pressure, exhaust, intake) = synthetic_cycle(4.0e5, 120.0 / 720.0);
        let jumped = EngineSnapshot {
            cylinder_pressure: pressure,
            exhaust_port_flow: exhaust,
            intake_port_flow: intake,
            ..calm
        };

        // Hand the two cycles over alternately at phases that walk right through
        // cylinder 0's blowdown, because the worst step a swap can make is the
        // one made while the pulse it is swapping is happening.
        let mut worst = 0.0f32;
        for k in 0..16 {
            slew_of(&mut synth, cycle_samples / 16 + 3);
            synth.set_snapshot(if k % 2 == 0 { &jumped } else { &calm });
            worst = worst.max(slew_of(&mut synth, cycle_samples / 8));
        }

        assert!(
            worst <= 1.2 * steady,
            "a new cycle stepped the excitation: {worst} against a steady {steady}"
        );
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
        // The cycle itself jumps too, and by more than any real frame could: a
        // different pressure curve through a valve event half as long. The
        // tables are the excitation now, so a step in them is a step in the
        // output unless something is bridging them.
        let (pressure, exhaust, intake) = synthetic_cycle(12.0e5, 120.0 / 720.0);
        hot.cylinder_pressure = pressure;
        hot.exhaust_port_flow = exhaust;
        hot.intake_port_flow = intake;
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

        // The excitation is played from a table now, and a table that stops
        // being refreshed is still a table. What must not happen is the
        // playback freezing on it: a held phase would present the pipes with one
        // slice of the blowdown as a DC offset, which is silence with a step in
        // front of it rather than an engine.
        let mut buffer = [0.0f32; 2];
        let (mut low, mut high) = (f32::MAX, f32::MIN);
        for _ in 0..48_000 {
            synth.render(&mut buffer, 2);
            low = low.min(synth.excitations[0]);
            high = high.max(synth.excitations[0]);
        }
        let amplitude = loaded_snapshot().blowdown_delta[0] / REFERENCE_BLOWDOWN;
        assert!(
            low < 0.05 * amplitude && high > 0.5 * amplitude,
            "starved playback stopped swinging: {low} to {high}"
        );
    }

    #[test]
    fn snapshot_stays_copy_and_free_of_indirection() {
        fn assert_copy<T: Copy>() {}
        assert_copy::<EngineSnapshot>();

        // The tables are what make this worth asserting. A `Vec` would be one
        // word here and a heap allocation everywhere else, and its `Drop` would
        // run inside the audio callback — which is the one thing the callback is
        // not allowed to do.
        assert!(!std::mem::needs_drop::<EngineSnapshot>());

        // Three cycle tables, the per-cylinder blowdown and intake flow arrays,
        // sixteen scalars and one padded bool. Every byte accounted for is a byte
        // that is not a pointer.
        let expected = 3 * CYCLE_TABLE * 4 + 2 * MAX_CYLINDERS * 4 + 16 * 4 + 4;
        assert_eq!(std::mem::size_of::<EngineSnapshot>(), expected);
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
            cylinder_intake_flow: [f32::NAN; MAX_CYLINDERS],
            throttle: f32::INFINITY,
            turbo_rpm: f32::NAN,
            turbo_surge: f32::NAN,
            unburnt_fuel_mass: f32::NAN,
            friction_mep: f32::NAN,
            spark_cut: true,
            knock_intensity: f32::NAN,
            bore: f32::NAN,
            peak_cylinder_pressure: f32::NAN,
            indicated_torque: f32::NAN,
            inertia: f32::NAN,
            cylinder_pressure: [f32::NAN; CYCLE_TABLE],
            exhaust_port_flow: [f32::NEG_INFINITY; CYCLE_TABLE],
            intake_port_flow: [f32::NAN; CYCLE_TABLE],
            exhaust_manifold_pressure: f32::NAN,
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
            synth.config.structure_level = 0.0;
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
            config.structure_level = 0.0;
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
        // The invariant firing jitter must not break. The jitter shifts the
        // phase each cylinder *reads* the cycle at; it must never make a
        // cylinder's excitation come round twice in one cycle, or skip one.
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
        synth.cycle.blend.snap(1.0);
        // Open the window in the gap between two cylinders' events rather than
        // on top of one. The jitter moves each event up to 0.0375 of a cycle
        // either way, and an event straddling the boundary would be counted at
        // both ends — an artefact of where the measurement starts, not of the
        // firing path.
        synth.phase_fixed = (0.09 * PHASE_ONE) as u32;

        let cycles = 5usize;
        let samples = (FS / (800.0 / 120.0)) as usize * cycles;
        let n = synth.config().cylinder_count();
        // Half the peak of a curve normalised to one: comfortably inside the
        // blowdown and comfortably above the exhaust stroke that follows it.
        let threshold = (idle_snapshot().blowdown_delta[0] / REFERENCE_BLOWDOWN) * 0.5;
        let mut fires = vec![0usize; n];
        let mut buffer = [0.0f32; 2];
        // Seed from the state the window actually opens in. A cylinder whose
        // blowdown is already under way at sample zero fired before the window,
        // not inside it.
        synth.render(&mut buffer, 2);
        let mut above: Vec<bool> = (0..n).map(|i| synth.excitations[i] > threshold).collect();
        for _ in 1..samples {
            synth.render(&mut buffer, 2);
            for i in 0..n {
                let now = synth.excitations[i] > threshold;
                if now && !above[i] {
                    fires[i] += 1;
                }
                above[i] = now;
            }
        }

        for (i, count) in fires.iter().enumerate() {
            assert_eq!(
                *count, cycles,
                "cylinder {i} excited {count} times over {cycles} cycles"
            );
        }
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
        // Start on the cycle the snapshot carries rather than fading into it,
        // so the measurement is not taken part-way through the fade.
        synth.cycle.blend.snap(1.0);
        synth.phase_fixed = 0;

        // A whole cycle, so both cylinders have had their turn. They play the
        // same normalised curve, so the ratio of the two peaks is the ratio of
        // the two pressure differences and nothing else.
        let cycle_samples = (FS / (800.0 / 120.0)) as usize;
        let mut buffer = [0.0f32; 2];
        let mut peaks = [0.0f32; 3];
        for _ in 0..cycle_samples {
            synth.render(&mut buffer, 2);
            for cylinder in [0usize, 2] {
                peaks[cylinder] = peaks[cylinder].max(synth.excitations[cylinder]);
            }
        }

        assert!(peaks[0] > 0.0, "cylinder 0 was never excited");
        let ratio = peaks[2] / peaks[0];
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
            // Start on the cycle the snapshot carries rather than fading into
            // it, so the measurement is not taken part-way through the fade.
            synth.cycle.blend.snap(1.0);
            synth.phase_fixed = (0.001 * PHASE_ONE) as u32;

            let expected_fires = 8 * cycles;
            // Half of each cylinder's own peak: the instant its excitation
            // crosses that on the way up is that cylinder's firing, and the
            // curve crosses it once a cycle.
            let thresholds: Vec<f32> = (0..8)
                .map(|i| 0.5 * snapshot.blowdown_delta[i] / REFERENCE_BLOWDOWN)
                .collect();
            let mut above = [false; 8];
            let mut firing_times = Vec::new();
            let mut buffer = [0.0f32; 2];
            let mut sample_idx = 0;

            // One extra, because the render starts a hair past cylinder 0's own
            // event and catches its edge part-way up. That first detection is
            // not a firing interval, it is where the measurement began.
            while firing_times.len() <= expected_fires && sample_idx < 100_000 {
                synth.render(&mut buffer, 2);
                for i in 0..8 {
                    let now = synth.excitations[i] > thresholds[i];
                    if now && !above[i] {
                        firing_times.push(sample_idx);
                    }
                    above[i] = now;
                }
                sample_idx += 1;
            }

            let intervals = firing_times[1..]
                .windows(2)
                .map(|w| (w[1] - w[0]) as f32)
                .collect();
            (intervals, firing_times.len() - 1)
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
        let idle_damping = synth.network.valve_damping(0);
        let idle_reflection = synth.network.valve_reflection(0);

        let mut fast = loaded_snapshot();
        fast.rpm = 7_000.0;
        synth.set_snapshot(&fast);
        render(&mut synth, 4 * 48_000);
        let fast_damping = synth.network.valve_damping(0);
        let fast_reflection = synth.network.valve_reflection(0);

        assert!(idle_damping > 0.85, "idle left undamped: {idle_damping}");
        assert!(fast_damping < 0.15, "top end over-damped: {fast_damping}");
        assert!(
            idle_reflection < 0.6 * fast_reflection,
            "reflection did not follow the rule: {idle_reflection} vs {fast_reflection}"
        );
    }

    #[test]
    fn the_rule_flattens_the_comb_at_idle_and_leaves_it_at_speed() {
        // The end-to-end claim. Settle the synth at a speed, take the network it
        // has actually arrived at, and measure the ripple in the magnitude
        // response from one cylinder's port to its own mouth — a comb filter
        // *is* deep regular ripple, and how deep it is is how audible the
        // flange is.
        let ripple_db = |rpm: f32, strength: f64| {
            let mut config = SynthConfig::cross_plane_v8(FS);
            config.waveguide_damping = strength;
            let mut synth = EngineSynth::new(config);
            let mut snapshot = idle_snapshot();
            snapshot.rpm = rpm;
            synth.set_snapshot(&snapshot);
            render(&mut synth, 4 * 48_000);

            let n_cyl = synth.config().cylinder_count();
            let n_banks = synth.config().bank_count;
            let (mut lo, mut hi) = (f32::MAX, 0.0f32);
            let mut f = 200.0f32;
            while f <= 2_000.0 {
                let mut network = synth.network.clone();
                let mut excitations = vec![0.0f32; n_cyl];
                let bank_excitations = vec![0.0f32; n_banks];
                let mut radiated = vec![0.0f32; n_banks];
                let (mut re, mut im) = (0.0f64, 0.0f64);
                for i in 0..12_000 {
                    let phase = TAU * f * i as f32 / FS;
                    excitations[0] = phase.sin();
                    network.step(&excitations, &bank_excitations, &mut radiated);
                    if i >= 6_000 {
                        re += radiated[0] as f64 * phase.sin() as f64;
                        im += radiated[0] as f64 * phase.cos() as f64;
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
        let fast_unruled = ripple_db(7_000.0, 0.0);

        // Measured on the network: 38.5 dB of idle ripple unruled, 27.7 dB with
        // the rule, and 38.6 dB at 7000 rpm either way. The rule is still doing
        // its job, but it can no longer flatten the response by half, and it
        // should not: most of what is left is the chamber, the tailpipe and the
        // mouth, which are the exhaust rather than an artefact of lumping it.
        // Until the solver's valve lift reaches the snapshot the head of every
        // primary is a permanently rigid end, and that is what the rule is
        // standing in for.
        assert!(
            idle_unruled - idle > 8.0,
            "the rule left the idle comb standing: {idle:.1} dB vs {idle_unruled:.1} dB unruled"
        );
        assert!(
            (fast - fast_unruled).abs() < 1.0,
            "the rule took the pipe away at speed: {fast:.1} dB vs {fast_unruled:.1} dB unruled"
        );
        assert!(
            fast > 1.2 * idle,
            "the rule damped the top end too: {fast:.1} dB at 7000 vs {idle:.1} dB at idle"
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
        let snapshot = EngineSnapshot {
            friction_mep: REFERENCE_FMEP,
            ..EngineSnapshot::default()
        };
        voice.tune(&snapshot, 0.0, 8);
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

        // The full rig at reference operating conditions also sits at unity RMS,
        // so `SynthConfig::mechanical_level` keeps its meaning when impulsive
        // sources and rumble sound together.
        let mut full_voice = MechanicalVoice::new(FS);
        let ref_snapshot = EngineSnapshot {
            rpm: 800.0,
            friction_mep: REFERENCE_FMEP,
            peak_cylinder_pressure: REFERENCE_PEAK_PRESSURE,
            spark_cut: false,
            ..EngineSnapshot::default()
        };
        let cycle_hz = 800.0 / 120.0;
        full_voice.tune(&ref_snapshot, cycle_hz, 8);
        // Settle gain smoothers to their steady-state values.
        for _ in 0..48_000 {
            full_voice.process(&mut noise);
        }

        let mut sum_sq_full = 0.0f64;
        let n_full = 192_000;
        for _ in 0..n_full {
            let y = full_voice.process(&mut noise);
            sum_sq_full += (y as f64) * (y as f64);
        }
        let full_rms = (sum_sq_full / n_full as f64).sqrt() as f32;
        assert!(
            (full_rms - 1.0).abs() < 0.05,
            "full mechanical rig RMS is out of calibration: {full_rms}"
        );
    }

    #[test]
    fn mechanical_event_rates_track_their_orders() {
        let cylinders = 8;
        let voice = MechanicalVoice::new(FS);

        for rpm in [800.0, 2400.0, 4800.0] {
            let crank_hz = (rpm / 60.0) as f32;
            let cycle_hz = (rpm / 120.0) as f32;

            let check_source = |src: &Option<ImpulsiveSource>, expected_order: f32, name: &str| {
                let s = src.as_ref().unwrap_or_else(|| panic!("missing {name}"));
                let eff_order = s.effective_order(cylinders);
                assert!(
                    (eff_order - expected_order).abs() < 1e-4,
                    "{name} effective order {eff_order} != expected {expected_order}"
                );
                let eff_hz = s.effective_hz(cycle_hz, cylinders);
                let expected_hz = expected_order * crank_hz;
                assert!(
                    (eff_hz - expected_hz).abs() < 1e-3,
                    "{name} at {rpm} rpm has rate {eff_hz} Hz != expected {expected_hz} Hz"
                );
            };

            check_source(&voice.intake_valve, 4.0, "intake_valve");
            check_source(&voice.exhaust_valve, 4.0, "exhaust_valve");
            check_source(&voice.piston_slap, 4.0, "piston_slap");
            check_source(&voice.injector, 4.0, "injector");
            check_source(&voice.timing_chain, 19.0, "timing_chain");
            check_source(&voice.gear_whine, 31.0, "gear_whine");
            check_source(&voice.accessory, 1.37, "accessory");
        }
    }

    #[test]
    fn accessory_order_is_incommensurate_with_crank() {
        let voice = MechanicalVoice::new(FS);
        let accessory = voice.accessory.expect("accessory drive must exist");
        let order = accessory.effective_order(8);

        // An integer or half-integer order repeats after 1 or 2 crank revolutions.
        // A non-integer order like 1.37 has fractional remainder after 1 and 2 revs.
        let rev1_events = order;
        let rev2_events = order * 2.0;
        assert!(
            (rev1_events - rev1_events.round()).abs() > 0.05,
            "accessory order {order} is integer"
        );
        assert!(
            (rev2_events - rev2_events.round()).abs() > 0.05,
            "accessory order {order} is commensurate with 720 deg cycle"
        );

        // Within one 720 degree cycle, events occur at distinct crank phases.
        let dtheta = 360.0 / order;
        let mut angles = Vec::new();
        let mut theta = 0.0f32;
        while theta < 720.0 {
            angles.push(theta);
            theta += dtheta;
        }
        for i in 0..angles.len() {
            for j in (i + 1)..angles.len() {
                let diff = (angles[i] - angles[j]).abs();
                assert!(
                    (diff % 360.0) > 1.0,
                    "accessory events coincided at crank phase: {} and {}",
                    angles[i],
                    angles[j]
                );
            }
        }
    }

    #[test]
    fn piston_slap_scales_with_pressure_and_vanishes_on_spark_cut() {
        let voice = MechanicalVoice::new(FS);
        let slap = voice.piston_slap.as_ref().unwrap();
        let intake = voice.intake_valve.as_ref().unwrap();
        let cycle_hz = 1000.0 / 120.0;

        let snap_normal = EngineSnapshot {
            rpm: 1000.0,
            friction_mep: REFERENCE_FMEP,
            peak_cylinder_pressure: REFERENCE_PEAK_PRESSURE,
            spark_cut: false,
            ..EngineSnapshot::default()
        };
        let slap_gain_normal = slap.level_law.compute(&snap_normal, cycle_hz) * slap.base_level;
        let intake_gain_normal =
            intake.level_law.compute(&snap_normal, cycle_hz) * intake.base_level;
        assert!(slap_gain_normal > 0.0, "slap must be live when firing");
        assert!(intake_gain_normal > 0.0, "intake must be live");

        // High pressure raises piston slap proportionally
        let snap_high = EngineSnapshot {
            rpm: 1000.0,
            friction_mep: REFERENCE_FMEP,
            peak_cylinder_pressure: REFERENCE_PEAK_PRESSURE * 2.0,
            spark_cut: false,
            ..EngineSnapshot::default()
        };
        let slap_gain_high = slap.level_law.compute(&snap_high, cycle_hz) * slap.base_level;
        assert!(
            (slap_gain_high - 2.0 * slap_gain_normal).abs() < 1e-3,
            "slap should scale with peak pressure: {slap_gain_high} vs 2 * {slap_gain_normal}"
        );

        // Spark cut completely silences piston slap, but valves continue seating
        let snap_cut = EngineSnapshot {
            rpm: 1000.0,
            friction_mep: REFERENCE_FMEP,
            peak_cylinder_pressure: REFERENCE_PEAK_PRESSURE,
            spark_cut: true,
            ..EngineSnapshot::default()
        };
        let slap_gain_cut = slap.level_law.compute(&snap_cut, cycle_hz) * slap.base_level;
        let intake_gain_cut = intake.level_law.compute(&snap_cut, cycle_hz) * intake.base_level;
        assert_eq!(slap_gain_cut, 0.0, "piston slap must vanish on spark cut");
        assert!(
            intake_gain_cut > 0.0,
            "valve seatings must continue on spark cut"
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
        let snapshot = EngineSnapshot {
            friction_mep: 1.0e5,
            rpm: 800.0,
            ..EngineSnapshot::default()
        };
        voice.tune(&snapshot, cycle_hz, 8);
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
