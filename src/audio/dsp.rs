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
//!   dP/dtheta per cylinder ───────────┼──> modal block ──────> centre ──┤
//!   knock burst ──────────────────────┘    (bending, pan, bore walls)   │
//!                                                                       ▼
//!                                    DC block ─> soft clip ─> out
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
//!   than resonates. Nothing closes it down by schedule any more: the wall loss
//!   follows `alpha(f)`, the mouth sheds vorticity into its own jet, and the
//!   valve at the head of each runner opens once a cycle, so the comb is neither
//!   lossless nor standing still.
//! - Combustion at idle is a series of widely spaced events, and an engine
//!   built from combustion alone has audible *gaps*. [`MechanicalVoice`] fills
//!   them with the noise floor a real engine never stops making — radiating,
//!   as it physically must, through the block rather than through the air.
//! - The exhaust path is bandpass-like end to end and leaves out the block
//!   itself, which at idle is most of what a listener hears as size.
//!   [`crate::audio::structure`] puts it back, by radiating it rather than by
//!   equalising the pipe.
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
    Biquad, BiquadCoeffs, DcBlocker, ModalBank, Noise, OnePole, OversampledClipper, Smoothed,
};
use crate::audio::intake_voice::IntakeNetwork;
use crate::audio::propagation::{Aperture, AperturePositions, Listener, PropagationModel};
use crate::audio::structure::{combustion_drive, shaking_drive, StructuralPath, StructuralSpec};
use crate::audio::waveguide::{ExhaustNetwork, ExhaustTemperatures, InjectionNode};
use crate::physics::cylinder::{CylinderGeometry, CYCLE_ANGLE};
use crate::physics::engine_block::bank_angle;
use crate::physics::plumbing::{ExhaustSystem, IntakeSystem, ThrottleLayout};

pub use crate::physics::engine_block::CYCLE_TABLE;

/// Samples between control-rate updates.
///
/// 16 samples is ~0.33 ms at 48 kHz — well below the 1.25 ms firing interval of a
/// V12 at 8000 rpm (60 samples), ensuring control-rate schedules and filters track
/// every firing cycle without quantisation coarseness.
pub const CONTROL_BLOCK: usize = 16;

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
    /// Exhaust gas temperature at the port, before any pipe has cooled it [K].
    ///
    /// The hottest station in the system and the head of the gradient; what the
    /// individual pipe sections are at is [`Self::primary_temperature`],
    /// [`Self::collector_temperature`] and [`Self::tailpipe_temperature`].
    pub exhaust_temperature: f32,
    /// Gas temperature in each cylinder's primary runner [K], in cylinder order.
    ///
    /// Every resonance a pipe has is `c / 4L` with `c = sqrt(gamma R T)`, so
    /// this is what decides where the exhaust note sits — and it is a state,
    /// integrated from the heat the runner wall has taken, not a constant. A
    /// cold header is the better part of an octave below a hot one.
    pub primary_temperature: [f32; MAX_CYLINDERS],
    /// Gas temperature arriving at the collector and the silencers [K].
    pub collector_temperature: f32,
    /// Gas temperature in the tailpipe [K].
    pub tailpipe_temperature: f32,
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
    /// How cold the block still is, `1` at ambient and `0` on the thermostat [-].
    ///
    /// [`EngineThermal::cold_fraction`](crate::physics::thermal::EngineThermal::cold_fraction)
    /// at the boundary. A cold engine is *louder* mechanically, not quieter:
    /// the clearances a warm block closes up are open, so the skirt has further
    /// to travel before it lands on the bore and the lifter has further to fall
    /// onto the valve. That is a change in impact velocity, which is a change
    /// in the impulse, not in the filter it rings.
    ///
    /// Zero on every soaked engine, which is every engine the recorded
    /// fingerprints were measured on — so everything keyed on this is exactly
    /// neutral there, by construction rather than by calibration.
    pub cold_fraction: f32,
    /// Tooth-mesh frequency of the starter pinion against the ring gear [Hz].
    ///
    /// Zero whenever the pinion is out of mesh, which is every frame of an
    /// engine that is running. One field carries both the pitch and whether
    /// there is anything to hear, because those are the same fact: a starter
    /// stops because it is thrown out of the gear it was singing with, not
    /// because something faded it down.
    pub starter_hz: f32,
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
    /// Effective flow area of the exhaust valve over the cycle [m^2].
    ///
    /// `C_d` times the curtain area, capped at the seat — the solver's own
    /// [`ValveEvent::effective_area`](crate::physics::thermodynamics::ValveEvent::effective_area),
    /// sampled at the same phases as everything else here.
    ///
    /// This is the boundary condition at the head of every primary runner, and
    /// without it a runner is a pipe with a rigid plug in the end of it for the
    /// whole cycle. A rigid end reflects everything, which is most of the
    /// reason a header rings: the valve is the only thing that ever lets a
    /// wave out of the top of a primary, and it does so once a cycle.
    pub exhaust_valve_area: [f32; CYCLE_TABLE],
    /// Effective flow area of the intake valve over the cycle [m^2].
    ///
    /// Same quantity at the other end of the engine, and the boundary condition
    /// at the head of every intake runner. Same phase convention as
    /// [`Self::cylinder_pressure`], so index zero is EVO and the intake event
    /// sits where the cam timing puts it relative to that.
    pub intake_valve_area: [f32; CYCLE_TABLE],
    /// Volume enclosed above the piston over the cycle [m^3].
    ///
    /// The other half of the valve's boundary condition, and the half that
    /// decides what a wave arriving at an *open* valve does. A valve off its
    /// seat does not open onto free space: it opens into a closed box bounded
    /// by the piston, and a box that is short against the wavelength is a
    /// compliance. A wave long enough not to see the box compresses it and
    /// comes straight back out, so the head of the runner stays very nearly
    /// rigid at the bottom of the band however far the valve is lifted, while
    /// the gap's own inertance and this compliance ring together somewhere in
    /// the midrange and swallow what lands on them there.
    ///
    /// Taking the area alone and calling the rest of the wave transmitted is
    /// what costs an exhaust its low end: it is an anechoic termination on a
    /// pipe that in fact ends in a closed cylinder. Same phase convention as
    /// [`Self::cylinder_pressure`], so index zero is EVO — which is very nearly
    /// BDC, where the box is at its largest, and it shrinks from there as the
    /// piston comes up the bore under the open valve.
    pub cylinder_volume: [f32; CYCLE_TABLE],
    /// Mean exhaust manifold pressure across the banks [Pa].
    ///
    /// What [`Self::cylinder_pressure`] is measured against: the pipe is driven
    /// by the cylinder's excess over the gas already in it, and an absolute
    /// trace on its own would present the manifold's own standing pressure as a
    /// permanent offset for the delay lines to integrate.
    pub exhaust_manifold_pressure: f32,
    /// Whether the active exhaust cutout flap is open.
    pub exhaust_cutout: bool,
    /// Whether anti-lag is active.
    pub anti_lag: bool,
}

impl Default for EngineSnapshot {
    /// A stopped engine at ambient: silent, but with sane gas properties so the
    /// filters tune to something physical before the first frame arrives.
    fn default() -> Self {
        Self {
            rpm: 0.0,
            blowdown_delta: [0.0; MAX_CYLINDERS],
            exhaust_temperature: 300.0,
            primary_temperature: [300.0; MAX_CYLINDERS],
            collector_temperature: 300.0,
            tailpipe_temperature: 300.0,
            exhaust_gamma: 1.33,
            exhaust_gas_constant: 287.0,
            intake_mass_flow: 0.0,
            cylinder_intake_flow: [0.0; MAX_CYLINDERS],
            throttle: 0.0,
            turbo_rpm: 0.0,
            turbo_surge: 0.0,
            unburnt_fuel_mass: 0.0,
            friction_mep: 0.0,
            cold_fraction: 0.0,
            starter_hz: 0.0,
            spark_cut: false,
            knock_intensity: 0.0,
            bore: 0.084,
            peak_cylinder_pressure: 0.0,
            indicated_torque: 0.0,
            inertia: 0.25,
            cylinder_pressure: [0.0; CYCLE_TABLE],
            exhaust_port_flow: [0.0; CYCLE_TABLE],
            intake_port_flow: [0.0; CYCLE_TABLE],
            exhaust_valve_area: [0.0; CYCLE_TABLE],
            intake_valve_area: [0.0; CYCLE_TABLE],
            cylinder_volume: [1e-4; CYCLE_TABLE],
            exhaust_manifold_pressure: 101_325.0,
            exhaust_cutout: false,
            anti_lag: false,
        }
    }
}

impl EngineSnapshot {
    /// Puts every exhaust station at one temperature.
    ///
    /// For callers with no thermal model behind them — a test rig, or a
    /// snapshot assembled by hand. The physics path fills the stations
    /// individually, because a real exhaust has a gradient down it.
    pub fn with_uniform_exhaust_temperature(mut self, temperature: f32) -> Self {
        self.exhaust_temperature = temperature;
        self.primary_temperature = [temperature; MAX_CYLINDERS];
        self.collector_temperature = temperature;
        self.tailpipe_temperature = temperature;
        self
    }

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
        for t in self.primary_temperature.iter_mut() {
            if !t.is_finite() {
                *t = fallback.exhaust_temperature;
            }
            *t = t.clamp(200.0, 2_500.0);
        }
        guard!(collector_temperature, 200.0, 2_500.0);
        guard!(tailpipe_temperature, 200.0, 2_500.0);
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
        guard!(cold_fraction, 0.0, 1.0);
        guard!(starter_hz, 0.0, 20_000.0);
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
        // A valve cannot flow through a negative area, and half a square metre
        // is larger than any bore in the catalogue. A NaN here would go
        // straight into a reflection coefficient and stay in a delay line.
        for a in self
            .exhaust_valve_area
            .iter_mut()
            .chain(self.intake_valve_area.iter_mut())
        {
            if !a.is_finite() {
                *a = 0.0;
            }
            *a = a.clamp(0.0, 0.5);
        }
        // A cylinder always encloses something — it is bounded below by a
        // piston that never reaches the head. Zero here would divide into an
        // infinite stiffness at the valve and put a rigid end on a pipe that
        // has an open one. One cubic centimetre is smaller than any clearance
        // volume in the catalogue, and a hundred litres is larger than any
        // swept one.
        for v in self.cylinder_volume.iter_mut() {
            if !v.is_finite() {
                *v = 1e-4;
            }
            *v = v.clamp(1e-6, 0.1);
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

/// What a Roots or twin-screw positive displacement supercharger sounds like.
///
/// Driven directly from the crankshaft by belt or gears, so its tone frequency
/// tracks engine speed with no spool lag. The fundamental whine frequency is
/// locked to a crank order given by the pulley drive ratio multiplied by the
/// rotor lobe count.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RootsVoicing {
    /// Pulley drive ratio: supercharger rotor speed relative to crankshaft speed [-].
    pub belt_ratio: f64,
    /// Number of lobes on each rotor (typically 3 or 4 for modern twin-screw / Eaton TVS).
    pub lobes: usize,
    /// Level of the supercharger whine layer in the mix [-].
    ///
    /// Still a level and not a derivation: the solver produces no boost at all
    /// (see [`Induction`](crate::audio::Induction)), so there is no pressure
    /// ratio across this machine for its loudness to come from. What *is*
    /// derived is how the loudness moves — see [`RootsVoice::tune`].
    pub level: f64,
}

impl RootsVoicing {
    /// Order of the whine fundamental relative to crankshaft speed [-].
    pub fn order(&self) -> f64 {
        self.belt_ratio * self.lobes as f64
    }
}

impl Default for RootsVoicing {
    /// A typical 4-lobe twin-screw / TVS blower with a 2.1:1 drive ratio.
    fn default() -> Self {
        Self {
            belt_ratio: 2.1,
            lobes: 4,
            level: 0.030,
        }
    }
}

/// What a centrifugal supercharger sounds like.
///
/// A centrifugal supercharger uses an impeller like a turbocharger compressor,
/// but is driven from the crankshaft through an internal step-up gear transmission
/// and belt drive. Its impeller speed is mechanically locked to engine rpm
/// (`gear_ratio * rpm`), producing a high-frequency shaft-order whistle like a
/// turbo, but with zero spool lag.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CentrifugalVoicing {
    /// Total gear and belt step-up ratio from crankshaft to impeller shaft [-].
    /// Typically 7.0 to 12.0 for centrifugal blowers (e.g. ProCharger, Vortech).
    pub gear_ratio: f64,
    /// Order of the compressor whine relative to impeller shaft speed [-].
    pub order: f64,
    /// Level of the centrifugal supercharger in the mix [-].
    ///
    /// A level, for the same reason [`RootsVoicing::level`] is one.
    pub level: f64,
}

impl CentrifugalVoicing {
    /// Effective crank order of the whine fundamental: `gear_ratio * order`.
    pub fn crank_order(&self) -> f64 {
        self.gear_ratio * self.order
    }
}

impl Default for CentrifugalVoicing {
    /// A typical street centrifugal supercharger: ~9.2:1 step-up, order 1.8.
    fn default() -> Self {
        Self {
            gear_ratio: 9.2,
            order: 1.8,
            level: 0.024,
        }
    }
}

/// What an atmospheric blow-off / dump valve sounds like.
///
/// Distinct from compressor surge flutter: rather than periodic cyclic stall
/// pulses chuffing through the compressor inlet, a dump valve vents trapped
/// charge air into the atmosphere upon throttle lift under boost as a smooth,
/// broadband whoosh.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BlowOffVoicing {
    /// Compressor speed fraction of reference speed required to open the valve [-].
    pub threshold: f32,
    /// Decay time constant for the venting whoosh envelope [s].
    pub decay_time: f32,
    /// Center frequency of the broadband venting noise [Hz].
    pub center_hz: f32,
    /// Level of the blow-off valve in the mix [-].
    pub level: f32,
}

impl Default for BlowOffVoicing {
    /// A fast atmospheric dump valve releasing a ~2.8 kHz rush of air.
    fn default() -> Self {
        Self {
            threshold: 0.35,
            decay_time: 0.15,
            center_hz: 2_800.0,
            level: 0.045,
        }
    }
}

/// What a fluttering wastegate valve sounds like under high boost.
///
/// Under heavy throttle near peak boost, as the wastegate cracks open to regulate
/// turbine enthalpy, the valve flap rapidly hunts against exhaust gas pulsations,
/// producing a metallic fluttering rattle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WastegateVoicing {
    /// Gate hunting flutter oscillation frequency [Hz].
    pub flutter_hz: f32,
    /// Center frequency of the metallic gate rattle resonance [Hz].
    pub rattle_hz: f32,
    /// Boost threshold (fraction of reference shaft speed) to crack the gate [-].
    pub threshold: f32,
    /// Level of the wastegate chatter in the mix [-].
    pub level: f32,
}

impl Default for WastegateVoicing {
    /// A typical internal wastegate hunting at ~65 Hz with a ~1.8 kHz metallic rattle.
    fn default() -> Self {
        Self {
            flutter_hz: 65.0,
            rattle_hz: 1_800.0,
            threshold: 0.75,
            level: 0.025,
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
    /// Valve float threshold speed [rev/min].
    ///
    /// Above this speed, the valve spring can no longer keep the lifter on
    /// the cam lobe profile, causing the valve to separate and crash back
    /// down onto the seat with excessive impact velocity. If absent (`None`),
    /// defaults to [`crate::physics::cylinder::default_float_rpm`] of the
    /// engine's redline.
    pub float_rpm: Option<f32>,
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
            float_rpm: None,
        }
    }
}

impl MechanicalSpec {
    /// Mechanical spec for a direct-injection diesel: the injector dominates.
    ///
    /// A common-rail injector is a solenoid snapping a needle open against two
    /// thousand bar, several times per cycle, and the pump that feeds it is
    /// hung off the same drive. That is why a diesel idling at the kerb is
    /// audibly a box of hammers while a petrol engine idling next to it is
    /// audibly a flame. The rest follows the same logic: the valve gear is
    /// heavier because the compression it seals against is higher, so it lands
    /// harder, and the piston is bigger and running a longer skirt in a colder
    /// bore, so it slaps harder. The cam and the injection pump are driven by a
    /// gear train rather than a chain, because a chain would stretch under the
    /// torque reversals the pump puts through it — and a gear train sings.
    pub fn diesel() -> Self {
        Self {
            intake_valve: Some(ImpulsiveSpec::per_cylinder(0.55)),
            exhaust_valve: Some(ImpulsiveSpec::per_cylinder(0.65)),
            piston_slap: Some(ImpulsiveSpec::per_cylinder(0.80)),
            injector: Some(ImpulsiveSpec::per_cylinder(1.00)),
            timing_chain: None,
            gear_whine: Some(ImpulsiveSpec::order(23.0, 0.30)),
            accessory: Some(ImpulsiveSpec::order(1.37, 0.25)),
            float_rpm: None,
        }
    }

    /// Mechanical spec for a rotary engine: no valves, no reciprocating pistons,
    /// but phasing gears, eccentric shaft drive, oil pump gear whine and accessories.
    ///
    /// The phasing gears mesh between the fixed housing gear and the gear
    /// bolted to the rotor, so what a listener hears as "gear whine" is
    /// tied to the rotor turning, not to the eccentric shaft the engine's
    /// own cycle coordinates run in: a Wankel's rotor completes one
    /// revolution for every three of the shaft's (see
    /// [`RotorGeometry`](crate::physics::rotor::RotorGeometry)), so an order
    /// quoted at rotor rate has to come down by a factor of three before it
    /// is an order this rig's `crank_hz` — which is shaft rate for the
    /// two-rotor preset — can use.
    /// [`RotorGeometry::shaft_order`](crate::physics::rotor::RotorGeometry::shaft_order)
    /// is that conversion. The accessory drive is not converted: it is a
    /// belt off the nose of the eccentric shaft itself, at shaft rate
    /// already, same as on a reciprocating engine.
    pub fn rotary() -> Self {
        use crate::physics::rotor::RotorGeometry;
        Self {
            intake_valve: None,
            exhaust_valve: None,
            piston_slap: None,
            injector: Some(ImpulsiveSpec::per_cylinder(0.35)),
            timing_chain: None,
            gear_whine: Some(ImpulsiveSpec::order(RotorGeometry::shaft_order(3.0), 0.30)),
            accessory: Some(ImpulsiveSpec::order(1.37, 0.20)),
            float_rpm: None,
        }
    }

    /// Sets the valve float threshold speed [rev/min].
    pub fn with_float_rpm(mut self, float_rpm: f32) -> Self {
        self.float_rpm = Some(float_rpm);
        self
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
    /// The Roots / twin-screw supercharger, or `None` if not fitted.
    pub roots: Option<RootsVoicing>,
    /// The centrifugal supercharger, or `None` if not fitted.
    pub centrifugal: Option<CentrifugalVoicing>,
    /// Blow-off / dump valve, or `None` if not fitted.
    pub blow_off: Option<BlowOffVoicing>,
    /// Wastegate chatter, or `None` if not fitted.
    pub wastegate: Option<WastegateVoicing>,
    /// Unburnt fuel per cycle above which backfires become possible [kg].
    pub backfire_fuel_threshold: f64,
    /// Runner temperature above which backfires become possible [K].
    pub backfire_temperature_threshold: f64,
    /// Mass and geometry the block's modes are derived from.
    ///
    /// The structural path is a second radiator, not a filter on the exhaust:
    /// combustion, the mechanical rig and knock all reach the listener through
    /// it. See [`crate::audio::structure`].
    pub structure: StructuralSpec,
    /// Slider-crank geometry the reciprocating shaking force is read from.
    ///
    /// The same [`CylinderGeometry`] the physics side solves the cycle with —
    /// bore, stroke, rod length and reciprocating mass — reused directly
    /// rather than re-derived, so the audio thread's shake and the physics
    /// thread's kinematics can never drift apart into two approximations of
    /// the same slider-crank. `reciprocating_mass` at zero makes
    /// [`EngineSynth::fill_excitations`]'s shake term an exact no-op.
    pub reciprocating: CylinderGeometry,
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
    /// Level of each layer in the final mix [-].
    ///
    /// `backfire_level` is above the others on purpose: a pop is an unmetered
    /// charge lighting off in open pipe, and it is genuinely larger than an
    /// ordinary blowdown.
    ///
    /// How far above depends on what the pop is *made of*, which is why this
    /// number moved when [`Pop`] did. A mouth radiates the rate of change of
    /// what it passes — see [`radiation`](crate::audio::radiation) — so a burst
    /// of near-white noise leaves it far louder than a pressure pulse of the
    /// same amplitude that takes a millisecond to rise. Reshaping the pop from
    /// the first into the second, which is what a deflagration in a pipe
    /// actually is, cost it about half its radiated peak, and the level it was
    /// mixed at did not follow: a limiter bounce came out 1.1 times the clean
    /// engine, which is an ignition cut nobody can hear as an event. At this
    /// figure it comes out 2.1 times, the catalogue still renders clean, and
    /// the pop is a crack rather than a click.
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
    /// Level of the starter motor in the mechanical mix [-].
    ///
    /// Its own level rather than a share of
    /// [`Self::mechanical_level`], because the two are not the same kind of
    /// thing: the mechanical floor is the engine's own racket and scales with
    /// how hard it is working, while the starter is a separate machine bolted
    /// to the bellhousing that is either meshed or not. It radiates through the
    /// same casting, though, which is why it is summed into the same drive.
    pub starter_level: f64,
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
    /// Physical radiating aperture locations on the vehicle chassis [m].
    pub aperture_positions: AperturePositions,
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
            roots: None,
            centrifugal: None,
            blow_off: None,
            wastegate: None,
            // A cut charge on the shipped V8 carries 24-36 mg of fuel to the
            // exhaust, so 12 mg puts a real spark cut at 2-3x the threshold —
            // enough to crackle hard, while a partial misfire stays below it.
            backfire_fuel_threshold: 12.0e-6,
            backfire_temperature_threshold: 900.0,
            // A 4-litre iron-decked V8 with pan and accessories: 80 Hz.
            structure: StructuralSpec::default(),
            // `CylinderGeometry::default` carries the same 86x86mm slug the
            // structural bore reference is built from, mass defaulted from its
            // own bore.
            reciprocating: CylinderGeometry::default(),
            // A healthy warm engine: COV of IMEP around 3 % where the lope
            // starts and 8 % at idle. A tired one would run higher, and this is
            // the knob that models it.
            combustion_variation_min: 0.03,
            combustion_variation_max: 0.08,
            combustion_variation_idle_rpm: 700.0,
            exhaust_level: 1.0,
            intake_level: 0.35,
            backfire_level: 0.8,
            // Calibrated to sit about 8 dB under the exhaust at idle: audible
            // in the gaps between firings, which is its whole job, without
            // becoming the thing the engine sounds like. The rig itself sits at
            // unity RMS, so this is the force it puts into the block relative to
            // the combustion drive alongside it.
            mechanical_level: 0.37,
            starter_level: 0.22,
            // Set by measurement against the catalogue, T1: the largest value
            // at which the block is clearly present in the bottom octave at
            // idle without dominating a loaded exhaust. It was 0.07, set
            // before the exhaust's wall loss was calibrated against a
            // measured pipe, before the network was rebuilt to lose what a
            // real one loses, and before T5 gave the silenced presets a real
            // loss term — each of which made the exhaust louder without this
            // constant moving. By 3000 rpm under load a block driven at 0.07
            // matched the exhaust in isolation, which no running V8 does.
            structure_level: 0.05,
            mechanical: MechanicalSpec::default(),
            master_gain: 0.20,
            aperture_positions: if banks > 1 {
                AperturePositions::front_engine_dual()
            } else {
                AperturePositions::front_engine_single()
            },
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
    pub fn with_mechanical(mut self, mut mechanical: MechanicalSpec) -> Self {
        if mechanical.float_rpm.is_none() {
            mechanical.float_rpm = self.mechanical.float_rpm;
        }
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
    /// for once per control block rather than every sample, so by the time it is
    /// noticed it may already be up to a whole block old — quantising `age` to
    /// the block boundary would put every pop in the engine on a
    /// [`CONTROL_BLOCK`]-sample grid, which a rapid string of them during a
    /// shift cut would make audible as a buzz at the control rate. Advancing the
    /// envelope analytically to its true starting point removes it.
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
        // Bounded below at zero and otherwise left open: the caller knows how
        // wide the control grid it was noticed on is, not this envelope.
        let age = age.max(0.0);
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

/// Corner of the backfire pulse's bandlimiting lowpass, as a fraction of the
/// sample rate [-].
///
/// A pop's carrier is broadband noise, uncorrelated from one sample to the
/// next and therefore full-bandwidth on its own regardless of how gently the
/// envelope around it rises. Reaching the output clipper still that wide, its
/// harmonics fold back over Nyquist as the inharmonic rasp the pops were
/// measured with. A real backfire's crack lives well under 10 kHz, so 0.2 of
/// the sample rate (9.6 kHz at 48 kHz) keeps it, and the tenth-order cascade
/// below has fallen to the render's own noise floor well before 0.45 * fs.
const BACKFIRE_LOWPASS_FRACTION: f32 = 0.20;

/// Butterworth Q values for the pulse's tenth-order (five-biquad) lowpass [-].
const BACKFIRE_LOWPASS_Q: [f32; 5] = [
    0.5062,
    0.5612,
    std::f32::consts::FRAC_1_SQRT_2,
    1.1013,
    3.1962,
];

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
    /// Bandlimits the pool's combined output. Shared across slots rather than
    /// one per pop: filtering is linear, so filtering the sum of overlapping
    /// pops is identical to filtering each and summing, at a quarter of the
    /// state. Retuned, not reset, on every trigger so state carries through a
    /// stutter of overlapping pops without a click.
    bandlimit: [Biquad; 5],
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
        let cutoff = BACKFIRE_LOWPASS_FRACTION * sample_rate;
        for (filter, q) in self.bandlimit.iter_mut().zip(BACKFIRE_LOWPASS_Q) {
            filter.set_coeffs(BiquadCoeffs::lowpass(sample_rate, cutoff, q));
        }
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
        let stage1 = self.bandlimit[0].process(sum);
        let stage2 = self.bandlimit[1].process(stage1);
        let stage3 = self.bandlimit[2].process(stage2);
        let stage4 = self.bandlimit[3].process(stage3);
        self.bandlimit[4].process(stage4)
    }

    fn reset(&mut self) {
        self.slots = [Pop::default(); 4];
        self.next = 0;
        for filter in self.bandlimit.iter_mut() {
            filter.reset();
        }
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
    /// Acoustic flux this cycle's port launches at its peak flow, $c \dot m$ [N].
    ///
    /// The shape above is normalised, so this is what says how big it is: a
    /// port moving gas at $u$ launches a wave of $\rho c u$ into the runner
    /// behind it, and $\rho u$ is $\dot m / A$, so dividing this by the
    /// runner's area gives the pressure. Taken from the gas state of the
    /// snapshot this cycle was read out of rather than from the synth's glide,
    /// so that the pascal scale an excitation is played at is a property of the
    /// cycle and changes only when the cycle does.
    launch_flux: f32,
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
    /// Rate of change of intake port flow per unit of cycle [kg/s].
    ///
    /// `dmdot/dtheta` in the cycle's own units, the exact counterpart of
    /// [`Self::pressure_slope`] and for the same reason: the water hammer at
    /// valve closing is driven by how fast the column is stopped, and taking
    /// that rate by differencing the *interpolated* flow sample to sample
    /// differentiates a piecewise-linear signal. The result is a staircase —
    /// constant between table points and stepping at every one of them — and a
    /// staircase is a train of discontinuities at `CYCLE_TABLE` times the cycle
    /// rate, which is 3.2 kHz at 3000 rpm and lands most of its energy in the
    /// band the induction note is supposed to occupy. Differencing the table
    /// and interpolating *that* keeps the derivative inside the bandwidth the
    /// table actually carries.
    intake_slope: [f32; CYCLE_TABLE],
    /// Effective exhaust valve flow area over the cycle [m^2].
    ///
    /// Not an excitation: a boundary condition. This is what the head of a
    /// primary runner is terminated into, and it is the only thing that ever
    /// opens that end of the pipe.
    exhaust_valve_area: [f32; CYCLE_TABLE],
    /// Effective intake valve flow area over the cycle [m^2].
    intake_valve_area: [f32; CYCLE_TABLE],
    /// Gas crossing the exhaust port over the cycle, positive out [kg/s].
    ///
    /// [`Self::exhaust`] is this curve normalised, because what drives the pipe
    /// is a *shape*. What resists at the valve is the flow itself, in kilograms
    /// per second, so the unnormalised trace is kept alongside it.
    exhaust_flow: [f32; CYCLE_TABLE],
    /// Space enclosed above the piston over the cycle [m^3].
    cylinder_volume: [f32; CYCLE_TABLE],
}

impl Default for CycleTables {
    /// A stopped engine: nothing to play.
    fn default() -> Self {
        Self {
            exhaust: [0.0; CYCLE_TABLE],
            launch_flux: 0.0,
            intake: [0.0; CYCLE_TABLE],
            pressure_slope: [0.0; CYCLE_TABLE],
            intake_slope: [0.0; CYCLE_TABLE],
            exhaust_valve_area: [0.0; CYCLE_TABLE],
            intake_valve_area: [0.0; CYCLE_TABLE],
            exhaust_flow: [0.0; CYCLE_TABLE],
            cylinder_volume: [1e-4; CYCLE_TABLE],
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

        // Central difference on the induction trace, wrapping like the cycle.
        let mut intake_slope = [0.0f32; CYCLE_TABLE];
        for (k, slope) in intake_slope.iter_mut().enumerate() {
            let next = snapshot.intake_port_flow[(k + 1) % CYCLE_TABLE];
            let previous = snapshot.intake_port_flow[(k + CYCLE_TABLE - 1) % CYCLE_TABLE];
            *slope = 0.5 * (next - previous) * CYCLE_TABLE as f32;
        }

        Self {
            exhaust,
            launch_flux: crate::audio::filters::speed_of_sound(
                snapshot.exhaust_gamma,
                snapshot.exhaust_gas_constant,
                snapshot.exhaust_temperature,
            ) * peak_flow,
            intake: snapshot.intake_port_flow,
            pressure_slope,
            intake_slope,
            exhaust_valve_area: snapshot.exhaust_valve_area,
            intake_valve_area: snapshot.intake_valve_area,
            exhaust_flow: snapshot.exhaust_port_flow,
            cylinder_volume: snapshot.cylinder_volume,
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

    /// Intake port flow slope at a cycle phase measured from EVO [kg/s/cycle].
    #[inline(always)]
    fn intake_slope_at(&self, phase: f32) -> f32 {
        Self::read(&self.intake_slope, phase)
    }

    /// Effective exhaust valve area at a cycle phase measured from EVO [m^2].
    #[inline(always)]
    fn exhaust_valve_area_at(&self, phase: f32) -> f32 {
        Self::read(&self.exhaust_valve_area, phase)
    }

    /// Effective intake valve area at a cycle phase measured from EVO [m^2].
    #[inline(always)]
    fn intake_valve_area_at(&self, phase: f32) -> f32 {
        Self::read(&self.intake_valve_area, phase)
    }

    /// Exhaust port mass flow at a cycle phase measured from EVO [kg/s].
    #[inline(always)]
    fn exhaust_flow_at(&self, phase: f32) -> f32 {
        Self::read(&self.exhaust_flow, phase)
    }

    /// Cylinder volume at a cycle phase measured from EVO [m^3].
    #[inline(always)]
    fn cylinder_volume_at(&self, phase: f32) -> f32 {
        Self::read(&self.cylinder_volume, phase)
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
        self.previous.launch_flux += (self.current.launch_flux - self.previous.launch_flux) * blend;
        self.current = CycleTables::from_snapshot(snapshot);
        self.blend.snap(0.0);
        self.blend.set_target(1.0);
    }

    /// Advances the fade by one sample and returns where it stands.
    #[inline(always)]
    fn advance(&mut self) -> f32 {
        self.blend.next_value()
    }

    /// Acoustic flux the cycle being played launches at its peak flow [N].
    #[inline(always)]
    fn launch_flux(&self, blend: f32) -> f32 {
        let a = self.previous.launch_flux;
        let b = self.current.launch_flux;
        a + (b - a) * blend
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

    /// Intake port flow slope at a cycle phase measured from EVO [kg/s/cycle].
    #[inline(always)]
    fn intake_slope_at(&self, phase: f32, blend: f32) -> f32 {
        let a = self.previous.intake_slope_at(phase);
        let b = self.current.intake_slope_at(phase);
        a + (b - a) * blend
    }

    /// Effective exhaust valve area at a cycle phase measured from EVO [m^2].
    #[inline(always)]
    fn exhaust_valve_area_at(&self, phase: f32, blend: f32) -> f32 {
        let a = self.previous.exhaust_valve_area_at(phase);
        let b = self.current.exhaust_valve_area_at(phase);
        a + (b - a) * blend
    }

    /// Effective intake valve area at a cycle phase measured from EVO [m^2].
    #[inline(always)]
    fn intake_valve_area_at(&self, phase: f32, blend: f32) -> f32 {
        let a = self.previous.intake_valve_area_at(phase);
        let b = self.current.intake_valve_area_at(phase);
        a + (b - a) * blend
    }

    /// Exhaust port mass flow at a cycle phase measured from EVO [kg/s].
    #[inline(always)]
    fn exhaust_flow_at(&self, phase: f32, blend: f32) -> f32 {
        let a = self.previous.exhaust_flow_at(phase);
        let b = self.current.exhaust_flow_at(phase);
        a + (b - a) * blend
    }

    /// Cylinder volume at a cycle phase measured from EVO [m^3].
    #[inline(always)]
    fn cylinder_volume_at(&self, phase: f32, blend: f32) -> f32 {
        let a = self.previous.cylinder_volume_at(phase);
        let b = self.current.cylinder_volume_at(phase);
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

/// Scaling of valve impact amplitude with float overspeed [1 / (rev/min)].
///
/// Above the valve float threshold speed, the valve separates from the cam
/// profile, flies over the crest, and re-seats under spring force at velocity
/// higher than the cam ramp intended. The impact impulse scales monotonically
/// with overspeed `(rpm - float_rpm)`.
const FLOAT_GAIN_PER_RPM: f32 = 0.005;

/// Scaling of valve impact resonant frequency (brightness) with float overspeed [1 / (rev/min)].
///
/// Higher impact velocity sharpens the contact pulse, shifting the spectral
/// centroid of the metallic seating ring upward into a brighter, harsher
/// clatter.
const FLOAT_FREQ_PER_RPM: f32 = 0.0006;

/// Extra piston slap on a stone-cold engine, as a fraction of the warm level [-].
///
/// A piston is aluminium in an iron bore and expands at twice the rate, so the
/// clearance it runs in is at its widest when the engine is coldest. The skirt
/// has further to travel before the side thrust at TDC lands it on the wall,
/// so it arrives faster and hits harder — and doubling the impulse is what a
/// cold engine's slap actually does. It goes away as the piston grows into the
/// bore, not as a timer runs out, which is why it is keyed on
/// [`EngineSnapshot::cold_fraction`].
const COLD_PISTON_SLAP_RISE: f32 = 1.00;

/// Extra valve seating impact on a stone-cold engine [-].
///
/// The same mechanism one deck up: valve lash is set for a hot head, so a cold
/// one runs loose, and a hydraulic lifter that has bled down overnight runs
/// looser still. Smaller than the slap because the clearance change is a
/// fraction of a millimetre against the skirt's several hundredths.
const COLD_TAPPET_RISE: f32 = 0.45;

/// Extra injector tick on a stone-cold engine [-].
///
/// Least of the three, and for a different reason: the needle's clearances
/// barely move, but the cold-start pulse is longer and the fuel behind it
/// denser, so the needle lands on its stop harder at both ends of the event.
const COLD_INJECTOR_RISE: f32 = 0.25;

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
    pub base_frequency: f32,
    pub base_q: f32,
    pub current_frequency: f32,
    pub float_rpm: Option<f32>,
    /// Extra level at [`EngineSnapshot::cold_fraction`] of 1, as a fraction [-].
    ///
    /// Zero on every source whose loudness has nothing to do with running
    /// clearances, and therefore zero for the whole rig on a warm engine —
    /// the multiplier is `1 + cold_rise * cold_fraction`, which is exactly one
    /// when either term is zero, so nothing that was measured warm moves.
    pub cold_rise: f32,
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
            base_frequency: 0.0,
            base_q: 0.0,
            current_frequency: 0.0,
            float_rpm: None,
            cold_rise: 0.0,
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

        // Clearances close as the block grows into itself, so this is a scaling
        // on the impulse the source delivers, not on the body it rings. Exactly
        // one on a warm engine, by construction: see [`Self::cold_rise`].
        let law_gain = self.level_law.compute(snapshot, cycle_hz)
            * (1.0 + self.cold_rise * snapshot.cold_fraction);

        let overspeed = match self.float_rpm {
            Some(threshold) if snapshot.rpm > threshold => snapshot.rpm - threshold,
            _ => 0.0,
        };

        if overspeed > 0.0 {
            let amp_scale = 1.0 + FLOAT_GAIN_PER_RPM * overspeed;
            self.gain.set_target(self.base_level * law_gain * amp_scale);

            if self.base_frequency > 0.0 {
                let float_freq = (self.base_frequency * (1.0 + FLOAT_FREQ_PER_RPM * overspeed))
                    .min(self.sample_rate * 0.45);
                self.body
                    .retune(self.sample_rate, 0, float_freq, self.base_q);
                self.current_frequency = float_freq;
            }
        } else {
            self.gain.set_target(self.base_level * law_gain);
            if self.current_frequency != self.base_frequency && self.base_frequency > 0.0 {
                self.body
                    .retune(self.sample_rate, 0, self.base_frequency, self.base_q);
                self.current_frequency = self.base_frequency;
            }
        }
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
            src.cold_rise = COLD_TAPPET_RISE;
            src.phase = 0.0;
            src.base_frequency = 3_800.0;
            src.base_q = 1.4;
            src.current_frequency = 3_800.0;
            src.float_rpm = spec.float_rpm;
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
            src.cold_rise = COLD_TAPPET_RISE;
            src.phase = 0.5;
            src.base_frequency = 2_600.0;
            src.base_q = 1.1;
            src.current_frequency = 2_600.0;
            src.float_rpm = spec.float_rpm;
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
            src.cold_rise = COLD_PISTON_SLAP_RISE;
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
            src.cold_rise = COLD_INJECTOR_RISE;
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

/// Level of the second mesh harmonic relative to the first [-].
///
/// A straight-cut pinion driving a straight-cut ring gear has a contact ratio
/// barely over one, so the tooth-to-tooth handover is nearly a discontinuity
/// and the mesh tone is anything but a sine. The second harmonic carries most
/// of what makes it read as gear rather than as tone.
const STARTER_MESH_SECOND: f32 = 0.55;

/// Level of the third mesh harmonic relative to the first [-].
const STARTER_MESH_THIRD: f32 = 0.25;

/// Brush and commutator hash, relative to the mesh tone [-].
///
/// The other half of the sound, and the half that says it is a motor rather
/// than a gearbox: a series-wound armature with a segmented commutator is an
/// arc being made and broken tens of times a revolution. Broadband, bandpassed
/// where the motor casing radiates.
const STARTER_BRUSH_LEVEL: f32 = 0.45;

/// Where the motor casing puts the brush hash [Hz].
const STARTER_BRUSH_HZ: f32 = 2_200.0;

/// Glide on the whine's amplitude [s].
///
/// Short, because the pinion coming out of mesh is a mechanical event and not a
/// fade — but not zero, because a gain that steps mid-cycle is a click, and the
/// one thing a release must not sound like is an edit.
const STARTER_GATE_SECONDS: f32 = 0.012;

/// Level the release is declared finished at [-].
///
/// An exponential glide approaches zero and never arrives, and "never arrives"
/// means a voice that is inaudible but still summing a sine into the block for
/// the rest of the session. Eighty decibels down is over, so it is snapped
/// there and the voice stops being asked for samples at all.
const STARTER_GATE_FLOOR: f32 = 1.0e-4;

/// The starter motor, heard rather than felt.
///
/// Two sources, both of them driven by the one number the physics sends across:
/// the pinion's tooth-mesh frequency, which is zero when the pinion is out.
///
/// - **Mesh whine.** Three harmonics of the tooth-mesh rate. It is a gear tone
///   and it tracks the crank exactly, because while the pinion is in mesh the
///   armature is geared to the crankshaft and has no speed of its own — which
///   is why the whine rises and falls with every compression stroke the engine
///   drags itself over, without anything modulating it.
/// - **Brush hash.** Bandpassed noise at the casing's own frequency, gated the
///   same way.
///
/// Nothing here knows what a starting sequence is. The voice sings while it is
/// handed a frequency and stops when it is handed a zero, and the physics
/// decides which.
#[derive(Debug, Clone)]
pub struct StarterVoice {
    phase: f32,
    hz: Smoothed,
    gain: Smoothed,
    brush: Biquad,
    sample_rate: f32,
}

impl StarterVoice {
    pub fn new(sample_rate: f32) -> Self {
        Self {
            phase: 0.0,
            hz: Smoothed::new(0.0, sample_rate, 0.010),
            gain: Smoothed::new(0.0, sample_rate, STARTER_GATE_SECONDS),
            brush: Biquad::new(BiquadCoeffs::bandpass(sample_rate, STARTER_BRUSH_HZ, 1.2)),
            sample_rate,
        }
    }

    /// Points the voice at this frame's mesh frequency [Hz].
    ///
    /// A zero is a pinion out of mesh, and it silences the voice rather than
    /// running it at nought hertz.
    pub fn tune(&mut self, mesh_hz: f32) {
        let meshed = mesh_hz > 1.0 && mesh_hz < self.sample_rate * 0.45;
        self.hz
            .set_target(if meshed { mesh_hz } else { self.hz.value() });
        self.gain.set_target(if meshed { 1.0 } else { 0.0 });
    }

    /// Whether the voice is currently making any sound at all.
    pub fn is_singing(&self) -> bool {
        self.gain.value() > STARTER_GATE_FLOOR || self.gain.target() > STARTER_GATE_FLOOR
    }

    #[inline(always)]
    pub fn process(&mut self, noise: &mut Noise) -> f32 {
        let gain = self.gain.next_value();
        let hz = self.hz.next_value();
        if gain <= STARTER_GATE_FLOOR {
            if self.gain.target() <= 0.0 {
                self.gain.snap(0.0);
            }
            // Still advance the filter's own state so a re-engagement does not
            // start from whatever was left in it a minute ago.
            self.brush.process(0.0);
            return 0.0;
        }

        let increment = hz / self.sample_rate;
        if (1e-9..0.5).contains(&increment) {
            self.phase += increment;
            if self.phase >= 1.0 {
                self.phase -= 1.0;
            }
        }
        let turn = TAU * self.phase;
        let mesh = turn.sin()
            + STARTER_MESH_SECOND * (2.0 * turn).sin()
            + STARTER_MESH_THIRD * (3.0 * turn).sin();
        let brush = self.brush.process(noise.next_bipolar()) * STARTER_BRUSH_LEVEL;
        (mesh + brush) * gain
    }

    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.gain.snap(0.0);
        self.hz.snap(0.0);
        self.brush.reset();
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

/// Scaling that normalises [`RootsVoice`] to roughly unity RMS at reference speed.
const ROOTS_UNITY_RMS: f32 = 1.326;

/// Roots / twin-screw supercharger whine voice.
///
/// Driven directly from the crankshaft by belt or gears, so its tone frequency
/// tracks engine speed with no spool lag. The whine fundamental is locked to
/// the crank order: belt drive ratio times rotor lobe count.
#[derive(Debug, Clone)]
struct RootsVoice {
    phase: f32,
    gain: Smoothed,
    tone_hz: Smoothed,
    sample_rate: f32,
}

impl RootsVoice {
    fn new(sample_rate: f32) -> Self {
        Self {
            phase: 0.0,
            gain: Smoothed::new(0.0, sample_rate, 0.020),
            tone_hz: Smoothed::new(0.0, sample_rate, 0.015),
            sample_rate,
        }
    }

    /// Retunes to the shaft and the gas it is moving.
    ///
    /// The pitch is the crank order the pulley and the lobe count give. The
    /// loudness is the *flow*: a displacement blower's noise is its rotors
    /// handing pockets of gas to the discharge port, and a volume velocity $Q$
    /// crossing an aperture launches $\rho c Q / A$, which in mass flow is
    /// $c \dot m / A$ — linear, like every other aperture in this synth.
    ///
    /// It replaces a law that went as the square of *speed* times a throttle
    /// term. Speed and throttle were standing in for flow, badly: a blower
    /// spinning fast on a shut throttle is pumping almost nothing round a
    /// bypass and should be almost silent, and that law had it at a third of
    /// full voice. What is left undercided is the absolute level, and it stays
    /// undecided until something in the solver produces boost.
    fn tune(&mut self, voicing: &RootsVoicing, rpm: f32, intake_mass_flow: f32) {
        let crank_hz = (rpm / 60.0).max(0.0);
        let tone = crank_hz * voicing.order() as f32;
        // Anti-alias limit: keep 3rd harmonic below Nyquist.
        self.tone_hz.set_target(tone.min(0.15 * self.sample_rate));
        self.gain
            .set_target((intake_mass_flow / REFERENCE_INTAKE_FLOW).clamp(0.0, 1.5));
    }

    #[inline(always)]
    fn process(&mut self) -> f32 {
        let gain = self.gain.next_value();
        let tone_hz = self.tone_hz.next_value();

        self.phase += tone_hz / self.sample_rate;
        if self.phase >= 1.0 {
            self.phase -= self.phase.floor();
        }
        let w = TAU * self.phase;
        let whine = w.sin() + 0.35 * (2.0 * w).sin() + 0.12 * (3.0 * w).sin();
        whine * gain * ROOTS_UNITY_RMS
    }

    fn reset(&mut self) {
        self.phase = 0.0;
        self.gain.snap(0.0);
        self.tone_hz.snap(0.0);
    }
}

/// Scaling that normalises [`CentrifugalVoice`] to roughly unity RMS at reference speed.
const CENTRIFUGAL_UNITY_RMS: f32 = 1.380;

/// Centrifugal supercharger voice.
///
/// An impeller driven from the crankshaft through a high-ratio internal step-up
/// gear transmission and belt. The whistle tracks impeller shaft order, but is
/// belt-locked to the crank with zero spool lag.
#[derive(Debug, Clone)]
struct CentrifugalVoice {
    phase: f32,
    gain: Smoothed,
    tone_hz: Smoothed,
    sample_rate: f32,
}

impl CentrifugalVoice {
    fn new(sample_rate: f32) -> Self {
        Self {
            phase: 0.0,
            gain: Smoothed::new(0.0, sample_rate, 0.020),
            tone_hz: Smoothed::new(0.0, sample_rate, 0.015),
            sample_rate,
        }
    }

    /// Retunes to the impeller and the gas it is moving.
    ///
    /// Belt-locked, so the pitch is a crank order like the Roots'. The loudness
    /// is the flow through it, for the reason given at [`RootsVoice::tune`] —
    /// an impeller passing no gas makes no noise however fast the belt is
    /// turning it.
    fn tune(&mut self, voicing: &CentrifugalVoicing, rpm: f32, intake_mass_flow: f32) {
        let crank_hz = (rpm / 60.0).max(0.0);
        let tone = crank_hz * voicing.crank_order() as f32;
        // Keep second harmonic below Nyquist.
        self.tone_hz.set_target(tone.min(0.22 * self.sample_rate));
        self.gain
            .set_target((intake_mass_flow / REFERENCE_INTAKE_FLOW).clamp(0.0, 1.5));
    }

    #[inline(always)]
    fn process(&mut self) -> f32 {
        let gain = self.gain.next_value();
        let tone_hz = self.tone_hz.next_value();

        self.phase += tone_hz / self.sample_rate;
        if self.phase >= 1.0 {
            self.phase -= self.phase.floor();
        }
        let w = TAU * self.phase;
        let whistle = w.sin() + 0.22 * (2.0 * w).sin() + 0.05 * (0.5 * w).sin();
        whistle * gain * CENTRIFUGAL_UNITY_RMS
    }

    fn reset(&mut self) {
        self.phase = 0.0;
        self.gain.snap(0.0);
        self.tone_hz.snap(0.0);
    }
}

/// Scaling that normalises [`BlowOffVoice`] to roughly unity RMS during peak venting.
const BLOW_OFF_UNITY_RMS: f32 = 2.8;

/// Atmospheric blow-off / dump valve voice.
///
/// Venting triggered when the driver lifts off the throttle with boost pressure
/// in the charge tract. Emits a smooth broadband whoosh that decays exponentially,
/// distinct from cyclic compressor surge flutter.
#[derive(Debug, Clone)]
struct BlowOffVoice {
    envelope: f32,
    decay_rate: f32,
    filter: Biquad,
    prev_throttle: f32,
    sample_rate: f32,
}

impl BlowOffVoice {
    fn new(sample_rate: f32) -> Self {
        Self {
            envelope: 0.0,
            decay_rate: (-1.0 / (0.15 * sample_rate)).exp(),
            filter: Biquad::new(BiquadCoeffs::bandpass(sample_rate, 2_800.0, 1.2)),
            prev_throttle: 0.0,
            sample_rate,
        }
    }

    fn tune(&mut self, voicing: &BlowOffVoicing, throttle: f32, turbo_rpm: f32, ref_rpm: f32) {
        let boost_ratio = (turbo_rpm / ref_rpm.max(1.0)).clamp(0.0, 1.5);
        // Lift condition: throttle was significantly open and is now closed.
        let is_lift = self.prev_throttle > 0.30 && throttle < 0.15;
        if is_lift && boost_ratio > voicing.threshold {
            // Valve snaps open with intensity proportional to trapped boost.
            self.envelope = (self.envelope + boost_ratio).min(1.5);
        }
        self.prev_throttle = throttle;

        self.decay_rate = (-1.0 / (voicing.decay_time.max(0.01) * self.sample_rate)).exp();
        self.filter.set_coeffs(BiquadCoeffs::bandpass(
            self.sample_rate,
            voicing.center_hz,
            1.2,
        ));
    }

    #[inline(always)]
    fn process(&mut self, noise: &mut Noise) -> f32 {
        if self.envelope < 1e-5 {
            return 0.0;
        }
        let env = self.envelope;
        self.envelope *= self.decay_rate;
        self.filter.process(noise.next_bipolar()) * env * BLOW_OFF_UNITY_RMS
    }

    fn reset(&mut self) {
        self.envelope = 0.0;
        self.prev_throttle = 0.0;
        self.filter.reset();
    }
}

/// Scaling that normalises [`WastegateVoice`] to roughly unity RMS during peak hunting.
const WASTEGATE_UNITY_RMS: f32 = 3.2;

/// Wastegate chatter voice.
///
/// Under high boost and heavy throttle, the wastegate valve disk flutters and
/// hunts against exhaust gas pressure pulsations, producing a rapid metallic
/// rattle modulated at the valve flutter rate.
#[derive(Debug, Clone)]
struct WastegateVoice {
    phase: f32,
    gain: Smoothed,
    flutter_hz: Smoothed,
    filter: Biquad,
    sample_rate: f32,
}

impl WastegateVoice {
    fn new(sample_rate: f32) -> Self {
        Self {
            phase: 0.0,
            gain: Smoothed::new(0.0, sample_rate, 0.025),
            flutter_hz: Smoothed::new(65.0, sample_rate, 0.050),
            filter: Biquad::new(BiquadCoeffs::bandpass(sample_rate, 1_800.0, 2.5)),
            sample_rate,
        }
    }

    fn tune(&mut self, voicing: &WastegateVoicing, throttle: f32, turbo_rpm: f32, ref_rpm: f32) {
        let shaft_load = turbo_rpm / ref_rpm.max(1.0);
        // Gate flutters only when boost exceeds threshold AND throttle is applied.
        let boost_excess =
            (shaft_load - voicing.threshold).max(0.0) / (1.0 - voicing.threshold).max(0.05);
        let load_factor = (throttle - 0.50).max(0.0) / 0.50;
        let activity = (boost_excess * load_factor).clamp(0.0, 1.0);

        self.gain.set_target(activity * activity);
        self.flutter_hz.set_target(voicing.flutter_hz);
        self.filter.set_coeffs(BiquadCoeffs::bandpass(
            self.sample_rate,
            voicing.rattle_hz,
            2.5,
        ));
    }

    #[inline(always)]
    fn process(&mut self, noise: &mut Noise) -> f32 {
        let gain = self.gain.next_value();
        let flutter_hz = self.flutter_hz.next_value();

        self.phase += flutter_hz / self.sample_rate;
        if self.phase >= 1.0 {
            self.phase -= self.phase.floor();
        }
        if gain < 1e-5 {
            return 0.0;
        }

        // Cubed raised cosine gives sharp fluttering pulses
        let cycle = 0.5 - 0.5 * (TAU * self.phase).cos();
        let burst = cycle * cycle * cycle;

        let rattle = self.filter.process(noise.next_bipolar());
        rattle * burst * gain * WASTEGATE_UNITY_RMS
    }

    fn reset(&mut self) {
        self.phase = 0.0;
        self.gain.snap(0.0);
        self.flutter_hz.snap(65.0);
        self.filter.reset();
    }
}

// ---------------------------------------------------------------------------
// Backfire
// ---------------------------------------------------------------------------

/// How long an ignition delay has to run before a charge has convected the
/// length of a typical exhaust [s].
///
/// Not measured against any one engine's own geometry — the network already
/// knows its own lengths, and threading them back into this choice would make
/// the node picked here and the delay the network actually measures disagree
/// by construction. This is a coarse convection-time budget: exhaust gas at a
/// few hundred K moving at tens of metres a second covers a couple of metres
/// of pipe on this order of time, and it is what maps "how long the charge has
/// had to travel" onto "how far down the pipe it got".
const IGNITION_TRAVEL_SECONDS: f32 = 0.04;

/// Picks where an ignition event lights, from the physics that produced it
/// rather than from a stored kind.
///
/// A charge that has just been dumped is large, hot and still concentrated
/// right behind the valve, and lights almost where it sits. A charge that has
/// had time to convect down the pipe before finding enough heat and oxygen
/// lights further along it. `seconds_since_cut` stands in for the second
/// effect; `fuel_excess` — how far the unburnt charge sits above the ignition
/// threshold, `0..=1` — pulls the reach back toward the port, because a
/// bigger, hotter charge does not need the extra distance to find ignition.
fn choose_injection_node(fuel_excess: f32, seconds_since_cut: f32) -> InjectionNode {
    let reach = (seconds_since_cut / IGNITION_TRAVEL_SECONDS).min(1.0)
        * (1.0 - 0.6 * fuel_excess.clamp(0.0, 1.0));
    match (reach * 4.0) as usize {
        0 => InjectionNode::Port,
        1 => InjectionNode::Collector,
        2 => InjectionNode::PostSilencer,
        _ => InjectionNode::Tailpipe,
    }
}

/// Fixed order the four injection nodes are held in per bank, in
/// [`EngineSynth::backfire_pulses`] and wherever else the four are indexed
/// together.
const BACKFIRE_NODE_ORDER: [InjectionNode; 4] = [
    InjectionNode::Port,
    InjectionNode::Collector,
    InjectionNode::PostSilencer,
    InjectionNode::Tailpipe,
];

/// This node's slot in [`BACKFIRE_NODE_ORDER`] and therefore in
/// [`EngineSynth::backfire_pulses`].
fn node_index(node: InjectionNode) -> usize {
    BACKFIRE_NODE_ORDER
        .iter()
        .position(|&n| n == node)
        .unwrap_or(0)
}

/// One detected ignition, ready to be triggered into the node it lights at.
#[derive(Debug, Clone, Copy)]
struct BackfireIgnition {
    /// Pop amplitude, pre-envelope-normalisation [-].
    amplitude: f32,
    /// Envelope decay [s].
    decay: f32,
    /// Envelope attack [s].
    attack: f32,
    /// Where in the network this ignition lights.
    node: InjectionNode,
}

/// Attack of a backfire pulse before the T6 bandlimit filter, at the port and
/// at zero unburnt mass [s].
///
/// The unfiltered rise time the physical event would have on its own — the
/// same 0.00018 s every pop used before this stage told them apart.
const BACKFIRE_BASE_ATTACK_SECONDS: f32 = 0.00018;

/// How much bigger a node's own trapped volume makes the attack, relative to
/// the port [-].
///
/// A deflagration in a bigger volume of trapped gas takes longer to develop
/// into a shock front simply because there is more gas between the ignition
/// point and a wall to reflect off. This is a coarse per-node stand-in for
/// that, not a combustion-chemistry model: the collector and tailpipe are
/// plain pipe and get a small allowance for the extra gas ahead of the
/// wavefront, while a fitted silencer's expansion chamber is the one place in
/// the system with real trapped volume, so post-silencer gets the most.
fn node_volume_scale(node: InjectionNode) -> f32 {
    match node {
        InjectionNode::Port => 1.0,
        InjectionNode::Collector => 1.3,
        InjectionNode::PostSilencer => 2.2,
        InjectionNode::Tailpipe => 1.15,
    }
}

/// Attack time of a backfire igniting at `node` with unburnt charge
/// `fuel_excess` above threshold, `0..=1` [s].
///
/// Two independent effects, stacked multiplicatively: a bigger charge takes
/// longer to burn through regardless of where it lights, and a bigger volume
/// takes longer regardless of charge size. A pop that is merely quieter is
/// the same shock made smaller; a pop that is genuinely slower is a different
/// event, and this is what tells the two apart.
fn backfire_attack_seconds(fuel_excess: f32, node: InjectionNode) -> f32 {
    let mass_scale = 1.0 + 1.5 * fuel_excess.clamp(0.0, 1.0);
    BACKFIRE_BASE_ATTACK_SECONDS * mass_scale * node_volume_scale(node)
}

/// The node an already-lit charge's flame front reaches next, for a companion
/// ignition spawned by [`BackfireVoice::poll`].
fn deeper_node(node: InjectionNode) -> InjectionNode {
    match node {
        InjectionNode::Port => InjectionNode::Collector,
        InjectionNode::Collector => InjectionNode::PostSilencer,
        InjectionNode::PostSilencer | InjectionNode::Tailpipe => InjectionNode::Tailpipe,
    }
}

/// A companion ignition already scheduled by an earlier one, waiting for its
/// own flame-travel delay to run out.
#[derive(Debug, Clone, Copy)]
struct PendingCompanion {
    /// Control blocks remaining before this companion fires.
    blocks_remaining: i32,
    ignition: BackfireIgnition,
}

/// Decides when unburnt fuel in a hot runner lights off, and where.
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
/// crackle rather than a buzz. Fuel accumulates, then lights, then the flame
/// keeps propagating: a detected ignition can spawn one companion further down
/// the pipe after its own short flame-travel delay, which is what turns a
/// single detected event into the burst a real crackle is.
#[derive(Debug, Clone, Copy)]
struct BackfireVoice {
    severity: f32,
    /// How far the unburnt charge sits above the ignition threshold, `0..=1`.
    fuel_excess: f32,
    /// How long the current cut has been running [s]. Resets the instant
    /// `tune` sees the cut end, so it always measures *this* cut.
    seconds_since_cut: f32,
    cooldown: usize,
    sample_rate: f32,
    companion: Option<PendingCompanion>,
}

impl BackfireVoice {
    fn new(sample_rate: f32) -> Self {
        Self {
            severity: 0.0,
            fuel_excess: 0.0,
            seconds_since_cut: 0.0,
            cooldown: 0,
            sample_rate,
            companion: None,
        }
    }

    /// Recomputes how primed the exhaust is, `0` when either condition fails.
    fn tune(&mut self, config: &SynthConfig, snapshot: &EngineSnapshot) {
        if !snapshot.spark_cut {
            self.severity = 0.0;
            self.fuel_excess = 0.0;
            self.seconds_since_cut = 0.0;
            return;
        }
        let fuel = snapshot.unburnt_fuel_mass / config.backfire_fuel_threshold.max(1e-12) as f32;
        let heat =
            (snapshot.exhaust_temperature - config.backfire_temperature_threshold as f32) / 250.0;
        if fuel <= 1.0 || heat <= 0.0 {
            self.severity = 0.0;
            self.fuel_excess = 0.0;
            self.seconds_since_cut = 0.0;
            return;
        }
        // Saturating at twice the threshold: a genuine ignition cut dumps the
        // whole metered charge, which is well past that, so a real cut sits at
        // full severity rather than at a third of it. The ramp below still
        // separates a cut from a partial misfire.
        self.fuel_excess = (fuel - 1.0).min(1.0);
        self.severity = self.fuel_excess * heat.min(1.0);
        self.seconds_since_cut += CONTROL_BLOCK as f32 / self.sample_rate;
    }

    /// Polls once per control block; returns the next ignition to trigger, if
    /// one fires this block.
    fn poll(&mut self, noise: &mut Noise, block: usize) -> Option<BackfireIgnition> {
        // A companion scheduled by an earlier ignition fires on its own clock:
        // it is the same charge continuing to burn, not a new decision, and it
        // runs even if the cut that lit it has since ended.
        if let Some(pending) = &mut self.companion {
            pending.blocks_remaining -= 1;
            if pending.blocks_remaining <= 0 {
                let ignition = pending.ignition;
                self.companion = None;
                return Some(ignition);
            }
        }

        self.cooldown = self.cooldown.saturating_sub(block);
        if self.severity <= 0.0 || self.cooldown > 0 {
            return None;
        }
        // Up to ~80 events per second when fully primed for rapid firecracker cadence.
        let rate = 80.0 * self.severity;
        let probability = rate * block as f32 / self.sample_rate;
        if noise.next_unit() >= probability {
            return None;
        }
        // 12 ms refractory: allows rapid machine-gun stutter while staying discrete.
        self.cooldown = (0.012 * self.sample_rate) as usize;
        let scale = 0.8 + 1.2 * noise.next_unit();
        let node = choose_injection_node(self.fuel_excess, self.seconds_since_cut);
        let ignition = BackfireIgnition {
            amplitude: self.severity * scale,
            // Pops in a pipe ring far longer than a blowdown crack does.
            decay: 0.006 + 0.014 * noise.next_unit(),
            attack: backfire_attack_seconds(self.fuel_excess, node),
            node,
        };

        // Roughly a third of the time enough of the charge survives past this
        // ignition to catch again further down the pipe, at lower amplitude
        // and after a short flame-travel delay. The random spacing this
        // produces is what makes one detected ignition into a burst rather
        // than a single click.
        if self.companion.is_none() && noise.next_unit() < 0.35 {
            let companion_node = deeper_node(node);
            let companion_fuel_excess = self.fuel_excess * 0.5;
            let delay_seconds = 0.002 + 0.008 * noise.next_unit();
            let blocks = ((delay_seconds * self.sample_rate) / block as f32)
                .round()
                .max(1.0) as i32;
            self.companion = Some(PendingCompanion {
                blocks_remaining: blocks,
                ignition: BackfireIgnition {
                    amplitude: ignition.amplitude * 0.5,
                    decay: 0.006 + 0.014 * noise.next_unit(),
                    attack: backfire_attack_seconds(companion_fuel_excess, companion_node),
                    node: companion_node,
                },
            });
        }

        Some(ignition)
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

/// Peak cylinder pressure oscillation of a cycle knocking at unit intensity [Pa].
///
/// Measured knock runs from under a bar — detectable on a transducer, inaudible
/// in the car — to twenty at the point where it starts putting holes in pistons.
/// The envelope this multiplies is `0.8 * knock_intensity` clamped at two, so a
/// light knock lands at a couple of bar and a heavy one at the top of the range,
/// which is where the audible band of the phenomenon is.
///
/// What matters structurally is that it is a *pressure with a frequency*: the
/// wall is driven by the product of the two, so the same oscillation ringing at
/// 6 kHz in a small bore hammers the block harder than at 4 kHz in a large one.
/// That is why a small engine's knock sounds so much sharper than a big one's.
pub const KNOCK_PRESSURE_AMPLITUDE: f32 = 1.5e6;

/// Resonances of burned gas ringing inside the cylinder bore cavity.
///
/// Knock is auto-ignition of the unburnt end-gas ahead of the flame front:
/// an abrupt volumetric heat release that excites the acoustic modes of the
/// combustion chamber. The dominant modes are the first circumferential
/// (rho_10 = 1.8412), second circumferential (rho_20 = 3.0542), and first
/// radial (rho_01 = 3.8317).
///
/// The voice is excited on cylinder firing events by a noise burst whose
/// amplitude scales with how far past 1.0 the Livengood-Wu knock integral went,
/// decaying in 2-5 ms. Its output is a normalised pressure oscillation inside
/// the bore, and it reaches the listener only through
/// [`crate::audio::structure`] — knock is a sound heard *through the block*, and
/// none of it goes out of the exhaust port, which is shut when it happens.
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
    /// One backfire pool per bank per injection node — [`BACKFIRE_NODE_ORDER`]
    /// gives the order. An ignition can light at any node the physics picks,
    /// and mixing two different nodes' events through one pool before the pop
    /// reaches the network would inject whichever fired last at every node it
    /// touched, so each node gets its own bandlimited pool.
    backfire_pulses: Vec<[PopPool; 4]>,
    /// Anti-lag's own periodic port train, one pool per bank. Kept apart from
    /// `backfire_pulses`' port pool because it is driven by firing events
    /// rather than the Poisson cut process, and the two must not steal each
    /// other's pop slots.
    antilag_pulses: Vec<PopPool>,
    /// Cylinder each bank's port-node injections are attributed to: the
    /// bank's first cylinder, a representative primary rather than whichever
    /// cylinder actually produced the charge. The network has one port
    /// injection point exposed per bank here, not one per cylinder.
    bank_first_cylinder: Vec<usize>,
    /// Pressure radiated from each bank's mouth this sample.
    radiated: Vec<f32>,
    /// Broadband pressure each port's jet is launching this sample [Pa].
    ///
    /// Kept beside the blowdown excitation rather than added into it: they are
    /// two different sources that happen to share a boundary, one a pulse the
    /// cycle table draws and one a noise the flow makes, and folding them
    /// together would leave nothing able to say which was which.
    port_jets: Vec<f32>,
    /// The two of them summed, which is what the port actually presents [Pa].
    port_drive: Vec<f32>,
    /// Shapes each port's own jet noise to the frequency it is loudest at.
    ///
    /// One per cylinder, because the two ports of a V8's two banks are at
    /// different lifts at any instant and a jet's pitch is its velocity over
    /// its gap. Retuned every sample: the gap goes from a slit to a hole and
    /// back inside a valve event, and a control block is a fifth of one.
    port_noise: Vec<OnePole>,
    /// Last white sample each port's shaper differenced, one per cylinder [-].
    port_noise_previous: Vec<f32>,
    /// Cross-sectional area of one exhaust primary [m^2].
    ///
    /// The area the port launches its wave into; see
    /// [`EngineSynth::port_launch_pressure`].
    primary_area: f32,
    /// Pascal scale the exhaust network is driven in and read back out of [Pa].
    ///
    /// Tracks [`EngineSynth::port_launch_pressure`] on a glide far longer than
    /// a cycle, and holds its last value rather than following the flow to
    /// zero. Both of those are about the same thing. This number multiplies the
    /// excitation going in and divides what the mouth radiates coming out, and
    /// the pipe between them remembers a few milliseconds — so anything the
    /// scale does faster than the pipe can forget is an amplitude modulation
    /// applied to the ringing but not to the pulse that caused it. A scale that
    /// wobbled with the phase ring's own refresh put four decibels on the big
    /// single's first order doing exactly that. A cycle is at most a fifth of a
    /// second even at a cranking idle, so half a second of glide is slower than
    /// any of it and still quick enough to follow a pull.
    launch: Smoothed,
    /// Whether [`EngineSynth::launch`] has seen a breathing port yet.
    ///
    /// The first scale is snapped rather than glided to: there is no earlier
    /// one to glide from, and starting at a placeholder would swell the first
    /// firing up out of nothing.
    launch_primed: bool,
    intake_network: IntakeNetwork,
    intake_excitations: Vec<f32>,
    /// Effective valve flow area presented to each network this control block.
    ///
    /// Two scratch buffers rather than one because the two valves are open at
    /// different times, and one per cylinder because the whole point is that
    /// they are open at different times *from each other*. Allocated at
    /// construction; nothing here touches the heap on the callback.
    exhaust_valve_areas: Vec<f64>,
    intake_valve_areas: Vec<f64>,
    /// The cylinder each valve opens into, this control block.
    ///
    /// One volume per cylinder rather than one for the engine, because the
    /// whole point of the load is that a cylinder is at a different point in
    /// its stroke from its neighbours; and the port flows beside it, because
    /// what the gap resists is what is crossing it.
    cylinder_volumes: Vec<f64>,
    exhaust_port_flows: Vec<f64>,
    intake_port_flows: Vec<f64>,
    turbo: TurboVoice,
    roots: RootsVoice,
    centrifugal: CentrifugalVoice,
    blow_off: BlowOffVoice,
    wastegate: WastegateVoice,
    backfire: BackfireVoice,
    mechanical: MechanicalVoice,
    starter: StarterVoice,
    knock: KnockVoice,
    propagation: PropagationModel,
    tailpipe_pressures: Vec<f32>,

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
    /// Gas temperature in each primary, glided; parallel to `config.cylinders`.
    ///
    /// Allocated once, like every other per-cylinder vector here. A warm-up
    /// moves these over minutes, but a throttle transient moves them in a
    /// breath, and a delay line whose length steps rather than glides clicks.
    primary_temperature: Vec<Smoothed>,
    collector_temperature: Smoothed,
    tailpipe_temperature: Smoothed,
    /// Scratch the control path fills before handing it to the network.
    ///
    /// Held rather than built per call: it is one `MAX_CYLINDERS` array and the
    /// callback may not allocate.
    stations: ExhaustTemperatures,
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
    clipper: [OversampledClipper; 2],
    /// The block as a radiating body: modes of the casting, pan and bore walls.
    structure: StructuralPath,
    /// Structural drive per unit of knock voice output [-].
    ///
    /// Recomputed at the control rate because it follows the knock mode
    /// frequency, which follows bore and charge temperature.
    knock_scale: f32,
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
        // Re-run the same clamp `with_reciprocating_mass` applies at
        // construction, in case a caller set the field directly rather than
        // through the builder — a negative or NaN mass would otherwise reach
        // the hot path.
        config.reciprocating = config
            .reciprocating
            .with_reciprocating_mass(config.reciprocating.reciprocating_mass);

        let snapshot = EngineSnapshot::default();
        // The same fallback the network builds its primaries with, so the
        // pascal scale the excitation is played at and the pipe it is played
        // into are describing one geometry.
        let primary_area = if config.exhaust.primaries.is_empty() {
            std::f64::consts::PI * 0.020 * 0.020
        } else {
            config.exhaust.primary_area()
        };
        let network = ExhaustNetwork::new(
            &config.exhaust,
            &config.cylinders,
            config.bank_count,
            fs,
            &snapshot,
        );
        let n_cyl = config.cylinders.len();
        // The first cylinder found on each bank, used to attribute a bank's
        // port-node backfire injections to one representative primary rather
        // than needing a per-cylinder pop pool for a node most engines only
        // ever put one event into at a time.
        let mut bank_first_cylinder = vec![0usize; config.bank_count.max(1)];
        let mut bank_seen = vec![false; config.bank_count.max(1)];
        for (i, tap) in config.cylinders.iter().enumerate() {
            let b = tap.bank % config.bank_count.max(1);
            if !bank_seen[b] {
                bank_first_cylinder[b] = i;
                bank_seen[b] = true;
            }
        }
        let blowdown_pa = (0..config.cylinders.len())
            .map(|_| Smoothed::new(0.0, fs, 0.005))
            .collect();
        let primary_temperature = (0..n_cyl)
            .map(|i| {
                Smoothed::new(
                    snapshot.primary_temperature[i.min(MAX_CYLINDERS - 1)],
                    fs,
                    0.080,
                )
            })
            .collect();

        let tailpipe_area = config.exhaust.tailpipe.area.max(1e-4) as f32;
        let intake_area = config
            .intake
            .snorkel
            .as_ref()
            .map(|s| s.area)
            .or_else(|| config.intake.airbox.as_ref().map(|a| a.area))
            .unwrap_or(match config.intake.throttle {
                ThrottleLayout::Single { bore } => std::f64::consts::PI * 0.25 * bore * bore,
                ThrottleLayout::IndividualBodies { bore } => {
                    std::f64::consts::PI * 0.25 * bore * bore * config.intake.runners.len() as f64
                }
            })
            .max(1e-4) as f32;

        // The block is a structural body radiating omnidirectionally as a monopole
        // (ka <= 1 across the audible range), so its effective aperture area has
        // corner frequency well above Nyquist.
        let block_area = 1.0e-6f32;

        let tailpipe_positions = if !config.aperture_positions.tailpipes.is_empty() {
            &config.aperture_positions.tailpipes
        } else {
            &AperturePositions::front_engine_single().tailpipes
        };

        let tailpipes = tailpipe_positions
            .iter()
            .map(|&pos| {
                Aperture::new(
                    [pos[0] as f32, pos[1] as f32, pos[2] as f32],
                    tailpipe_area,
                    [0.0, -1.0, 0.0],
                )
            })
            .collect();

        let intake_aperture = Aperture::new(
            [
                config.aperture_positions.intake[0] as f32,
                config.aperture_positions.intake[1] as f32,
                config.aperture_positions.intake[2] as f32,
            ],
            intake_area,
            [0.0, 1.0, 0.0],
        );

        let block_aperture = Aperture::new(
            [
                config.aperture_positions.block[0] as f32,
                config.aperture_positions.block[1] as f32,
                config.aperture_positions.block[2] as f32,
            ],
            block_area,
            [0.0, 0.0, 1.0],
        );

        let propagation = PropagationModel::new(
            Listener::default(),
            tailpipes,
            intake_aperture,
            block_aperture,
            fs,
        );

        let mut synth = Self {
            network,
            cycle: CyclePlayer::new(fs),
            excitations: vec![0.0; n_cyl],
            backfire_pulses: vec![[PopPool::default(); 4]; config.bank_count],
            antilag_pulses: vec![PopPool::default(); config.bank_count],
            bank_first_cylinder,
            radiated: vec![0.0; config.bank_count],
            port_jets: vec![0.0; n_cyl],
            port_drive: vec![0.0; n_cyl],
            port_noise: vec![OnePole::new(fs, 4_000.0); n_cyl],
            port_noise_previous: vec![0.0; n_cyl],
            primary_area: primary_area as f32,
            launch: Smoothed::new(1.0, fs, 0.500),
            launch_primed: false,
            intake_network: IntakeNetwork::new(&config.intake, config.cylinders.len(), fs),
            intake_excitations: vec![0.0; n_cyl],
            exhaust_valve_areas: vec![0.0; n_cyl],
            intake_valve_areas: vec![0.0; n_cyl],
            cylinder_volumes: vec![1e-4; n_cyl],
            exhaust_port_flows: vec![0.0; n_cyl],
            intake_port_flows: vec![0.0; n_cyl],
            turbo: TurboVoice::new(fs),
            roots: RootsVoice::new(fs),
            centrifugal: CentrifugalVoice::new(fs),
            blow_off: BlowOffVoice::new(fs),
            wastegate: WastegateVoice::new(fs),
            backfire: BackfireVoice::new(fs),
            mechanical: MechanicalVoice::from_spec(&config.mechanical, fs),
            starter: StarterVoice::new(fs),
            knock: KnockVoice::new(fs),
            propagation,
            tailpipe_pressures: vec![0.0; config.bank_count],
            variation: vec![CycleVariation::default(); config.cylinders.len()],
            variation_depth: 0.0,
            phase_fixed: 0,
            crank_omega_delta: 0.0,
            control_countdown: 0,
            exhaust_temperature: Smoothed::new(snapshot.exhaust_temperature, fs, 0.080),
            primary_temperature,
            collector_temperature: Smoothed::new(snapshot.collector_temperature, fs, 0.080),
            tailpipe_temperature: Smoothed::new(snapshot.tailpipe_temperature, fs, 0.080),
            stations: ExhaustTemperatures::uniform(snapshot.collector_temperature),
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
            clipper: [OversampledClipper::new(); 2],
            // The runner and muffler are both bandpass-like and between them
            // leave the bottom octave thin, where a large engine's felt weight
            // actually lives. The block's own bending mode fills it in — and it
            // is filled in by being *radiated*, from a body the combustion is
            // hammering, rather than by an equaliser sitting on the exhaust.
            structure: StructuralPath::new(fs, &config.structure.modes()),
            knock_scale: 0.0,
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

    /// Acoustic propagation model placing apertures and listener in space.
    pub fn propagation(&self) -> &PropagationModel {
        &self.propagation
    }

    /// Mutable access to the acoustic propagation model.
    pub fn propagation_mut(&mut self) -> &mut PropagationModel {
        &mut self.propagation
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
        for (i, smoother) in self.primary_temperature.iter_mut().enumerate() {
            smoother.set_target(snapshot.primary_temperature[i.min(MAX_CYLINDERS - 1)]);
        }
        self.collector_temperature
            .set_target(snapshot.collector_temperature);
        self.tailpipe_temperature
            .set_target(snapshot.tailpipe_temperature);
        self.exhaust_gamma.set_target(snapshot.exhaust_gamma);
        self.exhaust_gas_constant
            .set_target(snapshot.exhaust_gas_constant);
        self.intake_flow.set_target(snapshot.intake_mass_flow);
        self.throttle.set_target(snapshot.throttle);
        self.intake_network.set_throttle(snapshot.throttle);
        self.knock.set_intensity(snapshot.knock_intensity);
        self.network.set_cutout(snapshot.exhaust_cutout);
        self.cycle.accept(&snapshot);
    }

    /// Sets whether the exhaust cutout bypass junction is open.
    pub fn set_exhaust_cutout(&mut self, open: bool) {
        self.network.set_cutout(open);
    }

    /// Clears every filter and delay line without changing parameters.
    pub fn reset(&mut self) {
        self.network.reset();
        self.intake_network.reset();
        for pools in self.backfire_pulses.iter_mut() {
            for pool in pools.iter_mut() {
                pool.reset();
            }
        }
        for pool in self.antilag_pulses.iter_mut() {
            pool.reset();
        }
        self.cycle.reset();
        self.excitations.fill(0.0);
        self.intake_excitations.fill(0.0);
        self.exhaust_valve_areas.fill(0.0);
        self.intake_valve_areas.fill(0.0);
        self.exhaust_port_flows.fill(0.0);
        self.intake_port_flows.fill(0.0);
        self.radiated.fill(0.0);
        for smoother in self.blowdown_pa.iter_mut() {
            smoother.snap(0.0);
        }
        self.turbo.reset();
        self.roots.reset();
        self.centrifugal.reset();
        self.blow_off.reset();
        self.wastegate.reset();
        self.mechanical.reset();
        self.starter.reset();
        self.knock.reset();
        self.propagation.reset();
        self.tailpipe_pressures.fill(0.0);
        self.dc = [DcBlocker::default(); 2];
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

        for (i, smoother) in self.primary_temperature.iter_mut().enumerate() {
            self.stations.primaries[i.min(MAX_CYLINDERS - 1)] = smoother.advance(block);
        }
        self.stations.collector = self.collector_temperature.advance(block);
        self.stations.tailpipe = self.tailpipe_temperature.advance(block);
        self.network.tune(gamma, gas_constant, &self.stations);
        self.network
            .set_mean_flow(self.snapshot.intake_mass_flow, gamma, gas_constant);
        self.network.set_turbine_shaft_rpm(self.snapshot.turbo_rpm);
        self.variation_depth = self.variation_depth_at(rpm);
        self.mechanical
            .tune(&self.snapshot, cycle_hz, self.config.cylinders.len());
        // Straight off the physics, with no state of its own: a zero here is a
        // pinion out of mesh, and the voice goes quiet because there is nothing
        // left turning it.
        self.starter.tune(self.snapshot.starter_hz);
        self.intake_network.set_throttle(throttle);
        self.knock
            .tune(self.snapshot.bore, gamma, gas_constant, temperature);
        // What the bore wall feels is the rate of the knock oscillation, so the
        // voice's normalised output becomes a structural drive by way of its own
        // frequency and the amplitude a knocking cycle actually reaches.
        self.knock_scale = combustion_drive(
            TAU * self.knock.mode_frequencies()[0] * KNOCK_PRESSURE_AMPLITUDE,
            self.config.structure.bore as f32,
        );
        // Nothing to tune on an atmospheric engine: the voice is left cold and
        // never asked for a sample, rather than run at a level of zero.
        if let Some(voicing) = self.config.turbo {
            self.turbo
                .tune(&voicing, self.snapshot.turbo_rpm, self.snapshot.turbo_surge);
        }
        if let Some(voicing) = self.config.roots {
            self.roots
                .tune(&voicing, self.snapshot.rpm, self.intake_flow.value());
        }
        if let Some(voicing) = self.config.centrifugal {
            self.centrifugal
                .tune(&voicing, self.snapshot.rpm, self.intake_flow.value());
        }
        if let Some(voicing) = self.config.blow_off {
            let ref_rpm = self
                .config
                .turbo
                .map_or(130_000.0, |t| t.reference_rpm as f32);
            self.blow_off.tune(
                &voicing,
                self.snapshot.throttle,
                self.snapshot.turbo_rpm,
                ref_rpm,
            );
        }
        if let Some(voicing) = self.config.wastegate {
            let ref_rpm = self
                .config
                .turbo
                .map_or(130_000.0, |t| t.reference_rpm as f32);
            self.wastegate.tune(
                &voicing,
                self.snapshot.throttle,
                self.snapshot.turbo_rpm,
                ref_rpm,
            );
        }

        self.backfire.tune(&self.config, &self.snapshot);
        if let Some(ignition) = self.backfire.poll(&mut self.noise, CONTROL_BLOCK) {
            let bank = (self.noise.next_u32() as usize) % self.backfire_pulses.len();
            let amplitude = ignition.amplitude * self.config.backfire_level as f32 * 2.0;
            // Backfires combine an explosive positive expansion wave with
            // turbulent flame roar; sharing the runner and muffler gives them
            // the pipe's acoustic colour without reducing to a thin metallic click.
            // The poll that found this pop only runs once a block, so its true
            // ignition instant is uniformly distributed somewhere in the block
            // just finished; a fresh draw over that width is that instant's
            // correct distribution, not a quantised guess at the boundary.
            let age_samples = self.noise.next_unit() * CONTROL_BLOCK as f32;
            let node_slot = node_index(ignition.node);
            self.backfire_pulses[bank][node_slot].trigger(
                self.config.sample_rate,
                amplitude,
                ignition.attack,
                ignition.decay,
                0.80,
                age_samples,
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
                if self.snapshot.anti_lag {
                    // Anti-lag pops are periodic and locked to firing rather
                    // than the Poisson cut process `BackfireVoice` runs, and
                    // they light at the port: the charge anti-lag dumps is
                    // fresh and still right behind the valve on every cycle,
                    // the same reasoning that pulls a fresh cut's charge
                    // toward the port in `choose_injection_node`.
                    let bank = tap.bank % self.antilag_pulses.len().max(1);
                    let amplitude = self.config.backfire_level as f32 * 0.6;
                    self.antilag_pulses[bank].trigger(
                        self.config.sample_rate,
                        amplitude,
                        backfire_attack_seconds(0.0, InjectionNode::Port),
                        0.010,
                        0.80,
                        0.0,
                    );
                }
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

    /// Acoustic pressure this cycle's exhaust port launches at its peak flow [Pa].
    ///
    /// A wave in a duct is $p = \rho c u$, and a port moving $\dot m$ into a
    /// runner of area $A$ is moving gas at $u = \dot m / \rho A$, so the two
    /// densities cancel and the pressure the port launches is
    ///
    /// $$p = \frac{c \, \dot m}{A}$$
    ///
    /// — the same law the intake side launches its rarefaction by. Across the
    /// catalogue this runs from a third of an atmosphere at idle to two thirds
    /// at the limiter, and it is a good deal smaller than the pressure
    /// *difference* across the valve, because a port is a restriction and not
    /// an open end. A runner handed the full blowdown difference would be past
    /// the shock condition on every firing at every speed, which is the same as
    /// not modelling the shock condition at all.
    ///
    /// The excitation shape is normalised to a peak of one, so this is the
    /// pascal scale it is played at, and it is divided back out where the mouth
    /// radiates. It therefore cancels out of every linear element in the
    /// network — the level law is untouched by it — and survives only where
    /// propagation is not linear, which is the primaries.
    #[inline(always)]
    fn port_launch_pressure(&self, blend: f32) -> f32 {
        self.cycle.launch_flux(blend) / self.primary_area
    }

    /// Reads each cylinder's excitation out of the cycle at its own phase.
    ///
    /// Every cylinder plays the same curve, displaced by where its EVO sits in
    /// the master cycle — which is the audio-side statement of the same thing
    /// the phase ring does on the physics side.
    ///
    /// Returns the structural drive the whole engine is delivering this
    /// sample, as two independent terms: `dP/dtheta` summed over the
    /// cylinders at their own phases (the combustion term, in the cycle's own
    /// units), and the reciprocating shaking-force resultant summed over the
    /// same cylinders at the same phases (in newtons). Every cylinder hammers
    /// the same block, so unlike the exhaust — which has one primary per
    /// cylinder — each is a single scalar, and the firing order is in both by
    /// construction.
    #[inline(always)]
    fn fill_excitations(&mut self, turning: bool, cycle_hz: f32) -> (f32, f32) {
        // Advanced whether or not the crank is, so a fade cannot be left
        // half-finished by a stopped engine and resume when it restarts.
        let blend = self.cycle.advance();
        // Per sample, not per control block: it is a gain on the excitation and
        // a divisor on what the mouth radiates, and stepping a gain sixteen
        // samples at a time puts an edge in both.
        let live = self.port_launch_pressure(blend);
        if live > 0.0 {
            if self.launch_primed {
                self.launch.set_target(live);
            } else {
                self.launch.snap(live);
                self.launch_primed = true;
            }
        }
        let launch = self.launch.next_value();
        let c_exhaust = crate::audio::filters::speed_of_sound(
            self.exhaust_gamma.value(),
            self.exhaust_gas_constant.value(),
            self.exhaust_temperature.value(),
        );
        let port_density = crate::audio::waveguide::REFERENCE_PRESSURE_PA
            / (self.exhaust_gas_constant.value() * self.exhaust_temperature.value().max(1.0));
        let fs = self.config.sample_rate;
        if !turning {
            self.excitations.fill(0.0);
            self.port_jets.fill(0.0);
            self.intake_excitations.fill(0.0);
            // A stopped engine has its valves wherever the crank left them, and
            // leaving the last frame's areas in place is as good an answer as
            // any; what matters is that they stop moving with nothing driving
            // them.
            return (0.0, 0.0);
        }
        let phase = self.cycle_phase();
        let mut rise = 0.0;
        let mut shake = 0.0;
        // Crank angular speed, `cycles/s * rad/cycle`, matching the master
        // cycle's own phase convention — a "cycle" here is the full 720-degree
        // four-stroke, not one crank turn.
        let omega = cycle_hz as f64 * CYCLE_ANGLE;
        let c_intake = crate::audio::filters::speed_of_sound(
            crate::audio::intake_voice::INTAKE_AIR_GAMMA,
            crate::audio::intake_voice::INTAKE_GAS_CONSTANT,
            crate::audio::intake_voice::INTAKE_AMBIENT_TEMPERATURE_K,
        );
        for index in 0..self.config.cylinders.len() {
            let tap = self.config.cylinders[index];
            let variation = self.variation[index];
            let alive = self.blowdown_pa[index].value() > 1.0;
            // Strictly linear in the pressure difference, per the excitation
            // model; the curve it multiplies is normalised to a peak of one.
            //
            // In pascals, because the network is. Every boundary in it — the
            // valve load, the junctions, the wall loss — is linear and would
            // read the same from a normalised drive, but propagation down a
            // primary is not: a characteristic's speed is set by `p / P_0`, and
            // a pulse handed over as a number near one is a pulse a hundred
            // thousandth of an atmosphere tall, which steepens by nothing at
            // all. The scaling back to the mix happens once, where the mouth
            // radiates.
            //
            // The scale is the port's own ceiling and not the pressure
            // difference driving it. A wave in a duct carries `p = rho c u`,
            // the throat chokes at `u = c`, so the largest wave a port can
            // launch is `rho c^2 = gamma P_0` however far past that the
            // cylinder is — which is why a full-scale blowdown is a bar and a
            // third of pulse and not the four and a half bar of
            // [`REFERENCE_BLOWDOWN`] across the valve.
            let amplitude = if alive {
                (self.blowdown_pa[index].value() / REFERENCE_BLOWDOWN).min(2.0)
                    * launch
                    * variation.amplitude_scale
            } else {
                0.0
            };
            let cylinder = phase - tap.evo_phase;
            let combustion = cylinder - variation.phase_offset;
            self.excitations[index] = amplitude * self.cycle.exhaust_at(combustion, blend);
            // The same retard applies to the structural path, and for the same
            // reason: a cycle whose flame took longer to develop reaches the
            // block late as well as reaching the port late.
            if alive {
                rise += self.cycle.pressure_slope_at(combustion, blend) * variation.amplitude_scale;
            }
            // Reciprocating inertia force, at the bare cylinder phase rather
            // than `combustion`'s: the piston does not wait for the flame to
            // develop, so this term is not retarded by cycle-to-cycle
            // combustion variation the way the pressure-driven terms above
            // are. `cylinder` is this cylinder's own crank phase as a fraction
            // of the master 720-degree cycle, so scaling it by `CYCLE_ANGLE`
            // recovers the true crank angle `piston_position` is defined on.
            let theta = cylinder as f64 * CYCLE_ANGLE;
            let force = self.config.reciprocating.inertia_force(theta, omega);
            let angle = bank_angle(tap.bank as u8, self.config.bank_count);
            // The structural path is mono and has no separate image for a
            // sideways rock versus a vertical heave, so the two in-plane axes
            // are collapsed onto one scalar here rather than carried as a
            // vector any further.
            shake += (force * (angle.sin() + angle.cos())) as f32;
            // The valve at the head of this cylinder's two runners.
            //
            // A runner's far end is a junction and its near end is the valve,
            // so until this ran the near end was a rigid plug for the whole
            // cycle and every runner in the engine was a lossless quarter-wave
            // resonator with nowhere for its energy to go.
            //
            // Read per sample, not per control block. A reflection coefficient
            // is a gain inside a feedback loop, and stepping it sixteen samples
            // at a time puts a staircase in that loop whose edges land at
            // exactly `sample_rate / CONTROL_BLOCK` — 3 kHz at 48 kHz, with a
            // harmonic at 6, both of them standing still while the engine
            // sweeps past. Two divides a sample buys a coefficient that moves
            // as smoothly as the cam does.
            self.exhaust_valve_areas[index] =
                self.cycle.exhaust_valve_area_at(cylinder, blend) as f64;
            self.intake_valve_areas[index] =
                self.cycle.intake_valve_area_at(cylinder, blend) as f64;
            // The other two thirds of the boundary: what is behind the valve
            // and what is going through it. Both read at the bare cylinder
            // phase, like the areas — a piston's position and a port's flux are
            // geometry and gas dynamics, and neither waits for a flame.
            self.cylinder_volumes[index] = self.cycle.cylinder_volume_at(cylinder, blend) as f64;
            self.exhaust_port_flows[index] = self.cycle.exhaust_flow_at(cylinder, blend) as f64;

            // The port's own jet. Gas crossing a valve seat at a few hundred
            // metres a second tears itself apart on the edge, and what that
            // makes is broadband and goes down the runner with everything else.
            // It is the exhaust's half of what the intake already gets from its
            // throttle and its valves, and without it the pipe radiates a comb
            // with nothing at all between the teeth.
            let throat = crate::audio::waveguide::throat_velocity(
                self.exhaust_port_flows[index] as f32,
                self.exhaust_valve_areas[index] as f32,
                port_density,
                c_exhaust,
            );
            self.port_jets[index] = 0.0;
            let jet = crate::audio::waveguide::jet_pressure_fluctuation_pa(
                throat,
                port_density,
                c_exhaust,
                self.exhaust_valve_areas[index] as f32,
                self.primary_area,
            );
            if jet > 0.0 {
                let hz = crate::audio::waveguide::jet_peak_hz(
                    throat,
                    self.exhaust_valve_areas[index] as f32,
                );
                // A jet's spectrum climbs to its Strouhal peak and falls away
                // past it, so the noise is differenced on the way in for the
                // rise and one-poled at the peak for the fall: six decibels an
                // octave each side, which is the crudest shape that is still
                // the right shape. A lowpass alone would put most of the energy
                // below the peak, where the firings already are.
                let white = self.noise.next_bipolar();
                let step = white - self.port_noise_previous[index];
                self.port_noise_previous[index] = white;
                let shaper = &mut self.port_noise[index];
                shaper.set_cutoff(fs, hz);
                // Differenced white noise has variance 2 and autocorrelation
                // -1 at one sample, so through a pole at `1 - a` it comes out
                // with variance `2a^2 / (2 - a)`. Dividing that back out is
                // what makes the law above an amplitude rather than a number
                // that happens to come out near one.
                let a = shaper.coefficient().max(1e-9);
                let unity = (0.5 * (2.0 - a)).sqrt() / a;
                self.port_jets[index] = jet * unity * shaper.process(step);
            }

            // Induction is read at the bare cylinder phase. The retard and the
            // jitter are properties of *combustion* — how long the flame takes
            // to develop, and how much that varies — and a valve opening on the
            // intake side does not wait for a flame.
            let flow = self.cycle.intake_at(cylinder, blend);
            self.intake_port_flows[index] = flow as f64;

            // Per cycle from the table, times cycles per second: `dmdot/dt`.
            let d_flow_dt = self.cycle.intake_slope_at(cylinder, blend) * cycle_hz;

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
        (rise, shake)
    }

    /// Produces one stereo frame.
    #[inline(always)]
    fn tick(&mut self) -> (f32, f32) {
        let cycle_hz = self.cycle_hz.next_value();
        let turning = self.advance_crank(cycle_hz);
        // dP/dtheta in cycles, times cycles per second, is dP/dt — so the same
        // pressure curve drives the block harder at speed, which is why
        // combustion noise climbs with rpm on every engine ever measured.
        let (rise, shake) = self.fill_excitations(turning, cycle_hz);
        let rise = rise * cycle_hz;
        self.network.set_valve_loads(
            &self.exhaust_valve_areas,
            &self.cylinder_volumes,
            &self.exhaust_port_flows,
        );
        self.intake_network.set_valve_loads(
            &self.intake_valve_areas,
            &self.cylinder_volumes,
            &self.intake_port_flows,
        );

        let exhaust_level = self.exhaust_level.next_value();
        let launch = self.launch.value();
        for (bank, pools) in self.backfire_pulses.iter_mut().enumerate() {
            for (pool, &node) in pools.iter_mut().zip(BACKFIRE_NODE_ORDER.iter()) {
                // Into the same pascals the cylinders' own excitations arrive in.
                let value = pool.process(&mut self.noise) * launch;
                let index = match node {
                    InjectionNode::Port => self.bank_first_cylinder[bank],
                    _ => bank,
                };
                self.network.inject(node, index, value);
            }
            let antilag = self.antilag_pulses[bank].process(&mut self.noise) * launch;
            self.network
                .inject(InjectionNode::Port, self.bank_first_cylinder[bank], antilag);
        }
        for ((drive, pulse), jet) in self
            .port_drive
            .iter_mut()
            .zip(self.excitations.iter())
            .zip(self.port_jets.iter())
        {
            *drive = pulse + jet;
        }
        self.network.step(&self.port_drive, &mut self.radiated);
        for (tp, &out) in self.tailpipe_pressures.iter_mut().zip(self.radiated.iter()) {
            *tp = out * (exhaust_level / launch);
        }

        let turbo = match self.config.turbo {
            Some(voicing) => self.turbo.process(&mut self.noise) * voicing.level as f32,
            None => 0.0,
        };
        let roots = match self.config.roots {
            Some(voicing) => self.roots.process() * voicing.level as f32,
            None => 0.0,
        };
        let centrifugal = match self.config.centrifugal {
            Some(voicing) => self.centrifugal.process() * voicing.level as f32,
            None => 0.0,
        };
        let blow_off = match self.config.blow_off {
            Some(voicing) => self.blow_off.process(&mut self.noise) * voicing.level,
            None => 0.0,
        };
        let wastegate = match self.config.wastegate {
            Some(voicing) => self.wastegate.process(&mut self.noise) * voicing.level,
            None => 0.0,
        };
        let intake_rad =
            self.intake_network.step(&self.intake_excitations) / REFERENCE_INTAKE_PRESSURE;
        // Combustion, reciprocating inertia, the mechanical rig and knock all
        // arrive at the block as one force, because the block cannot tell them
        // apart: a lifter landing on a valve, a flame front arriving at the
        // piston crown and a piston reversing at the top of its stroke are all
        // metal being hit or hauled to a stop, and all of them reach the
        // listener only by shaking the casing. Summing them before the bank
        // rather than after is not an optimisation — it is the statement that
        // there is one structure, not four.
        let drive = rise * self.combustion_scale
            + shaking_drive(shake)
            + self.mechanical.process(&mut self.noise) * self.config.mechanical_level as f32
            + self.starter.process(&mut self.noise) * self.config.starter_level as f32
            + self.knock.process(&mut self.noise) * self.knock_scale;
        let block_rad = turbo
            + roots
            + centrifugal
            + blow_off
            + wastegate
            + self.structure.process(drive) * self.config.structure_level as f32;
        let intake_rad = intake_rad * self.config.intake_level as f32;

        let (left, right) = self
            .propagation
            .step(&self.tailpipe_pressures, intake_rad, block_rad);

        let gain = self.config.master_gain as f32 * self.fade_in.next_value();
        let mut out = [left, right];
        for (i, sample) in out.iter_mut().enumerate() {
            // The blowdown train carries a standing offset, and a DC offset
            // costs headroom in the clipper without being audible at all.
            *sample = self.clipper[i].process(self.dc[i].process(*sample) * gain);
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
mod tests;
