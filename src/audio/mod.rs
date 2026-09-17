//! Real-time procedural engine audio.
//!
//! The engine note here is not sampled, looped, or crossfaded between recorded
//! layers. Every sample is synthesised from the state of the thermodynamic
//! solver in [`crate::physics`], which means the sound is a consequence of the
//! simulation rather than an illustration of it: a lean cylinder, a cold pipe,
//! or a shift cut change the waveform because they change the physics that
//! produced it.
//!
//! - [`dsp`] — the synthesis voice, and the [`dsp::EngineSnapshot`] contract
//!   between the two threads.
//! - [`filters`] — biquads, delay lines and the gas-dynamic formulas that tune
//!   them.
//! - [`radiation`] — what an open end does: reflect, lengthen, and radiate.
//! - [`stream`] — the `cpal` output stream and the lock-free queue feeding it.
//! - [`structure`] — the block as a radiating body, not as an EQ on the bus.
//!
//! # Wiring it up
//!
//! ```no_run
//! use rust_engine_sim::audio::{EngineAudio, EngineControls, SnapshotSource, SynthConfig};
//! use rust_engine_sim::environment::Environment;
//! use rust_engine_sim::physics::engine_block::EngineBlock;
//!
//! let mut block = EngineBlock::cross_plane_v8(Environment::default());
//! // The stream replaces `sample_rate` with whatever the device negotiates.
//! let mut audio = EngineAudio::start(SynthConfig::from_block(&block, 48_000.0))?;
//! let mut source = SnapshotSource::new(&block);
//!
//! let (dt, rpm) = (1.0 / 240.0, 3_000.0);
//! loop {
//!     block.update(dt, rpm);
//!     audio.push(source.sample(&block, rpm, dt, EngineControls::wide_open()));
//!     std::thread::sleep(std::time::Duration::from_secs_f64(dt));
//! }
//! # Ok::<(), anyhow::Error>(())
//! ```
//!
//! # Forced induction
//!
//! The solver is atmospheric: it has no turbine, compressor or wastegate, and
//! nothing in it produces boost. A turbo therefore exists only in this module,
//! as a shaft speed that pitches a whistle and a surge margin that keys a
//! flutter — and only on the engines that are given one. See [`Induction`],
//! which a caller passes to both [`SnapshotSource::with_induction`] and
//! [`SynthConfig::with_induction`]; leave it out, as the example above does,
//! and the engine is naturally aspirated and silent between firings.
//!
//! # Where the split falls
//!
//! The physics thread owns *what the engine is doing*; the audio thread owns
//! *when each event lands*. [`SnapshotSource`] is the seam. It reads the block's
//! phase ring at the exhaust valve opening angle — which is available regardless
//! of where the master cylinder currently sits, because the ring stores a whole
//! cycle — and packages the handful of scalars the synth needs.

pub mod dsp;
pub mod filters;
pub mod intake_voice;
pub mod propagation;
pub mod radiation;
pub mod stream;
pub mod structure;
pub mod waveguide;

pub use dsp::{
    BlowOffVoicing, CentrifugalVoicing, CylinderTap, EngineSnapshot, EngineSynth, ImpulsiveSpec,
    MechanicalSpec, RootsVoicing, SourceRate, SynthConfig, TurboVoicing, WastegateVoicing,
    MAX_CYLINDERS,
};
pub use filters::MufflerGeometry;
pub use propagation::{Aperture, AperturePath, AperturePositions, Listener, PropagationModel};
pub use stream::{
    AudioScope, AudioSettings, AudioStats, EngineAudio, StreamInfo, PREFERRED_SAMPLE_RATE,
};
pub use structure::StructuralSpec;

use crate::physics::cylinder::{wrap_cycle, CYCLE_ANGLE};
use crate::physics::engine_block::EngineBlock;

// ---------------------------------------------------------------------------
// Driver inputs
// ---------------------------------------------------------------------------

/// What the driver is asking for, as far as the audio path is concerned.
///
/// The 0D block does not model a throttle plate or an ignition cut — it is given
/// a speed and solves the cycle. These are the two inputs the *sound* needs that
/// the solver does not currently carry, so they are supplied alongside it rather
/// than inferred from it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EngineControls {
    /// Throttle position, `0..=1` [-].
    pub throttle: f64,
    /// Whether ignition is cut this frame (upshift cut, launch limiter, overrun).
    pub spark_cut: bool,
    /// Whether the active exhaust cutout flap is open.
    pub exhaust_cutout: bool,
    /// Whether anti-lag is active.
    pub anti_lag: bool,
    /// Tooth-mesh frequency of the starter pinion against the ring gear [Hz].
    ///
    /// Zero whenever the pinion is out, which is every frame of every engine
    /// that is running — so this is the only field here that is normally zero
    /// and occasionally not, rather than the other way round. It carries both
    /// the pitch and the fact that there is anything to hear:
    /// [`Starter::whine_hz`](crate::physics::control::Starter::whine_hz)
    /// returns zero for a pinion out of mesh rather than a frequency nobody is
    /// listening to, because a starter is not faded out when the engine
    /// catches. It is physically thrown out of the gear it was singing with.
    pub starter_hz: f64,
}

impl Default for EngineControls {
    /// Closed throttle, ignition live.
    fn default() -> Self {
        Self {
            throttle: 0.0,
            spark_cut: false,
            exhaust_cutout: false,
            anti_lag: false,
            starter_hz: 0.0,
        }
    }
}

impl EngineControls {
    /// Full throttle, ignition live.
    pub fn wide_open() -> Self {
        Self {
            throttle: 1.0,
            spark_cut: false,
            exhaust_cutout: false,
            anti_lag: false,
            starter_hz: 0.0,
        }
    }

    /// Throttle held open with the spark cut — the condition that makes pops.
    pub fn on_the_limiter(throttle: f64) -> Self {
        Self {
            throttle: throttle.clamp(0.0, 1.0),
            spark_cut: true,
            exhaust_cutout: false,
            anti_lag: false,
            starter_hz: 0.0,
        }
    }

    /// Sets whether the active exhaust cutout flap is open.
    pub fn with_exhaust_cutout(mut self, open: bool) -> Self {
        self.exhaust_cutout = open;
        self
    }

    /// Sets whether anti-lag is active.
    pub fn with_anti_lag(mut self, enabled: bool) -> Self {
        self.anti_lag = enabled;
        self
    }
}

// ---------------------------------------------------------------------------
// Turbocharger
// ---------------------------------------------------------------------------

/// A first-order turbocharger shaft, for driving the audio only.
///
/// The block has no turbine, compressor or wastegate, so this is not a claim
/// about boost — it produces no pressure and feeds nothing back into the
/// solver. It exists because the shaft's *speed* is what the whistle is pitched
/// to and the surge margin is what the flutter keys off, and both need to move
/// with some plausible lag rather than snapping to the throttle.
///
/// The lag is asymmetric on purpose: a turbo spools up against the inertia of
/// the wheel and the time it takes exhaust energy to build, and coasts back down
/// on bearing drag alone, so spool-up is the slower of the two.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TurboModel {
    /// Shaft speed at the compressor's limit [rev/min].
    pub max_shaft_rpm: f64,
    /// Engine speed at which the turbine is fully driven [rev/min].
    pub reference_engine_rpm: f64,
    /// Time constant while gaining speed [s].
    pub spool_up: f64,
    /// Time constant while losing speed [s].
    pub spool_down: f64,
    /// Current shaft speed [rev/min].
    pub shaft_rpm: f64,
}

impl Default for TurboModel {
    /// A single mid-frame turbo on a 7000 rpm petrol engine.
    fn default() -> Self {
        Self {
            max_shaft_rpm: 165_000.0,
            reference_engine_rpm: 6_500.0,
            spool_up: 0.55,
            spool_down: 0.35,
            shaft_rpm: 0.0,
        }
    }
}

impl TurboModel {
    /// Advances the shaft one frame and returns its speed [rev/min].
    ///
    /// The turbine is driven by exhaust enthalpy, which rises with both mass
    /// flow and temperature, so the demand term uses engine speed (a proxy for
    /// flow) and throttle (a proxy for how much of that flow carries energy).
    pub fn update(&mut self, dt: f64, rpm: f64, controls: &EngineControls) -> f64 {
        let flow = (rpm / self.reference_engine_rpm.max(1.0)).clamp(0.0, 1.2);
        // A shut throttle still leaves the turbine turning on pumped air, which
        // is why a turbo does not stop dead the instant a driver lifts.
        // Anti-lag fires late into the exhaust header with fuel on overrun,
        // maintaining high turbine enthalpy and keeping the shaft spooled.
        let energy = if controls.anti_lag && controls.throttle < 0.25 && rpm >= 2_000.0 {
            0.85
        } else {
            0.12 + 0.88 * controls.throttle.clamp(0.0, 1.0)
        };
        let demand = self.max_shaft_rpm * flow.powf(0.85) * energy;

        let tau = if demand > self.shaft_rpm {
            self.spool_up
        } else {
            self.spool_down
        };
        // Exponential approach, exact for the frame rather than Euler, so the
        // result does not depend on how the caller chose `dt`.
        let alpha = 1.0 - (-dt.max(0.0) / tau.max(1e-3)).exp();
        self.shaft_rpm += (demand - self.shaft_rpm) * alpha;
        self.shaft_rpm
    }

    /// How far into surge the compressor is, `0..=1` [-].
    ///
    /// Surge is a mismatch, not a speed: the wheel is still spinning hard while
    /// the throttle downstream has shut, so the compressor is asked to push air
    /// into a closed pipe, stalls, and the flow reverses. That is exactly the
    /// lift-off-boost condition, and it is why the flutter appears on the
    /// *release* of the throttle rather than on the application.
    pub fn surge(&self, controls: &EngineControls) -> f64 {
        let spinning = (self.shaft_rpm / self.max_shaft_rpm.max(1.0)).clamp(0.0, 1.0);
        let outlet = controls.throttle.clamp(0.0, 1.0);
        // Needs real shaft speed *and* a shut throttle; either alone is nothing.
        ((spinning - 0.35) / 0.65).clamp(0.0, 1.0) * (1.0 - outlet).powi(2)
    }
}

// `Induction` now lives in `crate::physics::intake`: it describes hardware
// (TB3 gives the physics its own real compressor and turbine), not a sound,
// and the voicing types below hang off it rather than the other way round.
// Re-exported here so every existing `audio::Induction` caller keeps
// compiling — see `docs/TURBO_PLAN.md`'s TB3.
pub use crate::physics::intake::Induction;

// ---------------------------------------------------------------------------
// Physics to snapshot
// ---------------------------------------------------------------------------

/// Turns an [`EngineBlock`] into the snapshots the audio thread consumes.
///
/// Stateful only because of the turbo shaft; everything else is read straight
/// out of the block each frame.
#[derive(Debug, Clone)]
pub struct SnapshotSource {
    /// Exhaust valve opening angle in cycle coordinates [rad].
    evo_angle: f64,
    /// The audio-side turbo shaft, if the engine has one.
    turbo: Option<TurboModel>,
    /// Rotating assembly inertia [kg m^2].
    inertia: f64,
}

impl SnapshotSource {
    /// Builds a source for a naturally aspirated block, reading its valve timing.
    pub fn new(block: &EngineBlock) -> Self {
        Self::with_induction(block, Induction::NaturallyAspirated)
    }

    /// Builds a source for a block breathing a particular way.
    ///
    /// The induction has to be given rather than read off the block, because
    /// the block has no compressor in it to read: forced induction exists in
    /// this simulator only in the audio path.
    pub fn with_induction(block: &EngineBlock, induction: Induction) -> Self {
        Self {
            evo_angle: block.model.valves.exhaust.open_angle,
            turbo: induction.shaft(),
            inertia: 0.25,
        }
    }

    /// Sets the rotating assembly inertia [kg m^2].
    pub fn with_inertia(mut self, inertia: f64) -> Self {
        self.inertia = inertia.max(1e-3);
        self
    }

    /// Rotating assembly inertia [kg m^2].
    pub fn inertia(&self) -> f64 {
        self.inertia
    }

    /// The exhaust valve opening angle this source samples at [rad].
    pub fn evo_angle(&self) -> f64 {
        self.evo_angle
    }

    /// Whether a turbo is fitted.
    pub fn has_turbo(&self) -> bool {
        self.turbo.is_some()
    }

    /// Current shaft speed [rev/min], or zero with no turbo fitted.
    pub fn turbo_shaft_rpm(&self) -> f64 {
        self.turbo.map_or(0.0, |turbo| turbo.shaft_rpm)
    }

    /// How far into surge the compressor is, `0..=1`, or zero with none fitted.
    pub fn turbo_surge(&self, controls: &EngineControls) -> f64 {
        self.turbo.map_or(0.0, |turbo| turbo.surge(controls))
    }

    /// Reads one frame of state.
    ///
    /// `dt` is the wall-clock frame length the block was just advanced by, and
    /// is used only for the turbo shaft.
    pub fn sample(
        &mut self,
        block: &EngineBlock,
        rpm: f64,
        dt: f64,
        controls: EngineControls,
    ) -> EngineSnapshot {
        // The state at EVO, not the state right now. The phase ring holds a full
        // cycle, so the conditions at the exhaust valve's opening are available
        // no matter where the master cylinder happens to be in its stroke — and
        // that is the pressure that actually drives the blowdown pulse.
        let at_evo = block.ring.sample(self.evo_angle);

        let banks = block.exhaust_banks.len().max(1) as f64;
        let mut manifold_pressure = 0.0;
        let mut manifold_temperature = 0.0;
        for bank in &block.exhaust_banks {
            manifold_pressure += bank.port_pressure();
            manifold_temperature += bank.plenum.temperature;
        }
        manifold_pressure /= banks;
        manifold_temperature /= banks;

        // Fuel that reaches the exhaust unburnt.
        //
        // The solver has no ignition switch: its Wiebe profile fires on every
        // cycle, and the charge is fully burned (`x_b = 1`) by the time the
        // exhaust valve opens, so a snapshot taken straight from the ring always
        // reports zero unburnt fuel. A spark cut therefore has to be applied
        // here, at the boundary, by reading the same trapped charge as though it
        // had never lit — which is exactly what a cut does to it. The mass and
        // its timing are still the solver's; only the burn fraction is
        // overridden.
        //
        let tip_in = if block.ecu.dfco_tip_in {
            block.ecu.tip_in_fuel_mass
        } else {
            0.0
        };
        let limiter_spark = block.ecu.active_cut == crate::physics::control::LimiterCut::Spark;
        let limiter_fuel = block.ecu.active_cut == crate::physics::control::LimiterCut::Fuel;
        // A compression-ignition engine has no coil, so nothing can cut its
        // spark — not the limiter, not the driver holding the two-step. The
        // only way to stop it firing is to stop fuelling it, and a cylinder
        // that never got any fuel has none to send out unburnt. That is why a
        // diesel does not pop on a lift and does not bang off its limiter.
        // A cold misfire arrives here the same way a cut does, and for the same
        // reason: the solver's charge is fuelled, the ECU knows the flame never
        // propagated, and what leaves the exhaust valve is the whole of it.
        let has_spark = block.model.combustion.is_spark_ignited();
        let spark_cut = has_spark
            && (controls.spark_cut
                || block.ecu.dfco_tip_in
                || limiter_spark
                || block.ecu.misfiring)
            && !limiter_fuel;

        // The blowdown driver per cylinder. Clamped at zero because a negative difference
        // means the manifold is momentarily above the cylinder — reverse flow,
        // which is a scavenging event, not an acoustic excitation. On a cut cylinder
        // with fuel delivered, the compressed unburnt charge still expands and blows down
        // when EVO opens (~35% of combustion blowdown). Only a fuel cut zeroes the blowdown.
        let mut blowdown_delta = [0.0f32; MAX_CYLINDERS];
        for (i, cyl) in block
            .firing
            .cylinders
            .iter()
            .enumerate()
            .take(MAX_CYLINDERS)
        {
            if limiter_fuel || (!block.ecu.is_fuel_ok(i) && !block.ecu.is_spark_ok(i)) {
                blowdown_delta[i] = 0.0;
                continue;
            }
            let bank = (cyl.bank as usize) % block.exhaust_banks.len().max(1);
            let manifold_p = block
                .exhaust_banks
                .get(bank)
                .map_or(manifold_pressure, |b| b.port_pressure());
            let evo_p = block.cylinder_evo_pressure(i);
            let delta = (evo_p - manifold_p).max(0.0) as f32;
            blowdown_delta[i] = if spark_cut || !block.ecu.is_spark_ok(i) {
                delta * 0.35
            } else {
                delta
            };
        }

        // Instantaneous induction flux summed over the cylinders that are
        // actually drawing. This is a *sum of instants*, not a cycle average:
        // the intake roar is made by the individual gulps, so the value that
        // drives it has to keep their peaks.
        let mut cylinder_intake_flow = [0.0f32; MAX_CYLINDERS];
        let mut intake_mass_flow = 0.0f64;
        for (i, slot) in cylinder_intake_flow
            .iter_mut()
            .take(block.firing.len())
            .enumerate()
        {
            let flow = block.sample_of(i).intake_flow.max(0.0);
            *slot = flow as f32;
            intake_mass_flow += flow;
        }

        let burned_at_evo = if spark_cut {
            0.0
        } else {
            at_evo.burned_fraction
        };
        let dead_plug_count = (0..block.firing.len())
            .filter(|&i| !block.ecu.is_spark_ok(i) && block.ecu.is_fuel_ok(i))
            .count();
        let dead_plug_fuel = if spark_cut || limiter_fuel {
            0.0
        } else {
            let total = block.firing.len().max(1);
            let unburnt_charge = block.model.trapped_fuel_mass(at_evo.mass, 0.0);
            (unburnt_charge / total as f64) * dead_plug_count as f64
        };
        let anti_lag_active = (controls.anti_lag
            || block.ecu.is_anti_lag_active(controls.throttle, rpm))
            && controls.throttle < 0.20
            && rpm >= 2_000.0;
        let anti_lag_fuel = if anti_lag_active { 25.0e-6 } else { 0.0 };
        let unburnt_fuel_mass = block.model.trapped_fuel_mass(at_evo.mass, burned_at_evo)
            + tip_in
            + dead_plug_fuel
            + anti_lag_fuel;
        let manifold_temperature = if anti_lag_active {
            manifold_temperature.max(1_050.0)
        } else {
            manifold_temperature
        };

        let (gamma, gas_constant) = block
            .exhaust_banks
            .first()
            .map(|bank| (bank.plenum.gamma, bank.plenum.gas_constant))
            .unwrap_or((1.33, 287.0));

        // Friction mean effective pressure, straight off the same Chen-Flynn
        // correlation the solver bills the crankshaft for. It is the block's own
        // estimate of what it is spending on itself, and every term in it —
        // bearing shear, ring drag, windage, the valvetrain — is also a noise
        // source, which is what the audio thread uses it for.
        let peak_pressure = block.ring.peak_pressure();
        let friction_mep = block.friction.fmep(
            peak_pressure,
            block.model.geometry.mean_piston_speed(rpm.abs()),
            block.thermal.oil_temperature(),
        );

        // An atmospheric engine reports a dead shaft, which is what silences
        // the whistle and the flutter downstream.
        let (turbo_rpm, turbo_surge) = match self.turbo.as_mut() {
            Some(turbo) => (turbo.update(dt, rpm, &controls), turbo.surge(&controls)),
            None => (0.0, 0.0),
        };

        let knock_intensity = (block.master.knock_integral - 1.0).max(0.0) as f32;
        let bore = block.model.geometry.bore as f32;
        let peak_cylinder_pressure = if controls.spark_cut {
            0.0
        } else {
            peak_pressure as f32
        };

        // The cycle itself, downsampled off the phase ring and cut at EVO. This
        // is what the audio thread plays back at crank rate instead of
        // synthesising a pulse shape: the blowdown edge, the exhaust stroke and
        // the induction gulp are the solver's own curves, so an engine with a
        // different cam sounds different without a constant anywhere saying so.
        let cylinder_pressure = block.ring.downsample_from(self.evo_angle, |s| s.pressure);
        // The ring carries port flux positive *into* the cylinder. The exhaust
        // side is flipped so that positive means "leaving through the port",
        // which is the direction the runner sees, and left signed: gas pushed
        // back through an open valve is a real event, not a shut port.
        let exhaust_port_flow = block
            .ring
            .downsample_from(self.evo_angle, |s| -s.exhaust_flow);
        let intake_port_flow = block
            .ring
            .downsample_from(self.evo_angle, |s| s.intake_flow.max(0.0));
        // Valve lift is a pure function of crank angle, so it needs no ring —
        // but it needs the *same* cut and the same box average as the tables
        // that do, or the port opens at a different instant from the pulse that
        // leaves through it. These are the boundary conditions at the head of
        // every runner in both networks; without them a runner is a pipe with a
        // rigid plug in the end of it, which reflects everything and rings.
        let valves = block.model.valves;
        let exhaust_valve_area = crate::physics::engine_block::PhaseRing::downsample_angles_from(
            self.evo_angle,
            |theta| valves.exhaust.effective_area(theta),
        );
        let intake_valve_area = crate::physics::engine_block::PhaseRing::downsample_angles_from(
            self.evo_angle,
            |theta| valves.intake.effective_area(theta),
        );
        // What is behind those valves when they open. Also a pure function of
        // crank angle, cut and averaged the same way, because a boundary
        // condition made of two tables that disagree about where the cycle
        // starts is worse than either of them alone.
        let geometry = block.model.geometry;
        let cylinder_volume = crate::physics::engine_block::PhaseRing::downsample_angles_from(
            self.evo_angle,
            |theta| geometry.safe_volume(theta),
        );

        // The exhaust section by section. Each primary is at the temperature its
        // own wall has let its gas reach, so an engine whose header is still
        // cold resonates low and climbs as it warms — and the collector and the
        // tailpipe, further down the gradient, are cooler again.
        let mut primary_temperature = [0.0f32; MAX_CYLINDERS];
        for (i, slot) in primary_temperature.iter_mut().enumerate() {
            *slot = block.thermal.exhaust.primary_gas(i) as f32;
        }

        let n_cylinders = block.firing.len().max(1) as f64;
        let cycle_work = block.ring.indicated_work(block.crankcase_pressure) * n_cylinders;
        let mean_indicated_torque = cycle_work / crate::physics::cylinder::CYCLE_ANGLE;

        EngineSnapshot {
            rpm: rpm as f32,
            blowdown_delta,
            exhaust_temperature: manifold_temperature as f32,
            primary_temperature,
            collector_temperature: block.thermal.exhaust.collector_gas() as f32,
            tailpipe_temperature: block.thermal.exhaust.tailpipe_gas() as f32,
            exhaust_gamma: gamma as f32,
            exhaust_gas_constant: gas_constant as f32,
            intake_mass_flow: intake_mass_flow as f32,
            cylinder_intake_flow,
            throttle: controls.throttle as f32,
            turbo_rpm: turbo_rpm as f32,
            turbo_surge: turbo_surge as f32,
            unburnt_fuel_mass: unburnt_fuel_mass as f32,
            friction_mep: friction_mep as f32,
            cold_fraction: block.thermal.cold_fraction() as f32,
            starter_hz: controls.starter_hz as f32,
            spark_cut,
            knock_intensity,
            bore,
            peak_cylinder_pressure,
            indicated_torque: mean_indicated_torque as f32,
            inertia: self.inertia as f32,
            cylinder_pressure,
            exhaust_port_flow,
            intake_port_flow,
            exhaust_valve_area,
            intake_valve_area,
            cylinder_volume,
            exhaust_manifold_pressure: manifold_pressure as f32,
            exhaust_cutout: controls.exhaust_cutout,
            anti_lag: anti_lag_active,
        }
        .sanitized()
    }
}

impl SynthConfig {
    /// Builds a synth configuration from a block's firing order and banks.
    ///
    /// Each cylinder's trigger phase is where the *master* crank has to be for
    /// that cylinder to reach EVO. The block runs one master cylinder and reads
    /// the others out of the phase ring at an offset, so cylinder `i` sees angle
    /// `theta_master - firing_offset_i`; it therefore reaches EVO when the
    /// master is at `theta_EVO + firing_offset_i`.
    pub fn from_block(block: &EngineBlock, sample_rate: f32) -> Self {
        let evo = block.model.valves.exhaust.open_angle;
        let bank_count = block.firing.bank_count().max(1);
        // A block is as long as its longest bank, whatever the other one does.
        let cylinders_per_bank = (0..bank_count)
            .map(|bank| block.firing.cylinders_on_bank(bank as u8).len())
            .max()
            .unwrap_or(1);

        let cylinders = block
            .firing
            .cylinders
            .iter()
            .map(|cylinder| CylinderTap {
                evo_phase: (wrap_cycle(evo + cylinder.firing_offset) / CYCLE_ANGLE) as f32,
                bank: (cylinder.bank as usize) % bank_count,
            })
            .collect();

        let mut uniform = Self::uniform(sample_rate, block.firing.len().max(1), bank_count);
        uniform.mechanical.float_rpm =
            Some(crate::physics::cylinder::default_float_rpm(block.ecu.redline) as f32);
        Self {
            cylinders,
            bank_count,
            exhaust: block.exhaust.clone(),
            intake: block.intake_system.clone(),
            structure: StructuralSpec::new(
                block.block_mass,
                block.model.geometry.bore,
                cylinders_per_bank,
            ),
            reciprocating: block.model.geometry,
            ..uniform
        }
    }

    /// Fits — or removes — a turbo.
    ///
    /// The counterpart of [`SnapshotSource::with_induction`]: that one decides
    /// whether a shaft spins, this one decides whether anything is listening to
    /// it. Both have to agree, which is why a caller normally sets them from
    /// the same [`Induction`].
    pub fn with_induction(mut self, induction: Induction) -> Self {
        self.turbo = induction.voice();
        self.roots = induction.roots_voice();
        self.centrifugal = induction.centrifugal_voice();
        self.blow_off = induction.blow_off_voice();
        self.wastegate = induction.wastegate_voice();
        self
    }

    /// Fits an atmospheric blow-off / dump valve.
    pub fn with_blow_off(mut self, blow_off: BlowOffVoicing) -> Self {
        self.blow_off = Some(blow_off);
        self
    }

    /// Fits wastegate chatter.
    pub fn with_wastegate(mut self, wastegate: WastegateVoicing) -> Self {
        self.wastegate = Some(wastegate);
        self
    }
}

#[cfg(test)]
mod tests;
