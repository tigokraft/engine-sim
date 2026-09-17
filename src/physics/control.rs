//! Engine control unit: fuelling, ignition timing, knock closed-loop control,
//! limiter modes, and per-cylinder health.
//!
//! In a real engine, air-fuel ratio and spark advance are not constants — they
//! are dynamic control schedules mapped against engine speed and manifold load.
//!
//! - **Fuelling schedule**: Stoichiometric at idle/part-load, WOT enrichment
//!   near 12.5:1 for peak torque and knock-suppressing charge cooling, lean
//!   cruise (~15.5:1) for economy, transient acceleration enrichment, and
//!   deceleration fuel cut-off (DFCO) on overrun.
//! - **Ignition schedule**: Spark advance mapped against speed and load, with
//!   closed-loop knock retard driving the Livengood–Wu integral back below 1.
//! - **Limiter modes**: Hard cut, soft progressive cut, rotating stutter, with
//!   selectable fuel-cut versus spark-cut behaviour.
//! - **Per-cylinder health**: Individual cylinder spark and fuel health, allowing
//!   a fouled plug or dead injector to produce a realistic hole in the firing
//!   pattern.

use crate::physics::cylinder::{deg, wrap_cycle};
use crate::physics::thermodynamics::STOICH_AFR;

/// Maximum number of cylinders supported by the control unit.
pub const MAX_CYLINDERS: usize = 16;

/// Starter stall torque per litre of displacement, against the engine's drag [N m / L].
///
/// The first half of the sizing rule in [`Starter::for_engine`]: enough motor
/// to overcome what the whole engine costs to turn at all. Sized so the crank
/// settles a few hundred rev/min on a cold engine and climbs as the oil thins.
pub const STARTER_TORQUE_PER_LITRE: f64 = 40.0;

/// Margin a starter is sized over the compression it must cross [-].
///
/// The other half of the rule. A motor that can only just hold the engine
/// against its worst compression never gets past it; fifteen per cent over is
/// what turns "holds" into "turns".
pub const STARTER_HUMP_MARGIN: f64 = 1.15;

/// Crank angle a compression stroke is resisted over [rad].
///
/// Half a revolution nominally, but the gas only pushes back over the part of
/// it where the pressure has risen, which is the last quarter — so a quarter
/// revolution is what the flywheel actually has to bridge, and what the
/// starter's torque is integrated over in [`Starter::for_engine`].
pub const COMPRESSION_ANGLE: f64 = std::f64::consts::FRAC_PI_2;

/// Cranking speed a starter is sized to reach [rev/min].
///
/// Where the flywheel's stored energy is evaluated when the motor is specified.
/// Not a speed anything is held at: the crank finds its own, and this only sets
/// how much help the flywheel is assumed to be when the motor is chosen.
pub const STARTER_DESIGN_CRANK_RPM: f64 = 250.0;

/// Crank speed a starter runs to with nothing on the pinion [rev/min].
///
/// The other end of the machine's speed-torque line, and also the speed its
/// one-way clutch gives up at: past this the ring gear is driving the pinion
/// instead of the other way round. See [`Starter::update`].
pub const STARTER_FREE_SPEED: f64 = 450.0;

/// Ring gear teeth, which is the starter whine's order at the crank [-].
pub const STARTER_WHINE_ORDER: f64 = 129.0;

/// How long the engine must outrun the pinion before it is withdrawn [s].
///
/// Not a debounce. A one-way clutch freewheels the instant the ring gear gets
/// ahead of the pinion, and an engine that has not fired at all does that
/// between compression strokes — a single-cylinder engine spends two whole
/// revolutions accelerating into nothing before its next compression, and
/// passes the motor's own free speed doing it. What actually pulls the pinion
/// out is the solenoid dropping, and what drops the solenoid is somebody
/// deciding the engine is running. About a fifth of a second of continuous
/// overrun is that decision, and it is the same one a driver makes by ear.
pub const STARTER_RELEASE_HOLD: f64 = 0.20;

/// Speed drop below free speed tolerated during overrun hold [rev/min].
///
/// An engine catching cold has intra-cycle ripple: firing pulses accelerate the
/// crank past [`STARTER_FREE_SPEED`], but compression strokes before the next
/// chamber fires decelerate the crank by a few rev/min. Once continuous overrun
/// has started, allowing this modest margin prevents cyclic compression ripple
/// from resetting the release hold timer every stroke.
pub const STARTER_OVERRUN_HYSTERESIS_RPM: f64 = 2.5;

/// Fraction of commanded fuel that lands on a stone-cold port wall [-].
///
/// Petrol sprayed at a port that is at ambient does not all stay airborne: most
/// of the droplet mass hits the back of the valve and the port floor and stays
/// there as a liquid film. It is not lost — it evaporates and goes in later —
/// but "later" is a different cycle from the one it was metered for.
pub const COLD_WALL_FILM_FRACTION: f64 = 0.80;

/// Evaporation time constant of the port wall film on a stone-cold engine [s].
///
/// How long the film takes to give back what landed on it. Scales down with
/// the block temperature: a hot port boils fuel off it as fast as it arrives,
/// which is why a warm engine has no film and no lag at all.
pub const COLD_WALL_FILM_TAU: f64 = 0.90;

/// How much richer a stone-cold engine is commanded than a warm one [-].
///
/// Choke, by another name. The calibration knows the film is about to swallow
/// half of what it meters, so it meters more — and the two do not cancel,
/// because the enrichment is instant and the film is not.
pub const COLD_ENRICHMENT: f64 = 0.45;

/// Air-fuel ratio past which a homogeneous charge will not light [-].
///
/// The lean flammability limit of a petrol-air mixture in a cylinder, near
/// enough. Past it the spark makes a kernel and the kernel goes out, which
/// leaves a cylinder full of fuel and air going out of the exhaust valve
/// unburnt — the misfire that makes a cold engine stumble on its first few
/// firings, and the fuel that makes it pop once the pipe has warmed up.
pub const LEAN_MISFIRE_AFR: f64 = 20.5;

/// Crank speed below which the ECU has no signal to meter fuel against [rev/min].
///
/// An engine that is barely moving has no usable crank signal, so nothing is
/// injected and the cylinders are pure gas springs. Past it the ECU has sync,
/// fuel is metered on every cycle, and the first charge to find a spark at a
/// fireable condition is the one that catches.
pub const CRANK_SYNC_RPM: f64 = 80.0;

/// Limiter cut mechanism.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum LimiterCut {
    /// No cut active.
    None,
    /// Fuel cut: injectors disabled. No fuel reaches the exhaust, preventing backfires.
    Fuel,
    /// Spark cut: ignition disabled. Unburnt fuel enters the exhaust, enabling backfires.
    Spark,
}

/// Rev limiter intervention strategy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum LimiterMode {
    /// Hard cut: all cylinders cut simultaneously at or above redline.
    HardCut,
    /// Soft progressive cut: ignition retard and partial cutting in a margin below redline.
    SoftCut,
    /// Rotating per-cylinder stutter: cuts cylinders in round-robin sequence.
    RotatingStutter,
}

/// Individual cylinder operating health.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CylinderHealth {
    /// Whether spark ignition occurs normally.
    pub spark_ok: bool,
    /// Whether fuel injection occurs normally.
    pub fuel_ok: bool,
}

impl Default for CylinderHealth {
    fn default() -> Self {
        Self::healthy()
    }
}

impl CylinderHealth {
    /// A fully operational cylinder.
    pub fn healthy() -> Self {
        Self {
            spark_ok: true,
            fuel_ok: true,
        }
    }

    /// A fouled or disconnected spark plug: fuel is injected but never ignites.
    pub fn dead_plug() -> Self {
        Self {
            spark_ok: false,
            fuel_ok: true,
        }
    }

    /// A clogged or failed injector: no fuel delivered.
    pub fn dead_injector() -> Self {
        Self {
            spark_ok: true,
            fuel_ok: false,
        }
    }

    /// Completely dead cylinder: no fuel and no spark.
    pub fn dead() -> Self {
        Self {
            spark_ok: false,
            fuel_ok: false,
        }
    }

    /// Relative combustion power factor `[0.0, 1.0]`.
    pub fn combustion_factor(&self) -> f32 {
        if self.spark_ok && self.fuel_ok {
            1.0
        } else {
            0.0
        }
    }
}

/// The starter motor: the only thing that turns an engine which is not running.
///
/// A series-wound machine on a one-way clutch, described by the two numbers
/// that are its own — the torque it makes held still, and the speed it runs to
/// with nothing on the pinion. Between them the torque falls linearly, which is
/// close enough to a series motor's curve over the narrow part of it a starter
/// ever visits, and it is the *slope* that matters here rather than the shape:
/// a motor that makes more torque the harder it is held is a motor that cannot
/// be stalled by one compression stroke, only slowed by it.
///
/// # Nothing here shapes a cranking sound
///
/// There is no envelope in this struct and no chug in it. The chug is what
/// comes out when this torque meets
/// [`EngineBlock::instantaneous_indicated_torque`](crate::physics::engine_block::EngineBlock::instantaneous_indicated_torque):
/// the crank slows going up every compression stroke, the motor's torque rises
/// as it slows, and it drags the engine over the top and is given most of it
/// back down the other side. That is one chug per firing interval, at a rate
/// nobody wrote down, and it speeds up on a warm engine on its own because
/// thin oil is less friction for the same motor to work against.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Starter {
    /// Torque at the crankshaft with the pinion held still [N m].
    pub stall_torque: f64,
    /// Crankshaft speed the motor runs to with nothing to turn [rev/min].
    pub free_speed: f64,
    /// Tooth-mesh order of the pinion against the ring gear [per crank rev].
    pub whine_order: f64,
    /// Whether the pinion is meshed with the ring gear.
    pub engaged: bool,
    /// How long the engine has been continuously outrunning the pinion [s].
    pub overrun_time: f64,
}

impl Default for Starter {
    fn default() -> Self {
        Self {
            stall_torque: STARTER_TORQUE_PER_LITRE * 2.0,
            free_speed: STARTER_FREE_SPEED,
            whine_order: STARTER_WHINE_ORDER,
            engaged: false,
            overrun_time: 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Idle governor
// ---------------------------------------------------------------------------

/// Proportional gain on speed error [bypass travel per rev/min].
///
/// A hundred rpm low opens four thousandths of the bypass' travel, which
/// sounds like nothing until you notice how steep the other side of the loop
/// is: near idle a percent more bypass area is worth several hundred rpm, so
/// this is already most of the gain the loop can carry. Tuned on the stock
/// cam — a governor is calibrated on the engine it ships with. It is the
/// default [`IdleGovernor`] falls back to; a preset with more cam overlap or
/// less flywheel to damp it gets its own gains via [`IdleGovernor::tuned`]
/// instead of inheriting this one unexamined. See
/// [`crate::bench::EnginePreset::idle_governor`].
pub const IDLE_GOVERNOR_PROPORTIONAL: f64 = 4.0e-5;

/// Integral gain on speed error [bypass travel per rev/min per second].
///
/// Three times the proportional gain, which is a reset time of a third of a
/// second — the usual figure for a production idle loop. It is what removes
/// the droop a purely proportional governor has to live with, so the engine
/// idles at the speed it is asked for rather than a little under it. It is
/// also what lets the loop wind *past* that speed, which is the half of this
/// that matters here: an integrator plus a lag is the whole recipe for
/// overshoot, and overshoot is the whole recipe for a lope.
pub const IDLE_GOVERNOR_INTEGRAL: f64 = 1.2e-4;

/// Time constant of the bypass actuator and the air path behind it [s].
///
/// A stepper valve takes a tenth of a second to move, the manifold behind it
/// takes another to fill, and the cylinder that fills from it does not make
/// torque until it has finished a compression and an expansion stroke. All
/// three are the same lag as far as the loop is concerned and this is their
/// sum. **It is not a smoothing filter and must not be tuned like one**: it
/// is the phase lag that decides whether the governor settles or hunts, and
/// removing it removes the lope.
pub const IDLE_ACTUATOR_LAG: f64 = 0.18;

/// A PI idle governor on an air bypass, with an actuator that lags.
///
/// The thing a lopey idle was missing. The speed an engine idles at is the
/// speed at which the air the governor is letting past the throttle plate
/// makes exactly the torque the engine's own drag is taking away, and a
/// governor holds that point by measuring the error and moving a valve. Both
/// of those take time: the valve has mass, the manifold behind it has volume,
/// and the cylinder that fills from it is two strokes away from making the
/// torque that answers. Feed a controller with integral action through that
/// much phase lag into a torque curve steep enough and it does not settle —
/// it overshoots, is corrected, and overshoots the other way, for ever. That
/// is a lope, and it is why an engine with a cam too big for its idle hunts
/// while a stock one does not.
///
/// It is deliberately allowed to fail to converge. Clamping the error, or
/// slugging the actuator until it cannot overshoot, would make every engine
/// idle like a stock one, which is the bug this replaced.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IdleGovernor {
    /// Proportional gain [bypass travel per rev/min].
    pub proportional: f64,
    /// Integral gain [bypass travel per rev/min per second].
    pub integral: f64,
    /// Actuator and air-path time constant [s].
    pub actuator_lag: f64,
    /// Most bypass travel the governor is allowed to command [-].
    ///
    /// A real idle valve runs out of travel, and an engine that needs more air
    /// than it has travel for does not idle. That is a correct outcome and the
    /// clamp is how it happens.
    pub authority: f64,
    /// Accumulated speed error [rev/min s].
    pub error_integral: f64,
    /// Where the controller is asking the valve to be [-].
    pub command: f64,
    /// Where the valve actually is, one lag behind the command [-].
    pub position: f64,
}

impl Default for IdleGovernor {
    fn default() -> Self {
        Self {
            proportional: IDLE_GOVERNOR_PROPORTIONAL,
            integral: IDLE_GOVERNOR_INTEGRAL,
            actuator_lag: IDLE_ACTUATOR_LAG,
            authority: 1.0,
            error_integral: 0.0,
            command: 0.0,
            position: 0.0,
        }
    }
}

impl IdleGovernor {
    /// Builds a governor from its own gains rather than the stock ones.
    ///
    /// A real idle loop is calibrated on the engine it ships with, not
    /// borrowed from another one: more cam overlap or less flywheel changes
    /// how much a given bypass command is worth in rpm and how fast that
    /// shows up, so the gains that hold a mild engine's idle steady can
    /// overshoot into a lope, or fail to catch a WOT lift at all, on a
    /// different plant. This is that calibration, with a freshly reset
    /// actuator and no accumulated error.
    pub fn tuned(proportional: f64, integral: f64, actuator_lag: f64, authority: f64) -> Self {
        Self {
            proportional,
            integral,
            actuator_lag,
            authority,
            ..Self::default()
        }
    }
}

impl Starter {
    /// The motor this engine would be fitted with.
    ///
    /// Two requirements, and the motor has to meet both. It has to turn the
    /// engine against its steady drag at all, which scales with `displacement`
    /// [m^3]; and it has to get the crank over `peak_hump` [N m], the deepest
    /// the gas torque goes anywhere in a motored cycle. The second is where
    /// `inertia` [kg m^2] comes in: a flywheel arrives at a compression with
    /// energy already in it and gives all of that back on the way up, so the
    /// motor only has to find what is left over. That is why a six-litre V12
    /// is cranked by a modest motor and a single-cylinder thumper is not — the
    /// V12's compressions overlap and its flywheel is heavy, and the thumper's
    /// hump arrives once every two revolutions with nothing behind it.
    ///
    /// Both terms, and the larger wins. Sizing on drag alone leaves an engine
    /// the motor cannot get over TDC; sizing on the hump alone bolts a crane
    /// motor to a V12 that never needed one.
    pub fn for_engine(displacement: f64, peak_hump: f64, inertia: f64) -> Self {
        let drag_sized = STARTER_TORQUE_PER_LITRE * (displacement * 1_000.0).max(0.1);
        let design_omega = STARTER_DESIGN_CRANK_RPM * std::f64::consts::PI / 30.0;
        let flywheel = 0.5 * inertia.max(0.0) * design_omega * design_omega / COMPRESSION_ANGLE;
        let hump_sized = STARTER_HUMP_MARGIN * peak_hump.max(0.0) - flywheel;
        Self {
            stall_torque: drag_sized.max(hump_sized),
            ..Self::default()
        }
    }

    /// Torque the motor is putting into the crankshaft at this speed [N m].
    ///
    /// Zero when the pinion is out, and zero past [`Self::free_speed`] — a
    /// motor cannot drive a shaft that is already turning faster than it wants
    /// to, and the one-way clutch means it cannot be driven by one either.
    pub fn torque(&self, rpm: f64) -> f64 {
        if !self.engaged {
            return 0.0;
        }
        (self.stall_torque * (1.0 - rpm.max(0.0) / self.free_speed.max(1.0))).max(0.0)
    }

    /// Meshes the pinion: the key going to START.
    pub fn engage(&mut self) {
        self.engaged = true;
        self.overrun_time = 0.0;
    }

    /// Throws the pinion out once the engine is plainly running on its own.
    ///
    /// The condition is the machine's own, not a flag set by whatever decided
    /// the engine had caught: the engine has to be outrunning the motor, and it
    /// has to keep doing it for [`STARTER_RELEASE_HOLD`]. An engine that has
    /// fired holds that for as long as anyone cares to watch, because it is
    /// accelerating away; an engine merely coasting between compression strokes
    /// gives it back on the next one and the motor stays in, which is what a
    /// real one does to an engine that will not start.
    pub fn update(&mut self, rpm: f64, dt: f64) {
        if !self.engaged {
            return;
        }
        // Speed ripple between firing strokes dips slightly below free speed on
        // compression; once overrun has been established, allow a modest margin
        // so intra-cycle ripple on an engine that has caught does not reset the
        // release timer every stroke.
        let threshold = if self.overrun_time > 0.0 {
            self.free_speed - STARTER_OVERRUN_HYSTERESIS_RPM
        } else {
            self.free_speed
        };
        if rpm > threshold {
            self.overrun_time += dt;
            if self.overrun_time >= STARTER_RELEASE_HOLD && rpm >= self.free_speed {
                self.engaged = false;
            }
        } else {
            self.overrun_time = 0.0;
        }
    }

    /// Tooth-mesh frequency of the pinion against the ring gear [Hz].
    ///
    /// Zero with the pinion out, which is what silences the whine: the starter
    /// is not faded down on release, it stops being meshed with anything.
    pub fn whine_hz(&self, rpm: f64) -> f64 {
        if !self.engaged {
            return 0.0;
        }
        self.whine_order * rpm.max(0.0) / 60.0
    }
}

impl IdleGovernor {
    /// Advances the controller and its actuator one frame, returning the
    /// bypass position the engine will actually breathe through [-].
    ///
    /// The integrator is held whenever the command is against its own stop, so
    /// an engine being driven well over its idle by the pedal does not spend
    /// that time winding the integral down into a hole it has to climb back
    /// out of before it can catch the engine on the way down.
    pub fn update(&mut self, target_rpm: f64, rpm: f64, dt: f64) -> f64 {
        if !(dt.is_finite() && dt > 0.0) {
            return self.position;
        }
        let error = target_rpm - rpm;
        let proposed = self.error_integral + error * dt;
        let unclamped = self.proportional * error + self.integral * proposed;
        // Conditional integration: only accumulate if doing so would not drive
        // the command further past a limit it has already reached.
        if (unclamped > 0.0 || proposed > self.error_integral)
            && (unclamped < self.authority || proposed < self.error_integral)
        {
            self.error_integral = proposed;
        }
        self.command = (self.proportional * error + self.integral * self.error_integral)
            .clamp(0.0, self.authority);

        // First-order lag towards the command. Exponential rather than a fixed
        // step so the lag is a time and not a frame count.
        let alpha = 1.0 - (-dt / self.actuator_lag.max(1e-4)).exp();
        self.position += (self.command - self.position) * alpha;
        self.position
    }

    /// Resets the controller to a shut valve and no history.
    pub fn reset(&mut self) {
        self.error_integral = 0.0;
        self.command = 0.0;
        self.position = 0.0;
    }
}

/// Time constant of the mean the hunt is measured against [s].
///
/// Slower than any lope worth the name and faster than a warm-up, so the mean
/// follows the idle the engine is settling towards without following the
/// swing about it.
pub const IDLE_HUNT_MEAN_TAU: f64 = 4.0;

/// Longest gap between crossings still counted as a hunt [s].
///
/// Past this the engine is not oscillating, it is drifting, and reporting a
/// six second "period" would be worse than reporting none.
pub const IDLE_HUNT_TIMEOUT: f64 = 6.0;

/// Smallest speed swing counted as a hunt rather than as combustion roughness
/// [rev/min].
///
/// Every engine's speed ripples at firing rate; a lope is something a listener
/// hears as a *rate*, and under a few rev either side of the mean there is no
/// rate to hear.
pub const IDLE_HUNT_FLOOR_RPM: f64 = 3.0;

/// Fraction of the last swing a crossing has to clear to count [-].
///
/// The trigger is a Schmitt, not a comparator. Speed crosses its own mean
/// several times per cycle on the firing ripple alone, and timing between
/// those gives the ripple's period rather than the lope's — which is how a
/// hundred-rev hunt comes back reported as a seven-rev one.
pub const IDLE_HUNT_HYSTERESIS: f64 = 0.25;

/// Measures the period and depth of an idle that will not settle.
///
/// A lope is a limit cycle, so it has a period, and a period is a number — the
/// point of this is that the chop can be measured rather than argued about.
/// Engine speed is compared against its own slow mean through a Schmitt
/// trigger, and the time from one rise through the upper threshold to the next
/// is the period; the extremes between them are the depth.
///
/// Both come back zero on an engine that has converged, which is the honest
/// answer for one: there is no period, not a very long one.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct IdleHunt {
    /// Slow mean of engine speed, the line the swing is measured about [rev/min].
    pub mean: f64,
    /// Time since the speed last rose through the upper threshold [s].
    seconds_since_crossing: f64,
    /// Whether the trigger is currently latched high.
    latched_high: bool,
    /// Extremes seen since that crossing [rev/min].
    peak: f64,
    trough: f64,
    /// Whether a first crossing has happened, so an interval can be timed.
    started: bool,
    /// Period of the limit cycle, or `0` when the idle has settled [s].
    pub period: f64,
    /// Peak-to-peak speed swing over that period [rev/min].
    pub amplitude: f64,
}

impl IdleHunt {
    /// Feeds one frame of engine speed to the detector.
    pub fn observe(&mut self, rpm: f64, dt: f64) {
        if !(dt.is_finite() && dt > 0.0 && rpm.is_finite()) {
            return;
        }
        if self.mean <= 0.0 {
            self.mean = rpm;
            self.peak = rpm;
            self.trough = rpm;
            return;
        }
        self.mean += (rpm - self.mean) * (1.0 - (-dt / IDLE_HUNT_MEAN_TAU).exp());
        self.peak = self.peak.max(rpm);
        self.trough = self.trough.min(rpm);
        self.seconds_since_crossing += dt;

        // Sized from the swing already measured, so the trigger widens with
        // the lope it is following and stays narrow on an engine that is only
        // rippling.
        let band = (IDLE_HUNT_HYSTERESIS * self.amplitude).max(IDLE_HUNT_FLOOR_RPM);
        if self.latched_high {
            if rpm < self.mean - band {
                self.latched_high = false;
            }
        } else if rpm > self.mean + band {
            self.latched_high = true;
            if self.started {
                // One-pole over successive cycles: a limit cycle in a system
                // this nonlinear is periodic on average rather than exactly,
                // and a figure that jumps every cycle is not a measurement.
                let blend = 0.35;
                self.period += (self.seconds_since_crossing - self.period) * blend;
                self.amplitude += (self.peak - self.trough - self.amplitude) * blend;
            } else {
                self.period = self.seconds_since_crossing;
                self.amplitude = self.peak - self.trough;
                self.started = true;
            }
            self.seconds_since_crossing = 0.0;
            self.peak = rpm;
            self.trough = rpm;
        }

        // Checked outside the trigger, not inside its idle arm: a latch that
        // is stuck high because the band grew wider than the swing still has
        // to time out, and that is exactly the state an engine lands in when
        // it stops hunting.
        if self.seconds_since_crossing > IDLE_HUNT_TIMEOUT {
            // Nothing has crossed in long enough that whatever it is, it is
            // not a limit cycle.
            self.period = 0.0;
            self.amplitude = 0.0;
            self.started = false;
            self.latched_high = false;
            self.seconds_since_crossing = 0.0;
            self.peak = rpm;
            self.trough = rpm;
        }
    }

    /// Forgets everything, for a driveline leaving idle.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Rate of the limit cycle, or `0` when the idle has settled [Hz].
    pub fn hunt_hz(&self) -> f64 {
        if self.period > 0.0 {
            1.0 / self.period
        } else {
            0.0
        }
    }
}

/// Complete engine control unit managing fuelling, timing, knock, and limiters.
#[derive(Debug, Clone, PartialEq)]
pub struct EngineControlUnit {
    // --- Fuelling calibration ---
    /// Stoichiometric air-fuel ratio [-].
    pub stoich_afr: f64,
    /// Wide-open-throttle enriched target air-fuel ratio [-].
    pub wot_afr: f64,
    /// Lean cruise target air-fuel ratio [-].
    pub cruise_afr: f64,
    /// Idle target air-fuel ratio [-].
    pub idle_afr: f64,
    /// Transient acceleration enrichment gain [-].
    pub accel_enrichment_gain: f64,
    /// Acceleration enrichment decay time constant [s].
    pub accel_enrichment_decay: f64,
    /// Whether deceleration fuel cut-off is enabled.
    pub dfco_enabled: bool,
    /// Minimum engine speed for DFCO engagement [rev/min].
    pub dfco_rpm_threshold: f64,
    /// Recovery engine speed below which DFCO disengages to maintain idle [rev/min].
    pub dfco_recover_threshold: f64,
    /// Closed-throttle threshold under which pedal is considered lifted [-].
    pub dfco_throttle_threshold: f64,
    /// Current state of DFCO.
    pub dfco_active: bool,
    /// Whether a throttle tip-in event occurred after DFCO.
    pub dfco_tip_in: bool,
    /// Unburnt fuel mass injected during a tip-in transient [kg].
    pub tip_in_fuel_mass: f64,
    /// Current acceleration enrichment offset on AFR [-].
    pub accel_enrichment: f64,
    /// Previous throttle position for derivative calculation [-].
    pub prev_throttle: f64,

    // --- Anti-lag calibration ---
    /// Whether anti-lag is enabled on lift/overrun.
    pub anti_lag: bool,
    /// Minimum engine speed to engage anti-lag [rev/min].
    pub anti_lag_rpm_threshold: f64,
    /// Spark retard angle past TDC applied during anti-lag [deg].
    pub anti_lag_retard: f64,
    /// Target air-fuel ratio during anti-lag overrun fuelling.
    pub anti_lag_afr: f64,

    // --- Ignition calibration ---
    /// Nominal base spark advance [deg BTDC].
    pub base_spark_advance: f64,
    /// Spark advance at idle [deg BTDC].
    pub idle_advance: f64,
    /// Maximum allowable spark advance [deg BTDC].
    pub max_advance: f64,
    /// Spark retard under wide-open-throttle load [deg].
    pub wot_retard: f64,

    // --- Knock closed-loop control ---
    /// Cumulative knock retard angle [deg].
    pub knock_retard: f64,
    /// Retard step per knock event [deg].
    pub knock_step: f64,
    /// Advance recovery rate [deg/s].
    pub knock_decay: f64,
    /// Maximum knock retard clamp [deg].
    pub max_knock_retard: f64,

    /// Manual spark advance trim from test console [deg BTDC].
    pub spark_trim: f64,
    /// Manual air-fuel ratio trim from test console [-].
    pub afr_trim: f64,

    /// Base Wiebe burn duration [rad].
    pub base_wiebe_duration: f64,

    // --- Limiter calibration ---
    /// Rev limiter speed ceiling [rev/min].
    pub redline: f64,
    /// Intervention pattern mode.
    pub limiter_mode: LimiterMode,
    /// Cut mechanism (fuel vs spark).
    pub limiter_cut_type: LimiterCut,
    /// Speed margin below redline for soft cut engagement [rev/min].
    pub limiter_soft_margin: f64,
    /// Step counter for rotating cylinder stutter.
    pub stutter_counter: usize,
    /// Currently active limiter cut state.
    pub active_cut: LimiterCut,

    // --- Cylinder health ---
    /// Per-cylinder operational health.
    pub cylinder_health: [CylinderHealth; MAX_CYLINDERS],

    // --- Port wall film ---
    /// Fuel on the port walls, as a multiple of one cycle's commanded mass [-].
    pub wall_film: f64,
    /// Fuel reaching the cylinder, as a fraction of what was commanded [-].
    pub fuel_delivery: f64,
    /// Whether the charge this cycle is too lean to light.
    pub misfiring: bool,

    // --- Cranking ---
    /// Whether the engine is being turned by the starter rather than running.
    ///
    /// Below [`CRANK_SYNC_RPM`] the ECU has no crank signal to fuel against, so
    /// while this is set and the engine is that slow, nothing is metered and
    /// the cylinders are pure gas springs.
    pub cranking: bool,
    /// Whether the engine is being turned with the fuel deliberately off.
    ///
    /// What a compression test is, and what
    /// [`EngineBlock::prime_ring`](crate::physics::engine_block::EngineBlock::prime_ring)
    /// needs: a cycle of real gas with no combustion anywhere in it. Overrides
    /// the schedule outright rather than going through it.
    pub motoring: bool,
}

impl Default for EngineControlUnit {
    fn default() -> Self {
        Self::new(8_000.0)
    }
}

/// Flame speed multiplier relative to stoichiometric [-].
///
/// Gasoline flame speed peaks slightly rich at equivalence ratio phi ≈ 1.1
/// (AFR ≈ 13.3:1). At lean mixtures (phi < 1.0) or very rich mixtures (phi > 1.3),
/// flame speed falls and burn duration lengthens.
pub fn afr_flame_speed_factor(afr: f64) -> f64 {
    let phi = STOICH_AFR / afr.clamp(8.0, 25.0);
    // Normalized so phi = 1.0 (stoichiometric) gives 1.0, peaks near phi = 1.1
    (1.0 + 0.5 * (phi - 1.0) - 2.5 * (phi - 1.1).powi(2) + 0.025).clamp(0.4, 1.15)
}

impl EngineControlUnit {
    /// Builds an ECU with default calibrations for a given redline.
    pub fn new(redline: f64) -> Self {
        Self {
            stoich_afr: STOICH_AFR,
            wot_afr: 12.5,
            cruise_afr: 15.5,
            idle_afr: 14.7,
            accel_enrichment_gain: 1.5,
            accel_enrichment_decay: 0.15,
            dfco_enabled: true,
            dfco_rpm_threshold: 1_400.0,
            dfco_recover_threshold: 1_200.0,
            dfco_throttle_threshold: 0.03,
            dfco_active: false,
            dfco_tip_in: false,
            tip_in_fuel_mass: 30e-6,
            accel_enrichment: 0.0,
            prev_throttle: 0.0,

            anti_lag: false,
            anti_lag_rpm_threshold: 2_200.0,
            anti_lag_retard: 30.0,
            anti_lag_afr: 12.0,

            base_spark_advance: 25.0,
            idle_advance: 12.0,
            max_advance: 36.0,
            wot_retard: 6.0,

            knock_retard: 0.0,
            knock_step: 2.0,
            knock_decay: 1.0,
            max_knock_retard: 15.0,

            spark_trim: 0.0,
            afr_trim: 0.0,

            base_wiebe_duration: 1.0471975511965976, // 60 deg

            redline,
            limiter_mode: LimiterMode::HardCut,
            limiter_cut_type: LimiterCut::Spark,
            limiter_soft_margin: 200.0,
            stutter_counter: 0,
            active_cut: LimiterCut::None,

            cylinder_health: [CylinderHealth::healthy(); MAX_CYLINDERS],
            wall_film: 0.0,
            fuel_delivery: 1.0,
            misfiring: false,
            cranking: false,
            motoring: false,
        }
    }

    /// Trims the spark advance by delta degrees, clamped to +/- 15 deg.
    pub fn nudge_spark_trim(&mut self, delta: f64) {
        self.spark_trim = (self.spark_trim + delta).clamp(-15.0, 15.0);
    }

    /// Trims the target AFR by delta, clamped to +/- 3.0.
    pub fn nudge_afr_trim(&mut self, delta: f64) {
        self.afr_trim = (self.afr_trim + delta).clamp(-3.0, 3.0);
    }

    /// Resets live calibration trims to nominal factory values.
    pub fn reset_trims(&mut self) {
        self.spark_trim = 0.0;
        self.afr_trim = 0.0;
        self.cylinder_health = [CylinderHealth::healthy(); MAX_CYLINDERS];
    }

    /// Cycles through the available rev limiter modes.
    pub fn cycle_limiter_mode(&mut self) {
        self.limiter_mode = match self.limiter_mode {
            LimiterMode::HardCut => LimiterMode::SoftCut,
            LimiterMode::SoftCut => LimiterMode::RotatingStutter,
            LimiterMode::RotatingStutter => LimiterMode::HardCut,
        };
    }

    /// Cycles through the available rev limiter cut mechanisms (Spark vs Fuel vs None).
    pub fn cycle_limiter_cut(&mut self) {
        self.limiter_cut_type = match self.limiter_cut_type {
            LimiterCut::Spark => LimiterCut::Fuel,
            LimiterCut::Fuel => LimiterCut::None,
            LimiterCut::None => LimiterCut::Spark,
        };
    }

    /// Builder enabling or disabling anti-lag system.
    pub fn with_anti_lag(mut self, enabled: bool) -> Self {
        self.anti_lag = enabled;
        self
    }

    /// Builder setting the rev limiter mode (HardCut, SoftCut, RotatingStutter).
    pub fn with_limiter_mode(mut self, mode: LimiterMode) -> Self {
        self.limiter_mode = mode;
        self
    }

    /// Builder setting the rev limiter cut type (Spark, Fuel, None).
    pub fn with_limiter_cut_type(mut self, cut_type: LimiterCut) -> Self {
        self.limiter_cut_type = cut_type;
        self
    }

    /// Returns whether anti-lag is currently engaging (enabled, throttle closed, rpm above threshold).
    pub fn is_anti_lag_active(&self, throttle: f64, rpm: f64) -> bool {
        self.anti_lag
            && throttle <= self.dfco_throttle_threshold
            && rpm >= self.anti_lag_rpm_threshold
    }

    /// Returns the operational health of cylinder `i`.
    pub fn cylinder_health(&self, cylinder: usize) -> CylinderHealth {
        self.cylinder_health
            .get(cylinder)
            .copied()
            .unwrap_or_else(CylinderHealth::healthy)
    }

    /// Sets the operational health of cylinder `i`.
    pub fn set_cylinder_health(&mut self, cylinder: usize, health: CylinderHealth) {
        if cylinder < MAX_CYLINDERS {
            self.cylinder_health[cylinder] = health;
        }
    }

    /// Cycles cylinder operational health (Healthy -> DeadPlug -> DeadInjector -> Dead -> Healthy).
    pub fn toggle_cylinder_health(&mut self, cylinder: usize) {
        if cylinder < MAX_CYLINDERS {
            let current = self.cylinder_health[cylinder];
            self.cylinder_health[cylinder] = if current.spark_ok && current.fuel_ok {
                CylinderHealth::dead_plug()
            } else if !current.spark_ok && current.fuel_ok {
                CylinderHealth::dead_injector()
            } else if current.spark_ok && !current.fuel_ok {
                CylinderHealth::dead()
            } else {
                CylinderHealth::healthy()
            };
        }
    }

    /// Returns whether cylinder `i` has working spark.
    pub fn is_spark_ok(&self, cylinder: usize) -> bool {
        self.cylinder_health
            .get(cylinder)
            .is_none_or(|h| h.spark_ok)
    }

    /// Returns whether cylinder `i` has working fuel injection.
    pub fn is_fuel_ok(&self, cylinder: usize) -> bool {
        self.cylinder_health.get(cylinder).is_none_or(|h| h.fuel_ok)
    }

    /// Returns the combustion factor `[0.0, 1.0]` for cylinder `i`.
    pub fn cylinder_combustion_factor(&self, cylinder: usize) -> f32 {
        self.cylinder_health
            .get(cylinder)
            .map_or(1.0, |h| h.combustion_factor())
    }

    /// The mixture a cold engine is commanded, given the warm schedule's target.
    ///
    /// Exactly `afr` on a warm engine, by construction rather than by
    /// calibration: at `cold_fraction == 0` the divisor is one.
    pub fn cold_enriched_afr(&self, afr: f64, cold_fraction: f64) -> f64 {
        afr / (1.0 + COLD_ENRICHMENT * cold_fraction.clamp(0.0, 1.0))
    }

    /// Advances the port wall film and returns delivered over commanded fuel [-].
    ///
    /// The standard two-parameter film: a fraction `x` of what the injector
    /// meters lands on the wall instead of going in, and what is already on the
    /// wall evaporates with a time constant `tau`. Both scale with how cold the
    /// port is, so both vanish together on a warm engine — and at
    /// `cold_fraction == 0` this returns `commanded` itself, bit for bit, which
    /// is what keeps every fingerprint measured warm still valid.
    ///
    /// # It is a lag, not a loss
    ///
    /// In the steady state the film gives back exactly what it takes, so what
    /// reaches the cylinder is exactly what was metered however cold the engine
    /// is. Everything this costs is transient — and a start is the largest
    /// transient there is, because the film begins empty and the first cycles'
    /// fuel goes almost entirely onto the wall. That is why a cold engine
    /// cranks lean, stumbles, and catches a second later than the fuel says it
    /// should.
    ///
    /// The statement of that is the arithmetic itself: **what goes into the
    /// cylinder is what was metered, less whatever the wall is taking on this
    /// step**. Fuel is conserved by construction, so the steady state needs no
    /// cancellation to come out at one.
    ///
    /// # Why the film is integrated exactly
    ///
    /// `tau` goes to zero with the block temperature, and an explicit step of a
    /// lag shorter than the step itself does not converge — it oscillates, and
    /// it does so hardest at the temperatures where the film should be doing
    /// nothing at all. The closed form has no such limit and costs one
    /// exponential.
    pub fn update_wall_film(&mut self, commanded: f64, cold_fraction: f64, dt: f64) -> f64 {
        let cold = cold_fraction.clamp(0.0, 1.0);
        let deposited = COLD_WALL_FILM_FRACTION * cold;
        if deposited <= 0.0 {
            // A hot port is a dry port. Nothing is held and nothing is owed.
            self.wall_film = 0.0;
            self.fuel_delivery = commanded;
            return commanded;
        }
        let tau = (COLD_WALL_FILM_TAU * cold).max(1e-6);
        let settled = deposited * commanded * tau;
        let was = self.wall_film;
        self.wall_film = settled + (was - settled) * (-dt / tau).exp();
        let onto_wall = if dt > 0.0 {
            (self.wall_film - was) / dt
        } else {
            0.0
        };
        self.fuel_delivery = (commanded - onto_wall).max(0.0);
        self.fuel_delivery
    }

    /// Calibrated Wiebe duration adjusted for flame speed at the given AFR [rad].
    pub fn wiebe_duration(&self, afr: f64) -> f64 {
        let speed_factor = afr_flame_speed_factor(afr);
        self.base_wiebe_duration / speed_factor
    }

    /// Evaluates the rev limiter intervention for the current speed.
    ///
    /// - `HardCut`: 100% intervention when RPM >= redline.
    /// - `SoftCut`: progressive cut duty cycle in the margin below redline.
    /// - `RotatingStutter`: alternating cylinder cuts producing a rapid stutter pattern.
    pub fn evaluate_limiter(&mut self, rpm: f64) -> LimiterCut {
        self.stutter_counter = self.stutter_counter.wrapping_add(1);
        if self.limiter_cut_type == LimiterCut::None {
            self.active_cut = LimiterCut::None;
            return LimiterCut::None;
        }

        let is_cut = match self.limiter_mode {
            LimiterMode::HardCut => rpm >= self.redline,
            LimiterMode::SoftCut => {
                let margin = self.limiter_soft_margin.max(50.0);
                let threshold = self.redline - margin;
                if rpm < threshold {
                    false
                } else if rpm >= self.redline {
                    true
                } else {
                    let progress = (rpm - threshold) / margin;
                    let slot = self.stutter_counter % 8;
                    let cut_slots = (progress * 8.0).round() as usize;
                    slot < cut_slots
                }
            }
            LimiterMode::RotatingStutter => {
                if rpm >= self.redline {
                    self.stutter_counter.is_multiple_of(2)
                } else {
                    false
                }
            }
        };

        self.active_cut = if is_cut {
            self.limiter_cut_type
        } else {
            LimiterCut::None
        };
        self.active_cut
    }

    /// Evaluates deceleration fuel cut-off (DFCO) and tip-in states.
    ///
    /// When throttle is released (<= 0.03) at speeds above `dfco_rpm_threshold`,
    /// injectors are shut off entirely. As the engine slows below `dfco_recover_threshold`,
    /// or when throttle is reapplied, fuel is restored. Throttle reapplication triggers
    /// a tip-in unburnt fuel spike into the exhaust.
    pub fn update_dfco(&mut self, throttle: f64, rpm: f64) -> bool {
        if !self.dfco_enabled || self.is_anti_lag_active(throttle, rpm) {
            self.dfco_active = false;
            self.dfco_tip_in = false;
            return false;
        }

        if self.dfco_active {
            if throttle > self.dfco_throttle_threshold || rpm < self.dfco_recover_threshold {
                self.dfco_active = false;
                if throttle > self.dfco_throttle_threshold {
                    self.dfco_tip_in = true;
                }
            }
        } else {
            if throttle <= self.dfco_throttle_threshold && rpm > self.dfco_rpm_threshold {
                self.dfco_active = true;
                self.dfco_tip_in = false;
            } else if throttle > self.dfco_throttle_threshold {
                self.dfco_tip_in = false;
            }
        }

        self.dfco_active
    }

    /// Evaluates net spark advance [deg BTDC] against load and speed, including knock retard.
    ///
    /// Advance increases with engine speed to allow time for flame propagation at higher
    /// piston speeds. Light load adds vacuum advance for efficiency, while heavy load
    /// (high cylinder pressure/temperature) retards timing to protect against detonation.
    pub fn schedule_spark_advance(&self, load: f64, rpm: f64) -> f64 {
        self.schedule_spark_advance_with_throttle(load, rpm, self.prev_throttle)
    }

    /// Evaluates net spark advance [deg BTDC] against load, speed and throttle,
    /// deeply retarding past TDC when anti-lag is active.
    pub fn schedule_spark_advance_with_throttle(&self, load: f64, rpm: f64, throttle: f64) -> f64 {
        if self.is_anti_lag_active(throttle, rpm) {
            return -self.anti_lag_retard;
        }

        let rpm_frac = ((rpm - 800.0) / 5_200.0).clamp(0.0, 1.0);
        let speed_advance =
            self.idle_advance + rpm_frac * (self.base_spark_advance - self.idle_advance);

        let load_offset = if load < 0.5 {
            (0.5 - load) / 0.5 * 6.0
        } else if load > 0.7 {
            -((load - 0.7) / 0.3).min(1.0) * self.wot_retard
        } else {
            0.0
        };

        let total = speed_advance + load_offset - self.knock_retard + self.spark_trim;
        total.clamp(-10.0, self.max_advance)
    }

    /// Evaluates Wiebe spark angle [rad, cycle coords] from load and speed.
    pub fn spark_angle(&self, load: f64, rpm: f64) -> f64 {
        self.spark_angle_with_throttle(load, rpm, self.prev_throttle)
    }

    /// Evaluates Wiebe spark angle [rad, cycle coords] from load, speed, and throttle.
    pub fn spark_angle_with_throttle(&self, load: f64, rpm: f64, throttle: f64) -> f64 {
        let advance = self.schedule_spark_advance_with_throttle(load, rpm, throttle);
        wrap_cycle(deg(360.0 - advance))
    }

    /// Updates closed-loop knock retard from cycle knock status.
    ///
    /// When autoignition is detected (knock integral >= 1.0), timing is
    /// stepped back immediately by `knock_step` degrees (fast retard to suppress knock).
    /// Once the engine is operating cleanly without knock, timing slowly advances
    /// back towards the base map at `knock_decay` degrees per second.
    pub fn update_knock_retard(&mut self, knocked: bool, dt: f64) {
        if knocked {
            self.knock_retard = (self.knock_retard + self.knock_step).min(self.max_knock_retard);
        } else if self.knock_retard > 0.0 && dt > 0.0 {
            self.knock_retard = (self.knock_retard - self.knock_decay * dt).max(0.0);
        }
    }

    /// Evaluates target AFR from load and speed, applying WOT enrichment,
    /// lean cruise, and transient acceleration enrichment.
    pub fn schedule_afr(&mut self, load: f64, rpm: f64, throttle: f64, dt: f64) -> f64 {
        if dt > 1e-6 {
            let throttle_rate = (throttle - self.prev_throttle) / dt;
            if throttle_rate > 0.2 {
                self.accel_enrichment = (self.accel_enrichment
                    + self.accel_enrichment_gain * throttle_rate * dt)
                    .min(2.5);
            } else {
                let decay = (-dt / self.accel_enrichment_decay.max(1e-3)).exp();
                self.accel_enrichment *= decay;
            }
        }
        self.prev_throttle = throttle;

        let target = self.target_afr(load, rpm, throttle);
        (target - self.accel_enrichment).clamp(10.0, 20.0)
    }

    /// Base steady-state target AFR against load, speed and throttle.
    pub fn target_afr(&self, load: f64, rpm: f64, throttle: f64) -> f64 {
        // Anti-lag overrun fuelling: rich mixture into the exhaust manifold
        if self.is_anti_lag_active(throttle, rpm) {
            return (self.anti_lag_afr + self.afr_trim).clamp(9.0, 22.0);
        }

        // High load or wide throttle: WOT enrichment for peak power and charge cooling
        let base = if throttle >= 0.70 || load >= 0.85 {
            let t_blend = ((throttle - 0.70) / 0.25).clamp(0.0, 1.0);
            let l_blend = ((load - 0.70) / 0.25).clamp(0.0, 1.0);
            let wot_blend = t_blend.max(l_blend);
            self.stoich_afr + wot_blend * (self.wot_afr - self.stoich_afr)
        } else if throttle < 0.50
            && (1_500.0..=3_800.0).contains(&rpm)
            && (0.25..=0.65).contains(&load)
        {
            let rpm_factor = (1.0 - ((rpm - 2_650.0) / 1_150.0).abs()).clamp(0.0, 1.0);
            let load_factor = (1.0 - ((load - 0.45) / 0.20).abs()).clamp(0.0, 1.0);
            let cruise_blend = rpm_factor * load_factor;
            self.stoich_afr + cruise_blend * (self.cruise_afr - self.stoich_afr)
        } else {
            self.idle_afr
        };

        (base + self.afr_trim).clamp(9.0, 22.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_ecu_is_healthy_and_calibrated() {
        let ecu = EngineControlUnit::default();
        assert_eq!(ecu.stoich_afr, 14.7);
        assert_eq!(ecu.wot_afr, 12.5);
        assert_eq!(ecu.cruise_afr, 15.5);
        for i in 0..MAX_CYLINDERS {
            assert!(ecu.cylinder_health(i).spark_ok);
            assert!(ecu.cylinder_health(i).fuel_ok);
            assert_eq!(ecu.cylinder_health(i).combustion_factor(), 1.0);
        }
    }

    #[test]
    fn cylinder_health_reflects_failures() {
        let mut ecu = EngineControlUnit::default();
        ecu.set_cylinder_health(2, CylinderHealth::dead_plug());
        assert!(!ecu.cylinder_health(2).spark_ok);
        assert!(ecu.cylinder_health(2).fuel_ok);
        assert_eq!(ecu.cylinder_health(2).combustion_factor(), 0.0);

        ecu.set_cylinder_health(3, CylinderHealth::dead_injector());
        assert!(ecu.cylinder_health(3).spark_ok);
        assert!(!ecu.cylinder_health(3).fuel_ok);
        assert_eq!(ecu.cylinder_health(3).combustion_factor(), 0.0);
    }

    #[test]
    fn afr_enriches_at_wot_and_leans_at_cruise() {
        let mut ecu = EngineControlUnit::default();

        // Idle: near stoichiometric
        let afr_idle = ecu.schedule_afr(0.15, 800.0, 0.0, 0.01);
        assert!((afr_idle - 14.7).abs() < 0.1);

        // Lean cruise: 2500 rpm, 0.45 load
        let afr_cruise = ecu.schedule_afr(0.45, 2500.0, 0.25, 0.01);
        assert!(afr_cruise > 15.0, "cruise AFR should be lean: {afr_cruise}");

        // Wide-open throttle: rich near 12.5 (steady state)
        ecu.accel_enrichment = 0.0;
        ecu.prev_throttle = 1.0;
        let afr_wot = ecu.schedule_afr(1.0, 5000.0, 1.0, 0.01);
        assert!(
            (afr_wot - 12.5).abs() < 0.2,
            "WOT AFR should be near 12.5: {afr_wot}"
        );
    }

    #[test]
    fn accel_enrichment_temporarily_pulls_mixture_rich() {
        let mut ecu = EngineControlUnit::default();
        let steady_afr = ecu.schedule_afr(0.4, 2500.0, 0.2, 0.01);

        // Sudden throttle snap: 0.2 -> 0.8 in 20 ms
        let snap_afr = ecu.schedule_afr(0.6, 2500.0, 0.8, 0.02);
        assert!(
            snap_afr < steady_afr - 0.5,
            "throttle snap should enrich mixture: steady={steady_afr}, snap={snap_afr}"
        );

        // After some time, enrichment decays back
        for _ in 0..50 {
            ecu.schedule_afr(0.6, 2500.0, 0.8, 0.02);
        }
        let settled_afr = ecu.schedule_afr(0.6, 2500.0, 0.8, 0.02);
        assert!(
            settled_afr > snap_afr + 0.5,
            "enrichment must decay: settled={settled_afr}, snap={snap_afr}"
        );
    }

    #[test]
    fn flame_speed_peaks_slightly_rich() {
        let speed_stoich = afr_flame_speed_factor(14.7);
        let speed_rich = afr_flame_speed_factor(13.2); // phi ≈ 1.11
        let speed_lean = afr_flame_speed_factor(16.0);

        assert!(
            speed_rich > speed_stoich,
            "flame speed should peak slightly rich: rich={speed_rich} > stoich={speed_stoich}"
        );
        assert!(
            speed_lean < speed_stoich,
            "flame speed should drop when lean: lean={speed_lean} < stoich={speed_stoich}"
        );
    }

    #[test]
    fn dfco_engages_on_overrun_and_disengages_on_throttle_or_low_rpm() {
        let mut ecu = EngineControlUnit::default();

        // High rpm, throttle released: DFCO engages
        assert!(ecu.update_dfco(0.0, 3_000.0));
        assert!(ecu.dfco_active);
        assert!(!ecu.dfco_tip_in);

        // RPM drops below recovery threshold: DFCO disengages
        assert!(!ecu.update_dfco(0.0, 1_100.0));
        assert!(!ecu.dfco_active);

        // Accelerating back up: closed throttle at 2500 rpm engages DFCO again
        assert!(ecu.update_dfco(0.0, 2_500.0));
        assert!(ecu.dfco_active);

        // Driver tips into throttle: DFCO disengages and trips tip-in flag
        assert!(!ecu.update_dfco(0.4, 2_400.0));
        assert!(!ecu.dfco_active);
        assert!(ecu.dfco_tip_in);

        // Next frame with throttle open clears tip-in
        assert!(!ecu.update_dfco(0.4, 2_400.0));
        assert!(!ecu.dfco_tip_in);
    }

    #[test]
    fn decel_fuel_cut_off_silences_combustion_while_mechanical_floor_continues() {
        use crate::audio::{EngineControls, SnapshotSource};
        use crate::environment::Environment;
        use crate::physics::engine_block::EngineBlock;

        let mut block = EngineBlock::cross_plane_v8(Environment::default());
        // Settle the engine at 3000 RPM
        for _ in 0..120 {
            block.update(1.0 / 240.0, 3_000.0);
        }
        let fired_peak = block.ring.peak_pressure();

        // Release throttle: DFCO activates
        block.throttle = 0.0;
        for _ in 0..120 {
            block.update(1.0 / 240.0, 3_000.0);
        }
        assert!(block.model.fuel_cut, "fuel should be cut during DFCO");
        let dfco_peak = block.ring.peak_pressure();

        // Combustion pressure is silenced: peak pressure drops from combustion (~60 bar)
        // to purely motored compression (~20 bar)
        assert!(
            dfco_peak < 0.75 * fired_peak,
            "combustion was not silenced during DFCO: dfco={dfco_peak} vs fired={fired_peak}"
        );
        assert_eq!(
            block.ring.cycle_mean(|s| s.heat_release),
            0.0,
            "no chemical heat release during DFCO"
        );

        // Mechanical floor and pumping continue
        let mut source = SnapshotSource::new(&block);
        let snapshot = source.sample(&block, 3_000.0, 1.0 / 240.0, EngineControls::default());
        assert!(
            snapshot.friction_mep > 10_000.0,
            "mechanical noise floor must continue during DFCO"
        );
        assert_eq!(
            snapshot.unburnt_fuel_mass, 0.0,
            "fuel-cut overrun must send zero unburnt fuel to the exhaust"
        );

        // Tip-in: driver presses pedal, fuel returns and unburnt fuel spike is produced
        block.throttle = 0.5;
        block.update(1.0 / 240.0, 3_000.0);
        assert!(block.ecu.dfco_tip_in, "tip-in flag should be set");
        let tip_in_snap = source.sample(
            &block,
            3_000.0,
            1.0 / 240.0,
            EngineControls {
                throttle: 0.5,
                spark_cut: false,
                exhaust_cutout: false,
                anti_lag: false,
                starter_hz: 0.0,
            },
        );
        assert!(
            tip_in_snap.unburnt_fuel_mass > 0.0,
            "tip-in must produce unburnt fuel for an exhaust pop"
        );
        assert!(
            tip_in_snap.spark_cut,
            "tip-in must carry spark_cut flag to trigger backfire voice"
        );
    }

    #[test]
    fn spark_advances_with_rpm_and_retards_with_load() {
        let ecu = EngineControlUnit::default();

        // Idle advance
        let idle_adv = ecu.schedule_spark_advance(0.2, 800.0);
        assert!(
            (12.0..=18.0).contains(&idle_adv),
            "idle advance should be modest: {idle_adv}"
        );

        // High RPM advance at light cruise load
        let cruise_adv = ecu.schedule_spark_advance(0.3, 4_500.0);
        assert!(
            cruise_adv > idle_adv + 5.0,
            "speed must advance timing: cruise={cruise_adv} vs idle={idle_adv}"
        );

        // Heavy load (WOT) at same high speed: retarded relative to cruise
        let wot_adv = ecu.schedule_spark_advance(1.0, 4_500.0);
        assert!(
            wot_adv < cruise_adv - 4.0,
            "WOT load must retard spark relative to cruise: wot={wot_adv} vs cruise={cruise_adv}"
        );

        // Spark angle in Wiebe cycle coordinates: 20 deg BTDC corresponds to 340 deg
        let spark_rad = ecu.spark_angle(0.6, 3_000.0);
        let spark_deg = spark_rad * 180.0 / std::f64::consts::PI;
        assert!(
            spark_deg > 320.0 && spark_deg < 355.0,
            "Wiebe spark angle must fall in compression BTDC range: {spark_deg} deg"
        );
    }

    #[test]
    fn knock_retard_reduces_the_knock_integral_below_one_within_bounded_cycles() {
        use crate::environment::Environment;
        use crate::physics::cylinder::{deg, wrap_cycle, CylinderGeometry};
        use crate::physics::thermodynamics::{
            CycleLatch, CylinderModel, HeatRelease, PortConditions, Rk4Solver, ThermoState,
            WiebeProfile,
        };

        // Engine operating near the knock threshold: 10.5:1 compression ratio
        // with advanced base timing causes initial knock.
        let env = Environment::default();
        let omega = 3_000.0 * 2.0 * std::f64::consts::PI / 60.0;
        let geom = CylinderGeometry::new(0.086, 0.086, 0.1345, 10.5);
        let mut ecu = EngineControlUnit {
            base_spark_advance: 34.0,
            knock_step: 3.0,
            ..EngineControlUnit::default()
        };

        let solver = Rk4Solver::default();
        let mut knock_history = Vec::new();
        let dt = 1.0 / 240.0;

        for _cycle in 0..10 {
            let advance = ecu.schedule_spark_advance(1.0, 3_000.0);
            let spark_angle = wrap_cycle(deg(360.0 - advance));

            let model = CylinderModel {
                geometry: geom,
                combustion: HeatRelease::Spark(WiebeProfile::new(
                    spark_angle,
                    deg(60.0),
                    5.0,
                    2.0,
                    0.97,
                )),
                ..CylinderModel::default()
            };
            let ports = PortConditions::from_environment(&env, &model.gas);

            let mut st = ThermoState::at_ambient(&model.geometry, &model.gas, &env);
            st.cylinder.theta = std::f64::consts::PI;
            st.cylinder.mass = 1.0 * env.pressure * model.geometry.max_volume()
                / (model.gas.r_unburned * env.temperature);
            st.cylinder.temperature = 330.0;
            st.latch = CycleLatch {
                fuel_mass: model.trapped_fuel_mass(st.cylinder.mass, 0.0),
                dilution: 0.0,
                pressure: st.cylinder.pressure(&model.geometry, &model.gas),
                temperature: st.cylinder.temperature,
                volume: model.geometry.max_volume(),
                gamma: model.gas.gamma_unburned,
                autoignition: None,
            };

            for _ in 0..300 {
                st = solver.substep(&model, &st, omega, deg(1.0), &ports);
            }

            let integral = st.knock_integral;
            knock_history.push(integral);
            ecu.update_knock_retard(integral >= 1.0, dt);
        }

        // Initially the engine must have knocked (I >= 1.0)
        assert!(
            knock_history[0] >= 1.0,
            "engine must knock initially to test closed loop retard: {}",
            knock_history[0]
        );

        // Within a bounded number of cycles (< 6 cycles), knock retard must reduce I below 1.0
        let cycles_to_suppress = knock_history.iter().position(|&i| i < 1.0);
        assert!(
            cycles_to_suppress.is_some(),
            "knock was not suppressed within 10 cycles: {knock_history:?}"
        );
        let n_cycles = cycles_to_suppress.unwrap();
        assert!(
            n_cycles <= 5,
            "knock suppression took too many cycles: {n_cycles} cycles, history: {knock_history:?}"
        );
        assert!(
            ecu.knock_retard > 0.0,
            "knock retard must be positive to hold knock off: {}",
            ecu.knock_retard
        );
    }

    #[test]
    fn hard_cut_limiter_engages_sharply_at_redline() {
        let mut ecu = EngineControlUnit::new(6_500.0);
        ecu.limiter_mode = LimiterMode::HardCut;
        ecu.limiter_cut_type = LimiterCut::Spark;

        // Below redline: no cut
        assert_eq!(ecu.evaluate_limiter(6_490.0), LimiterCut::None);
        assert_eq!(ecu.active_cut, LimiterCut::None);

        // At and above redline: sharp cut
        assert_eq!(ecu.evaluate_limiter(6_500.0), LimiterCut::Spark);
        assert_eq!(ecu.active_cut, LimiterCut::Spark);
        assert_eq!(ecu.evaluate_limiter(6_600.0), LimiterCut::Spark);
    }

    #[test]
    fn soft_cut_limiter_engages_progressively_below_redline() {
        let mut ecu = EngineControlUnit::new(6_500.0);
        ecu.limiter_mode = LimiterMode::SoftCut;
        ecu.limiter_cut_type = LimiterCut::Fuel;
        ecu.limiter_soft_margin = 200.0; // threshold = 6300 RPM

        // Below soft cut threshold: 0 cuts
        let below = (0..16)
            .filter(|_| ecu.evaluate_limiter(6_250.0) != LimiterCut::None)
            .count();
        assert_eq!(below, 0, "no cuts below soft threshold");

        // Midway through margin (6400 RPM): partial intervention
        let midway = (0..16)
            .filter(|_| ecu.evaluate_limiter(6_400.0) != LimiterCut::None)
            .count();
        assert!(
            midway > 0 && midway < 16,
            "soft cut must progressively intervene: {midway}/16"
        );

        // At or above redline: full intervention
        let above = (0..16)
            .filter(|_| ecu.evaluate_limiter(6_550.0) != LimiterCut::None)
            .count();
        assert_eq!(above, 16, "full cut above redline");
    }

    #[test]
    fn rotating_stutter_alternates_cylinder_cuts() {
        let mut ecu = EngineControlUnit::new(6_500.0);
        ecu.limiter_mode = LimiterMode::RotatingStutter;
        ecu.limiter_cut_type = LimiterCut::Spark;

        // Below redline: no cuts
        for _ in 0..10 {
            assert_eq!(ecu.evaluate_limiter(6_000.0), LimiterCut::None);
        }

        // Above redline: alternating cut pattern (stutter)
        let cuts: Vec<bool> = (0..6)
            .map(|_| ecu.evaluate_limiter(6_600.0) == LimiterCut::Spark)
            .collect();
        // Alternating true/false
        for pair in cuts.windows(2) {
            assert_ne!(
                pair[0], pair[1],
                "stutter limiter must alternate firing cuts"
            );
        }
    }

    #[test]
    fn cylinder_health_reflects_dead_plug_and_injector() {
        let mut ecu = EngineControlUnit::default();
        assert!(ecu.is_spark_ok(0));
        assert!(ecu.is_fuel_ok(0));
        assert_eq!(ecu.cylinder_combustion_factor(0), 1.0);

        ecu.set_cylinder_health(2, CylinderHealth::dead_plug());
        assert!(!ecu.is_spark_ok(2));
        assert!(ecu.is_fuel_ok(2));
        assert_eq!(ecu.cylinder_combustion_factor(2), 0.0);

        ecu.set_cylinder_health(3, CylinderHealth::dead_injector());
        assert!(ecu.is_spark_ok(3));
        assert!(!ecu.is_fuel_ok(3));
        assert_eq!(ecu.cylinder_combustion_factor(3), 0.0);

        ecu.set_cylinder_health(3, CylinderHealth::healthy());
        assert_eq!(ecu.cylinder_combustion_factor(3), 1.0);
    }

    #[test]
    fn anti_lag_retards_spark_past_tdc_and_delivers_exhaust_fuelling_on_lift() {
        let mut ecu = EngineControlUnit::new(7_000.0).with_anti_lag(true);
        let rpm = 4_500.0;
        let lift_throttle = 0.0;

        // 1. Anti-lag must engage on throttle lift above threshold RPM
        assert!(
            ecu.is_anti_lag_active(lift_throttle, rpm),
            "anti-lag must be active on lift above threshold rpm"
        );

        // 2. DFCO must be inhibited so fuel is not cut
        assert!(
            !ecu.update_dfco(lift_throttle, rpm),
            "DFCO must be inhibited when anti-lag is active"
        );

        // 3. Exhaust fuelling must deliver rich AFR
        let afr = ecu.target_afr(0.2, rpm, lift_throttle);
        assert_eq!(
            afr, ecu.anti_lag_afr,
            "anti-lag must command rich exhaust fuelling: got {afr}"
        );

        // 4. Spark timing must retard past TDC (negative BTDC advance, angle > 360 deg)
        let advance = ecu.schedule_spark_advance_with_throttle(0.2, rpm, lift_throttle);
        assert!(
            advance < 0.0,
            "advance must be negative (retarded past TDC): got {advance} deg"
        );
        let spark_rad = ecu.spark_angle_with_throttle(0.2, rpm, lift_throttle);
        let spark_deg = spark_rad.to_degrees();
        assert!(
            spark_deg > 360.0 && spark_deg < 420.0,
            "spark angle must be past TDC (360 deg) into expansion: got {spark_deg} deg"
        );

        // 5. On throttle reapplication, anti-lag disengages and timing advances normally
        let wot_throttle = 1.0;
        assert!(!ecu.is_anti_lag_active(wot_throttle, rpm));
        let wot_adv = ecu.schedule_spark_advance_with_throttle(0.9, rpm, wot_throttle);
        assert!(
            wot_adv > 10.0,
            "WOT advance must be positive: got {wot_adv} deg"
        );
    }

    #[test]
    fn builder_configures_limiter_mode_and_cut_type() {
        let ecu = EngineControlUnit::new(7_500.0)
            .with_limiter_mode(LimiterMode::RotatingStutter)
            .with_limiter_cut_type(LimiterCut::Spark);
        assert_eq!(ecu.limiter_mode, LimiterMode::RotatingStutter);
        assert_eq!(ecu.limiter_cut_type, LimiterCut::Spark);
    }

    #[test]
    fn manual_calibration_trims_modify_spark_and_afr() {
        let mut ecu = EngineControlUnit::new(7_000.0);
        let base_adv = ecu.schedule_spark_advance(0.5, 3_000.0);
        let base_afr = ecu.target_afr(0.5, 3_000.0, 0.4);

        // Nudge spark advance by +3 degrees
        ecu.nudge_spark_trim(3.0);
        let trimmed_adv = ecu.schedule_spark_advance(0.5, 3_000.0);
        assert!((trimmed_adv - (base_adv + 3.0)).abs() < 1e-6);

        // Nudge AFR by -1.2 (enrich)
        ecu.nudge_afr_trim(-1.2);
        let trimmed_afr = ecu.target_afr(0.5, 3_000.0, 0.4);
        assert!((trimmed_afr - (base_afr - 1.2)).abs() < 1e-6);

        // Reset trims
        ecu.reset_trims();
        assert_eq!(ecu.spark_trim, 0.0);
        assert_eq!(ecu.afr_trim, 0.0);
        assert_eq!(ecu.schedule_spark_advance(0.5, 3_000.0), base_adv);
        assert_eq!(ecu.target_afr(0.5, 3_000.0, 0.4), base_afr);
    }

    #[test]
    fn cylinder_health_toggling_cycles_states() {
        let mut ecu = EngineControlUnit::new(7_000.0);
        assert_eq!(ecu.cylinder_health(2), CylinderHealth::healthy());
        ecu.toggle_cylinder_health(2);
        assert_eq!(ecu.cylinder_health(2), CylinderHealth::dead_plug());
        ecu.toggle_cylinder_health(2);
        assert_eq!(ecu.cylinder_health(2), CylinderHealth::dead_injector());
        ecu.toggle_cylinder_health(2);
        assert_eq!(ecu.cylinder_health(2), CylinderHealth::dead());
        ecu.toggle_cylinder_health(2);
        assert_eq!(ecu.cylinder_health(2), CylinderHealth::healthy());
    }

    #[test]
    fn cycle_limiter_cut_cycles_all_mechanisms() {
        let mut ecu = EngineControlUnit::new(7_000.0);
        assert_eq!(ecu.limiter_cut_type, LimiterCut::Spark);
        ecu.cycle_limiter_cut();
        assert_eq!(ecu.limiter_cut_type, LimiterCut::Fuel);
        ecu.cycle_limiter_cut();
        assert_eq!(ecu.limiter_cut_type, LimiterCut::None);
        ecu.cycle_limiter_cut();
        assert_eq!(ecu.limiter_cut_type, LimiterCut::Spark);
    }

    #[test]
    fn a_warm_port_has_no_film_and_delivers_exactly_what_was_metered() {
        // The neutrality guarantee, at the source. Bit-exact, not within a
        // tolerance: every recorded fingerprint was measured on an engine at
        // `cold_fraction == 0`, so anything this returns other than the number
        // it was handed has moved the whole catalogue.
        let mut ecu = EngineControlUnit::default();
        let dt = 1.0 / 480.0;
        for step in 0..2_000 {
            // Including the step from nothing to full fuelling, which is the
            // transient the film exists to smear on a cold engine.
            let commanded = if step < 100 { 0.0 } else { 1.0 };
            assert_eq!(ecu.update_wall_film(commanded, 0.0, dt), commanded);
            assert_eq!(ecu.wall_film, 0.0);
        }
        assert_eq!(ecu.cold_enriched_afr(14.7, 0.0), 14.7);
    }

    #[test]
    fn a_cold_port_delays_delivered_fuel_and_the_delay_shortens_as_it_warms() {
        // Step the injector on from nothing and count how long the cylinder
        // waits for what it was promised. A colder port takes longer, and the
        // whole of that ordering has to hold without anything being told what
        // the answer should be.
        let fill_time = |cold: f64| -> f64 {
            let mut ecu = EngineControlUnit::default();
            let dt = 1.0 / 480.0;
            // The first injection is short by whatever the wall takes.
            let first = ecu.update_wall_film(1.0, cold, dt);
            assert!(
                first < 1.0,
                "a port at {cold} cold delivered all of the first injection"
            );
            for step in 1..4_800 {
                if ecu.update_wall_film(1.0, cold, dt) > 0.98 {
                    return step as f64 * dt;
                }
            }
            f64::INFINITY
        };

        let stone_cold = fill_time(1.0);
        let warming = fill_time(0.5);
        let nearly_warm = fill_time(0.2);
        assert!(
            stone_cold > warming && warming > nearly_warm,
            "the lag did not shorten as the block warmed: \
             {stone_cold:.2} s cold, {warming:.2} s half warm, {nearly_warm:.2} s nearly warm"
        );
        assert!(
            stone_cold.is_finite() && stone_cold > 0.3,
            "a stone-cold port caught up in {stone_cold:.3} s"
        );
    }

    #[test]
    fn the_wall_film_gives_back_everything_it_takes() {
        // It is a lag and not a loss, so in the steady state the cylinder gets
        // exactly what the injector was told to meter — at any temperature.
        let mut ecu = EngineControlUnit::default();
        let dt = 1.0 / 480.0;
        for _ in 0..4_800 {
            ecu.update_wall_film(1.0, 1.0, dt);
        }
        assert!(
            (ecu.fuel_delivery - 1.0).abs() < 1e-3,
            "a settled cold port is still {:.4} of what was metered",
            ecu.fuel_delivery
        );

        // And it hands back what it is holding once the injector shuts off:
        // fuel keeps arriving after a cut, off the wall rather than the rail.
        let held = ecu.wall_film;
        assert!(held > 0.0, "a stone-cold port held no fuel at all");
        let mut returned = 0.0;
        for _ in 0..4_800 {
            returned += ecu.update_wall_film(0.0, 1.0, dt) * dt;
        }
        assert!(
            (returned - held).abs() < 0.02 * held,
            "the wall held {held:.4} and gave back {returned:.4}"
        );
    }

    #[test]
    fn a_cold_engine_is_commanded_richer_than_a_warm_one() {
        let ecu = EngineControlUnit::default();
        let warm = ecu.cold_enriched_afr(14.7, 0.0);
        let cold = ecu.cold_enriched_afr(14.7, 1.0);
        assert!(
            cold < warm,
            "a cold engine was not enriched: {cold} against {warm}"
        );
        assert!(
            cold > 8.0,
            "enriched past anything that would burn at all: {cold}"
        );
    }
}
