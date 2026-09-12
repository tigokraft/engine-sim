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

/// Limiter cut mechanism.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LimiterCut {
    /// No cut active.
    None,
    /// Fuel cut: injectors disabled. No fuel reaches the exhaust, preventing backfires.
    Fuel,
    /// Spark cut: ignition disabled. Unburnt fuel enters the exhaust, enabling backfires.
    Spark,
}

/// Rev limiter intervention strategy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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

    /// Cycles through the available rev limiter cut mechanisms (Spark vs Fuel).
    pub fn cycle_limiter_cut(&mut self) {
        self.limiter_cut_type = match self.limiter_cut_type {
            LimiterCut::Spark => LimiterCut::Fuel,
            LimiterCut::Fuel => LimiterCut::Spark,
            LimiterCut::None => LimiterCut::Spark,
        };
    }

    /// Builder enabling or disabling anti-lag system.
    pub fn with_anti_lag(mut self, enabled: bool) -> Self {
        self.anti_lag = enabled;
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
}
