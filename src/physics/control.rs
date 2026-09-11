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
    /// Current state of DFCO.
    pub dfco_active: bool,
    /// Current acceleration enrichment offset on AFR [-].
    pub accel_enrichment: f64,
    /// Previous throttle position for derivative calculation [-].
    pub prev_throttle: f64,

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

    // --- Cylinder health ---
    /// Per-cylinder operational health.
    pub cylinder_health: [CylinderHealth; MAX_CYLINDERS],
}

impl Default for EngineControlUnit {
    fn default() -> Self {
        Self::new(6_500.0)
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
            dfco_active: false,
            accel_enrichment: 0.0,
            prev_throttle: 0.0,

            base_spark_advance: 25.0,
            idle_advance: 12.0,
            max_advance: 36.0,
            wot_retard: 6.0,

            knock_retard: 0.0,
            knock_step: 2.0,
            knock_decay: 1.0,
            max_knock_retard: 15.0,

            base_wiebe_duration: 1.0471975511965976, // 60 deg

            redline,
            limiter_mode: LimiterMode::HardCut,
            limiter_cut_type: LimiterCut::Spark,
            limiter_soft_margin: 200.0,
            stutter_counter: 0,

            cylinder_health: [CylinderHealth::healthy(); MAX_CYLINDERS],
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
        // High load or wide throttle: WOT enrichment for peak power and charge cooling
        if throttle >= 0.70 || load >= 0.85 {
            let t_blend = ((throttle - 0.70) / 0.25).clamp(0.0, 1.0);
            let l_blend = ((load - 0.70) / 0.25).clamp(0.0, 1.0);
            let wot_blend = t_blend.max(l_blend);
            return self.stoich_afr + wot_blend * (self.wot_afr - self.stoich_afr);
        }

        // Moderate speed and light-to-medium load: lean cruise (only at part throttle)
        if throttle < 0.50 && (1_500.0..=3_800.0).contains(&rpm) && (0.25..=0.65).contains(&load) {
            let rpm_factor = (1.0 - ((rpm - 2_650.0) / 1_150.0).abs()).clamp(0.0, 1.0);
            let load_factor = (1.0 - ((load - 0.45) / 0.20).abs()).clamp(0.0, 1.0);
            let cruise_blend = rpm_factor * load_factor;
            return self.stoich_afr + cruise_blend * (self.cruise_afr - self.stoich_afr);
        }

        // Idle or light low-speed load: stoichiometric
        self.idle_afr
    }

    /// Sets the health of a specific cylinder.
    pub fn set_cylinder_health(&mut self, cylinder: usize, health: CylinderHealth) {
        if cylinder < MAX_CYLINDERS {
            self.cylinder_health[cylinder] = health;
        }
    }

    /// Returns the health of a specific cylinder.
    pub fn cylinder_health(&self, cylinder: usize) -> CylinderHealth {
        if cylinder < MAX_CYLINDERS {
            self.cylinder_health[cylinder]
        } else {
            CylinderHealth::healthy()
        }
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
}
