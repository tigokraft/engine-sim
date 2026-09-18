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

#[test]
fn a_sequential_valve_stays_shut_below_its_open_threshold() {
    let mut valve = SequentialValve::new(5_000.0, 4_000.0, 0.2);
    for _ in 0..1_000 {
        valve.update(1.0 / 480.0, 3_000.0);
    }
    assert_eq!(valve.fraction(), 0.0);
}

#[test]
fn a_sequential_valve_opens_and_shuts_with_hysteresis_and_a_bounded_ramp() {
    let mut valve = SequentialValve::new(5_000.0, 4_000.0, 0.2);
    let dt = 1.0 / 480.0;

    // Above the open threshold it drives toward open, but not in one step.
    let first = valve.update(dt, 5_500.0);
    assert!(
        first > 0.0 && first < 1.0,
        "a single frame past threshold should not snap the valve open: {first}"
    );
    for _ in 0..2_000 {
        valve.update(dt, 5_500.0);
    }
    assert!(valve.fraction() > 0.99, "valve did not settle open");

    // Between the two thresholds it holds wherever it already was —
    // that gap is the hysteresis band, and without it the valve would
    // chatter every frame the signal sat near one threshold.
    valve.update(dt, 4_500.0);
    assert!(
        valve.fraction() > 0.99,
        "valve closed inside the hysteresis band instead of holding open"
    );

    // Below the close threshold it drives shut, again with a ramp.
    for _ in 0..2_000 {
        valve.update(dt, 3_000.0);
    }
    assert!(valve.fraction() < 0.01, "valve did not settle shut");
}
