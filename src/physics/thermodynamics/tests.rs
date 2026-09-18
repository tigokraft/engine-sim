use super::*;

fn approx(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() <= tol, "expected {b}, got {a} (tol {tol})");
}

fn rpm_to_omega(rpm: f64) -> f64 {
    rpm * 2.0 * PI / 60.0
}

/// A sealed cylinder: no valves, no fuel, no wall loss. Pure compression.
fn adiabatic_model() -> CylinderModel {
    let mut m = CylinderModel::default();
    m.heat.scaling = 0.0;
    m.valves.intake.duration = 0.0;
    m.valves.exhaust.duration = 0.0;
    if let Some(wiebe) = m.combustion.spark_mut() {
        wiebe.combustion_efficiency = 0.0;
    }
    m
}

#[test]
fn wiebe_runs_from_zero_to_almost_one_over_its_duration() {
    let w = WiebeProfile::default();
    approx(w.burned_fraction(w.spark_angle), 0.0, 1e-12);
    // a = 5 leaves exp(-5) ~ 0.67 % unburned at the end of the duration.
    let end = w.burned_fraction(w.spark_angle + w.duration * 0.999_999);
    approx(end, 1.0 - (-5.0f64).exp(), 1e-4);
    assert!(w.dburned_dtheta(w.spark_angle - deg(1.0)) == 0.0);
    assert!(w.dburned_dtheta(w.spark_angle + w.duration + deg(1.0)) == 0.0);
}

#[test]
fn wiebe_rate_integrates_back_to_the_wiebe_profile() {
    let w = WiebeProfile::default();
    let steps = 20_000;
    let h = w.duration / steps as f64;
    let mut x = 0.0;
    for i in 0..steps {
        // Midpoint rule: second order, plenty for a 1e-6 check.
        x += w.dburned_dtheta(w.spark_angle + (i as f64 + 0.5) * h) * h;
    }
    approx(
        x,
        w.burned_fraction(w.spark_angle + w.duration * 0.999_999),
        1e-6,
    );
}

#[test]
fn wiebe_rate_peaks_inside_the_burn_window() {
    let w = WiebeProfile::default();
    let mut peak_u = 0.0;
    let mut peak = 0.0;
    for i in 0..=1000 {
        let u = i as f64 / 1000.0;
        let rate = w.dburned_dtheta(w.spark_angle + u * w.duration);
        if rate > peak {
            peak = rate;
            peak_u = u;
        }
    }
    // n = 2 puts the peak burn rate a bit past mid-burn.
    assert!(
        (0.4..0.8).contains(&peak_u),
        "peak burn rate at u = {peak_u}, expected mid-burn"
    );
}

#[test]
fn an_undiluted_charge_burns_exactly_as_it_did_before_dilution_existed() {
    // The neutrality half of the dilution model. A cylinder that scavenged
    // perfectly has nothing inert in it, so nothing about its burn may
    // move — not by a tolerance, exactly.
    let wiebe = WiebeProfile::default();
    assert_eq!(wiebe.diluted(0.0), wiebe);
    assert_eq!(
        wiebe.diluted(-1.0),
        wiebe,
        "a negative residual is no residual"
    );
}

#[test]
fn a_diluted_charge_burns_slower_and_less_completely() {
    // Both halves of the claim, against the analytic Wiebe rather than
    // against a rendered anything. `x_b` at the end of the nominal burn is
    // `1 - exp(-a)` by construction, so the completeness is readable
    // straight off the efficiency parameter.
    let wiebe = WiebeProfile::default();
    let clean = 1.0 - (-wiebe.efficiency_parameter).exp();

    let mut previous_completeness = clean;
    let mut previous_duration = wiebe.duration;
    for residual in [0.05, 0.10, 0.20, 0.30] {
        let diluted = wiebe.diluted(residual);
        let completeness = 1.0 - (-diluted.efficiency_parameter).exp();
        assert!(
            completeness < previous_completeness,
            "burn completeness must fall with residual: {completeness:.4} at \
             {residual} is not under {previous_completeness:.4}"
        );
        assert!(
            diluted.duration > previous_duration,
            "burn duration must rise with residual: {:.1} deg at {residual} is \
             not over {:.1} deg",
            diluted.duration.to_degrees(),
            previous_duration.to_degrees()
        );
        previous_completeness = completeness;
        previous_duration = diluted.duration;
    }

    // At the dilution limit the flame is a misfire rather than a divide by
    // zero: a finite, very long burn that consumes almost nothing.
    let dead = wiebe.diluted(DILUTION_LIMIT);
    assert!(dead.duration.is_finite() && dead.duration > 4.0 * wiebe.duration);
    let dead_completeness = 1.0 - (-dead.efficiency_parameter).exp();
    assert!(
        dead_completeness < 0.3,
        "a charge past the dilution limit must barely light: {dead_completeness:.3}"
    );

    // And the fuel that does not burn is what leaves through the port.
    // A tenth residual is a percent or two of the charge's fuel going out
    // unburnt, which is the quantity the backfire voice reads.
    let lopey = wiebe.diluted(0.10);
    let unburnt = (-lopey.efficiency_parameter).exp();
    assert!(
        (0.005..0.10).contains(&unburnt),
        "a tenth residual should leave a few per cent of the fuel unburnt, got {unburnt:.4}"
    );
}

#[test]
fn a_stock_ramp_reproduces_the_raised_cosine_it_replaced() {
    // The neutrality proof for the profile parameter. Every preset in the
    // catalogue ships the default ramp, so the lift curve they were
    // measured on has to come back out of the new two-flank form
    // unchanged, or the catalogue has been quietly re-cammed.
    //
    // The opening flank is exact: at a half ramp the flank angle is
    // `PI / 0.5 * u`, and dividing by a half is exact, so it is the same
    // `2 PI u` the single raised cosine used. The closing flank reflects
    // about the peak and asks for the cosine of `2 PI (1 - u)` instead,
    // which is the same angle to the mathematics and a different one to
    // the last bit of an f64 — hence a tolerance of a few ulp rather than
    // an equality, on the closing half only.
    let v = ValveEvent::new(deg(700.0), deg(240.0), 0.010, 0.037, 0.65);
    assert_eq!(v.ramp_fraction, RAISED_COSINE_RAMP);
    for step in 0..=240 {
        let theta = deg(700.0) + deg(step as f64);
        // The pre-change formula, phase arithmetic and all.
        let phase = wrap_cycle(theta - v.open_angle);
        let u = phase / v.duration;
        let raised_cosine = if u >= 1.0 {
            0.0
        } else {
            0.5 * 0.010 * (1.0 - (2.0 * PI * u).cos())
        };
        let lift = v.lift(theta);
        if u <= 0.5 {
            assert_eq!(
                lift, raised_cosine,
                "lift moved at {step} deg into the event"
            );
        } else {
            assert!(
                (lift - raised_cosine).abs() <= 8.0 * f64::EPSILON * 0.010,
                "lift moved at {step} deg into the event: {lift:e} against {raised_cosine:e}"
            );
        }
    }
    approx(v.ramp_rate(), 1.0, 0.0);
}

#[test]
fn an_aggressive_ramp_opens_faster_and_flows_more_for_the_same_duration() {
    let stock = ValveTrain::default();
    let roller = stock.with_aggressiveness(1.0);

    // Same card: same timing, same duration, same lift.
    assert_eq!(roller.intake.open_angle, stock.intake.open_angle);
    assert_eq!(roller.intake.duration, stock.intake.duration);
    assert_eq!(roller.intake.max_lift, stock.intake.max_lift);
    assert_eq!(roller.overlap(), stock.overlap());

    // Different lobe: it comes off the seat over three times as fast.
    let ratio = roller.intake.ramp_rate() / stock.intake.ramp_rate();
    assert!(
        (3.0..3.6).contains(&ratio),
        "the fastest flank should be a bit over three times the raised \
         cosine\'s peak velocity, got {ratio:.2}"
    );

    // And it is up sooner, so there is more area under the curve. Sampled
    // rather than integrated analytically because the flow the solver sees
    // is the effective area, curtain cap and all.
    let area = |v: &ValveEvent| -> f64 {
        (0..2_400)
            .map(|i| v.effective_area(v.open_angle + v.duration * i as f64 / 2_400.0))
            .sum::<f64>()
    };
    assert!(
        area(&roller.intake) > area(&stock.intake) * 1.10,
        "a faster flank must flow measurably more over the same duration: \
         {:.6} against {:.6}",
        area(&roller.intake),
        area(&stock.intake)
    );

    // Peak lift is still the peak, not something the dwell exceeded.
    let peak = roller
        .intake
        .lift(roller.intake.open_angle + roller.intake.duration / 2.0);
    approx(peak, stock.intake.max_lift, 1e-15);

    // Round trip through the parameter it was asked for.
    approx(roller.aggressiveness(), 1.0, 1e-12);
    approx(stock.aggressiveness(), 0.0, 1e-12);
}

#[test]
fn flow_function_chokes_at_the_critical_pressure_ratio() {
    let g = 1.4;
    let critical = (2.0f64 / (g + 1.0)).powf(g / (g - 1.0));
    approx(critical, 0.5283, 1e-4);

    // Continuous across the choke point.
    let just_above = flow_function(critical + 1e-9, g);
    let just_below = flow_function(critical - 1e-9, g);
    approx(just_above, just_below, 1e-6);

    // Flat once choked, and zero with no pressure difference.
    approx(flow_function(0.1, g), flow_function(0.4, g), 1e-12);
    approx(flow_function(1.0, g), 0.0, 1e-12);
    // Monotone rising as the ratio falls to the choke point.
    assert!(flow_function(0.9, g) < flow_function(0.7, g));
    assert!(flow_function(0.7, g) < flow_function(critical, g));
}

#[test]
fn valve_lift_is_smooth_and_closed_outside_its_event() {
    let v = ValveEvent::new(deg(700.0), deg(240.0), 0.010, 0.037, 0.65);
    approx(v.lift(deg(700.0)), 0.0, 1e-15);
    approx(v.lift(v.close_angle() - 1e-9), 0.0, 1e-9);
    approx(v.lift(deg(700.0 + 120.0)), 0.010, 1e-12);
    assert_eq!(v.lift(deg(400.0)), 0.0, "shut mid-power-stroke");
    assert_eq!(v.effective_area(deg(400.0)), 0.0);
    assert!(v.is_open(deg(10.0)), "event wraps through 720 degrees");
    approx(v.close_angle().to_degrees(), 220.0, 1e-9);
}

/// The load-bearing accuracy claim: on a closed, adiabatic, non-reacting
/// cylinder the solver must reproduce `T V^(gamma-1) = const` to well under
/// a Kelvin over a full compression stroke.
#[test]
fn rk4_reproduces_the_isentrope_on_a_sealed_cylinder() {
    let model = adiabatic_model();
    let env = Environment::default();
    let ports = PortConditions::from_environment(&env, &model.gas);
    let solver = Rk4Solver::default();
    let omega = rpm_to_omega(3000.0);

    let mut st = ThermoState::at_ambient(&model.geometry, &model.gas, &env);
    st.cylinder.theta = PI; // BDC, start of compression
    st.cylinder.mass =
        env.pressure * model.geometry.max_volume() / (model.gas.r_unburned * env.temperature);
    st.cylinder.burned_fraction = 0.0;

    let t0 = st.cylinder.temperature;
    let v0 = model.geometry.safe_volume(st.cylinder.theta);
    let gamma = model.gas.gamma(0.0);

    // 180 degrees of compression at 1 degree a step.
    for _ in 0..180 {
        st = solver.substep(&model, &st, omega, deg(1.0), &ports);
    }

    let v1 = model.geometry.safe_volume(st.cylinder.theta);
    let expected = t0 * (v0 / v1).powf(gamma - 1.0);
    approx(st.cylinder.temperature, expected, 0.05);
    // Mass is exactly conserved with both valves shut.
    approx(
        st.cylinder.mass,
        env.pressure * v0 / (model.gas.r_unburned * env.temperature),
        1e-15,
    );
}

/// RK4 is fourth order, so quartering the step should cut the error by ~256.
/// Anything better than 100x confirms we have not silently dropped to a
/// lower-order scheme in the stage construction.
#[test]
fn rk4_converges_at_fourth_order() {
    let model = adiabatic_model();
    let env = Environment::default();
    let ports = PortConditions::from_environment(&env, &model.gas);
    let solver = Rk4Solver::default();
    let omega = rpm_to_omega(3000.0);
    let gamma = model.gas.gamma(0.0);

    let error_at = |dtheta_deg: f64| -> f64 {
        let mut st = ThermoState::at_ambient(&model.geometry, &model.gas, &env);
        st.cylinder.theta = PI;
        let t0 = st.cylinder.temperature;
        let v0 = model.geometry.safe_volume(st.cylinder.theta);
        let steps = (180.0 / dtheta_deg).round() as usize;
        for _ in 0..steps {
            st = solver.substep(&model, &st, omega, deg(dtheta_deg), &ports);
        }
        let v1 = model.geometry.safe_volume(st.cylinder.theta);
        (st.cylinder.temperature - t0 * (v0 / v1).powf(gamma - 1.0)).abs()
    };

    let coarse = error_at(4.0);
    let fine = error_at(1.0);
    assert!(
        fine * 100.0 < coarse,
        "expected ~4th order convergence, got {coarse} -> {fine}"
    );
}

#[test]
fn combustion_raises_pressure_and_temperature() {
    let model = CylinderModel::default();
    let env = Environment::default();
    let ports = PortConditions::from_environment(&env, &model.gas);
    let solver = Rk4Solver::default();
    let omega = rpm_to_omega(3000.0);

    // Trap a fresh charge at BDC and run the real compression-and-burn path
    // rather than dropping a burn onto a cold cylinder mid-stroke.
    let mut st = ThermoState::at_ambient(&model.geometry, &model.gas, &env);
    st.cylinder.theta = PI;
    st.cylinder.mass =
        env.pressure * model.geometry.max_volume() / (model.gas.r_unburned * env.temperature);
    st.latch = CycleLatch {
        fuel_mass: model.trapped_fuel_mass(st.cylinder.mass, 0.0),
        dilution: 0.0,
        pressure: env.pressure,
        temperature: env.temperature,
        volume: model.geometry.max_volume(),
        gamma: model.gas.gamma_unburned,
        autoignition: None,
    };

    let before = st.cylinder.pressure(&model.geometry, &model.gas);
    // 180 degrees BDC to TDC, then 80 more to finish the burn.
    for _ in 0..260 {
        st = solver.substep(&model, &st, omega, deg(1.0), &ports);
    }
    let after = st.cylinder.pressure(&model.geometry, &model.gas);

    assert!(
        after > before * 10.0,
        "burn produced no pressure rise: {before} -> {after}"
    );
    assert!(
        st.cylinder.burned_fraction > 0.99,
        "burn did not complete: x_b = {}",
        st.cylinder.burned_fraction
    );
    assert!(
        (1500.0..4000.0).contains(&st.cylinder.temperature),
        "unphysical peak temperature {}",
        st.cylinder.temperature
    );
}

#[test]
fn trailing_plug_fires_second_and_smaller() {
    let two_plug = TwoPlugCombustion::new(WiebeProfile::default(), deg(12.0), 0.35);
    assert!(
        two_plug.trailing_delay() > 0.0,
        "the trailing plug must fire after the leading one"
    );
    assert!(
        two_plug.trailing_share < 0.5,
        "the trailing plug must account for less of the charge than the leading one"
    );
}

/// Runs a compression-and-burn cycle on a two-plug model, returning the
/// pressure trace from BDC through the end of both burns.
fn two_plug_pressure_trace(combustion: HeatRelease) -> Vec<f64> {
    let mut model = CylinderModel::default();
    model.combustion = combustion;
    let env = Environment::default();
    let ports = PortConditions::from_environment(&env, &model.gas);
    let solver = Rk4Solver::default();
    let omega = rpm_to_omega(3000.0);

    let mut st = ThermoState::at_ambient(&model.geometry, &model.gas, &env);
    st.cylinder.theta = PI;
    st.cylinder.mass =
        env.pressure * model.geometry.max_volume() / (model.gas.r_unburned * env.temperature);
    st.latch = CycleLatch {
        fuel_mass: model.trapped_fuel_mass(st.cylinder.mass, 0.0),
        dilution: 0.0,
        pressure: env.pressure,
        temperature: env.temperature,
        volume: model.geometry.max_volume(),
        gamma: model.gas.gamma_unburned,
        autoignition: None,
    };

    let mut trace = Vec::with_capacity(260);
    for _ in 0..260 {
        st = solver.substep(&model, &st, omega, deg(1.0), &ports);
        trace.push(st.cylinder.pressure(&model.geometry, &model.gas));
    }
    trace
}

#[test]
fn removing_the_trailing_plug_measurably_changes_the_pressure_trace() {
    let leading = WiebeProfile::new(deg(340.0), deg(60.0), 5.0, 2.0, 0.97);
    let with_trailing = TwoPlugCombustion::new(leading, deg(12.0), 0.35);
    let mut without_trailing = with_trailing;
    without_trailing.trailing_share = 0.0;

    let trace_with = two_plug_pressure_trace(HeatRelease::TwoPlug(with_trailing));
    let trace_without = two_plug_pressure_trace(HeatRelease::TwoPlug(without_trailing));

    let peak_with = trace_with.iter().cloned().fold(f64::MIN, f64::max);
    let peak_without = trace_without.iter().cloned().fold(f64::MIN, f64::max);
    assert!(
        (peak_with - peak_without).abs() > 1.0e4,
        "the trailing plug's contribution must measurably move peak pressure: \
         {peak_with:.0} Pa against {peak_without:.0} Pa"
    );

    let max_diff = trace_with
        .iter()
        .zip(trace_without.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0, f64::max);
    assert!(
        max_diff > 1.0e4,
        "removing the trailing plug must move the pressure trace itself, not just its peak: \
         {max_diff:.0} Pa max difference"
    );
}

#[test]
fn wall_loss_cools_the_charge_relative_to_adiabatic() {
    let env = Environment::default();
    let omega = rpm_to_omega(3000.0);
    let solver = Rk4Solver::default();

    let run = |scaling: f64| {
        let mut model = adiabatic_model();
        model.heat.scaling = scaling;
        let ports = PortConditions::from_environment(&env, &model.gas);
        let mut st = ThermoState::at_ambient(&model.geometry, &model.gas, &env);
        st.cylinder.theta = PI;
        st.cylinder.temperature = 900.0; // well above the 450 K wall
        for _ in 0..180 {
            st = solver.substep(&model, &st, omega, deg(1.0), &ports);
        }
        st.cylinder.temperature
    };

    assert!(run(1.0) < run(0.0), "Woschni loss must cool the charge");
}

#[test]
fn knock_integral_accumulates_faster_on_low_octane_fuel() {
    let hot = KnockModel::default();
    let low = KnockModel {
        octane_number: 80.0,
        ..KnockModel::default()
    };

    let p = 40e5;
    let t = 850.0;
    assert!(
        low.ignition_delay(p, t) < hot.ignition_delay(p, t),
        "lower octane must autoignite sooner"
    );
    // Pressure shortens the delay, temperature shortens it much faster.
    assert!(hot.ignition_delay(80e5, t) < hot.ignition_delay(40e5, t));
    assert!(hot.ignition_delay(p, 1000.0) < hot.ignition_delay(p, 700.0));
    assert!(hot.ignition_delay(p, t).is_finite());
    // Cold end gas is effectively inert, not infinite.
    assert!(hot.ignition_delay(1e5, 300.0).is_finite());
}

/// A charge lit exactly at top dead centre, so the two-stage shape can be
/// looked at on its own without solving a cycle for it.
fn lit_at_tdc(premixed_fraction: f64) -> Autoignition {
    Autoignition {
        angle: deg(360.0),
        delay: 1.0e-3,
        premixed_fraction,
    }
}

#[test]
fn two_stage_burn_spikes_before_it_humps() {
    // The shape the whole stage exists for: a narrow premixed spike a
    // degree or two after ignition, then a broad diffusion hump twenty-odd
    // degrees later. One Wiebe cannot do that, which is why there are two.
    let d = DieselCombustion::default();
    let ign = lit_at_tdc(0.3);
    let step = deg(0.1);
    let samples: Vec<(f64, f64)> = (0..900)
        .map(|i| {
            let phase = i as f64 * step;
            (phase, d.dburned_dtheta(ign.angle + phase, &ign))
        })
        .collect();

    let peaks: Vec<(f64, f64)> = samples
        .windows(3)
        .filter(|w| w[1].1 > w[0].1 && w[1].1 >= w[2].1)
        .map(|w| w[1])
        .collect();
    assert_eq!(
        peaks.len(),
        2,
        "a two-stage release must have two peaks, found {}: {:?}",
        peaks.len(),
        peaks
            .iter()
            .map(|(p, _)| p.to_degrees())
            .collect::<Vec<_>>()
    );

    let (premixed_at, premixed_rate) = peaks[0];
    let (diffusion_at, diffusion_rate) = peaks[1];
    assert!(
        premixed_at < deg(5.0),
        "the premixed spike arrived {:.1} degrees after ignition, not promptly",
        premixed_at.to_degrees()
    );
    assert!(
        diffusion_at > deg(10.0) && diffusion_at < deg(45.0),
        "the diffusion hump landed at {:.1} degrees",
        diffusion_at.to_degrees()
    );
    assert!(
        premixed_rate > 2.0 * diffusion_rate,
        "the spike ({premixed_rate:.2}/rad) is not sharper than the hump \
         ({diffusion_rate:.2}/rad), so it is not a spike"
    );
}

#[test]
fn two_stage_rate_integrates_back_to_its_own_fraction() {
    let d = DieselCombustion::default();
    for f in [0.05, 0.3, 0.6] {
        let ign = lit_at_tdc(f);
        let steps = 40_000;
        let span = d.diffusion_duration.max(d.premixed_duration);
        let h = span / steps as f64;
        let mut x = 0.0;
        for i in 0..steps {
            x += d.dburned_dtheta(ign.angle + (i as f64 + 0.5) * h, &ign) * h;
        }
        // Both stages leave the usual exp(-a) unburned, so the pair does
        // too, whatever the split between them. The tolerance is the
        // quadrature's, not the model's: the spike's form factor is below
        // one, so its rate starts with an infinite slope and the midpoint
        // rule pays for that in the first degree.
        approx(x, 1.0 - (-d.efficiency_parameter).exp(), 1e-3);
        // The fraction the two stages report together is monotone and
        // arrives at the whole charge.
        let mut previous = 0.0;
        for i in 0..=720 {
            let at = d.burned_fraction(ign.angle + i as f64 * deg(0.1), &ign);
            assert!(at >= previous - 1e-12, "burn ran backwards at step {i}");
            previous = at;
        }
        assert!(
            previous > 0.99,
            "the charge never finished burning: {previous}"
        );
    }
}

/// A charge trapped at BDC at a stated pressure and temperature.
fn trapped(geometry: &CylinderGeometry, pressure: f64, temperature: f64) -> CycleLatch {
    CycleLatch {
        fuel_mass: 1.0e-5,
        dilution: 0.0,
        pressure,
        temperature,
        volume: geometry.max_volume(),
        gamma: 1.38,
        autoignition: None,
    }
}

#[test]
fn ignition_delay_lengthens_on_a_colder_compression() {
    // The whole reason a diesel is hard to start and clatters when it is:
    // the charge has to reach the temperature on its own, and a cylinder
    // that starts colder takes longer to get there. Nothing schedules this.
    let diesel = DieselCombustion::default();
    let geometry = CylinderGeometry::new(0.083, 0.092, 0.147, 18.0);
    let omega = rpm_to_omega(1_500.0);

    let hot = diesel
        .autoignition(&trapped(&geometry, 1.6e5, 330.0), &geometry, omega)
        .expect("a warm charge must light");
    let cold = diesel
        .autoignition(&trapped(&geometry, 1.6e5, 290.0), &geometry, omega)
        .expect("a cool charge must still light on 18:1");

    assert!(
        cold.delay > hot.delay,
        "a colder charge must be slower to light: {:.2} ms against {:.2} ms",
        cold.delay * 1e3,
        hot.delay * 1e3
    );
    assert!(
        cold.angle > hot.angle,
        "a longer delay must also put ignition later in the cycle"
    );
    // And the late one piles up more fuel while it waits, which is the
    // clatter.
    assert!(
        cold.premixed_fraction > hot.premixed_fraction,
        "the colder cycle did not premix more: {:.3} against {:.3}",
        cold.premixed_fraction,
        hot.premixed_fraction
    );
    // A real DI diesel lights within a millisecond or two of the injector.
    assert!(
        (0.2e-3..3.0e-3).contains(&hot.delay),
        "implausible delay: {:.2} ms",
        hot.delay * 1e3
    );
}

#[test]
fn compression_ratio_is_what_makes_a_diesel_light_at_all() {
    let diesel = DieselCombustion::default();
    let omega = rpm_to_omega(1_500.0);
    let squeeze = |ratio: f64| {
        let geometry = CylinderGeometry::new(0.083, 0.092, 0.147, ratio);
        diesel.autoignition(&trapped(&geometry, 1.6e5, 330.0), &geometry, omega)
    };

    let high = squeeze(20.0).expect("20:1 must light");
    let low = squeeze(14.0).expect("14:1 must still light on a warm charge");
    assert!(
        low.delay > high.delay,
        "less squeeze must mean a longer wait: {:.2} ms against {:.2} ms",
        low.delay * 1e3,
        high.delay * 1e3
    );
    // A petrol engine's squeeze cannot light diesel at all, which is why
    // one has no injectors in it.
    assert!(
        squeeze(9.0).is_none(),
        "9:1 compression lit a diesel charge, which no engine has ever done"
    );
}

#[test]
fn the_latch_solves_ignition_only_on_a_compression_engine() {
    let env = Environment::default();
    let omega = rpm_to_omega(1_500.0);
    let mut model = CylinderModel {
        geometry: CylinderGeometry::new(0.083, 0.092, 0.147, 18.0),
        ..CylinderModel::default()
    };
    let mut cylinder =
        CylinderState::at_ambient(&model.geometry, &model.gas, env.pressure, env.temperature);
    cylinder.theta = PI;
    cylinder.mass =
        env.pressure * model.geometry.max_volume() / (model.gas.r_unburned * env.temperature);

    assert!(
        model.latch(&cylinder, omega).autoignition.is_none(),
        "a spark engine has nothing to autoignite"
    );

    model.combustion = HeatRelease::Compression(DieselCombustion::default());
    let latch = model.latch(&cylinder, omega);
    let ignition = latch.autoignition.expect("the diesel charge must light");
    assert!(ignition.angle > model.combustion.commanded_angle());
    // And with the fuel cut there is no spray to light, so there is no
    // ignition point either.
    model.fuel_cut = true;
    assert!(model.latch(&cylinder, omega).autoignition.is_none());
}

#[test]
fn a_longer_delay_puts_more_of_the_charge_in_the_spike() {
    // The one link that makes the clatter physical rather than voiced: the
    // split between the stages is the fraction the injector delivered while
    // the air was still too cold to light it.
    let d = DieselCombustion::default();
    let short = d.premixed_fraction(deg(3.0));
    let long = d.premixed_fraction(deg(12.0));
    assert!(
        long > short,
        "a longer delay must pile up more premixed fuel: {long:.3} against {short:.3}"
    );
    assert!((0.0..=1.0).contains(&d.premixed_fraction(deg(180.0))));
}

#[test]
fn diesel_delay_shortens_with_cetane_pressure_and_temperature() {
    let pump = IgnitionDelay::default();
    let poor = IgnitionDelay {
        cetane_number: 35.0,
        ..IgnitionDelay::default()
    };

    let (p, t) = (50e5, 850.0);
    assert!(
        poor.delay(p, t) > pump.delay(p, t),
        "a low-cetane fuel must be slower to light, not faster"
    );
    // The reference fuel is quoted at the reference cetane, so its prefactor
    // is Wolfer's own number with nothing else on it.
    approx(
        IgnitionDelay {
            cetane_number: REFERENCE_CETANE,
            ..IgnitionDelay::default()
        }
        .delay(BAR, 0.5 * pump.activation_temperature),
        pump.time_constant * std::f64::consts::E.powi(2),
        1e-9,
    );
    assert!(pump.delay(80e5, t) < pump.delay(40e5, t));
    assert!(pump.delay(p, 950.0) < pump.delay(p, 750.0));
    // A real diesel lights within a millisecond or so of the injector
    // opening at top dead centre; anything else is a typo in a coefficient.
    let tdc = pump.delay(55e5, 900.0);
    assert!(
        (0.2e-3..2.0e-3).contains(&tdc),
        "implausible delay at TDC conditions: {:.3} ms",
        tdc * 1e3
    );
    assert!(pump.delay(1e5, 300.0).is_finite());
    assert!(pump.rate(p, t).is_finite() && pump.rate(p, t) > 0.0);
}

#[test]
fn advancing_the_spark_drives_the_knock_integral_up() {
    let env = Environment::default();
    let omega = rpm_to_omega(3000.0);

    let run = |spark_deg: f64| {
        let model = CylinderModel {
            combustion: HeatRelease::Spark(WiebeProfile::new(
                deg(spark_deg),
                deg(60.0),
                5.0,
                2.0,
                0.97,
            )),
            // A 14:1 squeeze to put the end gas firmly into knock territory.
            geometry: CylinderGeometry::new(0.086, 0.086, 0.1345, 14.0),
            ..CylinderModel::default()
        };
        let ports = PortConditions::from_environment(&env, &model.gas);
        let solver = Rk4Solver::default();

        let mut st = ThermoState::at_ambient(&model.geometry, &model.gas, &env);
        st.cylinder.theta = PI;
        st.cylinder.mass = 2.2 * env.pressure * model.geometry.max_volume()
            / (model.gas.r_unburned * env.temperature);
        st.cylinder.temperature = 380.0;
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
        st.knock_integral
    };

    let retarded = run(355.0);
    let advanced = run(325.0);
    assert!(
        advanced > retarded,
        "advancing the spark must raise I_knock: {advanced} vs {retarded}"
    );
    assert!(advanced.is_finite() && retarded >= 0.0);
}

#[test]
fn watchdog_resets_a_corrupted_state_and_keeps_the_phase() {
    let model = CylinderModel::default();
    let env = Environment::default();
    let mut dog = Watchdog::default();

    let mut st = ThermoState::at_ambient(&model.geometry, &model.gas, &env);
    st.cylinder.theta = deg(123.0);
    st.cylinder.temperature = f64::NAN;
    st.cylinder.mass = f64::INFINITY;

    assert!(dog.guard(&mut st, &model.geometry, &model.gas, &env));
    assert_eq!(dog.resets, 1);
    assert!(dog.tripped);
    assert!(!st.is_corrupt());
    approx(st.cylinder.theta.to_degrees(), 123.0, 1e-9);
    approx(st.cylinder.temperature, env.temperature, 1e-9);

    // A healthy state passes through untouched and does not count a reset.
    let before = st;
    assert!(!dog.guard(&mut st, &model.geometry, &model.gas, &env));
    assert_eq!(dog.resets, 1);
    assert_eq!(st, before);
}

#[test]
fn watchdog_recovers_the_solver_from_an_injected_nan() {
    let model = CylinderModel::default();
    let env = Environment::default();
    let ports = PortConditions::from_environment(&env, &model.gas);
    let mut solver = Rk4Solver::default();
    let omega = rpm_to_omega(3000.0);

    let mut st = ThermoState::at_ambient(&model.geometry, &model.gas, &env);
    st.cylinder.temperature = f64::NAN;

    let report = solver.step_frame(&model, &mut st, omega, 1.0 / 60.0, &ports, &env, |_, _| {});
    assert!(report.watchdog_tripped);
    assert!(!st.is_corrupt());
    assert!(st
        .cylinder
        .pressure(&model.geometry, &model.gas)
        .is_finite());
    assert!(solver.watchdog.resets >= 1);
}

#[test]
fn adaptive_budget_clamps_lag_spikes_and_stretches_dtheta() {
    let budget = AdaptiveBudget::default();
    let omega = rpm_to_omega(7000.0);

    // A 60 Hz frame at redline still fits the budget at the nominal step:
    // stretching is meant to be a lag response, not the normal case.
    let normal = budget.plan(1.0 / 60.0, omega);
    assert!(!normal.clamped && !normal.stretched);
    assert!(normal.dtheta <= budget.base_dtheta + 1e-12);
    approx(normal.dtheta * normal.substeps as f64, omega / 60.0, 1e-9);

    // A two-second stall is clamped to 1/30 s: the engine cannot skip ahead.
    let spike = budget.plan(2.0, omega);
    assert!(spike.clamped);
    approx(spike.dt, 1.0 / 30.0, 1e-15);
    assert!(spike.substeps <= budget.max_substeps);
    // The clamp is what caps the work, so the same travel is covered either
    // way: 1/30 s at 7000 rpm is 1400 degrees, which does need stretching.
    assert!(spike.stretched);
    assert!(spike.dtheta <= budget.max_dtheta + 1e-12);
    approx(spike.dtheta * spike.substeps as f64, omega / 30.0, 1e-9);

    // Once dtheta hits the accuracy floor the substep count is allowed to
    // exceed the target: correctness wins over the CPU budget, and the
    // frame clamp is what stops that from being unbounded.
    let fast = budget.plan(1.0 / 30.0, rpm_to_omega(20_000.0));
    approx(fast.dtheta, budget.max_dtheta, 1e-3);
    assert!(fast.substeps > budget.target_substeps);
    assert!(fast.substeps <= budget.max_substeps);

    // Garbage input degrades to a dropped frame rather than a NaN plan.
    for bad in [f64::NAN, f64::INFINITY, -1.0] {
        let p = budget.plan(bad, omega);
        assert!(p.dt.is_finite() && p.dt >= 0.0 && p.dtheta.is_finite());
    }
    // A stopped engine asks for no substeps at all.
    assert_eq!(budget.plan(1.0 / 60.0, 0.0).substeps, 0);
}

#[test]
fn crossed_is_wrap_aware() {
    assert!(crossed(deg(719.0), deg(4.0), deg(2.0)));
    assert!(crossed(deg(10.0), deg(4.0), deg(12.0)));
    assert!(!crossed(deg(10.0), deg(1.0), deg(20.0)));
    // The interval is half-open `[from, from + dtheta)`: the start angle
    // counts, the end angle belongs to the next step. That is what makes
    // consecutive steps catch a target exactly once.
    assert!(crossed(deg(20.0), deg(4.0), deg(20.0)));
    assert!(!crossed(deg(20.0), deg(4.0), deg(24.0)));
    assert!(crossed(deg(24.0), deg(4.0), deg(24.0)));

    // Walking a whole cycle at any stride must trip a given target once.
    for stride_deg in [0.25, 1.0, 3.0, 4.0] {
        let stride = deg(stride_deg);
        let steps = (CYCLE_ANGLE / stride).round() as usize;
        let hits = (0..steps)
            .filter(|i| crossed(stride * *i as f64, stride, deg(220.0)))
            .count();
        assert_eq!(hits, 1, "target hit {hits} times at {stride_deg} deg/step");
    }
}

/// Walks a full four-stroke cycle with the valves live and checks the gas
/// exchange actually happened: the cylinder must breathe in and blow down.
#[test]
fn full_cycle_breathes_and_stays_bounded() {
    let model = CylinderModel::default();
    let env = Environment::default();
    let ports = PortConditions::from_environment(&env, &model.gas);
    let solver = Rk4Solver::default();
    let omega = rpm_to_omega(3000.0);

    let mut st = ThermoState::at_ambient(&model.geometry, &model.gas, &env);
    let mut min_mass = f64::MAX;
    let mut max_mass: f64 = 0.0;
    let mut max_pressure: f64 = 0.0;

    // Six cycles: enough for the trapped mass to settle into a limit cycle.
    for _ in 0..(720 * 6) {
        let previous = st.cylinder.theta;
        st = solver.substep(&model, &st, omega, deg(1.0), &ports);
        if crossed(previous, deg(1.0), model.valves.intake.close_angle()) {
            st.latch = CycleLatch {
                fuel_mass: model.trapped_fuel_mass(st.cylinder.mass, st.cylinder.burned_fraction),
                dilution: st.cylinder.burned_fraction,
                pressure: st.cylinder.pressure(&model.geometry, &model.gas),
                temperature: st.cylinder.temperature,
                volume: model.geometry.safe_volume(st.cylinder.theta),
                gamma: model.gas.gamma(st.cylinder.burned_fraction),
                autoignition: None,
            };
        }
        let p = st.cylinder.pressure(&model.geometry, &model.gas);
        assert!(p.is_finite() && st.cylinder.temperature.is_finite());
        min_mass = min_mass.min(st.cylinder.mass);
        max_mass = max_mass.max(st.cylinder.mass);
        max_pressure = max_pressure.max(p);
    }

    assert!(
        max_mass > min_mass * 5.0,
        "cylinder did not breathe: mass ranged {min_mass} to {max_mass}"
    );
    assert!(
        (20e5..250e5).contains(&max_pressure),
        "peak pressure {max_pressure} Pa is outside any plausible SI engine"
    );
}

#[test]
fn a_stopped_engine_does_not_move_the_state() {
    let model = CylinderModel::default();
    let env = Environment::default();
    let ports = PortConditions::from_environment(&env, &model.gas);
    let solver = Rk4Solver::default();
    let st = ThermoState::at_ambient(&model.geometry, &model.gas, &env);
    assert_eq!(solver.substep(&model, &st, 0.0, deg(1.0), &ports), st);
}
