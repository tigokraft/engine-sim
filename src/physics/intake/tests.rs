use super::*;

fn approx(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() <= tol, "expected {b}, got {a} (tol {tol})");
}

fn gas() -> GasProperties {
    GasProperties::default()
}

fn env() -> Environment {
    Environment::default()
}

/// A 4.0 litre V8's worth of plenum on the default 70 mm throttle.
fn plenum() -> IntakePlenum {
    IntakePlenum::at_ambient(3.2e-3, ThrottleBody::default(), &env(), &gas())
}

fn upstream() -> PortState {
    IntakePlenum::ambient_upstream(&env(), &gas())
}

/// The mass flow a bank of cylinders takes at `rpm`, as a volumetric pump.
///
/// A real cylinder swallows a *volume* per cycle and gets whatever mass the
/// manifold density puts in it, so the draw has to fall with manifold
/// pressure. A fixed mass draw would happily pump the plenum to absolute
/// vacuum, which is exactly the error this coupling exists to avoid.
fn pumped_draw(plenum: &IntakePlenum, displacement: f64, rpm: f64, ve: f64) -> ValveDraw {
    let volume_rate = displacement * (rpm / 60.0) * 0.5 * ve;
    ValveDraw::new(plenum.density() * volume_rate, 900.0)
}

#[test]
fn critical_pressure_ratio_matches_the_isentropic_formula() {
    for gamma in [1.26, 1.35, 1.4] {
        let expected = (2.0 / (gamma + 1.0f64)).powf(gamma / (gamma - 1.0));
        approx(
            ThrottleBody::critical_pressure_ratio(gamma),
            expected,
            1e-12,
        );
    }
    // Air at the charge gamma chokes just above half an atmosphere.
    approx(ThrottleBody::critical_pressure_ratio(1.35), 0.5367, 1e-3);
}

#[test]
fn throttle_area_sweeps_from_the_leak_to_the_full_bore() {
    let body = ThrottleBody::default();
    let bore = body.bore_area();

    approx(body.open_area(0.0), bore * body.leak_area_fraction, 1e-12);
    approx(body.open_area(1.0), bore, 1e-12);
    approx(
        body.effective_area(1.0),
        body.discharge_coefficient * bore,
        1e-12,
    );

    // Monotonic in pedal, and gentle where it needs to be: the first 10 %
    // of travel must open far less area than the last 10 %.
    let mut previous = body.open_area(0.0);
    for i in 1..=100 {
        let area = body.open_area(i as f64 / 100.0);
        assert!(area >= previous, "area went backwards at TPS {i}%");
        previous = area;
    }
    let first = body.open_area(0.1) - body.open_area(0.0);
    let last = body.open_area(1.0) - body.open_area(0.9);
    assert!(
        first < last * 0.5,
        "tip-in is too aggressive: {first} vs {last}"
    );
}

#[test]
fn throttle_area_is_clamped_outside_the_pedal_range() {
    let body = ThrottleBody::default();
    approx(body.open_area(-1.0), body.open_area(0.0), 1e-12);
    approx(body.open_area(7.0), body.open_area(1.0), 1e-12);
}

#[test]
fn throttle_flow_chokes_below_the_critical_pressure_ratio() {
    let body = ThrottleBody::default();
    let up = upstream();
    let critical = ThrottleBody::critical_pressure_ratio(up.gamma);

    let at = |ratio: f64| {
        let mut manifold = up;
        manifold.pressure = up.pressure * ratio;
        (
            body.mass_flow(1.0, &up, &manifold),
            body.is_choked(&up, manifold.pressure),
        )
    };

    let (deep, deep_choked) = at(0.05);
    let (mid, mid_choked) = at(0.30);
    let (edge, edge_choked) = at(critical * 0.999);
    assert!(deep_choked && mid_choked && edge_choked);

    // The defining property: below critical the throat cannot hear the
    // manifold, so pulling more vacuum buys no more air.
    approx(mid, deep, deep.abs() * 1e-9);
    approx(edge, deep, deep.abs() * 1e-6);

    // And it equals the analytic sonic plateau.
    let g: f64 = up.gamma;
    let psi = g.sqrt() * (2.0 / (g + 1.0)).powf((g + 1.0) / (2.0 * (g - 1.0)));
    let expected =
        body.effective_area(1.0) * up.pressure / (up.gas_constant * up.temperature).sqrt() * psi;
    approx(deep, expected, expected * 1e-9);

    // Just above critical it must already be falling away from the plateau.
    let (subsonic, subsonic_choked) = at(critical * 1.05);
    assert!(!subsonic_choked);
    assert!(subsonic < deep, "subsonic flow should be below the plateau");
}

#[test]
fn throttle_flow_vanishes_with_no_pressure_difference() {
    let body = ThrottleBody::default();
    let up = upstream();
    approx(body.mass_flow(1.0, &up, &up), 0.0, 1e-12);
}

#[test]
fn throttle_flow_reverses_when_the_manifold_is_above_the_airbox() {
    let body = ThrottleBody::default();
    let up = upstream();
    let mut boosted = up;
    boosted.pressure = up.pressure * 1.8;
    assert!(
        body.mass_flow(1.0, &up, &boosted) < 0.0,
        "flow must run back out of the plate"
    );
}

#[test]
fn a_shut_throttle_still_passes_the_leak_but_no_more() {
    let body = ThrottleBody::default();
    let up = upstream();
    let mut manifold = up;
    manifold.pressure = 20_000.0;

    let shut = body.mass_flow(0.0, &up, &manifold);
    let open = body.mass_flow(1.0, &up, &manifold);
    assert!(shut > 0.0, "a shut plate is not a seal");
    assert!(
        shut < open * 0.02,
        "the leak is doing too much work: {shut} vs {open}"
    );

    let sealed = ThrottleBody::new(0.070, 0.9, 0.0);
    approx(sealed.mass_flow(0.0, &up, &manifold), 0.0, 1e-12);
}

#[test]
fn the_ideal_gas_closure_holds_after_every_step() {
    let mut p = plenum();
    let up = upstream();
    for i in 0..200 {
        let tps = (i as f64 / 200.0).sin().abs();
        let draw = pumped_draw(&p, 4.0e-3, 3000.0, 0.85);
        p.advance(1.0e-3, tps, &up, &[draw]);
        approx(
            p.pressure(),
            p.mass * p.gas_constant * p.temperature / p.volume,
            1e-9,
        );
        assert!(p.pressure().is_finite() && p.pressure() > 0.0);
    }
}

#[test]
fn an_open_throttle_fills_to_ambient_and_stops_there() {
    let mut p = IntakePlenum::new(
        3.2e-3,
        ThrottleBody::default(),
        20_000.0,
        env().temperature,
        gas().r_unburned,
        gas().gamma_unburned,
    );
    let up = upstream();

    for _ in 0..2_000 {
        p.advance(1.0e-3, 1.0, &up, &[]);
    }

    // Equalises, and critically does not sail past: an unclamped explicit
    // plenum overshoots here and quietly supercharges the engine.
    approx(p.pressure(), up.pressure, up.pressure * 1e-4);
    assert!(
        p.pressure() <= up.pressure * (1.0 + 1e-6),
        "a throttle cannot supercharge: {} Pa",
        p.pressure()
    );
}

/// Independent check on the energy equation against a closed-form result.
///
/// Charging a rigid, adiabatic, initially evacuated vessel from a large
/// reservoir is the textbook transient-filling problem, and its answer does
/// not depend on the orifice at all:
///
/// ```text
/// u_final = h_reservoir   =>   c_v T_final = c_p T_up   =>   T_final = gamma T_up
/// ```
///
/// Every incoming parcel arrives with its flow work included, and with no
/// wall and no outflow there is nowhere for that work to go but internal
/// energy. If the `c_p` on the inflow term were mistakenly a `c_v`, or the
/// `c_v T dm/dt` product-rule term were dropped, this lands on `T_up`
/// instead and the error would be invisible in a steady-state test.
#[test]
fn adiabatic_filling_of_an_evacuated_plenum_lands_on_gamma_times_upstream() {
    let gas = gas();
    let up = upstream();
    let mut p = IntakePlenum::new(
        3.2e-3,
        ThrottleBody::default(),
        up.pressure * 1e-4,
        up.temperature,
        gas.r_unburned,
        gas.gamma_unburned,
    );
    p.wall_conductance = 0.0;

    for _ in 0..5_000 {
        p.advance(1.0e-3, 1.0, &up, &[]);
    }

    let expected = p.gamma * up.temperature;
    // The residual gap is the charge that was already in there at ambient
    // temperature, which is 0.01 % of the final mass.
    approx(p.temperature, expected, expected * 2e-3);
    approx(p.pressure(), up.pressure, up.pressure * 1e-4);
}

#[test]
fn a_shut_throttle_under_draw_pulls_a_deep_vacuum() {
    let mut p = plenum();
    let up = upstream();

    for _ in 0..3_000 {
        let draw = pumped_draw(&p, 4.0e-3, 2000.0, 0.85);
        p.advance(1.0e-3, 0.0, &up, &[draw]);
    }

    let map = p.pressure();
    assert!(
        map < up.pressure * 0.5,
        "a shut throttle at 2000 rpm should be a strong vacuum, got {map} Pa"
    );
    assert!(map > 0.0 && map.is_finite());
    assert!(
        p.gauge_pressure(up.pressure) < 0.0,
        "gauge pressure must read vacuum"
    );
}

/// The headline transient: 0 % to 100 % pedal in one frame.
#[test]
fn a_throttle_snap_decays_the_manifold_vacuum_and_stabilizes() {
    let mut p = plenum();
    let up = upstream();
    let ambient = up.pressure;
    let rpm = 2000.0;
    let displacement = 4.0e-3;
    let frame = 1.0e-3;

    // Settle on a shut throttle first, so the snap starts from real vacuum.
    for _ in 0..3_000 {
        let draw = pumped_draw(&p, displacement, rpm, 0.85);
        p.advance(frame, 0.0, &up, &[draw]);
    }
    let idle_map = p.pressure();
    assert!(
        idle_map < ambient * 0.5,
        "expected vacuum before the snap, got {idle_map} Pa"
    );

    // Snap to wide open and watch the vacuum collapse.
    let mut trace = Vec::new();
    let mut peak_charge_temperature = p.temperature;
    for _ in 0..600 {
        let draw = pumped_draw(&p, displacement, rpm, 0.85);
        p.advance(frame, 1.0, &up, &[draw]);
        trace.push(p.pressure());
        peak_charge_temperature = peak_charge_temperature.max(p.temperature);
    }

    // 1. The vacuum decays monotonically for as long as it is decaying.
    //
    //    The plenum has two relaxation modes and they are decades apart: the
    //    mass channel equalises with the airbox in a few milliseconds, while
    //    the charge temperature — spiked by the fill itself — bleeds off to
    //    the wall over hundreds. Once the plate has stopped flowing, the
    //    still-cooling charge drops `P = m R T / V` by a pascal or so before
    //    the plate tops the mass back up. So monotonicity is asserted where
    //    it is physically required, over the fill, and the settled tail is
    //    checked separately below. Asserting it over both would be
    //    asserting that the manifold has one state, and it has two.
    let fill_ceiling = ambient * 0.995;
    let mut filling = trace
        .iter()
        .position(|&p| p >= fill_ceiling)
        .expect("the manifold never filled");
    filling = filling.max(1);
    for pair in trace[..=filling].windows(2) {
        assert!(
            pair[1] > pair[0],
            "manifold pressure fell during the fill: {pair:?}"
        );
    }
    assert!(
        trace[0] > idle_map,
        "the first frame after the snap should already be filling"
    );

    // The fill is a few milliseconds, not a few hundred: this is a plenum
    // being equalised through a 70 mm bore, not a slow leak.
    assert!(
        filling < 20,
        "a wide-open 70 mm bore should fill 3.2 litres in milliseconds, took {filling} ms"
    );

    // 2. It gets most of the way there fast. A 3.2 litre plenum behind a
    //    70 mm bore has a filling time of a few milliseconds, so 50 ms of
    //    pedal is many time constants.
    let at_50ms = trace[49];
    assert!(
        at_50ms > idle_map + 0.9 * (ambient - idle_map),
        "fill is far too slow: {at_50ms} Pa after 50 ms"
    );

    // 3. It stabilises, and at a pressure just below ambient — the residual
    //    depression is the pumping loss across a wide-open plate, which is
    //    small but must not be zero.
    let settled = *trace.last().unwrap();
    let previous = trace[trace.len() - 51];
    assert!(
        (settled - previous).abs() < ambient * 1e-4,
        "still moving at the end of the pull: {previous} -> {settled} Pa"
    );
    assert!(
        settled > ambient * 0.9,
        "wide-open throttle should be near ambient, got {settled} Pa"
    );
    assert!(
        settled < ambient,
        "a naturally aspirated manifold cannot exceed ambient, got {settled} Pa"
    );

    // 4. Past the fill, nothing rings: the tail is a slow thermal drift of
    //    a few pascals, not an oscillation about ambient. An explicit
    //    plenum stepped past its filling time constant fails here.
    let tail = &trace[filling..];
    let (lo, hi) = tail
        .iter()
        .fold((f64::MAX, f64::MIN), |(lo, hi), &p| (lo.min(p), hi.max(p)));
    assert!(
        hi - lo < ambient * 1e-3,
        "the settled manifold is ringing: {lo} .. {hi} Pa"
    );

    // 5. The gauge agrees: vacuum has decayed to almost nothing.
    assert!(p.gauge_pressure(ambient) < 0.0);
    assert!(p.gauge_pressure(ambient) > -0.1 * ambient);

    // 6. Filling a near-vacuum is irreversible, so the charge arrives hot:
    //    the flow work done pushing air into the plenum shows up as
    //    temperature, heading for `gamma * T_up` in the limit of filling
    //    from nothing. A plenum that fills isothermally has lost that term.
    let limit = p.gamma * up.temperature;
    assert!(
        peak_charge_temperature > up.temperature + 40.0,
        "the fill should have heated the charge, peaked at {peak_charge_temperature} K"
    );
    assert!(
        peak_charge_temperature < limit,
        "charge cannot exceed the filling limit {limit} K, got {peak_charge_temperature} K"
    );
    // And it sheds that heat afterwards, back towards the wall.
    assert!(
        p.temperature < peak_charge_temperature - 20.0,
        "the hot charge should have relaxed, sitting at {} K",
        p.temperature
    );
}

#[test]
fn a_throttle_lift_restores_the_vacuum() {
    let mut p = plenum();
    let up = upstream();
    let draws = |p: &IntakePlenum| [pumped_draw(p, 4.0e-3, 2500.0, 0.85)];

    for _ in 0..1_000 {
        p.advance(1.0e-3, 1.0, &up, &draws(&p));
    }
    let wide_open = p.pressure();

    for _ in 0..1_000 {
        p.advance(1.0e-3, 0.0, &up, &draws(&p));
    }
    let lifted = p.pressure();

    assert!(
        lifted < wide_open * 0.5,
        "lifting off must re-establish vacuum: {wide_open} -> {lifted} Pa"
    );
}

#[test]
fn manifold_pressure_rises_monotonically_with_pedal() {
    let up = upstream();
    let mut previous = 0.0;

    for step in 0..=10 {
        let tps = step as f64 / 10.0;
        let mut p = plenum();
        for _ in 0..2_000 {
            let draw = pumped_draw(&p, 4.0e-3, 2500.0, 0.85);
            p.advance(1.0e-3, tps, &up, &[draw]);
        }
        let map = p.pressure();
        assert!(
            map > previous,
            "MAP did not rise from TPS {tps}: {previous} -> {map} Pa"
        );
        previous = map;
    }
    assert!(previous < up.pressure, "still cannot exceed ambient");
}

#[test]
fn reversion_from_a_hot_cylinder_warms_the_plenum() {
    let mut p = plenum();
    // Isolate the valve term from the wall term.
    p.wall_conductance = 0.0;
    let up = upstream();
    let before = p.temperature;

    // Overlap reversion: mass pushed back up the runner at exhaust
    // temperature, with the plate shut so nothing cool dilutes it.
    let reversion = ValveDraw::new(-2.0e-3, 1_100.0);
    for _ in 0..50 {
        p.advance(1.0e-4, 0.0, &up, &[reversion]);
    }

    assert!(
        p.temperature > before + 1.0,
        "hot reversion must heat the charge: {before} -> {} K",
        p.temperature
    );
    assert!(p.pressure() > up.pressure * 0.9, "and pressurise it");
}

#[test]
fn a_heat_soaked_wall_warms_the_charge_and_thins_it() {
    let up = upstream();
    let draw_rate = |p: &IntakePlenum| [pumped_draw(p, 4.0e-3, 2500.0, 0.85)];

    let mut cold = plenum();
    cold.wall_temperature = env().temperature;
    cold.wall_conductance = 5.0;

    let mut soaked = plenum();
    soaked.wall_temperature = env().temperature + 80.0;
    soaked.wall_conductance = 5.0;

    for _ in 0..2_000 {
        cold.advance(1.0e-3, 0.6, &up, &draw_rate(&cold));
        soaked.advance(1.0e-3, 0.6, &up, &draw_rate(&soaked));
    }

    assert!(
        soaked.temperature > cold.temperature + 1.0,
        "the hot wall must raise charge temperature: {} vs {} K",
        soaked.temperature,
        cold.temperature
    );
    assert!(
        soaked.density() < cold.density(),
        "warmer charge at a similar pressure must be less dense"
    );
}

#[test]
fn a_coarse_frame_reaches_the_same_steady_state_as_a_fine_one() {
    let up = upstream();
    let settle = |frame: f64, frames: usize| {
        let mut p = plenum();
        for _ in 0..frames {
            let draw = pumped_draw(&p, 4.0e-3, 2500.0, 0.85);
            p.advance(frame, 0.45, &up, &[draw]);
        }
        p.pressure()
    };

    // Same 2 s of simulated time at 0.1 ms and at 20 ms per frame. The
    // internal sub-stepping is what makes these agree; a single RK4 step
    // per 20 ms frame spans several filling time constants.
    let fine = settle(1.0e-4, 20_000);
    let coarse = settle(2.0e-2, 100);
    approx(coarse, fine, fine * 5e-3);
}

#[test]
fn the_plenum_survives_nonsense_inputs() {
    let mut p = plenum();
    let up = upstream();
    let reference = p.pressure();

    for dt in [0.0, -1.0e-3, f64::NAN, f64::INFINITY] {
        p.advance(dt, 1.0, &up, &[]);
        approx(p.pressure(), reference, 1e-9);
    }

    // A NaN pedal is treated as shut, and a NaN draw is ignored rather
    // than laundered into the state.
    p.advance(1.0e-3, f64::NAN, &up, &[ValveDraw::new(f64::NAN, 300.0)]);
    assert!(p.pressure().is_finite() && p.pressure() > 0.0);
    assert!(p.temperature.is_finite() && p.temperature > 0.0);

    // Absurd geometry is clamped at construction, not left to divide by zero.
    let tiny = IntakePlenum::new(0.0, ThrottleBody::new(0.0, 5.0, -1.0), 0.0, 0.0, 0.0, 0.0);
    assert!(tiny.volume > 0.0);
    assert!(tiny.mass >= MASS_FLOOR);
    assert!(tiny.pressure().is_finite());
}

#[test]
fn a_pumped_dry_plenum_does_not_produce_a_singular_temperature() {
    let mut p = plenum();
    let up = upstream();
    // Sealed plate, absurd draw: the control volume is being emptied faster
    // than anything can refill it.
    p.throttle = ThrottleBody::new(0.070, 0.9, 0.0);
    for _ in 0..1_000 {
        p.advance(1.0e-3, 0.0, &up, &[ValveDraw::new(1.0, 900.0)]);
    }
    assert!(p.mass >= MASS_FLOOR);
    assert!(p.temperature.is_finite());
    assert!((TEMPERATURE_BOUNDS.0..=TEMPERATURE_BOUNDS.1).contains(&p.temperature));
    assert!(p.pressure().is_finite() && p.pressure() >= 0.0);
}

#[test]
fn the_gas_property_accessors_agree_with_their_definitions() {
    let p = plenum();
    approx(p.cv(), p.gas_constant / (p.gamma - 1.0), 1e-12);
    approx(p.cp(), p.gamma * p.cv(), 1e-12);
    // Mayer's relation, as a cross-check that c_p and c_v are consistent.
    approx(p.cp() - p.cv(), p.gas_constant, 1e-9);
    approx(
        p.speed_of_sound(),
        (p.gamma * p.gas_constant * p.temperature).sqrt(),
        1e-12,
    );
    approx(p.density(), p.mass / p.volume, 1e-12);
    // ~340 m/s in ambient charge air.
    assert!((300.0..380.0).contains(&p.speed_of_sound()));
}

#[test]
fn the_throttle_flow_helper_matches_the_orifice_it_wraps() {
    let mut p = plenum();
    let up = upstream();
    for _ in 0..500 {
        let draw = pumped_draw(&p, 4.0e-3, 3000.0, 0.85);
        p.advance(1.0e-3, 0.3, &up, &[draw]);
    }
    // The convenience wrapper must solve the same orifice against the same
    // closure the RK4 stages use, or telemetry and physics disagree.
    approx(
        p.throttle_flow(0.3, &up),
        p.throttle.mass_flow(0.3, &up, &p.port_state()),
        1e-15,
    );
    assert!(p.throttle_flow(0.3, &up) > 0.0, "should be drawing air in");
    assert!(
        p.throttle_flow(1.0, &up) > p.throttle_flow(0.3, &up),
        "more pedal at the same manifold state must pass more air"
    );
}

#[test]
fn wall_heat_flow_is_signed_out_of_the_charge() {
    let mut p = plenum();
    p.wall_conductance = 5.0;

    // Charge hotter than the wall: heat leaves the gas, so positive.
    p.wall_temperature = 300.0;
    p.temperature = 400.0;
    approx(p.wall_heat_flow(), 5.0 * 100.0, 1e-9);

    // Heat-soaked wall: the sign flips and the charge is being warmed.
    p.wall_temperature = 400.0;
    p.temperature = 300.0;
    approx(p.wall_heat_flow(), -5.0 * 100.0, 1e-9);

    // In equilibrium, and with no conductance, it is exactly nothing.
    p.temperature = 400.0;
    approx(p.wall_heat_flow(), 0.0, 1e-12);
    p.wall_conductance = 0.0;
    p.temperature = 300.0;
    approx(p.wall_heat_flow(), 0.0, 1e-12);
}

#[test]
fn the_port_state_is_the_boundary_condition_the_valves_see() {
    let mut p = plenum();
    let up = upstream();
    for _ in 0..1_000 {
        let draw = pumped_draw(&p, 4.0e-3, 3000.0, 0.85);
        p.advance(1.0e-3, 0.25, &up, &[draw]);
    }

    let port = p.port_state();
    approx(port.pressure, p.pressure(), 1e-9);
    approx(port.temperature, p.temperature, 1e-12);
    approx(port.gas_constant, p.gas_constant, 1e-12);
    approx(port.gamma, p.gamma, 1e-12);
    approx(port.burned_fraction, 0.0, 1e-12);

    // It must be a *throttled* condition, not ambient: that difference is
    // the entire point of feeding it to the cylinders.
    assert!(port.pressure < up.pressure * 0.9);
    approx(port.cp(), p.cp(), 1e-9);
}

fn turbine_map() -> TurbineMap {
    use crate::physics::turbine::{TurbineMapPoint, TurbineSpeedLine};
    let point = |expansion_ratio: f64, reduced_flow: f64, efficiency: f64| TurbineMapPoint {
        expansion_ratio,
        reduced_flow,
        efficiency,
    };
    TurbineMap::new(vec![
        TurbineSpeedLine::new(
            40_000.0,
            vec![
                point(1.0, 0.015, 0.45),
                point(1.5, 0.045, 0.66),
                point(2.5, 0.075, 0.58),
            ],
        ),
        TurbineSpeedLine::new(
            140_000.0,
            vec![
                point(1.0, 0.025, 0.50),
                point(1.8, 0.085, 0.72),
                point(3.0, 0.135, 0.60),
            ],
        ),
    ])
}

fn forced_induction() -> ForcedInduction {
    ForcedInduction::new(
        crate::physics::compressor::CompressorMap::stock(
            crate::physics::compressor::FrameSize::Small,
        ),
        turbine_map(),
        6.0e-5,
        0.98,
        BearingType::Journal,
        1.5e-3,
        None,
        &env(),
        &gas(),
    )
}

/// Spins the shaft up over `seconds` under a hot, high pressure-ratio
/// exhaust and a low downstream pressure, the way a boosted engine would.
fn spool_up(fi: &mut ForcedInduction, seconds: f64) {
    let hot_upstream = PortState {
        pressure: 250_000.0,
        temperature: 1000.0,
        gas_constant: gas().r_burned,
        gamma: gas().gamma_burned,
        burned_fraction: 1.0,
    };
    let dt = 1.0e-3;
    let mut t = 0.0;
    while t < seconds {
        fi.advance_exhaust(dt, &hot_upstream, env().pressure, 1.0);
        t += dt;
    }
}

/// Runs `advance_intake` at a fixed throttle draw until the charge pipe's
/// capacitance has settled, and returns the resulting pressure.
fn settled_charge_pipe_pressure(fi: &mut ForcedInduction, throttle_flow: f64) -> f64 {
    let mut pressure = fi.charge_pipe.pressure();
    for _ in 0..2_000 {
        pressure = fi
            .advance_intake(1.0e-3, throttle_flow, env().pressure, &env(), &gas())
            .pressure;
    }
    pressure
}

#[test]
fn a_spun_up_shaft_raises_the_charge_pipe_further_above_ambient() {
    // Even a stalled shaft reads as the map's slowest cataloged speed
    // line — the same nearest-neighbour convention
    // `turbine::turbine_operating_point` documents — so "at rest" already
    // carries a small pressure ratio. What must still hold is that a
    // genuinely spun-up shaft settles well past that floor.
    let mut fi = forced_induction();
    let at_rest = settled_charge_pipe_pressure(&mut fi, 0.02);

    let mut spun = forced_induction();
    spool_up(&mut spun, 2.0);
    assert!(
        spun.shaft.shaft_rpm() > 1_000.0,
        "shaft failed to spool: {}",
        spun.shaft.shaft_rpm()
    );

    let boosted = settled_charge_pipe_pressure(&mut spun, 0.02);
    assert!(
        boosted > at_rest * 1.1,
        "spinning the shaft must settle the charge pipe well past its stalled reading: \
         at rest {at_rest}, boosted {boosted}"
    );
}

#[test]
fn an_intercooler_lowers_the_charge_temperature_at_the_same_operating_point() {
    let mut hot = forced_induction();
    let mut cooled = ForcedInduction {
        intercooler: Some(Intercooler {
            core_volume: 1.0e-3,
            effectiveness: 0.7,
            loss_coefficient: 0.0,
        }),
        ..forced_induction()
    };

    spool_up(&mut hot, 2.0);
    spool_up(&mut cooled, 2.0);
    // Match the shafts exactly so the comparison isolates the intercooler.
    cooled.shaft = hot.shaft;

    let hot_state = hot.advance_intake(1.0e-3, 0.03, env().pressure, &env(), &gas());
    let cooled_state = cooled.advance_intake(1.0e-3, 0.03, env().pressure, &env(), &gas());
    assert!(
        cooled_state.temperature < hot_state.temperature,
        "intercooled charge ({}) must be cooler than bare ({})",
        cooled_state.temperature,
        hot_state.temperature
    );
}

#[test]
fn compressor_power_rises_with_shaft_speed() {
    let mut idle = forced_induction();
    idle.advance_intake(1.0e-3, 0.02, env().pressure, &env(), &gas());
    let idle_power = idle.compressor_power;

    let mut spun = forced_induction();
    spool_up(&mut spun, 2.0);
    spun.advance_intake(1.0e-3, 0.02, env().pressure, &env(), &gas());
    assert!(
        spun.compressor_power > idle_power,
        "spinning the shaft must draw more compressor power: idle {idle_power}, spun {}",
        spun.compressor_power
    );
}

#[test]
fn a_real_expansion_ratio_spools_the_shaft_and_a_flat_one_does_not() {
    let mut spooling = forced_induction();
    spool_up(&mut spooling, 1.0);
    assert!(spooling.shaft.shaft_rpm() > 100.0);

    let mut idle = forced_induction();
    let flat = PortState {
        pressure: env().pressure,
        temperature: env().temperature,
        gas_constant: gas().r_burned,
        gamma: gas().gamma_burned,
        burned_fraction: 1.0,
    };
    for _ in 0..1_000 {
        idle.advance_exhaust(1.0e-3, &flat, env().pressure, 1.0);
    }
    assert!(idle.shaft.shaft_rpm() < 1.0);
}
