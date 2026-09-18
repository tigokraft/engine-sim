use super::*;
use crate::physics::cylinder::deg;

fn approx(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() <= tol, "expected {b}, got {a} (tol {tol})");
}

// -- phase ring ---------------------------------------------------------

#[test]
fn phase_index_is_one_based_over_the_whole_cycle() {
    assert_eq!(PhaseRing::phase_index(deg(0.0)), 1);
    assert_eq!(PhaseRing::phase_index(deg(0.5)), 1);
    assert_eq!(PhaseRing::phase_index(deg(1.0)), 2);
    assert_eq!(PhaseRing::phase_index(deg(719.9)), 720);
    // Wraps rather than overflowing.
    assert_eq!(PhaseRing::phase_index(deg(720.0)), 1);
    // Whole-degree angles must snap to the cell they open, not the one
    // below, however the radian round-trip rounds.
    assert_eq!(PhaseRing::phase_index(deg(725.0)), 6);
    assert_eq!(PhaseRing::phase_index(deg(5.0)), 6);
    assert_eq!(PhaseRing::phase_index(deg(-90.0)), 631);
    for d in [-3600.0, 3600.0, 12345.6, -0.000_001] {
        let idx = PhaseRing::phase_index(deg(d));
        assert!((1..=PHASE_CELLS).contains(&idx), "index {idx} out of range");
    }
}

/// Plays a downsampled table back the way the audio thread does: linear
/// interpolation between the two points either side of a cycle phase.
fn playback(table: &[f32; CYCLE_TABLE], phase: f64) -> f64 {
    let x = phase.rem_euclid(1.0) * CYCLE_TABLE as f64 - 0.5;
    let i = x.floor().rem_euclid(CYCLE_TABLE as f64) as usize;
    let frac = x - x.floor();
    let a = table[i] as f64;
    let b = table[(i + 1) % CYCLE_TABLE] as f64;
    a + (b - a) * frac
}

#[test]
fn downsampled_cycle_plays_back_the_curve_it_came_from() {
    // A two-cycle-per-revolution pressure curve — four periods over 720
    // degrees, which is far faster than anything the solver's own pressure
    // trace does and therefore a pessimistic test of the resampling.
    let curve = |degrees: f64| 20.0e5 + 15.0e5 * (4.0 * 2.0 * PI * degrees / 720.0).sin();
    let mut ring = PhaseRing::new();
    for cell in 0..PHASE_CELLS {
        ring.record(
            deg(cell as f64 + 0.5),
            PhaseSample {
                pressure: curve(cell as f64 + 0.5),
                ..PhaseSample::default()
            },
        );
    }

    let table = ring.downsample_from(0.0, |s| s.pressure);
    // Read back at the original resolution. Box-averaging over 5.625 cells
    // and interpolating between the results costs a little amplitude at
    // this rate; 3 % of the swing is the whole of the error.
    let mut worst = 0.0f64;
    for cell in 0..PHASE_CELLS {
        let degrees = cell as f64 + 0.5;
        let played = playback(&table, degrees / 720.0);
        worst = worst.max((played - curve(degrees)).abs());
    }
    assert!(worst < 0.03 * 15.0e5, "playback error {worst:.0} Pa");
}

#[test]
fn downsampling_cuts_the_cycle_at_the_origin_it_is_given() {
    let mut ring = PhaseRing::new();
    for cell in 0..PHASE_CELLS {
        ring.record(
            deg(cell as f64 + 0.5),
            PhaseSample {
                pressure: cell as f64,
                ..PhaseSample::default()
            },
        );
    }
    // Cell zero of a table cut at 360 degrees is the mean of cells 360..365.
    let table = ring.downsample_from(deg(360.0), |s| s.pressure);
    approx(table[0] as f64, 362.0, 1e-3);
    // And the table still spans the whole cycle, wrapping at the end.
    let last = table[CYCLE_TABLE - 1] as f64;
    approx(last, 356.5, 1.0);
}

#[test]
fn downsampling_averages_rather_than_decimates() {
    // A cell-to-cell alternation is pure Nyquist for the ring and must not
    // survive into a table running at a fifth of its rate.
    let mut ring = PhaseRing::new();
    for cell in 0..PHASE_CELLS {
        ring.record(
            deg(cell as f64 + 0.5),
            PhaseSample {
                pressure: if cell % 2 == 0 { 1.0 } else { -1.0 },
                ..PhaseSample::default()
            },
        );
    }
    let table = ring.downsample_from(0.0, |s| s.pressure);
    let worst = table.iter().fold(0.0f32, |m, &v| m.max(v.abs()));
    assert!(worst <= 0.2, "alias survived downsampling: {worst}");
}

#[test]
fn ring_round_trips_a_sample_at_its_own_angle() {
    let mut ring = PhaseRing::new();
    let sample = PhaseSample {
        pressure: 42e5,
        ..PhaseSample::default()
    };
    ring.record(deg(123.4), sample);
    approx(ring.sample(deg(123.0)).pressure, 42e5, 0.0);
    approx(ring.sample(deg(123.99)).pressure, 42e5, 0.0);
    // The neighbouring cell was not touched.
    approx(ring.sample(deg(124.0)).pressure, 0.0, 0.0);
    assert_eq!(ring.filled_cells(), 1);
    assert!(!ring.is_primed());
}

#[test]
fn record_span_leaves_no_holes_when_dtheta_is_stretched() {
    let mut ring = PhaseRing::new();
    // Walk the whole cycle in 4-degree strides, the coarsest the budget allows.
    let mut theta = 0.0;
    for i in 0..180 {
        let next = wrap_cycle(theta + deg(4.0));
        ring.record_span(
            theta,
            next,
            PhaseSample {
                pressure: i as f64 + 1.0,
                ..PhaseSample::default()
            },
        );
        theta = next;
    }
    assert!(
        ring.is_primed(),
        "only {} cells filled",
        ring.filled_cells()
    );
    for cell in ring.cells() {
        assert!(cell.pressure > 0.0, "hole left in the ring");
    }
}

#[test]
fn record_span_handles_the_720_degree_wrap() {
    let mut ring = PhaseRing::new();
    let sample = PhaseSample {
        pressure: 7.0,
        ..PhaseSample::default()
    };
    ring.record_span(deg(718.0), deg(2.0), sample);
    // The span covers the cells *entered*: 719, 0, 1 and 2. The cell the
    // step departed from was written by the step before it, which is what
    // makes consecutive spans tile the ring exactly once.
    for d in [719.5, 0.5, 1.5, 2.5] {
        approx(ring.sample(deg(d)).pressure, 7.0, 0.0);
    }
    for d in [717.5, 718.5, 3.5] {
        approx(ring.sample(deg(d)).pressure, 0.0, 0.0);
    }
}

#[test]
fn indicated_work_integrates_a_synthetic_pv_loop() {
    // Put a constant gauge pressure in every cell: net work over a closed
    // cycle must be zero, because the volume returns to where it started.
    let geometry = CylinderGeometry::default();
    let mut ring = PhaseRing::new();
    for i in 0..PHASE_CELLS {
        let theta = deg(i as f64 + 0.5);
        ring.record(
            theta,
            PhaseSample {
                pressure: 3e5,
                dvolume_dtheta: geometry.dvolume_dtheta(theta),
                ..PhaseSample::default()
            },
        );
    }
    approx(ring.indicated_work(0.0), 0.0, 1e-6);

    // Now pressurise only the expansion strokes: the loop must do net
    // positive work.
    for i in 0..PHASE_CELLS {
        let theta = deg(i as f64 + 0.5);
        let expanding = geometry.dvolume_dtheta(theta) > 0.0;
        ring.record(
            theta,
            PhaseSample {
                pressure: if expanding { 30e5 } else { 1e5 },
                dvolume_dtheta: geometry.dvolume_dtheta(theta),
                ..PhaseSample::default()
            },
        );
    }
    assert!(ring.indicated_work(0.0) > 0.0);
}

// -- firing order -------------------------------------------------------

#[test]
fn cross_plane_v8_offsets_follow_the_firing_order() {
    let order = FiringOrder::cross_plane_v8();
    assert_eq!(order.len(), 8);
    approx(order.interval.to_degrees(), 90.0, 1e-12);

    // Cylinders come out sorted by number; 1-8-7-2-6-5-4-3 at 90 degrees.
    let expected = [0.0, 270.0, 630.0, 540.0, 450.0, 360.0, 180.0, 90.0];
    for (cyl, want) in order.cylinders.iter().zip(expected) {
        approx(cyl.firing_offset.to_degrees(), want, 1e-9);
    }
    assert_eq!(order.cylinders[0].number, 1);
    assert_eq!(order.bank_count(), 2);
}

/// The defining acoustic signature of a cross-plane V8: each bank fires
/// unevenly. This is the test that would catch a firing order typo.
#[test]
fn cross_plane_banks_fire_unevenly_and_flat_plane_banks_do_not() {
    let cross = FiringOrder::cross_plane_v8();
    let mut left = cross.bank_gaps(0);
    left.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let left_deg: Vec<f64> = left.iter().map(|g| g.to_degrees()).collect();
    for (got, want) in left_deg.iter().zip([90.0, 180.0, 180.0, 270.0]) {
        approx(*got, want, 1e-9);
    }
    // Gaps must still add up to a full cycle.
    approx(left_deg.iter().sum::<f64>(), 720.0, 1e-9);

    let flat = FiringOrder::flat_plane_v8();
    for gap in flat.bank_gaps(0) {
        approx(gap.to_degrees(), 180.0, 1e-9);
    }
    for gap in flat.bank_gaps(1) {
        approx(gap.to_degrees(), 180.0, 1e-9);
    }
}

#[test]
fn boxer_six_offsets_alternate_banks_evenly() {
    let boxer = FiringOrder::boxer_six();
    assert_eq!(boxer.len(), 6);
    approx(boxer.interval.to_degrees(), 120.0, 1e-12);
    assert_eq!(boxer.bank_count(), 2);

    // Sequence is 1-6-2-4-3-5.
    // Firing times [deg]:
    // Cyl 1 (bank 0): 0 deg
    // Cyl 6 (bank 1): 120 deg
    // Cyl 2 (bank 0): 240 deg
    // Cyl 4 (bank 1): 360 deg
    // Cyl 3 (bank 0): 480 deg
    // Cyl 5 (bank 1): 600 deg
    // Sorted by cylinder number 1..=6:
    // [0.0, 240.0, 480.0, 360.0, 600.0, 120.0]
    let expected = [0.0, 240.0, 480.0, 360.0, 600.0, 120.0];
    for (cyl, want) in boxer.cylinders.iter().zip(expected) {
        approx(cyl.firing_offset.to_degrees(), want, 1e-9);
    }

    // Bank 0 (1, 2, 3) and Bank 1 (4, 5, 6) each have three cylinders
    // firing at exact 240-degree intervals.
    for gap in boxer.bank_gaps(0) {
        approx(gap.to_degrees(), 240.0, 1e-9);
    }
    for gap in boxer.bank_gaps(1) {
        approx(gap.to_degrees(), 240.0, 1e-9);
    }

    // Alternating bank sequence: 0, 1, 0, 1, 0, 1.
    let bank_pattern: Vec<u8> = boxer
        .sequence
        .iter()
        .map(|&n| boxer.cylinders.iter().find(|c| c.number == n).unwrap().bank)
        .collect();
    assert_eq!(bank_pattern, vec![0, 1, 0, 1, 0, 1]);
}

#[test]
fn every_cylinder_lands_on_a_distinct_phase_cell() {
    let env = Environment::default();
    let block = EngineBlock::cross_plane_v8(env);
    let mut seen: Vec<usize> = (0..block.firing.len())
        .map(|i| block.phase_index_of(i))
        .collect();
    seen.sort_unstable();
    seen.dedup();
    assert_eq!(seen.len(), 8, "two cylinders collided on one phase cell");
    for idx in seen {
        assert!((1..=PHASE_CELLS).contains(&idx));
    }
}

/// Amplitude of a `2*pi`-periodic function's second harmonic, by direct
/// quadrature against `cos(2 theta)` and `sin(2 theta)` over one crank
/// revolution — the piston kinematics repeat every revolution, not every
/// 720-degree master cycle, so that is the period the harmonic lives in.
fn second_harmonic_amplitude(f: impl Fn(f64) -> f64) -> f64 {
    let steps = 3600;
    let dtheta = CYCLE_ANGLE / 2.0 / steps as f64;
    let (mut re, mut im) = (0.0, 0.0);
    for i in 0..steps {
        let theta = dtheta * i as f64;
        let v = f(theta);
        let (s, c) = (2.0 * theta).sin_cos();
        re += v * c;
        im += v * s;
    }
    (re * re + im * im).sqrt()
}

/// Second-harmonic magnitude of a firing order's shaking-force resultant,
/// combining both in-plane axes.
fn resultant_second_harmonic(order: &FiringOrder, geometry: &CylinderGeometry, omega: f64) -> f64 {
    let x = second_harmonic_amplitude(|theta| order.shaking_force(geometry, theta, omega).0);
    let y = second_harmonic_amplitude(|theta| order.shaking_force(geometry, theta, omega).1);
    (x * x + y * y).sqrt()
}

#[test]
fn inline_four_secondary_does_not_cancel_and_v8_layouts_differ() {
    // This is the test that proves the block layout is actually being
    // read: an inline-four has nothing to cancel against (one bank, every
    // cylinder's inertia force on the same axis), a crossplane V8's four
    // throws per bank already span a full quadrant and cancel on their
    // own, and a flatplane V8 shares the crossplane's bank angle and bank
    // split population but not its phase pairing, so it does not.
    let geometry = CylinderGeometry::default().with_reciprocating_mass(0.5);
    let omega = 500.0; // rad/s, shared so only layout differs

    let inline_four = resultant_second_harmonic(&FiringOrder::inline_four(), &geometry, omega);
    let cross_plane = resultant_second_harmonic(&FiringOrder::cross_plane_v8(), &geometry, omega);
    let flat_plane = resultant_second_harmonic(&FiringOrder::flat_plane_v8(), &geometry, omega);

    assert!(
        inline_four > 1.0,
        "an inline-four's secondary should not cancel: {inline_four}"
    );
    assert!(
        cross_plane < 0.01 * inline_four,
        "a crossplane V8's secondary should cancel against an inline-four's: \
         {cross_plane} vs {inline_four}"
    );
    assert!(
        (flat_plane - cross_plane).abs() > 0.1 * inline_four,
        "a flatplane and a crossplane V8 should differ in resultant: \
         {flat_plane} vs {cross_plane}"
    );
}

// -- friction -----------------------------------------------------------

/// The temperature the Chen-Flynn coefficients were measured at [K].
fn warm_oil() -> f64 {
    OilViscosity::default().reference_temperature
}

#[test]
fn chen_flynn_grows_with_load_and_speed() {
    let cf = ChenFlynn::default();
    let t = warm_oil();
    let idle = cf.fmep(20e5, 3.0, t);
    let loaded = cf.fmep(80e5, 3.0, t);
    let fast = cf.fmep(20e5, 15.0, t);
    assert!(loaded > idle, "peak pressure must raise FMEP");
    assert!(fast > idle, "piston speed must raise FMEP");
    // A warm V8 at 3000 rpm should land in the usual 0.5-2.5 bar band.
    let cruise = cf.fmep(60e5, 8.6, t);
    assert!(
        (0.5e5..2.5e5).contains(&cruise),
        "FMEP {cruise} Pa is outside the plausible band"
    );
}

#[test]
fn cold_oil_raises_fmep_and_warm_oil_leaves_it_alone() {
    let cf = ChenFlynn::default();
    let warm = cf.fmep(20e5, 3.0, warm_oil());
    let cold = cf.fmep(20e5, 3.0, 293.15);
    assert!(
        cold > warm,
        "thick oil must cost more: {cold} Pa cold against {warm} Pa warm"
    );
    // The shear terms roughly double; the constant and the ring load do not
    // move at all, so the whole FMEP goes up by something under a half.
    let rise = cold / warm - 1.0;
    assert!(
        (0.15..0.60).contains(&rise),
        "a cold idle's FMEP rose by {:.0} %, which is not what a cold engine does",
        rise * 100.0
    );
}

#[test]
fn friction_torque_matches_the_fmep_definition() {
    let cf = ChenFlynn::default();
    let displacement = 4.0e-3; // 4.0 litre
    let t = warm_oil();
    let fmep = cf.fmep(60e5, 8.6, t);
    approx(
        cf.torque(60e5, 8.6, displacement, t),
        fmep * displacement / (4.0 * PI),
        1e-9,
    );
}

// -- manifolds ----------------------------------------------------------

#[test]
fn plenum_conserves_mass_and_tracks_the_ideal_gas_law() {
    let gas = GasProperties::default();
    let mut p = Plenum::new(2e-3, 101_325.0, 300.0, gas.r_unburned, gas.gamma_unburned);
    approx(p.pressure(), 101_325.0, 1e-6);

    let m0 = p.mass;
    p.integrate(1e-3, 0.01, 300.0, 0.0);
    approx(p.mass, m0 + 1e-5, 1e-15);
    assert!(p.pressure() > 101_325.0, "filling must raise pressure");

    // Hot inflow must raise the plenum temperature.
    let t0 = p.temperature;
    p.integrate(1e-3, 0.05, 900.0, 0.0);
    assert!(p.temperature > t0);

    // Balanced flow at the plenum's own temperature is a no-op on mass.
    let m1 = p.mass;
    p.integrate(1e-3, 0.02, p.temperature, 0.02);
    approx(p.mass, m1, 1e-15);
}

#[test]
fn pipe_round_trip_matches_two_l_over_c() {
    let mut pipe = AcousticPipe::new(0.75, 1e-3, 16);
    let c = 550.0;
    approx(pipe.transit_time(c), 2.0 * 0.75 / c, 1e-15);
    approx(pipe.fundamental(c), c / (2.0 * 0.75), 1e-9);

    // A single-step impulse must come back inverted after one round trip.
    pipe.damping = 1.0;
    pipe.open_end_reflection = 1.0;
    pipe.closed_end_reflection = 0.0;
    let tau_seg = 0.75 / (16.0 * c);
    pipe.integrate(tau_seg, c, 1.0 / (c / pipe.area), 1.0); // unit source, one step
    let injected = pipe.forward[0];
    assert!(injected > 0.0);

    // Propagate the rest of the round trip with no further source.
    for _ in 0..(2 * 16 - 1) {
        pipe.integrate(tau_seg, c, 0.0, 1.0);
    }
    assert!(
        pipe.valve_end_perturbation() < 0.0,
        "open end must return an expansion wave, got {}",
        pipe.valve_end_perturbation()
    );
}

#[test]
fn pipe_stays_bounded_under_continuous_excitation() {
    let mut pipe = AcousticPipe::new(0.8, 1.2e-3, 24);
    for _ in 0..20_000 {
        pipe.integrate(1e-5, 560.0, 0.08, 1.0);
        assert!(pipe.valve_end_perturbation().is_finite());
    }
    assert!(
        pipe.valve_end_perturbation().abs() < 5e5,
        "wave field ran away: {}",
        pipe.valve_end_perturbation()
    );
}

#[test]
fn transit_rule_engages_on_resonance_and_releases_off_it() {
    let rule = ManifoldTransit::default();
    let pipe = AcousticPipe::new(0.75, 1e-3, 24);
    let c = 560.0;
    let interval = deg(180.0);

    // tau_pulse = 2*0.75/560 = 2.679 ms. Find the rpm where the 180-degree
    // bank interval is exactly two round trips.
    let tau = pipe.transit_time(c);
    // T_pulse = 180/(6 rpm) = 30/rpm, so rpm = 30/(k tau).
    let rpm_resonant = 30.0 / (2.0 * tau);
    let ratio = rule.tuning_ratio(interval, rpm_resonant, &pipe, c);
    approx(ratio, 2.0, 1e-9);
    approx(rule.detuning(ratio), 0.0, 1e-9);
    assert_eq!(
        rule.evaluate(ManifoldMode::Plenum, ratio),
        ManifoldMode::Acoustic
    );

    // Halfway between two harmonics is maximally detuned: stay lumped.
    let off = rule.tuning_ratio(interval, 30.0 / (2.5 * tau), &pipe, c);
    approx(off, 2.5, 1e-9);
    assert_eq!(
        rule.evaluate(ManifoldMode::Plenum, off),
        ManifoldMode::Plenum
    );
    assert_eq!(
        rule.evaluate(ManifoldMode::Acoustic, off),
        ManifoldMode::Plenum
    );
}

#[test]
fn transit_rule_has_hysteresis_so_it_cannot_chatter() {
    let rule = ManifoldTransit::default();
    // A ratio in the dead band between the two tolerances holds whichever
    // mode is already running.
    let between = 2.0 + 0.5 * (rule.engage_tolerance + rule.release_tolerance);
    assert!(between > rule.engage_tolerance + 2.0);
    assert!(between - 2.0 <= rule.release_tolerance);
    assert_eq!(
        rule.evaluate(ManifoldMode::Plenum, between),
        ManifoldMode::Plenum
    );
    assert_eq!(
        rule.evaluate(ManifoldMode::Acoustic, between),
        ManifoldMode::Acoustic
    );
}

#[test]
fn detuning_rejects_garbage_ratios() {
    let rule = ManifoldTransit::default();
    assert!(rule.detuning(f64::NAN).is_infinite());
    assert!(rule.detuning(0.0).is_infinite());
    assert!(rule.detuning(-3.0).is_infinite());
    // Beyond the highest harmonic the rule stops finding resonances.
    assert!(rule.detuning(50.0) > rule.engage_tolerance);
}

// -- the block ----------------------------------------------------------

#[test]
fn v8_runs_a_bounded_cycle_and_produces_positive_brake_torque() {
    let env = Environment::default();
    let mut block = EngineBlock::cross_plane_v8(env);
    let rpm = 3000.0;

    // Twelve cycles at 3000 rpm to settle the trapped mass and manifolds.
    let cycle_seconds = 120.0 / rpm;
    let frame = 1.0 / 600.0;
    let frames = (12.0 * cycle_seconds / frame) as usize;

    let mut last = block.update(frame, rpm);
    for _ in 1..frames {
        last = block.update(frame, rpm);
        assert!(last.indicated_torque.is_finite());
        assert!(last.peak_pressure.is_finite());
    }

    assert!(block.ring.is_primed(), "ring never filled");
    assert!(
        last.brake_torque > 0.0,
        "V8 made no net torque: {} N m",
        last.brake_torque
    );
    assert!(
        (20e5..200e5).contains(&last.peak_pressure),
        "peak pressure {} Pa is implausible",
        last.peak_pressure
    );
    assert!(
        (2e5..20e5).contains(&last.imep),
        "IMEP {} Pa is implausible",
        last.imep
    );
    assert_eq!(last.cylinder_pressures.len(), 8);
    assert_eq!(
        block.solver.watchdog.resets, 0,
        "watchdog should never fire"
    );
}

#[test]
fn a_lag_spike_does_not_destabilise_the_block() {
    let env = Environment::default();
    let mut block = EngineBlock::cross_plane_v8(env);
    for _ in 0..600 {
        block.update(1.0 / 600.0, 3000.0);
    }
    let healthy = block.update(1.0 / 600.0, 3000.0).peak_pressure;

    // A two-second stall, then business as usual.
    let spike = block.update(2.0, 3000.0);
    assert!(spike.step.plan.clamped);
    approx(spike.step.plan.dt, 1.0 / 30.0, 1e-15);
    assert!(spike.step.plan.substeps <= block.solver.budget.max_substeps);

    for _ in 0..600 {
        let out = block.update(1.0 / 600.0, 3000.0);
        assert!(out.brake_torque.is_finite());
    }
    let recovered = block.update(1.0 / 600.0, 3000.0).peak_pressure;
    assert!(
        (recovered - healthy).abs() < healthy * 0.5,
        "block did not recover from the lag spike: {healthy} -> {recovered}"
    );
}

#[test]
fn torque_sums_over_all_eight_cylinders() {
    let env = Environment::default();
    let mut block = EngineBlock::cross_plane_v8(env);
    for _ in 0..2000 {
        block.update(1.0 / 600.0, 3000.0);
    }
    let out = block.update(1.0 / 600.0, 3000.0);
    let summed: f64 = out.cylinder_torques.iter().sum();
    approx(out.indicated_torque, summed, 1e-9);
    approx(
        out.brake_torque,
        out.indicated_torque - out.friction_torque,
        1e-9,
    );
}

/// A cross-plane V8's torque pulses must be evenly spaced overall (90
/// degrees), even though each bank is uneven — that is the crank's whole job.
#[test]
fn firing_pulses_are_evenly_spaced_across_the_whole_engine() {
    let order = FiringOrder::cross_plane_v8();
    let mut offsets: Vec<f64> = order
        .cylinders
        .iter()
        .map(|c| c.firing_offset.to_degrees())
        .collect();
    offsets.sort_by(|a, b| a.partial_cmp(b).unwrap());
    for (i, off) in offsets.iter().enumerate() {
        approx(*off, i as f64 * 90.0, 1e-9);
    }
}

/// Stage M5 changes the two-rotor's ports and its mechanical voices, not
/// its firing order: four events across 720 degrees of eccentric shaft,
/// evenly spaced, is the correct order 2 and must stay exactly that.
#[test]
fn two_rotor_wankel_still_fires_four_times_across_the_shaft() {
    let order = FiringOrder::two_rotor_wankel();
    assert_eq!(order.cylinders.len(), 4);
    let mut offsets: Vec<f64> = order
        .cylinders
        .iter()
        .map(|c| c.firing_offset.to_degrees())
        .collect();
    offsets.sort_by(|a, b| a.partial_cmp(b).unwrap());
    for (i, off) in offsets.iter().enumerate() {
        approx(*off, i as f64 * 180.0, 1e-9);
    }
}

#[test]
fn block_exposes_per_cylinder_evo_pressure() {
    let env = Environment::default();
    let mut block = EngineBlock::cross_plane_v8(env);
    let evo = block.model.valves.exhaust.open_angle;
    block.ring.record(
        evo,
        PhaseSample {
            pressure: 3.8e5,
            ..PhaseSample::default()
        },
    );

    // Every cylinder defaults to reading the master trace at EVO.
    for i in 0..8 {
        approx(block.cylinder_evo_pressure(i), 3.8e5, 1e-6);
        approx(block.evo_pressure_of(i), 3.8e5, 1e-6);
    }
    let pressures = block.evo_pressures();
    assert_eq!(pressures.len(), 8);
    assert!(pressures.iter().all(|&p| (p - 3.8e5).abs() < 1e-6));

    // Setting an override changes that cylinder's EVO pressure.
    block.set_cylinder_evo_pressure(1, 2.5e5);
    approx(block.cylinder_evo_pressure(0), 3.8e5, 1e-6);
    approx(block.cylinder_evo_pressure(1), 2.5e5, 1e-6);
    assert_eq!(block.evo_pressures()[1], 2.5e5);
}

// -- thermal state ------------------------------------------------------

/// The solver clamps a frame to `1/30 s` and ages the thermal state on the
/// same clock, so every warm-up here is stepped inside that.
const THERMAL_DT: f64 = 1.0 / 120.0;

/// A cold cross-plane V8, ready to be warmed up.
fn cold_v8() -> EngineBlock {
    let mut block = EngineBlock::cross_plane_v8(Environment::default());
    block.cold_start();
    block
}

/// Runs `block` at a fixed speed, reading `probe` every `interval` seconds.
fn warm_up(
    block: &mut EngineBlock,
    rpm: f64,
    interval: f64,
    samples: usize,
    probe: impl Fn(&EngineBlock) -> f64,
) -> Vec<f64> {
    let steps = (interval / THERMAL_DT).round() as usize;
    let mut trace = Vec::with_capacity(samples + 1);
    trace.push(probe(block));
    for _ in 0..samples {
        for _ in 0..steps {
            block.update(THERMAL_DT, rpm);
        }
        trace.push(probe(block));
    }
    trace
}

fn assert_rising(trace: &[f64], what: &str) {
    for pair in trace.windows(2) {
        assert!(
            pair[1] > pair[0],
            "{what} fell from {:.2} to {:.2} during warm-up: {trace:.1?}",
            pair[0],
            pair[1]
        );
    }
}

#[test]
fn from_cold_every_pipe_warms_monotonically_to_a_plateau() {
    let mut block = cold_v8();
    let ambient = block.environment.temperature;
    let trace = warm_up(&mut block, 3_000.0, 10.0, 20, |b| {
        b.thermal.exhaust.primaries[0].wall.temperature
    });

    assert!(
        (trace[0] - ambient).abs() < 1e-9,
        "a cold start must begin at ambient, not {:.1} K",
        trace[0]
    );
    assert_rising(&trace, "the primary wall");

    // A plateau, not a ramp that ran out of test: the last ten seconds have
    // to move the wall by a small fraction of what the first ten did.
    let first = trace[1] - trace[0];
    let last = trace[trace.len() - 1] - trace[trace.len() - 2];
    assert!(
        last < 0.05 * first,
        "still climbing at {last:.1} K per ten seconds against {first:.1} K at the start"
    );
    // And it plateaued below the gas driving it, which is the only place a
    // wall heated by that gas can settle.
    assert!(
        *trace.last().unwrap() < block.thermal.exhaust.primary_gas(0),
        "the wall passed the gas heating it"
    );
}

#[test]
fn the_exhaust_keeps_a_gradient_down_its_length() {
    let mut block = cold_v8();
    for _ in 0..(120.0 / THERMAL_DT) as usize {
        block.update(THERMAL_DT, 3_000.0);
    }
    let port = block.exhaust_banks[0].plenum.temperature;
    let primary = block.thermal.exhaust.primary_gas(0);
    let tailpipe = block.thermal.exhaust.tailpipe_gas();
    assert!(
        port > primary && primary > tailpipe,
        "no gradient: port {port:.0} K, primary {primary:.0} K, tailpipe {tailpipe:.0} K"
    );
}

#[test]
fn a_stopped_hot_engine_cools_at_the_modelled_time_constant() {
    let mut block = EngineBlock::cross_plane_v8(Environment::default());
    let ambient = block.environment.temperature;
    // Started just under the thermostat's rating, where the valve is shut
    // and the only path out is the bypass — so the conductance, and with it
    // the time constant, is a constant over the whole decay.
    let start = block.thermal.thermostat.open_temperature - 6.0;
    block.thermal.block.temperature = start;
    block.thermal.block.conductance = block.thermal.thermostat.conductance(start);
    let tau = block.thermal.block.time_constant();
    assert!(
        (600.0..3_600.0).contains(&tau),
        "an engine cooling in still air takes {tau:.0} s, which is not an engine"
    );

    for _ in 0..(tau / THERMAL_DT) as usize {
        block.update(THERMAL_DT, 0.0);
    }

    // One time constant leaves `1/e` of the excess over ambient.
    let ratio = (block.thermal.block_temperature() - ambient) / (start - ambient);
    approx(ratio, 1.0 / std::f64::consts::E, 1e-3);
}

#[test]
fn fmep_falls_monotonically_as_the_block_warms() {
    let mut block = cold_v8();
    let open = block.thermal.thermostat.open_temperature;
    let oil = warm_up(&mut block, 2_000.0, 5.0, 16, |b| {
        b.thermal.oil_temperature()
    });

    // Read at one fixed load and speed throughout, so what moves is the oil
    // and nothing else: the claim is about viscosity, not about the engine
    // making a different peak pressure when it is cold.
    let fmep: Vec<f64> = oil
        .iter()
        .map(|&t| block.friction.fmep(60e5, 8.6, t))
        .collect();

    // Only while the engine is actually warming. Once the thermostat has it
    // the temperature is flat to the last bit, and so is the friction; a
    // strict inequality there would be asserting on rounding.
    let warming = oil.iter().take_while(|&&t| t < open).count();
    assert!(
        warming > 4,
        "the block reached its thermostat too fast to test"
    );
    assert_rising(&oil[..warming], "the oil temperature");
    // Settled means flat, not "close to where it crossed the threshold":
    // comparing a single sample right at the crossing against the last one
    // is sensitive to exactly which frame the trace happened to sample it
    // on. The tail spread is not.
    let tail = &oil[oil.len() - 4..];
    let tail_spread = tail.iter().cloned().fold(f64::MIN, f64::max)
        - tail.iter().cloned().fold(f64::MAX, f64::min);
    assert!(
        tail_spread < oil[1] - oil[0],
        "the block never settled on its thermostat: {oil:.1?}"
    );
    for pair in fmep[..warming].windows(2) {
        assert!(
            pair[1] < pair[0],
            "FMEP rose from {:.0} to {:.0} Pa while the block was warming",
            pair[0],
            pair[1]
        );
    }
    assert!(
        fmep[0] > 1.15 * fmep[warming - 1],
        "a cold engine's FMEP is {:.0} Pa against {:.0} Pa warm, which is no change at all",
        fmep[0],
        fmep[warming - 1]
    );
}

#[test]
fn dead_cylinder_zeroes_blowdown_and_reduces_indicated_torque() {
    let mut block = EngineBlock::cross_plane_v8(Environment::default());
    for _ in 0..600 {
        block.update(1.0 / 240.0, 3_000.0);
    }
    let healthy_evo = block.cylinder_evo_pressure(1);
    assert!(healthy_evo > block.environment.pressure * 1.5);

    block.set_cylinder_health(1, CylinderHealth::dead_plug());
    let dead_evo = block.cylinder_evo_pressure(1);
    let bank_idx = (block.firing.cylinders[1].bank as usize) % block.exhaust_banks.len().max(1);
    let manifold_p = block.exhaust_banks[bank_idx].port_pressure();
    assert!(
        (dead_evo - manifold_p).abs() < 1e-3,
        "dead cylinder EVO pressure ({dead_evo:.1}) must equal manifold ({manifold_p:.1})"
    );
}

#[test]
fn lpp_and_volumetric_efficiency_diagnostics() {
    let mut block = EngineBlock::cross_plane_v8(Environment::default());
    for _ in 0..600 {
        block.update(1.0 / 240.0, 3_000.0);
    }

    let lpp = block.lpp_deg_atdc();
    assert!(
        (5.0..36.0).contains(&lpp),
        "LPP must sit safely after compression TDC (360 deg) in expansion: got {lpp:.1} deg ATDC"
    );

    let eta_v = block.volumetric_efficiency();
    assert!(
        (0.5..1.5).contains(&eta_v),
        "naturally aspirated volumetric efficiency must be plausible: got {eta_v:.2}"
    );
}

#[test]
fn load_fraction_coincides_with_volumetric_efficiency_na() {
    // Both ask "how much of a full atmospheric charge did the cylinder
    // trap"; for a naturally aspirated engine within load_fraction's
    // tighter clamp they must agree exactly.
    let mut block = EngineBlock::cross_plane_v8(Environment::default());
    for _ in 0..600 {
        block.update(1.0 / 240.0, 3_000.0);
    }
    assert_eq!(block.load_fraction(), block.volumetric_efficiency());
}

#[test]
fn wide_open_throttle_leaves_intake_flow_ungated() {
    let mut block = EngineBlock::cross_plane_v8(Environment::default());
    block.throttle = 1.0;
    // A real deficit below ambient for the gate to have something to restrict.
    block.intake.mass *= 0.9;

    let upstream = IntakePlenum::ambient_upstream(&block.environment, &block.model.gas);
    let wide_open = block.intake.throttle_flow(1.0, &upstream);
    let half_open = block.intake.throttle_flow(0.5, &upstream);

    // Throttle = 1.0 must be exactly the unrestricted orifice, not
    // something a bypass or a leak term has widened or narrowed further.
    assert_eq!(
        wide_open,
        block
            .intake
            .throttle
            .mass_flow(1.0, &upstream, &block.intake.port_state()),
        "throttle = 1.0 must not gate flow through any path but the plate itself"
    );
    assert!(
        wide_open > half_open,
        "wide open must pass more air than half open"
    );
}

#[test]
fn closing_the_throttle_reduces_trapped_mass_and_load_fraction() {
    let rpm = 3_000.0;
    let frame = 1.0 / 240.0;
    let frames = 600;

    let mut open = EngineBlock::cross_plane_v8(Environment::default());
    open.throttle = 1.0;
    for _ in 0..frames {
        open.update(frame, rpm);
    }

    let mut closed = EngineBlock::cross_plane_v8(Environment::default());
    closed.throttle = 0.25;
    for _ in 0..frames {
        closed.update(frame, rpm);
    }

    assert!(
        closed.load_fraction() < open.load_fraction(),
        "a more closed throttle must trap less: closed={} open={}",
        closed.load_fraction(),
        open.load_fraction()
    );
}

/// Settles a fresh V8 at `rpm` on a fixed throttle for long enough for
/// trapped mass, exhaust temperature and the phase ring to stop moving.
fn settled_at_throttle(rpm: f64, throttle: f64) -> EngineBlock {
    let mut block = EngineBlock::cross_plane_v8(Environment::default());
    block.throttle = throttle;
    for _ in 0..600 {
        block.update(1.0 / 240.0, rpm);
    }
    block
}

#[test]
fn a_gear_change_that_raises_required_torque_raises_load_fraction() {
    // `RoadLoad`/`Gearbox` already prove (see `physics::vehicle::tests`)
    // that two gears at the same rpm imply different required crank
    // torque; a driver meets more required torque with more pedal. This
    // is the other half: more pedal really does trap more mass, so the
    // schedules downstream see a different load, not the same one.
    let light = settled_at_throttle(3_000.0, 0.08);
    let heavy = settled_at_throttle(3_000.0, 0.15);
    assert!(
        heavy.load_fraction() > light.load_fraction(),
        "more pedal must trap more mass: light={:.3} heavy={:.3}",
        light.load_fraction(),
        heavy.load_fraction()
    );
}

#[test]
fn grade_raises_load_fraction_and_enriches_afr() {
    use crate::physics::vehicle::RoadLoad;

    // Climbing a grade at the same road speed raises the torque the
    // engine must produce — pure algebra, no engine involved yet.
    let level = RoadLoad::generic_road_car();
    let uphill = RoadLoad {
        grade: 0.08, // steep, so the effect is unmistakable
        ..RoadLoad::generic_road_car()
    };
    let speed_mps = 25.0;
    let overall_ratio = 4.0;
    let air_density = 1.2041;
    assert!(
        uphill.crank_torque(speed_mps, air_density, overall_ratio)
            > level.crank_torque(speed_mps, air_density, overall_ratio),
        "a grade must raise the torque required to hold the same speed"
    );

    // Meeting that extra torque takes more pedal, and more pedal really
    // does raise load fraction — leaving the lean-cruise band the ECU
    // uses to save fuel at light, steady load, and enriching toward
    // stoichiometric the way a real ECU does once cruise gives way to
    // sustained pull.
    let cruising = settled_at_throttle(3_000.0, 0.2);
    let climbing = settled_at_throttle(3_000.0, 0.4);
    let cruising_load = cruising.load_fraction();
    let climbing_load = climbing.load_fraction();
    assert!(
        climbing_load > cruising_load,
        "more load must follow more pedal: cruising={cruising_load:.3} climbing={climbing_load:.3}"
    );

    let cruising_afr = cruising.ecu.target_afr(cruising_load, 3_000.0, 0.02);
    let climbing_afr = climbing.ecu.target_afr(climbing_load, 3_000.0, 0.1);
    assert!(
        climbing_afr < cruising_afr,
        "the AFR schedule must enrich (lower number) under more load: \
         cruising={cruising_afr:.2} climbing={climbing_afr:.2}"
    );
}

#[test]
fn higher_load_raises_exhaust_temperature_and_moves_primary_tuning() {
    // Both throttles are open enough to stay past the deep-vacuum regime
    // where overlap reversion dominates the intake charge (see
    // `TURBO_PLAN.md` TB3): under that regime a hotter, more diluted
    // reversion charge at light throttle can legitimately run hotter than
    // a cleaner, less diluted charge at a bit more throttle, which is a
    // real effect and not what this test means by "load".
    let light = settled_at_throttle(3_000.0, 0.3);
    let heavy = settled_at_throttle(3_000.0, 0.5);
    assert!(heavy.load_fraction() > light.load_fraction());

    let light_temp = light.exhaust_banks[0].plenum.temperature;
    let heavy_temp = heavy.exhaust_banks[0].plenum.temperature;
    assert!(
        heavy_temp > light_temp,
        "higher load must run a hotter exhaust: light={light_temp:.1} heavy={heavy_temp:.1}"
    );

    let light_tuning = light.exhaust_banks[0].tuning_ratio;
    let heavy_tuning = heavy.exhaust_banks[0].tuning_ratio;
    assert!(
        (heavy_tuning - light_tuning).abs() > 1e-3,
        "the primaries' tuning ratio must move with exhaust temperature: \
         light={light_tuning:.4} heavy={heavy_tuning:.4}"
    );
}

#[test]
fn two_gears_at_the_same_rpm_schedule_different_spark_advance() {
    use crate::physics::vehicle::{Gear, Gearbox, RoadLoad};

    // The plan's own example: cruising in sixth against pulling in
    // second, both at the same rpm — the gears alone already put a
    // different torque demand on the crank.
    let mut gearbox = Gearbox::generic_six_speed();
    gearbox.gear = Gear::Engaged(2);
    let low_gear_ratio = gearbox.overall_ratio().unwrap();
    gearbox.gear = Gear::Engaged(6);
    let high_gear_ratio = gearbox.overall_ratio().unwrap();

    let road = RoadLoad::generic_road_car();
    let speed_mps = 25.0;
    let air_density = 1.2041;
    assert!(
        road.crank_torque(speed_mps, air_density, low_gear_ratio)
            != road.crank_torque(speed_mps, air_density, high_gear_ratio),
        "second and sixth must not ask the crank for the same torque"
    );

    // That difference in demanded torque is met with a different pedal
    // position, which traps a different mass at the same rpm.
    let rpm = 3_000.0;
    let pulling = settled_at_throttle(rpm, 0.15);
    let cruising = settled_at_throttle(rpm, 0.08);
    let pulling_load = pulling.load_fraction();
    let cruising_load = cruising.load_fraction();
    assert!(
        (pulling_load - cruising_load).abs() > 1e-3,
        "the same rpm in two gears must trap different mass: \
         2nd={pulling_load:.3} 6th={cruising_load:.3}"
    );

    let pulling_spark = pulling.ecu.schedule_spark_advance(pulling_load, rpm);
    let cruising_spark = cruising.ecu.schedule_spark_advance(cruising_load, rpm);
    assert!(
        (pulling_spark - cruising_spark).abs() > 1e-3,
        "different load fractions at the same rpm must schedule different \
         spark advance: 2nd={pulling_spark:.2} 6th={cruising_spark:.2}"
    );
}

// -- TB3: close the loop -------------------------------------------------

fn test_turbo_hardware(env: &Environment) -> crate::physics::intake::ForcedInduction {
    use crate::physics::compressor::{CompressorMap, FrameSize};
    use crate::physics::turbine::{BearingType, TurbineMap, TurbineMapPoint, TurbineSpeedLine};

    let point = |expansion_ratio: f64, reduced_flow: f64, efficiency: f64| TurbineMapPoint {
        expansion_ratio,
        reduced_flow,
        efficiency,
    };
    let turbine_map = TurbineMap::new(vec![
        TurbineSpeedLine::new(
            60_000.0,
            vec![
                point(1.0, 0.05, 0.50),
                point(1.5, 0.14, 0.68),
                point(2.2, 0.22, 0.60),
            ],
        ),
        TurbineSpeedLine::new(
            160_000.0,
            vec![
                point(1.0, 0.08, 0.55),
                point(2.0, 0.28, 0.74),
                point(3.2, 0.42, 0.62),
            ],
        ),
    ]);

    crate::physics::intake::ForcedInduction::new(
        CompressorMap::stock(FrameSize::Medium),
        turbine_map,
        8.0e-5,
        0.97,
        BearingType::BallBearing,
        2.0e-3,
        None,
        env,
        &GasProperties::default(),
    )
}

fn settled_v8(rpm: f64, throttle: f64, forced: bool) -> EngineBlock {
    let env = Environment::default();
    let mut block = EngineBlock::cross_plane_v8(env);
    block.throttle = throttle;
    if forced {
        block.fit_forced_induction(test_turbo_hardware(&env), vec![0, 1], false);
    }
    for _ in 0..3_000 {
        block.update(1.0 / 480.0, rpm);
    }
    block
}

#[test]
fn an_atmospheric_preset_is_unchanged_by_the_forced_induction_field() {
    // `forced_induction: None` must be a complete no-op: an engine that
    // never sets it breathes exactly as it did before TB3 existed.
    let env = Environment::default();
    let mut with_none = EngineBlock::cross_plane_v8(env);
    let mut untouched = EngineBlock::cross_plane_v8(env);
    with_none.throttle = 1.0;
    untouched.throttle = 1.0;
    for _ in 0..500 {
        with_none.update(1.0 / 480.0, 4_000.0);
        untouched.update(1.0 / 480.0, 4_000.0);
    }
    assert_eq!(with_none.forced_induction.is_empty(), true);
    approx(
        with_none.intake.pressure(),
        untouched.intake.pressure(),
        1e-9,
    );
    approx(
        with_none.exhaust_banks[0].plenum.pressure(),
        untouched.exhaust_banks[0].plenum.pressure(),
        1e-9,
    );
}

#[test]
fn boost_raises_trapped_mass_and_exhaust_back_pressure() {
    let na = settled_v8(4_000.0, 1.0, false);
    let boosted = settled_v8(4_000.0, 1.0, true);

    let shaft_rpm = boosted
        .forced_induction
        .first()
        .expect("forced induction fitted")
        .hardware
        .shaft
        .shaft_rpm();
    assert!(
        shaft_rpm > 5_000.0,
        "turbo failed to spool: {shaft_rpm} rpm"
    );

    assert!(
        boosted.intake.pressure() > na.intake.pressure() * 1.1,
        "a spooled turbo must trap more mass than atmospheric: na={:.0} Pa, boosted={:.0} Pa",
        na.intake.pressure(),
        boosted.intake.pressure()
    );

    let na_back_pressure = na.exhaust_banks[0].plenum.pressure();
    let boosted_back_pressure = boosted.exhaust_banks[0].plenum.pressure();
    assert!(
        boosted_back_pressure > na_back_pressure * 1.05,
        "a wheel in the exhaust must raise back pressure: na={na_back_pressure:.0} Pa, \
         boosted={boosted_back_pressure:.0} Pa"
    );

    let na_torque = na.instantaneous_indicated_torque();
    let boosted_torque = boosted.instantaneous_indicated_torque();
    assert!(
        boosted_torque > na_torque,
        "more trapped mass must make more torque: na={na_torque:.1}, boosted={boosted_torque:.1}"
    );
}

#[test]
fn shutting_the_throttle_at_boost_moves_the_charge_pipe_off_its_wot_pressure() {
    // With no blow-off valve fitted (that is TB4), a lifted throttle gives
    // the compressor nowhere to send its flow, so the charge pipe moves
    // *toward* the compressor's surge boundary rather than venting back to
    // ambient — this is the real reason a lift makes a stock turbo car
    // want a blow-off valve at all. What TB3 owns is that the pipe's
    // capacitance actually responds to the lift instead of sitting inert;
    // making that response collapse smoothly once a real vent exists is
    // TB4's job.
    let mut block = settled_v8(4_000.0, 1.0, true);
    let pressurized = block.forced_induction[0].hardware.charge_pipe.pressure();
    assert!(
        pressurized > block.environment.pressure * 1.1,
        "the rig must actually be boosted before the lift: {pressurized:.0} Pa"
    );

    block.throttle = 0.0;
    for _ in 0..500 {
        block.update(1.0 / 480.0, 4_000.0);
    }
    let after_lift = block.forced_induction[0].hardware.charge_pipe.pressure();
    assert!(
        (after_lift - pressurized).abs() > pressurized * 0.1,
        "a shut throttle must move the charge pipe well off its WOT \
         pressure, whichever direction an unvented compressor takes it: \
         was {pressurized:.0} Pa, now {after_lift:.0} Pa"
    );
}

#[test]
fn sustained_high_boost_produces_knock_without_any_knock_specific_change() {
    let na = settled_v8(5_500.0, 1.0, false);
    let boosted = settled_v8(5_500.0, 1.0, true);
    assert!(
        boosted.master.knock_integral > na.master.knock_integral,
        "a boosted engine at the same load must show more knock tendency \
         than atmospheric with no change to the knock model itself: \
         na={:.3}, boosted={:.3}",
        na.master.knock_integral,
        boosted.master.knock_integral
    );
}

// -- TB4: boost control and the valves -----------------------------------

fn test_turbo_hardware_with_wastegate(
    env: &Environment,
    target_pressure_ratio: f64,
    gain: f64,
    max_flow_area: f64,
) -> crate::physics::intake::ForcedInduction {
    use crate::physics::turbine::{BoostController, Wastegate, WastegateFitment};

    let wastegate = Wastegate {
        fitment: WastegateFitment::Internal,
        max_flow_area,
        spring_preload: 0.4e5,
        opening_span: 0.3e5,
    };
    let controller = BoostController::new(target_pressure_ratio, gain, 0.6e5, 0.15);
    test_turbo_hardware(env)
        .with_wastegate(wastegate)
        .with_boost_controller(controller)
}

fn charge_pipe_pressure_ratio(block: &EngineBlock, env: &Environment) -> f64 {
    block
        .forced_induction
        .first()
        .expect("forced induction fitted")
        .hardware
        .charge_pipe
        .pressure()
        / env.pressure
}

/// Counts how many times a trace changes direction by more than
/// `deadband`, i.e. how many local extrema it has past numerical noise —
/// zero or one is a settling trace, several is an oscillation.
fn direction_changes(trace: &[f64], deadband: f64) -> usize {
    let mut changes = 0;
    let mut anchor = trace[0];
    let mut rising: Option<bool> = None;
    for &v in &trace[1..] {
        let delta = v - anchor;
        if delta.abs() < deadband {
            continue;
        }
        let now_rising = delta > 0.0;
        if let Some(previous) = rising {
            if now_rising != previous {
                changes += 1;
            }
        }
        rising = Some(now_rising);
        anchor = v;
    }
    changes
}

#[test]
fn a_boost_controller_holds_its_target_pressure_ratio() {
    let env = Environment::default();
    let target = 1.6;
    let mut block = EngineBlock::cross_plane_v8(env);
    block.throttle = 1.0;
    block.fit_forced_induction(
        test_turbo_hardware_with_wastegate(&env, target, 3.0e5, 8.0e-4),
        vec![0, 1],
        false,
    );
    for _ in 0..4_000 {
        block.update(1.0 / 480.0, 5_000.0);
    }
    let pr = charge_pipe_pressure_ratio(&block, &env);
    assert!(
        (pr - target).abs() < target * 0.2,
        "a closed loop wastegate should hold near its target pressure ratio: \
         target={target}, got={pr:.2}"
    );
}

#[test]
fn an_undersized_gate_creeps_past_target_at_high_rpm() {
    let env = Environment::default();
    let target = 1.6;
    let mut adequate = EngineBlock::cross_plane_v8(env);
    adequate.throttle = 1.0;
    adequate.fit_forced_induction(
        test_turbo_hardware_with_wastegate(&env, target, 3.0e5, 8.0e-4),
        vec![0, 1],
        false,
    );

    let mut undersized = EngineBlock::cross_plane_v8(env);
    undersized.throttle = 1.0;
    undersized.fit_forced_induction(
        test_turbo_hardware_with_wastegate(&env, target, 3.0e5, 0.01e-4),
        vec![0, 1],
        false,
    );

    for _ in 0..4_000 {
        adequate.update(1.0 / 480.0, 6_500.0);
        undersized.update(1.0 / 480.0, 6_500.0);
    }

    let pr_adequate = charge_pipe_pressure_ratio(&adequate, &env);
    let pr_undersized = charge_pipe_pressure_ratio(&undersized, &env);
    assert!(
        pr_undersized > pr_adequate * 1.01,
        "an undersized gate must creep past target more than an adequate \
         one at the same rpm: adequate={pr_adequate:.2}, undersized={pr_undersized:.2}"
    );
}

#[test]
fn a_fast_tip_in_transient_is_sensitive_to_controller_gain() {
    // A wastegate can only ever bypass *more* flow to relieve pressure,
    // never less than a bare spring would — so unlike a heater/cooler
    // pair this loop cannot ring past target and back, and higher gain
    // reins in the tip-in transient tighter rather than growing its
    // overshoot the way a bidirectional actuator's would. What TB4 does
    // own is that gain measurably changes that transient at all, which is
    // what a real, responding closed loop implies and a fixed-threshold
    // wastegate could not show.
    let env = Environment::default();
    let target = 1.35;
    let rpm = 4_200.0;

    let run = |gain: f64| -> f64 {
        let mut block = EngineBlock::cross_plane_v8(env);
        block.throttle = 0.2;
        block.fit_forced_induction(
            test_turbo_hardware_with_wastegate(&env, target, gain, 10.0e-4),
            vec![0, 1],
            false,
        );
        for _ in 0..1_500 {
            block.update(1.0 / 480.0, rpm);
        }
        block.throttle = 1.0;
        let mut previous = charge_pipe_pressure_ratio(&block, &env);
        let mut rising = true;
        for _ in 0..1_500 {
            block.update(1.0 / 480.0, rpm);
            let pr = charge_pipe_pressure_ratio(&block, &env);
            if rising && pr < previous {
                return previous;
            }
            rising = pr >= previous;
            previous = pr;
        }
        previous
    };

    let low_gain_peak = run(0.6e5);
    let high_gain_peak = run(3.0e5);
    assert!(
        (high_gain_peak - low_gain_peak).abs() > 0.05,
        "controller gain should measurably change the tip-in transient's \
         first peak: low={low_gain_peak:.2}, high={high_gain_peak:.2}"
    );
}

#[test]
fn no_blow_off_drives_the_compressor_into_a_surge_oscillation() {
    let mut block = settled_v8(4_000.0, 1.0, true);
    block.throttle = 0.0;
    let mut trace = Vec::with_capacity(3_000);
    for _ in 0..3_000 {
        block.update(1.0 / 480.0, 4_000.0);
        trace.push(block.forced_induction[0].hardware.charge_pipe.pressure());
    }
    let changes = direction_changes(&trace, 2_000.0);
    assert!(
        changes >= 2,
        "an unvented compressor past surge should cycle rather than settle \
         monotonically: {changes} direction changes"
    );
}

#[test]
fn a_recirculating_blow_off_valve_prevents_surge() {
    use crate::physics::intake::{BlowOffFitment, BlowOffValve};

    let env = Environment::default();
    let mut block = EngineBlock::cross_plane_v8(env);
    block.throttle = 1.0;
    let bov = BlowOffValve {
        fitment: BlowOffFitment::Recirculating,
        max_flow_area: 4.0e-4,
        spring_preload: 0.3e5,
        opening_span: 0.2e5,
    };
    block.fit_forced_induction(
        test_turbo_hardware(&env).with_blow_off(bov),
        vec![0, 1],
        false,
    );
    for _ in 0..3_000 {
        block.update(1.0 / 480.0, 4_000.0);
    }

    block.throttle = 0.0;
    let mut trace = Vec::with_capacity(3_000);
    for _ in 0..3_000 {
        block.update(1.0 / 480.0, 4_000.0);
        trace.push(block.forced_induction[0].hardware.charge_pipe.pressure());
    }
    let changes = direction_changes(&trace, 2_000.0);
    assert!(
        changes < 2,
        "a recirculating blow-off valve should settle the charge pipe \
         rather than let it surge: {changes} direction changes"
    );
}

// -- TB5: transients and architectures -----------------------------------

#[test]
fn twin_scroll_shrinks_the_bank_collector_to_the_divided_volume() {
    let env = Environment::default();
    let mut block = EngineBlock::new(CylinderModel::default(), FiringOrder::inline_four(), env);
    let full_volume = block.exhaust_banks[0].plenum.volume;

    let groups = block.firing.twin_scroll_groups(0);
    let scroll_cylinders = groups[0].len() as f64;
    let bank_cylinders = block.firing.cylinders_on_bank(0).len() as f64;
    let expected_divided_volume = full_volume * scroll_cylinders / bank_cylinders;

    block.fit_forced_induction(test_turbo_hardware(&env), vec![0], true);

    let divided_volume = block.exhaust_banks[0].plenum.volume;
    assert!(
        divided_volume < full_volume,
        "a twin-scroll collector must be smaller than the whole bank's own \
         collector: full={full_volume:.6} m^3, divided={divided_volume:.6} m^3"
    );
    approx(divided_volume, expected_divided_volume, 1e-9);
}

#[test]
fn a_sequential_secondary_brings_a_bounded_transient_with_no_discontinuity() {
    use crate::physics::compressor::{CompressorMap, FrameSize};
    use crate::physics::control::SequentialValve;
    use crate::physics::turbine::{BearingType, TurbineMap, TurbineMapPoint, TurbineSpeedLine};

    // A real sequential secondary is a genuinely smaller unit brought in
    // alongside a primary that already covers the bank on its own — not a
    // second, identically-sized turbo, which would ask a V8's two banks to
    // feed two full-size wheels at once and starve both.
    fn small_secondary_hardware(env: &Environment) -> crate::physics::intake::ForcedInduction {
        use crate::physics::turbine::{BoostController, Wastegate, WastegateFitment};

        let point = |expansion_ratio: f64, reduced_flow: f64, efficiency: f64| TurbineMapPoint {
            expansion_ratio,
            reduced_flow,
            efficiency,
        };
        let turbine_map = TurbineMap::new(vec![
            TurbineSpeedLine::new(
                60_000.0,
                vec![
                    point(1.0, 0.02, 0.48),
                    point(1.6, 0.06, 0.66),
                    point(2.4, 0.09, 0.56),
                ],
            ),
            TurbineSpeedLine::new(
                160_000.0,
                vec![
                    point(1.0, 0.03, 0.52),
                    point(2.0, 0.10, 0.72),
                    point(3.2, 0.15, 0.58),
                ],
            ),
        ]);
        crate::physics::intake::ForcedInduction::new(
            CompressorMap::stock(FrameSize::Small),
            turbine_map,
            3.0e-5,
            0.97,
            BearingType::BallBearing,
            1.0e-3,
            None,
            env,
            &GasProperties::default(),
        )
        .with_wastegate(Wastegate {
            fitment: WastegateFitment::Internal,
            max_flow_area: 4.0e-4,
            spring_preload: 0.4e5,
            opening_span: 0.3e5,
        })
        .with_boost_controller(BoostController::new(1.6, 3.0e5, 0.6e5, 0.15))
    }

    let env = Environment::default();
    let mut block = EngineBlock::cross_plane_v8(env);
    block.throttle = 1.0;
    block.fit_forced_induction(
        test_turbo_hardware_with_wastegate(&env, 1.6, 3.0e5, 8.0e-4),
        vec![0, 1],
        false,
    );
    block.fit_forced_induction(small_secondary_hardware(&env), vec![0, 1], false);
    block.forced_induction[1].activation = Some(SequentialValve::new(4_200.0, 3_800.0, 0.15));

    let dt = 1.0 / 480.0;
    for _ in 0..3_000 {
        block.update(dt, 3_000.0);
    }
    assert_eq!(
        block.forced_induction[1]
            .activation
            .as_ref()
            .unwrap()
            .fraction(),
        0.0,
        "valve should still be shut below its open threshold"
    );

    // Step rpm straight across the valve's open threshold in one frame — the
    // sharpest possible signal — and check the valve's own realized effect
    // immediately after, rather than downstream aggregates like manifold
    // pressure or brake torque: both have their own large frame-to-frame
    // swings in a free-running multi-turbo engine (combustion events, an
    // unregulated wheel's own surge) entirely unrelated to this valve, which
    // would swamp the much smaller step the valve itself could ever cause.
    // A real actuator cannot jump straight to fully open in one small step,
    // so even a step input to the signal it watches should still leave the
    // valve mostly shut on the very next frame.
    block.update(dt, 4_500.0);
    let fraction_after_one_frame = block.forced_induction[1]
        .activation
        .as_ref()
        .unwrap()
        .fraction();
    assert!(
        fraction_after_one_frame > 0.0 && fraction_after_one_frame < 0.1,
        "the valve should open gradually even after a step past its \
         threshold, not snap open in a single frame: {fraction_after_one_frame}"
    );

    // Hold well past the threshold for long enough to fully open and for
    // the secondary to actually spool up — the changeover is bounded, not
    // permanently blocked.
    for _ in 0..3_000 {
        block.update(dt, 5_000.0);
    }
    assert!(
        block.forced_induction[1]
            .activation
            .as_ref()
            .unwrap()
            .fraction()
            > 0.99,
        "valve did not settle open"
    );
    assert!(
        block.forced_induction[1].hardware.shaft.shaft_rpm() > 1_000.0,
        "the secondary never actually came online once its valve opened: {}",
        block.forced_induction[1].hardware.shaft.shaft_rpm()
    );
}

#[test]
fn closing_vgt_vanes_spools_sooner_and_raises_back_pressure() {
    use crate::physics::turbine::VgtActuator;

    fn run(vane_target: f64) -> EngineBlock {
        let env = Environment::default();
        let mut block = EngineBlock::cross_plane_v8(env);
        block.throttle = 1.0;
        let hardware = test_turbo_hardware(&env).with_vgt(VgtActuator::new(0.1));
        block.fit_forced_induction(hardware, vec![0, 1], false);
        block.forced_induction[0]
            .hardware
            .vgt
            .as_mut()
            .unwrap()
            .target_area_fraction = vane_target;
        for _ in 0..2_500 {
            block.update(1.0 / 480.0, 4_000.0);
        }
        block
    }

    let closed = run(0.4);
    let open = run(1.0);

    assert!(
        closed.forced_induction[0]
            .hardware
            .vgt
            .unwrap()
            .area_fraction()
            < 0.41,
        "vanes did not actually settle toward their commanded closed position"
    );
    assert!(
        closed.forced_induction[0].hardware.shaft.shaft_rpm()
            > open.forced_induction[0].hardware.shaft.shaft_rpm(),
        "closed vanes should spool the wheel faster than open ones from a \
         cold start: closed={:.0} rpm, open={:.0} rpm",
        closed.forced_induction[0].hardware.shaft.shaft_rpm(),
        open.forced_induction[0].hardware.shaft.shaft_rpm()
    );
    assert!(
        closed.exhaust_banks[0].port_pressure() > open.exhaust_banks[0].port_pressure(),
        "closed vanes should raise back pressure relative to open ones: \
         closed={:.0} Pa, open={:.0} Pa",
        closed.exhaust_banks[0].port_pressure(),
        open.exhaust_banks[0].port_pressure()
    );
}

#[test]
fn anti_lag_holds_shaft_speed_and_raises_turbine_inlet_temperature_at_a_shut_throttle() {
    use crate::physics::turbine::TURBINE_INLET_TEMPERATURE_LIMIT;

    fn spooled_then_lifted(anti_lag: bool) -> EngineBlock {
        let env = Environment::default();
        let mut block = EngineBlock::cross_plane_v8(env);
        block.throttle = 1.0;
        block.fit_forced_induction(test_turbo_hardware(&env), vec![0, 1], false);
        block.ecu.anti_lag = anti_lag;
        let dt = 1.0 / 480.0;
        for _ in 0..3_000 {
            block.update(dt, 4_000.0);
        }
        // Shut throttle, foot off — the overrun condition anti-lag exists for.
        block.throttle = 0.0;
        for _ in 0..2_000 {
            block.update(dt, 3_000.0);
        }
        block
    }

    let without = spooled_then_lifted(false);
    let with = spooled_then_lifted(true);

    assert!(
        with.forced_induction[0].hardware.shaft.shaft_rpm()
            > without.forced_induction[0].hardware.shaft.shaft_rpm(),
        "anti-lag should hold shaft speed better at a shut throttle than \
         letting it decay: with={:.0} rpm, without={:.0} rpm",
        with.forced_induction[0].hardware.shaft.shaft_rpm(),
        without.forced_induction[0].hardware.shaft.shaft_rpm()
    );
    assert!(
        with.forced_induction[0]
            .hardware
            .shaft
            .turbine_inlet_temperature()
            > without.forced_induction[0]
                .hardware
                .shaft
                .turbine_inlet_temperature(),
        "anti-lag should raise turbine inlet temperature at a shut throttle"
    );
    assert!(
        with.forced_induction[0]
            .hardware
            .shaft
            .turbine_inlet_temperature()
            > TURBINE_INLET_TEMPERATURE_LIMIT,
        "sustained anti-lag should be able to reach the TB2 turbine inlet \
         temperature limit: {}",
        with.forced_induction[0]
            .hardware
            .shaft
            .turbine_inlet_temperature()
    );
}
