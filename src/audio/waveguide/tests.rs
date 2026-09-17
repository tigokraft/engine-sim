use super::*;

/// Amplitude of `signal` at `frequency`, by Goertzel-style projection.
fn magnitude_at(signal: &[f32], frequency: f32, sample_rate: f32) -> f32 {
    let (mut re, mut im) = (0.0f64, 0.0f64);
    for (i, &x) in signal.iter().enumerate() {
        let phase = std::f32::consts::TAU * frequency * i as f32 / sample_rate;
        re += x as f64 * phase.sin() as f64;
        im += x as f64 * phase.cos() as f64;
    }
    (2.0 * (re * re + im * im).sqrt() / signal.len() as f64) as f32
}

#[test]
fn wall_loss_follows_alpha_across_the_band() {
    // The claim the cascade exists to keep: one pass down a pipe costs
    // `alpha(f) L` nepers, and `alpha` goes as the square root of
    // frequency. A first-order lowpass cannot hold that shape over three
    // decades — fitted at its own -3 dB point it leaves the whole audible
    // band below the corner attenuated by almost nothing and buries
    // everything above it at 6 dB an octave. Three shelves hold it to a
    // fraction of a decibel.
    const FS: f32 = 48_000.0;
    const GAMMA: f32 = 1.33;
    const R: f32 = 287.0;

    for (radius, length, temperature) in [
        (0.022f32, 0.55f32, 900.0f32), // a V8 primary
        (0.030, 1.50, 600.0),          // its tailpipe
        (0.019, 0.40, 1_100.0),        // a four-cylinder's, hotter and narrower
    ] {
        let c = speed_of_sound(GAMMA, R, temperature);
        let mut loss = ViscothermalLoss::new(radius, length, c, GAMMA, R, temperature, FS);

        // The analytic target, straight from the formula in Appendix A.
        let nu = kinematic_viscosity(temperature, R) * BOUNDARY_LAYER_TURBULENCE_FACTOR;
        let analytic_db = |f: f64| {
            let alpha = viscothermal_alpha(
                f,
                radius as f64,
                c as f64,
                nu as f64,
                GAMMA as f64,
                EXHAUST_PRANDTL_NUMBER as f64,
            );
            alpha * length as f64 * 8.685_889_638_065_035
        };

        // Checked to 8 kHz and no further: a 60 mm duct cuts its first
        // cross mode on near 5 kHz, above which a one-dimensional
        // waveguide has nothing to claim and the mouth's own
        // [`DUCT_CUT_ON`] rolls the band away regardless. The cascade is
        // 2.5 dB shy of `alpha` at 16 kHz and that is not worth a fourth
        // shelf to fix.
        for f in [31.0f32, 125.0, 500.0, 2_000.0, 8_000.0] {
            // What the filter claims, against the formula.
            let claimed = loss.attenuation_db(f) as f64;
            let wanted = analytic_db(f as f64);
            assert!(
                (claimed - wanted).abs() < 1e-3,
                "{radius} m x {length} m at {f} Hz: slope says {claimed:.4} dB, \
                 alpha says {wanted:.4} dB"
            );

            // And what it actually does, against what it claims.
            let (mut re, mut im) = (0.0f64, 0.0f64);
            let (settle, measure) = (24_000usize, 24_000usize);
            for i in 0..settle + measure {
                let phase = std::f32::consts::TAU * f * i as f32 / FS;
                let y = loss.process(phase.sin());
                if i >= settle {
                    re += y as f64 * phase.sin() as f64;
                    im += y as f64 * phase.cos() as f64;
                }
            }
            let gain = 2.0 * (re * re + im * im).sqrt() / measure as f64;
            let measured_db = -20.0 * gain.max(1e-12).log10();
            assert!(
                (measured_db - wanted).abs() < 1.5,
                "{radius} m x {length} m at {f} Hz: {measured_db:.2} dB measured, \
                 {wanted:.2} dB from alpha"
            );
        }
    }
}

#[test]
fn cold_air_is_less_viscous_than_hot_exhaust() {
    // Sutherland over the ideal gas. The number matters because the wall
    // loss goes as its square root, so one figure for the whole synth damps
    // a cold intake runner two and a half times too hard.
    let cold = kinematic_viscosity(300.0, 287.0);
    let hot = kinematic_viscosity(900.0, 287.0);
    assert!(
        (cold - 1.57e-5).abs() < 1.0e-6,
        "air at 300 K should be near 1.57e-5 m^2/s, got {cold:e}"
    );
    assert!(
        (hot - 9.9e-5).abs() < 5.0e-6,
        "exhaust at 900 K should be near 9.9e-5 m^2/s, got {hot:e}"
    );
    // And the attenuation follows the square root of it.
    let ratio = (hot / cold).sqrt();
    let c = 400.0;
    let slope_cold = loss_slope_db(0.02, 0.5, c, 1.4, 287.0, 300.0, SMOOTH_WALL);
    let slope_hot = loss_slope_db(0.02, 0.5, c, 1.4, 287.0, 900.0, SMOOTH_WALL);
    assert!(
        ((slope_hot / slope_cold) - ratio).abs() < 0.02,
        "attenuation did not follow sqrt(nu): {:.3} against {ratio:.3}",
        slope_hot / slope_cold
    );
}

#[test]
fn a_hot_tailpipe_attenuates_the_way_a_measured_one_does() {
    // The calibration [`BOUNDARY_LAYER_TURBULENCE_FACTOR`] answers to.
    // A 60 mm automotive tailpipe running hot loses something like 0.2 to
    // 0.6 dB of a 250 Hz to 1 kHz wave per metre. That band and that pipe
    // are where an exhaust note is decided, and a constant that puts the
    // loss outside it takes the note's harmonics with it — which is what
    // sixteen did, at three to four times the top of the range.
    let (radius, length, gamma, gas_constant, temperature) = (0.030, 1.5, 1.33, 287.0, 800.0);
    let c = speed_of_sound(gamma, gas_constant, temperature);
    let loss = ViscothermalLoss::new(
        radius,
        length,
        c,
        gamma,
        gas_constant,
        temperature,
        48_000.0,
    );
    for hz in [250.0, 500.0, 1_000.0] {
        let db_per_m = loss.attenuation_db(hz) / length;
        assert!(
            (0.2..=0.6).contains(&db_per_m),
            "a hot tailpipe took {db_per_m:.3} dB/m out of {hz:.0} Hz; measurement says 0.2 to 0.6"
        );
    }
}

#[test]
fn a_smooth_wall_loses_half_of_what_a_rough_one_does() {
    // The enhancement multiplies the viscosity, so it is worth its square
    // root in attenuation: an intake tract that declares itself smooth is
    // less lossy than a header at the same size and gas, by exactly the
    // root of what it declares.
    let c = 343.0;
    let mut rough = ViscothermalLoss::new(0.021, 0.25, c, 1.4, 287.0, 300.0, 48_000.0);
    let mut smooth = ViscothermalLoss::new(0.021, 0.25, c, 1.4, 287.0, 300.0, 48_000.0);
    smooth.set_wall_enhancement(SMOOTH_WALL);
    smooth.tune(0.021, 0.25, c, 1.4, 287.0, 300.0, 48_000.0);
    rough.tune(0.021, 0.25, c, 1.4, 287.0, 300.0, 48_000.0);
    let ratio = rough.slope_db() / smooth.slope_db();
    assert!(
        (ratio - BOUNDARY_LAYER_TURBULENCE_FACTOR.sqrt()).abs() < 0.01,
        "enhancement bought {ratio:.3} in attenuation, not its own square root"
    );
}

#[test]
fn closed_open_pipe_resonates_at_c_over_four_l() {
    // A pipe shut at one end and open at the other is a quarter-wave
    // resonator: the closed end forces a pressure antinode, the open end a
    // node, and the lowest mode that fits is f = c / 4L. This is the single
    // claim the whole network rests on — every tuned length in an exhaust is
    // some version of it.
    const FS: f32 = 48_000.0;
    const GAMMA: f32 = 1.4;
    const R: f32 = 287.0;
    const TEMPERATURE: f32 = 300.0;

    for (length, radius) in [(0.5f64, 0.025f64), (0.9, 0.030), (0.35, 0.020)] {
        let area = std::f64::consts::PI * radius * radius;
        let mouth = Mouth::new(
            FS,
            radius as f32,
            false,
            speed_of_sound(GAMMA, R, TEMPERATURE),
        );
        // The open end acts as if the pipe ran on past its edge; the network
        // adds the same correction, so the resonance is set by the effective
        // length rather than the machined one.
        let effective = length + mouth.end_correction() as f64;
        let mut pipe = WaveguidePipe::new(effective, area, FS, GAMMA, R, TEMPERATURE);
        pipe.set_boundary_phase_delay(mouth.phase_delay_samples());
        pipe.tune(GAMMA, R, TEMPERATURE);
        // The delay smoother glides over 40 ms and this pipe rings for
        // about 50, so an impulse fired the instant after a retune measures
        // the glide as much as the pipe. A running network is always
        // settled; put the test in the same state.
        pipe.snap_delays();
        let mut mouth = mouth;

        // Impulse in at the closed end, radiated pressure out at the mouth.
        let mut radiated = vec![0.0f32; 1 << 16];
        for (i, out) in radiated.iter_mut().enumerate() {
            let (p_at_closed, p_at_mouth) = pipe.read_outputs();
            let (p_reflected, p_rad) = mouth.step(p_at_mouth);
            // A rigid closed end reflects in phase: r = +1.
            let excitation = if i == 0 { 1.0 } else { 0.0 };
            pipe.push_inputs(excitation + p_at_closed, p_reflected);
            *out = p_rad;
        }

        let c = speed_of_sound(GAMMA, R, TEMPERATURE);
        let expected = c / (4.0 * effective as f32);

        // Sweep a band around the prediction and take the peak.
        let mut best = (0.0f32, 0.0f32);
        let mut f = expected * 0.7;
        while f <= expected * 1.3 {
            let m = magnitude_at(&radiated[1..], f, FS);
            if m > best.1 {
                best = (f, m);
            }
            f += 0.05;
        }

        let error = (best.0 - expected).abs() / expected;
        assert!(
            error < 0.01,
            "L = {length} m: resonance at {:.2} Hz, expected {:.2} Hz ({:.2} % off)",
            best.0,
            expected,
            error * 100.0
        );
    }
}

#[test]
fn fundamental_shifts_with_mach_number() {
    // Convective mean flow biases forward and backward acoustic wave travel:
    // waves move downstream at c + u = c(1 + M) and upstream at c - u = c(1 - M).
    // For a closed-open pipe (quarter-wave resonator), the round trip period is:
    //   T = 2 * (L / (c + u) + L / (c - u)) = 4L / (c * (1 - M^2))
    // So the fundamental mode frequency shifts as:
    //   f_1 = c * (1 - M^2) / (4L)
    // in both forward (M > 0) and backward (M < 0) directions.
    const FS: f32 = 48_000.0;
    const GAMMA: f32 = 1.4;
    const R: f32 = 287.0;
    const TEMPERATURE: f32 = 300.0;
    let c = speed_of_sound(GAMMA, R, TEMPERATURE);

    let length = 0.5f64;
    let radius = 0.025f64;
    let area = std::f64::consts::PI * radius * radius;
    let mouth = Mouth::new(FS, radius as f32, false, c);
    let effective = length + mouth.end_correction() as f64;

    // Test with M = 0.20 and M = -0.20 (in both directions)
    for mach in [0.20f32, -0.20f32] {
        let mut pipe = WaveguidePipe::new(effective, area, FS, GAMMA, R, TEMPERATURE);
        pipe.set_boundary_phase_delay(mouth.phase_delay_samples());
        pipe.tune(GAMMA, R, TEMPERATURE);
        pipe.set_mach(mach);
        pipe.snap_delays();
        let mut mouth = mouth;

        // Check that forward and backward delays are asymmetric:
        let tau0 = (effective as f32) / c;
        let expected_fwd = tau0 / (1.0 + mach);
        let expected_bwd = tau0 / (1.0 - mach);
        let actual_fwd = pipe.forward_delay_samples() / FS;
        let actual_bwd = pipe.backward_delay_samples() / FS;
        assert!(
            (actual_fwd - expected_fwd).abs() / expected_fwd < 0.02,
            "Forward transit delay mismatch: actual {actual_fwd:.5}, expected {expected_fwd:.5}"
        );
        assert!(
            (actual_bwd - expected_bwd).abs() / expected_bwd < 0.02,
            "Backward transit delay mismatch: actual {actual_bwd:.5}, expected {expected_bwd:.5}"
        );
        assert!(
            (pipe.forward_delay_samples() - pipe.backward_delay_samples()).abs() > 2.0,
            "Forward and backward delays must be asymmetric under mean flow"
        );

        // Impulse response measurement
        let mut radiated = vec![0.0f32; 1 << 16];
        for (i, out) in radiated.iter_mut().enumerate() {
            let (p_at_closed, p_at_mouth) = pipe.read_outputs();
            let (p_reflected, p_rad) = mouth.step(p_at_mouth);
            let excitation = if i == 0 { 1.0 } else { 0.0 };
            pipe.push_inputs(excitation + p_at_closed, p_reflected);
            *out = p_rad;
        }

        let expected_f1 = c * (1.0 - mach * mach) / (4.0 * effective as f32);
        let mut best = (0.0f32, 0.0f32);
        let mut f = expected_f1 * 0.7;
        while f <= expected_f1 * 1.3 {
            let m = magnitude_at(&radiated[1..], f, FS);
            if m > best.1 {
                best = (f, m);
            }
            f += 0.05;
        }

        let error = (best.0 - expected_f1).abs() / expected_f1;
        assert!(
            error < 0.01,
            "Mach = {mach}: resonance at {:.2} Hz, expected {:.2} Hz ({:.2} % off)",
            best.0,
            expected_f1,
            error * 100.0
        );
    }
}

#[test]
fn pipe_with_hot_end_and_cold_end_resonates_between_bulk_predictions() {
    // A pipe that runs hot at the port (~1000 K) and cool at the tailpipe (~400 K)
    // has a sound speed that varies along its length. Its acoustic travel time
    // is the sum of the travel times through the hot and cool zones, so its
    // fundamental resonance must sit strictly between the two bulk predictions:
    //   f_cold < f_resonance < f_hot
    // and cannot land at either.
    const FS: f32 = 48_000.0;
    const GAMMA: f32 = 1.4;
    const R: f32 = 287.0;
    const T_HOT: f32 = 1000.0;
    const T_COLD: f32 = 400.0;

    let c_hot = speed_of_sound(GAMMA, R, T_HOT);
    let c_cold = speed_of_sound(GAMMA, R, T_COLD);

    let l1 = 0.40f64; // hot section
    let l2 = 0.40f64; // cold section
    let radius = 0.025f64;
    let area = std::f64::consts::PI * radius * radius;

    let mouth = Mouth::new(FS, radius as f32, false, c_cold);
    let eff_l2 = l2 + mouth.end_correction() as f64;
    let total_effective = l1 + eff_l2;

    let f_bulk_hot = c_hot / (4.0 * total_effective as f32);
    let f_bulk_cold = c_cold / (4.0 * total_effective as f32);

    let mut pipe_hot = WaveguidePipe::new(l1, area, FS, GAMMA, R, T_HOT);
    let mut pipe_cold = WaveguidePipe::new(eff_l2, area, FS, GAMMA, R, T_COLD);
    pipe_cold.set_boundary_phase_delay(mouth.phase_delay_samples());
    pipe_cold.tune(GAMMA, R, T_COLD);
    pipe_hot.snap_delays();
    pipe_cold.snap_delays();
    let junction = ScatteringJunction::new(&[pipe_hot.admittance(), pipe_cold.admittance()]);
    let mut mouth = mouth;

    let mut j_in = [0.0f32; 2];
    let mut j_out = [0.0f32; 2];

    let mut radiated = vec![0.0f32; 1 << 16];
    for (i, out) in radiated.iter_mut().enumerate() {
        let (p_at_closed, p1_to_j) = pipe_hot.read_outputs();
        let (p2_to_j, p2_to_mouth) = pipe_cold.read_outputs();

        j_in[0] = p1_to_j;
        j_in[1] = p2_to_j;
        junction.scatter(&j_in, &mut j_out);

        let (p_reflected, p_rad) = mouth.step(p2_to_mouth);

        let excitation = if i == 0 { 1.0 } else { 0.0 };
        pipe_hot.push_inputs(excitation + p_at_closed, j_out[0]);
        pipe_cold.push_inputs(j_out[1], p_reflected);
        *out = p_rad;
    }

    let mut best = (0.0f32, 0.0f32);
    let mut f = f_bulk_cold * 0.9;
    while f <= f_bulk_hot * 1.1 {
        let m = magnitude_at(&radiated[1..], f, FS);
        if m > best.1 {
            best = (f, m);
        }
        f += 0.05;
    }

    let measured_f = best.0;
    assert!(
        measured_f > f_bulk_cold + 2.0,
        "Resonance ({measured_f:.1} Hz) must be strictly above cold prediction ({f_bulk_cold:.1} Hz)"
    );
    assert!(
        measured_f < f_bulk_hot - 10.0,
        "Resonance ({measured_f:.1} Hz) must be strictly below hot prediction ({f_bulk_hot:.1} Hz)"
    );
}

#[test]
fn chamber_transmission_loss_matches_theory() {
    // A single-expansion chamber terminated anechoically has a closed-form
    // *lossless* transmission loss that depends only on the area ratio and
    // how many wavelengths fit the cavity:
    //
    //   TL = 10 log10[ 1 + (1/4) (m - 1/m)^2 sin^2(kL) ]
    //
    // It is zero whenever kL is a multiple of pi — the two junctions and
    // the cavity between them are exactly lossless, so at those
    // frequencies the chamber is transparent — and peaks at the
    // quarter-wave points. Stage T5 adds two small loss terms on top of
    // that reflection (see `ExpansionChamber`'s own docs), so the measured
    // figure is the lossless prediction plus a fixed amount this chamber's
    // geometry adds on every forward pass, not the lossless figure alone.
    // The two area steps and the pipe between them still have to scatter
    // correctly, which is what this test continues to check.
    const FS: f32 = 48_000.0;
    const GAMMA: f32 = 1.4;
    const R: f32 = 287.0;
    const TEMPERATURE: f32 = 300.0;

    let c = speed_of_sound(GAMMA, R, TEMPERATURE);
    let pipe_area = std::f64::consts::PI * 0.030 * 0.030;
    let cavity_length = 0.30f64;
    let area_ratio = 4.0f64;

    let lossless = |f: f32| {
        let k = std::f32::consts::TAU * f / c;
        let m = area_ratio as f32;
        let s = (k * cavity_length as f32).sin();
        10.0 * (1.0 + 0.25 * (m - 1.0 / m).powi(2) * s * s).log10()
    };
    // The fixed amount one forward pass through this geometry's entry
    // expansion, cavity wall and exit contraction adds, independent of
    // frequency. Read straight off a throwaway chamber rather than
    // re-derived, so this test moves automatically if the loss formulas do.
    let probe = ExpansionChamber::new(
        pipe_area,
        area_ratio,
        cavity_length,
        FS,
        GAMMA,
        R,
        TEMPERATURE,
    );
    let fixed_loss_db = -20.0
        * (probe.entry_pass() as f64 * probe.wall_pass() as f64 * probe.exit_pass() as f64)
            .log10();
    let analytic = |f: f32| lossless(f) + fixed_loss_db as f32;

    // Quarter-wave point (peak loss), half-wave point (transparent in the
    // lossless theory), and a frequency between the two.
    let quarter = c / (4.0 * cavity_length as f32);
    for f in [quarter, 2.0 * quarter, 0.5 * quarter, 1.5 * quarter] {
        let mut chamber = ExpansionChamber::new(
            pipe_area,
            area_ratio,
            cavity_length,
            FS,
            GAMMA,
            R,
            TEMPERATURE,
        );

        // Drive with a sine and read the transmitted wave. Handing the
        // chamber a zero backward wave from downstream *is* the anechoic
        // termination the analytic result assumes.
        let settle = 24_000;
        let measure = 24_000;
        let mut transmitted = vec![0.0f32; measure];
        for i in 0..(settle + measure) {
            let phase = std::f32::consts::TAU * f * i as f32 / FS;
            let (_, out) = chamber.step(phase.sin(), 0.0);
            if i >= settle {
                transmitted[i - settle] = out;
            }
        }

        let amplitude = magnitude_at(&transmitted, f, FS);
        let measured = -20.0 * amplitude.max(1e-9).log10();
        let expected = analytic(f);

        // Wider than the pre-T5 1.0 dB: `fixed_loss_db` is a single-pass
        // estimate and the losses also touch the resonance's internal
        // bounces, which the closed-form theory does not model at all.
        assert!(
            (measured - expected).abs() < 2.5,
            "at {f:.0} Hz (kL = {:.2} rad): {measured:.2} dB measured, {expected:.2} dB from \
             lossless theory + {fixed_loss_db:.2} dB fixed loss",
            std::f32::consts::TAU * f / c * cavity_length as f32
        );
    }
}

#[test]
fn expansion_chamber_transmission_loss_is_positive_at_the_transparent_frequency() {
    // The lossless closed-form TL is exactly zero at the half-wave point
    // (`kL = pi`) -- a pair of lossless scattering junctions around a
    // lossless cavity really can be perfectly transparent there. Stage T5
    // adds a wall-transmission loss and a flow loss at each area step
    // specifically so that claim stops being true: this measures TL at
    // that exact frequency and checks it is now strictly positive, which
    // only the new loss terms can produce -- the reflection math this
    // frequency was chosen to null out contributes nothing here.
    const FS: f32 = 48_000.0;
    const GAMMA: f32 = 1.4;
    const R: f32 = 287.0;
    const TEMPERATURE: f32 = 300.0;

    let c = speed_of_sound(GAMMA, R, TEMPERATURE);
    let pipe_area = std::f64::consts::PI * 0.030 * 0.030;
    let cavity_length = 0.30f64;
    let area_ratio = 4.0f64;
    let half_wave = c / (2.0 * cavity_length as f32);

    let mut chamber = ExpansionChamber::new(
        pipe_area,
        area_ratio,
        cavity_length,
        FS,
        GAMMA,
        R,
        TEMPERATURE,
    );

    let settle = 24_000;
    let measure = 24_000;
    let mut transmitted = vec![0.0f32; measure];
    for i in 0..(settle + measure) {
        let phase = std::f32::consts::TAU * half_wave * i as f32 / FS;
        let (_, out) = chamber.step(phase.sin(), 0.0);
        if i >= settle {
            transmitted[i - settle] = out;
        }
    }

    let amplitude = magnitude_at(&transmitted, half_wave, FS);
    let measured_tl = -20.0 * amplitude.max(1e-9).log10();

    assert!(
        measured_tl > 0.2,
        "at the half-wave point ({half_wave:.0} Hz) a lossless chamber \
         measures 0 dB TL; this one measures {measured_tl:.3} dB, which \
         should be clearly positive now that the chamber dissipates"
    );
}

#[test]
fn expansion_chamber_radiates_less_energy_than_no_silencer() {
    // The bug this stage exists to close, stated as directly as possible:
    // "a muffler that makes the engine 9.7 dB louder is not a muffler."
    // Two lossless scattering junctions around a cavity can only reflect
    // and store, so in a network whose only sink is the mouth a chamber
    // with no loss term measures *louder* than no silencer at all. This
    // drives a whole exhaust network, chamber fitted vs. straight pipe,
    // with the same pulse train, and requires the chamber to radiate
    // strictly less total energy.
    use crate::physics::plumbing::{
        Collector, Crossover, ExhaustSystem, PipeSection, Silencer,
    };

    const FS: f32 = 48_000.0;

    let exhaust = |silencers: Vec<Silencer>| ExhaustSystem {
        primaries: vec![PipeSection::from_diameter(0.45, 0.040, 850.0); 4],
        collector: Collector::from_diameter(4, 0.060, 0.15),
        secondary: vec![],
        crossover: Crossover::None,
        silencers,
        tailpipe: PipeSection::from_diameter(1.0, 0.060, 600.0),
        tailpipe_flanged: false,
        cutout_fitted: false,
        turbine: None,
    };

    let cylinders: Vec<crate::audio::dsp::CylinderTap> = (0..4)
        .map(|i| crate::audio::dsp::CylinderTap {
            evo_phase: i as f32 / 4.0,
            bank: 0,
        })
        .collect();
    let snapshot = crate::audio::dsp::EngineSnapshot::default();

    let total_radiated_energy = |silencers: Vec<Silencer>| -> f64 {
        let system = exhaust(silencers);
        let mut network = ExhaustNetwork::new(&system, &cylinders, 1, FS, &snapshot);
        let mut radiated = [0.0f32; 1];
        let mut energy = 0.0f64;
        for i in 0..(4 * FS as usize) {
            let pulse = if i % 240 < 6 { 1.0 } else { 0.0 };
            let excitations = [pulse, 0.0, 0.0, 0.0];
            network.step(&excitations, &mut radiated);
            energy += (radiated[0] as f64).powi(2);
        }
        energy
    };

    let muffled = total_radiated_energy(vec![Silencer::ExpansionChamber {
        length: 0.40,
        area_ratio: 4.0,
        stages: 2,
    }]);
    let straight = total_radiated_energy(vec![Silencer::Straight]);

    assert!(
        muffled < straight,
        "an expansion chamber must radiate less total energy than no \
         silencer at all: muffled={muffled:.6}, straight={straight:.6}"
    );
}

#[test]
fn packed_absorption_rises_with_frequency() {
    // The claim that separates a muffler from a volume knob: packing takes
    // the top out and leaves the bottom alone. Fibre dissipates by dragging
    // gas through itself, and the gas is still within a quarter wavelength
    // of the shell, so a layer of depth t is transparent below
    // c_packing / 4t and fully effective above it.
    //
    // Modelled as the shelf that argument implies,
    //
    //   |H(x)|^2 = (1 + p^2 x^2) / (1 + x^2),   x = f / f_q
    //
    // which is unity at DC and the packing's rated gain p far above the
    // corner. The core is given the same area as the pipe either side of
    // it, so the two area steps are transparent and what is left to measure
    // is the packing alone.
    const FS: f32 = 48_000.0;
    const GAMMA: f32 = 1.4;
    const R: f32 = 287.0;
    const TEMPERATURE: f32 = 300.0;

    let area = std::f64::consts::PI * 0.030 * 0.030;
    let length = 0.50f64;
    let thickness = 0.035f64;
    let loss_db_per_m = 20.0f64;

    let c = speed_of_sound(GAMMA, R, TEMPERATURE);
    let corner = packing_corner_hz(thickness, c);
    let pass = 10f64.powf(-loss_db_per_m * length / 20.0) as f32;

    let shelf_loss = |f: f32| {
        let x = f / corner;
        -10.0 * ((1.0 + pass * pass * x * x) / (1.0 + x * x)).log10()
    };
    // The core is a duct like any other and its wall takes its own
    // `sqrt(f)` out of whatever passes through. That is not the packing, so
    // it is predicted separately and charged separately rather than left to
    // contaminate the shelf it would otherwise be mistaken for.
    let wall_loss = |f: f32| {
        loss_slope_db(
            0.030,
            length as f32,
            c,
            GAMMA,
            R,
            TEMPERATURE,
            BOUNDARY_LAYER_TURBULENCE_FACTOR,
        ) * f.max(0.0).sqrt()
    };
    let analytic_loss = |f: f32| shelf_loss(f) + wall_loss(f);

    let measured_loss = |f: f32| {
        let mut silencer = AbsorptiveSilencer::new(
            area,
            area,
            length,
            thickness,
            loss_db_per_m,
            FS,
            GAMMA,
            R,
            TEMPERATURE,
        );
        let (settle, measure) = (24_000, 24_000);
        let mut transmitted = vec![0.0f32; measure];
        for i in 0..(settle + measure) {
            let phase = std::f32::consts::TAU * f * i as f32 / FS;
            // A zero backward wave from downstream is the anechoic
            // termination, as in the chamber test above.
            let (_, out) = silencer.step(phase.sin(), 0.0);
            if i >= settle {
                transmitted[i - settle] = out;
            }
        }
        -20.0 * magnitude_at(&transmitted, f, FS).max(1e-9).log10()
    };

    // Transparent an octave and a half below the corner, working hard two
    // octaves above it, and never going backwards in between. The upper
    // bound on the low point is the whole difference from a flat gain,
    // which would have taken all 10 dB out down here as well.
    let low = measured_loss(corner / 8.0);
    let mid = measured_loss(corner);
    let high = measured_loss(4.0 * corner);
    let packing_only = low - wall_loss(corner / 8.0);
    assert!(
        packing_only < 0.5,
        "packing is not transparent below its corner: {packing_only:.2} dB at {:.0} Hz,              beyond the {:.2} dB the core's own wall takes",
        corner / 8.0,
        wall_loss(corner / 8.0)
    );
    assert!(
        high > 7.0,
        "packing is not working above its corner: {high:.2} dB at {:.0} Hz",
        4.0 * corner
    );
    assert!(
        low < mid && mid < high,
        "absorption did not rise with frequency: {low:.2}, {mid:.2}, {high:.2} dB"
    );

    // And it follows the shelf, not merely the right direction. The core's
    // wall loss is in the prediction now rather than excused from it, so
    // what is left over is the shelf's own discretisation.
    for f in [corner / 8.0, corner / 2.0, corner, 2.0 * corner] {
        let (measured, expected) = (measured_loss(f), analytic_loss(f));
        assert!(
            (measured - expected).abs() < 0.5,
            "at {f:.0} Hz (f/f_q = {:.2}): {measured:.2} dB measured, {expected:.2} dB from the shelf",
            f / corner
        );
    }
}

#[test]
fn crossover_transfers_energy_between_banks() {
    // What separates a flat-plane V8 from a cross-plane one at equal firing
    // order is whether the banks can hear each other. Fire only bank 0 and
    // listen at bank 1's tailpipe: with an X-pipe, energy arrives; with
    // `Crossover::None` the banks are two separate exhausts and nothing
    // does. Nothing here is a mixing coefficient — the transfer is whatever
    // the 4-port junction scatters.
    use crate::physics::plumbing::{
        Collector, Crossover, ExhaustSystem, PipeSection, Silencer,
    };

    const FS: f32 = 48_000.0;

    let system = |crossover: Crossover| ExhaustSystem {
        primaries: vec![PipeSection::from_diameter(0.45, 0.040, 850.0); 8],
        collector: Collector::from_diameter(4, 0.060, 0.15),
        secondary: vec![],
        crossover,
        silencers: vec![Silencer::Straight],
        tailpipe: PipeSection::from_diameter(1.0, 0.060, 600.0),
        tailpipe_flanged: false,
        cutout_fitted: false,
        turbine: None,
    };

    // A cross-plane V8's banks: cylinders 0, 2, 3, 7 on one, the rest on the other.
    let cylinders: Vec<crate::audio::dsp::CylinderTap> = [0, 1, 0, 0, 1, 1, 1, 0]
        .iter()
        .enumerate()
        .map(|(i, &bank)| crate::audio::dsp::CylinderTap {
            evo_phase: i as f32 / 8.0,
            bank,
        })
        .collect();

    let snapshot = crate::audio::dsp::EngineSnapshot::default();
    let far_bank_energy = |crossover: Crossover| {
        let exhaust = system(crossover);
        let mut network = ExhaustNetwork::new(&exhaust, &cylinders, 2, FS, &snapshot);
        let mut excitations = vec![0.0f32; cylinders.len()];
        let mut radiated = vec![0.0f32; 2];

        let mut near = 0.0f64;
        let mut far = 0.0f64;
        for i in 0..48_000 {
            // Impulse into one cylinder of bank 0 only. Every other port
            // stays silent, so anything at bank 1's mouth crossed over.
            excitations.fill(0.0);
            if i == 0 {
                excitations[0] = 1.0;
            }
            network.step(&excitations, &mut radiated);
            near += (radiated[0] as f64).powi(2);
            far += (radiated[1] as f64).powi(2);
        }
        (near, far)
    };

    let (isolated_near, isolated_far) = far_bank_energy(Crossover::None);
    let (crossed_near, crossed_far) = far_bank_energy(Crossover::XPipe { position: 0.80 });

    assert!(
        isolated_near > 0.0 && crossed_near > 0.0,
        "the fired bank radiated nothing at all"
    );
    assert_eq!(
        isolated_far, 0.0,
        "energy reached the far bank with no crossover fitted: {isolated_far:e}"
    );
    assert!(
        crossed_far > 0.05 * crossed_near,
        "the X-pipe passed almost nothing across: {crossed_far:e} against {crossed_near:e} on the fired bank"
    );
}

#[test]
fn stub_notches_at_c_over_four_l_stub() {
    // A closed side branch is a drone killer: the round trip up and back
    // covers half a wavelength at f = c / 4L, and the rigid end reflects in
    // phase, so what returns to the junction arrives inverted and cancels.
    // The notch frequency is set by the branch length and nothing else.
    const FS: f32 = 48_000.0;
    const GAMMA: f32 = 1.4;
    const R: f32 = 287.0;
    const TEMPERATURE: f32 = 300.0;

    let c = speed_of_sound(GAMMA, R, TEMPERATURE);
    let pipe_area = std::f64::consts::PI * 0.030 * 0.030;
    let stub_area = std::f64::consts::PI * 0.020 * 0.020;

    for stub_length in [0.25f64, 0.40] {
        let expected = c / (4.0 * stub_length as f32);

        // Sweep the band around the prediction and find the deepest point.
        let mut deepest = (0.0f32, f32::MAX);
        let mut f = expected * 0.75;
        while f <= expected * 1.25 {
            let mut stub = QuarterWaveStub::new(
                pipe_area,
                stub_area,
                stub_length,
                FS,
                GAMMA,
                R,
                TEMPERATURE,
            );
            let settle = 24_000;
            let measure = 24_000;
            let mut transmitted = vec![0.0f32; measure];
            for i in 0..(settle + measure) {
                let phase = std::f32::consts::TAU * f * i as f32 / FS;
                let (_, out) = stub.step(phase.sin(), 0.0);
                if i >= settle {
                    transmitted[i - settle] = out;
                }
            }
            let m = magnitude_at(&transmitted, f, FS);
            if m < deepest.1 {
                deepest = (f, m);
            }
            f += expected * 0.002;
        }

        let error = (deepest.0 - expected).abs() / expected;
        assert!(
            error < 0.02,
            "stub of {stub_length} m notched at {:.1} Hz, expected {:.1} Hz ({:.1} % off)",
            deepest.0,
            expected,
            error * 100.0
        );
        assert!(
            deepest.1 < 0.5,
            "the notch is not a notch: {:.3} of the drive still gets through",
            deepest.1
        );
    }
}

#[test]
fn a_narrow_pipe_is_duller_than_a_wide_one() {
    // Wall loss goes as sqrt(f) / a, so a narrow pipe swallows its top end
    // and a wide one does not. This is free differentiation between a bike's
    // 38 mm primary and a truck's 76 mm pipe — no tone control anywhere.
    const FS: f32 = 48_000.0;
    const GAMMA: f32 = 1.4;
    const R: f32 = 287.0;
    const TEMPERATURE: f32 = 300.0;

    let brightness = |radius: f64| {
        let area = std::f64::consts::PI * radius * radius;
        let mut pipe = WaveguidePipe::new(0.6, area, FS, GAMMA, R, TEMPERATURE);
        // Straight through: impulse in one end, read what leaves the other,
        // with nothing reflecting at either boundary.
        let mut out = vec![0.0f32; 4_096];
        for (i, sample) in out.iter_mut().enumerate() {
            let (_, p_far) = pipe.read_outputs();
            pipe.push_inputs(if i == 0 { 1.0 } else { 0.0 }, 0.0);
            *sample = p_far;
        }
        let high = magnitude_at(&out, 6_000.0, FS);
        let low = magnitude_at(&out, 300.0, FS);
        high / low.max(1e-9)
    };

    // What the law predicts for the same two pipes: the tilt between the
    // two probe frequencies is the difference of the attenuations there,
    // and it doubles when the radius halves.
    let predicted = |radius: f64| {
        let alpha = |f: f64| {
            viscothermal_alpha(
                f,
                radius,
                speed_of_sound(GAMMA, R, TEMPERATURE) as f64,
                (kinematic_viscosity(TEMPERATURE, R) * BOUNDARY_LAYER_TURBULENCE_FACTOR) as f64,
                GAMMA as f64,
                EXHAUST_PRANDTL_NUMBER as f64,
            )
        };
        (-(alpha(6_000.0) - alpha(300.0)) * 0.6).exp() as f32
    };

    let narrow = brightness(0.019);
    let wide = brightness(0.038);
    assert!(
        wide > narrow,
        "the narrow pipe was not duller: {narrow:.3} against {wide:.3} wide"
    );
    // The ordering alone is cheap — a filter that rolls off at 6 dB/octave
    // instead of as sqrt(f) also gets it right, and gets the size of it
    // wrong by an octave's worth. So the tilt is held to the analytic
    // figure, within the tolerance of the three-shelf fit and the delay
    // line's own interpolation.
    for (radius, measured) in [(0.019f64, narrow), (0.038, wide)] {
        let want = predicted(radius);
        let error_db = 20.0 * (measured / want).log10();
        assert!(
            error_db.abs() < 1.5,
            "a {radius} m pipe tilted {measured:.3} between 300 Hz and 6 kHz,                  against {want:.3} from alpha ({error_db:+.2} dB out)"
        );
    }
}

#[test]
fn two_pipe_junction_reflects_exact_area_ratio() {
    let test_cases = [
        (0.0010, 0.0020), // expansion: r = (1-2)/(1+2) = -1/3
        (0.0030, 0.0010), // contraction: r = (3-1)/(3+1) = +0.5
        (0.0025, 0.0025), // matched: r = 0
        (0.0012, 0.0060), // large expansion: r = (1.2-6)/(1.2+6) = -4.8/7.2 = -2/3
    ];

    for (a1, a2) in test_cases {
        let junction = ScatteringJunction::from_areas(&[a1, a2]);
        let expected_r = ((a1 - a2) / (a1 + a2)) as f32;
        let expected_t = (2.0 * a1 / (a1 + a2)) as f32;

        let p_plus = [1.0f32, 0.0f32];
        let mut p_minus = [0.0f32, 0.0f32];
        let p_j = junction.scatter(&p_plus, &mut p_minus);

        assert!(
            (p_minus[0] - expected_r).abs() < 1e-6,
            "expected reflection {expected_r}, got {}",
            p_minus[0]
        );
        assert!(
            (p_minus[1] - expected_t).abs() < 1e-6,
            "expected transmission {expected_t}, got {}",
            p_minus[1]
        );
        assert!(
            (p_j - expected_t).abs() < 1e-6,
            "junction pressure should match transmitted pressure"
        );
    }
}

#[test]
fn n_port_junction_conserves_volume_flow_and_power() {
    // 4-1 collector junction: 4 primaries of 40 mm bore meeting 60 mm outlet
    let a_primary = std::f64::consts::PI * 0.020 * 0.020;
    let a_outlet = std::f64::consts::PI * 0.030 * 0.030;
    let junction =
        ScatteringJunction::from_areas(&[a_primary, a_primary, a_primary, a_primary, a_outlet]);

    let p_plus = [1.0f32, 0.3f32, -0.2f32, 0.0f32, -0.5f32];
    let mut p_minus = [0.0f32; 5];
    junction.scatter(&p_plus, &mut p_minus);

    let y = junction.admittances();

    // 1. Volume flow conservation: sum_i Y_i (p_i^+ - p_i^-) == 0
    let mut net_flow = 0.0f32;
    for i in 0..5 {
        net_flow += y[i] * (p_plus[i] - p_minus[i]);
    }
    assert!(
        net_flow.abs() < 1e-6,
        "volume flow not conserved: net_flow = {net_flow}"
    );

    // 2. Power conservation: sum_i Y_i (p_i^+)^2 == sum_i Y_i (p_i^-)^2
    let mut power_in = 0.0f32;
    let mut power_out = 0.0f32;
    for i in 0..5 {
        power_in += y[i] * p_plus[i] * p_plus[i];
        power_out += y[i] * p_minus[i] * p_minus[i];
    }
    assert!(
        (power_out - power_in).abs() < 1e-6 * power_in,
        "power not conserved: in={power_in}, out={power_out}"
    );
    assert!(
        power_out <= power_in + 1e-7,
        "outgoing energy exceeds incoming"
    );
}

#[test]
fn port_jet_noise_is_the_cube_of_throat_velocity() {
    // Curle's dipole is a sixth power in radiated *power*, which is a cube
    // in pressure. Doubling the velocity has to be eighteen decibels, not
    // twelve — that is the whole difference between a chuff at each valve
    // event and a hiss across the cycle.
    let rho = 0.39;
    let c = 600.0;
    let area = 5.0e-4;
    let pipe = std::f32::consts::PI * 0.020 * 0.020;
    let at = |u: f32| jet_pressure_fluctuation_pa(u, rho, c, area, pipe);
    let ratio = at(300.0) / at(150.0);
    assert!(
        (ratio - 8.0).abs() < 1e-3,
        "doubling velocity must be eight times the pressure: {ratio}"
    );
    assert_eq!(
        at(0.0),
        0.0,
        "a port with no flow through it makes no noise"
    );
}

#[test]
fn a_port_that_chokes_stops_getting_louder() {
    // The gap passes no more velocity past its sonic limit however much
    // harder the cylinder pushes, so the noise has a ceiling and it is the
    // gas that sets it.
    let rho = 0.39;
    let c = 600.0;
    let area = 5.0e-4;
    let sonic = throat_velocity(rho * area * c, area, rho, c);
    let beyond = throat_velocity(rho * area * c * 4.0, area, rho, c);
    assert!(
        (sonic - c).abs() < 1e-3,
        "the throat should be at Mach 1: {sonic}"
    );
    assert_eq!(sonic, beyond, "a choked throat cannot go faster");
}

#[test]
fn a_narrower_gap_hisses_higher() {
    // St = f d / u. At one velocity the pitch is set by the gap alone, and
    // it goes as one over the diameter, so a quarter of the area is twice
    // the frequency.
    let wide = jet_peak_hz(300.0, 4.0e-4);
    let narrow = jet_peak_hz(300.0, 1.0e-4);
    assert!(
        ((narrow / wide) - 2.0).abs() < 1e-3,
        "quartering the area must double the pitch: {wide:.0} Hz to {narrow:.0} Hz"
    );
    // And the absolute number is the Strouhal law, not a tuning.
    let d = 2.0 * (4.0e-4f32 / std::f32::consts::PI).sqrt();
    assert!((wide - JET_STROUHAL_NUMBER * 300.0 / d).abs() < 1e-3);
}

#[test]
fn steepening_raises_high_orders_with_amplitude() {
    // A crest rides on its own induced flow through gas it has itself
    // heated, so it gains on the trough ahead of it and the front stands
    // up. At a fixed fundamental that shows as high-order content that
    // grows with amplitude — and vanishes when the pulse is small, because
    // beta is p / P_0 and a quiet wave has nothing to gain on.
    const FS: f32 = 48_000.0;
    const FREQ: f32 = 200.0; // 240 samples per cycle at 48 kHz
    const GAMMA: f32 = 1.35;
    const R: f32 = 287.0;
    const TEMPERATURE: f32 = 800.0;
    let area = std::f64::consts::PI * 0.020 * 0.020;

    let measure_harmonic_ratio = |amp: f32| -> f32 {
        let mut pipe = WaveguidePipe::new(0.5, area, FS, GAMMA, R, TEMPERATURE);
        pipe.set_steepening(true);
        pipe.snap_delays();

        let n_total = 4800; // 20 complete cycles
        let n_eval = 2400; // 10 cycles for steady-state evaluation
        let mut re_fund = 0.0f64;
        let mut im_fund = 0.0f64;
        let mut re_h2 = 0.0f64;
        let mut im_h2 = 0.0f64;

        for i in 0..n_total {
            let t = i as f32 / FS;
            let input = amp * (std::f32::consts::TAU * FREQ * t).sin();
            pipe.push_inputs(input, 0.0);
            let (_out0, out1) = pipe.read_outputs();

            if i >= n_total - n_eval {
                let phase1 = std::f32::consts::TAU * FREQ * t;
                let phase2 = std::f32::consts::TAU * 2.0 * FREQ * t;
                re_fund += out1 as f64 * phase1.sin() as f64;
                im_fund += out1 as f64 * phase1.cos() as f64;
                re_h2 += out1 as f64 * phase2.sin() as f64;
                im_h2 += out1 as f64 * phase2.cos() as f64;
            }
        }

        let m_fund = (re_fund * re_fund + im_fund * im_fund).sqrt() / n_eval as f64;
        let m_h2 = (re_h2 * re_h2 + im_h2 * im_h2).sqrt() / n_eval as f64;
        (m_h2 / m_fund.max(1e-12)) as f32
    };

    // A hundred pascals is a loud noise and a thousandth of an atmosphere.
    let ratio_quiet = measure_harmonic_ratio(100.0);
    // A blowdown is a fair fraction of an atmosphere.
    let ratio_loud = measure_harmonic_ratio(0.5 * REFERENCE_PRESSURE_PA);

    assert!(
        ratio_quiet < 0.001,
        "high orders should be absent at low amplitude: got {ratio_quiet}"
    );
    assert!(
        ratio_loud > 0.05,
        "high orders should rise with amplitude: got {ratio_loud}"
    );
    assert!(
        ratio_loud > 50.0 * ratio_quiet,
        "high-order content must rise strongly with amplitude: loud={ratio_loud}, quiet={ratio_quiet}"
    );
}

#[test]
fn opening_the_cutout_raises_high_order_content_and_lowers_back_pressure() {
    use crate::physics::plumbing::{
        Collector, Crossover, ExhaustSystem, PipeSection, Silencer,
    };

    const FS: f32 = 48_000.0;

    let exhaust = ExhaustSystem {
        primaries: vec![PipeSection::from_diameter(0.45, 0.040, 850.0); 4],
        collector: Collector::from_diameter(4, 0.060, 0.15),
        secondary: vec![],
        crossover: Crossover::None,
        silencers: vec![
            Silencer::ExpansionChamber {
                length: 0.40,
                area_ratio: 4.0,
                stages: 2,
            },
            Silencer::Absorptive {
                length: 0.35,
                area: std::f64::consts::PI * 0.030 * 0.030,
                packing_thickness: 0.035,
                packing_absorption: 0.85,
            },
        ],
        tailpipe: PipeSection::from_diameter(1.0, 0.060, 600.0),
        tailpipe_flanged: false,
        cutout_fitted: false,
        turbine: None,
    };

    // 1. Back pressure must be strictly lower with cutout open than closed
    let mass_flow = 0.20; // 200 g/s exhaust flow
    let bp_closed = exhaust.back_pressure(mass_flow, false, None);
    let bp_open = exhaust.back_pressure(mass_flow, true, None);
    assert!(
        bp_open < bp_closed * 0.70,
        "opening cutout must significantly lower back pressure: open={bp_open}, closed={bp_closed}"
    );

    // 2. High-order harmonic content in radiated sound must be higher with cutout open
    let cylinders: Vec<crate::audio::dsp::CylinderTap> = (0..4)
        .map(|i| crate::audio::dsp::CylinderTap {
            evo_phase: i as f32 / 4.0,
            bank: 0,
        })
        .collect();

    let snapshot = crate::audio::dsp::EngineSnapshot::default();
    let run_and_measure_high_order_energy = |cutout_open: bool| -> f64 {
        let mut network = ExhaustNetwork::new(&exhaust, &cylinders, 1, FS, &snapshot);
        network.set_cutout(cutout_open);

        let mut high_energy = 0.0f64;
        let mut radiated = [0.0f32; 1];

        for i in 0..9600 {
            let pulse = if i % 240 < 6 { 1.0 } else { 0.0 };
            let excitations = [pulse, 0.0, 0.0, 0.0];
            network.step(&excitations, &mut radiated);

            if i >= 2400 {
                for freq in [1500.0, 2000.0, 3000.0, 4000.0] {
                    let phase = std::f32::consts::TAU * freq * (i as f32) / FS;
                    high_energy += (radiated[0] as f64 * phase.sin() as f64).powi(2);
                }
            }
        }
        high_energy
    };

    let high_closed = run_and_measure_high_order_energy(false);
    let high_open = run_and_measure_high_order_energy(true);

    assert!(
        high_open > high_closed * 2.0,
        "opening the cutout bypass must raise high-order spectral content: open={high_open}, closed={high_closed}"
    );
}

#[test]
fn cutout_fitment_alone_does_not_open_the_cutout() {
    // The other half of the split: `cutout_fitted` says a bypass valve
    // exists, not that it is open. `ExhaustNetwork::new` used to seed its
    // live `cutout_open` state straight from that fitment flag, so
    // `cross_plane_v8` and `twin_turbo_v8` — both shipped with a cutout
    // fitted — rendered wide open from frame zero regardless of what any
    // caller asked for. Construct the same silencer chain with fitment
    // true and false and, without ever calling `set_cutout`, require the
    // renders to be bit-identical: a fitted-but-unopened cutout must sound
    // exactly like no cutout at all.
    use crate::physics::plumbing::{
        Collector, Crossover, ExhaustSystem, PipeSection, Silencer,
    };

    const FS: f32 = 48_000.0;

    let exhaust = |cutout_fitted: bool| ExhaustSystem {
        primaries: vec![PipeSection::from_diameter(0.45, 0.040, 850.0); 4],
        collector: Collector::from_diameter(4, 0.060, 0.15),
        secondary: vec![],
        crossover: Crossover::None,
        silencers: vec![Silencer::ExpansionChamber {
            length: 0.40,
            area_ratio: 4.0,
            stages: 2,
        }],
        tailpipe: PipeSection::from_diameter(1.0, 0.060, 600.0),
        tailpipe_flanged: false,
        cutout_fitted,
        turbine: None,
    };

    let cylinders: Vec<crate::audio::dsp::CylinderTap> = (0..4)
        .map(|i| crate::audio::dsp::CylinderTap {
            evo_phase: i as f32 / 4.0,
            bank: 0,
        })
        .collect();
    let snapshot = crate::audio::dsp::EngineSnapshot::default();

    let render = |cutout_fitted: bool| -> Vec<f32> {
        let system = exhaust(cutout_fitted);
        let mut network = ExhaustNetwork::new(&system, &cylinders, 1, FS, &snapshot);
        assert!(!network.is_cutout_open(), "must construct closed");
        let mut radiated = [0.0f32; 1];
        let mut out = Vec::with_capacity(4800);
        for i in 0..4800 {
            let pulse = if i % 240 < 6 { 1.0 } else { 0.0 };
            let excitations = [pulse, 0.0, 0.0, 0.0];
            network.step(&excitations, &mut radiated);
            out.push(radiated[0]);
        }
        out
    };

    let fitted = render(true);
    let not_fitted = render(false);
    assert_eq!(
        fitted, not_fitted,
        "a cutout fitted but never opened must render bit-identically to no cutout at all"
    );
}

#[test]
fn open_headers_and_cutout_enable_tailpipe_steepening() {
    use crate::physics::plumbing::ExhaustSystem;

    let primaries =
        vec![crate::physics::plumbing::PipeSection::from_diameter(0.60, 0.040, 850.0); 4];
    let collector = crate::physics::plumbing::Collector::from_diameter(4, 0.060, 0.15);
    let exhaust = ExhaustSystem::open_headers(primaries, collector);
    let cylinders = vec![
        crate::audio::dsp::CylinderTap {
            evo_phase: 0.0,
            bank: 0,
        },
        crate::audio::dsp::CylinderTap {
            evo_phase: 0.5,
            bank: 0,
        },
    ];
    let snapshot = crate::audio::dsp::EngineSnapshot::default();
    let mut network = ExhaustNetwork::new(&exhaust, &cylinders, 1, 48_000.0, &snapshot);
    assert!(network.tailpipes[0].steepening());

    network.set_cutout(false);
    assert!(!network.tailpipes[0].steepening());

    network.set_cutout(true);
    assert!(network.tailpipes[0].steepening());
}

/// Builds a single-cylinder exhaust that is, acoustically, one
/// uninterrupted pipe: a collector with one inlet whose outlet area
/// matches the primary's leaves nothing for the junction to reflect, so
/// the primary, the collector's taper and the tailpipe scatter as though
/// they were cut from the same tube. Closed at the valve (unset, so
/// [`ValveTermination`] reflects at unity — see its `new`), open at the
/// mouth. That is Stage T4's "single pipe with no silencers and no
/// collector": the only boundaries left are the two ends.
fn single_pipe(
    l_primary: f64,
    taper: f64,
    tailpipe: f64,
    diameter: f64,
) -> crate::physics::plumbing::ExhaustSystem {
    single_pipe_flanged(l_primary, taper, tailpipe, diameter, false)
}

/// As [`single_pipe`], but with the mouth's flange choice exposed.
fn single_pipe_flanged(
    l_primary: f64,
    taper: f64,
    tailpipe: f64,
    diameter: f64,
    flanged: bool,
) -> crate::physics::plumbing::ExhaustSystem {
    use crate::physics::plumbing::{
        Collector, Crossover, ExhaustSystem, PipeSection, Silencer,
    };
    let area = std::f64::consts::PI * (diameter * 0.5).powi(2);
    ExhaustSystem {
        primaries: vec![PipeSection::from_diameter(l_primary, diameter, 300.0)],
        collector: Collector::new(1, area, taper),
        secondary: vec![],
        crossover: Crossover::None,
        silencers: vec![Silencer::Straight],
        tailpipe: PipeSection::from_diameter(tailpipe, diameter, 300.0),
        tailpipe_flanged: flanged,
        cutout_fitted: false,
        turbine: None,
    }
}

/// The strongest resonance a single impulse rings up in a network's
/// radiated output, scanned by Goertzel projection near `guess_hz`.
///
/// One impulse excites every mode of the pipe at once; scanning the
/// projection of the resulting ring-down onto each candidate frequency and
/// keeping the largest finds where the energy actually piled up, which for
/// an undamped-ish closed-open pipe is its quarter-wave fundamental.
fn impulse_resonance_hz(
    exhaust: &crate::physics::plumbing::ExhaustSystem,
    fs: f32,
    guess_hz: f32,
    search_fraction: f32,
) -> f32 {
    let cylinders = vec![crate::audio::dsp::CylinderTap {
        evo_phase: 0.0,
        bank: 0,
    }];
    let snapshot = crate::audio::dsp::EngineSnapshot::default();
    let mut network = ExhaustNetwork::new(exhaust, &cylinders, 1, fs, &snapshot);

    let capture = 4 * fs as usize; // 4 s: hundreds of round trips, 0.25 Hz Goertzel resolution
    let mut response = Vec::with_capacity(capture);
    let mut radiated = [0.0f32; 1];
    for i in 0..capture {
        let excitations = [if i == 0 { 1.0 } else { 0.0 }];
        network.step(&excitations, &mut radiated);
        response.push(radiated[0]);
    }

    let mut best = (0.0f32, -1.0f32);
    let lo = guess_hz * (1.0 - search_fraction);
    let hi = guess_hz * (1.0 + search_fraction);
    let mut f = lo;
    while f <= hi {
        let m = magnitude_at(&response, f, fs);
        if m > best.1 {
            best = (f, m);
        }
        f += guess_hz * 0.0005;
    }
    best.0
}

#[test]
fn single_pipe_resonates_at_the_quarter_wave_prediction() {
    // Appendix A's `f = c(1 - M^2) / 4L`, with M = 0 here (no mean flow is
    // set on this network) and L the pipe's *acoustic* length — physical
    // length plus the mouth's own Karal-Flugge end correction, per
    // `examples/calibrate.rs`'s treatment of the same mode.
    use crate::audio::radiation::end_correction;

    const FS: f32 = 48_000.0;
    const DIAMETER: f64 = 0.045;
    const TAPER: f64 = 0.02; // TaperedCollector's own floor, see its `new`
    const TAILPIPE: f64 = 0.05;
    let radius = DIAMETER * 0.5;
    let delta = end_correction(radius, false);
    let c = speed_of_sound(1.33, 287.0, 300.0);

    for &l_primary in &[0.50f64, 0.80, 1.20] {
        let exhaust = single_pipe(l_primary, TAPER, TAILPIPE, DIAMETER);
        let total_length = l_primary + TAPER + TAILPIPE + delta;
        let expected = c / (4.0 * total_length as f32);

        let measured = impulse_resonance_hz(&exhaust, FS, expected, 0.3);
        let error = (measured - expected).abs() / expected;
        assert!(
            error < 0.02,
            "primary {l_primary} m: measured {measured:.1} Hz against {expected:.1} Hz \
             predicted ({:.1} % off)",
            error * 100.0
        );
    }
}

#[test]
fn doubling_primary_length_halves_the_tuning_peak() {
    // Long enough that the fixed taper, tailpipe and end correction are a
    // small fraction of the total, so doubling the primary comes close to
    // doubling the whole acoustic length and the peak comes close to
    // halving. The stage's own claim is about the primary, not about a
    // pipe with nothing else attached to it.
    const FS: f32 = 48_000.0;
    const DIAMETER: f64 = 0.045;
    const TAPER: f64 = 0.02;
    const TAILPIPE: f64 = 0.05;
    let c = speed_of_sound(1.33, 287.0, 300.0);

    let guess = |l: f64| c / (4.0 * (l + TAPER + TAILPIPE) as f32);

    let short = 1.0f64;
    let long = 2.0 * short;
    let exhaust_short = single_pipe(short, TAPER, TAILPIPE, DIAMETER);
    let exhaust_long = single_pipe(long, TAPER, TAILPIPE, DIAMETER);

    let f_short = impulse_resonance_hz(&exhaust_short, FS, guess(short), 0.3);
    let f_long = impulse_resonance_hz(&exhaust_long, FS, guess(long), 0.3);

    let ratio = f_long / f_short;
    assert!(
        (ratio - 0.5).abs() < 0.05,
        "doubling the primary from {short} m to {long} m moved the peak from \
         {f_short:.1} Hz to {f_long:.1} Hz, a ratio of {ratio:.3} rather than one half"
    );
}

#[test]
fn unflanged_tailpipe_effective_length_matches_karal_flugge() {
    // A tailpipe read through the full `ExhaustNetwork` — valve, junction,
    // taper and the collector's one-sample return register, see that
    // field's own doc comment — carries a little extra latency beside the
    // mouth's, on the order of the "about 7 mm of pipe" the network
    // documents for that register alone. That is shared by any tailpipe on
    // this rig regardless of its flange, so building the *same* geometry
    // flanged and unflanged and differencing the two measured peaks
    // cancels it and leaves exactly the term this test is about: the
    // 0.8216a and 0.6133a end corrections read back off where the peak
    // actually landed, rather than asserted.
    use crate::audio::radiation::{FLANGED_END_CORRECTION, UNFLANGED_END_CORRECTION};

    const FS: f32 = 48_000.0;
    const DIAMETER: f64 = 0.050;
    const TAPER: f64 = 0.02;
    const TAILPIPE: f64 = 0.35;
    const L_PRIMARY: f64 = 0.60;
    let radius = DIAMETER * 0.5;
    let c = speed_of_sound(1.33, 287.0, 300.0);
    let guess = c / (4.0 * (L_PRIMARY + TAPER + TAILPIPE) as f32);

    let unflanged = single_pipe_flanged(L_PRIMARY, TAPER, TAILPIPE, DIAMETER, false);
    let flanged = single_pipe_flanged(L_PRIMARY, TAPER, TAILPIPE, DIAMETER, true);

    let f_unflanged = impulse_resonance_hz(&unflanged, FS, guess, 0.3);
    let f_flanged = impulse_resonance_hz(&flanged, FS, guess, 0.3);

    // A wider end correction is a longer effective pipe, and a longer pipe
    // resonates lower: the flanged case must land under the unflanged one.
    assert!(
        f_flanged < f_unflanged,
        "the flanged mouth ({f_flanged:.1} Hz) should resonate lower than the \
         unflanged one ({f_unflanged:.1} Hz), not higher or the same"
    );

    let implied_gap = c / (4.0 * f_flanged) - c / (4.0 * f_unflanged);
    let expected_gap = ((FLANGED_END_CORRECTION - UNFLANGED_END_CORRECTION) * radius) as f32;

    let error = (implied_gap - expected_gap).abs() / expected_gap;
    assert!(
        error < 0.20,
        "flanged vs unflanged moved the effective length by {implied_gap:.4} m; the \
         Karal-Flugge constants ({FLANGED_END_CORRECTION} - {UNFLANGED_END_CORRECTION}) x \
         {radius:.4} m radius predict {expected_gap:.4} m ({:.1} % off)",
        error * 100.0
    );
}

#[test]
fn exhaust_modes_raise_high_order_content_monotonically() {
    // Same shape of claim as `opening_the_cutout_raises_high_order_content`
    // above, over the three exhaust modes instead of the cutout: muffled
    // (a real silencer fitted, not `Silencer::Straight`, or this could not
    // tell muffled from straight-pipe) should carry the least top-end,
    // straight-pipe more, and open-headers — no tailpipe, no crossover to
    // cross into — the most.
    //
    // `Silencer::Absorptive` rather than `ExpansionChamber`, so this test
    // does not depend on the reactive chamber's own loss term (added in
    // Stage T5) being tuned any particular way — it is about T4's claim,
    // not T5's, and reaches for the silencer that has always attenuated.
    use crate::physics::plumbing::{
        Collector, Crossover, ExhaustSystem, PipeSection, Silencer,
    };

    const FS: f32 = 48_000.0;

    let exhaust = ExhaustSystem {
        primaries: vec![PipeSection::from_diameter(0.45, 0.040, 850.0); 4],
        collector: Collector::from_diameter(4, 0.060, 0.15),
        secondary: vec![],
        crossover: Crossover::None,
        silencers: vec![Silencer::Absorptive {
            length: 0.45,
            area: std::f64::consts::PI * 0.030 * 0.030,
            packing_thickness: 0.035,
            packing_absorption: 0.85,
        }],
        tailpipe: PipeSection::from_diameter(1.0, 0.060, 600.0),
        tailpipe_flanged: false,
        cutout_fitted: false,
        turbine: None,
    };

    let cylinders: Vec<crate::audio::dsp::CylinderTap> = (0..4)
        .map(|i| crate::audio::dsp::CylinderTap {
            evo_phase: i as f32 / 4.0,
            bank: 0,
        })
        .collect();
    let snapshot = crate::audio::dsp::EngineSnapshot::default();

    // A handful of fixed probe tones (as `opening_the_cutout_...` above
    // uses) works when the two configurations differ by a bypass, but a
    // change of tailpipe length instead moves a whole comb of harmonics,
    // and a few discrete probe points can land on or off that comb by
    // luck rather than by how open the pipe is. `octave_bands` reads the
    // energy actually captured across each whole band instead, which is
    // what the bench's own `--mode` report uses for the same comparison.
    let high_frequency_share = |exhaust: &ExhaustSystem| -> f64 {
        let mut network = ExhaustNetwork::new(exhaust, &cylinders, 1, FS, &snapshot);
        let mut radiated = [0.0f32; 1];
        let samples = 4 * FS as usize;
        let mut response = Vec::with_capacity(samples);
        for i in 0..samples {
            let pulse = if i % 240 < 6 { 1.0 } else { 0.0 };
            let excitations = [pulse, 0.0, 0.0, 0.0];
            network.step(&excitations, &mut radiated);
            response.push(radiated[0]);
        }
        let bands = crate::analysis::orders::octave_bands(&response, FS as f64);
        // Indices 6..10 of the ISO table are 2, 4, 8 and 16 kHz.
        (bands[6] + bands[7] + bands[8] + bands[9]) / 4.0
    };

    let muffled = high_frequency_share(&exhaust);
    let straight = high_frequency_share(&exhaust.clone().into_straight_pipe());
    let open = high_frequency_share(&exhaust.clone().into_open_headers());

    assert!(
        muffled < straight,
        "muffled ({muffled:.1} dB) should carry less high-frequency share than \
         straight-pipe ({straight:.1} dB)"
    );
    assert!(
        straight < open,
        "straight-pipe ({straight:.1} dB) should carry less high-frequency share than \
         open-headers ({open:.1} dB)"
    );
}

fn turbo_test_geometry() -> crate::physics::plumbing::TurbineGeometry {
    crate::physics::plumbing::TurbineGeometry {
        housing_ar: 0.7,
        blade_count: 9,
    }
}

#[test]
fn a_tighter_housing_reflects_more_and_transmits_less() {
    const FS: f32 = 48_000.0;
    const GAMMA: f32 = 1.35;
    const R: f32 = 287.0;
    const TEMPERATURE: f32 = 900.0;
    let pipe_area = std::f64::consts::PI * 0.022 * 0.022;
    let f = 400.0f32;

    let probe = |housing_ar: f64| -> (f32, f32) {
        let geometry = crate::physics::plumbing::TurbineGeometry {
            housing_ar,
            blade_count: 9,
        };
        let mut turbine = Turbine::new(&geometry, pipe_area, FS, GAMMA, R, TEMPERATURE);
        let settle = 12_000;
        let measure = 12_000;
        let mut reflected = vec![0.0f32; measure];
        let mut transmitted = vec![0.0f32; measure];
        for i in 0..(settle + measure) {
            let phase = std::f32::consts::TAU * f * i as f32 / FS;
            let (r, t) = turbine.step(phase.sin(), 0.0);
            if i >= settle {
                reflected[i - settle] = r;
                transmitted[i - settle] = t;
            }
        }
        (
            magnitude_at(&reflected, f, FS),
            magnitude_at(&transmitted, f, FS),
        )
    };

    let (tight_r, tight_t) = probe(0.35);
    let (open_r, open_t) = probe(1.4);

    assert!(
        tight_r > open_r,
        "a tighter housing should reflect more: tight={tight_r:.4}, open={open_r:.4}"
    );
    assert!(
        tight_t < open_t,
        "a tighter housing should transmit less: tight={tight_t:.4}, open={open_t:.4}"
    );
}

#[test]
fn turbine_attenuates_high_orders_more_than_low_ones() {
    const FS: f32 = 48_000.0;
    const GAMMA: f32 = 1.35;
    const R: f32 = 287.0;
    const TEMPERATURE: f32 = 900.0;
    let pipe_area = std::f64::consts::PI * 0.022 * 0.022;
    let geometry = turbo_test_geometry();

    let transmission_loss_db = |f: f32| -> f32 {
        let mut turbine = Turbine::new(&geometry, pipe_area, FS, GAMMA, R, TEMPERATURE);
        let period = FS / f;
        let settle = (period * 20.0).round() as usize;
        let measure = (period * 40.0).round() as usize;
        let mut transmitted = vec![0.0f32; measure];
        for i in 0..(settle + measure) {
            let phase = std::f32::consts::TAU * f * i as f32 / FS;
            let (_, out) = turbine.step(phase.sin(), 0.0);
            if i >= settle {
                transmitted[i - settle] = out;
            }
        }
        let amplitude = magnitude_at(&transmitted, f, FS);
        -20.0 * amplitude.max(1e-9).log10()
    };

    let low_loss = transmission_loss_db(150.0);
    let high_loss = transmission_loss_db(4_000.0);

    assert!(
        high_loss > low_loss,
        "a turbine should attenuate high orders harder than low ones: \
         150 Hz loss {low_loss:.2} dB, 4,000 Hz loss {high_loss:.2} dB"
    );
}

#[test]
fn turbine_radiates_less_energy_than_no_turbine() {
    // Same shape of claim as `expansion_chamber_radiates_less_energy_than_no_silencer`:
    // fitting the element must not make the engine louder overall.
    use crate::physics::plumbing::{
        Collector, Crossover, ExhaustSystem, PipeSection, Silencer, TurbineGeometry,
    };

    const FS: f32 = 48_000.0;

    let exhaust = |turbine: Option<TurbineGeometry>| ExhaustSystem {
        primaries: vec![PipeSection::from_diameter(0.45, 0.040, 850.0); 4],
        collector: Collector::from_diameter(4, 0.060, 0.15),
        secondary: vec![],
        crossover: Crossover::None,
        silencers: vec![Silencer::Straight],
        tailpipe: PipeSection::from_diameter(1.0, 0.060, 600.0),
        tailpipe_flanged: false,
        cutout_fitted: false,
        turbine,
    };

    let cylinders: Vec<crate::audio::dsp::CylinderTap> = (0..4)
        .map(|i| crate::audio::dsp::CylinderTap {
            evo_phase: i as f32 / 4.0,
            bank: 0,
        })
        .collect();
    let snapshot = crate::audio::dsp::EngineSnapshot::default();

    let total_radiated_energy = |turbine: Option<TurbineGeometry>| -> f64 {
        let system = exhaust(turbine);
        let mut network = ExhaustNetwork::new(&system, &cylinders, 1, FS, &snapshot);
        let mut radiated = [0.0f32; 1];
        let mut energy = 0.0f64;
        for i in 0..(4 * FS as usize) {
            let pulse = if i % 240 < 6 { 1.0 } else { 0.0 };
            let excitations = [pulse, 0.0, 0.0, 0.0];
            network.step(&excitations, &mut radiated);
            energy += (radiated[0] as f64).powi(2);
        }
        energy
    };

    let fitted = total_radiated_energy(Some(turbo_test_geometry()));
    let bare = total_radiated_energy(None);

    assert!(
        fitted < bare,
        "a turbine must radiate less total energy than no turbine at all: \
         fitted={fitted:.6}, bare={bare:.6}"
    );
}

#[test]
fn manifold_upstream_of_the_turbine_gets_its_own_resonance() {
    // Before a turbine is fitted the primary/collector run continues into
    // whatever is downstream with no boundary of its own there; the
    // plan's claim is that fitting a turbine gives that run a boundary it
    // did not have, turning it into its own quarter-wave resonator at a
    // frequency the system had no reason to ring at before. Driven
    // continuously at that candidate frequency, a real boundary builds a
    // standing wave at the closed end that a matched, boundary-free run
    // never does — the same steady-state methodology
    // `chamber_transmission_loss_matches_theory` uses, rather than an
    // impulse ring-down, because the turbine's reflection here is only
    // partial and the ring dies out too fast for a long Goertzel window
    // to see it.
    const FS: f32 = 48_000.0;
    const GAMMA: f32 = 1.35;
    const R: f32 = 287.0;
    const TEMPERATURE: f32 = 900.0;
    let c = speed_of_sound(GAMMA, R, TEMPERATURE);

    let manifold_length = 0.55f64;
    let pipe_area = std::f64::consts::PI * 0.022 * 0.022;
    let geometry = turbo_test_geometry();
    let expected = c / (4.0 * manifold_length as f32);

    let standing_wave_at = |fitted: bool, f: f32| -> f32 {
        let mut manifold =
            WaveguidePipe::new(manifold_length, pipe_area, FS, GAMMA, R, TEMPERATURE);
        manifold.tune(GAMMA, R, TEMPERATURE);
        manifold.snap_delays();
        let mut turbine = Turbine::new(&geometry, pipe_area, FS, GAMMA, R, TEMPERATURE);

        let settle = 24_000;
        let measure = 24_000;
        let mut closed_end = vec![0.0f32; measure];
        for i in 0..(settle + measure) {
            let (p_closed, p_open) = manifold.read_outputs();
            let excitation = std::f32::consts::TAU * f * i as f32 / FS;
            let excitation = excitation.sin();

            let p_reflected = if fitted {
                turbine.step(p_open, 0.0).0
            } else {
                // No boundary at all here: the run continues, matched to
                // the manifold's own impedance, so nothing comes back.
                0.0
            };
            manifold.push_inputs(excitation + p_closed, p_reflected);
            if i >= settle {
                closed_end[i - settle] = p_closed;
            }
        }
        magnitude_at(&closed_end, f, FS)
    };

    let fitted_peak = standing_wave_at(true, expected);
    let unfitted_peak = standing_wave_at(false, expected);

    assert!(
        fitted_peak > 3.0 * unfitted_peak.max(1e-6),
        "fitting a turbine should give the manifold its own resonance near \
         {expected:.0} Hz -- present with the turbine fitted, absent without it: \
         fitted peak {fitted_peak:.4}, unfitted {unfitted_peak:.4}"
    );
}

#[test]
fn a_step_edge_disperses_rather_than_only_quietening() {
    // Isolates dispersion from the wheel's dissipation and the nozzle's
    // reflection: the same allpass cascade `Turbine` runs internally,
    // driven directly by a step edge.
    const FS: f32 = 48_000.0;
    let wheel_radius = 0.014f32;
    let c = 580.0f32;
    let breaks = turbine_dispersion_breaks_hz(wheel_radius, c);
    let mut stages: Vec<Allpass> = breaks.iter().map(|&hz| Allpass::new(FS, hz)).collect();

    let n = 4_800;
    let edge = n / 2;
    let mut input = vec![0.0f32; n];
    let mut output = vec![0.0f32; n];
    for i in 0..n {
        let x = if i >= edge { 1.0 } else { 0.0 };
        input[i] = x;
        output[i] = stages.iter_mut().fold(x, |acc, s| s.process(acc));
    }

    // Last sample from the edge onward that is still more than 10 % away
    // from the final value: how long the edge takes to settle and stay
    // settled, robust to an overshoot flipping sign on the way there.
    let settle_time = |signal: &[f32]| -> usize {
        let mut last_unsettled = edge;
        for (i, &y) in signal.iter().enumerate().skip(edge) {
            if (y - 1.0).abs() > 0.1 {
                last_unsettled = i;
            }
        }
        last_unsettled - edge
    };

    let input_settle = settle_time(&input);
    let output_settle = settle_time(&output);
    assert!(
        output_settle > input_settle,
        "dispersion should lengthen the edge's settling time: input {input_settle} \
         samples, output {output_settle} samples"
    );

    let energy = |signal: &[f32]| -> f64 { signal.iter().map(|&x| (x as f64).powi(2)).sum() };
    let input_energy = energy(&input);
    let output_energy = energy(&output);
    assert!(
        (output_energy - input_energy).abs() / input_energy < 0.02,
        "dispersion must preserve total energy: input {input_energy:.3}, \
         output {output_energy:.3}"
    );
}

#[test]
fn blade_pass_content_tracks_shaft_speed() {
    const FS: f32 = 48_000.0;
    const GAMMA: f32 = 1.35;
    const R: f32 = 287.0;
    const TEMPERATURE: f32 = 900.0;
    let pipe_area = std::f64::consts::PI * 0.022 * 0.022;
    let geometry = turbo_test_geometry();

    let probe = |rpm: f32| -> f32 {
        let mut turbine = Turbine::new(&geometry, pipe_area, FS, GAMMA, R, TEMPERATURE);
        turbine.set_flow(0.15);
        turbine.set_shaft_rpm(rpm);
        let n = 48_000;
        let mut transmitted = vec![0.0f32; n];
        for slot in transmitted.iter_mut() {
            *slot = turbine.step(0.0, 0.0).1;
        }
        let expected_hz = geometry.blade_count as f32 * rpm / 60.0;
        magnitude_at(&transmitted, expected_hz, FS)
    };

    let low = probe(60_000.0);
    let high = probe(120_000.0);
    assert!(
        low > 1e-4,
        "blade pass should be audible at 60,000 rpm, got {low:.6}"
    );
    assert!(
        high > 1e-4,
        "blade pass should be audible at 120,000 rpm, got {high:.6}"
    );

    // Silent with no flow, the same way a compressor moving no air is.
    let mut silent_turbine = Turbine::new(&geometry, pipe_area, FS, GAMMA, R, TEMPERATURE);
    silent_turbine.set_shaft_rpm(90_000.0);
    let hz = geometry.blade_count as f32 * 90_000.0 / 60.0;
    let n = 4_800;
    let mut silent = vec![0.0f32; n];
    for slot in silent.iter_mut() {
        *slot = silent_turbine.step(0.0, 0.0).1;
    }
    let mag = magnitude_at(&silent, hz, FS);
    assert!(
        mag < 1e-6,
        "blade pass with no flow should be silent, got {mag:.8}"
    );
}

#[test]
fn tailpipe_injection_reaches_the_mouth_before_port_injection() {
    // The whole claim Stage M3 of docs/MECHANISM_PLAN.md rests on: an
    // event injected close to the mouth radiates sooner, and by less pipe,
    // than the same event injected at the port. No new physics is needed
    // for this -- the network already has one-way delay accessors for
    // every leg the port injection has to cross that the tailpipe
    // injection does not.
    use crate::physics::plumbing::{Collector, Crossover, ExhaustSystem, PipeSection};

    const FS: f32 = 48_000.0;
    let exhaust = ExhaustSystem {
        primaries: vec![PipeSection::from_diameter(0.60, 0.040, 850.0)],
        collector: Collector::from_diameter(1, 0.045, 0.12),
        secondary: vec![],
        crossover: Crossover::None,
        silencers: vec![],
        tailpipe: PipeSection::from_diameter(0.50, 0.045, 600.0),
        tailpipe_flanged: false,
        cutout_fitted: false,
        turbine: None,
    };
    let cylinders = vec![crate::audio::dsp::CylinderTap {
        evo_phase: 0.0,
        bank: 0,
    }];
    let snapshot = crate::audio::dsp::EngineSnapshot::default();

    // First sample at which the mouth's output rises clear of numerical
    // silence, starting from a network with nothing else exciting it.
    let onset = |node: InjectionNode| -> usize {
        let mut network = ExhaustNetwork::new(&exhaust, &cylinders, 1, FS, &snapshot);
        let excitations = [0.0f32];
        let mut radiated = [0.0f32; 1];
        network.inject(node, 0, 1.0);
        for i in 0..4_000 {
            network.step(&excitations, &mut radiated);
            if radiated[0].abs() > 1e-5 {
                return i;
            }
        }
        panic!("{node:?} injection never reached the mouth");
    };

    let port_onset = onset(InjectionNode::Port);
    let tailpipe_onset = onset(InjectionNode::Tailpipe);
    assert!(
        tailpipe_onset < port_onset,
        "tailpipe injection ({tailpipe_onset} samples) should reach the \
         mouth before port injection ({port_onset} samples)"
    );

    let network = ExhaustNetwork::new(&exhaust, &cylinders, 1, FS, &snapshot);
    let expected_extra_delay = network.primary_delay_samples(0)
        + network.collector_delay_samples(0)
        + network.tailpipe_delay_samples(0);
    let measured_extra_delay = (port_onset - tailpipe_onset) as f32;
    let tolerance = 0.15 * expected_extra_delay + 4.0;
    assert!(
        (measured_extra_delay - expected_extra_delay).abs() < tolerance,
        "measured transit-time difference ({measured_extra_delay} samples) \
         should match the network's own one-way delays for the primary, \
         collector taper and tailpipe it crosses ({expected_extra_delay} \
         samples), within {tolerance}"
    );
}
