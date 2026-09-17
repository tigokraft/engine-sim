use super::*;

fn approx(a: f32, b: f32, tol: f32) {
    assert!((a - b).abs() <= tol, "expected {b}, got {a} (tol {tol})");
}

/// Runs an impulse through a filter and returns the peak magnitude of the
/// response at `frequency`, measured by quadrature correlation.
fn magnitude_at(coeffs: BiquadCoeffs, sample_rate: f32, frequency: f32) -> f32 {
    let mut filter = Biquad::new(coeffs);
    let n = 8192;
    // Discard the transient, then correlate the steady state.
    let (mut re, mut im) = (0.0f32, 0.0f32);
    for i in 0..n {
        let t = i as f32 / sample_rate;
        let y = filter.process((TAU * frequency * t).sin());
        if i >= n / 2 {
            let phase = TAU * frequency * t;
            re += y * phase.sin();
            im += y * phase.cos();
        }
    }
    let half = (n / 2) as f32;
    2.0 * (re * re + im * im).sqrt() / half
}

#[test]
fn bandpass_peaks_at_unity_on_centre() {
    let fs = 48_000.0;
    let coeffs = BiquadCoeffs::bandpass(fs, 120.0, 2.0);
    approx(magnitude_at(coeffs, fs, 120.0), 1.0, 0.02);
    // And rolls off either side.
    assert!(magnitude_at(coeffs, fs, 20.0) < 0.3);
    assert!(magnitude_at(coeffs, fs, 1200.0) < 0.3);
}

#[test]
fn bandpass_gain_is_independent_of_q() {
    let fs = 48_000.0;
    for q in [0.5, 2.0, 8.0] {
        let coeffs = BiquadCoeffs::bandpass(fs, 200.0, q);
        approx(magnitude_at(coeffs, fs, 200.0), 1.0, 0.03);
    }
}

#[test]
fn biquad_stays_stable_above_nyquist_request() {
    let mut filter = Biquad::new(BiquadCoeffs::bandpass(48_000.0, 96_000.0, 20.0));
    let mut noise = Noise::default();
    for _ in 0..48_000 {
        let y = filter.process(noise.next_bipolar());
        assert!(y.is_finite() && y.abs() < 100.0, "unstable: {y}");
    }
}

#[test]
fn delay_line_reads_back_what_went_in() {
    let mut line = DelayLine::with_max_delay(64);
    for i in 0..32 {
        line.push(i as f32);
    }
    // Delay of 1 is the most recent sample.
    approx(line.read(1.0), 31.0, 1e-6);
    approx(line.read_linear(8.0), 24.0, 1e-6);
    // Halfway between samples 24 and 23 with linear interpolation.
    approx(line.read_linear(8.5), 23.5, 1e-6);
}

#[test]
fn lagrange_interpolation_is_exact_on_a_cubic() {
    // A 4-point Lagrange read reproduces any polynomial of degree 3 or less
    // exactly, which is the property a jumping reader needs: it can land
    // anywhere between samples without a state that assumes it crept there.
    let mut line = DelayLine::with_max_delay(64);
    let curve = |x: f64| 0.5 * x * x * x - 3.0 * x * x + 2.0 * x + 7.0;
    for i in 0..48 {
        line.push(curve(i as f64) as f32);
    }
    // Delay d reads the sample pushed at index 47 - (d - 1).
    for &d in &[8.0f32, 8.25, 8.5, 12.75, 20.5] {
        let want = curve(48.0 - d as f64) as f32;
        approx(line.read_lagrange3(d), want, 1e-2);
    }
    // Integer delays land on the stored sample itself.
    approx(line.read_lagrange3(10.0), curve(38.0) as f32, 1e-3);
}

#[test]
fn thiran_allpass_glide_has_no_measurable_amplitude_modulation() {
    // Linear interpolation in a delay line acts as a lowpass filter whose
    // cutoff moves with the fractional delay: at frac = 0.5 and high frequencies,
    // linear interpolation causes amplitude droop, producing an audible
    // "breathing" amplitude modulation as delay glides.
    // A 1st-order Thiran allpass filter has |H(e^jw)| = 1.0 identically at
    // all frequencies, eliminating amplitude modulation during glides.
    const FS: f32 = 48_000.0;
    const FREQ: f32 = 6_000.0; // 6 kHz test tone
    let mut line = DelayLine::with_max_delay(128);

    // Run a tone while smoothly gliding delay from 16.0 to 17.0 over 2000 samples
    let n_samples = 2000;
    for i in 0..n_samples {
        let t = i as f32 / FS;
        let input = (std::f32::consts::TAU * FREQ * t).sin();
        line.push(input);

        let delay = 16.0 + (i as f32 / n_samples as f32);
        let _ = line.read(delay);
    }

    // Measure steady-state Fourier magnitude at frac ~ 0.5
    let mut re = 0.0f64;
    let mut im = 0.0f64;
    let n_eval = 480; // 60 cycles of 6 kHz
    for i in 0..n_eval {
        let t = (2000 + i) as f32 / FS;
        line.push((std::f32::consts::TAU * FREQ * t).sin());
        let out = line.read(16.5);
        let phase = std::f32::consts::TAU * FREQ * i as f32 / FS;
        re += out as f64 * phase.sin() as f64;
        im += out as f64 * phase.cos() as f64;
    }
    let mag = (2.0 * (re * re + im * im).sqrt() / n_eval as f64) as f32;

    // With Thiran allpass, magnitude response is unity (|H| = 1.0) at all fractional delays
    assert!(
        (mag - 1.0).abs() < 0.005,
        "Thiran allpass at frac 0.5 must have unity gain: mag was {mag:.4}"
    );
}

#[test]
fn delay_line_clamps_instead_of_reading_the_future() {
    let mut line = DelayLine::with_max_delay(16);
    for _ in 0..64 {
        line.push(1.0);
    }
    assert!(line.read(0.0).is_finite());
    assert!(line.read(1e9).is_finite());
}

#[test]
fn speed_of_sound_matches_hand_calculation() {
    // Air at 293 K: sqrt(1.4 * 287 * 293) = 343.1 m/s.
    approx(speed_of_sound(1.4, 287.0, 293.0), 343.1, 1.0);
    // Exhaust at 900 K is much faster, which is why hot pipes ring higher.
    assert!(speed_of_sound(1.33, 287.0, 900.0) > 580.0);
}

#[test]
fn runner_delay_shortens_as_gas_heats() {
    let cold = runner_delay_seconds(0.45, 1.33, 287.0, 400.0);
    let hot = runner_delay_seconds(0.45, 1.33, 287.0, 1100.0);
    assert!(hot < cold, "hot gas must transit faster");
    // 0.45 m at 400 K: c = 390 m/s, tau = 1.15 ms.
    approx(cold, 0.45 / 390.6, 5e-5);
}

#[test]
fn helmholtz_matches_closed_form() {
    let (gamma, r, t) = (1.33f32, 287.0f32, 900.0f32);
    let (area, volume, neck) = (2.0e-3f32, 8.0e-3f32, 0.10f32);
    let c = speed_of_sound(gamma, r, t);
    let expected = (c / TAU) * (area / (volume * neck)).sqrt();
    approx(
        helmholtz_frequency(gamma, r, t, area, volume, neck),
        expected,
        1e-3,
    );
    // Sanity: a road muffler sits in the low hundreds of Hz.
    assert!((50.0..600.0).contains(&expected), "unphysical: {expected}");
}

#[test]
fn runner_loop_is_stable_under_delay_modulation() {
    let mut runner = ExhaustRunner::new(48_000.0, 0.5, 0.019, 0.9, 1.33, 287.0, 900.0);
    let mut noise = Noise::default();
    for i in 0..96_000 {
        // Sweep the temperature across the whole plausible range while
        // driving it hard.
        if i % 64 == 0 {
            let t = 400.0 + 700.0 * ((i as f32 / 4800.0).sin() * 0.5 + 0.5);
            runner.tune(1.33, 287.0, t);
        }
        let y = runner.process(noise.next_bipolar());
        assert!(y.is_finite() && y.abs() < 50.0, "runner blew up: {y}");
    }
}

#[test]
fn smoothed_glides_without_overshoot() {
    let mut p = Smoothed::new(0.0, 48_000.0, 0.010);
    p.set_target(1.0);
    let mut previous = 0.0;
    for _ in 0..4_800 {
        let v = p.next_value();
        assert!(v >= previous - 1e-9 && v <= 1.0, "overshoot: {v}");
        previous = v;
    }
    // One time constant is 63 %; 100 ms is ten of them.
    approx(p.value(), 1.0, 1e-3);
}

#[test]
fn smoothed_ignores_non_finite_targets() {
    let mut p = Smoothed::new(0.5, 48_000.0, 0.01);
    p.set_target(f32::NAN);
    p.set_target(f32::INFINITY);
    for _ in 0..1000 {
        assert!(p.next_value().is_finite());
    }
}

#[test]
fn dc_blocker_removes_offset_but_keeps_signal() {
    let mut blocker = DcBlocker::default();
    let mut last = 0.0;
    for i in 0..48_000 {
        last = blocker.process(1.0 + 0.5 * (TAU * 200.0 * i as f32 / 48_000.0).sin());
    }
    assert!(last.abs() < 0.55, "offset survived: {last}");
    // The 200 Hz component must still be there.
    let mut peak: f32 = 0.0;
    for i in 48_000..49_000 {
        let y = blocker.process(1.0 + 0.5 * (TAU * 200.0 * i as f32 / 48_000.0).sin());
        peak = peak.max(y.abs());
    }
    approx(peak, 0.5, 0.05);
}

#[test]
fn soft_clip_is_monotone_and_bounded() {
    let mut previous = f32::NEG_INFINITY;
    let mut x = -20.0;
    while x <= 20.0 {
        let y = soft_clip(x);
        assert!(y.abs() <= 1.05, "unbounded at {x}: {y}");
        assert!(y >= previous - 1e-6, "non-monotone at {x}");
        previous = y;
        x += 0.01;
    }
    // Near-unity gain in the linear region, so quiet signals pass clean.
    approx(soft_clip(0.1), 0.1, 2e-3);
    approx(soft_clip(0.0), 0.0, 1e-9);
}

#[test]
fn noise_is_zero_mean_and_bounded() {
    let mut noise = Noise::new(12345);
    let mut sum = 0.0f64;
    for _ in 0..200_000 {
        let v = noise.next_bipolar();
        assert!((-1.0..1.0).contains(&v));
        sum += v as f64;
    }
    assert!(
        (sum / 200_000.0).abs() < 0.01,
        "biased: {}",
        sum / 200_000.0
    );
}

#[test]
fn gaussian_draws_are_standard_normal_and_bounded() {
    let mut noise = Noise::new(4242);
    let n = 400_000;
    let (mut sum, mut sum_sq) = (0.0f64, 0.0f64);
    let mut extreme = 0.0f32;
    for _ in 0..n {
        let v = noise.next_gaussian();
        sum += v as f64;
        sum_sq += (v as f64) * (v as f64);
        extreme = extreme.max(v.abs());
    }
    let mean = sum / n as f64;
    let variance = sum_sq / n as f64 - mean * mean;
    assert!(mean.abs() < 0.01, "biased: {mean}");
    assert!((variance - 1.0).abs() < 0.02, "variance {variance}, want 1");
    // Irwin-Hall with four terms cannot leave +/-sqrt(12).
    assert!(extreme <= 12.0f32.sqrt(), "left its support: {extreme}");
    assert!(extreme > 2.5, "suspiciously narrow: {extreme}");
}

#[test]
fn firing_interval_follows_the_four_stroke_law() {
    // A V8 bank of four at 800 rpm: a 150 ms cycle shared by four cylinders.
    approx(firing_interval_seconds(800.0, 4), 0.0375, 1e-6);
    // Twice the speed, half the interval.
    approx(firing_interval_seconds(1_600.0, 4), 0.01875, 1e-6);
    // A stopped engine never fires again.
    assert_eq!(firing_interval_seconds(0.0, 4), f32::MAX);
}

#[test]
fn damping_shortens_the_runner_ringdown() {
    // Kick the loop once and measure how long it takes to fall 40 dB.
    let ringdown = |damping: f32| {
        let mut runner = ExhaustRunner::new(48_000.0, 0.45, 0.019, 0.85, 1.33, 287.0, 900.0);
        runner.set_damping(damping);
        // Let the glided reflection reach its target before the kick.
        for _ in 0..12_000 {
            runner.process(0.0);
        }
        let mut peak = runner.process(1.0).abs();
        let mut last_loud = 0usize;
        for i in 1..48_000 {
            let y = runner.process(0.0).abs();
            peak = peak.max(y);
            if y > peak * 0.01 {
                last_loud = i;
            }
        }
        last_loud
    };

    let open = ringdown(0.0);
    let damped = ringdown(1.0);
    assert!(open > 0, "the undamped pipe did not ring at all");
    assert!(
        damped * 3 < open,
        "damping barely shortened the ringdown: {damped} vs {open} samples"
    );

    // The damped loop must still die inside one idle firing interval —
    // that is the whole point of the rule.
    let idle_interval = (firing_interval_seconds(800.0, 4) * 48_000.0) as usize;
    assert!(
        damped < idle_interval,
        "still ringing when the next pulse lands: {damped} vs {idle_interval}"
    );
}

/// Peak-to-trough ripple of a runner's magnitude response over the band
/// where the comb lives [dB].
///
/// This is a direct measurement of the flanging artefact: a comb filter is
/// exactly a response with deep regular ripple, and how deep it is, is how
/// audible the flange is.
fn comb_ripple_db(damping: f32) -> f32 {
    let fs = 48_000.0;
    let (mut lo, mut hi) = (f32::MAX, 0.0f32);
    let mut f = 200.0f32;
    while f <= 2_000.0 {
        let mut runner = ExhaustRunner::new(fs, 0.45, 0.019, 0.55, 1.33, 287.0, 700.0);
        runner.set_damping(damping);
        // Settle the glided reflection, then the tone's own transient.
        for _ in 0..12_000 {
            runner.process(0.0);
        }
        let (mut re, mut im) = (0.0f64, 0.0f64);
        for i in 0..12_000 {
            let phase = TAU * f * i as f32 / fs;
            let y = runner.process(phase.sin());
            if i >= 6_000 {
                re += y as f64 * phase.sin() as f64;
                im += y as f64 * phase.cos() as f64;
            }
        }
        let m = (2.0 * (re * re + im * im).sqrt() / 6_000.0) as f32;
        lo = lo.min(m);
        hi = hi.max(m);
        f += 10.0;
    }
    20.0 * (hi / lo.max(1e-9)).log10()
}

#[test]
fn damping_flattens_the_comb() {
    // The artefact, measured: an undamped runner has a deep regular ripple
    // across the whole midrange, which on broadband excitation is heard as
    // flanging. Damping it is what the Transit-Time Decision Rule buys.
    let open = comb_ripple_db(0.0);
    let damped = comb_ripple_db(1.0);
    assert!(
        open > 8.0,
        "the undamped pipe barely combs at all: {open:.1} dB"
    );
    assert!(
        damped < 0.45 * open,
        "damping left the comb standing: {damped:.1} dB vs {open:.1} dB"
    );
    // But it must not flatten it away entirely — the pipe still has to be a
    // pipe, or the exhaust loses its character along with its flange.
    assert!(damped > 1.0, "damped into a plain delay: {damped:.1} dB");
}

#[test]
fn damping_lowers_reflection_and_darkens_the_return_path() {
    let mut runner = ExhaustRunner::new(48_000.0, 0.45, 0.019, 0.80, 1.33, 287.0, 900.0);
    let nominal = runner.reflection();
    approx(nominal, 0.80, 1e-6);
    assert_eq!(runner.damping(), 0.0);

    runner.set_damping(1.0);
    assert_eq!(runner.damping(), 1.0);
    // Glided, not stepped: it has not arrived yet.
    assert!(runner.reflection() > 0.7, "reflection jumped");
    for _ in 0..48_000 {
        runner.process(0.0);
    }
    approx(runner.reflection(), 0.80 * RUNNER_DAMPED_REFLECTION, 1e-3);

    // Never unstable, at any damping, under a hard drive.
    let mut noise = Noise::new(11);
    for step in 0..24_000 {
        if step % 64 == 0 {
            runner.set_damping((step as f32 / 24_000.0).sin().abs());
        }
        let y = runner.process(noise.next_bipolar());
        assert!(y.is_finite() && y.abs() < 50.0, "blew up: {y}");
    }
}

#[test]
fn block_resonance_falls_with_mass() {
    // The reference block sits on the reference frequency by definition.
    approx(
        block_resonance_hz(BLOCK_REFERENCE_MASS),
        BLOCK_REFERENCE_HZ,
        1e-3,
    );
    // f ~ 1/sqrt(m): four times the mass is half the frequency, before the
    // clamp catches it.
    let heavy = block_resonance_hz(BLOCK_REFERENCE_MASS * 4.0);
    assert_eq!(heavy, BLOCK_MIN_HZ, "should have clamped to the floor");
    approx(
        block_resonance_hz(BLOCK_REFERENCE_MASS * 1.4),
        BLOCK_REFERENCE_HZ / 1.4f32.sqrt(),
        0.5,
    );
    // A light alloy four rings high, a big iron V8 rings low, and both stay
    // inside the band the model claims.
    for mass in [40.0, 90.0, 180.0, 260.0, 400.0] {
        let f = block_resonance_hz(mass);
        assert!(
            (BLOCK_MIN_HZ..=BLOCK_MAX_HZ).contains(&f),
            "{mass} kg -> {f} Hz"
        );
    }
    assert!(block_resonance_hz(90.0) > block_resonance_hz(260.0));
    // Degenerate input must not divide by zero.
    assert!(block_resonance_hz(0.0).is_finite());
}

#[test]
fn muffler_tracks_temperature() {
    let mut muffler = Muffler::new(
        48_000.0,
        MufflerGeometry::default(),
        0.030,
        false,
        1.33,
        287.0,
        500.0,
    );
    let cold = muffler.centre_frequency();
    muffler.tune(1.33, 287.0, 1100.0);
    let hot = muffler.centre_frequency();
    // f ~ c ~ sqrt(T), so a 2.2x temperature ratio is a 1.48x pitch ratio.
    approx(hot / cold, (1100.0f32 / 500.0).sqrt(), 0.01);
}

#[test]
fn modal_bank_resonates_at_designed_modes() {
    let fs = 48_000.0;
    let bank = ModalBank::dual(fs, (1000.0, 4.0, 0.7), (3000.0, 4.0, 0.3));
    assert_eq!(bank.mode_count(), 2);

    // Correlate response to pure tones at mode 1, mode 2, and an off-resonance frequency.
    let response_at = |freq: f32| {
        let mut b = bank;
        let n = 8192;
        let (mut re, mut im) = (0.0f32, 0.0f32);
        for i in 0..n {
            let t = i as f32 / fs;
            let y = b.process((TAU * freq * t).sin());
            if i >= n / 2 {
                let phase = TAU * freq * t;
                re += y * phase.sin();
                im += y * phase.cos();
            }
        }
        let half = (n / 2) as f32;
        2.0 * (re * re + im * im).sqrt() / half
    };

    let resp_1k = response_at(1000.0);
    let resp_3k = response_at(3000.0);
    let resp_off = response_at(200.0);

    // Peak response near the individual mode weights.
    approx(resp_1k, 0.7, 0.05);
    approx(resp_3k, 0.3, 0.05);
    assert!(
        resp_off < 0.15,
        "off-resonance response too high: {resp_off}"
    );
}

#[test]
fn no_aliasing_products_above_the_excitations_bandlimit_after_oversampling() {
    // An excitation bandlimited to 10 kHz fed into a soft clipper generates
    // odd harmonics (30 kHz, 50 kHz, ...).
    // In a discrete-time system at 48 kHz without oversampling, the 3rd harmonic
    // at 30 kHz aliases across the 24 kHz Nyquist boundary down to 18 kHz,
    // appearing as a prominent spurious tone above the 10 kHz excitation bandlimit.
    // With 2x oversampling, the 30 kHz harmonic is created below the 96 kHz Nyquist
    // (48 kHz) and attenuated by the decimation half-band filter before downsampling,
    // leaving no measurable aliasing products above the excitation bandlimit.
    const FS: f32 = 48_000.0;
    const FREQ_IN: f32 = 10_000.0;
    const N: usize = 8192;
    let amp = 1.5; // Drives clipper into saturation

    // 1. Without oversampling: measure aliased 3rd harmonic at 18 kHz
    let mut re_no_os = 0.0f64;
    let mut im_no_os = 0.0f64;
    for i in 0..N {
        let t = i as f32 / FS;
        let inp = amp * (TAU * FREQ_IN * t).sin();
        let out = soft_clip(inp);
        if i >= N / 2 {
            let phase = TAU * 18_000.0 * (i as f32 / FS);
            re_no_os += out as f64 * phase.sin() as f64;
            im_no_os += out as f64 * phase.cos() as f64;
        }
    }
    let half = (N / 2) as f64;
    let mag_no_os = 2.0 * (re_no_os * re_no_os + im_no_os * im_no_os).sqrt() / half;

    // 2. With 2x oversampling:
    let mut clipper = OversampledClipper::new();
    let mut re_os = 0.0f64;
    let mut im_os = 0.0f64;
    for i in 0..N {
        let t = i as f32 / FS;
        let inp = amp * (TAU * FREQ_IN * t).sin();
        let out = clipper.process(inp);
        if i >= N / 2 {
            let phase = TAU * 18_000.0 * (i as f32 / FS);
            re_os += out as f64 * phase.sin() as f64;
            im_os += out as f64 * phase.cos() as f64;
        }
    }
    let mag_os = 2.0 * (re_os * re_os + im_os * im_os).sqrt() / half;

    // Without oversampling, 18 kHz alias is prominent (> -20 dBFS, ~0.12)
    assert!(
        mag_no_os > 0.10,
        "Expected strong aliasing at 18 kHz without oversampling, got {mag_no_os}"
    );
    // With 2x oversampling, aliasing product above excitation bandlimit is suppressed below 0.01 (-40 dBFS)
    assert!(
        mag_os < 0.01,
        "Expected aliasing at 18 kHz to be eliminated/suppressed by oversampler, got {mag_os}"
    );
    assert!(
        mag_os < 0.1 * mag_no_os,
        "Oversampling must suppress aliasing by at least 20 dB: got {mag_os} vs {mag_no_os}"
    );
}

#[test]
fn allpass_preserves_magnitude_at_every_frequency() {
    // The whole point of reaching for an allpass instead of a lowpass to
    // disperse an edge: it must not touch the spectrum, only the timing.
    const FS: f32 = 48_000.0;
    for break_hz in [100.0f32, 800.0, 4_000.0] {
        for f in [50.0f32, 300.0, 1_000.0, 5_000.0, 12_000.0] {
            let mut stage = Allpass::new(FS, break_hz);
            // An integer number of cycles in the measurement window, so
            // the correlation does not leak across bins at a low `f`.
            let period = FS / f;
            let settle = (period * 20.0).round() as usize;
            let measure = (period * 40.0).round() as usize;
            let (mut re, mut im) = (0.0f64, 0.0f64);
            for i in 0..(settle + measure) {
                let phase = TAU * f * i as f32 / FS;
                let y = stage.process(phase.sin());
                if i >= settle {
                    re += y as f64 * phase.sin() as f64;
                    im += y as f64 * phase.cos() as f64;
                }
            }
            let mag = 2.0 * (re * re + im * im).sqrt() / measure as f64;
            assert!(
                (mag - 1.0).abs() < 0.02,
                "allpass at break {break_hz} Hz should pass {f} Hz at unity, measured {mag:.4}"
            );
        }
    }
}
