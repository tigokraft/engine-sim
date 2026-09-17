use super::*;

const RATE: f64 = 48_000.0;

/// A steady sinusoid of a given amplitude and frequency.
fn tone(hz: f64, amplitude: f64, seconds: f64) -> Vec<f32> {
    let n = (seconds * RATE) as usize;
    (0..n)
        .map(|i| (amplitude * (2.0 * PI * hz * i as f64 / RATE).sin()) as f32)
        .collect()
}

/// A fixed draw of white noise, so a test of it is the same every run.
fn noise(seconds: f64) -> Vec<f32> {
    let mut state = 0x2545_F491u32;
    (0..(seconds * RATE) as usize)
        .map(|_| {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (state >> 8) as f32 / (1u32 << 23) as f32 - 1.0
        })
        .collect()
}

/// A fixed draw of pink noise: equal power per octave, `1/f`.
///
/// Paul Kellet's economy filter — a bank of six one-pole stages on white
/// noise, chosen so their corners overlap and the sum approximates a
/// `1/sqrt(f)` amplitude response across the whole audible band to within
/// half a decibel. That is what an octave-band test needs: real pink
/// noise, not a single pole's `1/f^2` power slope.
fn pink_noise(seconds: f64) -> Vec<f32> {
    let white = noise(seconds);
    let mut b = [0.0f32; 7];
    white
        .into_iter()
        .map(|white| {
            b[0] = 0.99886 * b[0] + white * 0.0555179;
            b[1] = 0.99332 * b[1] + white * 0.0750759;
            b[2] = 0.96900 * b[2] + white * 0.153_852;
            b[3] = 0.86650 * b[3] + white * 0.3104856;
            b[4] = 0.55000 * b[4] + white * 0.5329522;
            b[5] = -0.7616 * b[5] - white * 0.0168980;
            let pink = b[0] + b[1] + b[2] + b[3] + b[4] + b[5] + b[6] + white * 0.5362;
            b[6] = white * 0.115926;
            pink * 0.11
        })
        .collect()
}

/// A tone that tracks `order` of an engine sweeping `from` to `to`.
///
/// The phase is the running integral of the instantaneous frequency, which
/// is what makes this a chirp on that order rather than a tone whose
/// frequency is stepped.
fn swept_order(order: f64, from: f64, to: f64, amplitude: f64, seconds: f64) -> Vec<f32> {
    let n = (seconds * RATE) as usize;
    let mut phase = 0.0f64;
    (0..n)
        .map(|i| {
            let rpm = from + (to - from) * (i as f64 / n as f64);
            let sample = amplitude * phase.sin();
            phase += 2.0 * PI * order * rpm / 60.0 / RATE;
            sample as f32
        })
        .collect()
}

#[test]
fn a_full_scale_sine_reads_zero_dbfs() {
    let samples = tone(200.0, 1.0, 1.0);
    let table = track(&samples, RATE, &RpmCurve::constant(3_000.0), &[4.0]);
    let level = table.level(4.0).unwrap();
    assert!(
        (level.mean_db - 0.0).abs() < 0.1,
        "a full-scale sine should be the 0 dB reference, read {:.2} dB",
        level.mean_db
    );
}

/// The headline claim: order 4 of a 3000 rpm engine is 200 Hz, and a tone
/// there is order 4 and nothing else.
#[test]
fn a_tone_on_order_four_reads_as_order_four() {
    let samples = tone(200.0, 0.5, 2.0);
    let table = track(
        &samples,
        RATE,
        &RpmCurve::constant(3_000.0),
        &[2.0, 3.0, 3.5, 4.0, 4.5, 5.0, 6.0, 8.0],
    );

    let fourth = table.level(4.0).unwrap();
    assert!(
        (fourth.mean_db - (-6.02)).abs() < 0.2,
        "half scale is -6.02 dBFS, read {:.2} dB",
        fourth.mean_db
    );
    assert!(fourth.frames > 0, "order 4 was never resolvable");

    // The whole orders either side are 50 Hz away, eight bins at this
    // window, and read more than 60 dB down.
    for order in [2.0, 3.0, 5.0, 6.0, 8.0] {
        let neighbour = table.level(order).unwrap();
        assert!(
            neighbour.mean_db < fourth.mean_db - 50.0,
            "order {order} read {:.1} dB against order 4's {:.1} dB; \
             the tone has leaked out of its own order",
            neighbour.mean_db,
            fourth.mean_db
        );
    }

    // The half-orders either side are only 25 Hz away — four bins, which is
    // the width of the window's own main lobe — so they can be shown to be
    // far down but not to be empty. Separating a half-order at 3000 rpm is
    // a question about the window, not about the signal.
    for order in [3.5, 4.5] {
        let neighbour = table.level(order).unwrap();
        assert!(
            neighbour.mean_db < fourth.mean_db - 25.0,
            "half-order {order} read {:.1} dB against order 4's {:.1} dB",
            neighbour.mean_db,
            fourth.mean_db
        );
    }

    assert_eq!(table.loudest().map(|l| l.order), Some(4.0));
}

/// The level must not depend on where the component falls between two bins.
///
/// Reading the tallest bin instead of summing the main lobe would lose up
/// to 1.4 dB here, which is larger than most of the differences these
/// tables exist to detect.
#[test]
fn a_level_is_independent_of_bin_alignment() {
    let bin = RATE / WINDOW as f64;
    let on_bin = 34.0 * bin;
    let between = 34.5 * bin;

    let a = track(
        &tone(on_bin, 0.5, 1.0),
        RATE,
        &RpmCurve::constant(60.0 * on_bin / 4.0),
        &[4.0],
    );
    let b = track(
        &tone(between, 0.5, 1.0),
        RATE,
        &RpmCurve::constant(60.0 * between / 4.0),
        &[4.0],
    );

    let (a, b) = (a.level(4.0).unwrap(), b.level(4.0).unwrap());
    assert!(
        (a.mean_db - b.mean_db).abs() < 0.2,
        "scalloping: {:.2} dB on a bin against {:.2} dB between two",
        a.mean_db,
        b.mean_db
    );
}

/// An order that sweeps while the window is open keeps its level.
#[test]
fn a_swept_order_keeps_its_level() {
    let seconds = 4.0;
    let (from, to) = (850.0, 7_000.0);
    let samples = swept_order(4.0, from, to, 0.5, seconds);

    let dt = 1.0 / 240.0;
    let steps = (seconds / dt) as usize;
    let speeds = (0..=steps)
        .map(|i| from + (to - from) * (i as f64 * dt / seconds))
        .collect();
    let table = track(
        &samples,
        RATE,
        &RpmCurve::new(speeds, dt),
        &[2.0, 3.0, 4.0, 5.0, 6.0, 8.0],
    );

    let fourth = table.level(4.0).unwrap();
    assert!(
        (fourth.mean_db - (-6.02)).abs() < 1.0,
        "a swept order 4 read {:.2} dB where a steady one reads -6.02",
        fourth.mean_db
    );
    // The neighbours cannot be asserted empty here and it would be dishonest
    // to try: at the bottom of the pull order 4 is 57 Hz and order 5 is
    // 71 Hz, two bins apart, and no window this long tells them apart. What
    // survives the sweep is which order the energy belongs to.
    assert_eq!(
        table.loudest().map(|l| l.order),
        Some(4.0),
        "the swept tone stopped being order 4"
    );
}

/// An order under the window's resolution is absent, not zero.
#[test]
fn an_unresolvable_order_reports_no_frames() {
    let samples = tone(200.0, 0.5, 1.0);
    let table = track(&samples, RATE, &RpmCurve::constant(850.0), &[0.5, 4.0]);

    // Order 0.5 of an engine at 850 rpm is 7.1 Hz, under the 17.6 Hz floor.
    let low = table.level(0.5).unwrap();
    assert_eq!(low.frames, 0);
    assert_eq!(low.mean_db, SILENCE_DB);
    assert!(table.level(4.0).unwrap().frames > 0);
}

/// A supplied sweep is exact between its endpoints.
#[test]
fn a_supplied_sweep_is_linear_between_its_endpoints() {
    let curve = RpmCurve::sweep(1_000.0, 7_000.0, 6.0);
    assert!((curve.seconds() - 6.0).abs() < 1e-9);
    assert!((curve.at(0.0) - 1_000.0).abs() < 1e-9);
    assert!((curve.at(3.0) - 4_000.0).abs() < 1e-9);
    assert!((curve.at(6.0) - 7_000.0).abs() < 1e-9);
}

/// A hand-logged curve keeps the points it was given and interpolates
/// between them, however they arrived.
#[test]
fn a_logged_curve_passes_through_its_points() {
    // Deliberately out of order and unevenly spaced, as a curve read off a
    // tachometer arrives.
    let points = [(2.0, 4_000.0), (0.0, 900.0), (3.0, 4_200.0), (1.0, 2_500.0)];
    let curve = RpmCurve::from_points(&points, 1.0 / 240.0);

    assert!((curve.seconds() - 3.0).abs() < 0.01);
    for (t, rpm) in points {
        assert!(
            (curve.at(t) - rpm).abs() < 1.0,
            "{t} s came back as {:.0} rpm, not {rpm:.0}",
            curve.at(t)
        );
    }
    // Between two logged points, and past the end where it holds.
    assert!((curve.at(1.5) - 3_250.0).abs() < 10.0);
    assert!((curve.at(99.0) - 4_200.0).abs() < 1.0);
}

#[test]
fn a_speed_curve_interpolates_and_clamps() {
    let curve = RpmCurve::new(vec![1_000.0, 2_000.0, 3_000.0], 0.5);
    assert_eq!(curve.seconds(), 1.0);
    assert!((curve.at(0.25) - 1_500.0).abs() < 1e-9);
    assert!((curve.at(-1.0) - 1_000.0).abs() < 1e-9);
    assert!((curve.at(99.0) - 3_000.0).abs() < 1e-9);
}

/// A speed excursion wholly inside a window still widens that order's band.
#[test]
fn a_speed_range_sees_inside_the_window() {
    let curve = RpmCurve::new(vec![7_000.0, 6_880.0, 7_000.0], 0.05);
    let (slow, fast) = curve.range(0.0, 0.1);
    assert!((slow - 6_880.0).abs() < 1e-9, "missed the bounce: {slow}");
    assert!((fast - 7_000.0).abs() < 1e-9);
}

/// A balance is the same whatever the gain, which is the whole reason it
/// exists: two renders twenty decibels apart in level have to compare as
/// identical, or a disagreement could be closed by turning a knob.
#[test]
fn a_balance_is_invariant_to_gain() {
    let loud: Vec<f32> = tone(200.0, 0.5, 2.0)
        .iter()
        .zip(tone(400.0, 0.125, 2.0))
        .map(|(a, b)| a + b)
        .collect();
    let quiet: Vec<f32> = loud.iter().map(|s| s * 0.1).collect();

    let rpm = RpmCurve::constant(3_000.0);
    let wanted = [2.0, 4.0, 8.0];
    let a = track(&loud, RATE, &rpm, &wanted).balance(4.0);
    let b = track(&quiet, RATE, &rpm, &wanted).balance(4.0);

    // Twenty decibels apart in absolute level.
    assert!(
        (a.reference_db - b.reference_db - 20.0).abs() < 0.1,
        "the two renders were {:.1} dB apart, not 20",
        a.reference_db - b.reference_db
    );
    // Order 8 is the 400 Hz tone: a quarter of the amplitude of the one on
    // order 4, so 12 dB down, in both.
    assert!(
        (a.at(8.0).unwrap() - (-12.04)).abs() < 0.2,
        "{:?}",
        a.at(8.0)
    );
    let comparison = b.against(&a);
    assert!(
        comparison.max_abs_db() < 0.05,
        "a gain change moved the balance by {:.3} dB",
        comparison.max_abs_db()
    );
    assert_eq!(comparison.deltas.len(), 3);
}

/// A comparison reports where two balances part company, and by how much.
#[test]
fn a_comparison_finds_the_order_that_disagrees() {
    let reference = Balance::from_levels(
        4.0,
        &[
            (2.0, Some(-30.0)),
            (4.0, Some(-20.0)),
            (6.0, Some(-40.0)),
            (8.0, None),
        ],
    );
    let measured = Balance::from_levels(
        4.0,
        &[
            (2.0, Some(-24.0)),
            (4.0, Some(-14.0)),
            (6.0, Some(-28.0)),
            (8.0, Some(-50.0)),
        ],
    );

    // Six decibels of gain between them, which the balance divides out, and
    // six decibels of real disagreement on order 6, which it keeps.
    let comparison = measured.against(&reference);
    assert_eq!(comparison.deltas.len(), 3);
    assert_eq!(comparison.missing, 1, "order 8 should not be compared");
    assert!((comparison.at(2.0).unwrap() - 0.0).abs() < 1e-9);
    assert!((comparison.at(6.0).unwrap() - 6.0).abs() < 1e-9);
    assert_eq!(comparison.worst().map(|d| d.order), Some(6.0));
    assert!((comparison.max_abs_db() - 6.0).abs() < 1e-9);
    assert!(comparison.within(6.5) && !comparison.within(5.5));
}

/// The plan's claim for the peak picker: two known tones, both found, both
/// within one per cent.
#[test]
fn the_peak_picker_finds_both_tones_of_a_two_tone_signal() {
    let (low, high) = (220.0, 733.0);
    let a = tone(low, 0.5, 2.0);
    let b = tone(high, 0.25, 2.0);
    let mixed: Vec<f32> = a.iter().zip(&b).map(|(x, y)| x + y).collect();

    let peaks = resonances(&mixed, RATE, 8);
    assert!(peaks.len() >= 2, "found {} peaks, wanted two", peaks.len());

    for (wanted, level) in [(low, -6.02), (high, -12.04)] {
        let found = peaks
            .iter()
            .min_by(|p, q| (p.hz - wanted).abs().total_cmp(&(q.hz - wanted).abs()))
            .unwrap();
        let error = (found.hz - wanted).abs() / wanted;
        assert!(
            error < 0.01,
            "{wanted} Hz came back as {:.2} Hz, {:.2} % out",
            found.hz,
            error * 100.0
        );
        assert!(
            (found.db - level).abs() < 0.5,
            "{wanted} Hz read {:.2} dB, wanted {level:.2}",
            found.db
        );
    }
}

/// Interpolation, not rounding: a tone deliberately between two bins comes
/// back at its own frequency rather than at the nearest bin centre.
#[test]
fn a_peak_between_two_bins_is_interpolated() {
    let bin = RATE / WINDOW as f64;
    let wanted = 60.4 * bin;
    let peaks = resonances(&tone(wanted, 0.5, 2.0), RATE, 4);

    let found = peaks
        .iter()
        .min_by(|p, q| (p.hz - wanted).abs().total_cmp(&(q.hz - wanted).abs()))
        .expect("no peak at all");
    assert!(
        (found.hz - wanted).abs() < 0.25 * bin,
        "{wanted:.2} Hz came back as {:.2} Hz, and the bin is {bin:.2} Hz wide",
        found.hz
    );
}

/// White noise has a flat floor, and a one-pole filter tilts it by the
/// 6 dB an octave a single pole is worth.
///
/// The tilt is measured against a signal whose slope is known from theory
/// rather than from a golden file: above its corner a one-pole lowpass
/// falls as `1 / f`, which is -6.02 dB per octave, and the measurement has
/// to find that without being dragged around by the scatter left in the
/// average.
#[test]
fn the_tilt_reads_a_one_pole_slope_as_six_db_an_octave() {
    let white = noise(4.0);
    let flat = AverageSpectrum::of(&white, RATE)
        .tilt_db_per_octave(200.0, 12_000.0)
        .expect("no tilt from white noise");
    assert!(
        flat.abs() < 0.6,
        "white noise tilted {flat:.2} dB per octave"
    );

    // A one-pole lowpass at 100 Hz, so the whole measured band is above its
    // corner and on its asymptote.
    let k = (-std::f64::consts::TAU * 100.0 / RATE).exp() as f32;
    let mut y = 0.0f32;
    let pink: Vec<f32> = white
        .iter()
        .map(|&x| {
            y = (1.0 - k) * x + k * y;
            y
        })
        .collect();
    let tilted = AverageSpectrum::of(&pink, RATE)
        .tilt_db_per_octave(200.0, 12_000.0)
        .expect("no tilt from filtered noise");
    assert!(
        (tilted - (-6.02)).abs() < 0.6,
        "a single pole read {tilted:.2} dB per octave, not -6.02"
    );
}

/// A peak riding on the floor must not become the floor: the median over an
/// octave is there to ignore it, and a tone 40 dB up has to leave the
/// slope where it was.
#[test]
fn a_loud_tone_does_not_tilt_the_floor() {
    let white = noise(4.0);
    let bare = AverageSpectrum::of(&white, RATE)
        .tilt_db_per_octave(200.0, 12_000.0)
        .unwrap();

    let with_tone: Vec<f32> = white
        .iter()
        .zip(tone(400.0, 0.5, 4.0))
        .map(|(n, t)| n + t)
        .collect();
    let dragged = AverageSpectrum::of(&with_tone, RATE)
        .tilt_db_per_octave(200.0, 12_000.0)
        .unwrap();
    assert!(
        (dragged - bare).abs() < 0.3,
        "one tone moved the floor from {bare:.2} to {dragged:.2} dB per octave"
    );
}

/// Noise alone has no resonances in it.
#[test]
fn a_flat_noise_floor_has_no_prominent_peaks() {
    let samples = noise(2.0);
    let peaks = resonances(&samples, RATE, 32);
    assert!(
        peaks.len() < 8,
        "picked {} peaks out of white noise",
        peaks.len()
    );
}

/// Placement against a prediction: a tone where it was expected is found
/// and measured, one far from any prediction is reported missing, and the
/// error says which way it is out.
#[test]
fn placement_finds_a_predicted_mode_and_misses_an_absent_one() {
    // A pipe predicted at 250 Hz whose audio actually resonates at 230:
    // 8 % low, which is a pipe 8 % long.
    let peaks = resonances(&tone(230.0, 0.5, 2.0), RATE, 8);

    let found = place(250.0, &peaks, PLACEMENT_WINDOW_PCT);
    let error = found.error_pct().expect("the mode was not found at all");
    assert!(
        (error - (-8.0)).abs() < 0.5,
        "a mode at 230 Hz against a 250 Hz prediction read {error:.2} %"
    );
    assert!(!found.within(5.0) && found.within(10.0));

    // And the length the audio is really behaving as: a quarter-wave mode
    // 8 % low is a pipe 8.7 % longer than the 0.40 m it was predicted from.
    let implied = found.implied_length(0.40).unwrap();
    assert!(
        (implied - 0.435).abs() < 0.002,
        "a 250 Hz prediction from 0.40 m measured at 230 Hz implies {implied:.4} m"
    );

    // Nothing resonates at 1 kHz in this signal, and saying so is not the
    // same as finding it in the wrong place.
    let absent = place(1_000.0, &peaks, PLACEMENT_WINDOW_PCT);
    assert!(absent.measured_hz.is_none());
    assert!(absent.error_pct().is_none());
    assert!(!absent.within(100.0));
}

/// Extraction off a "recording": a different sample rate, a supplied sweep
/// curve, an order that follows it and a resonance that does not.
///
/// Stands in for the real thing while no reference recording is licensed
/// into this repository — it is synthesised from a chirp and a tone, so
/// every number in it is known in advance — and it exercises exactly the
/// path a recording takes: a rate that is not the render's, a curve that
/// came from outside the audio, and one analysis for both.
#[test]
fn extraction_reads_a_recording_at_its_own_rate() {
    // 44.1 kHz, because that is what a recording arrives at.
    let rate = 44_100.0;
    let seconds = 6.0;
    let (from, to) = (1_200.0, 6_000.0);
    let n = (seconds * rate) as usize;

    let mut phase = 0.0f64;
    let samples: Vec<f32> = (0..n)
        .map(|i| {
            let t = i as f64 / rate;
            let rpm = from + (to - from) * (t / seconds);
            // Order 4 sweeping with the engine, a fixed 500 Hz pipe mode a
            // quarter of its amplitude, and a little noise under both.
            let swept = 0.5 * phase.sin();
            phase += 2.0 * PI * 4.0 * rpm / 60.0 / rate;
            let fixed = 0.125 * (2.0 * PI * 500.0 * t).sin();
            (swept + fixed) as f32
        })
        .collect();

    let rpm = RpmCurve::sweep(from, to, seconds);
    let reference = Reference::extract(&samples, rate, &rpm, &half_orders());

    assert!((reference.seconds - seconds).abs() < 0.01);
    assert!((reference.speed.0 - from).abs() < 1.0);
    assert!((reference.speed.1 - to).abs() < 1.0);

    // The swept component is order 4 and the balance is gain-free: the
    // fixed tone is 12 dB down on it and lands wherever the sweep drags
    // order 4 past 500 Hz, which is 7500 rpm — above this pull, so it never
    // reads as order 4 and never inflates it.
    let balance = reference.balance(4.0);
    assert_eq!(reference.orders.loudest().map(|l| l.order), Some(4.0));
    assert!(
        balance.at(2.0).unwrap() < -20.0,
        "order 2 read {:.1} dB under order 4",
        balance.at(2.0).unwrap()
    );

    // And the fixed tone is a resonance: it stands still while the orders
    // sweep past it, so it survives the average and lands within a per cent.
    let peaks = reference.peaks(8);
    let placement = place(500.0, &peaks, PLACEMENT_WINDOW_PCT);
    let error = placement
        .error_pct()
        .expect("the 500 Hz mode was not found at all");
    assert!(
        error.abs() < 1.0,
        "a 500 Hz mode came back {error:.2} % out"
    );
}

/// The crank's comb for an even-firing four: every even order, nothing
/// anywhere else, and no plumbing involved in saying so.
#[test]
fn an_even_firing_four_puts_nothing_on_the_odd_orders() {
    let offsets: Vec<f64> = (0..4).map(|k| k as f64 * PI).collect();
    let orders = half_orders();
    let comb = firing_comb(&offsets, &orders);

    for (&order, &amplitude) in orders.iter().zip(&comb) {
        let even = (order / 2.0).fract().abs() < 1e-9;
        if even {
            assert!(
                (amplitude - 4.0).abs() < 1e-9,
                "order {order} came back at {amplitude:.3}, not the four \
                 firings it is in phase with"
            );
        } else {
            assert!(
                amplitude < 1e-9,
                "order {order} came back at {amplitude:.3e} on a crank that \
                 cannot drive it"
            );
        }
    }
}

/// The cross-plane V8's burble is a crank fact before it is a sound: its
/// uneven bank leaves order 1.5 a few decibels under the firing order,
/// where the flat-plane crank leaves it nothing at all.
#[test]
fn only_the_uneven_bank_puts_energy_on_the_half_orders() {
    use crate::physics::engine_block::FiringOrder;

    let orders = half_orders();
    for (name, firing, burbles) in [
        ("cross-plane", FiringOrder::cross_plane_v8(), true),
        ("flat-plane", FiringOrder::flat_plane_v8(), false),
    ] {
        let banks: Vec<Vec<f64>> = (0..firing.bank_count())
            .map(|b| firing.bank_offsets(b as u8))
            .collect();
        let balance = crank_balance(&banks, &orders, 4.0);

        let half = balance.at(1.5).unwrap();
        if burbles {
            assert!(
                (half - (-3.7)).abs() < 0.2,
                "{name} put order 1.5 at {half:.2} dB under its firing order"
            );
        } else {
            assert!(
                half < -100.0,
                "{name} put order 1.5 at {half:.2} dB, and an even bank \
                 cannot drive it at all"
            );
        }
        // Either way nothing stands above the firing order. It is not the
        // *only* order at that level — a train of impulses is in phase
        // again at every multiple of it, so 8, 12 and 24 tie with it — but
        // nothing the crank drives exceeds it.
        assert_eq!(balance.at(4.0), Some(0.0));
        assert!(
            balance
                .levels
                .iter()
                .all(|l| l.relative_db.unwrap_or(SILENCE_DB) <= 1e-9),
            "{name} put something above its own firing order"
        );
    }
}

/// One peak cannot be three modes: the nearest prediction claims it and the
/// others are reported as not found.
#[test]
fn one_peak_is_claimed_by_one_prediction() {
    // A spectrum with a single mode in it, at 410 Hz.
    let peaks = resonances(&tone(410.0, 0.5, 2.0), RATE, 8);

    // Three predictions inside the window of it: an intake runner, an
    // exhaust primary and a silencer, as a sparse spectrum really does
    // offer.
    let placements = place_all(&[367.0, 400.0, 470.0], &peaks, PLACEMENT_WINDOW_PCT);
    let found: Vec<Option<f64>> = placements.iter().map(|p| p.measured_hz).collect();
    assert_eq!(
        found.iter().filter(|hz| hz.is_some()).count(),
        1,
        "three predictions claimed {found:?} between them"
    );
    // And it is the closest one that has it: 400 Hz is 2.4 % away, against
    // 11.7 % and 12.8 %.
    assert!(placements[1].measured_hz.is_some());
    assert!(placements[1].within(3.0));
}

/// A render shorter than one window has nothing to average.
#[test]
fn a_render_under_one_window_reports_no_frames() {
    let spectrum = AverageSpectrum::of(&tone(200.0, 0.5, 0.05), RATE);
    assert_eq!(spectrum.frames(), 0);
    assert!(spectrum.peaks(8, MIN_PROMINENCE_DB).is_empty());
}

/// Pink noise carries equal power in every octave by construction — that
/// is what "pink" means — so a correct octave-band reading has to come
/// back flat.
#[test]
fn pink_noise_reads_flat_across_the_bands() {
    let shares = octave_bands(&pink_noise(6.0), RATE);
    let mean = shares.iter().sum::<f64>() / shares.len() as f64;
    for (i, &share) in shares.iter().enumerate() {
        assert!(
            (share - mean).abs() < 1.0,
            "band {} ({} Hz) read {share:.2} dB against a {mean:.2} dB mean",
            i,
            OCTAVE_CENTERS_HZ[i]
        );
    }
}

/// A pure tone puts almost all of its energy in the one band it falls in
/// and its two neighbours, and none worth mentioning anywhere else.
#[test]
fn a_pure_tone_concentrates_in_its_own_band_and_its_neighbours() {
    let shares = octave_bands(&tone(1_000.0, 0.5, 4.0), RATE);

    // Linear power, not decibels, to sum a "mostly" claim honestly.
    let power = |db: f64| 10f64.powf(db / 10.0);
    let near: f64 = shares[4..=6].iter().map(|&s| power(s)).sum();
    let total: f64 = shares.iter().map(|&s| power(s)).sum();
    assert!(
        near / total > 0.90,
        "the 500 Hz, 1 kHz and 2 kHz bands hold only {:.1} % of the energy",
        100.0 * near / total
    );

    // Every other band is a Hann window's sidelobe leakage and nothing
    // else — far enough down to be negligible, though a finite window
    // never reaches the clamp floor exactly the way true silence does.
    for (i, &share) in shares.iter().enumerate() {
        if (4..=6).contains(&i) {
            continue;
        }
        assert!(
            share < -60.0,
            "band {} ({} Hz) read {share:.1} dB for a 1 kHz tone",
            i,
            OCTAVE_CENTERS_HZ[i]
        );
    }
}

/// A share is a ratio to the render's own total energy, so a gain change
/// must not move it — the same invariance an order [`Balance`] has to its
/// own reference order.
#[test]
fn octave_share_does_not_move_with_output_gain() {
    let quiet = octave_bands(&tone(440.0, 0.25, 4.0), RATE);
    let loud = octave_bands(&tone(440.0, 0.5, 4.0), RATE);
    for (i, (&q, &l)) in quiet.iter().zip(loud.iter()).enumerate() {
        assert!(
            (q - l).abs() < 0.05,
            "band {i} moved {:.3} dB on a 6 dB gain change",
            l - q
        );
    }
}

/// A square wave's peak equals its RMS; a sine's peak is `sqrt(2)` times
/// its RMS. At equal RMS the two differ by the analytic 3.01 dB.
#[test]
fn crest_factor_separates_a_square_wave_from_a_sine_by_three_db() {
    let n = (2.0 * RATE) as usize;
    let square: Vec<f32> = (0..n)
        .map(|i| if i % 100 < 50 { 0.5 } else { -0.5 })
        .collect();
    let sine = tone(RATE / 100.0, 0.5, 2.0);

    let square_crest = crest_db(&square);
    let sine_crest = crest_db(&sine);
    assert!(
        (square_crest - 0.0).abs() < 0.05,
        "a square wave's crest factor read {square_crest:.3} dB, not 0"
    );
    assert!(
        ((sine_crest - square_crest) - 3.01).abs() < 0.1,
        "the sine and square differed by {:.3} dB, not 3.01",
        sine_crest - square_crest
    );
}

/// Crest factor is a ratio of the render to itself, so a gain change must
/// not move it.
#[test]
fn crest_factor_does_not_move_with_output_gain() {
    let quiet = crest_db(&tone(440.0, 0.25, 2.0));
    let loud = crest_db(&tone(440.0, 0.5, 2.0));
    assert!(
        (quiet - loud).abs() < 1e-6,
        "crest factor moved {:.6} dB on a 6 dB gain change",
        loud - quiet
    );
}
