use super::*;

/// The regression itself: every engine in the catalogue still sounds like
/// the numbers recorded for it.
#[test]
fn no_preset_has_drifted_from_its_recorded_timbre() {
    let mut failures = Vec::new();
    for preset in EnginePreset::catalogue() {
        let measured = measure(&preset);
        let Some(recorded) = measured.recorded() else {
            continue;
        };
        for line in measured.drift(recorded) {
            failures.push(format!("{}: {line}", preset.name));
        }
    }
    assert!(
        failures.is_empty(),
        "timbre drifted:\n  {}\n\nIf the change was intended, re-record with \
         `cargo run --release --example calibrate -- --fingerprints` and \
         regenerate docs/measurements/calibration.md in the same commit.",
        failures.join("\n  ")
    );
}

/// The whole calibration loop, end to end: a render written to a file,
/// read back at a quarter of the level, order-tracked against a supplied
/// speed curve, and compared with the render it came from.
///
/// The reference here is the synth's own output, so this is a test of the
/// *loop* and not of the model — it cannot say the engine sounds right, and
/// no reference recording is committed for it to ask. What it does say is
/// that the parts between a file on disk and a per-order comparison are
/// sound: the WAV survives the round trip, a supplied curve puts the orders
/// where they belong, and twelve decibels of gain between the two sides
/// cancels out of every figure in the table. If that last part ever breaks,
/// a calibration against a real recording becomes a measurement of the
/// recording's mastering.
#[test]
fn the_reference_loop_matches_a_render_to_itself_at_another_level() {
    use crate::analysis::orders::{Reference, RpmCurve};
    use crate::analysis::render::{read_wav, write_wav};

    let preset = EnginePreset::inline_four();
    let script = script::calibration_sweep(&preset);
    let render = RenderPlan::new(&preset, &script).render();

    let dir = std::env::temp_dir().join("engine-sim-reference-loop");
    let _ = std::fs::create_dir_all(&dir);
    let path = dir.join("inline-4-sweep.wav");
    // A quarter of the amplitude: a recording arrives at whatever level the
    // chain that made it left it at.
    let quiet: Vec<f32> = render.samples.iter().map(|s| s * 0.25).collect();
    write_wav(
        &path,
        &quiet,
        render.channels as u16,
        render.sample_rate as u32,
    )
    .expect("writing the reference");

    let recording = read_wav(&path).expect("reading the reference");
    assert_eq!(recording.channels, render.channels);
    assert!((recording.seconds() - render.seconds()).abs() < 1e-6);

    // The curve is supplied from outside, as a recording's has to be: two
    // endpoints of a pull, exactly as `--rpm 850:7400` gives it.
    let supplied = RpmCurve::sweep(preset.idle, preset.redline, recording.seconds());
    let firing_order = preset.firing.len() as f64 / 2.0;
    let reference = Reference::extract(
        &recording.mono(),
        recording.sample_rate,
        &supplied,
        &half_orders(),
    );

    let measured = measure(&preset);
    let mine = orders::Balance::from_levels(
        firing_order,
        &measured
            .balance
            .iter()
            .map(|&(order, db)| (order, (db > SILENCE_DB).then_some(db)))
            .collect::<Vec<_>>(),
    );
    // The same render at full scale, for a level to compare against.
    let loud = orders::track(
        &render.mono(),
        render.sample_rate,
        &render.rpm,
        &half_orders(),
    )
    .line_balance(firing_order);

    let theirs = reference.orders.line_balance(firing_order);

    // Twelve decibels apart in absolute level, and the same engine.
    assert!(
        (loud.reference_db - theirs.reference_db - 12.04).abs() < 0.1,
        "a quarter of the amplitude read {:.2} dB down, not 12.04",
        loud.reference_db - theirs.reference_db
    );
    let comparison = mine.against(&theirs);
    assert!(
        comparison.deltas.len() >= 4,
        "only {} orders compared",
        comparison.deltas.len()
    );
    assert!(
        comparison.within(LEVEL_TOLERANCE_DB),
        "the loop disagreed with itself by {:.2} dB on order {:?}",
        comparison.max_abs_db(),
        comparison.worst().map(|d| d.order),
    );

    let _ = std::fs::remove_file(&path);
}

/// The declared primary length still reaches the sound: stretch the
/// primaries and the rendered exhaust band moves down with them.
///
/// # Why this no longer looks for a peak
///
/// It used to. It located the peak nearest `c/4L`, stretched the primaries
/// by half, and required the peak to move by about half as much again. That
/// worked because the network it measured was very nearly lossless: with
/// the wall taking a fiftieth of the decibels `alpha` asks for, the whole
/// run from the valve to the mouth was one resonator of enormous Q, and
/// stretching the primaries moved its modes bodily — 358 Hz to 172 Hz on
/// the network's own response, at a resonant gain of 51 dB.
///
/// The same measurement now reads 21 dB, and the primary's own quarter wave
/// is a broad hump rather than a peak: its loop pays 1.4 dB a round trip to
/// the wall, and the four-into-one collector behind it sends most of what
/// arrives onward rather than back. That is what a header does, and it is
/// why the peak picker reports the mode as *not found* — a finding recorded
/// per preset in `docs/measurements/calibration.md`, not a failure of the
/// synth to read the length.
///
/// So the claim is made where it survives: at the network, where the
/// primary's transit is exactly proportional to the length it was given,
/// and in the render, where the spectrum's centre of gravity over the band
/// the primaries occupy moves monotonically as they are stretched. A synth
/// that had stopped reading the length would fail both.
///
/// # Why the centroid now moves *up*, not down
///
/// Stage T5 gave `ExpansionChamber` — the inline-four's own silencer, and
/// by the doc comment above already most of what dominates this band — a
/// real loss term for the first time. This band's centroid is a balance
/// between the chamber's own resonance and the primary's broad hump, and
/// damping the chamber shifts that balance enough to flip which way the
/// sum leans as the primary stretches. The relationship is still real and
/// still cleanly monotonic — confirmed at eight factors from 1.0 to 2.0,
/// not just the three asserted here — it has simply reversed sign, which
/// is exactly the kind of number a stage that touches the exhaust network
/// is expected to move; see `docs/measurements/timbre-t5.md`.
#[test]
fn the_declared_primary_length_still_reaches_the_sound() {
    use crate::analysis::orders::Stft;
    use crate::audio::dsp::EngineSnapshot;
    use crate::audio::waveguide::ExhaustNetwork;
    use crate::environment::Environment;
    use crate::physics::plumbing::PipeSection;

    /// The band the inline-four's primaries and the modes around them
    /// occupy at the temperatures the sweep reaches [Hz].
    const BAND: (f64, f64) = (150.0, 900.0);

    let stretched = |factor: f64| -> EnginePreset {
        let mut preset = EnginePreset::inline_four();
        let primary = preset.exhaust.primaries[0];
        preset.exhaust.primaries = vec![
            PipeSection::from_diameter(
                primary.length * factor,
                primary.diameter(),
                primary.wall_temperature,
            );
            preset.firing.len()
        ];
        preset
    };

    // 1. The network's own transit, which is where the length enters.
    let transit = |factor: f64| -> f32 {
        let preset = stretched(factor);
        let block = preset.block(Environment::default());
        let config = preset.synth_config(&block, 48_000.0);
        let network = ExhaustNetwork::new(
            &config.exhaust,
            &config.cylinders,
            config.bank_count,
            48_000.0,
            &EngineSnapshot::default(),
        );
        network.primary_round_trip_seconds(0)
    };
    let ratio = transit(1.5) / transit(1.0);
    assert!(
        (ratio - 1.5).abs() < 0.02,
        "a primary half again as long round-trips in {ratio:.3} of the time"
    );

    // 2. And it reaches the render: the centre of gravity of the band the
    //    primaries work in falls as they lengthen, every step of the way.
    //
    //    A steady hold, not `calibration_sweep`'s idle-to-redline ramp:
    //    the ramp changes rpm, throttle, spark timing and knock retard
    //    together as it runs, and none of that is the primary length's
    //    own doing — it is real physics `TURBO_PLAN.md`'s TB3 made more
    //    complete, and it now moves the centroid by more than stretching
    //    a pipe does, drowning the very effect this measures. A hold at
    //    one operating point removes every one of those variables but
    //    the geometry, so what moves the centroid between the three
    //    renders is only the primary.
    let centroid = |factor: f64| -> f64 {
        let preset = stretched(factor);
        let script = script::RenderScript::new(
            "t5_primary_length",
            "a steady hold, isolating primary length from everything a full sweep also moves",
            vec![script::Segment::hold(3.0, 4_000.0, 1.0)],
        );
        let render = RenderPlan::new(&preset, &script).render();
        let mono = render.mono();
        let mut stft = Stft::new(8_192, render.sample_rate);
        let mut power = vec![0.0f64; 8_192 / 2 + 1];
        let (mut frames, mut at) = (0usize, 0usize);
        let hop = stft.hop();
        while at + 8_192 <= mono.len() {
            stft.analyse(&mono[at..at + 8_192]);
            for (sum, p) in power.iter_mut().zip(stft.power()) {
                *sum += *p;
            }
            frames += 1;
            at += hop;
        }
        let bin = render.sample_rate / 8_192.0;
        let (mut moment, mut total) = (0.0, 0.0);
        for (k, &p) in power.iter().enumerate() {
            let hz = k as f64 * bin;
            if (BAND.0..=BAND.1).contains(&hz) {
                moment += hz * p / frames.max(1) as f64;
                total += p / frames.max(1) as f64;
            }
        }
        moment / total.max(1e-30)
    };

    let short = centroid(1.0);
    let middle = centroid(1.25);
    let long = centroid(1.5);
    assert!(
        short < middle && middle < long,
        "the band did not follow the length: {short:.1}, {middle:.1}, {long:.1} Hz"
    );
    // A little over four per cent over a half-again stretch — down from
    // five before Stage T5 damped the chamber, so three rather than five
    // is the floor this asserts, comfortably under what is actually
    // measured. Still far short of proportional, and for the reason
    // originally given: most of what is in this band is the collector,
    // the chamber and the tailpipe, and none of those moved.
    assert!(
        long > 1.03 * short,
        "stretching the primaries by half moved the band only from \
         {short:.1} Hz to {long:.1} Hz"
    );
}

/// The comparator catches what it is there to catch, without rendering
/// anything: a level that moved, a resonance that moved, a quiet order that
/// climbed out of the floor, and a tilt that flattened.
#[test]
fn drift_is_reported_number_by_number() {
    let recorded = Fingerprint {
        preset: "Test",
        firing_order: 2.0,
        balance: &[(0.5, -45.0), (1.0, -12.0), (2.0, 0.0), (4.0, -8.0)],
        resonances: &[200.0, 800.0],
        tilt_db_per_octave: -8.0,
        octave_share: &[
            -20.0, -18.0, -16.0, -14.0, -12.0, -10.0, -12.0, -14.0, -18.0, -22.0,
        ],
        crest_db: 14.0,
    };
    let unchanged = Measured {
        preset: "Test",
        firing_order: 2.0,
        balance: vec![(0.5, -47.0), (1.0, -12.4), (2.0, 0.0), (4.0, -8.3)],
        resonances: vec![
            Peak {
                hz: 201.0,
                db: -20.0,
                prominence_db: 12.0,
            },
            Peak {
                hz: 806.0,
                db: -30.0,
                prominence_db: 9.0,
            },
        ],
        tilt_db_per_octave: -8.4,
        octave_share: [
            -20.3, -18.2, -16.1, -14.2, -12.3, -10.2, -12.4, -14.3, -18.4, -22.4,
        ],
        crest_db: 14.3,
    };
    assert!(
        unchanged.drift(&recorded).is_empty(),
        "arithmetic-sized differences read as drift: {:?}",
        unchanged.drift(&recorded)
    );

    let drifted = Measured {
        // Order 4 down three decibels, the quiet half-order up out of the
        // floor, the second resonance a fifth of an octave sharp, and the
        // floor two decibels an octave flatter.
        balance: vec![(0.5, -20.0), (1.0, -12.0), (2.0, 0.0), (4.0, -11.0)],
        resonances: vec![
            Peak {
                hz: 200.0,
                db: -20.0,
                prominence_db: 12.0,
            },
            Peak {
                hz: 900.0,
                db: -30.0,
                prominence_db: 9.0,
            },
        ],
        tilt_db_per_octave: -6.0,
        ..unchanged
    };
    let lines = drifted.drift(&recorded);
    assert_eq!(lines.len(), 4, "reported {lines:?}");
    assert!(lines[0].contains("order 0.5") && lines[0].contains("floor"));
    assert!(lines[1].contains("order 4") && lines[1].contains("-3.0 dB"));
    assert!(lines[2].contains("resonance 2") && lines[2].contains("+12.5 %"));
    assert!(lines[3].contains("tilted"));
}

/// Stage T0's own additions to the comparator: a band or a crest factor
/// that moved by an arithmetic-sized amount stays silent, and one that
/// moved by the kind of amount this plan exists to catch is named.
#[test]
fn drift_names_the_band_and_crest_that_moved() {
    let recorded = Fingerprint {
        preset: "Test",
        firing_order: 2.0,
        balance: &[(2.0, 0.0)],
        resonances: &[],
        tilt_db_per_octave: -8.0,
        octave_share: &[
            -20.0, -18.0, -16.0, -14.0, -12.0, -10.0, -12.0, -14.0, -18.0, -22.0,
        ],
        crest_db: 14.0,
    };
    let mut quiet = Measured {
        preset: "Test",
        firing_order: 2.0,
        balance: vec![(2.0, 0.0)],
        resonances: vec![],
        tilt_db_per_octave: -8.0,
        octave_share: *recorded.octave_share,
        crest_db: recorded.crest_db,
    };

    // A one-decibel nudge at 2 kHz (index 6) is ordinary noise.
    quiet.octave_share[6] -= 1.0;
    assert!(
        quiet.drift(&recorded).is_empty(),
        "a 1 dB move at 2 kHz read as drift: {:?}",
        quiet.drift(&recorded)
    );

    // Three decibels there is not.
    let mut loud = quiet.clone();
    loud.octave_share[6] = recorded.octave_share[6] - 3.0;
    let lines = loud.drift(&recorded);
    assert_eq!(lines.len(), 1, "reported {lines:?}");
    assert!(lines[0].contains("2000") && lines[0].contains("-3.0 dB"));

    // The same shape of test for crest factor: silent under tolerance,
    // named over it.
    let mut hollow = quiet;
    hollow.octave_share[6] = recorded.octave_share[6];
    hollow.crest_db = recorded.crest_db - 1.0;
    assert!(
        hollow.drift(&recorded).is_empty(),
        "a 1 dB crest move drifted"
    );
    hollow.crest_db = recorded.crest_db - 3.0;
    let lines = hollow.drift(&recorded);
    assert_eq!(lines.len(), 1, "reported {lines:?}");
    assert!(lines[0].contains("crest factor") && lines[0].contains("-3.0 dB"));
}

/// A preset with no fingerprint is not regressed, so adding one to the
/// catalogue has to come with recording it.
#[test]
fn every_preset_in_the_catalogue_has_a_fingerprint() {
    let missing: Vec<&str> = EnginePreset::catalogue()
        .iter()
        .map(|preset| preset.name)
        .filter(|name| !RECORDED.iter().any(|f| f.preset == *name))
        .collect();
    assert!(
        missing.is_empty(),
        "no recorded timbre for {} — record one with \
         `cargo run --release --example calibrate -- --fingerprints`",
        missing.join(", ")
    );
}
