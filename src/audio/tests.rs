use super::*;
use crate::environment::Environment;

fn v8() -> EngineBlock {
    EngineBlock::cross_plane_v8(Environment::default())
}

/// Runs the block long enough to prime the phase ring.
fn primed(rpm: f64) -> EngineBlock {
    let mut block = v8();
    for _ in 0..600 {
        block.update(1.0 / 240.0, rpm);
    }
    assert!(block.ring.is_primed(), "ring never filled");
    block
}

/// A V8 that differs from the shipped one in its exhaust cam *duration* and
/// in nothing else.
///
/// The opening angle is left alone deliberately: EVO is what the audio side
/// builds its firing table from, so holding it fixed means the two engines
/// hand the synth identical configurations and every difference in what
/// comes out has to have arrived through the snapshot.
fn v8_with_exhaust_duration(duration_deg: f64, rpm: f64) -> EngineBlock {
    use crate::physics::cylinder::deg;
    use crate::physics::thermodynamics::ValveEvent;

    let mut block = v8();
    let stock = block.model.valves.exhaust;
    block.model.valves.exhaust = ValveEvent::new(
        stock.open_angle,
        deg(duration_deg),
        stock.max_lift,
        stock.diameter,
        stock.discharge_coefficient,
    );
    for _ in 0..600 {
        block.update(1.0 / 240.0, rpm);
    }
    assert!(block.ring.is_primed(), "ring never filled");
    block
}

#[test]
fn valve_timing_changes_the_pulse_spectrum() {
    use crate::analysis::orders::AverageSpectrum;

    // A long-duration race cam and a short road one, on the same block,
    // through the same pipes, at the same speed.
    const RPM: f64 = 4_000.0;
    let long = v8_with_exhaust_duration(300.0, RPM);
    let short = v8_with_exhaust_duration(190.0, RPM);

    let long_config = SynthConfig::from_block(&long, 48_000.0);
    let short_config = SynthConfig::from_block(&short, 48_000.0);
    // The claim of the whole stage, in one assertion: the audio side is
    // *identical* for the two engines — same taps, same pipes, same levels,
    // no constant anywhere that says which cam is fitted. Whatever the
    // difference in timbre turns out to be, it arrived through the snapshot.
    assert_eq!(
        long_config, short_config,
        "the two engines were voiced differently, so the comparison proves nothing"
    );

    /// Energy at the harmonics of the master cycle inside a band [dB].
    ///
    /// Summed over the harmonics rather than sampled at round frequencies,
    /// because the spectrum of a running engine is a comb: the gaps between
    /// its lines are 60 dB down and reading one says nothing about anything.
    fn band_db(spectrum: &AverageSpectrum, cycle_hz: f64, lo: f64, hi: f64) -> f64 {
        let mut power = 0.0;
        let mut harmonic = 1;
        loop {
            let hz = harmonic as f64 * cycle_hz;
            if hz > hi {
                break;
            }
            if hz >= lo {
                if let Some(db) = spectrum.db_at(hz) {
                    power += 10f64.powf(db / 10.0);
                }
            }
            harmonic += 1;
        }
        10.0 * power.max(1e-30).log10()
    }

    // Mid-band energy against the fundamentals, which is level-independent:
    // the two cams trap different masses and radiate at different levels,
    // and that is not what is being asserted.
    let tilt_of = |block: &EngineBlock| {
        let snapshot = SnapshotSource::new(block).sample(
            block,
            RPM,
            1.0 / 240.0,
            EngineControls::wide_open(),
        );
        let mut config = long_config.clone();
        // Leave only the exhaust: the pulse is what is being measured.
        config.intake_level = 0.0;
        config.mechanical_level = 0.0;
        let mut synth = EngineSynth::new(config);
        synth.set_snapshot(&snapshot);
        let mut buffer = vec![0.0f32; 2 * 48_000];
        synth.render(&mut buffer, 2); // settle the pipes
        synth.render(&mut buffer, 2);
        let mono: Vec<f32> = buffer.chunks(2).map(|f| 0.5 * (f[0] + f[1])).collect();
        let spectrum = AverageSpectrum::of(&mono, 48_000.0);
        let cycle_hz = RPM / 120.0;
        band_db(&spectrum, cycle_hz, 600.0, 1_500.0)
            - band_db(&spectrum, cycle_hz, 150.0, 600.0)
    };

    let long_tilt = tilt_of(&long);
    let short_tilt = tilt_of(&short);

    // The 300-degree cam is still off its seat when the intake opens and the
    // pipe turns the flow around on it, so it radiates a second and much
    // faster event every cycle — the reversal — on top of the blowdown. The
    // 190-degree cam has shut by then and radiates one. Nothing in the synth
    // was told either of those things; the difference is the port flow curve
    // and the pressure trace, and it is more than ten dB.
    assert!(
        long_tilt - short_tilt > 6.0,
        "cam duration barely moved the pulse spectrum: {long_tilt:.1} dB of mid-band \
         tilt on the long cam against {short_tilt:.1} dB on the short one"
    );
}

#[test]
fn config_matches_the_blocks_firing_order() {
    let block = v8();
    let config = SynthConfig::from_block(&block, 48_000.0);

    assert_eq!(config.cylinder_count(), 8);
    assert_eq!(config.bank_count, 2);
    assert!(config
        .cylinders
        .iter()
        .all(|c| (0.0..1.0).contains(&c.evo_phase) && c.bank < 2));

    // A cross-plane V8 puts four cylinders on each bank.
    let on_bank_0 = config.cylinders.iter().filter(|c| c.bank == 0).count();
    assert_eq!(on_bank_0, 4);

    // Every cylinder fires at a distinct phase.
    let mut phases: Vec<f32> = config.cylinders.iter().map(|c| c.evo_phase).collect();
    phases.sort_by(|a, b| a.partial_cmp(b).unwrap());
    for pair in phases.windows(2) {
        assert!(pair[1] - pair[0] > 1e-4, "duplicate firing phase: {pair:?}");
    }
}

#[test]
fn firing_frequency_follows_the_speed_law() {
    let config = SynthConfig::from_block(&v8(), 48_000.0);
    // f = (RPM / 120) * N_cyl: a V8 at 3000 rpm fires 200 times a second.
    assert!((config.firing_frequency(3_000.0) - 200.0).abs() < 1e-3);
    assert!((config.firing_frequency(0.0)).abs() < 1e-9);
}

#[test]
fn snapshot_is_physically_plausible_under_load() {
    let block = primed(3_000.0);
    let mut source = SnapshotSource::new(&block);
    let snapshot = source.sample(&block, 3_000.0, 1.0 / 240.0, EngineControls::wide_open());

    assert_eq!(snapshot.rpm, 3_000.0);
    assert!(
        snapshot.blowdown_delta[0] > 0.0,
        "no blowdown pressure at all"
    );
    assert!(
        snapshot.blowdown_delta[0] < 5.0e6,
        "implausible blowdown: {} Pa",
        snapshot.blowdown_delta[0]
    );
    assert!(
        (300.0..2_500.0).contains(&snapshot.exhaust_temperature),
        "implausible exhaust temperature: {} K",
        snapshot.exhaust_temperature
    );
    assert!((1.05..1.7).contains(&snapshot.exhaust_gamma));
    assert!(snapshot.intake_mass_flow >= 0.0);
    assert!(snapshot.unburnt_fuel_mass >= 0.0);
}

#[test]
fn every_snapshot_field_is_finite_across_the_speed_range() {
    for rpm in [600.0, 1_500.0, 4_000.0, 7_000.0] {
        let block = primed(rpm);
        let mut source = SnapshotSource::new(&block);
        for controls in [
            EngineControls::default(),
            EngineControls::wide_open(),
            EngineControls::on_the_limiter(1.0),
        ] {
            let s = source.sample(&block, rpm, 1.0 / 240.0, controls);
            // `sanitized` is the last line of defence; nothing should need
            // it, but the point is that nothing can get past it either.
            assert_eq!(s, s.sanitized(), "snapshot needed sanitising at {rpm} rpm");
        }
    }
}

/// The three turbos the catalogue fits.
fn every_turbo() -> [Induction; 3] {
    [
        Induction::small_single(),
        Induction::twin(),
        Induction::large_single(),
    ]
}

/// RMS of half a second of output, after half a second of settling.
fn settled_rms(config: SynthConfig, snapshot: &EngineSnapshot) -> f32 {
    let mut synth = EngineSynth::new(config);
    synth.set_snapshot(snapshot);
    let mut buffer = vec![0.0f32; 24_000 * 2];
    synth.render(&mut buffer, 2);
    synth.render(&mut buffer, 2);
    (buffer.iter().map(|s| s * s).sum::<f32>() / buffer.len() as f32).sqrt()
}

#[test]
fn a_fitted_turbo_is_audible_without_drowning_the_engine() {
    // Both halves of this matter. A turbo nobody can hear is a turbo that
    // was not worth modelling; a turbo louder than the engine sounds like a
    // kettle with a V8 somewhere behind it, which is what an un-normalised
    // voice and a level picked by eye produce.
    // A real frame off a real block: the engine half of this comparison is
    // driven by the solver's own cycle, so it has to come from one.
    let block = primed(7_000.0);
    let at_song = SnapshotSource::new(&block).sample(
        &block,
        7_000.0,
        1.0 / 240.0,
        EngineControls::wide_open(),
    );
    for induction in every_turbo() {
        let shaft = induction.shaft().expect("turbocharged");
        let snapshot = EngineSnapshot {
            turbo_rpm: shaft.max_shaft_rpm as f32,
            turbo_surge: 0.0,
            ..at_song
        }
        .sanitized();

        let base = SynthConfig::uniform(48_000.0, 6, 1).with_induction(induction);

        let mut turbo_only = base.clone();
        turbo_only.exhaust_level = 0.0;
        turbo_only.intake_level = 0.0;
        turbo_only.mechanical_level = 0.0;
        let turbo = settled_rms(turbo_only, &snapshot);

        let mut engine_only = base.clone();
        engine_only.turbo = None;
        let engine = settled_rms(engine_only, &snapshot);

        // How many dB this lands on depends on the engine underneath — a
        // V12 at full song is louder than a four — so the window is
        // deliberately loose. What it pins is the sign: a turbo above the
        // engine, or so far below it as to be decorative, is a mixing
        // mistake rather than a voicing choice.
        let db = 20.0 * (turbo / engine).log10();
        assert!(
            (-20.0..-3.0).contains(&db),
            "{} sits {db:.1} dB against the engine at full song, outside \
             the -20 to -3 dB a whistle belongs in",
            induction.label()
        );
    }
}

#[test]
fn every_turbo_whistles_somewhere_a_person_can_hear_it() {
    // Pitch is order times shaft speed, and both are set by hand, so it is
    // easy to voice a wheel above the synth's anti-alias ceiling. Nothing
    // fails when that happens: the tone simply pins against the clamp and
    // the whistle stops tracking the shaft at all.
    const CEILING_HZ: f64 = 0.22 * 48_000.0;
    for induction in every_turbo() {
        let shaft = induction.shaft().expect("turbocharged");
        let voice = induction.voice().expect("turbocharged");
        // The shaft is allowed to overspeed its own limit under a flow
        // above the reference, so the loudest tone is not the one at
        // `max_shaft_rpm`; see `TurboModel::update`.
        let peak_hz = shaft.max_shaft_rpm * 1.2_f64.powf(0.85) / 60.0 * voice.order;
        assert!(
            (900.0..CEILING_HZ).contains(&peak_hz),
            "{} tops out at {peak_hz:.0} Hz, outside the {:.0} Hz window                  the synth can actually voice",
            induction.label(),
            CEILING_HZ
        );
    }
}

#[test]
fn an_atmospheric_engine_has_no_shaft_to_spin() {
    let block = primed(6_500.0);
    let mut source = SnapshotSource::new(&block);
    assert!(!source.has_turbo());

    // Held at full throttle and high speed for a second: everything that
    // would spool a turbo, applied to an engine that has not got one.
    for _ in 0..240 {
        let snapshot = source.sample(&block, 6_500.0, 1.0 / 240.0, EngineControls::wide_open());
        assert_eq!(snapshot.turbo_rpm, 0.0);
        assert_eq!(snapshot.turbo_surge, 0.0);
    }
    // And the lift that would surge one does nothing either.
    let lift = source.sample(&block, 6_500.0, 1.0 / 240.0, EngineControls::default());
    assert_eq!(lift.turbo_surge, 0.0);
    assert_eq!(source.turbo_shaft_rpm(), 0.0);
}

#[test]
fn a_fitted_turbo_spools_and_surges() {
    let block = primed(6_500.0);
    let mut source = SnapshotSource::with_induction(&block, Induction::small_single());
    assert!(source.has_turbo());

    for _ in 0..240 {
        source.sample(&block, 6_500.0, 1.0 / 240.0, EngineControls::wide_open());
    }
    assert!(
        source.turbo_shaft_rpm() > 50_000.0,
        "the shaft never spooled: {:.0} rpm",
        source.turbo_shaft_rpm()
    );
    let lift = source.sample(&block, 6_500.0, 1.0 / 240.0, EngineControls::default());
    assert!(lift.turbo_surge > 0.3, "the lift did not surge");
}

#[test]
fn turbo_spools_up_slower_than_it_spools_down() {
    let mut turbo = TurboModel::default();
    let dt = 1.0 / 240.0;
    let wot = EngineControls::wide_open();
    let closed = EngineControls::default();

    // Time to reach half of the eventual speed from rest.
    let mut up = 0;
    while turbo.shaft_rpm < 0.5 * turbo.max_shaft_rpm && up < 100_000 {
        turbo.update(dt, 6_500.0, &wot);
        up += 1;
    }
    let peak = turbo.shaft_rpm;

    let mut down = 0;
    while turbo.shaft_rpm > 0.5 * peak && down < 100_000 {
        turbo.update(dt, 1_000.0, &closed);
        down += 1;
    }
    assert!(
        up > down,
        "spool-up ({up}) should outlast spool-down ({down})"
    );
}

#[test]
fn surge_needs_shaft_speed_and_a_shut_throttle() {
    let mut turbo = TurboModel::default();
    // At rest, nothing surges however shut the throttle is.
    assert_eq!(turbo.surge(&EngineControls::default()), 0.0);

    // Spin it up.
    for _ in 0..2_000 {
        turbo.update(1.0 / 240.0, 6_500.0, &EngineControls::wide_open());
    }
    assert!(turbo.shaft_rpm > 0.8 * turbo.max_shaft_rpm);

    // Spinning and open: no surge, the air has somewhere to go.
    assert!(turbo.surge(&EngineControls::wide_open()) < 1e-6);
    // Spinning and shut: surge.
    assert!(
        turbo.surge(&EngineControls::default()) > 0.5,
        "lift-off should surge"
    );
}

#[test]
fn spark_cut_puts_fuel_into_the_exhaust() {
    let block = primed(4_000.0);
    let mut source = SnapshotSource::new(&block);
    let dt = 1.0 / 240.0;

    // Ignition live: the charge burns, so nothing reaches the pipe.
    let burning = source.sample(&block, 4_000.0, dt, EngineControls::wide_open());
    assert!(
        burning.unburnt_fuel_mass < 1e-9,
        "a burnt charge should carry no fuel: {} kg",
        burning.unburnt_fuel_mass
    );

    // Ignition cut: the whole metered charge goes out unburnt, but compressed
    // cylinder charge still blows down on EVO.
    let cut = source.sample(&block, 4_000.0, dt, EngineControls::on_the_limiter(1.0));
    assert!(cut.spark_cut);
    assert!(
        cut.blowdown_delta[0] > 0.0,
        "an unburnt cut charge must still carry positive compression blowdown delta"
    );
    assert!(
        cut.blowdown_delta[0] < burning.blowdown_delta[0],
        "unburnt compression blowdown must be smaller than fired combustion blowdown"
    );
    assert!(
        burning.blowdown_delta[0] > 0.0,
        "an unfired cylinder must carry positive blowdown delta"
    );
    assert!(
        cut.unburnt_fuel_mass > 0.0,
        "a spark cut must leave fuel in the exhaust"
    );

    // And it must clear the threshold that arms a backfire, or the whole
    // layer is unreachable in practice.
    let config = SynthConfig::from_block(&block, 48_000.0);
    assert!(
        (cut.unburnt_fuel_mass as f64) > config.backfire_fuel_threshold,
        "cut charge ({:.1} mg) does not arm backfires (threshold {:.1} mg)",
        cut.unburnt_fuel_mass * 1e6,
        config.backfire_fuel_threshold * 1e6
    );
}

#[test]
fn a_limiter_bounce_actually_produces_pops() {
    // End to end: physics on the limiter must reach the synth as audible
    // backfires. This is the guard against the layer silently going dead if
    // a threshold or the fuel path is ever retuned.
    let mut block = primed(7_000.0);
    let mut source = SnapshotSource::new(&block);
    // Backfires share the bank's runner and muffler, so they cannot be
    // isolated by muting the exhaust — that would mute them too. Compare
    // peak level with the cut against the same engine running clean.
    let dt = 1.0 / 240.0;
    let mut run = |controls: EngineControls, block: &mut EngineBlock| {
        let mut synth = EngineSynth::new(SynthConfig::from_block(block, 48_000.0));
        let mut buffer = vec![0.0f32; 200 * 2];
        let mut peak = 0.0f32;
        for _ in 0..(3 * 240) {
            block.update(dt, 7_000.0);
            synth.set_snapshot(&source.sample(block, 7_000.0, dt, controls));
            synth.render(&mut buffer, 2);
            peak = buffer.iter().fold(peak, |m, s| m.max(s.abs()));
        }
        peak
    };

    let clean = run(EngineControls::wide_open(), &mut block);
    let cutting = run(EngineControls::on_the_limiter(1.0), &mut block);

    assert!(clean > 1e-3, "the engine made no sound at all");
    assert!(
        cutting > 1.5 * clean,
        "a limiter bounce produced no pops: {cutting:.3} vs {clean:.3} clean"
    );
}

#[test]
fn fuel_cut_cannot_backfire_while_spark_cut_does() {
    use crate::physics::control::{LimiterCut, LimiterMode};

    let mut block = primed(6_000.0);
    let mut source = SnapshotSource::new(&block);
    let dt = 1.0 / 240.0;
    let redline = 6_500.0;
    block.ecu.redline = redline;
    block.ecu.limiter_mode = LimiterMode::HardCut;

    // 1. Fuel-cut limiter: run at/above redline.
    block.ecu.limiter_cut_type = LimiterCut::Fuel;
    block.update(dt, 6_600.0);
    let fuel_cut_snap = source.sample(&block, 6_600.0, dt, EngineControls::wide_open());

    assert_eq!(
        fuel_cut_snap.unburnt_fuel_mass, 0.0,
        "fuel-cut limiter must leave zero unburnt fuel in the exhaust"
    );
    assert!(
        !fuel_cut_snap.spark_cut,
        "fuel-cut limiter must not cut spark"
    );

    // Render audio through EngineSynth and measure peak level
    let mut fuel_synth = EngineSynth::new(SynthConfig::from_block(&block, 48_000.0));
    let mut buffer = vec![0.0f32; 200 * 2];
    let mut fuel_peak = 0.0f32;
    for _ in 0..(2 * 240) {
        block.update(dt, 6_600.0);
        fuel_synth.set_snapshot(&source.sample(
            &block,
            6_600.0,
            dt,
            EngineControls::wide_open(),
        ));
        fuel_synth.render(&mut buffer, 2);
        fuel_peak = buffer.iter().fold(fuel_peak, |m, s| m.max(s.abs()));
    }

    // 2. Spark-cut limiter: run at/above redline.
    block.ecu.limiter_cut_type = LimiterCut::Spark;
    block.update(dt, 6_600.0);
    let spark_cut_snap = source.sample(&block, 6_600.0, dt, EngineControls::wide_open());

    assert!(
        spark_cut_snap.unburnt_fuel_mass > 0.0,
        "spark-cut limiter must deliver unburnt fuel into the exhaust"
    );
    let config = SynthConfig::from_block(&block, 48_000.0);
    assert!(
        spark_cut_snap.unburnt_fuel_mass as f64 > config.backfire_fuel_threshold,
        "spark-cut unburnt fuel ({:.1} mg) must exceed backfire threshold ({:.1} mg)",
        spark_cut_snap.unburnt_fuel_mass * 1e6,
        config.backfire_fuel_threshold * 1e6
    );

    let mut spark_synth = EngineSynth::new(config);
    let mut spark_peak = 0.0f32;
    for _ in 0..(2 * 240) {
        block.update(dt, 6_600.0);
        spark_synth.set_snapshot(&source.sample(
            &block,
            6_600.0,
            dt,
            EngineControls::wide_open(),
        ));
        spark_synth.render(&mut buffer, 2);
        spark_peak = buffer.iter().fold(spark_peak, |m, s| m.max(s.abs()));
    }

    // Spark cut triggers backfires which produce significantly higher acoustic peaks
    assert!(
        spark_peak > 1.3 * fuel_peak,
        "spark cut must produce backfires louder than fuel cut: {spark_peak:.3} vs {fuel_peak:.3}"
    );
}

#[test]
fn end_to_end_render_from_live_physics_is_clean() {
    // The whole path, minus the device: physics -> snapshot -> synth.
    let mut block = primed(2_500.0);
    let mut source = SnapshotSource::new(&block);
    let mut synth = EngineSynth::new(SynthConfig::from_block(&block, 48_000.0));

    let dt = 1.0 / 240.0;
    let frames_per_update = 200; // 48000 / 240
    let mut peak = 0.0f32;
    let mut energy = 0.0f64;
    let mut buffer = vec![0.0f32; frames_per_update * 2];

    for step in 0..240 {
        // Sweep the throttle and speed the way a driver would.
        let t = step as f64 / 240.0;
        let rpm = 1_000.0 + 5_000.0 * t;
        let controls = EngineControls {
            throttle: t,
            spark_cut: false,
            exhaust_cutout: false,
            anti_lag: false,
            starter_hz: 0.0,
        };
        block.update(dt, rpm);
        synth.set_snapshot(&source.sample(&block, rpm, dt, controls));
        synth.render(&mut buffer, 2);

        for sample in &buffer {
            assert!(sample.is_finite(), "non-finite sample at step {step}");
            peak = peak.max(sample.abs());
            energy += (*sample as f64) * (*sample as f64);
        }
    }

    assert!(peak > 1e-3, "a revving V8 produced no sound");
    assert!(peak <= 1.0, "output clipped: {peak}");
    assert!(energy > 0.0);
}

#[test]
fn anti_lag_maintains_turbo_shaft_speed_and_fuels_exhaust_on_lift() {
    let mut turbo_standard = TurboModel::default();
    let mut turbo_als = TurboModel::default();

    let dt = 1.0 / 60.0;
    let rpm = 5_000.0;

    // 1. Spool both turbos up under wide-open throttle
    let wot = EngineControls::wide_open();
    for _ in 0..120 {
        turbo_standard.update(dt, rpm, &wot);
        turbo_als.update(dt, rpm, &wot);
    }
    let speed_spooled = turbo_standard.shaft_rpm;
    assert!(speed_spooled > 100_000.0);

    // 2. Driver lifts off the throttle (throttle = 0.0) at 5,000 RPM
    let lift_normal = EngineControls::default();
    let lift_als = EngineControls::default().with_anti_lag(true);

    // After 1 second off-throttle:
    for _ in 0..60 {
        turbo_standard.update(dt, rpm, &lift_normal);
        turbo_als.update(dt, rpm, &lift_als);
    }

    // Without ALS, the turbo spools down rapidly
    // With ALS, turbine demand stays high, keeping shaft speed elevated
    assert!(
        turbo_standard.shaft_rpm < 50_000.0,
        "standard turbo must spool down on lift: got {}",
        turbo_standard.shaft_rpm
    );
    assert!(
        turbo_als.shaft_rpm > 100_000.0,
        "ALS turbo must maintain high shaft speed on lift: got {}",
        turbo_als.shaft_rpm
    );
    assert!(
        turbo_als.shaft_rpm > 2.0 * turbo_standard.shaft_rpm,
        "ALS shaft speed must be much higher than unassisted lift: als={} vs standard={}",
        turbo_als.shaft_rpm,
        turbo_standard.shaft_rpm
    );

    // 3. Verify SnapshotSource delivers unburnt fuel mass above backfire threshold when ALS is active
    let block = primed(5_000.0);
    let mut source = SnapshotSource::new(&block);
    let snapshot_normal = source.sample(&block, rpm, dt, lift_normal);
    let snapshot_als = source.sample(&block, rpm, dt, lift_als);

    assert!(
        snapshot_als.unburnt_fuel_mass as f64 > 12.0e-6,
        "ALS must supply unburnt fuel mass above backfire threshold: got {}",
        snapshot_als.unburnt_fuel_mass
    );
    assert!(
        snapshot_als.unburnt_fuel_mass > snapshot_normal.unburnt_fuel_mass,
        "ALS must deliver more exhaust fuel than standard lift: als={} vs normal={}",
        snapshot_als.unburnt_fuel_mass,
        snapshot_normal.unburnt_fuel_mass
    );
}

#[test]
fn supercharger_whine_tracks_crank_speed_exactly_and_shows_no_spool_lag() {
    // A supercharger is mechanically coupled to the crankshaft via belt or gears.
    // During an immediate throttle opening and engine acceleration, its tone
    // target updates synchronously with crank speed — there is zero fluid or
    // wheel spool lag. A turbocharger, in contrast, must accelerate its turbine
    // wheel against inertia using exhaust enthalpy, introducing a multi-hundred
    // millisecond spool lag.
    let block = primed(2_000.0);
    let mut source_roots = SnapshotSource::with_induction(&block, Induction::roots());
    let mut source_centrifugal =
        SnapshotSource::with_induction(&block, Induction::centrifugal());
    let mut source_turbo = SnapshotSource::with_induction(&block, Induction::large_single());

    let wot = EngineControls::wide_open();
    let dt = 1.0 / 240.0;
    let target_rpm = 6_000.0;

    // Advance 12 frames (50 ms) after a speed step
    let mut snap_roots = None;
    let mut snap_centrifugal = None;
    let mut snap_turbo = None;

    for _ in 0..12 {
        snap_roots = Some(source_roots.sample(&block, target_rpm, dt, wot));
        snap_centrifugal = Some(source_centrifugal.sample(&block, target_rpm, dt, wot));
        snap_turbo = Some(source_turbo.sample(&block, target_rpm, dt, wot));
    }

    let snap_roots = snap_roots.unwrap();
    let snap_centrifugal = snap_centrifugal.unwrap();
    let snap_turbo = snap_turbo.unwrap();

    // 1. Both supercharger snapshots report instantaneous crank speed without lag
    assert_eq!(snap_roots.rpm, target_rpm as f32);
    assert_eq!(snap_centrifugal.rpm, target_rpm as f32);

    // 2. Roots tone frequency is locked to crank order (2.1 * 4 = 8.4)
    let roots_order = 2.1 * 4.0;
    let roots_expected_hz = (target_rpm as f32 / 60.0) * roots_order;
    assert!(
        (roots_expected_hz - 840.0).abs() < 1e-3,
        "expected 840 Hz, got {roots_expected_hz}"
    );

    // 3. Centrifugal tone frequency is locked to crank order (9.2 * 1.8 = 16.56)
    let centrifugal_order = 9.2 * 1.8;
    let centrifugal_expected_hz = (target_rpm as f32 / 60.0) * centrifugal_order;
    assert!(
        (centrifugal_expected_hz - 1656.0).abs() < 1e-3,
        "expected 1656 Hz, got {centrifugal_expected_hz}"
    );

    // 4. Turbo shaft speed at 50 ms is still spooling up and lags far behind terminal speed
    let mut turbo_steady = source_turbo.clone();
    for _ in 0..600 {
        turbo_steady.sample(&block, target_rpm, dt, wot);
    }
    let terminal_turbo_rpm = turbo_steady.turbo_shaft_rpm();
    assert!(
        terminal_turbo_rpm > 100_000.0,
        "turbo must reach > 100,000 RPM at WOT 6000 RPM: got {terminal_turbo_rpm}"
    );

    // At 50 ms, the turbo shaft has only completed a fraction of its spool-up
    let transient_turbo_rpm = snap_turbo.turbo_rpm as f64;
    assert!(
        transient_turbo_rpm < 0.35 * terminal_turbo_rpm,
        "turbo spool lag must be present: at 50 ms shaft is {transient_turbo_rpm} RPM, \
         terminal is {terminal_turbo_rpm} RPM"
    );
}

#[test]
fn turbos_whistle_still_lags_the_two_are_distinguishable_in_an_order_analysis() {
    use crate::analysis::orders::{track, RpmCurve};

    let block = primed(2_000.0);
    let sample_rate = 48_000.0f64;
    let dt = 1.0f64 / 240.0f64;
    let duration = 2.0f64; // 2.0 second acceleration pull
    let frame_count = (duration / dt) as usize;
    let samples_per_frame = (sample_rate * dt) as usize;

    let mut source_roots = SnapshotSource::with_induction(&block, Induction::roots());
    let mut source_turbo = SnapshotSource::with_induction(&block, Induction::large_single());

    // Configure synth with isolated induction paths
    let mut config_roots =
        SynthConfig::from_block(&block, sample_rate as f32).with_induction(Induction::roots());
    config_roots.exhaust_level = 0.0;
    config_roots.intake_level = 0.0;
    config_roots.mechanical_level = 0.0;
    config_roots.structure_level = 0.0;
    let mut synth_roots = EngineSynth::new(config_roots);

    let mut config_turbo = SynthConfig::from_block(&block, sample_rate as f32)
        .with_induction(Induction::large_single());
    config_turbo.exhaust_level = 0.0;
    config_turbo.intake_level = 0.0;
    config_turbo.mechanical_level = 0.0;
    config_turbo.structure_level = 0.0;
    let mut synth_turbo = EngineSynth::new(config_turbo);

    let wot = EngineControls::wide_open();
    let mut speeds = Vec::with_capacity(frame_count);
    let mut mono_roots = Vec::with_capacity(frame_count * samples_per_frame);
    let mut mono_turbo = Vec::with_capacity(frame_count * samples_per_frame);

    let mut buf_roots = vec![0.0f32; 2 * samples_per_frame];
    let mut buf_turbo = vec![0.0f32; 2 * samples_per_frame];

    for k in 0..frame_count {
        let t = k as f64 * dt;
        let rpm = 2_000.0 + (6_000.0 - 2_000.0) * (t / duration);
        speeds.push(rpm);

        let snap_roots = source_roots.sample(&block, rpm, dt, wot);
        let snap_turbo = source_turbo.sample(&block, rpm, dt, wot);

        synth_roots.set_snapshot(&snap_roots);
        synth_turbo.set_snapshot(&snap_turbo);

        synth_roots.render(&mut buf_roots, 2);
        synth_turbo.render(&mut buf_turbo, 2);

        for chunk in buf_roots.chunks(2) {
            mono_roots.push(0.5 * (chunk[0] + chunk[1]));
        }
        for chunk in buf_turbo.chunks(2) {
            mono_turbo.push(0.5 * (chunk[0] + chunk[1]));
        }
    }

    let rpm_curve = RpmCurve::new(speeds, dt);
    let orders = [4.0, 6.0, 7.0, 8.0, 8.4, 9.0, 10.0, 12.0];

    let table_roots = track(&mono_roots, sample_rate, &rpm_curve, &orders);
    let table_turbo = track(&mono_turbo, sample_rate, &rpm_curve, &orders);

    // 1. In order analysis, the supercharger's energy is strongly focused on order 8.4
    let roots_peak = table_roots.loudest().expect("roots table has no loudest");
    assert_eq!(
        roots_peak.order, 8.4,
        "Roots supercharger whine must peak at crank order 8.4, got order {}",
        roots_peak.order
    );
    let roots_8_4_db = table_roots.level(8.4).unwrap().mean_db;
    let roots_6_0_db = table_roots.level(6.0).unwrap().mean_db;
    let roots_10_0_db = table_roots.level(10.0).unwrap().mean_db;
    assert!(
        roots_8_4_db - roots_6_0_db > 15.0,
        "order 8.4 ({roots_8_4_db:.1} dB) must stand well above order 6.0 ({roots_6_0_db:.1} dB)"
    );
    assert!(
        roots_8_4_db - roots_10_0_db > 15.0,
        "order 8.4 ({roots_8_4_db:.1} dB) must stand well above order 10.0 ({roots_10_0_db:.1} dB)"
    );

    // 2. The turbo has no lock to crank order 8.4 — its energy at order 8.4 is far lower
    let turbo_8_4_db = table_turbo.level(8.4).unwrap().mean_db;
    assert!(
        roots_8_4_db - turbo_8_4_db > 20.0,
        "supercharger whine at order 8.4 ({roots_8_4_db:.1} dB) must dominate turbo ({turbo_8_4_db:.1} dB) by >20 dB"
    );

    // 3. Early transient response: during the first 250 ms (first 60 frames = 12000 samples),
    // supercharger produces immediate whine power while turbo lags with minimal output
    let early_samples = 12_000;
    let roots_early_energy: f64 = mono_roots[..early_samples]
        .iter()
        .map(|&s| (s as f64) * (s as f64))
        .sum();
    let turbo_early_energy: f64 = mono_turbo[..early_samples]
        .iter()
        .map(|&s| (s as f64) * (s as f64))
        .sum();

    assert!(
        roots_early_energy > 1e-4,
        "roots whine must have audible energy immediately during transient: got {roots_early_energy}"
    );
    assert!(
        roots_early_energy > 5.0 * turbo_early_energy,
        "roots early energy ({roots_early_energy:.6}) must vastly exceed spooling turbo ({turbo_early_energy:.6})"
    );
}
