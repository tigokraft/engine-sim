use super::*;

const FS: f32 = 48_000.0;

/// Exhaust manifold pressure the synthetic cycles below sit on top of [Pa].
const TEST_MANIFOLD_PA: f32 = 1.2e5;

/// A cycle in the shape the solver produces, for tests that are not about
/// the solver.
///
/// Cut at EVO like the real thing: blowdown decaying towards the manifold,
/// the exhaust valve open for `exhaust_duration` of the cycle with a
/// raised-cosine flow under it, induction on the following stroke, and the
/// combustion pressure rise just before the cycle comes back round to EVO.
///
/// The trace joins up across the seam, because the seam is not an event: the
/// cylinder is at its combustion pressure when the valve cracks, and blows
/// down from *there*. A curve that stepped 55 bar in one cell would be a
/// cycle no engine runs, and the structural path differentiates this.
/// Peak mass flow through one exhaust port on the synthetic cycle [kg/s].
const EXHAUST_PORT_PEAK_FLOW: f32 = 0.12;

fn synthetic_cycle(
    peak: f32,
    exhaust_duration: f32,
) -> ([f32; CYCLE_TABLE], [f32; CYCLE_TABLE], [f32; CYCLE_TABLE]) {
    let mut pressure = [TEST_MANIFOLD_PA; CYCLE_TABLE];
    let mut exhaust = [0.0f32; CYCLE_TABLE];
    let mut intake = [0.0f32; CYCLE_TABLE];
    // IVO 200 degrees after EVO, IVC 440 after, on the default cam.
    let (ivo, ivc) = (200.0 / 720.0, 440.0 / 720.0);
    for k in 0..CYCLE_TABLE {
        let phi = (k as f32 + 0.5) / CYCLE_TABLE as f32;
        if phi < exhaust_duration {
            let u = phi / exhaust_duration;
            pressure[k] = TEST_MANIFOLD_PA + 15.0 * peak * (-phi / 0.04).exp();
            // Kilograms a second, and the figure matters now: the valve
            // boundary resists what crosses it. A 5 litre V8 at 3000 rpm
            // pushes something over a tenth of a kilogram a second through
            // one port at the top of the stroke, and the excitation this
            // curve also drives is normalised, so the scale is free
            // everywhere else it is read.
            exhaust[k] = EXHAUST_PORT_PEAK_FLOW * 0.5 * (1.0 - (TAU * u).cos());
        } else if phi < ivc {
            let u = ((phi - ivo) / (ivc - ivo)).clamp(0.0, 1.0);
            intake[k] = 0.05 * 0.5 * (1.0 - (TAU * u).cos());
        } else {
            // Compression and burn, peaking a little before the valve opens.
            let u = (phi - ivc) / (1.0 - ivc);
            pressure[k] = TEST_MANIFOLD_PA + 15.0 * peak * u.powi(6);
        }
    }
    (pressure, exhaust, intake)
}

/// A valve event on the same synthetic cycle: open over `span` of the
/// cycle from `open`, raised-cosine lift, peaking at `peak_area` [m^2].
fn synthetic_valve(open: f32, span: f32, peak_area: f32) -> [f32; CYCLE_TABLE] {
    let mut area = [0.0f32; CYCLE_TABLE];
    for (k, a) in area.iter_mut().enumerate() {
        let phi = (k as f32 / CYCLE_TABLE as f32 - open).rem_euclid(1.0);
        if phi < span {
            *a = peak_area * 0.5 * (1.0 - (TAU * phi / span).cos());
        }
    }
    area
}

/// The swept volume over the same synthetic cycle, from `clearance` to
/// `clearance + displacement` [m^3].
///
/// Index zero is EVO, which on a real cam is close enough to BDC to put the
/// piston at the bottom of the bore, so the box starts at its largest and
/// halves its way up twice a cycle.
fn synthetic_volume(clearance: f32, displacement: f32) -> [f32; CYCLE_TABLE] {
    let mut volume = [0.0f32; CYCLE_TABLE];
    for (k, v) in volume.iter_mut().enumerate() {
        let phi = k as f32 / CYCLE_TABLE as f32;
        *v = clearance + 0.5 * displacement * (1.0 + (2.0 * TAU * phi).cos());
    }
    volume
}

fn loaded_snapshot() -> EngineSnapshot {
    let (cylinder_pressure, exhaust_port_flow, intake_port_flow) =
        synthetic_cycle(4.0e5, 240.0 / 720.0);
    // Matching the windows `synthetic_cycle` opens its ports over.
    let exhaust_valve_area = synthetic_valve(0.0, 240.0 / 720.0, 4.5e-4);
    let intake_valve_area = synthetic_valve(200.0 / 720.0, 260.0 / 720.0, 7.0e-4);
    let cylinder_volume = synthetic_volume(6.2e-5, 6.2e-4);
    EngineSnapshot {
        rpm: 3_000.0,
        blowdown_delta: [4.0e5; MAX_CYLINDERS],
        exhaust_temperature: 950.0,
        primary_temperature: [950.0; MAX_CYLINDERS],
        collector_temperature: 950.0,
        tailpipe_temperature: 950.0,
        exhaust_gamma: 1.33,
        exhaust_gas_constant: 287.0,
        intake_mass_flow: 0.20,
        cylinder_intake_flow: [0.20 / 8.0; MAX_CYLINDERS],
        throttle: 0.8,
        turbo_rpm: 90_000.0,
        turbo_surge: 0.0,
        unburnt_fuel_mass: 0.0,
        // Chen-Flynn on the shipped V8 at 3000 rpm under load.
        friction_mep: 1.5e5,
        cold_fraction: 0.0,
        starter_hz: 0.0,
        spark_cut: false,
        knock_intensity: 0.0,
        bore: 0.084,
        peak_cylinder_pressure: 60.0e5,
        indicated_torque: 250.0,
        inertia: 0.25,
        cylinder_pressure,
        exhaust_port_flow,
        intake_port_flow,
        exhaust_valve_area,
        intake_valve_area,
        cylinder_volume,
        exhaust_manifold_pressure: TEST_MANIFOLD_PA,
        exhaust_cutout: false,
        anti_lag: false,
    }
}

fn render(synth: &mut EngineSynth, frames: usize) -> Vec<f32> {
    let mut buffer = vec![0.0; frames * 2];
    synth.render(&mut buffer, 2);
    buffer
}

/// Renders in irregular chunks, the way a real device asks for samples.
fn render_chunked(synth: &mut EngineSynth, frames: usize, sizes: &[usize]) -> Vec<f32> {
    let mut out = Vec::with_capacity(frames * 2);
    let mut i = 0;
    while out.len() < frames * 2 {
        let n = sizes[i % sizes.len()];
        let mut chunk = vec![0.0; n * 2];
        synth.render(&mut chunk, 2);
        out.extend_from_slice(&chunk);
        i += 1;
    }
    out.truncate(frames * 2);
    out
}

fn peak(samples: &[f32]) -> f32 {
    samples.iter().fold(0.0f32, |m, s| m.max(s.abs()))
}

fn approx(a: f32, b: f32) {
    assert!((a - b).abs() <= 1e-4, "expected {b}, got {a}");
}

fn rms(samples: &[f32]) -> f32 {
    let sum: f64 = samples.iter().map(|s| (*s as f64) * (*s as f64)).sum();
    (sum / samples.len().max(1) as f64).sqrt() as f32
}

/// Largest step between consecutive samples — the thing a "pop" actually is.
/// Amplitude of one frequency in an interleaved stereo buffer.
fn magnitude_at(samples: &[f32], hz: f32, sample_rate: f32) -> f32 {
    let mono: Vec<f32> = samples.chunks(2).map(|f| 0.5 * (f[0] + f[1])).collect();
    let w = TAU * hz / sample_rate;
    let (mut re, mut im) = (0.0f32, 0.0f32);
    for (n, &x) in mono.iter().enumerate() {
        let phase = w * n as f32;
        re += x * phase.cos();
        im += x * phase.sin();
    }
    2.0 * (re * re + im * im).sqrt() / mono.len() as f32
}

fn max_slew(samples: &[f32], channels: usize) -> f32 {
    samples
        .chunks(channels)
        .zip(samples.chunks(channels).skip(1))
        .fold(0.0f32, |m, (a, b)| m.max((b[0] - a[0]).abs()))
}

#[test]
fn silent_before_any_snapshot() {
    let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
    let out = render(&mut synth, 4_800);
    assert_eq!(peak(&out), 0.0, "a stopped engine must be silent");
}

#[test]
fn output_is_finite_and_bounded_under_load() {
    let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
    synth.set_snapshot(&loaded_snapshot());
    let out = render(&mut synth, 48_000);
    assert!(out.iter().all(|s| s.is_finite()), "non-finite sample");
    assert!(peak(&out) <= 1.0, "clipped past full scale: {}", peak(&out));
    assert!(rms(&out) > 1e-3, "engine produced no sound");
}

#[test]
fn firing_frequency_tracks_the_speed_law() {
    // f = (RPM / 120) * N_cyl. Count blowdown triggers over a known span and
    // compare against the closed form.
    let config = SynthConfig::uniform(FS, 4, 1);
    let mut synth = EngineSynth::new(config);
    let mut snapshot = loaded_snapshot();
    snapshot.rpm = 6_000.0;
    synth.set_snapshot(&snapshot);
    // Let the smoothed speed settle before counting.
    render(&mut synth, 24_000);

    let expected = 6_000.0 / 120.0 * 4.0; // 200 Hz
    assert!((synth.firing_frequency() - expected).abs() < 1.0);

    // And the phase really advances at RPM / 120 cycles per second.
    let before = synth.cycle_phase();
    render(&mut synth, FS as usize); // exactly one second
    let cycles = 6_000.0 / 120.0;
    let expected_phase = (before + cycles).rem_euclid(1.0);
    assert!(
        (synth.cycle_phase() - expected_phase).abs() < 1e-2,
        "phase drifted: {} vs {expected_phase}",
        synth.cycle_phase()
    );
}

#[test]
fn pulse_amplitude_is_linear_in_pressure_difference() {
    let level_at = |delta: f32| {
        // Isolate the exhaust: no intake and no mechanical floor (the V8
        // has no turbo to silence). The floor matters most here — it is the
        // only layer that is *continuous*, so leaving it in would put a
        // constant term under both measurements and pull any ratio toward
        // one.
        let mut config = SynthConfig::cross_plane_v8(FS);
        config.mechanical_level = 0.0;
        // The intake plays the cycle's own port flow now, so zeroing the
        // snapshot's mass flow no longer silences it; the level is what
        // takes it out of the mix. The structure has to go for the same
        // reason and one more: it is driven by the pressure *curve*, which
        // this test does not vary, so it would sit under both measurements
        // as a constant.
        config.intake_level = 0.0;
        config.structure_level = 0.0;
        let mut synth = EngineSynth::new(config);
        let mut snapshot = loaded_snapshot();
        snapshot.blowdown_delta = [delta; MAX_CYLINDERS];
        snapshot.turbo_rpm = 0.0;
        synth.set_snapshot(&snapshot);
        render(&mut synth, 24_000); // settle
        rms(&render(&mut synth, 48_000))
    };

    let single = level_at(1.0e5);
    let double = level_at(2.0e5);
    let ratio = double / single;
    assert!(
        (1.85..2.15).contains(&ratio),
        "amplitude is not linear in dP: ratio {ratio}"
    );
}

#[test]
fn induction_is_a_train_of_gulps_at_the_firing_order() {
    // The intake layer used to be noise whose loudness moved with a scalar,
    // which has no rate in it at all. Reading the port flow curve at each
    // cylinder's own phase gives the layer the engine's firing order for
    // free — the gulps *are* the modulation.
    let mut config = SynthConfig::cross_plane_v8(FS);
    config.exhaust_level = 0.0;
    config.mechanical_level = 0.0;
    let mut synth = EngineSynth::new(config);
    synth.exhaust_level.snap(0.0);
    synth.set_snapshot(&loaded_snapshot());
    render(&mut synth, 24_000);
    let out = render(&mut synth, 48_000);

    // A V8 at 3000 rpm draws 200 times a second.
    let firing = 3_000.0 / 120.0 * 8.0;
    let at_firing = magnitude_at(&out, firing, FS);
    // Two frequencies either side that are not orders of anything.
    let off =
        0.5 * (magnitude_at(&out, firing * 0.63, FS) + magnitude_at(&out, firing * 1.47, FS));
    assert!(
        at_firing > 4.0 * off,
        "induction is not pitched: {at_firing:.2e} at the firing order against {off:.2e} beside it"
    );
}

/// Runs a cold cross-plane V8 at `rpm` for `seconds`, returning its snapshot.
///
/// The whole warm-up chain in one call: Woschni's wall loss into the block,
/// the block into the chamber wall, the port gas into the pipe walls, and
/// the pipe walls back into the gas the audio path tunes on.
#[cfg(test)]
fn warmed_snapshot(seconds: f64, rpm: f64) -> EngineSnapshot {
    use crate::audio::{EngineControls, SnapshotSource};
    use crate::environment::Environment;
    use crate::physics::engine_block::EngineBlock;

    let mut block = EngineBlock::cross_plane_v8(Environment::default());
    block.cold_start();
    let mut source = SnapshotSource::new(&block);
    let dt = 1.0 / 120.0;
    let mut snapshot = EngineSnapshot::default();
    for _ in 0..(seconds / dt) as usize {
        block.update(dt, rpm);
        snapshot = source.sample(&block, rpm, dt, EngineControls::default());
    }
    snapshot
}

/// Settles a synth on a snapshot and reads back what its pipes are tuned to.
///
/// Returns `(primary, collector, tailpipe)` one-way delays [samples].
#[cfg(test)]
fn settled_delays(snapshot: &EngineSnapshot) -> (f32, f32, f32) {
    let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
    synth.set_snapshot(snapshot);
    // Every temperature in the synth glides on an 80 ms constant; a second
    // of audio leaves them all on target.
    render(&mut synth, FS as usize);
    (
        synth.network.primary_delay_samples(0),
        synth.network.collector_delay_samples(0),
        synth.network.tailpipe_delay_samples(0),
    )
}

#[test]
fn warming_up_raises_every_pipe_resonance_by_sqrt_of_the_ratio() {
    // Two seconds in the exhaust is still near ambient; three minutes has it
    // on its plateau. Nothing else about the engine differs.
    let cold = warmed_snapshot(2.0, 3_000.0);
    let hot = warmed_snapshot(180.0, 3_000.0);

    let (cold_prim, cold_coll, cold_tail) = settled_delays(&cold);
    let (hot_prim, hot_coll, hot_tail) = settled_delays(&hot);

    // A pipe of fixed length resonates at `c / 4L` with `c = sqrt(gamma R T)`,
    // so the whole of what warming does to its pitch is `sqrt(T2 / T1)` — and
    // the delay it is built on is the reciprocal of that. Each station is
    // checked against its own gas, because they are no longer the same gas.
    let stations: [(&str, f32, f32, f32, f32); 3] = [
        (
            "primary",
            cold.primary_temperature[0],
            hot.primary_temperature[0],
            cold_prim,
            hot_prim,
        ),
        (
            "collector",
            cold.collector_temperature,
            hot.collector_temperature,
            cold_coll,
            hot_coll,
        ),
        (
            "tailpipe",
            cold.tailpipe_temperature,
            hot.tailpipe_temperature,
            cold_tail,
            hot_tail,
        ),
    ];
    for (name, t_cold, t_hot, delay_cold, delay_hot) in stations {
        assert!(
            t_hot > 1.20 * t_cold,
            "{name} barely warmed: {t_cold:.0} K to {t_hot:.0} K"
        );
        let expected = (t_hot / t_cold).sqrt();
        let measured = delay_cold / delay_hot;
        assert!(
            (measured / expected - 1.0).abs() < 0.01,
            "{name} rose by {measured:.4}, not the sqrt({t_hot:.0}/{t_cold:.0}) = {expected:.4} \
             its gas temperature says"
        );
    }

    // And the gradient survives the trip through the snapshot: the back of
    // the system is cooler than the front, so it is also flatter.
    assert!(
        hot.primary_temperature[0] > hot.tailpipe_temperature,
        "the exhaust arrived at the audio path with no gradient in it"
    );
}

#[test]
fn enrichment_lowers_the_pipe_resonances() {
    use crate::audio::{EngineControls, SnapshotSource};
    use crate::environment::Environment;
    use crate::physics::engine_block::EngineBlock;

    let mut stoich_block = EngineBlock::cross_plane_v8(Environment::default());
    let mut rich_block = EngineBlock::cross_plane_v8(Environment::default());

    stoich_block.ecu.wot_afr = 14.7;
    stoich_block.ecu.stoich_afr = 14.7;
    stoich_block.ecu.idle_afr = 14.7;
    stoich_block.ecu.accel_enrichment_gain = 0.0;

    rich_block.ecu.wot_afr = 11.5;
    rich_block.ecu.stoich_afr = 11.5;
    rich_block.ecu.idle_afr = 11.5;
    rich_block.ecu.accel_enrichment_gain = 0.0;

    let dt = 1.0 / 120.0;
    let rpm = 4_000.0;
    let mut stoich_source = SnapshotSource::new(&stoich_block);
    let mut rich_source = SnapshotSource::new(&rich_block);

    let mut stoich_snap = EngineSnapshot::default();
    let mut rich_snap = EngineSnapshot::default();

    for _ in 0..(5.0 / dt) as usize {
        stoich_block.update(dt, rpm);
        rich_block.update(dt, rpm);
        stoich_snap = stoich_source.sample(&stoich_block, rpm, dt, EngineControls::wide_open());
        rich_snap = rich_source.sample(&rich_block, rpm, dt, EngineControls::wide_open());
    }

    assert!(
        rich_snap.exhaust_temperature < stoich_snap.exhaust_temperature,
        "enrichment must lower EGT: rich {:.1} K vs stoich {:.1} K",
        rich_snap.exhaust_temperature,
        stoich_snap.exhaust_temperature
    );

    let (stoich_prim, stoich_coll, stoich_tail) = settled_delays(&stoich_snap);
    let (rich_prim, rich_coll, rich_tail) = settled_delays(&rich_snap);

    assert!(
        rich_prim > stoich_prim,
        "primary resonance must be lowered by enrichment: delay {rich_prim:.2} > {stoich_prim:.2}"
    );
    assert!(
        rich_coll > stoich_coll,
        "collector resonance must be lowered by enrichment: delay {rich_coll:.2} > {stoich_coll:.2}"
    );
    assert!(
        rich_tail > stoich_tail,
        "tailpipe resonance must be lowered by enrichment: delay {rich_tail:.2} > {stoich_tail:.2}"
    );
}

#[test]
fn a_dead_cylinder_lopes_at_the_cycle_rate() {
    use crate::audio::{EngineControls, SnapshotSource};
    use crate::environment::Environment;
    use crate::physics::control::CylinderHealth;
    use crate::physics::engine_block::EngineBlock;

    let rpm = 2_400.0;
    let dt = 1.0 / 240.0;
    let f_cycle = (rpm / 120.0) as f32; // 20.0 Hz (order 0.5)

    // 1. Healthy engine
    let mut healthy_block = EngineBlock::cross_plane_v8(Environment::default());
    for _ in 0..400 {
        healthy_block.update(dt, rpm);
    }
    let mut healthy_source = SnapshotSource::new(&healthy_block);
    let mut healthy_synth = EngineSynth::new(SynthConfig::from_block(&healthy_block, FS));
    let frames = (FS * 2.0) as usize; // 2 seconds of audio
    let mut healthy_buf = vec![0.0f32; frames * 2];
    let chunk = 200;
    for c in 0..(frames / chunk) {
        healthy_block.update(dt, rpm);
        let snap = healthy_source.sample(&healthy_block, rpm, dt, EngineControls::wide_open());
        healthy_synth.set_snapshot(&snap);
        healthy_synth.render(&mut healthy_buf[c * chunk * 2..(c + 1) * chunk * 2], 2);
    }

    // 2. Engine with one dead cylinder (dead plug on cylinder 2)
    let mut dead_block = EngineBlock::cross_plane_v8(Environment::default());
    for _ in 0..400 {
        dead_block.update(dt, rpm);
    }
    dead_block.set_cylinder_health(2, CylinderHealth::dead_plug());
    let mut dead_source = SnapshotSource::new(&dead_block);
    let mut dead_synth = EngineSynth::new(SynthConfig::from_block(&dead_block, FS));
    let mut dead_buf = vec![0.0f32; frames * 2];
    for c in 0..(frames / chunk) {
        dead_block.update(dt, rpm);
        let snap = dead_source.sample(&dead_block, rpm, dt, EngineControls::wide_open());
        dead_synth.set_snapshot(&snap);
        dead_synth.render(&mut dead_buf[c * chunk * 2..(c + 1) * chunk * 2], 2);
    }

    // Measure energy at the cycle rate (order 0.5) and firing order in the settled half of the buffer
    let healthy_cycle_energy = magnitude_at(&healthy_buf[frames..], f_cycle, FS);
    let dead_cycle_energy = magnitude_at(&dead_buf[frames..], f_cycle, FS);

    // Over a narrow band rather than at the single bin. Combustion varies
    // cycle to cycle, so the firing order is not a line: its energy is
    // smeared across a couple of hertz, and a probe sitting on exactly
    // 160 Hz samples one point of that and swings by a quarter on burn
    // changes far too small to hear. The band is +/- one cycle rate,
    // which is the spacing of the sidebands the variation puts there.
    let f_firing = 8.0 * f_cycle; // 160.0 Hz (order 4.0)
    let firing_band = |buf: &[f32]| -> f32 {
        let mut total = 0.0;
        let mut n = 0;
        let mut offset = -f_cycle;
        while offset <= f_cycle + 1e-3 {
            total += magnitude_at(buf, f_firing + offset, FS);
            n += 1;
            offset += 0.5 * f_cycle;
        }
        total / n as f32
    };
    let healthy_firing = firing_band(&healthy_buf[frames..]);
    let dead_firing = firing_band(&dead_buf[frames..]);

    // A single dead cylinder shows as a missing order component and a lope at the cycle rate
    assert!(
        dead_firing < healthy_firing,
        "dead cylinder must show missing firing order component: dead {dead_firing:.4} < healthy {healthy_firing:.4}"
    );
    assert!(
        dead_cycle_energy > 3.0 * healthy_cycle_energy,
        "dead cylinder must lope at the cycle rate: dead {dead_cycle_energy:.4} vs healthy {healthy_cycle_energy:.4}"
    );
}

#[test]
fn hotter_exhaust_raises_every_resonance_by_sqrt_of_the_ratio() {
    let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
    let snapshot = loaded_snapshot().with_uniform_exhaust_temperature(400.0);
    synth.set_snapshot(&snapshot);
    render(&mut synth, 24_000);
    let cold_delay = synth.network.primary_delay_samples(0);

    let snapshot = snapshot.with_uniform_exhaust_temperature(1_200.0);
    synth.set_snapshot(&snapshot);
    render(&mut synth, 48_000);
    let hot_delay = synth.network.primary_delay_samples(0);

    // Every transit time in the network is L / c, so tripling the absolute
    // temperature shortens all of them — and lifts every resonance built on
    // them — by sqrt(T2 / T1).
    let expected = (1_200.0f32 / 400.0).sqrt();
    assert!(
        (cold_delay / hot_delay - expected).abs() < 0.05,
        "primary: {cold_delay}/{hot_delay}, expected a factor of {expected}"
    );
}

#[test]
fn buffer_size_does_not_change_the_output() {
    // The whole point of the control-block scheme is that it is invisible.
    // Rendering the same state in 64-frame and ragged chunks must agree.
    let make = || {
        let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
        synth.set_snapshot(&loaded_snapshot());
        synth
    };
    let frames = 20_000;
    let reference = render(&mut make(), frames);
    let ragged = render_chunked(&mut make(), frames, &[1, 7, 31, 32, 33, 128, 511]);

    assert_eq!(reference.len(), ragged.len());
    let worst = reference
        .iter()
        .zip(&ragged)
        .fold(0.0f32, |m, (a, b)| m.max((a - b).abs()));
    assert!(worst < 1e-6, "output depends on buffer size: {worst}");
}

/// A synth running at a fixed speed with no crank ripple and no jitter, so
/// the only thing moving the playback phase is the integrator.
fn steady_synth(rpm: f32) -> EngineSynth {
    let mut config = SynthConfig::cross_plane_v8(FS);
    config.combustion_variation_max = 0.0;
    config.combustion_variation_min = 0.0;
    let mut synth = EngineSynth::new(config);
    let mut snapshot = loaded_snapshot();
    snapshot.rpm = rpm;
    // No mean indicated torque means no intra-cycle speed ripple, so the
    // crank turns at exactly the nominal rate.
    snapshot.indicated_torque = 0.0;
    synth.set_snapshot(&snapshot);
    synth.cycle_hz.snap(rpm / 120.0);
    for (i, smoother) in synth.blowdown_pa.iter_mut().enumerate() {
        smoother.snap(snapshot.blowdown_delta[i]);
    }
    synth.cycle.blend.snap(1.0);
    synth.phase_fixed = 0;
    synth
}

#[test]
fn excitation_playback_does_not_drift_against_the_crank() {
    // The excitation is read at a phase the synth integrates itself, one
    // sample at a time, for as long as the stream is open. A part-per-
    // million bias in that integrator is inaudible for a second and a
    // quarter of a cycle out after twenty — which is how a synth that
    // sounded right in a test ends up with the banks of a vee engine
    // walking apart on a long drive.
    let rpm = 3_000.0f32;
    let mut synth = steady_synth(rpm);

    const SAMPLES: usize = 1_000_000;
    let mut buffer = vec![0.0f32; 2 * 1_000];
    for _ in 0..SAMPLES / 1_000 {
        synth.render(&mut buffer, 2);
    }

    // Cycles turned is time times the cycle rate, exactly.
    let cycles = SAMPLES as f64 * (rpm as f64 / 120.0) / FS as f64;
    let expected = cycles.rem_euclid(1.0) as f32;
    let drift = (synth.cycle_phase() - expected).abs();
    let drift = drift.min(1.0 - drift);
    assert!(
        drift < 1e-3,
        "playback phase drifted {drift} of a cycle over {SAMPLES} samples"
    );
}

#[test]
fn excitation_playback_tracks_crank_speed() {
    // One blowdown per cylinder per cycle, at whatever rate the crank is
    // turning. Nothing in the playback path sets a rate of its own.
    for rpm in [800.0f32, 3_000.0, 7_000.0] {
        let mut synth = steady_synth(rpm);
        let threshold = 0.5 * loaded_snapshot().blowdown_delta[0] / REFERENCE_BLOWDOWN;
        let seconds = 2.0;
        let samples = (seconds * FS) as usize;

        let mut buffer = [0.0f32; 2];
        synth.render(&mut buffer, 2);
        let mut above = synth.excitations[0] > threshold;
        let mut events = 0usize;
        for _ in 1..samples {
            synth.render(&mut buffer, 2);
            let now = synth.excitations[0] > threshold;
            if now && !above {
                events += 1;
            }
            above = now;
        }

        // A four-stroke cylinder fires once per two revolutions.
        let expected = seconds * rpm / 120.0;
        assert!(
            (events as f32 - expected).abs() <= 1.0,
            "{events} blowdowns in {seconds} s at {rpm} rpm, expected {expected}"
        );
    }
}

#[test]
fn a_new_cycle_fades_in_rather_than_stepping() {
    // The excitation is a *table* now, and the physics hands over a new one
    // at whatever rate its loop runs. Swapping it outright would put a step
    // into every cylinder's pulse at the physics frame rate, which is the
    // artefact the firing path exists to avoid.
    let mut config = SynthConfig::cross_plane_v8(FS);
    config.combustion_variation_max = 0.0;
    config.combustion_variation_min = 0.0;
    let mut synth = EngineSynth::new(config);
    let calm = loaded_snapshot();
    synth.set_snapshot(&calm);
    synth.cycle_hz.snap(calm.rpm / 120.0);
    for (i, smoother) in synth.blowdown_pa.iter_mut().enumerate() {
        smoother.snap(calm.blowdown_delta[i]);
    }
    synth.cycle.blend.snap(1.0);

    let cycle_samples = (FS / (calm.rpm / 120.0)) as usize;
    let mut buffer = [0.0f32; 2];
    let mut slew_of = |synth: &mut EngineSynth, samples: usize| {
        let mut worst = 0.0f32;
        let mut previous = synth.excitations[0];
        for _ in 0..samples {
            synth.render(&mut buffer, 2);
            worst = worst.max((synth.excitations[0] - previous).abs());
            previous = synth.excitations[0];
        }
        worst
    };
    // The steepest the blowdown edge itself gets: the bar every other step
    // in the excitation has to stay under.
    let steady = slew_of(&mut synth, 2 * cycle_samples);

    // A cycle off a different engine — the same pressure difference through
    // a valve event half as long.
    let (pressure, exhaust, intake) = synthetic_cycle(4.0e5, 120.0 / 720.0);
    let jumped = EngineSnapshot {
        cylinder_pressure: pressure,
        exhaust_port_flow: exhaust,
        intake_port_flow: intake,
        ..calm
    };

    // Hand the two cycles over alternately at phases that walk right through
    // cylinder 0's blowdown, because the worst step a swap can make is the
    // one made while the pulse it is swapping is happening.
    let mut worst = 0.0f32;
    for k in 0..16 {
        slew_of(&mut synth, cycle_samples / 16 + 3);
        synth.set_snapshot(if k % 2 == 0 { &jumped } else { &calm });
        worst = worst.max(slew_of(&mut synth, cycle_samples / 8));
    }

    assert!(
        worst <= 1.2 * steady,
        "a new cycle stepped the excitation: {worst} against a steady {steady}"
    );
}

#[test]
fn no_discontinuity_when_state_jumps() {
    // A snapshot that steps hard: idle to full load in one frame. Nothing in
    // the output may step with it.
    let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
    let idle = EngineSnapshot {
        rpm: 800.0,
        blowdown_delta: [0.6e5; MAX_CYLINDERS],
        exhaust_temperature: 600.0,
        intake_mass_flow: 0.02,
        throttle: 0.05,
        ..loaded_snapshot()
    };
    synth.set_snapshot(&idle);
    let settle = render(&mut synth, 48_000);
    let quiet_slew = max_slew(&settle, 2);

    let mut hot = loaded_snapshot();
    hot.rpm = 7_000.0;
    hot.blowdown_delta = [6.0e5; MAX_CYLINDERS];
    hot.exhaust_temperature = 1_250.0;
    hot.intake_mass_flow = 0.45;
    hot.throttle = 1.0;
    hot.turbo_rpm = 160_000.0;
    // The cycle itself jumps too, and by more than any real frame could: a
    // different pressure curve through a valve event half as long. The
    // tables are the excitation now, so a step in them is a step in the
    // output unless something is bridging them.
    let (pressure, exhaust, intake) = synthetic_cycle(12.0e5, 120.0 / 720.0);
    hot.cylinder_pressure = pressure;
    hot.exhaust_port_flow = exhaust;
    hot.intake_port_flow = intake;
    synth.set_snapshot(&hot);
    let after = render(&mut synth, 48_000);

    assert!(after.iter().all(|s| s.is_finite()));
    // The loud state legitimately has faster slew than idle, but a parameter
    // step must not produce a full-scale jump.
    let jump = max_slew(&after, 2);
    assert!(jump < 0.9, "step discontinuity in output: {jump}");
    assert!(quiet_slew < 0.5);
}

#[test]
fn starvation_keeps_the_engine_running() {
    // Render ten seconds without ever updating the snapshot; the note must
    // continue, not decay to silence or freeze on a DC value.
    let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
    synth.set_snapshot(&loaded_snapshot());
    render(&mut synth, 48_000);
    let early = rms(&render(&mut synth, 48_000));
    let late = rms(&render(&mut synth, 9 * 48_000));
    assert!(late > 0.5 * early, "coasting decayed: {early} -> {late}");
    assert!(late < 2.0 * early, "coasting grew: {early} -> {late}");

    // The excitation is played from a table now, and a table that stops
    // being refreshed is still a table. What must not happen is the
    // playback freezing on it: a held phase would present the pipes with one
    // slice of the blowdown as a DC offset, which is silence with a step in
    // front of it rather than an engine.
    let mut buffer = [0.0f32; 2];
    let (mut low, mut high) = (f32::MAX, f32::MIN);
    for _ in 0..48_000 {
        synth.render(&mut buffer, 2);
        low = low.min(synth.excitations[0]);
        high = high.max(synth.excitations[0]);
    }
    let amplitude = loaded_snapshot().blowdown_delta[0] / REFERENCE_BLOWDOWN;
    assert!(
        low < 0.05 * amplitude && high > 0.5 * amplitude,
        "starved playback stopped swinging: {low} to {high}"
    );
}

#[test]
fn the_induction_derivative_is_not_a_staircase() {
    // The water hammer at valve closing is driven by `dmdot/dt`. Taking
    // that by differencing the *interpolated* flow sample to sample
    // differentiates a piecewise-linear signal, and the result is constant
    // between table points and steps at every one of them. A staircase is a
    // train of discontinuities at `CYCLE_TABLE` times the cycle rate — 3.2
    // kHz at 3000 rpm — and it lands most of its energy in the band the
    // induction note occupies.
    //
    // Differencing the table and interpolating that instead keeps the
    // derivative inside the bandwidth the table carries, which is what this
    // measures: the excitation's energy above the table's own Nyquist,
    // against its energy below.
    let snapshot = loaded_snapshot();
    let tables = CycleTables::from_snapshot(&snapshot);
    let cycle_hz = snapshot.rpm / 120.0;
    let table_nyquist = 0.5 * CYCLE_TABLE as f32 * cycle_hz;

    // Play the slope back the way the synth does, for one whole cycle.
    let samples = (FS / cycle_hz) as usize;
    let slope: Vec<f32> = (0..samples)
        .map(|i| tables.intake_slope_at(i as f32 / samples as f32) * cycle_hz)
        .collect();

    let band = |lo: f32, hi: f32| {
        let mut total = 0.0f64;
        let mut hz = lo;
        while hz < hi {
            let m = {
                let (mut re, mut im) = (0.0f64, 0.0f64);
                for (n, &x) in slope.iter().enumerate() {
                    let phase = TAU * hz * n as f32 / FS;
                    re += x as f64 * phase.cos() as f64;
                    im -= x as f64 * phase.sin() as f64;
                }
                (re * re + im * im).sqrt() / slope.len() as f64
            };
            total += m * m;
            hz += cycle_hz;
        }
        total.sqrt()
    };

    let inside = band(cycle_hz, table_nyquist);
    let outside = band(table_nyquist, 4.0 * table_nyquist);
    let leak_db = 20.0 * (outside / inside.max(1e-30)).log10();
    assert!(
        leak_db < -20.0,
        "the induction derivative put {leak_db:.1} dB above the table's own \
         Nyquist, which is a staircase and not a rate"
    );
}

#[test]
fn snapshot_stays_copy_and_free_of_indirection() {
    fn assert_copy<T: Copy>() {}
    assert_copy::<EngineSnapshot>();

    // The tables are what make this worth asserting. A `Vec` would be one
    // word here and a heap allocation everywhere else, and its `Drop` would
    // run inside the audio callback — which is the one thing the callback is
    // not allowed to do.
    assert!(!std::mem::needs_drop::<EngineSnapshot>());

    // Five cycle tables — pressure, the two port flows and the two valve
    // areas — the per-cylinder blowdown, intake flow and primary
    // temperature arrays, twenty scalars and one padded bool. Every byte
    // accounted for is a byte that is not a pointer.
    let expected = 6 * CYCLE_TABLE * 4 + 3 * MAX_CYLINDERS * 4 + 20 * 4 + 4;
    assert_eq!(std::mem::size_of::<EngineSnapshot>(), expected);
}

#[test]
fn nan_snapshot_cannot_reach_the_output() {
    let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
    synth.set_snapshot(&EngineSnapshot {
        rpm: f32::NAN,
        blowdown_delta: [f32::INFINITY; MAX_CYLINDERS],
        exhaust_temperature: f32::NAN,
        primary_temperature: [f32::NAN; MAX_CYLINDERS],
        collector_temperature: f32::INFINITY,
        tailpipe_temperature: f32::NAN,
        exhaust_gamma: -1.0,
        exhaust_gas_constant: 0.0,
        intake_mass_flow: f32::NAN,
        cylinder_intake_flow: [f32::NAN; MAX_CYLINDERS],
        throttle: f32::INFINITY,
        turbo_rpm: f32::NAN,
        turbo_surge: f32::NAN,
        unburnt_fuel_mass: f32::NAN,
        friction_mep: f32::NAN,
        cold_fraction: f32::NAN,
        starter_hz: f32::NAN,
        spark_cut: true,
        knock_intensity: f32::NAN,
        bore: f32::NAN,
        peak_cylinder_pressure: f32::NAN,
        indicated_torque: f32::NAN,
        inertia: f32::NAN,
        cylinder_pressure: [f32::NAN; CYCLE_TABLE],
        exhaust_port_flow: [f32::NEG_INFINITY; CYCLE_TABLE],
        intake_port_flow: [f32::NAN; CYCLE_TABLE],
        exhaust_valve_area: [f32::NAN; CYCLE_TABLE],
        intake_valve_area: [f32::NEG_INFINITY; CYCLE_TABLE],
        cylinder_volume: [f32::NAN; CYCLE_TABLE],
        exhaust_manifold_pressure: f32::NAN,
        exhaust_cutout: false,
        anti_lag: false,
    });
    let out = render(&mut synth, 48_000);
    assert!(out.iter().all(|s| s.is_finite()), "NaN reached the device");
    assert!(peak(&out) <= 1.0);
}

#[test]
fn extreme_speeds_do_not_break_the_trigger() {
    for rpm in [0.0, 1.0, 500.0, 12_000.0, 30_000.0] {
        let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
        let mut snapshot = loaded_snapshot();
        snapshot.rpm = rpm;
        synth.set_snapshot(&snapshot);
        let out = render(&mut synth, 24_000);
        assert!(out.iter().all(|s| s.is_finite()), "broke at {rpm} rpm");
        assert!(peak(&out) <= 1.0, "clipped at {rpm} rpm");
    }
}

#[test]
fn backfires_need_fuel_heat_and_a_spark_cut() {
    let count_pops = |snapshot: EngineSnapshot| {
        let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
        // Silence everything but the pops so they can be counted by level.
        // The turbo needs no silencing: the V8 is naturally aspirated.
        synth.config.exhaust_level = 0.0;
        synth.exhaust_level.snap(0.0);
        synth.config.intake_level = 0.0;
        synth.config.mechanical_level = 0.0;
        synth.config.structure_level = 0.0;
        synth.set_snapshot(&snapshot);
        let out = render(&mut synth, 5 * 48_000);
        peak(&out)
    };

    let mut primed = loaded_snapshot();
    primed.spark_cut = true;
    primed.unburnt_fuel_mass = 30.0e-6;
    primed.exhaust_temperature = 1_150.0;

    // Exhaust level is zeroed, so anything audible came from a backfire
    // injected into the same bank chain... which is also silenced. Instead
    // check the decision logic directly.
    assert_eq!(count_pops(primed), 0.0);

    let mut voice = BackfireVoice::new(FS);
    let config = SynthConfig::cross_plane_v8(FS);
    let mut noise = Noise::new(7);

    voice.tune(&config, &primed);
    assert!(voice.severity > 0.0, "primed exhaust should be able to pop");

    let mut fired = 0;
    for _ in 0..(5 * 48_000 / CONTROL_BLOCK) {
        if voice.poll(&mut noise, CONTROL_BLOCK).is_some() {
            fired += 1;
        }
    }
    assert!((50..=450).contains(&fired), "implausible pop rate: {fired}");

    // Each condition on its own must produce nothing.
    for spoiler in [
        EngineSnapshot {
            spark_cut: false,
            ..primed
        },
        EngineSnapshot {
            unburnt_fuel_mass: 0.0,
            ..primed
        },
        EngineSnapshot {
            exhaust_temperature: 500.0,
            ..primed
        },
    ] {
        let mut voice = BackfireVoice::new(FS);
        voice.tune(&config, &spoiler);
        assert_eq!(voice.severity, 0.0);
        assert!(voice.poll(&mut noise, CONTROL_BLOCK).is_none());
    }
}

#[test]
fn backfire_pop_attack_is_sub_millisecond_shock() {
    let mut pop = Pop::default();
    // Attack of 0.18 ms is ~8.6 samples at 48 kHz.
    pop.trigger(FS, 1.0, 0.00018, 0.010, 0.0, 0.0);
    let mut peak_val = 0.0f32;
    let mut peak_idx = 0;
    for i in 0..100 {
        let val = pop.process(0.0);
        if val > peak_val {
            peak_val = val;
            peak_idx = i;
        }
    }
    let rise_time_sec = peak_idx as f32 / FS;
    assert!(
        rise_time_sec < 0.0008,
        "pop attack must be an explosive shock under 0.8 ms: {rise_time_sec:.6} s ({peak_idx} samples)"
    );
    assert!(
        peak_val > 0.8,
        "pop should reach near-unit normalized peak: {peak_val}"
    );
}

#[test]
fn backfire_pulse_has_no_energy_above_nyquist_bandlimit() {
    // A steady stream of overlapping pops, rendered alone (no engine, no
    // exhaust network), so any energy above the bandlimit is the pulse's
    // own synthesis and nothing else.
    let mut pool = PopPool::default();
    let mut noise = Noise::new(3);
    let n = 4 * 48_000;
    let mut samples = Vec::with_capacity(n);
    let mut cooldown = 0usize;
    for _ in 0..n {
        if cooldown == 0 {
            pool.trigger(FS, 1.0, 0.00018, 0.010, 0.80, 0.0);
            cooldown = (0.02 * FS) as usize;
        } else {
            cooldown -= 1;
        }
        samples.push(pool.process(&mut noise));
    }

    let spectrum = crate::analysis::orders::AverageSpectrum::of(&samples, FS as f64);
    let passband = spectrum.db_at(3_000.0).unwrap();
    let edge = spectrum.db_at(0.45 * FS as f64).unwrap();
    let floor = spectrum.db_at(0.48 * FS as f64).unwrap();
    assert!(
        passband - edge > 60.0,
        "expected the pulse's real content well clear of 0.45 * fs: passband {passband:.1} dB, edge {edge:.1} dB"
    );
    assert!(
        (edge - floor).abs() < 1.0,
        "expected 0.45 * fs to already sit at the render's own noise floor: edge {edge:.1} dB, floor {floor:.1} dB"
    );
}

#[test]
fn backfire_ages_one_sample_apart_are_a_pure_delay() {
    // Two pops a sample apart in `age` must differ by exactly one sample
    // of delay and nothing else — no amplitude step from quantising to a
    // control-rate grid, which is what sample-accurate triggering buys.
    let mut younger = Pop::default();
    let mut older = Pop::default();
    younger.trigger(FS, 1.0, 0.00018, 0.010, 0.0, 3.0);
    older.trigger(FS, 1.0, 0.00018, 0.010, 0.0, 4.0);

    let samples: Vec<(f32, f32)> = (0..64)
        .map(|_| (younger.process(0.0), older.process(0.0)))
        .collect();
    for i in 0..samples.len() - 1 {
        let (_, older_now) = samples[i];
        let (younger_next, _) = samples[i + 1];
        assert!(
            (older_now - younger_next).abs() < 1e-5,
            "sample {i}: older pop ({older_now}) should equal the younger pop \
             one sample later ({younger_next}), not a quantised amplitude step"
        );
    }
}

#[test]
fn ignition_node_moves_downstream_as_the_cut_ages_and_back_with_charge_size() {
    // A charge that has just been dumped is large and lights at the port;
    // the same severity given time to convect lights further down the
    // pipe. A bigger charge pulls the reach back toward the port at any
    // given age, because it does not need the extra distance to ignite.
    assert_eq!(choose_injection_node(1.0, 0.0), InjectionNode::Port);
    assert_eq!(
        choose_injection_node(0.0, IGNITION_TRAVEL_SECONDS),
        InjectionNode::Tailpipe
    );
    assert_eq!(
        choose_injection_node(1.0, IGNITION_TRAVEL_SECONDS),
        InjectionNode::Collector,
        "a bigger charge should not have reached as far downstream as a \
         smaller one at the same age"
    );
}

#[test]
fn larger_unburnt_mass_produces_a_slower_backfire_attack() {
    // A bigger charge takes longer to burn through regardless of where it
    // lights: this must be a genuinely different, slower shock, not the
    // same shock made quieter or louder.
    let quiet = backfire_attack_seconds(0.0, InjectionNode::Port);
    let loud = backfire_attack_seconds(1.0, InjectionNode::Port);
    assert!(
        loud > quiet,
        "a bigger unburnt charge should slow the attack: {quiet} -> {loud}"
    );
    assert!(
        loud / quiet > 1.5,
        "attack should meaningfully lengthen with charge size, not barely \
         move: {quiet} -> {loud}"
    );

    // Holds at every node, not just the port.
    for node in [
        InjectionNode::Port,
        InjectionNode::Collector,
        InjectionNode::PostSilencer,
        InjectionNode::Tailpipe,
    ] {
        assert!(
            backfire_attack_seconds(1.0, node) > backfire_attack_seconds(0.0, node),
            "attack should grow with charge size at {node:?}"
        );
    }
}

#[test]
fn one_cut_fires_more_than_one_event_with_non_constant_spacing() {
    // Fuel accumulates, then lights, then the flame keeps propagating: a
    // single detected ignition can spawn a companion further down the
    // pipe, which is what turns one cut into a burst rather than a click.
    let mut primed = loaded_snapshot();
    primed.spark_cut = true;
    primed.unburnt_fuel_mass = 30.0e-6;
    primed.exhaust_temperature = 1_150.0;
    let config = SynthConfig::cross_plane_v8(FS);

    let mut voice = BackfireVoice::new(FS);
    let mut noise = Noise::new(11);

    let blocks = 2 * 48_000 / CONTROL_BLOCK;
    let mut fire_samples = Vec::new();
    for b in 0..blocks {
        voice.tune(&config, &primed);
        if voice.poll(&mut noise, CONTROL_BLOCK).is_some() {
            fire_samples.push(b * CONTROL_BLOCK);
        }
    }

    assert!(
        fire_samples.len() >= 2,
        "a 2 s cut should fire more than one event: {}",
        fire_samples.len()
    );
    let gaps: Vec<usize> = fire_samples.windows(2).map(|w| w[1] - w[0]).collect();
    let all_equal = gaps.windows(2).all(|w| w[0] == w[1]);
    assert!(
        !all_equal,
        "inter-event spacing should not be constant: {gaps:?}"
    );
    // A companion fires inside the primary ignition's own 12 ms
    // refractory window, which a second independent Poisson draw cannot:
    // finding a gap shorter than that is proof a companion fired.
    let refractory_samples = (0.012 * FS) as usize;
    assert!(
        gaps.iter().any(|&g| g < refractory_samples),
        "expected at least one companion inside the primary's refractory \
         window ({refractory_samples} samples): {gaps:?}"
    );
}

#[test]
fn anti_lag_produces_a_periodic_port_train_a_limiter_cut_does_not() {
    // Anti-lag routes to the port and is locked to firing, not to the
    // Poisson process `BackfireVoice` runs during a cut. Counted through
    // `PopPool::next`, which advances by exactly one on every `trigger`
    // call regardless of how the envelopes it started overlap.
    let trigger_samples = |anti_lag: bool, spark_cut: bool| -> Vec<usize> {
        let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
        let mut snapshot = loaded_snapshot();
        snapshot.anti_lag = anti_lag;
        snapshot.spark_cut = spark_cut;
        // Removes crank-speed ripple, so the firing instants this test
        // checks for even spacing land on an exact period.
        snapshot.indicated_torque = 0.0;
        if spark_cut {
            snapshot.unburnt_fuel_mass = 30.0e-6;
            snapshot.exhaust_temperature = 1_150.0;
        }
        synth.set_snapshot(&snapshot);

        let mut buffer = [0.0f32; 2];
        // Let `cycle_hz` glide onto the snapshot's rpm before measuring.
        for _ in 0..(FS * 0.3) as usize {
            synth.render(&mut buffer, 2);
        }

        let mut samples = Vec::new();
        let mut last_next = synth.antilag_pulses[0].next;
        for i in 0..FS as usize {
            synth.render(&mut buffer, 2);
            let now_next = synth.antilag_pulses[0].next;
            if now_next != last_next {
                samples.push(i);
                last_next = now_next;
            }
        }
        samples
    };

    let firing = trigger_samples(true, false);
    assert!(
        firing.len() > 10,
        "anti-lag should fire a steady train of port events over 1 s: {}",
        firing.len()
    );
    // Locked to firing, not evenly spaced: the cross-plane V8's bank
    // fires its four cylinders at uneven intervals within a cycle (that
    // unevenness is what "cross-plane" means), so the proof of lock is
    // that the same four-gap pattern repeats exactly every cycle, not
    // that the gaps are equal.
    let gaps: Vec<i64> = firing
        .windows(2)
        .map(|w| w[1] as i64 - w[0] as i64)
        .collect();
    let cylinders_per_bank = 4;
    assert!(
        gaps.len() > cylinders_per_bank * 3,
        "need several cycles' worth of gaps to check repetition: {gaps:?}"
    );
    for i in 0..(gaps.len() - cylinders_per_bank) {
        let this_cycle = gaps[i];
        let next_cycle = gaps[i + cylinders_per_bank];
        assert!(
            (this_cycle - next_cycle).abs() <= 2,
            "anti-lag's port train should repeat the same firing pattern \
             every cycle: gap {i} was {this_cycle} samples, {next_cycle} \
             one cycle later: {gaps:?}"
        );
    }

    let cutting = trigger_samples(false, true);
    assert!(
        cutting.is_empty(),
        "a limiter cut without anti-lag should not produce any port-node \
         train: {} events",
        cutting.len()
    );
}

#[test]
fn every_backfire_node_is_bandlimited_to_the_t6_standard() {
    // T6 established one bandlimit for the pop: no energy above 0.45 * fs,
    // clear of the passband by 60 dB. Every node's pool shares that same
    // filter design (`PopPool` is node-agnostic), so triggering each in
    // turn and rendering it alone must all measure the same standard.
    for node in [
        InjectionNode::Port,
        InjectionNode::Collector,
        InjectionNode::PostSilencer,
        InjectionNode::Tailpipe,
    ] {
        let mut pool = PopPool::default();
        let mut noise = Noise::new(5);
        let n = 4 * 48_000;
        let mut samples = Vec::with_capacity(n);
        let mut cooldown = 0usize;
        let attack = backfire_attack_seconds(0.5, node);
        for _ in 0..n {
            if cooldown == 0 {
                pool.trigger(FS, 1.0, attack, 0.010, 0.80, 0.0);
                cooldown = (0.02 * FS) as usize;
            } else {
                cooldown -= 1;
            }
            samples.push(pool.process(&mut noise));
        }

        let spectrum = crate::analysis::orders::AverageSpectrum::of(&samples, FS as f64);
        let passband = spectrum.db_at(3_000.0).unwrap();
        let edge = spectrum.db_at(0.45 * FS as f64).unwrap();
        assert!(
            passband - edge > 60.0,
            "{node:?}: expected the pulse's real content well clear of \
             0.45 * fs: passband {passband:.1} dB, edge {edge:.1} dB"
        );
    }
}

/// T6's re-levelling target on the reference case (cross-plane V8, 3000 rpm
/// loaded) `docs/TIMBRE_PLAN.md` measured its 13 dB complaint against: a
/// limiter-bounce section a few dB above the fired engine, not 13+.
#[test]
fn backfire_contrast_against_the_fired_engine_lands_a_few_db_up() {
    let mono_db_and_crest = |snapshot: &EngineSnapshot| -> (f32, f64) {
        let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
        synth.set_snapshot(snapshot);
        let stereo = render(&mut synth, 2 * 48_000);
        // Skip the first 100 ms so a filter's own settle-in is not read as
        // part of the level.
        let mono: Vec<f32> = stereo[9_600..]
            .chunks_exact(2)
            .map(|c| 0.5 * (c[0] + c[1]))
            .collect();
        let rms = rms(&mono);
        let db = if rms > 1e-6 {
            20.0 * rms.log10()
        } else {
            -99.9
        };
        (db, crate::analysis::orders::crest_db(&mono))
    };

    let fired = loaded_snapshot();
    let mut cut = loaded_snapshot();
    cut.rpm = 6_000.0;
    cut.spark_cut = true;
    cut.unburnt_fuel_mass = 30.0e-6;
    cut.exhaust_temperature = 1_150.0;

    let (fired_db, _) = mono_db_and_crest(&fired);
    let (cut_db, cut_crest) = mono_db_and_crest(&cut);
    let contrast = cut_db - fired_db;

    assert!(
        (2.0..=10.0).contains(&contrast),
        "expected the limiter bounce a few dB above the fired engine, not \
         the old 13+ dB firecracker: fired {fired_db:.1} dBFS, cut {cut_db:.1} \
         dBFS, contrast {contrast:.1} dB"
    );
    // Bandlimiting the pulse (T6's first three commits) must not turn the
    // crack into a thud: it should still read as a sharp transient, not a
    // flattened one.
    assert!(
        cut_crest > 12.0,
        "limiter bounce should still be a sharp crack, not a thud: crest \
         {cut_crest:.1} dB"
    );
}

#[test]
fn control_block_is_shorter_than_v12_firing_interval() {
    // A V12 at 8000 rpm fires every 1.25 ms:
    // f_cycle = 8000 / 120 = 66.67 Hz -> 12 * 66.67 = 800 Hz -> T_fire = 1.25 ms.
    // At 48 kHz, this is 60 samples.
    // The control block must be strictly shorter than the firing interval so
    // control-rate schedules and filters are not quantised coarser than the events
    // they track.
    const FS: f32 = 48_000.0;
    let v12_firing_interval_sec = 120.0 / (12.0 * 8000.0); // 1.25 ms
    let control_block_sec = CONTROL_BLOCK as f32 / FS;
    assert!(
        control_block_sec < v12_firing_interval_sec,
        "CONTROL_BLOCK ({control_block_sec:.4} s) must be shorter than V12 firing interval ({v12_firing_interval_sec:.4} s)"
    );
    let firing_samples = (v12_firing_interval_sec * FS) as usize;
    assert!(
        CONTROL_BLOCK < firing_samples,
        "CONTROL_BLOCK ({CONTROL_BLOCK}) must be fewer samples than firing interval ({firing_samples})"
    );
}

#[test]
fn surge_flutter_modulates_at_the_surge_rate() {
    let mut config = SynthConfig::cross_plane_v8(FS);
    // The catalogue V8 is atmospheric, so this test has to fit the turbo it
    // means to measure.
    config.turbo = Some(TurboVoicing::default());
    let mut synth = EngineSynth::new(config);
    synth.config.exhaust_level = 0.0;
    synth.exhaust_level.snap(0.0);
    synth.config.intake_level = 0.0;
    // The mechanical floor is continuous by construction; it would fill in
    // the gaps between chuffs and hide exactly what this measures.
    synth.config.mechanical_level = 0.0;
    let mut snapshot = loaded_snapshot();
    snapshot.blowdown_delta = [0.0; MAX_CYLINDERS];
    snapshot.intake_mass_flow = 0.0;
    snapshot.turbo_rpm = 120_000.0;
    snapshot.turbo_surge = 1.0;
    synth.set_snapshot(&snapshot);
    render(&mut synth, 48_000);

    let out = render(&mut synth, 48_000);
    // The chuff envelope should make the level swing substantially within a
    // second, at a rate in the surge band.
    let window = (FS / 400.0) as usize; // 2.5 ms
    let levels: Vec<f32> = out
        .chunks(window * 2)
        .map(|c| c.iter().fold(0.0f32, |m, s| m.max(s.abs())))
        .collect();
    let hi = levels.iter().cloned().fold(0.0f32, f32::max);
    let lo = levels.iter().cloned().fold(f32::MAX, f32::min);
    assert!(hi > 1e-4, "no turbo output at all");
    assert!(lo < 0.6 * hi, "flutter did not modulate: {lo} vs {hi}");
}

#[test]
fn only_a_fitted_turbo_makes_turbo_sound() {
    // Everything but the turbo silenced, and a shaft spinning hard enough
    // that a fitted one would be at full song.
    let level_with = |turbo: Option<TurboVoicing>| {
        let mut config = SynthConfig::cross_plane_v8(FS);
        config.turbo = turbo;
        config.exhaust_level = 0.0;
        config.intake_level = 0.0;
        config.mechanical_level = 0.0;
        config.structure_level = 0.0;
        let mut synth = EngineSynth::new(config);
        synth.exhaust_level.snap(0.0);
        let mut snapshot = loaded_snapshot();
        snapshot.blowdown_delta = [0.0; MAX_CYLINDERS];
        snapshot.intake_mass_flow = 0.0;
        snapshot.turbo_rpm = 150_000.0;
        snapshot.turbo_surge = 0.8;
        synth.set_snapshot(&snapshot);
        render(&mut synth, 48_000); // settle
        peak(&render(&mut synth, 48_000))
    };

    // Not "quiet": an engine with no compressor on it has nothing to make
    // the sound with, so the only correct level is exactly zero.
    assert_eq!(
        level_with(None),
        0.0,
        "an atmospheric engine whistled anyway"
    );
    assert!(
        level_with(Some(TurboVoicing::default())) > 1e-3,
        "a fitted turbo made no sound"
    );
}

/// An idle snapshot with a plausible warm FMEP behind it.
fn idle_snapshot() -> EngineSnapshot {
    EngineSnapshot {
        rpm: 800.0,
        blowdown_delta: [1.0e5; MAX_CYLINDERS],
        intake_mass_flow: 0.025,
        throttle: 0.04,
        turbo_rpm: 0.0,
        // Chen-Flynn at a warm idle: ~0.73 bar (~29 N m friction torque).
        friction_mep: 0.73e5,
        indicated_torque: 30.0,
        ..loaded_snapshot()
    }
    .with_uniform_exhaust_temperature(700.0)
}

/// Peak level inside each successive window of `window` frames.
fn window_peaks(samples: &[f32], window: usize) -> Vec<f32> {
    samples
        .chunks(window * 2)
        .map(|c| c.iter().fold(0.0f32, |m, s| m.max(s.abs())))
        .collect()
}

/// Coefficient of variation of a series, `sigma / mean`.
fn coefficient_of_variation(values: &[f32]) -> f32 {
    let n = values.len().max(1) as f64;
    let mean = values.iter().map(|v| *v as f64).sum::<f64>() / n;
    if mean <= 1e-12 {
        return 0.0;
    }
    let var = values
        .iter()
        .map(|v| (*v as f64 - mean) * (*v as f64 - mean))
        .sum::<f64>()
        / n;
    (var.sqrt() / mean) as f32
}

// -- cycle-to-cycle combustion variation ---------------------------------

#[test]
fn combustion_variation_depth_follows_the_speed_schedule() {
    let synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
    let config = synth.config().clone();

    // Full depth at and below the idle speed.
    approx(
        synth.variation_depth_at(config.combustion_variation_idle_rpm as f32),
        config.combustion_variation_max as f32,
    );
    approx(
        synth.variation_depth_at(300.0),
        config.combustion_variation_max as f32,
    );
    // The floor depth as it reaches the threshold, and nothing above it.
    assert!(
        (synth.variation_depth_at(CCV_THRESHOLD_RPM - 1.0)
            - config.combustion_variation_min as f32)
            .abs()
            < 1e-3
    );
    assert_eq!(synth.variation_depth_at(CCV_THRESHOLD_RPM), 0.0);
    assert_eq!(synth.variation_depth_at(6_000.0), 0.0);

    // Monotone in between: an engine cannot get rougher as it slows down
    // past some interior speed.
    let mut previous = 0.0;
    let mut rpm = CCV_THRESHOLD_RPM - 1.0;
    while rpm >= 400.0 {
        let d = synth.variation_depth_at(rpm);
        assert!(d >= previous - 1e-6, "depth fell as speed fell at {rpm}");
        previous = d;
        rpm -= 10.0;
    }
}

#[test]
fn combustion_variation_draws_match_the_requested_sigma() {
    let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
    let depth = 0.08;
    let spacing = 1.0 / 8.0;

    let n = 40_000;
    let mut amplitudes = Vec::with_capacity(n);
    let mut phases = Vec::with_capacity(n);
    for _ in 0..n {
        synth.reroll_variation(0, depth);
        amplitudes.push(synth.variation[0].amplitude_scale);
        phases.push(synth.variation[0].phase_offset);
    }

    let stats = |v: &[f32]| {
        let n = v.len() as f64;
        let mean = v.iter().map(|x| *x as f64).sum::<f64>() / n;
        let var = v.iter().map(|x| (*x as f64 - mean).powi(2)).sum::<f64>() / n;
        (mean as f32, var.sqrt() as f32)
    };

    // Amplitude: unbiased around nominal, with the requested spread (reduced
    // to the stochastic remainder, half the total depth). The tolerance on
    // the mean is three standard errors of the estimator itself, sigma/sqrt(n).
    let amplitude_sigma = 0.5 * depth;
    let standard_error = amplitude_sigma / (n as f32).sqrt();
    let (mean, sigma) = stats(&amplitudes);
    assert!(
        (mean - 1.0).abs() < 3.0 * standard_error,
        "amplitude draw is biased: {mean}"
    );
    assert!(
        (sigma - amplitude_sigma).abs() < 0.004,
        "amplitude sigma {sigma}, want {amplitude_sigma}"
    );

    // Phase: sigma is the depth as a fraction of the firing interval, so at
    // 8 % on a V8 it is 8 % of 45 crank degrees — about 3.6 degrees.
    let (mean, sigma) = stats(&phases);
    assert!(
        mean.abs() < 3.0 * standard_error * spacing,
        "phase draw is biased: {mean}"
    );
    assert!(
        (sigma - depth * spacing).abs() < 0.0006,
        "phase sigma {sigma}, want {}",
        depth * spacing
    );

    // Nothing ever escapes the guard rails.
    assert!(amplitudes.iter().all(|a| (1.0 - CCV_MAX_AMPLITUDE_EXCURSION
        ..=1.0 + CCV_MAX_AMPLITUDE_EXCURSION)
        .contains(a)));
    let bound = CCV_MAX_PHASE_FRACTION * spacing;
    assert!(phases.iter().all(|p| p.abs() <= bound));

    // Zero depth is the nominal cycle, exactly.
    synth.reroll_variation(0, 0.0);
    assert_eq!(synth.variation[0].amplitude_scale, 1.0);
    assert_eq!(synth.variation[0].phase_offset, 0.0);
}

#[test]
fn every_cylinder_fires_exactly_once_per_cycle_however_it_is_jittered() {
    // The invariant firing jitter must not break. The jitter shifts the
    // phase each cylinder *reads* the cycle at; it must never make a
    // cylinder's excitation come round twice in one cycle, or skip one.
    let mut config = SynthConfig::cross_plane_v8(FS);
    // Well past anything the speed schedule would ask for, so the draw is
    // hitting its clamps rather than sitting near nominal.
    config.combustion_variation_max = 0.25;
    config.combustion_variation_min = 0.25;
    let mut synth = EngineSynth::new(config);
    synth.set_snapshot(&idle_snapshot());
    // Start exactly on a cycle boundary at a settled speed, so the count is
    // a whole number of cycles by construction.
    synth.cycle_hz.snap(800.0 / 120.0);
    for (i, smoother) in synth.blowdown_pa.iter_mut().enumerate() {
        smoother.snap(idle_snapshot().blowdown_delta[i]);
    }
    synth.cycle.blend.snap(1.0);
    // Open the window in the gap between two cylinders' events rather than
    // on top of one. The jitter moves each event up to 0.0375 of a cycle
    // either way, and an event straddling the boundary would be counted at
    // both ends — an artefact of where the measurement starts, not of the
    // firing path.
    synth.phase_fixed = (0.09 * PHASE_ONE) as u32;

    let cycles = 5usize;
    let samples = (FS / (800.0 / 120.0)) as usize * cycles;
    let n = synth.config().cylinder_count();
    // Half the peak of a curve normalised to one: comfortably inside the
    // blowdown and comfortably above the exhaust stroke that follows it.
    let threshold = (idle_snapshot().blowdown_delta[0] / REFERENCE_BLOWDOWN) * 0.5;
    let mut fires = vec![0usize; n];
    let mut buffer = [0.0f32; 2];
    // Seed from the state the window actually opens in. A cylinder whose
    // blowdown is already under way at sample zero fired before the window,
    // not inside it.
    synth.render(&mut buffer, 2);
    let mut above: Vec<bool> = (0..n)
        .map(|i| synth.excitations[i] > threshold * synth.launch.value())
        .collect();
    for _ in 1..samples {
        synth.render(&mut buffer, 2);
        for i in 0..n {
            let now = synth.excitations[i] > threshold * synth.launch.value();
            if now && !above[i] {
                fires[i] += 1;
            }
            above[i] = now;
        }
    }

    for (i, count) in fires.iter().enumerate() {
        assert_eq!(
            *count, cycles,
            "cylinder {i} excited {count} times over {cycles} cycles"
        );
    }
}

#[test]
fn cylinders_fire_at_different_amplitudes() {
    let mut config = SynthConfig::cross_plane_v8(FS);
    // Turn off stochastic variation so differences are purely deterministic.
    config.combustion_variation_max = 0.0;
    config.combustion_variation_min = 0.0;
    let mut synth = EngineSynth::new(config);

    let mut snapshot = idle_snapshot();
    // Set distinct blowdown pressures for cylinder 0 and cylinder 2 (both on bank 0).
    snapshot.blowdown_delta[0] = 1.0e5;
    snapshot.blowdown_delta[2] = 2.0e5;
    synth.set_snapshot(&snapshot);

    synth.cycle_hz.snap(800.0 / 120.0);
    for (i, smoother) in synth.blowdown_pa.iter_mut().enumerate() {
        smoother.snap(snapshot.blowdown_delta[i]);
    }
    // Start on the cycle the snapshot carries rather than fading into it,
    // so the measurement is not taken part-way through the fade.
    synth.cycle.blend.snap(1.0);
    synth.phase_fixed = 0;

    // A whole cycle, so both cylinders have had their turn. They play the
    // same normalised curve, so the ratio of the two peaks is the ratio of
    // the two pressure differences and nothing else.
    let cycle_samples = (FS / (800.0 / 120.0)) as usize;
    let mut buffer = [0.0f32; 2];
    let mut peaks = [0.0f32; 3];
    for _ in 0..cycle_samples {
        synth.render(&mut buffer, 2);
        for cylinder in [0usize, 2] {
            peaks[cylinder] = peaks[cylinder].max(synth.excitations[cylinder]);
        }
    }

    assert!(peaks[0] > 0.0, "cylinder 0 was never excited");
    let ratio = peaks[2] / peaks[0];
    assert!(
        (ratio - 2.0).abs() < 1e-3,
        "amplitude ratio was {ratio}, expected 2.0 (proportional to blowdown delta)"
    );
}

#[test]
fn knock_is_silent_when_integral_below_one() {
    let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
    let mut snapshot = loaded_snapshot();
    snapshot.knock_intensity = 0.0;
    synth.set_snapshot(&snapshot);

    // Render audio through multiple cycles.
    let mut buffer = [0.0f32; 2];
    for _ in 0..1000 {
        synth.render(&mut buffer, 2);
    }

    // When knock intensity is 0.0 (knock integral below 1.0), the voice is completely silent.
    assert_eq!(synth.knock.envelope, 0.0);
    let mut noise = Noise::new(42);
    assert_eq!(synth.knock.process(&mut noise), 0.0);
}

#[test]
fn knock_pitch_scales_inversely_with_bore() {
    let mut voice_small = KnockVoice::new(FS);
    let mut voice_large = KnockVoice::new(FS);

    let bore_small = 0.078;
    let bore_large = 0.078 * 2.0;
    let gamma = 1.33;
    let gas_constant = 287.0;
    let temperature = 1150.0;

    voice_small.tune(bore_small, gamma, gas_constant, temperature);
    voice_large.tune(bore_large, gamma, gas_constant, temperature);

    let modes_small = voice_small.mode_frequencies();
    let modes_large = voice_large.mode_frequencies();

    for i in 0..3 {
        let ratio = modes_small[i] / modes_large[i];
        assert!(
            (ratio - 2.0).abs() < 1e-4,
            "mode {i} ratio {ratio} != 2.0: doubling bore must halve mode frequency"
        );
    }

    // Also verify through EngineSynth update_control.
    let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
    let mut snapshot1 = loaded_snapshot();
    snapshot1.bore = 0.078;
    synth
        .exhaust_temperature
        .snap(snapshot1.exhaust_temperature);
    synth.set_snapshot(&snapshot1);
    synth.update_control();
    let f_small = synth.knock().mode_frequencies()[0];

    let mut snapshot2 = loaded_snapshot();
    snapshot2.bore = 0.078 * 2.0;
    synth
        .exhaust_temperature
        .snap(snapshot2.exhaust_temperature);
    synth.set_snapshot(&snapshot2);
    synth.update_control();
    let f_large = synth.knock().mode_frequencies()[0];

    let ratio = f_small / f_large;
    assert!(
        (ratio - 2.0).abs() < 1e-4,
        "EngineSynth knock mode ratio {ratio} != 2.0"
    );
}

#[test]
fn firing_intervals_ripple_at_idle_and_converge_at_limiter() {
    // Measures interval in samples between successive cylinder firings across the engine.
    let measure_intervals = |rpm: f32, torque: f32, cycles: usize| -> (Vec<f32>, usize) {
        let mut config = SynthConfig::cross_plane_v8(FS);
        // Disable stochastic CCV so interval variations are purely from crank dynamics.
        config.combustion_variation_max = 0.0;
        config.combustion_variation_min = 0.0;
        let mut synth = EngineSynth::new(config);

        let mut snapshot = idle_snapshot();
        snapshot.rpm = rpm;
        snapshot.indicated_torque = torque;
        // Distinct per-cylinder blowdowns (physical cylinder differences, Stage 1b).
        for i in 0..8 {
            snapshot.blowdown_delta[i] = 1.0e5 + (i as f32 * 0.2e5);
        }
        synth.set_snapshot(&snapshot);
        synth.cycle_hz.snap(rpm / 120.0);
        for (i, smoother) in synth.blowdown_pa.iter_mut().enumerate() {
            smoother.snap(snapshot.blowdown_delta[i]);
        }
        // Start on the cycle the snapshot carries rather than fading into
        // it, so the measurement is not taken part-way through the fade.
        synth.cycle.blend.snap(1.0);
        synth.phase_fixed = (0.001 * PHASE_ONE) as u32;

        let expected_fires = 8 * cycles;
        // Half of each cylinder's own peak: the instant its excitation
        // crosses that on the way up is that cylinder's firing, and the
        // curve crosses it once a cycle. As a fraction of the scale the
        // port is launching at, because that is the other half of what
        // decides how tall the pulse is.
        let thresholds: Vec<f32> = (0..8)
            .map(|i| 0.5 * snapshot.blowdown_delta[i] / REFERENCE_BLOWDOWN)
            .collect();
        let mut above = [false; 8];
        let mut firing_times = Vec::new();
        let mut buffer = [0.0f32; 2];
        let mut sample_idx = 0;

        // One extra, because the render starts a hair past cylinder 0's own
        // event and catches its edge part-way up. That first detection is
        // not a firing interval, it is where the measurement began.
        while firing_times.len() <= expected_fires && sample_idx < 100_000 {
            synth.render(&mut buffer, 2);
            for i in 0..8 {
                let now = synth.excitations[i] > thresholds[i] * synth.launch.value();
                if now && !above[i] {
                    firing_times.push(sample_idx);
                }
                above[i] = now;
            }
            sample_idx += 1;
        }

        let intervals = firing_times[1..]
            .windows(2)
            .map(|w| (w[1] - w[0]) as f32)
            .collect();
        (intervals, firing_times.len() - 1)
    };

    let cycles = 5;
    let (idle_intervals, idle_fires) = measure_intervals(800.0, 50.0, cycles);
    let (limiter_intervals, limiter_fires) = measure_intervals(7000.0, 50.0, cycles);

    // Every cylinder still fires exactly once per cycle (8 cylinders * 5 cycles = 40).
    let expected_fires = 8 * cycles;
    assert_eq!(
        idle_fires, expected_fires,
        "idle: {idle_fires} firings over {cycles} cycles, expected {expected_fires}"
    );
    assert_eq!(
        limiter_fires, expected_fires,
        "limiter: {limiter_fires} firings over {cycles} cycles, expected {expected_fires}"
    );

    // At idle, intra-cycle torque acceleration and compression deceleration
    // cause the sample interval between consecutive cylinder firings to ripple.
    let idle_cv = coefficient_of_variation(&idle_intervals);
    assert!(
        idle_cv > 0.01,
        "idle firing intervals must ripple: COV {idle_cv:.4}"
    );

    // At limiter, large rotating inertia (I * omega) dominates gas torque,
    // so firing intervals converge toward uniform.
    let limiter_cv = coefficient_of_variation(&limiter_intervals);
    assert!(
        limiter_cv < idle_cv * 0.25,
        "firing intervals must converge toward uniform at limiter: {limiter_cv:.4} vs {idle_cv:.4}"
    );
}

#[test]
fn combustion_variation_is_inert_above_the_threshold() {
    // Above CCV_THRESHOLD_RPM no draw is taken, so the whole synth — every
    // layer downstream of the shared noise generator included — must be
    // bit-identical whether or not the model is configured on.
    let render_with = |max: f64| {
        let mut config = SynthConfig::cross_plane_v8(FS);
        config.combustion_variation_max = max;
        config.combustion_variation_min = max * 0.4;
        let mut synth = EngineSynth::new(config);
        let mut snapshot = loaded_snapshot();
        snapshot.rpm = 3_000.0;
        synth.set_snapshot(&snapshot);
        // Start *at* speed rather than gliding up to it. The smoothed speed
        // otherwise sweeps through the low-rpm band on its way, where the
        // model is legitimately live and does consume draws — a spin-up is
        // not the steady state this asserts about.
        synth.cycle_hz.snap(3_000.0 / 120.0);
        render(&mut synth, 48_000)
    };
    assert_eq!(
        render_with(0.0),
        render_with(0.08),
        "CCV leaked above 1500 rpm"
    );
}

#[test]
fn combustion_variation_lopes_the_idle() {
    // At idle the same configuration must produce a *different* engine with
    // the model on, and specifically one whose firing-to-firing peak level
    // wanders. That wander is the lope.
    let peaks_with = |max: f64| {
        let mut config = SynthConfig::cross_plane_v8(FS);
        config.combustion_variation_max = max;
        config.combustion_variation_min = max * 0.4;
        // Only the exhaust: the noise layers have a spread of their own and
        // would mask the one being measured.
        config.intake_level = 0.0;
        config.mechanical_level = 0.0;
        let mut synth = EngineSynth::new(config);
        synth.set_snapshot(&idle_snapshot());
        render(&mut synth, 48_000); // settle
        let out = render(&mut synth, 4 * 48_000);
        // One firing interval on a V8 at 800 rpm: 720/8 degrees = 11.25 ms.
        window_peaks(&out, (FS * 0.01125) as usize)
    };

    let steady = coefficient_of_variation(&peaks_with(0.0));
    let loping = coefficient_of_variation(&peaks_with(0.08));
    // A fifth again as much spread. The measurement understates the effect:
    // the windows are a fixed grid while the firings now move about inside
    // it, so a jittered pulse that straddles a boundary is counted at its
    // full height in both windows.
    assert!(
        loping > 1.2 * steady,
        "no lope: COV {loping:.4} with variation vs {steady:.4} without"
    );
}

// -- block resonance -----------------------------------------------------

/// Energy in the bottom octave, `low..high` Hz, of the left channel.
///
/// Crude quadrature correlation on a handful of probe frequencies — enough
/// to compare two runs of the same signal, which is all these tests do.
fn band_energy(out: &[f32], low: f32, high: f32) -> f32 {
    let mut total = 0.0f64;
    let steps = 12;
    for i in 0..steps {
        let hz = low * (high / low).powf(i as f32 / (steps - 1) as f32);
        let (mut re, mut im) = (0.0f64, 0.0f64);
        for (n, frame) in out.chunks(2).enumerate() {
            let phase = TAU as f64 * hz as f64 * n as f64 / FS as f64;
            re += frame[0] as f64 * phase.sin();
            im += frame[0] as f64 * phase.cos();
        }
        let frames = (out.len() / 2) as f64;
        total += (re * re + im * im) / (frames * frames);
    }
    (total / steps as f64).sqrt() as f32
}

#[test]
fn block_rumble_is_strongest_at_idle_and_gone_at_speed() {
    // The behaviour is the same as it always was; what has gone is the
    // schedule that used to declare it. A modal bank driven by an impulse
    // train answers hardest when the train's fundamental is sitting in the
    // mode — 57 Hz at a V8's idle, right on the 80 Hz bending mode — and
    // barely at all when the same train has climbed to 400 Hz and the
    // structure is being driven well above resonance, where a mass is
    // stiff. Nothing tapers it: the filter does it, because that is what
    // the filter is a model of.
    // What the block *adds* to the bottom octave, against the level of the
    // whole engine: the difference between the mix with it and the mix
    // without, which is the only thing a listener could call rumble.
    let share_at = |snapshot: &EngineSnapshot| {
        let band = |level: f64| {
            let mut config = SynthConfig::cross_plane_v8(FS);
            config.structure_level = level;
            // Isolate the combustion-driven modal response under test from
            // the reciprocating shake, which is a second, independent
            // source into the same bank with its own schedule against rpm.
            config.reciprocating = config.reciprocating.with_reciprocating_mass(0.0);
            let mut synth = EngineSynth::new(config);
            synth.set_snapshot(snapshot);
            render(&mut synth, 2 * 48_000);
            let out = render(&mut synth, 2 * 48_000);
            (band_energy(&out, 60.0, 120.0), rms(&out))
        };
        let (silent, _) = band(0.0);
        let (radiating, level) = band(SynthConfig::default().structure_level);
        (radiating - silent) / level
    };

    let idle = share_at(&idle_snapshot());
    let mut fast = loaded_snapshot();
    fast.rpm = 6_000.0;
    let quick = share_at(&fast);
    // The margin used to be a factor of three, then under two, and now
    // measures 1.50. The block has not changed once: the mix it is a share
    // *of* has, twice. First an exhaust whose wall loss was a fiftieth of
    // `alpha` rang its way to a level it had no business at, hardest at
    // speed where the orders sit in the pipe's own modes; taking that
    // inflation out lifted the block's share. Then the exhaust got its
    // midrange back — a wall loss calibrated against a measured pipe, and
    // a valve loaded by the cylinder rather than by a hole — which is
    // level the block is once again a share of, and it arrives at speed
    // more than at idle for the same reason it left at speed.
    //
    // Recorded rather than smoothed over: the number is evidence about the
    // exhaust and not about the block, and a margin quietly moved is a
    // margin nobody can read the history of.
    assert!(
        idle > 1.4 * quick,
        "the block is no quieter at speed: {idle:.5} of the mix at idle \
         against {quick:.5} at 6000 rpm"
    );
}

#[test]
fn a_heavier_block_rumbles_lower() {
    let first_mode_for = |mass: f64| {
        let mut config = SynthConfig::cross_plane_v8(FS);
        config.structure.dressed_mass = mass;
        let mut synth = EngineSynth::new(config);
        synth.set_snapshot(&idle_snapshot());
        render(&mut synth, 4_800);
        synth.structure.mode_frequencies()[0]
    };
    let alloy_four = first_mode_for(95.0);
    let iron_v8 = first_mode_for(240.0);
    assert!(
        alloy_four > iron_v8 + 15.0,
        "mass barely moved the mode: {alloy_four} vs {iron_v8} Hz"
    );
    // Both inside the band the model advertises.
    for f in [alloy_four, iron_v8] {
        assert!((60.0..=120.0).contains(&f), "outside 60-120 Hz: {f}");
    }

    // And it is audible, not merely tabulated: the heavier block puts its
    // weight lower down.
    let rumble_for = |mass: f64| {
        let mut config = SynthConfig::cross_plane_v8(FS);
        config.structure.dressed_mass = mass;
        let mut synth = EngineSynth::new(config);
        synth.set_snapshot(&idle_snapshot());
        render(&mut synth, 2 * 48_000);
        let out = render(&mut synth, 2 * 48_000);
        (
            band_energy(&out, 60.0, 75.0),
            band_energy(&out, 100.0, 120.0),
        )
    };
    let (heavy_low, heavy_high) = rumble_for(240.0);
    let (light_low, light_high) = rumble_for(95.0);
    assert!(
        heavy_low / heavy_high > light_low / light_high,
        "the heavy block did not sit lower: {:.3} against {:.3}",
        heavy_low / heavy_high,
        light_low / light_high
    );
}

#[test]
fn block_rumble_puts_weight_in_the_bottom_octave() {
    // The audible claim, unchanged from when a peaking filter made it: at
    // idle the engine has more low-frequency energy with the block in the
    // mix than without, and no more peak level than the clipper allows.
    let low_energy = |level: f64| {
        let mut config = SynthConfig::cross_plane_v8(FS);
        config.structure_level = level;
        let mut synth = EngineSynth::new(config);
        synth.set_snapshot(&idle_snapshot());
        render(&mut synth, 48_000);
        let out = render(&mut synth, 2 * 48_000);
        assert!(peak(&out) < 1.0, "the block overloaded the clipper");
        band_energy(&out, 60.0, 120.0)
    };

    let silent = low_energy(0.0);
    let radiating = low_energy(SynthConfig::default().structure_level);
    assert!(
        radiating > 1.5 * silent,
        "the block added no weight: {radiating:.5} vs {silent:.5}"
    );
}

// -- combustion noise ----------------------------------------------------

/// A cycle whose combustion rise takes `rise` of the cycle, reaching the
/// same `peak` however long it takes, and blowing down identically after.
///
/// The whole point is that everything except the *steepness* is held: same
/// peak pressure, same blowdown, same valve events. What is left to hear is
/// combustion noise.
fn cycle_with_rise(peak: f32, rise: f32) -> [f32; CYCLE_TABLE] {
    /// Cycle phase the rise finishes at, leaving a short plateau.
    const RISE_END: f32 = 0.97;
    let mut pressure = [TEST_MANIFOLD_PA; CYCLE_TABLE];
    for (k, p) in pressure.iter_mut().enumerate() {
        let phi = (k as f32 + 0.5) / CYCLE_TABLE as f32;
        if phi < 0.25 {
            // Blowdown from the peak the cycle reached, through the open
            // valve, exactly as the previous stroke left it.
            *p = TEST_MANIFOLD_PA + peak * (-phi / 0.04).exp();
        } else if phi > RISE_END {
            // Held at the peak for the last few degrees before the valve
            // opens, so both traces reach it on a sampled point rather than
            // between two of them.
            *p = TEST_MANIFOLD_PA + peak;
        } else if phi > RISE_END - rise {
            // A raised-cosine rise: smooth at both ends, so the only thing
            // that changes between two of these is how long it takes.
            let u = (phi - (RISE_END - rise)) / rise;
            *p = TEST_MANIFOLD_PA + peak * 0.5 * (1.0 - (TAU * 0.5 * u).cos());
        }
    }
    pressure
}

#[test]
fn a_steeper_pressure_rise_radiates_more() {
    // The diesel-clatter mechanism, and the reason the structural path is
    // driven by `dP/dtheta` rather than by peak pressure: a charge that
    // arrives all at once puts its energy where a stiff lump of iron will
    // answer, and one that arrives gently does not. Both cycles here reach
    // exactly 60 bar.
    let level_for = |rise: f32| {
        let mut config = SynthConfig::cross_plane_v8(FS);
        // Everything but the block silenced, including the two other
        // sources that drive it and the reciprocating shake, which is a
        // fourth source into the same drive that does not care about
        // cylinder pressure at all.
        config.exhaust_level = 0.0;
        config.intake_level = 0.0;
        config.mechanical_level = 0.0;
        config.reciprocating = config.reciprocating.with_reciprocating_mass(0.0);
        let mut synth = EngineSynth::new(config);
        synth.exhaust_level.snap(0.0);
        let mut snapshot = loaded_snapshot();
        snapshot.knock_intensity = 0.0;
        snapshot.cylinder_pressure = cycle_with_rise(60.0e5, rise);
        synth.set_snapshot(&snapshot);
        render(&mut synth, 48_000);
        rms(&render(&mut synth, 96_000))
    };

    // 60 crank degrees of rise against 20 — a relaxed petrol burn against a
    // direct-injection diesel's.
    let petrol = level_for(60.0 / 720.0);
    let diesel = level_for(20.0 / 720.0);

    // The peaks really are equal, so nothing here is a level difference in
    // disguise.
    let tall = cycle_with_rise(60.0e5, 60.0 / 720.0);
    let quick = cycle_with_rise(60.0e5, 20.0 / 720.0);
    let top = |t: [f32; CYCLE_TABLE]| t.iter().cloned().fold(0.0f32, f32::max);
    assert!(
        (top(tall) - top(quick)).abs() < 1.0,
        "the two cycles do not peak alike: {} vs {}",
        top(tall),
        top(quick)
    );

    assert!(
        diesel > 1.5 * petrol,
        "steepness did not reach the block: {diesel:.5} on a 20 degree rise \
         against {petrol:.5} on a 60 degree one"
    );
}

#[test]
fn the_rig_and_knock_radiate_only_through_the_block() {
    // Neither the valvetrain nor the end gas has any business in the
    // exhaust: one is outside the cylinder entirely and the other happens
    // with the valve shut. Both reach the listener by shaking the block or
    // not at all, and switching the block off must take them with it.
    let level = |structure: f64, knock: f32| {
        let mut config = SynthConfig::cross_plane_v8(FS);
        config.exhaust_level = 0.0;
        config.intake_level = 0.0;
        config.structure_level = structure;
        // A fourth source into the same drive, silenced along with
        // combustion so the two under test are truly on their own.
        config.reciprocating = config.reciprocating.with_reciprocating_mass(0.0);
        let mut synth = EngineSynth::new(config);
        synth.exhaust_level.snap(0.0);
        let mut snapshot = loaded_snapshot();
        snapshot.knock_intensity = knock;
        // A cylinder that never changes pressure, so the third source into
        // the block — combustion — contributes nothing and the two under
        // test are on their own.
        snapshot.cylinder_pressure = [TEST_MANIFOLD_PA; CYCLE_TABLE];
        synth.set_snapshot(&snapshot);
        render(&mut synth, 48_000);
        rms(&render(&mut synth, 96_000))
    };

    let default_level = SynthConfig::default().structure_level;
    assert_eq!(
        level(0.0, 6.0),
        0.0,
        "the rig and a knocking cylinder found a way out with the block mute"
    );

    let rig_only = level(default_level, 0.0);
    let knocking = level(default_level, 6.0);
    assert!(rig_only > 1e-5, "the rig never reached the block");
    assert!(
        knocking > 1.2 * rig_only,
        "knock never reached the block: {knocking:.6} against {rig_only:.6}"
    );
}

#[test]
fn structural_output_is_bounded_and_free_of_dc() {
    // A resonator chain driven by a signal with a standing offset is the
    // classic way to lose headroom to something nobody can hear.
    for rpm in [0.0, 800.0, 3_000.0, 7_000.0] {
        let mut config = SynthConfig::cross_plane_v8(FS);
        config.exhaust_level = 0.0;
        config.intake_level = 0.0;
        let mut synth = EngineSynth::new(config);
        synth.exhaust_level.snap(0.0);
        let mut snapshot = loaded_snapshot();
        snapshot.rpm = rpm;
        snapshot.knock_intensity = 4.0;
        synth.set_snapshot(&snapshot);
        render(&mut synth, 48_000);
        let out = render(&mut synth, 4 * 48_000);

        assert!(out.iter().all(|s| s.is_finite()), "{rpm} rpm: not finite");
        assert!(peak(&out) < 1.0, "{rpm} rpm: peak {}", peak(&out));
        let mean = out.iter().map(|&s| s as f64).sum::<f64>() / out.len() as f64;
        assert!(
            mean.abs() < 1e-4,
            "{rpm} rpm: {mean} of standing offset on the structural path"
        );
    }
}

#[test]
fn reciprocating_shake_is_a_no_op_at_zero_mass() {
    // Zero mass must make the whole shaking-force path disappear, not just
    // happen to come out quiet for one geometry — so it is proven here by
    // showing geometry stops mattering at all once mass is zero, which
    // only holds if every per-cylinder force is exactly zero rather than
    // merely small.
    let render_with = |geometry: CylinderGeometry| {
        let mut config = SynthConfig::cross_plane_v8(FS);
        config.reciprocating = geometry;
        let mut synth = EngineSynth::new(config);
        synth.set_snapshot(&loaded_snapshot());
        render(&mut synth, 48_000);
        render(&mut synth, 48_000)
    };

    let zero_default = CylinderGeometry::default().with_reciprocating_mass(0.0);
    let zero_other =
        CylinderGeometry::new(0.060, 0.050, 0.090, 9.0).with_reciprocating_mass(0.0);
    assert_eq!(
        render_with(zero_default),
        render_with(zero_other),
        "a zeroed reciprocating mass should make geometry irrelevant to the mix"
    );

    let nonzero = CylinderGeometry::default();
    assert_ne!(
        render_with(zero_default),
        render_with(nonzero),
        "a non-zero reciprocating mass should audibly change the mix"
    );
}

// -- the valve boundary --------------------------------------------------

/// Frequencies the valve boundary is probed at across the audio band.
const VALVE_PROBE_HZ: [f32; 9] = [
    30.0, 60.0, 125.0, 250.0, 500.0, 1_000.0, 2_000.0, 4_000.0, 8_000.0,
];

/// Walks cylinder zero's valve through one cycle at 3000 rpm and reports,
/// for each of [`VALVE_PROBE_HZ`], the weakest reflection it ever presents,
/// alongside the fraction of the cycle the valve spent on its seat.
fn valve_reflection_floor(synth: &mut EngineSynth) -> ([f32; 9], f32) {
    let cycle_samples = (FS / (3_000.0 / 120.0)) as usize;
    let mut floor = [1.0f32; 9];
    let mut seated = 0usize;
    let mut buffer = [0.0f32; 2];
    for _ in 0..cycle_samples {
        synth.render(&mut buffer, 2);
        if !synth.network.valve_helmholtz_hz(0).is_finite() {
            seated += 1;
        }
        for (slot, &hz) in floor.iter_mut().zip(VALVE_PROBE_HZ.iter()) {
            *slot = slot.min(synth.network.valve_reflection_at(0, hz));
        }
    }
    (floor, seated as f32 / cycle_samples as f32)
}

#[test]
fn the_valve_opens_the_head_of_the_runner_once_a_cycle() {
    // The head of a primary is rigid while the valve is on its seat, and
    // the valve is on its seat for most of the cycle.
    let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
    synth.set_snapshot(&loaded_snapshot());
    render(&mut synth, 48_000);
    let (_, seated) = valve_reflection_floor(&mut synth);

    // A four-stroke exhaust valve is off its seat for something like a
    // third of the cycle and shut for the rest.
    let open_fraction = 1.0 - seated;
    assert!(
        (0.15..0.55).contains(&open_fraction),
        "the valve was open for {open_fraction:.2} of the cycle"
    );
}

#[test]
fn the_open_valve_keeps_the_low_end_and_takes_the_midrange() {
    // The reason the cylinder is modelled and not just its port area. A
    // box the wave cannot compress is a wall, so a long wave arriving at a
    // fully lifted valve turns round almost intact; the same valve swallows
    // the band its own gap and volume resonate at, somewhere in the middle.
    //
    // An area step, r = (A_p - A_v) / (A_p + A_v), gets this backwards in
    // the place it costs most. It is flat, so whatever it takes out of the
    // midrange it takes out of 30 Hz too — and at full lift on this header
    // that is a quarter of every bounce, a third of every cycle, which is
    // an exhaust with no rumble left in it.
    let config = SynthConfig::cross_plane_v8(FS);
    let pipe_area = config.exhaust.primaries[0].area as f32;
    let mut synth = EngineSynth::new(config);
    synth.set_snapshot(&loaded_snapshot());
    render(&mut synth, 48_000);
    let (floor, _) = valve_reflection_floor(&mut synth);

    // The bottom of the band comes back off this boundary all cycle.
    assert!(
        floor[0] > 0.85,
        "the valve bled 30 Hz away: |r| fell to {:.3}",
        floor[0]
    );
    // The absorption is above it, and it is deep.
    let deepest = floor[1..8].iter().cloned().fold(1.0f32, f32::min);
    assert!(
        deepest < 0.7,
        "the cylinder absorbed nothing in the midrange: |r| only fell to {deepest:.3}"
    );
    // And it is the midrange that is absorbed, not the band underneath it —
    // which is the difference between a pipe with an engine on the end of
    // it and a pipe with a hole in the end of it.
    assert!(
        floor[0] > deepest + 0.15,
        "the valve took as much from 30 Hz ({:.3}) as from its own band ({deepest:.3})",
        floor[0]
    );

    // The flat area step this replaced would have done the same at 30 Hz as
    // it did at 500, which is the whole complaint against it.
    let peak_area = loaded_snapshot()
        .exhaust_valve_area
        .iter()
        .cloned()
        .fold(0.0f32, f32::max);
    let area_step = (pipe_area - peak_area) / (pipe_area + peak_area);
    assert!(
        floor[0] > area_step + 0.2,
        "the cylinder load reflects {:.3} at 30 Hz, the area step {area_step:.3}",
        floor[0]
    );
}

// -- mechanical noise floor ----------------------------------------------

#[test]
fn mechanical_layers_sit_at_unity() {
    // MECHANICAL_RUMBLE_MAKEUP exists to make `mechanical_level` mean the
    // same thing as the other level controls: with FMEP at the reference,
    // the rumble path leaves the voice at unity RMS.
    let mut voice = MechanicalVoice::new(FS);
    let mut noise = Noise::new(19);
    let snapshot = EngineSnapshot {
        friction_mep: REFERENCE_FMEP,
        ..EngineSnapshot::default()
    };
    voice.tune(&snapshot, 0.0, 8);
    voice.rumble_gain.snap(1.0);
    voice.click_gain.snap(0.0);
    voice.event_hz.snap(0.0);

    let mut sum_sq = 0.0f64;
    let n = 192_000;
    for _ in 0..n {
        let y = voice.process(&mut noise);
        sum_sq += (y as f64) * (y as f64);
    }
    let rms = (sum_sq / n as f64).sqrt() as f32;
    assert!(
        (rms - 1.0).abs() < 0.05,
        "rumble makeup is out of calibration: {rms} (adjust MECHANICAL_RUMBLE_MAKEUP)"
    );

    // The full rig at reference operating conditions also sits at unity RMS,
    // so `SynthConfig::mechanical_level` keeps its meaning when impulsive
    // sources and rumble sound together.
    let mut full_voice = MechanicalVoice::new(FS);
    let ref_snapshot = EngineSnapshot {
        rpm: 800.0,
        friction_mep: REFERENCE_FMEP,
        peak_cylinder_pressure: REFERENCE_PEAK_PRESSURE,
        spark_cut: false,
        ..EngineSnapshot::default()
    };
    let cycle_hz = 800.0 / 120.0;
    full_voice.tune(&ref_snapshot, cycle_hz, 8);
    // Settle gain smoothers to their steady-state values.
    for _ in 0..48_000 {
        full_voice.process(&mut noise);
    }

    let mut sum_sq_full = 0.0f64;
    let n_full = 192_000;
    for _ in 0..n_full {
        let y = full_voice.process(&mut noise);
        sum_sq_full += (y as f64) * (y as f64);
    }
    let full_rms = (sum_sq_full / n_full as f64).sqrt() as f32;
    assert!(
        (full_rms - 1.0).abs() < 0.05,
        "full mechanical rig RMS is out of calibration: {full_rms}"
    );
}

#[test]
fn mechanical_event_rates_track_their_orders() {
    let cylinders = 8;
    let voice = MechanicalVoice::new(FS);

    for rpm in [800.0, 2400.0, 4800.0] {
        let crank_hz = (rpm / 60.0) as f32;
        let cycle_hz = (rpm / 120.0) as f32;

        let check_source = |src: &Option<ImpulsiveSource>, expected_order: f32, name: &str| {
            let s = src.as_ref().unwrap_or_else(|| panic!("missing {name}"));
            let eff_order = s.effective_order(cylinders);
            assert!(
                (eff_order - expected_order).abs() < 1e-4,
                "{name} effective order {eff_order} != expected {expected_order}"
            );
            let eff_hz = s.effective_hz(cycle_hz, cylinders);
            let expected_hz = expected_order * crank_hz;
            assert!(
                (eff_hz - expected_hz).abs() < 1e-3,
                "{name} at {rpm} rpm has rate {eff_hz} Hz != expected {expected_hz} Hz"
            );
        };

        check_source(&voice.intake_valve, 4.0, "intake_valve");
        check_source(&voice.exhaust_valve, 4.0, "exhaust_valve");
        check_source(&voice.piston_slap, 4.0, "piston_slap");
        check_source(&voice.injector, 4.0, "injector");
        check_source(&voice.timing_chain, 19.0, "timing_chain");
        check_source(&voice.gear_whine, 31.0, "gear_whine");
        check_source(&voice.accessory, 1.37, "accessory");
    }
}

#[test]
fn rotary_gear_whine_sits_at_a_third_of_the_shaft_order() {
    // Stage M5: a rotor's mechanical noise — bearings, seals, the gear
    // that meshes with it — vibrates at rotor rate, one third of the
    // eccentric shaft rate this rig's `crank_hz` is quoted in. Gear
    // whine is the rotor-driven source in `MechanicalSpec::rotary`; the
    // accessory drive is a belt on the shaft nose and stays at shaft
    // rate, which this test also pins down so a future edit cannot
    // "fix" it by accident.
    use crate::physics::rotor::RotorGeometry;

    let spec = MechanicalSpec::rotary();
    let gear_whine = spec.gear_whine.expect("rotary gear whine must exist");
    let shaft_order = match gear_whine.rate {
        SourceRate::Order(o) => o,
        SourceRate::PerCylinder => panic!("gear whine must be an explicit order"),
    };
    let rotor_order = shaft_order * RotorGeometry::SHAFT_TO_ROTOR_RATIO as f32;
    assert!(
        (rotor_order.round() - rotor_order).abs() < 1e-3,
        "gear whine order {shaft_order} is not a clean multiple of 1/3: rotor order {rotor_order}"
    );
    assert!(
        (shaft_order - RotorGeometry::shaft_order(rotor_order.round())).abs() < 1e-6,
        "gear whine order {shaft_order} does not round-trip through RotorGeometry::shaft_order"
    );

    let accessory = spec.accessory.expect("rotary accessory drive must exist");
    assert_eq!(
        accessory.rate,
        SourceRate::Order(1.37),
        "the accessory belt is on the shaft nose and must stay at shaft rate"
    );
}

#[test]
fn accessory_order_is_incommensurate_with_crank() {
    let voice = MechanicalVoice::new(FS);
    let accessory = voice.accessory.expect("accessory drive must exist");
    let order = accessory.effective_order(8);

    // An integer or half-integer order repeats after 1 or 2 crank revolutions.
    // A non-integer order like 1.37 has fractional remainder after 1 and 2 revs.
    let rev1_events = order;
    let rev2_events = order * 2.0;
    assert!(
        (rev1_events - rev1_events.round()).abs() > 0.05,
        "accessory order {order} is integer"
    );
    assert!(
        (rev2_events - rev2_events.round()).abs() > 0.05,
        "accessory order {order} is commensurate with 720 deg cycle"
    );

    // Within one 720 degree cycle, events occur at distinct crank phases.
    let dtheta = 360.0 / order;
    let mut angles = Vec::new();
    let mut theta = 0.0f32;
    while theta < 720.0 {
        angles.push(theta);
        theta += dtheta;
    }
    for i in 0..angles.len() {
        for j in (i + 1)..angles.len() {
            let diff = (angles[i] - angles[j]).abs();
            assert!(
                (diff % 360.0) > 1.0,
                "accessory events coincided at crank phase: {} and {}",
                angles[i],
                angles[j]
            );
        }
    }
}

#[test]
fn piston_slap_scales_with_pressure_and_vanishes_on_spark_cut() {
    let voice = MechanicalVoice::new(FS);
    let slap = voice.piston_slap.as_ref().unwrap();
    let intake = voice.intake_valve.as_ref().unwrap();
    let cycle_hz = 1000.0 / 120.0;

    let snap_normal = EngineSnapshot {
        rpm: 1000.0,
        friction_mep: REFERENCE_FMEP,
        peak_cylinder_pressure: REFERENCE_PEAK_PRESSURE,
        spark_cut: false,
        ..EngineSnapshot::default()
    };
    let slap_gain_normal = slap.level_law.compute(&snap_normal, cycle_hz) * slap.base_level;
    let intake_gain_normal =
        intake.level_law.compute(&snap_normal, cycle_hz) * intake.base_level;
    assert!(slap_gain_normal > 0.0, "slap must be live when firing");
    assert!(intake_gain_normal > 0.0, "intake must be live");

    // High pressure raises piston slap proportionally
    let snap_high = EngineSnapshot {
        rpm: 1000.0,
        friction_mep: REFERENCE_FMEP,
        peak_cylinder_pressure: REFERENCE_PEAK_PRESSURE * 2.0,
        spark_cut: false,
        ..EngineSnapshot::default()
    };
    let slap_gain_high = slap.level_law.compute(&snap_high, cycle_hz) * slap.base_level;
    assert!(
        (slap_gain_high - 2.0 * slap_gain_normal).abs() < 1e-3,
        "slap should scale with peak pressure: {slap_gain_high} vs 2 * {slap_gain_normal}"
    );

    // Spark cut completely silences piston slap, but valves continue seating
    let snap_cut = EngineSnapshot {
        rpm: 1000.0,
        friction_mep: REFERENCE_FMEP,
        peak_cylinder_pressure: REFERENCE_PEAK_PRESSURE,
        spark_cut: true,
        ..EngineSnapshot::default()
    };
    let slap_gain_cut = slap.level_law.compute(&snap_cut, cycle_hz) * slap.base_level;
    let intake_gain_cut = intake.level_law.compute(&snap_cut, cycle_hz) * intake.base_level;
    assert_eq!(slap_gain_cut, 0.0, "piston slap must vanish on spark cut");
    assert!(
        intake_gain_cut > 0.0,
        "valve seatings must continue on spark cut"
    );
}

#[test]
fn a_warm_engine_is_bit_for_bit_what_it_was_before_the_cold_path() {
    // The contract the whole stage rests on. Every recorded fingerprint was
    // measured on a soaked engine, so if the cold multiplier is anything but
    // exactly one at `cold_fraction == 0` the catalogue has silently moved.
    // Asserted on the float itself, not within a tolerance: `1.0 + r * 0.0`
    // is exactly one and `x * 1.0` is exactly `x`, and anything that stops
    // being true of should fail here rather than in a re-record.
    let mut warm = loaded_snapshot();
    warm.cold_fraction = 0.0;
    let cycle_hz = warm.rpm / 120.0;

    let mut rig = MechanicalVoice::from_spec(&MechanicalSpec::default(), FS);
    rig.tune(&warm, cycle_hz, 8);

    let sources: [(&str, &Option<ImpulsiveSource>); 7] = [
        ("intake_valve", &rig.intake_valve),
        ("exhaust_valve", &rig.exhaust_valve),
        ("piston_slap", &rig.piston_slap),
        ("injector", &rig.injector),
        ("timing_chain", &rig.timing_chain),
        ("gear_whine", &rig.gear_whine),
        ("accessory", &rig.accessory),
    ];
    for (name, source) in sources {
        let source = source.as_ref().unwrap_or_else(|| panic!("{name} missing"));
        let without_cold = source.base_level * source.level_law.compute(&warm, cycle_hz);
        assert_eq!(
            source.gain.target(),
            without_cold,
            "{name} is not exactly where it was warm"
        );
    }
}

#[test]
fn a_cold_engine_hits_its_own_clearances_harder() {
    // The other end of the same multiplier, at the stated factors. Slap
    // doubles, the valve gear goes up by 45 %, the injector by 25 %, and
    // nothing whose loudness is not a clearance moves at all.
    let base = loaded_snapshot();
    let cycle_hz = base.rpm / 120.0;

    let targets = |cold: f32| {
        let mut snapshot = base;
        snapshot.cold_fraction = cold;
        let mut rig = MechanicalVoice::from_spec(&MechanicalSpec::default(), FS);
        rig.tune(&snapshot, cycle_hz, 8);
        [
            rig.intake_valve.as_ref().unwrap().gain.target(),
            rig.exhaust_valve.as_ref().unwrap().gain.target(),
            rig.piston_slap.as_ref().unwrap().gain.target(),
            rig.injector.as_ref().unwrap().gain.target(),
            rig.timing_chain.as_ref().unwrap().gain.target(),
            rig.gear_whine.as_ref().unwrap().gain.target(),
            rig.accessory.as_ref().unwrap().gain.target(),
        ]
    };
    let warm = targets(0.0);
    let cold = targets(1.0);

    let expected = [
        ("intake_valve", 1.0 + COLD_TAPPET_RISE),
        ("exhaust_valve", 1.0 + COLD_TAPPET_RISE),
        ("piston_slap", 1.0 + COLD_PISTON_SLAP_RISE),
        ("injector", 1.0 + COLD_INJECTOR_RISE),
        ("timing_chain", 1.0),
        ("gear_whine", 1.0),
        ("accessory", 1.0),
    ];
    for (i, (name, factor)) in expected.into_iter().enumerate() {
        assert!(
            warm[i] > 0.0,
            "{name} was silent warm, so nothing is proved"
        );
        let ratio = cold[i] / warm[i];
        assert!(
            (ratio - factor).abs() < 1e-5,
            "{name} rose by {ratio:.3} cold, not {factor:.3}"
        );
    }
}

#[test]
fn the_starter_whine_sings_while_it_is_meshed_and_stops_when_it_is_not() {
    // The release, heard. There is no fade and no envelope: the voice is
    // handed a mesh frequency while the pinion is in and a zero when it is
    // out, and a zero is silence rather than a tone nobody can hear.
    let mut voice = StarterVoice::new(FS);
    let mut noise = Noise::new(0x57A2);

    voice.tune(430.0);
    let cranking: Vec<f32> = (0..4_800).map(|_| voice.process(&mut noise)).collect();
    let meshed_rms = rms(&cranking);
    assert!(
        meshed_rms > 0.05,
        "a meshed starter was silent: {meshed_rms:.5}"
    );
    assert!(voice.is_singing());

    // Pinion out. The gate is short but not instant, so the first block
    // still carries the tail of it; the second must be nothing at all.
    voice.tune(0.0);
    let _release: Vec<f32> = (0..24_000).map(|_| voice.process(&mut noise)).collect();
    let after: Vec<f32> = (0..24_000).map(|_| voice.process(&mut noise)).collect();
    assert_eq!(
        rms(&after),
        0.0,
        "the starter was still singing a tenth of a second after it let go"
    );
    assert!(!voice.is_singing());

    // And the pitch it sings at is the one it was given, not one of its own.
    // The whine follows the crank because the pinion is geared to it, so a
    // starter dragging a slow engine sings low and the same starter on a
    // faster one sings high, with nothing scheduling either.
    let sung_at = |mesh_hz: f32| {
        let mut voice = StarterVoice::new(FS);
        let mut noise = Noise::new(0x51A7);
        voice.tune(mesh_hz);
        (0..4_800).for_each(|_| {
            voice.process(&mut noise);
        });
        (0..48_000)
            .map(|_| voice.process(&mut noise))
            .collect::<Vec<f32>>()
    };
    // `magnitude_at` de-interleaves a stereo bus; this voice is one channel.
    let tone = |x: &[f32], hz: f32| {
        let w = TAU * hz / FS;
        let (mut re, mut im) = (0.0f32, 0.0f32);
        for (n, &v) in x.iter().enumerate() {
            let phase = w * n as f32;
            re += v * phase.cos();
            im += v * phase.sin();
        }
        2.0 * (re * re + im * im).sqrt() / x.len() as f32
    };
    let slow = sung_at(200.0);
    let fast = sung_at(400.0);
    assert!(
        tone(&slow, 200.0) > 5.0 * tone(&fast, 200.0),
        "a starter turning half as fast did not sing an octave down"
    );
    // 1200 Hz is the fast one's third mesh harmonic and nothing at all of
    // the slow one's, which only reaches 600.
    assert!(
        tone(&fast, 1_200.0) > 5.0 * tone(&slow, 1_200.0),
        "the whine's harmonics did not move with its fundamental"
    );
}

#[test]
fn a_running_engine_carries_no_starter_at_all() {
    // The neutrality half: every recorded fingerprint is an engine that is
    // already running, and a running engine has its pinion out. Asserted
    // sample for sample against a synth with the voice's level at zero —
    // if a snapshot carrying `starter_hz` of zero differed from one with no
    // starter in the mix at all, the whole catalogue would have moved.
    let render_with = |starter_hz: f32, level: f64| {
        let mut config = SynthConfig::cross_plane_v8(FS);
        config.starter_level = level;
        let mut synth = EngineSynth::new(config);
        let mut snapshot = loaded_snapshot();
        snapshot.starter_hz = starter_hz;
        synth.set_snapshot(&snapshot);
        render(&mut synth, 4_800);
        render(&mut synth, 24_000)
    };

    let running = render_with(0.0, SynthConfig::default().starter_level);
    let without = render_with(0.0, 0.0);
    assert_eq!(
        running, without,
        "a pinion out of mesh still changed what came out"
    );

    let meshed = render_with(430.0, SynthConfig::default().starter_level);
    assert!(
        meshed != without,
        "a meshed starter never reached the output, so nothing is proved"
    );
}

#[test]
fn cold_fraction_crosses_the_snapshot_boundary() {
    // The gate for every cold-start behaviour in the synth, and the one
    // number that says the recorded fingerprints are still being measured
    // on the engine they were recorded on: a soaked block reads exactly
    // zero here, so nothing keyed on it can move.
    use crate::audio::{EngineControls, SnapshotSource};
    use crate::environment::Environment;
    use crate::physics::engine_block::EngineBlock;

    let sample = |cold: bool| {
        let mut block = EngineBlock::cross_plane_v8(Environment::default());
        if cold {
            block.cold_start();
        }
        let mut source = SnapshotSource::new(&block);
        block.update(1.0 / 120.0, 800.0);
        source.sample(&block, 800.0, 1.0 / 120.0, EngineControls::default())
    };

    assert_eq!(
        sample(false).cold_fraction,
        0.0,
        "a soaked engine is not at zero cold fraction, so every fingerprint moved"
    );
    assert!(
        sample(true).cold_fraction > 0.99,
        "an engine that stood overnight reported {} cold",
        sample(true).cold_fraction
    );
    assert_eq!(EngineSnapshot::default().cold_fraction, 0.0);
}

#[test]
fn a_cold_engine_carries_a_louder_mechanical_floor() {
    // The other half of the warm-up: thick oil is more friction, more
    // friction is a louder valvetrain and bearing racket, and nothing in the
    // audio path had to be told about the temperature to make that happen.
    let cold = warmed_snapshot(2.0, 3_000.0);
    let hot = warmed_snapshot(180.0, 3_000.0);
    assert!(
        cold.friction_mep > 1.15 * hot.friction_mep,
        "a cold engine reported {:.0} Pa of FMEP against a warm {:.0}",
        cold.friction_mep,
        hot.friction_mep
    );

    let floor = |snapshot: &EngineSnapshot| {
        let mut config = SynthConfig::cross_plane_v8(FS);
        config.exhaust_level = 0.0;
        config.intake_level = 0.0;
        // Cold and hot run the same rpm, so the reciprocating shake is
        // identical between them; left in, it would dilute the friction
        // difference under test rather than change the comparison's
        // direction, but this isolates the mechanism the test is named for.
        config.reciprocating = config.reciprocating.with_reciprocating_mass(0.0);
        let mut synth = EngineSynth::new(config);
        synth.exhaust_level.snap(0.0);
        synth.set_snapshot(snapshot);
        render(&mut synth, 48_000);
        rms(&render(&mut synth, 48_000))
    };
    let cold_floor = floor(&cold);
    let hot_floor = floor(&hot);
    assert!(
        cold_floor > 1.10 * hot_floor,
        "the mechanical floor did not follow the oil: {cold_floor:.6} cold, {hot_floor:.6} warm"
    );
}

#[test]
fn mechanical_floor_scales_with_friction() {
    let level_at = |fmep: f32| {
        let mut config = SynthConfig::cross_plane_v8(FS);
        config.exhaust_level = 0.0;
        config.intake_level = 0.0;
        let mut synth = EngineSynth::new(config);
        synth.exhaust_level.snap(0.0);
        let mut snapshot = idle_snapshot();
        snapshot.friction_mep = fmep;
        synth.set_snapshot(&snapshot);
        render(&mut synth, 48_000);
        rms(&render(&mut synth, 48_000))
    };

    let light = level_at(0.7e5);
    let heavy = level_at(2.1e5);
    assert!(light > 1e-5, "no mechanical noise at idle friction");
    assert!(
        heavy > 1.5 * light,
        "friction did not drive the floor: {heavy:.6} vs {light:.6}"
    );

    // A stopped engine has no mechanical noise, whatever the correlation's
    // constant term says.
    let mut config = SynthConfig::cross_plane_v8(FS);
    config.exhaust_level = 0.0;
    let mut synth = EngineSynth::new(config);
    synth.exhaust_level.snap(0.0);
    synth.set_snapshot(&EngineSnapshot {
        rpm: 0.0,
        friction_mep: 0.35e5,
        ..EngineSnapshot::default()
    });
    render(&mut synth, 48_000);
    assert!(
        rms(&render(&mut synth, 24_000)) < 1e-5,
        "a stopped engine idled"
    );
}

#[test]
fn mechanical_floor_fills_the_gaps_between_firings() {
    // The failure this guards against: an idle built from combustion alone
    // has audible silence between its pulses, which is the loudest tell
    // that a sound is synthesised.
    // Measured on a four rather than the V8: at 800 rpm a four fires every
    // 37 ms and its pulses are gone in 4, so the gaps are real. A V8 at the
    // same speed fires three times as often and partly fills its own.
    let gap_floor = |mechanical: f64| {
        let mut config = SynthConfig::uniform(FS, 4, 1);
        config.mechanical_level = mechanical;
        config.intake_level = 0.0;
        let mut synth = EngineSynth::new(config);
        synth.set_snapshot(&idle_snapshot());
        render(&mut synth, 48_000);
        let out = render(&mut synth, 2 * 48_000);
        let mut peaks = window_peaks(&out, (FS * 0.001) as usize);
        peaks.sort_by(|a, b| a.partial_cmp(b).unwrap());
        peaks[peaks.len() / 10]
    };

    let bare = gap_floor(0.0);
    let filled = gap_floor(SynthConfig::default().mechanical_level);
    // The margin was a factor of 1.5, then 1.25 once the pipe started
    // ringing down between firings on its own wall loss instead of on a
    // schedule, and is now 1.1: `mechanical` reaches the mix only through
    // `structure_level`, and T1 turned that constant down because it had
    // been calibrated against an exhaust several stages quieter than the
    // one it now shares a mix with. The floor still fills the gaps — it
    // does so by less.
    assert!(
        filled > 1.1 * bare,
        "the floor did not fill the gaps: {filled:.6} vs {bare:.6}"
    );
}

#[test]
fn valve_clicks_keep_time_with_the_camshaft() {
    // Two seatings per cylinder per cycle, and the cam does not care about
    // the firing order — so the rate is the cycle rate times 2 N_cyl.
    let mut voice = MechanicalVoice::new(FS);
    let mut noise = Noise::new(5);
    let cycle_hz = 800.0 / 120.0; // 800 rpm
    let snapshot = EngineSnapshot {
        friction_mep: 1.0e5,
        rpm: 800.0,
        ..EngineSnapshot::default()
    };
    voice.tune(&snapshot, cycle_hz, 8);
    voice
        .event_hz
        .snap(cycle_hz * 8.0 * VALVE_EVENTS_PER_CYLINDER);
    voice.rumble_gain.snap(0.0);
    voice.click_gain.snap(1.0);

    let mut events = 0usize;
    let seconds = 4;
    for _ in 0..(seconds * FS as usize) {
        let before = voice.click_phase;
        voice.process(&mut noise);
        if voice.click_phase < before {
            events += 1;
        }
    }
    let expected = (cycle_hz * 8.0 * VALVE_EVENTS_PER_CYLINDER) as usize * seconds;
    assert!(
        events.abs_diff(expected) <= 2,
        "valve events: {events}, expected about {expected}"
    );
}

#[test]
fn valve_float_is_bit_zero_below_threshold() {
    let sample_rate = FS;
    let float_rpm = 7_000.0;
    let spec_with_float = MechanicalSpec {
        float_rpm: Some(float_rpm),
        ..MechanicalSpec::default()
    };
    let spec_without_float = MechanicalSpec {
        float_rpm: None,
        ..MechanicalSpec::default()
    };

    let mut voice_float = MechanicalVoice::from_spec(&spec_with_float, sample_rate);
    let mut voice_none = MechanicalVoice::from_spec(&spec_without_float, sample_rate);

    // Test at several speeds below float_rpm, including right at float_rpm.
    for rpm in [800.0, 3_000.0, 6_000.0, 6_999.0, 7_000.0] {
        let snap = EngineSnapshot {
            rpm,
            friction_mep: REFERENCE_FMEP,
            peak_cylinder_pressure: REFERENCE_PEAK_PRESSURE,
            spark_cut: false,
            ..EngineSnapshot::default()
        };
        let cycle_hz = (rpm / 120.0) as f32;

        voice_float.tune(&snap, cycle_hz, 4);
        voice_none.tune(&snap, cycle_hz, 4);

        let mut noise_float = Noise::new(12345);
        let mut noise_none = Noise::new(12345);

        for _ in 0..1_000 {
            let s_float = voice_float.process(&mut noise_float);
            let s_none = voice_none.process(&mut noise_none);
            assert_eq!(
                s_float.to_bits(),
                s_none.to_bits(),
                "float voice drifted at {rpm} rpm: {s_float} vs {s_none}"
            );
        }
    }
}

#[test]
fn valve_impact_level_rises_monotonically_above_threshold() {
    let sample_rate = FS;
    let float_rpm = 6_500.0;
    let spec = MechanicalSpec {
        float_rpm: Some(float_rpm),
        ..MechanicalSpec::default()
    };
    let mut voice = MechanicalVoice::from_spec(&spec, sample_rate);

    let mut prev_gain = 0.0f32;
    let mut prev_freq = 0.0f32;

    for overspeed in [0.0, 100.0, 250.0, 500.0, 1_000.0, 2_000.0] {
        let rpm = float_rpm + overspeed;
        let snap = EngineSnapshot {
            rpm,
            friction_mep: REFERENCE_FMEP,
            peak_cylinder_pressure: REFERENCE_PEAK_PRESSURE,
            spark_cut: false,
            ..EngineSnapshot::default()
        };
        let cycle_hz = (rpm / 120.0) as f32;
        voice.tune(&snap, cycle_hz, 4);

        let intake = voice.intake_valve.as_ref().unwrap();
        let cur_gain = intake.gain.target();
        let cur_freq = intake.current_frequency;

        if overspeed > 0.0 {
            assert!(
                cur_gain > prev_gain,
                "target gain must rise monotonically above float_rpm: {cur_gain} <= {prev_gain}"
            );
            assert!(
                cur_freq > prev_freq,
                "resonant frequency must rise monotonically above float_rpm: {cur_freq} <= {prev_freq}"
            );
        }

        prev_gain = cur_gain;
        prev_freq = cur_freq;
    }
}

#[test]
fn catalogue_fingerprints_unchanged_below_float() {
    // Every preset in the catalogue specifies redline < float_rpm (by default float_rpm is 1.06 * redline).
    // Since calibration sweeps and steady operation used for catalogue measurements stay at or below redline,
    // the valve float impact schedule is completely inactive and does not perturb the voice.
    let sample_rate = FS;
    for preset in crate::bench::EnginePreset::catalogue() {
        assert!(
            preset.redline <= preset.float_rpm,
            "preset {} redline {:.0} exceeds float_rpm {:.0}",
            preset.name,
            preset.redline,
            preset.float_rpm
        );

        let spec_with_preset = MechanicalSpec {
            float_rpm: Some(preset.float_rpm as f32),
            ..MechanicalSpec::default()
        };
        let spec_without = MechanicalSpec {
            float_rpm: None,
            ..MechanicalSpec::default()
        };

        let mut v1 = MechanicalVoice::from_spec(&spec_with_preset, sample_rate);
        let mut v2 = MechanicalVoice::from_spec(&spec_without, sample_rate);

        let snap = EngineSnapshot {
            rpm: preset.redline as f32,
            friction_mep: REFERENCE_FMEP,
            peak_cylinder_pressure: REFERENCE_PEAK_PRESSURE,
            spark_cut: false,
            ..EngineSnapshot::default()
        };
        let cycle_hz = (preset.redline / 120.0) as f32;
        v1.tune(&snap, cycle_hz, preset.firing.len());
        v2.tune(&snap, cycle_hz, preset.firing.len());

        let mut noise1 = Noise::new(777);
        let mut noise2 = Noise::new(777);
        for _ in 0..500 {
            assert_eq!(
                v1.process(&mut noise1).to_bits(),
                v2.process(&mut noise2).to_bits(),
                "preset {} valve voice output differed at redline",
                preset.name
            );
        }
    }
}

#[test]
fn limiter_bounce_float_band_appears_and_disappears_with_each_cut() {
    // When an engine hits a limiter that allows overspeed excursions beyond float_rpm,
    // energy in the valve impact band (e.g. 3 kHz to 8 kHz) rises sharply during the excursion
    // and drops when cut intervention brings engine speed back below float_rpm.
    let sample_rate = FS;
    let float_rpm = 7_000.0;
    let spec = MechanicalSpec {
        intake_valve: Some(ImpulsiveSpec::per_cylinder(1.0)),
        exhaust_valve: Some(ImpulsiveSpec::per_cylinder(1.0)),
        piston_slap: None,
        injector: None,
        timing_chain: None,
        gear_whine: None,
        accessory: None,
        float_rpm: Some(float_rpm),
    };
    let mut voice = MechanicalVoice::from_spec(&spec, sample_rate);
    let mut noise = Noise::new(999);

    // A bandpass filter centered around 4.5 kHz isolates the valve float impact band.
    let filter_coeffs =
        crate::audio::filters::BiquadCoeffs::bandpass(sample_rate, 4_500.0, 1.5);
    let mut filter = crate::audio::filters::Biquad::new(filter_coeffs);

    // Cycle through 3 limiter bounce cycles: below float -> overspeed excursion -> recovered below float
    for cycle in 0..3 {
        // Below float threshold (approaching limiter or cut recovery)
        let snap_under = EngineSnapshot {
            rpm: 6_900.0,
            friction_mep: REFERENCE_FMEP,
            peak_cylinder_pressure: REFERENCE_PEAK_PRESSURE,
            spark_cut: false,
            ..EngineSnapshot::default()
        };
        voice.tune(&snap_under, (6_900.0 / 120.0) as f32, 4);
        // Let smoothers settle fully
        for _ in 0..10_000 {
            let s = voice.process(&mut noise);
            filter.process(s);
        }

        let mut rms_under = 0.0f64;
        for _ in 0..8_000 {
            let s = filter.process(voice.process(&mut noise)) as f64;
            rms_under += s * s;
        }
        rms_under = (rms_under / 8_000.0).sqrt();

        // Overspeed excursion beyond float threshold (e.g. inertia bounce)
        let snap_over = EngineSnapshot {
            rpm: 7_600.0, // 600 rpm over float
            friction_mep: REFERENCE_FMEP,
            peak_cylinder_pressure: REFERENCE_PEAK_PRESSURE,
            spark_cut: true,
            ..EngineSnapshot::default()
        };
        voice.tune(&snap_over, (7_600.0 / 120.0) as f32, 4);
        for _ in 0..10_000 {
            let s = voice.process(&mut noise);
            filter.process(s);
        }

        let mut rms_over = 0.0f64;
        for _ in 0..8_000 {
            let s = filter.process(voice.process(&mut noise)) as f64;
            rms_over += s * s;
        }
        rms_over = (rms_over / 8_000.0).sqrt();

        assert!(
            rms_over > 1.5 * rms_under,
            "cycle {cycle}: float band must rise sharply during overspeed excursion ({rms_over} vs {rms_under})"
        );

        // Cut intervenes, dropping engine back below float threshold
        voice.tune(&snap_under, (6_900.0 / 120.0) as f32, 4);
        for _ in 0..10_000 {
            let s = voice.process(&mut noise);
            filter.process(s);
        }

        let mut rms_recovered = 0.0f64;
        for _ in 0..8_000 {
            let s = filter.process(voice.process(&mut noise)) as f64;
            rms_recovered += s * s;
        }
        rms_recovered = (rms_recovered / 8_000.0).sqrt();

        assert!(
            rms_recovered < rms_over * 0.75,
            "cycle {cycle}: float band must disappear once speed drops below float ({rms_recovered} vs {rms_over})"
        );
    }
}

#[test]
fn mono_and_multichannel_buffers_are_filled_completely() {
    for channels in [1usize, 2, 4] {
        let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
        synth.set_snapshot(&loaded_snapshot());
        let mut buffer = vec![f32::NAN; 512 * channels];
        synth.render(&mut buffer, channels);
        assert!(
            buffer.iter().all(|s| s.is_finite()),
            "{channels} channels: buffer left unwritten"
        );
    }
}

#[test]
fn ragged_buffer_lengths_are_handled() {
    let mut synth = EngineSynth::new(SynthConfig::cross_plane_v8(FS));
    synth.set_snapshot(&loaded_snapshot());
    // A length that is not a whole number of stereo frames.
    let mut buffer = vec![f32::NAN; 101];
    synth.render(&mut buffer, 2);
    assert!(buffer.iter().all(|s| s.is_finite()));
    let mut empty: Vec<f32> = Vec::new();
    synth.render(&mut empty, 2);
}

#[test]
fn degenerate_configs_do_not_panic() {
    let mut config = SynthConfig::uniform(FS, 1, 1);
    config.cylinders.clear();
    config.bank_count = 0;
    let mut synth = EngineSynth::new(config);
    synth.set_snapshot(&loaded_snapshot());
    assert!(render(&mut synth, 4_800).iter().all(|s| s.is_finite()));

    // A tap pointing outside the bank list must be folded, not panic.
    let mut config = SynthConfig::uniform(FS, 4, 2);
    config.cylinders[2].bank = 99;
    let mut synth = EngineSynth::new(config);
    synth.set_snapshot(&loaded_snapshot());
    assert!(render(&mut synth, 4_800).iter().all(|s| s.is_finite()));
}

#[test]
fn roots_supercharger_whine_tracks_crank_order_with_no_lag() {
    let voicing = RootsVoicing {
        belt_ratio: 2.0,
        lobes: 4,
        level: 0.05,
    };
    assert_eq!(voicing.order(), 8.0);

    let mut voice = RootsVoice::new(FS);
    voice.tune(&voicing, 3_000.0, REFERENCE_INTAKE_FLOW);
    let crank_hz_1 = 3_000.0 / 60.0;
    let expected_hz_1 = crank_hz_1 * 8.0;
    assert!(
        (voice.tone_hz.target() - expected_hz_1).abs() < 1e-3,
        "expected tone target {expected_hz_1}, got {}",
        voice.tone_hz.target()
    );

    // Immediate step in RPM: tone target updates instantaneously without spool lag
    voice.tune(&voicing, 6_000.0, REFERENCE_INTAKE_FLOW);
    let crank_hz_2 = 6_000.0 / 60.0;
    let expected_hz_2 = crank_hz_2 * 8.0;
    assert!(
        (voice.tone_hz.target() - expected_hz_2).abs() < 1e-3,
        "expected tone target {expected_hz_2}, got {}",
        voice.tone_hz.target()
    );

    // Render samples to ensure output is finite and nonzero
    let mut sum = 0.0f32;
    for _ in 0..1000 {
        let s = voice.process();
        assert!(s.is_finite());
        sum += s.abs();
    }
    assert!(sum > 0.01, "roots voice produced silence");
}

#[test]
fn centrifugal_supercharger_whine_tracks_shaft_order_without_lag() {
    let voicing = CentrifugalVoicing {
        gear_ratio: 9.0,
        order: 1.5,
        level: 0.04,
    };
    assert_eq!(voicing.crank_order(), 13.5);

    let mut voice = CentrifugalVoice::new(FS);
    voice.tune(&voicing, 3_000.0, REFERENCE_INTAKE_FLOW);
    let crank_hz_1 = 3_000.0 / 60.0;
    let expected_hz_1 = crank_hz_1 * 13.5;
    assert!(
        (voice.tone_hz.target() - expected_hz_1).abs() < 1e-3,
        "expected centrifugal tone target {expected_hz_1}, got {}",
        voice.tone_hz.target()
    );

    // Immediate step in RPM: tone target updates instantaneously without spool lag
    voice.tune(&voicing, 6_000.0, REFERENCE_INTAKE_FLOW);
    let crank_hz_2 = 6_000.0 / 60.0;
    let expected_hz_2 = crank_hz_2 * 13.5;
    assert!(
        (voice.tone_hz.target() - expected_hz_2).abs() < 1e-3,
        "expected centrifugal tone target {expected_hz_2}, got {}",
        voice.tone_hz.target()
    );

    let mut sum = 0.0f32;
    for _ in 0..1000 {
        let s = voice.process();
        assert!(s.is_finite());
        sum += s.abs();
    }
    assert!(sum > 0.01, "centrifugal voice produced silence");
}

#[test]
fn dump_valve_fires_on_lift_with_boost_present_and_never_without_boost() {
    let voicing = BlowOffVoicing::default();
    let ref_rpm = 130_000.0;
    let mut noise = Noise::new(42);

    // Case 1: Lift without boost (turbo shaft not spinning)
    let mut voice = BlowOffVoice::new(FS);
    // Throttle opened without boost
    voice.tune(&voicing, 1.0, 0.0, ref_rpm);
    // Driver lifts
    voice.tune(&voicing, 0.0, 0.0, ref_rpm);
    assert_eq!(
        voice.envelope, 0.0,
        "dump valve must not trigger without boost"
    );
    let mut silent_sum = 0.0f32;
    for _ in 0..2000 {
        silent_sum += voice.process(&mut noise).abs();
    }
    assert_eq!(
        silent_sum, 0.0,
        "dump valve without boost must produce zero audio"
    );

    // Case 2: Lift with boost (turbo spinning fast)
    // Throttle opened with boost
    voice.tune(&voicing, 1.0, 110_000.0, ref_rpm);
    // Driver lifts
    voice.tune(&voicing, 0.0, 110_000.0, ref_rpm);
    let initial_env = voice.envelope;
    assert!(
        initial_env > 0.5,
        "dump valve must trigger on lift with boost present, got envelope {initial_env}"
    );

    // Process venting whoosh
    let mut whoosh_sum = 0.0f32;
    for _ in 0..2000 {
        let s = voice.process(&mut noise);
        assert!(s.is_finite());
        whoosh_sum += s.abs();
    }
    assert!(
        whoosh_sum > 1.0,
        "dump valve with boost must produce audible venting whoosh"
    );
    assert!(
        voice.envelope < initial_env,
        "dump valve envelope must decay while venting"
    );

    // Advance further (~0.5s) so the envelope decays fully
    for _ in 0..24_000 {
        voice.process(&mut noise);
    }
    assert!(
        voice.envelope < 0.05,
        "dump valve envelope should decay toward zero over half a second, got {}",
        voice.envelope
    );
}

#[test]
fn wastegate_chatter_flutters_at_high_boost_under_load() {
    let voicing = WastegateVoicing::default();
    let ref_rpm = 130_000.0;
    let mut noise = Noise::new(1234);
    let mut voice = WastegateVoice::new(FS);

    // Under low throttle or low boost, wastegate stays quiet
    voice.tune(&voicing, 0.2, 50_000.0, ref_rpm);
    let mut quiet_sum = 0.0f32;
    for _ in 0..1000 {
        quiet_sum += voice.process(&mut noise).abs();
    }
    assert_eq!(
        quiet_sum, 0.0,
        "wastegate must not chatter below boost and load threshold"
    );

    // Under high boost and heavy throttle, wastegate flutters
    voice.tune(&voicing, 1.0, 125_000.0, ref_rpm);
    // Advance smoothing
    for _ in 0..500 {
        voice.process(&mut noise);
    }
    let mut chatter_sum = 0.0f32;
    for _ in 0..1000 {
        let s = voice.process(&mut noise);
        assert!(s.is_finite());
        chatter_sum += s.abs();
    }
    assert!(
        chatter_sum > 0.1,
        "wastegate must chatter under high boost and heavy load, got sum {chatter_sum}"
    );
}
