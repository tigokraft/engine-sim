//! Drives the physics and the audio engine together over a scripted rev cycle.
//!
//! ```text
//! cargo run --release --example engine_audio
//! cargo run --release --example engine_audio -- --offline engine.wav
//! cargo run --release --example engine_audio -- --seconds 20 --gain 0.3
//! ```
//!
//! Live mode opens the default output device and feeds it from a 240 Hz physics
//! loop, printing the stream's health once a second. Offline mode runs the same
//! simulation and the same synth against a virtual clock, writes a 32-bit float
//! WAV, and measures the two things that matter for continuity:
//!
//! - the largest sample-to-sample step in the output, which is what a "pop"
//!   physically is, and
//! - whether any block of samples came out silent, which is what a dropout
//!   would look like.
//!
//! Offline mode needs no audio hardware, so it is the mode to run in CI.

use std::env;
use std::path::Path;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};

use rust_engine_sim::analysis::render::write_wav;
use rust_engine_sim::audio::dsp::EngineSynth;
use rust_engine_sim::audio::{EngineAudio, EngineControls, Induction, SnapshotSource, SynthConfig};
use rust_engine_sim::environment::Environment;
use rust_engine_sim::physics::engine_block::EngineBlock;

/// Physics updates per second.
const PHYSICS_HZ: f64 = 240.0;
/// Sample rate used for offline rendering.
const OFFLINE_RATE: f32 = 48_000.0;

/// The induction fitted to this example's V8.
///
/// The block itself is atmospheric, and so is the catalogue engine it is built
/// from; forced induction lives in the audio path, and this example bolts on a
/// pair because the lift-off surge is one of the layers it sets out to
/// demonstrate.
const DEMO_INDUCTION: Induction = Induction::twin();

// ---------------------------------------------------------------------------
// The drive cycle
// ---------------------------------------------------------------------------

/// Where the engine is in the scripted run, as a function of elapsed time.
///
/// The script is chosen to exercise every acoustic layer in turn rather than to
/// be a realistic lap: a slow pull for the exhaust and induction, a limiter
/// bounce for the backfires, and a hard lift for the turbo surge.
fn drive_cycle(t: f64, total: f64) -> (f64, EngineControls) {
    let phase = (t / total).clamp(0.0, 1.0);
    match phase {
        // Idle: quiet, cold-ish pipe, no boost.
        p if p < 0.12 => (
            850.0,
            EngineControls {
                throttle: 0.06,
                spark_cut: false,
            },
        ),
        // Pull to the redline. Throttle leads rpm, so the turbo spools first.
        p if p < 0.55 => {
            let u = (p - 0.12) / 0.43;
            (
                850.0 + 6_150.0 * u,
                EngineControls {
                    throttle: (0.15 + 1.6 * u).min(1.0),
                    spark_cut: false,
                },
            )
        }
        // On the limiter: throttle wide, ignition cutting. Unburnt fuel meets a
        // hot pipe, which is where the pops come from.
        p if p < 0.70 => {
            let u = (p - 0.55) / 0.15;
            (
                7_000.0 - 120.0 * (u * 24.0).sin().abs(),
                EngineControls::on_the_limiter(1.0),
            )
        }
        // Lift. The shaft is still turning hard into a shut throttle: surge.
        p if p < 0.85 => {
            let u = (p - 0.70) / 0.15;
            (
                7_000.0 - 4_500.0 * u,
                EngineControls {
                    throttle: 0.0,
                    spark_cut: false,
                },
            )
        }
        // Settle back to idle.
        p => {
            let u = (p - 0.85) / 0.15;
            (
                2_500.0 - 1_650.0 * u,
                EngineControls {
                    throttle: 0.05,
                    spark_cut: false,
                },
            )
        }
    }
}

// ---------------------------------------------------------------------------
// Offline rendering
// ---------------------------------------------------------------------------

/// Renders the drive cycle to a buffer without touching audio hardware.
fn render_offline(seconds: f64, gain: f64) -> (Vec<f32>, usize) {
    let mut block = EngineBlock::cross_plane_v8(Environment::default());
    let dt = 1.0 / PHYSICS_HZ;

    // Prime the phase ring so the first snapshot describes a running engine
    // rather than a cylinder full of ambient air.
    for _ in 0..600 {
        block.update(dt, 850.0);
    }

    // The catalogue V8 is atmospheric. This example exists partly to
    // demonstrate the surge on the lift, so it fits a pair of turbos itself.
    let mut config = SynthConfig::from_block(&block, OFFLINE_RATE).with_induction(DEMO_INDUCTION);
    config.master_gain *= gain;
    let mut synth = EngineSynth::new(config);
    let mut source = SnapshotSource::with_induction(&block, DEMO_INDUCTION);

    let frames_per_step = (OFFLINE_RATE as f64 / PHYSICS_HZ).round() as usize;
    let steps = (seconds * PHYSICS_HZ) as usize;
    let mut output = Vec::with_capacity(steps * frames_per_step * 2);
    let mut chunk = vec![0.0f32; frames_per_step * 2];

    for step in 0..steps {
        let t = step as f64 * dt;
        let (rpm, controls) = drive_cycle(t, seconds);
        block.update(dt, rpm);
        synth.set_snapshot(&source.sample(&block, rpm, dt, controls));
        synth.render(&mut chunk, 2);
        output.extend_from_slice(&chunk);
    }

    (output, 2)
}

/// Continuity report for a rendered buffer.
struct Continuity {
    peak: f32,
    rms: f32,
    max_slew: f32,
    silent_blocks: usize,
    non_finite: usize,
}

/// Measures the things that distinguish clean output from a glitchy one.
fn analyse(samples: &[f32], channels: usize) -> Continuity {
    let mut peak = 0.0f32;
    let mut energy = 0.0f64;
    let mut non_finite = 0;
    for s in samples {
        if !s.is_finite() {
            non_finite += 1;
            continue;
        }
        peak = peak.max(s.abs());
        energy += (*s as f64) * (*s as f64);
    }

    // A pop is a step discontinuity, so measure the largest jump between
    // consecutive samples of the same channel.
    let mut max_slew = 0.0f32;
    for channel in 0..channels {
        let mut previous = 0.0f32;
        for (i, frame) in samples.chunks(channels).enumerate() {
            let value = frame[channel];
            if i > 0 && value.is_finite() && previous.is_finite() {
                max_slew = max_slew.max((value - previous).abs());
            }
            previous = value;
        }
    }

    // A dropout is a run of samples the synth failed to fill. Ignore the first
    // half-second, which is legitimately near-silent while the fade-in runs and
    // the engine is idling.
    let block = 256 * channels;
    let skip = (OFFLINE_RATE as usize / 2) * channels;
    let silent_blocks = samples[skip.min(samples.len())..]
        .chunks(block)
        .filter(|c| c.len() == block && c.iter().all(|s| s.abs() < 1e-7))
        .count();

    Continuity {
        peak,
        rms: (energy / samples.len().max(1) as f64).sqrt() as f32,
        max_slew,
        silent_blocks,
        non_finite,
    }
}

// ---------------------------------------------------------------------------
// Live playback
// ---------------------------------------------------------------------------

fn run_live(seconds: f64, gain: f64) -> Result<()> {
    let mut block = EngineBlock::cross_plane_v8(Environment::default());
    let dt = 1.0 / PHYSICS_HZ;
    for _ in 0..600 {
        block.update(dt, 850.0);
    }

    // The sample rate here is provisional: `EngineAudio` overwrites it with
    // whatever the device negotiates before building the synth.
    let mut config = SynthConfig::from_block(&block, 48_000.0).with_induction(DEMO_INDUCTION);
    config.master_gain *= gain;

    let mut audio = EngineAudio::start(config).context("could not open the audio device")?;
    let info = audio.info().clone();
    println!("== audio device ==");
    println!("  device            {}", info.device);
    println!(
        "  sample rate       {} Hz{}",
        info.sample_rate,
        if info.sample_rate_honoured {
            ""
        } else {
            "  (device would not accept 48 kHz)"
        }
    );
    println!("  channels          {}", info.channels);
    match info.buffer_frames {
        Some(frames) => println!(
            "  buffer            {frames} frames  ({:.2} ms)",
            frames as f64 * 1e3 / info.sample_rate as f64
        ),
        None => println!("  buffer            device default"),
    }
    println!("\nrunning a {seconds:.0} s drive cycle at gain {gain:.2}\n");

    let mut source = SnapshotSource::with_induction(&block, DEMO_INDUCTION);
    let start = Instant::now();
    let mut next_report = 1.0;
    let mut next_step = start;

    loop {
        let elapsed = start.elapsed().as_secs_f64();
        if elapsed >= seconds {
            break;
        }

        let (rpm, controls) = drive_cycle(elapsed, seconds);
        block.update(dt, rpm);
        audio.push(source.sample(&block, rpm, dt, controls));

        if elapsed >= next_report {
            let stats = audio.stats();
            println!(
                "  t={elapsed:5.1}s  {rpm:6.0} rpm  throttle {:.2}  turbo {:6.0} rpm  \
                 peak {:.3}  callbacks {:6}  starved {:.1}%  errors {}  dropped {}",
                controls.throttle,
                source.turbo_shaft_rpm(),
                stats.peak(),
                stats.callbacks(),
                stats.starvation_ratio() * 100.0,
                stats.errors(),
                audio.dropped_snapshots(),
            );
            next_report += 1.0;
        }

        // Pace the simulation against the wall clock so the engine note runs at
        // the right speed rather than as fast as the CPU allows.
        next_step += Duration::from_secs_f64(dt);
        if let Some(wait) = next_step.checked_duration_since(Instant::now()) {
            std::thread::sleep(wait);
        } else {
            // Fell behind: give up on catching up rather than spiralling.
            next_step = Instant::now();
        }
    }

    let stats = audio.stats();
    println!("\n== stream health ==");
    println!("  callbacks         {}", stats.callbacks());
    println!("  frames rendered   {}", stats.frames());
    println!(
        "  starved           {} ({:.2} %)",
        stats.starved(),
        stats.starvation_ratio() * 100.0
    );
    println!("  stream errors     {}", stats.errors());
    println!("  device lost       {}", stats.disconnected());
    println!("  snapshots dropped {}", audio.dropped_snapshots());

    audio.pause().ok();
    Ok(())
}

// ---------------------------------------------------------------------------

fn main() -> Result<()> {
    let mut seconds: f64 = 12.0;
    let mut gain: f64 = 1.0;
    let mut offline: Option<String> = None;

    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--offline" => offline = Some(args.next().unwrap_or_else(|| "engine.wav".into())),
            "--seconds" => seconds = args.next().and_then(|v| v.parse().ok()).unwrap_or(seconds),
            "--gain" => gain = args.next().and_then(|v| v.parse().ok()).unwrap_or(gain),
            "--help" | "-h" => {
                println!("engine_audio [--offline FILE] [--seconds N] [--gain G]");
                return Ok(());
            }
            other => eprintln!("ignoring unknown argument: {other}"),
        }
    }
    let seconds = seconds.clamp(0.5, 600.0);
    let gain = gain.clamp(0.0, 4.0);

    match offline {
        None => run_live(seconds, gain),
        Some(path) => {
            let started = Instant::now();
            let (samples, channels) = render_offline(seconds, gain);
            let render_time = started.elapsed().as_secs_f64();
            let report = analyse(&samples, channels);

            write_wav(
                Path::new(&path),
                &samples,
                channels as u16,
                OFFLINE_RATE as u32,
            )?;

            println!("== offline render ==");
            println!("  file              {path}");
            println!(
                "  audio             {:.2} s stereo at {} Hz",
                samples.len() as f64 / (channels as f64 * OFFLINE_RATE as f64),
                OFFLINE_RATE as u32
            );
            println!(
                "  render time       {render_time:.2} s  ({:.0}x real time)",
                seconds / render_time.max(1e-9)
            );
            println!("\n== continuity ==");
            println!("  peak              {:.4}", report.peak);
            println!("  rms               {:.4}", report.rms);
            println!(
                "  max sample step   {:.4}  (a pop would be a step near full scale)",
                report.max_slew
            );
            println!("  silent blocks     {}", report.silent_blocks);
            println!("  non-finite        {}", report.non_finite);

            let clean = report.non_finite == 0
                && report.silent_blocks == 0
                && report.peak <= 1.0
                && report.max_slew < 0.5;
            println!(
                "\n  verdict           {}",
                if clean {
                    "continuous: no dropouts, no discontinuities"
                } else {
                    "FAILED continuity checks"
                }
            );
            if !clean {
                anyhow::bail!("offline render failed its continuity checks");
            }
            Ok(())
        }
    }
}
