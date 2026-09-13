//! Drives the physics and the audio engine together over a scripted rev cycle.
//!
//! ```text
//! cargo run --release --example engine_audio
//! cargo run --release --example engine_audio -- --offline engine.wav
//! cargo run --release --example engine_audio -- --seconds 20 --gain 0.3
//! ```
//!
//! Live mode opens the default output device and feeds it from a 240 Hz physics
//! loop, printing the stream's health once a second. Offline mode hands the same
//! drive cycle to the measurement harness in
//! [`analysis::render`](rust_engine_sim::analysis::render), which runs the same
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

use rust_engine_sim::analysis::render::{RenderPlan, PHYSICS_HZ};
use rust_engine_sim::analysis::script::{RenderScript, Segment};
use rust_engine_sim::audio::{EngineAudio, Induction, SnapshotSource, SynthConfig};
use rust_engine_sim::bench::EnginePreset;
use rust_engine_sim::environment::Environment;

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

/// The scripted rev cycle, stretched to fill `seconds`.
///
/// Chosen to exercise every acoustic layer in turn rather than to be a
/// realistic lap: a slow pull for the exhaust and induction, a limiter bounce
/// for the backfires, and a hard lift for the turbo surge. The legs are written
/// as fractions of the whole and scaled at the end, so `--seconds` changes how
/// long the cycle takes without changing its shape.
///
/// It is a [`RenderScript`] like the measurement harness's own fixed profiles,
/// so this example and `measure` drive the engine through the same machinery.
fn drive_cycle(seconds: f64) -> RenderScript {
    let (idle, redline) = (850.0, 7_000.0);
    RenderScript::new(
        "demo",
        "a pull, a limiter bounce and a hard lift",
        vec![
            // Idle: quiet, cold-ish pipe, no boost.
            Segment::hold(0.12, idle, 0.06),
            // Pull to the redline. Throttle leads rpm, so the turbo spools first.
            Segment::ramp(0.43, (idle, redline), (0.15, 1.0)),
            // On the limiter: throttle wide, ignition cutting. Unburnt fuel meets
            // a hot pipe, which is where the pops come from.
            Segment::hold(0.15, redline, 1.0)
                .cutting()
                .bouncing(120.0, 3.0),
            // Lift. The shaft is still turning hard into a shut throttle: surge.
            Segment::ramp(0.15, (redline, 2_500.0), (0.0, 0.0)),
            // Settle back to idle.
            Segment::ramp(0.15, (2_500.0, idle), (0.05, 0.05)),
        ],
    )
    .scaled_to(seconds)
}

// ---------------------------------------------------------------------------
// Live playback
// ---------------------------------------------------------------------------

fn run_live(seconds: f64, gain: f64) -> Result<()> {
    let preset = EnginePreset::cross_plane_v8();
    let script = drive_cycle(seconds);
    let mut block = preset.block(Environment::default());
    let dt = 1.0 / PHYSICS_HZ;
    for _ in 0..600 {
        block.update(dt, script.start_rpm());
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

        let (rpm, controls) = script.at(elapsed);
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
            let preset = EnginePreset::cross_plane_v8();
            let script = drive_cycle(seconds);
            let render = RenderPlan::new(&preset, &script)
                .with_induction(DEMO_INDUCTION)
                .with_gain(gain)
                .render();
            let report = render.continuity();

            render.write_wav(Path::new(&path))?;

            println!("== offline render ==");
            println!("  file              {path}");
            println!(
                "  audio             {:.2} s stereo at {} Hz",
                render.seconds(),
                render.sample_rate as u32
            );
            println!(
                "  render time       {:.2} s  ({:.0}x real time)",
                render.cost.wall_seconds,
                render.cost.realtime_multiple()
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

            println!(
                "\n  verdict           {}",
                if report.is_clean() {
                    "continuous: no dropouts, no discontinuities"
                } else {
                    "FAILED continuity checks"
                }
            );
            if !report.is_clean() {
                anyhow::bail!("offline render failed its continuity checks");
            }
            Ok(())
        }
    }
}
