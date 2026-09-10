//! Interactive rev demo — drive the engine from the keyboard and listen.
//!
//! ```text
//! cargo run --release --example engine_rev            # drive it yourself
//! cargo run --release --example engine_rev -- --auto  # hands-free, no TTY needed
//! ```
//!
//! ```text
//!   W / Up      open the throttle          SPACE   hold the ignition cut
//!   S / Down    close the throttle         0-9     set throttle directly
//!   Q / Esc     quit
//! ```
//!
//! Unlike [`engine_audio`](engine_audio.rs), which follows a fixed script, this
//! lets the engine find its own speed: throttle produces torque, torque
//! accelerates a flywheel, and the resulting rpm feeds back into the solver. So
//! the note you hear is the engine responding to load rather than to a ramp.
//!
//! Things worth listening for:
//!
//! - **The cross-plane burble.** Each bank fires at 90-180-270-180 degree gaps
//!   rather than evenly, and the banks are panned apart, so the offbeat lives in
//!   the stereo image. Try headphones.
//! - **Backfires.** Hold the throttle open into the limiter, or press SPACE at
//!   any speed. Unburnt fuel meeting a hot runner is what makes the pops, so
//!   they are louder and more frequent when the pipe is hot and the engine is
//!   swallowing a lot of fuel.
//! - **Turbo surge.** Pull to high rpm, then close the throttle with S. The
//!   shaft is still turning hard into a shut plate — that chuffing is surge.
//! - **The pipe warming up.** From a standing start the exhaust is cold, so both
//!   the runner resonance and the muffler cavity sit low and climb as the gas
//!   heats. Both scale with `sqrt(T)`, so they rise together.
//!
//! # A caveat on the throttle
//!
//! The 0D block has no throttle plate: it is given a speed and solves the cycle.
//! The pedal here scales the solved torque, which is enough to make the engine
//! drivable and to feed the audio a throttle position, but it is a demo control
//! rather than a manifold-pressure model. Everything downstream of it — pressure
//! at EVO, exhaust temperature, induction flow — is the solver's own output.

use std::io::{stdout, Write};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use crossterm::event::{self, Event, KeyCode, KeyEventKind};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode};

use rust_engine_sim::audio::{EngineAudio, EngineControls, Induction, SnapshotSource, SynthConfig};
use rust_engine_sim::environment::Environment;
use rust_engine_sim::physics::engine_block::EngineBlock;

/// Physics updates per second.
const PHYSICS_HZ: f64 = 240.0;
/// Rotating inertia of crank, rods and flywheel [kg m^2].
const INERTIA: f64 = 0.45;
/// Ignition cuts above this speed [rev/min].
const LIMITER_RPM: f64 = 7_000.0;
/// Speed the idle governor holds [rev/min].
const IDLE_RPM: f64 = 850.0;
/// Below this the engine has stalled [rev/min].
const STALL_RPM: f64 = 400.0;

/// The induction fitted to this example's V8.
///
/// The block itself is atmospheric, and so is the catalogue engine it is built
/// from; forced induction lives in the audio path, and this example bolts on a
/// pair because the lift-off surge is one of the layers it sets out to
/// demonstrate.
const DEMO_INDUCTION: Induction = Induction::twin();

/// Restores the terminal however the program exits, including on a panic.
///
/// Raw mode is a global change to the user's terminal: leaving it on turns the
/// shell unusable after a crash, so the restore has to run from a destructor
/// rather than from the end of `main`.
struct RawMode;

impl Drop for RawMode {
    fn drop(&mut self) {
        let _ = disable_raw_mode();
        let _ = stdout().flush();
    }
}

/// The bit of driveline the block does not model: a flywheel with a load on it.
struct Driveline {
    /// Crankshaft speed [rev/min].
    rpm: f64,
    /// Throttle actually applied, after pedal travel [-].
    throttle: f64,
    /// Where the pedal is being asked to go [-].
    throttle_target: f64,
    /// Whether the driver is holding the ignition cut.
    manual_cut: bool,
}

impl Driveline {
    fn new() -> Self {
        Self {
            rpm: IDLE_RPM,
            throttle: 0.0,
            throttle_target: 0.0,
            manual_cut: false,
        }
    }

    /// Whether ignition is cut this frame, by the limiter or by the driver.
    fn cutting(&self) -> bool {
        self.manual_cut || self.rpm >= LIMITER_RPM
    }

    /// Advances the flywheel one frame from the block's solved torque.
    fn update(&mut self, block: &EngineBlock, dt: f64) {
        // Pedal travel. A step input would be both unrealistic and a parameter
        // discontinuity for the audio thread to chase.
        let slew = 1.0 - (-dt / 0.12).exp();
        self.throttle += (self.throttle_target - self.throttle) * slew;

        // Idle governor: enough throttle to hold IDLE_RPM, and no more. This is
        // what stops the engine stalling the moment the pedal comes up.
        let governor = ((IDLE_RPM + 60.0 - self.rpm) / 500.0).clamp(0.0, 0.30);
        let effective = self.throttle.max(governor);

        // The block solves torque for a speed, not for a throttle, so scale it
        // here. See the module docs.
        let drive = if self.cutting() {
            0.0
        } else {
            block.mean_brake_torque(self.rpm) * (0.05 + 0.95 * effective)
        };

        // Accessories, then bearing drag, then windage. The quadratic term
        // gives the engine a natural free-revving ceiling instead of an
        // unbounded climb, but it has to stay well under the torque the block
        // actually makes (~250 N m) at the limiter, or the engine plateaus below
        // it and never bounces.
        let omega = self.rpm * std::f64::consts::PI / 30.0;
        let load = 6.0 + 0.02 * omega + 1.5e-4 * omega * omega;

        let alpha = (drive - load) / INERTIA;
        let omega = (omega + alpha * dt).max(STALL_RPM * std::f64::consts::PI / 30.0);
        self.rpm = omega * 30.0 / std::f64::consts::PI;
    }

    fn controls(&self) -> EngineControls {
        EngineControls {
            throttle: self.throttle.clamp(0.0, 1.0),
            spark_cut: self.cutting(),
        }
    }
}

/// Handles one key press. Returns `false` when the driver wants to quit.
fn handle_key(code: KeyCode, driveline: &mut Driveline) -> bool {
    match code {
        KeyCode::Char('q') | KeyCode::Char('Q') | KeyCode::Esc => return false,
        KeyCode::Char('w') | KeyCode::Char('W') | KeyCode::Up => driveline.throttle_target = 1.0,
        KeyCode::Char('s') | KeyCode::Char('S') | KeyCode::Down => driveline.throttle_target = 0.0,
        // SPACE toggles rather than tracks the key: terminals report a key
        // press, not a key being held, so a hold-to-cut control would either
        // stutter with the auto-repeat delay or never release.
        KeyCode::Char(' ') => driveline.manual_cut = !driveline.manual_cut,
        KeyCode::Char(c @ '0'..='9') => {
            driveline.throttle_target = (c as u8 - b'0') as f64 / 9.0;
        }
        _ => {}
    }
    true
}

/// Pedal position for the hands-free demo at time `t` [s].
///
/// This scripts the *throttle*, not the speed: where `engine_audio` ramps rpm
/// directly, everything here has to come back through torque and the flywheel,
/// so the engine finds the limiter on its own and falls off it on its own.
fn auto_pedal(t: f64) -> (f64, bool) {
    match t {
        t if t < 1.5 => (0.0, false),  // idle
        t if t < 6.0 => (1.0, false),  // pull to the limiter and bounce on it
        t if t < 9.0 => (0.0, false),  // lift: turbo surge
        t if t < 10.0 => (1.0, false), // blip
        t if t < 11.0 => (0.0, true),  // overrun on a cut: pops
        _ => (0.0, false),             // settle back to idle
    }
}

/// Runs the scripted pedal sequence without touching the terminal.
fn run_auto(
    mut block: EngineBlock,
    mut audio: EngineAudio,
    mut source: SnapshotSource,
    seconds: f64,
) -> Result<()> {
    let dt = 1.0 / PHYSICS_HZ;
    let mut driveline = Driveline::new();
    let start = Instant::now();
    let mut next_step = start;
    let mut next_report = start;
    let mut reached_limiter = false;

    loop {
        let elapsed = start.elapsed().as_secs_f64();
        if elapsed >= seconds {
            break;
        }
        let (pedal, cut) = auto_pedal(elapsed);
        driveline.throttle_target = pedal;
        driveline.manual_cut = cut;

        driveline.update(&block, dt);
        block.update(dt, driveline.rpm);
        audio.push(source.sample(&block, driveline.rpm, dt, driveline.controls()));
        reached_limiter |= driveline.rpm >= LIMITER_RPM;

        let now = Instant::now();
        if now >= next_report {
            println!(
                "  t={elapsed:5.1}s{}",
                status(&driveline, &source, audio.stats().peak())
            );
            next_report = now + Duration::from_millis(250);
        }

        next_step += Duration::from_secs_f64(dt);
        match next_step.checked_duration_since(Instant::now()) {
            Some(wait) => std::thread::sleep(wait),
            None => next_step = Instant::now(),
        }
    }

    audio.pause().ok();
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
    println!("  snapshots dropped {}", audio.dropped_snapshots());
    println!(
        "  reached limiter   {}",
        if reached_limiter {
            "yes"
        } else {
            "NO — flywheel never got there"
        }
    );
    Ok(())
}

/// One line of live telemetry, redrawn in place.
fn status(driveline: &Driveline, source: &SnapshotSource, peak: f32) -> String {
    let bar = |value: f64, width: usize| {
        let filled = ((value.clamp(0.0, 1.0)) * width as f64).round() as usize;
        format!("{}{}", "#".repeat(filled), "-".repeat(width - filled))
    };
    let tacho = (driveline.rpm / LIMITER_RPM).clamp(0.0, 1.0);

    format!(
        "  {:5.0} rpm [{}]  throttle [{}]  turbo {:6.0}  peak {:.2}  {}",
        driveline.rpm,
        bar(tacho, 24),
        bar(driveline.throttle, 10),
        source.turbo_shaft_rpm(),
        peak,
        if driveline.manual_cut {
            "CUT     "
        } else if driveline.rpm >= LIMITER_RPM {
            "LIMITER "
        } else {
            "        "
        },
    )
}

fn main() -> Result<()> {
    let mut auto = false;
    let mut seconds = 13.0;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--auto" => auto = true,
            "--seconds" => seconds = args.next().and_then(|v| v.parse().ok()).unwrap_or(seconds),
            "--help" | "-h" => {
                println!("engine_rev [--auto] [--seconds N]");
                return Ok(());
            }
            other => eprintln!("ignoring unknown argument: {other}"),
        }
    }

    let mut block = EngineBlock::cross_plane_v8(Environment::default());
    let dt = 1.0 / PHYSICS_HZ;

    // Prime the phase ring so the first sound is a running engine rather than a
    // cylinder full of ambient air.
    for _ in 0..600 {
        block.update(dt, IDLE_RPM);
    }

    // The catalogue V8 is atmospheric, and half the point of this example is
    // the surge on the lift, so it fits a pair of turbos itself.
    let config = SynthConfig::from_block(&block, 48_000.0).with_induction(DEMO_INDUCTION);
    let source = SnapshotSource::with_induction(&block, DEMO_INDUCTION);
    let mut audio = EngineAudio::start(config).context("could not open the audio device")?;
    let info = audio.info().clone();

    println!();
    println!(
        "  {} — {} Hz, {} ch{}",
        info.device,
        info.sample_rate,
        info.channels,
        match info.buffer_frames {
            Some(frames) => format!(", {frames}-frame buffer"),
            None => String::new(),
        }
    );
    println!();
    println!("  W / Up    open throttle        SPACE  toggle ignition cut");
    println!("  S / Down  close throttle       0-9    set throttle");
    println!("  Q / Esc   quit");
    println!();
    if auto {
        println!("  hands-free demo: idle -> limiter -> lift -> blip -> overrun cut");
        println!();
        return run_auto(block, audio, source, seconds);
    }

    println!("  Try: hold W into the limiter for pops, then S for turbo surge.");
    println!();

    // From here on the terminal is in raw mode, so lines need an explicit \r.
    enable_raw_mode().context("could not put the terminal in raw mode")?;
    let _restore = RawMode;

    let mut driveline = Driveline::new();
    let mut source = source;
    let mut next_step = Instant::now();
    let mut next_redraw = Instant::now();
    let mut out = stdout();

    'driving: loop {
        // Drain every key waiting, so a burst of auto-repeat does not queue up
        // and apply itself over the following frames.
        while event::poll(Duration::ZERO).unwrap_or(false) {
            if let Ok(Event::Key(key)) = event::read() {
                if key.kind == KeyEventKind::Press && !handle_key(key.code, &mut driveline) {
                    break 'driving;
                }
            }
        }

        driveline.update(&block, dt);
        block.update(dt, driveline.rpm);
        audio.push(source.sample(&block, driveline.rpm, dt, driveline.controls()));

        let now = Instant::now();
        if now >= next_redraw {
            write!(
                out,
                "\r{}",
                status(&driveline, &source, audio.stats().peak())
            )?;
            out.flush()?;
            next_redraw = now + Duration::from_millis(50);
        }

        // Pace against the wall clock so the engine runs at real speed rather
        // than as fast as the CPU allows.
        next_step += Duration::from_secs_f64(dt);
        match next_step.checked_duration_since(Instant::now()) {
            Some(wait) => std::thread::sleep(wait),
            // Fell behind: resynchronise rather than trying to catch up, which
            // would spiral on a loaded machine.
            None => next_step = Instant::now(),
        }
    }

    audio.pause().ok();
    let stats = audio.stats();
    write!(out, "\r\n\r\n  stream health: ")?;
    write!(
        out,
        "{} callbacks, {} frames, {} starved ({:.2} %), {} errors, {} dropped\r\n",
        stats.callbacks(),
        stats.frames(),
        stats.starved(),
        stats.starvation_ratio() * 100.0,
        stats.errors(),
        audio.dropped_snapshots(),
    )?;
    out.flush()?;
    Ok(())
}
