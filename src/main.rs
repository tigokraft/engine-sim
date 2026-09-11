//! The interactive benchtop engine simulator.
//!
//! ```text
//! cargo run --release
//! ```
//!
//! ```text
//!   Up / W, Down / S   throttle, 0-100 %
//!   SPACE              ignition cut — 2-step launch control, pops on the way out
//!   1 - 6              inline-4, cross-plane V8, flat-plane V8, V10, V12, 2-rotor
//!   M                  mute
//!   Q / Esc            quit
//! ```
//!
//! # Three rates, three threads
//!
//! ```text
//!   simulation thread            main thread                audio callback
//!   ─────────────────            ───────────                ──────────────
//!   480 Hz, elevated             60 fps                     device-driven, RT
//!   RK4 substepping              read telemetry             drain to newest
//!   flywheel + block             draw ratatui               synth.render()
//!        │                            │                          ▲
//!        ├─ Mutex<Telemetry> ────────▶│                          │
//!        │◀──── mpsc<Command> ────────┤                          │
//!        └─ rtrb<EngineSnapshot> ────────────────────────────────┘
//!                                     ◀──── rtrb<f32> scope tap ─┘
//! ```
//!
//! The three rates are unrelated on purpose, and nothing waits on anything else:
//!
//! - The **audio callback** has a hard deadline — about 10 ms at 48 kHz with a
//!   512-frame buffer — and misses it audibly. It therefore takes no lock and
//!   makes no allocation; see [`rust_engine_sim::audio::stream`].
//! - The **simulation thread** runs at a fixed 480 Hz, eight times the draw rate,
//!   because the audio wants fresh state far more often than a person wants a
//!   new picture. Decoupling it from the draw is what stops a slow terminal —
//!   over ssh, or scrolled, or resized — from changing the engine note.
//! - The **main thread** draws at 60 fps and owns the terminal. It reads a
//!   consistent snapshot of the telemetry under a mutex and never touches the
//!   simulation's own state.
//!
//! The simulation thread asks the scheduler for a real-time priority (see
//! [`request_realtime_priority`]) and carries on at normal priority if it is
//! refused; the header shows which it got.

use std::sync::mpsc::{self, Receiver, Sender, TryRecvError};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};

use rust_engine_sim::audio::{EngineAudio, EngineControls, SnapshotSource};
use rust_engine_sim::bench::{Driveline, EnginePreset};
use rust_engine_sim::environment::Environment;
use rust_engine_sim::physics::engine_block::{EngineBlock, PHASE_CELLS};
use rust_engine_sim::ui::telemetry::{
    AudioHealth, Command, CurveTrace, SharedTelemetry, SimEvent, Telemetry,
};
use rust_engine_sim::ui::terminal::{Dashboard, TerminalSession};

/// Simulation frames per second.
///
/// Each of these frames is itself substepped by the RK4 solver, which plans its
/// own angular step from the shaft speed — a degree of crank at a time under
/// normal conditions, stretched to four when the engine is spinning fast enough
/// that a degree would cost too many substeps. So the real integration rate at
/// 7000 rpm is around 20 kHz, not 480 Hz; this is the rate at which the flywheel
/// closes the loop and the audio thread is handed new state.
const SIM_HZ: f64 = 480.0;

/// Frames per second on the terminal.
const DRAW_HZ: f64 = 60.0;

/// How often the simulation publishes telemetry [Hz].
///
/// Matched to the draw rate: publishing faster would take the lock more often
/// for frames nobody draws.
const PUBLISH_HZ: f64 = 60.0;

/// The longest simulated frame that will be honoured [s].
///
/// If the thread is descheduled for longer than this — a laptop suspending, or
/// the machine paging heavily — the missing time is dropped rather than
/// simulated in one enormous catch-up step. Dropping it makes the engine appear
/// to hesitate; simulating it would put a single 200 ms step into a solver whose
/// substep budget is measured in degrees of crank, and the result would not be
/// an engine at all.
const MAX_CATCH_UP: f64 = 0.05;

fn main() -> Result<()> {
    let telemetry: SharedTelemetry = Arc::new(Mutex::new(Telemetry::default()));
    let (commands, command_rx) = mpsc::channel::<Command>();
    let (event_tx, events) = mpsc::channel::<SimEvent>();

    let simulation = {
        let telemetry = Arc::clone(&telemetry);
        thread::Builder::new()
            .name("engine-simulation".to_string())
            .spawn(move || simulation_thread(telemetry, command_rx, event_tx))
            .context("could not start the simulation thread")?
    };

    // The terminal is taken over only after the simulation is running, so an
    // early failure prints somewhere the user can read it.
    let result = run_dashboard(&telemetry, &commands, &events);

    // Whatever happened above — a clean quit, a draw error, or a panic already
    // unwound — the simulation is told to stop and given the chance to close the
    // audio device properly. The send failing means it has already gone.
    let _ = commands.send(Command::Quit);
    drop(commands);
    let stopped = simulation.join();

    // The terminal has been restored by now (`run_dashboard` drops its session
    // before returning), so anything printed here is visible.
    result?;
    match stopped {
        Ok(Ok(summary)) => {
            println!("{summary}");
            Ok(())
        }
        Ok(Err(error)) => Err(error),
        Err(_) => anyhow::bail!("the simulation thread panicked"),
    }
}

// ---------------------------------------------------------------------------
// Main thread: draw at 60 fps
// ---------------------------------------------------------------------------

/// Owns the terminal and draws until the driver quits.
fn run_dashboard(
    telemetry: &SharedTelemetry,
    commands: &Sender<Command>,
    events: &Receiver<SimEvent>,
) -> Result<()> {
    let mut session = TerminalSession::open()?;
    let mut dashboard = Dashboard::new();

    let frame_time = Duration::from_secs_f64(1.0 / DRAW_HZ);
    let mut next_frame = Instant::now();
    let mut last = Instant::now();

    loop {
        // Keystrokes first: a command issued this frame should reach the
        // simulation before the frame it will be seen in is drawn.
        dashboard.drain_input(|command| {
            let _ = commands.send(command);
        })?;

        for event in events.try_iter() {
            match event {
                SimEvent::AudioReady { info, scope } => {
                    let status = format!(
                        "{} · {} Hz · {} ch{}",
                        info.device,
                        info.sample_rate,
                        info.channels,
                        if info.sample_rate_honoured {
                            ""
                        } else {
                            " (device chose the rate)"
                        }
                    );
                    dashboard.attach_scope(*scope, status);
                }
                SimEvent::AudioFailed(reason) => dashboard.audio_failed(reason),
                SimEvent::Stopped => {}
            }
        }

        // One lock, one clone: the dashboard wants a consistent set of numbers,
        // and holding the lock across the draw would stall the simulation for
        // as long as the terminal takes to flush.
        match telemetry.lock() {
            Ok(published) => dashboard.telemetry.clone_from(&published),
            // A poisoned mutex means the simulation thread panicked while
            // holding it. The dashboard keeps its last good frame; the join in
            // `main` will surface the panic.
            Err(poisoned) => dashboard.telemetry.clone_from(&poisoned.into_inner()),
        }

        let now = Instant::now();
        dashboard.tick(now - last);
        last = now;

        session
            .terminal()
            .draw(|frame| dashboard.draw(frame))
            .context("could not draw the dashboard")?;

        if dashboard.quitting() {
            break;
        }

        // Fixed cadence rather than sleep-per-frame, so a slow draw is absorbed
        // by the next frame instead of making every frame late.
        next_frame += frame_time;
        match next_frame.checked_duration_since(Instant::now()) {
            Some(wait) => thread::sleep(wait),
            None => next_frame = Instant::now(),
        }
    }

    // Explicit, so the terminal is restored before `main` prints anything.
    drop(session);
    Ok(())
}

// ---------------------------------------------------------------------------
// Simulation thread
// ---------------------------------------------------------------------------

/// One engine, its flywheel, and the audio stream it is feeding.
///
/// Grouped because switching presets replaces all of them together: a different
/// cylinder count means a different [`SynthConfig`], which means a new stream.
struct Rig {
    preset: EnginePreset,
    index: usize,
    block: EngineBlock,
    driveline: Driveline,
    source: SnapshotSource,
    audio: Option<EngineAudio>,
    curve: CurveTrace,
}

impl Rig {
    /// Builds a rig on catalogue entry `index`, opening an audio stream for it.
    ///
    /// A failure to open the device is reported and then ignored: an engine
    /// simulator with no sound card is still a working engine simulator, and
    /// exiting would be a worse answer than a silent one.
    fn new(index: usize, environment: Environment, events: &Sender<SimEvent>) -> Self {
        let catalogue = EnginePreset::catalogue();
        let index = index.min(catalogue.len() - 1);
        let preset = catalogue[index].clone();

        // The dashboard is somebody starting an engine, not a dyno reading a
        // steady state: it begins stone cold and warms up while they drive it.
        let mut block = preset.block(environment);
        block.cold_start();
        let driveline = Driveline::new(&preset);
        // Both halves of the induction come from the preset, so a turbo that
        // spins is a turbo that is heard and an atmospheric engine has neither.
        let source = preset.snapshot_source(&block);

        // The stream replaces this sample rate with whatever the device
        // negotiates and rebuilds the synth for it, so the engine is always
        // tuned to the rate it will actually be played at.
        let config = preset.synth_config(&block, 48_000.0);
        let audio = match EngineAudio::start(config) {
            Ok(mut audio) => {
                let info = Box::new(audio.info().clone());
                match audio.take_scope() {
                    Some(scope) => {
                        let _ = events.send(SimEvent::AudioReady {
                            info,
                            scope: Box::new(scope),
                        });
                    }
                    None => {
                        let _ = events.send(SimEvent::AudioFailed(
                            "the output tap was already taken".to_string(),
                        ));
                    }
                }
                Some(audio)
            }
            Err(error) => {
                // The whole chain, not just the top: "no default audio output
                // device" is useful, "failed to open a stream" on its own is not.
                let mut reason = format!("silent — {error}");
                for cause in error.chain().skip(1) {
                    reason.push_str(&format!(": {cause}"));
                }
                let _ = events.send(SimEvent::AudioFailed(reason));
                None
            }
        };

        Self {
            preset,
            index,
            block,
            driveline,
            source,
            audio,
            curve: CurveTrace::new(),
        }
    }
}

/// Runs the engine until told to stop. Returns the closing summary.
fn simulation_thread(
    telemetry: SharedTelemetry,
    commands: Receiver<Command>,
    events: Sender<SimEvent>,
) -> Result<String> {
    let realtime = request_realtime_priority();
    let environment = Environment::default();

    let mut rig = Rig::new(0, environment, &events);
    let mut muted = false;

    let dt = 1.0 / SIM_HZ;
    let step = Duration::from_secs_f64(dt);
    let publish_every = Duration::from_secs_f64(1.0 / PUBLISH_HZ);

    let started = Instant::now();
    let mut next_step = Instant::now();
    let mut next_publish = Instant::now();
    let mut measured_hz = SIM_HZ;
    let mut last_step_at = Instant::now();
    let mut frames: u64 = 0;

    loop {
        // --- commands ---------------------------------------------------
        let mut quit = false;
        loop {
            match commands.try_recv() {
                Ok(Command::Quit) => quit = true,
                Ok(Command::Throttle(delta)) => rig.driveline.nudge_throttle(delta),
                Ok(Command::ToggleCut) => {
                    // A toggle, not a hold: terminals report key presses, not a
                    // key being held down, so tracking it as a hold would either
                    // stutter with the auto-repeat delay or never release.
                    rig.driveline.manual_cut = !rig.driveline.manual_cut;
                }
                Ok(Command::ToggleMute) => {
                    muted = !muted;
                    if let Some(audio) = rig.audio.as_mut() {
                        let _ = if muted { audio.pause() } else { audio.play() };
                    }
                }
                Ok(Command::SelectPreset(index)) => {
                    if index != rig.index {
                        // Dropped before the new one is built, so the old device
                        // is released before another is asked for — some
                        // backends will not open a second stream otherwise.
                        rig.audio = None;
                        rig = Rig::new(index, environment, &events);
                        if muted {
                            if let Some(audio) = rig.audio.as_mut() {
                                let _ = audio.pause();
                            }
                        }
                    }
                }
                Err(TryRecvError::Empty) => break,
                // The dashboard has gone: shut down rather than run headless.
                Err(TryRecvError::Disconnected) => quit = true,
            }
            if quit {
                break;
            }
        }
        if quit {
            break;
        }

        // --- one simulated frame ----------------------------------------
        let now = Instant::now();
        let elapsed = (now - last_step_at).as_secs_f64().min(MAX_CATCH_UP);
        last_step_at = now;
        // A smoothed measure of what the thread is really achieving, which is
        // the honest number to show — the nominal rate is only a request.
        if elapsed > 1e-9 {
            measured_hz += (1.0 / elapsed - measured_hz) * 0.02;
        }

        rig.driveline.update(&rig.block, dt);
        let output = rig.block.update(dt, rig.driveline.rpm);
        frames += 1;

        let controls = EngineControls {
            throttle: rig.driveline.throttle.clamp(0.0, 1.0),
            spark_cut: rig.driveline.cutting(),
            exhaust_cutout: false,
            anti_lag: false,
        };
        let snapshot = rig
            .source
            .sample(&rig.block, rig.driveline.rpm, dt, controls);

        // While muted the stream is paused, so its callback is not running and
        // the queue would only fill up and start reporting drops that mean
        // nothing. The synth picks up the current state on the first callback
        // after unmuting.
        if !muted {
            if let Some(audio) = rig.audio.as_mut() {
                audio.push(snapshot);
            }
        }

        // The torque curve is only meaningful once the phase ring holds a whole
        // cycle; before that the block is still integrating its way into the
        // first one and the work integral is over a partly empty ring.
        if rig.block.ring.is_primed() {
            rig.curve
                .record(rig.driveline.rpm, rig.driveline.torque.max(0.0));
        }

        // --- publish ----------------------------------------------------
        if now >= next_publish {
            next_publish = now + publish_every;
            if let Ok(mut shared) = telemetry.lock() {
                publish(&mut shared, &rig, &output, muted, realtime, measured_hz);
            }
        }

        // --- pace -------------------------------------------------------
        next_step += step;
        match next_step.checked_duration_since(Instant::now()) {
            Some(wait) => thread::sleep(wait),
            // Behind schedule: resynchronise rather than trying to make the time
            // back, which would only push the thread further behind.
            None => next_step = Instant::now(),
        }
    }

    // --- graceful shutdown ---------------------------------------------
    //
    // Pause before dropping so the device stops asking for samples while the
    // stream is still whole, then drop it, which closes it and releases the
    // callback's borrow of the queue.
    let dropped = rig
        .audio
        .as_ref()
        .map_or(0, |audio| audio.dropped_snapshots());
    let starvation = rig
        .audio
        .as_ref()
        .map_or(0.0, |audio| audio.stats().starvation_ratio());
    if let Some(audio) = rig.audio.as_mut() {
        let _ = audio.pause();
    }
    rig.audio = None;
    let _ = events.send(SimEvent::Stopped);

    let seconds = started.elapsed().as_secs_f64().max(1e-9);
    Ok(format!(
        "engine stopped — {} at {:.0} rpm after {:.1} s\n\
         simulation {:.0} frames, {:.1} Hz average{}\n\
         audio: {:.2} % of callbacks ran without fresh physics, {dropped} snapshots dropped",
        rig.preset.name,
        rig.driveline.rpm,
        seconds,
        frames as f64,
        frames as f64 / seconds,
        if realtime {
            " (real-time priority)"
        } else {
            " (normal priority)"
        },
        starvation * 100.0,
    ))
}

/// Copies the rig's state into the shared telemetry.
///
/// Writes in place rather than replacing the struct: the `Vec`s keep their
/// capacity from frame to frame, so publishing sixty times a second allocates
/// nothing after the first call.
fn publish(
    shared: &mut Telemetry,
    rig: &Rig,
    output: &rust_engine_sim::physics::engine_block::BlockOutput,
    muted: bool,
    realtime: bool,
    measured_hz: f64,
) {
    let driveline = &rig.driveline;
    let block = &rig.block;

    if shared.preset_name != rig.preset.name {
        shared.preset_name.clear();
        shared.preset_name.push_str(rig.preset.name);
        shared.preset_note.clear();
        shared.preset_note.push_str(rig.preset.note);
        shared.preset_spec = rig.preset.spec();
    }
    shared.induction = rig.preset.induction.label();
    shared.preset = rig.index;
    shared.cylinders = block.firing.len();
    shared.banks = block.exhaust_banks.len();
    shared.redline = driveline.redline;

    shared.rpm = driveline.rpm;
    shared.throttle = driveline.throttle;
    shared.throttle_target = driveline.throttle_target;
    shared.manual_cut = driveline.manual_cut;
    shared.limiter = driveline.on_the_limiter();
    shared.muted = muted;

    shared.map_pa = block.intake.pressure();
    shared.ambient_pa = block.environment.pressure;
    let banks = block.exhaust_banks.len().max(1) as f64;
    shared.exhaust_pa = block
        .exhaust_banks
        .iter()
        .map(|bank| bank.port_pressure())
        .sum::<f64>()
        / banks;
    shared.exhaust_k = block
        .exhaust_banks
        .iter()
        .map(|bank| bank.plenum.temperature)
        .sum::<f64>()
        / banks;
    shared.manifold_modes.clear();
    shared
        .manifold_modes
        .extend_from_slice(&output.manifold_modes);

    // The mean over the cycle, not the instantaneous value: `output.brake_torque`
    // swings from strongly negative on compression to hugely positive just after
    // ignition, and a readout of it would be unreadable. This is the number a
    // dyno would print.
    shared.torque = driveline.torque;
    shared.power_kw = driveline.torque * driveline.rpm * std::f64::consts::PI / 30.0 / 1_000.0;
    shared.imep = output.imep;
    shared.bmep = output.bmep;
    shared.peak_pressure = output.peak_pressure;
    shared.mean_piston_speed = block.geometry().mean_piston_speed(driveline.rpm);
    shared.knock_integral = output.knock_integral;
    shared.knocking = output.knocking;

    shared.turbo_fitted = rig.source.has_turbo();
    shared.turbo_rpm = rig.source.turbo_shaft_rpm();
    shared.turbo_surge = rig.source.turbo_surge(&driveline.controls());

    // The whole cycle, every other cell. 360 points is finer than a braille
    // canvas can resolve in any terminal this will run in, and halves what the
    // lock is held for.
    shared.pv.clear();
    let cells = block.ring.cells();
    shared.pv.extend(
        cells
            .iter()
            .step_by(2)
            .map(|cell| (cell.volume * 1e6, cell.pressure / 1e5)),
    );
    debug_assert!(shared.pv.len() <= PHASE_CELLS);
    shared.curve.copy_from(&rig.curve);

    shared.substeps = output.step.plan.substeps;
    shared.dtheta_deg = output.step.plan.dtheta.to_degrees();
    shared.physics_hz = measured_hz;
    shared.ring_primed = block.ring.is_primed();
    shared.realtime = realtime;

    shared.audio = match rig.audio.as_ref() {
        Some(audio) => {
            let stats = audio.stats();
            AudioHealth {
                device: audio.info().device.clone(),
                sample_rate: audio.info().sample_rate,
                channels: audio.info().channels,
                playing: audio.is_playing(),
                open: true,
                starvation: stats.starvation_ratio(),
                peak: stats.peak(),
                dropped: audio.dropped_snapshots(),
                errors: stats.errors(),
                disconnected: stats.disconnected(),
            }
        }
        None => AudioHealth::default(),
    };
}

// ---------------------------------------------------------------------------
// Thread priority
// ---------------------------------------------------------------------------

/// Asks the scheduler to run this thread at a real-time priority.
///
/// Returns whether it was granted. The standard library has no priority API, so
/// this is a direct `pthread` call, and it is a request rather than a guarantee:
/// a container, a hardened kernel or an RLIMIT_RTPRIO of zero will refuse it and
/// the simulation carries on at normal priority. That degradation is real but
/// mild — the audio callback is scheduled by the audio system at a priority of
/// its own and is unaffected, so what is lost is only the freshness of the state
/// the callback reads, and the synth coasts through a late frame rather than
/// glitching.
///
/// The priority asked for is deliberately modest — a third of the way up the
/// real-time band — because the audio thread must continue to outrank this one.
/// A simulation that could preempt the callback it is feeding would be the one
/// arrangement worse than no elevation at all.
#[cfg(unix)]
fn request_realtime_priority() -> bool {
    // SAFETY: `sched_get_priority_{min,max}` and `pthread_setschedparam` are
    // called with a valid policy, this thread's own handle, and a fully
    // initialised `sched_param`. None of them can fail in a way that is
    // undefined; they report refusal through their return value, which is
    // checked. Nothing here touches memory the Rust side owns.
    unsafe {
        let policy = libc::SCHED_RR;
        let (min, max) = (
            libc::sched_get_priority_min(policy),
            libc::sched_get_priority_max(policy),
        );
        if min < 0 || max < min {
            return false;
        }

        let mut param: libc::sched_param = std::mem::zeroed();
        param.sched_priority = min + (max - min) / 3;
        libc::pthread_setschedparam(libc::pthread_self(), policy, &param) == 0
    }
}

/// Not implemented off unix; the simulation runs at normal priority.
#[cfg(not(unix))]
fn request_realtime_priority() -> bool {
    false
}
