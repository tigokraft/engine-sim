//! Cross-plane V8 bench: 4.0 litre, 3000 rpm, wide open throttle.
//!
//! Runs the block until the trapped mass and the manifolds settle into a limit
//! cycle, then reports the converged cylinder pressure trace, the summed
//! indicated torque over a full 720 degrees, and the wall-clock cost of the
//! solver.
//!
//! ```text
//! cargo run --release --example v8_bench
//! ```

use std::time::Instant;

use anyhow::Result;

use rust_engine_sim::environment::Environment;
use rust_engine_sim::physics::cylinder::{deg, CYCLE_ANGLE};
use rust_engine_sim::physics::engine_block::{EngineBlock, ManifoldMode, PhaseRing, PHASE_CELLS};

/// Engine speed for the whole run [rpm].
const RPM: f64 = 3000.0;
/// Frame rate the block is driven at [Hz].
const FRAME_RATE: f64 = 600.0;
/// Cycles to run before sampling, so the limit cycle is converged.
const WARMUP_CYCLES: f64 = 40.0;

fn main() -> Result<()> {
    let env = Environment::default();
    let mut block = EngineBlock::cross_plane_v8(env);

    let frame_dt = 1.0 / FRAME_RATE;
    let cycle_seconds = 120.0 / RPM;
    let warmup_frames = (WARMUP_CYCLES * cycle_seconds / frame_dt).round() as usize;

    header(&block);

    let started = Instant::now();
    let mut output = block.update(frame_dt, RPM);
    for _ in 1..warmup_frames {
        output = block.update(frame_dt, RPM);
    }
    let elapsed = started.elapsed();

    summary(&block, &output, elapsed, warmup_frames);
    pressure_profile(&block);
    torque_profile(&block);
    firing_table(&block);
    rpm_sweep();

    Ok(())
}

/// Sweeps the speed range to show the manifold transit rule switching.
///
/// The runner is 0.75 m long, so `tau_pulse = 2L/c` is around 2.1 ms. The
/// 180-degree mean bank interval takes `30/rpm` seconds, so the tuning ratio
/// falls as `~14200/rpm` and sweeps down through the harmonics: each time it
/// passes close to a whole number the reflected wave lands back at the valve in
/// phase with the next pulse and the lumped plenum stops being a valid model.
fn rpm_sweep() {
    println!("\n== rpm sweep: manifold transit rule ==");
    println!(
        "  {:>6}  {:>9}  {:>8}  {:>6}  {:>9}  {:>8}  {:>8}",
        "rpm", "torque", "power", "ratio", "mode", "BMEP", "P_peak"
    );

    let env = Environment::default();
    for step in 0..=11 {
        let rpm = 1500.0 + step as f64 * 500.0;
        let mut block = EngineBlock::cross_plane_v8(env);
        let frame_dt = 1.0 / FRAME_RATE;
        let frames = (20.0 * (120.0 / rpm) / frame_dt).round() as usize;

        let mut out = block.update(frame_dt, rpm);
        for _ in 1..frames {
            out = block.update(frame_dt, rpm);
        }

        let torque = block.mean_brake_torque(rpm);
        let omega = rpm * 2.0 * std::f64::consts::PI / 60.0;
        let bank = &block.exhaust_banks[0];
        println!(
            "  {:>6.0}  {:>7.1} Nm  {:>6.1} kW  {:>6.2}  {:>9}  {:>6.2} b  {:>6.1} b",
            rpm,
            torque,
            torque * omega / 1e3,
            bank.tuning_ratio,
            match bank.mode {
                ManifoldMode::Plenum => "plenum",
                ManifoldMode::Acoustic => "ACOUSTIC",
            },
            out.bmep / 1e5,
            out.peak_pressure / 1e5,
        );
    }
    println!(
        "  (ACOUSTIC rows are where tau_pulse divides the firing interval a whole\n   number of times, so the runner is switched to the 1D segmented model)"
    );
}

fn header(block: &EngineBlock) {
    let g = block.geometry();
    println!("== cross-plane V8 bench ==");
    println!(
        "  layout            {} cylinders, {} banks, firing {:?}",
        block.firing.len(),
        block.firing.bank_count(),
        block.firing.sequence
    );
    println!(
        "  bore x stroke     {:.1} x {:.1} mm  ({:.2} L total)",
        g.bore * 1e3,
        g.stroke * 1e3,
        block.total_displacement() * 1e3
    );
    println!("  compression       {:.1} :1", g.compression_ratio);
    println!(
        "  spark / burn      {:.0} deg BTDC / {:.0} deg duration",
        360.0 - block.model.wiebe.spark_angle.to_degrees(),
        block.model.wiebe.duration.to_degrees()
    );
    println!(
        "  valve timing      IVO {:.0} IVC {:.0} / EVO {:.0} EVC {:.0} (cycle deg)",
        block.model.valves.intake.open_angle.to_degrees(),
        block.model.valves.intake.close_angle().to_degrees(),
        block.model.valves.exhaust.open_angle.to_degrees(),
        block.model.valves.exhaust.close_angle().to_degrees(),
    );
    println!(
        "  speed             {:.0} rpm  ({:.1} m/s mean piston speed)",
        RPM,
        g.mean_piston_speed(RPM)
    );
}

fn summary(
    block: &EngineBlock,
    out: &rust_engine_sim::physics::engine_block::BlockOutput,
    elapsed: std::time::Duration,
    frames: usize,
) {
    let mean_brake = block.mean_brake_torque(RPM);
    let omega = RPM * 2.0 * std::f64::consts::PI / 60.0;

    println!("\n== converged cycle ==");
    println!("  peak cyl pressure {:>10.1} bar", out.peak_pressure / 1e5);
    println!("  IMEP (net)        {:>10.2} bar", out.imep / 1e5);
    println!("  BMEP              {:>10.2} bar", out.bmep / 1e5);
    println!(
        "  friction (FMEP)   {:>10.2} bar",
        block
            .friction
            .fmep(out.peak_pressure, block.geometry().mean_piston_speed(RPM))
            / 1e5
    );
    println!("  mean brake torque {:>10.1} N m", mean_brake);
    println!(
        "  brake power       {:>10.1} kW  ({:.0} hp)",
        mean_brake * omega / 1e3,
        mean_brake * omega / 745.7
    );
    println!(
        "  knock integral    {:>10.3}  {}",
        out.knock_integral,
        if out.knocking { "KNOCK" } else { "clear" }
    );

    println!("\n== manifolds ==");
    for (i, bank) in block.exhaust_banks.iter().enumerate() {
        let c = bank.speed_of_sound();
        println!(
            "  bank {i}: {:8}  tau_pulse {:>6.2} ms  ratio {:>5.2}  c {:.0} m/s  P_port {:.2} bar",
            match bank.mode {
                ManifoldMode::Plenum => "plenum",
                ManifoldMode::Acoustic => "acoustic",
            },
            bank.pipe.transit_time(c) * 1e3,
            bank.tuning_ratio,
            c,
            bank.port_pressure() / 1e5,
        );
    }
    println!(
        "  intake plenum     {:>10.3} bar",
        block.intake.pressure() / 1e5
    );

    println!("\n== solver ==");
    println!(
        "  substeps/frame    {:>10}  at {:.2} deg  ({} clamped)",
        out.step.plan.substeps,
        out.step.plan.dtheta.to_degrees(),
        if out.step.plan.clamped { "was" } else { "not" }
    );
    println!("  watchdog resets   {:>10}", block.solver.watchdog.resets);
    let per_frame = elapsed.as_secs_f64() / frames as f64;
    println!(
        "  wall clock        {:>10.2} ms for {frames} frames ({:.1} us/frame)",
        elapsed.as_secs_f64() * 1e3,
        per_frame * 1e6
    );
    println!(
        "  real-time budget  {:>10.1} x  (engine time / cpu time)",
        (frames as f64 / FRAME_RATE) / elapsed.as_secs_f64()
    );
}

/// Cylinder-1 pressure against crank angle, straight out of the phase ring.
fn pressure_profile(block: &EngineBlock) {
    println!("\n== cylinder 1 pressure trace ==");
    println!(
        "  {:>5}  {:>9}  {:>8}  {:>8}  {:>6}  log P",
        "deg", "P [bar]", "T [K]", "V [cc]", "x_b"
    );

    let peak = block.ring.peak_pressure().max(1.0);
    for i in (0..PHASE_CELLS).step_by(15) {
        let cell = block.ring.cell(i);
        let bar = cell.pressure / 1e5;
        println!(
            "  {:>5}  {:>9.2}  {:>8.0}  {:>8.2}  {:>6.3}  {}",
            i,
            bar,
            cell.temperature,
            cell.volume * 1e6,
            cell.burned_fraction,
            log_bar(cell.pressure, peak),
        );
    }
}

/// A log-scaled bar, so the 0.5 bar intake and the 60 bar peak both show.
fn log_bar(pressure: f64, peak: f64) -> String {
    const WIDTH: usize = 42;
    let lo = 0.3f64.ln();
    let hi = (peak / 1e5).max(1.0).ln();
    let v = (pressure / 1e5).max(0.3).ln();
    let n = (((v - lo) / (hi - lo)) * WIDTH as f64)
        .round()
        .clamp(0.0, WIDTH as f64) as usize;
    "#".repeat(n)
}

/// Summed indicated torque over a full cycle, reconstructed from the ring.
///
/// This is the phase-ring rule doing its job: eight cylinders read out of one
/// logged cycle at eight different offsets.
fn torque_profile(block: &EngineBlock) {
    println!("\n== summed indicated torque over 720 deg ==");

    let mut curve = vec![0.0f64; PHASE_CELLS];
    for (cell, value) in curve.iter_mut().enumerate() {
        let master = deg(cell as f64 + 0.5);
        *value = block
            .firing
            .cylinders
            .iter()
            .map(|c| {
                block
                    .ring
                    .cell(PhaseRing::phase_index(master - c.firing_offset) - 1)
                    .indicated_torque(block.crankcase_pressure)
            })
            .sum();
    }

    let lo = curve.iter().cloned().fold(f64::MAX, f64::min);
    let hi = curve.iter().cloned().fold(f64::MIN, f64::max);
    let mean = curve.iter().sum::<f64>() / curve.len() as f64;
    println!(
        "  min {:.0} N m   mean {:.0} N m   max {:.0} N m   (ripple {:.0} N m)",
        lo,
        mean,
        hi,
        hi - lo
    );
    println!("  {:>5}  {:>10}  profile", "deg", "tau [N m]");

    for i in (0..PHASE_CELLS).step_by(10) {
        println!(
            "  {:>5}  {:>10.0}  {}",
            i,
            curve[i],
            signed_bar(curve[i], lo, hi)
        );
    }

    // Mean torque from this curve must agree with the block's own figure.
    let from_curve = mean;
    let friction = block.friction.torque(
        block.ring.peak_pressure(),
        block.geometry().mean_piston_speed(RPM),
        block.total_displacement(),
    );
    println!(
        "\n  mean indicated {:.1} N m  -  friction {:.1} N m  =  brake {:.1} N m",
        from_curve,
        friction,
        from_curve - friction
    );
    println!(
        "  cross-check vs EngineBlock::mean_brake_torque: {:.1} N m",
        block.mean_brake_torque(RPM)
    );
}

/// A zero-centred bar so negative (pumping) torque reads as negative.
fn signed_bar(value: f64, lo: f64, hi: f64) -> String {
    const WIDTH: usize = 40;
    let span = (hi - lo).max(1e-9);
    let zero = ((0.0 - lo) / span * WIDTH as f64)
        .round()
        .clamp(0.0, WIDTH as f64) as usize;
    let pos = ((value - lo) / span * WIDTH as f64)
        .round()
        .clamp(0.0, WIDTH as f64) as usize;
    let mut row = vec![' '; WIDTH + 1];
    row[zero] = '|';
    let (a, b) = if pos >= zero {
        (zero, pos)
    } else {
        (pos, zero)
    };
    for c in row.iter_mut().take(b + 1).skip(a) {
        *c = if pos >= zero { '#' } else { '-' };
    }
    row[zero] = if pos == zero { '|' } else { row[zero] };
    row.into_iter().collect()
}

/// The bank-by-bank firing spacing that gives a cross-plane V8 its voice.
fn firing_table(block: &EngineBlock) {
    println!("\n== firing offsets ==");
    println!(
        "  {:>4}  {:>5}  {:>9}  {:>7}",
        "cyl", "bank", "offset", "cell"
    );
    for (i, c) in block.firing.cylinders.iter().enumerate() {
        println!(
            "  {:>4}  {:>5}  {:>7.0} deg  {:>7}",
            c.number,
            c.bank,
            c.firing_offset.to_degrees(),
            block.phase_index_of(i)
        );
    }
    for bank in 0..block.firing.bank_count() as u8 {
        let gaps: Vec<String> = block
            .firing
            .bank_gaps(bank)
            .iter()
            .map(|g| format!("{:.0}", g.to_degrees()))
            .collect();
        println!("  bank {bank} firing gaps: {} deg", gaps.join("-"));
    }
    println!(
        "  (a whole cycle is {:.0} deg; even overall spacing, uneven per bank\n   is what makes a cross-plane V8 burble)",
        CYCLE_ANGLE.to_degrees()
    );
}
