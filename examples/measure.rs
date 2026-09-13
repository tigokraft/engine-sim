//! The measurement harness: every engine in the catalogue, through every fixed
//! profile, reported as numbers.
//!
//! ```text
//! cargo run --release --example measure
//! cargo run --release --example measure -- --out target/measurements
//! cargo run --release --example measure -- --preset V12 --no-wav
//! cargo run --release --example measure -- --markdown docs/measurements
//! ```
//!
//! Three things come out of a run, and each answers a question a later stage
//! will ask:
//!
//! - **An order table.** The level of every half-order from 0.5 to 24, in dBFS,
//!   for every profile. "The V12 sits an octave above the four" is a claim
//!   about two rows of this table, and either it holds or it does not.
//! - **A resonance list.** The peaks in the long-term average spectrum of the
//!   two sweeping profiles, where a pipe mode stands still while the orders
//!   sweep past it. Stages 3 to 6 replace the plumbing, and these are the
//!   frequencies they have to move.
//! - **A CPU figure.** How many seconds of audio come out per second of wall
//!   clock, and how much of one core the synth alone would need to keep up.
//!   Recorded now, before Stage 5's seventeen delay lines and Stage 10d's
//!   oversampling, so every later stage can be charged for what it costs.
//!
//! The audio itself is written out as one 32-bit float WAV per preset per
//! profile, which is what a person listens to when the table says something
//! surprising.
//!
//! Everything but the CPU figure is deterministic: two runs of one build
//! produce identical WAVs and identical tables. The timings are the one thing
//! that is a property of the machine rather than of the build, and they are
//! labelled as such wherever they are recorded.

use std::env;
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};

use rust_engine_sim::analysis::orders::{self, half_orders, OrderTable, Peak};
use rust_engine_sim::analysis::render::{
    Continuity, Render, RenderCost, RenderPlan, OFFLINE_RATE, PHYSICS_HZ,
};
use rust_engine_sim::analysis::script;
use rust_engine_sim::bench::EnginePreset;

/// Resonance peaks reported per sweeping render.
const PEAKS: usize = 12;

/// Speed ratio a profile has to cover before its peaks mean anything.
const SWEPT_RATIO: f64 = 2.0;

/// Share of a render that has to be spent sweeping before its peaks mean
/// anything.
///
/// A resonance is what stands still while the orders move past it, so peak
/// picking only says something about the plumbing on a render the engine spent
/// moving. Hold one speed for a quarter of a render and its order lines pile up
/// in the average exactly as a resonance would, and the table fills with firing
/// frequencies. Of the six profiles only the two sweeps clear this, which is
/// what they are for.
const SWEPT_FRACTION: f64 = 0.75;

/// Column width of one profile in the printed order table.
///
/// One wider than the longest profile name, so the columns never touch.
const COLUMN: usize = 15;

// ---------------------------------------------------------------------------
// Measuring
// ---------------------------------------------------------------------------

/// One preset through one profile.
struct Run {
    /// Profile name, which is also the WAV's.
    script: &'static str,
    /// What the profile is there to expose.
    note: &'static str,
    /// Length of the render [s].
    seconds: f64,
    /// The slowest and fastest the engine turned [rev/min].
    speed: (f64, f64),
    /// Level of every order.
    orders: OrderTable,
    /// Peaks in the long-term average spectrum; empty unless the profile swept.
    peaks: Vec<Peak>,
    /// Dropouts, clipping and discontinuities.
    continuity: Continuity,
    /// What the render cost in wall clock.
    cost: RenderCost,
}

/// Everything measured about one engine.
struct Measured {
    preset: EnginePreset,
    runs: Vec<Run>,
}

/// Renders one preset through every fixed profile and measures each render.
fn measure(preset: EnginePreset, scale: f64, wav_dir: Option<&Path>) -> Result<Measured> {
    let wanted = half_orders();
    let mut runs = Vec::new();

    for script in script::catalogue(&preset) {
        let full = script.seconds();
        let script = script.scaled_to(full * scale);
        let render = RenderPlan::new(&preset, &script).render();
        let mono = render.mono();

        if let Some(dir) = wav_dir {
            let path = dir.join(format!("{}-{}.wav", slug(preset.name), script.name));
            render
                .write_wav(&path)
                .with_context(|| format!("writing {}", path.display()))?;
        }

        let speed = render.rpm.range(0.0, render.seconds());
        let sweeping = sweeping_span(&render).filter(|&(first, last)| {
            let frames = mono.len().max(1);
            let (slow, fast) = render.rpm.range(
                first as f64 / render.sample_rate,
                last as f64 / render.sample_rate,
            );
            (last - first) as f64 >= SWEPT_FRACTION * frames as f64
                && slow > 0.0
                && fast / slow > SWEPT_RATIO
        });

        runs.push(Run {
            script: script.name,
            note: script.note,
            seconds: render.seconds(),
            speed,
            orders: orders::track(&mono, render.sample_rate, &render.rpm, &wanted),
            peaks: match sweeping {
                Some((first, last)) => {
                    orders::resonances(&mono[first..last], render.sample_rate, PEAKS)
                }
                None => Vec::new(),
            },
            continuity: render.continuity(),
            cost: render.cost,
        });
    }

    Ok(Measured { preset, runs })
}

/// The longest stretch of a render during which the speed only moved one way.
///
/// Returned as frame indices into the mono buffer. A held speed contributes its
/// own order lines to a long-term average, all at one frequency, which is
/// exactly what a resonance is supposed to look like — so the average is taken
/// over the part of the render the engine spent moving and over nothing else.
fn sweeping_span(render: &Render) -> Option<(usize, usize)> {
    let h = 1.0 / PHYSICS_HZ;
    let steps = (render.seconds() / h).floor() as usize;
    if steps < 2 {
        return None;
    }

    // Below a rev a second the engine is being held, whatever the last decimal
    // place of the ramp says.
    let deadband = h;

    let (mut best, mut start, mut direction) = ((0usize, 0usize), 0usize, 0i32);
    for step in 0..steps {
        let delta = render.rpm.at((step + 1) as f64 * h) - render.rpm.at(step as f64 * h);
        let moving = if delta > deadband {
            1
        } else if delta < -deadband {
            -1
        } else {
            0
        };
        if moving != direction {
            direction = moving;
            start = step;
        }
        if moving != 0 && step + 1 - start > best.1 - best.0 {
            best = (start, step + 1);
        }
    }
    if best.1 == best.0 {
        return None;
    }

    let frames = render.samples.len() / render.channels.max(1);
    let frame_at = |step: usize| ((step as f64 * h) * render.sample_rate) as usize;
    Some((frame_at(best.0), frame_at(best.1).min(frames)))
}

impl Measured {
    /// The four-stroke firing order: one firing per cylinder every two turns.
    fn firing_order(&self) -> f64 {
        self.preset.firing.len() as f64 / 2.0
    }

    /// A one-line description of the engine.
    fn headline(&self) -> String {
        format!(
            "{}  ·  {}  ·  idle {:.0} rpm  ·  redline {:.0} rpm  ·  firing order {}",
            self.preset.spec(),
            self.preset.induction.label(),
            self.preset.idle,
            self.preset.redline,
            trim(self.firing_order()),
        )
    }

    /// The most prominent resonance found on a named profile.
    fn top_peak(&self, script: &str) -> Option<&Peak> {
        self.runs
            .iter()
            .find(|r| r.script == script)?
            .peaks
            .iter()
            .max_by(|a, b| a.prominence_db.total_cmp(&b.prominence_db))
    }

    /// Whether every profile rendered without a dropout or a discontinuity.
    fn is_clean(&self) -> bool {
        self.runs.iter().all(|r| r.continuity.is_clean())
    }
}

// ---------------------------------------------------------------------------
// Formatting
// ---------------------------------------------------------------------------

/// A filename-safe form of an engine's name: `"Cross-plane V8"` to
/// `"cross-plane-v8"`.
fn slug(name: &str) -> String {
    name.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() {
                c.to_ascii_lowercase()
            } else {
                '-'
            }
        })
        .collect()
}

/// `4` rather than `4.0`, but `4.5` still `4.5`.
fn trim(order: f64) -> String {
    if (order.fract()).abs() < 1e-9 {
        format!("{order:.0}")
    } else {
        format!("{order:.1}")
    }
}

/// One order's level, or a dash where the window could not resolve it.
fn cell(table: &OrderTable, order: f64) -> String {
    match table.level(order) {
        Some(level) if level.frames > 0 => format!("{:.1}", level.mean_db),
        _ => "—".into(),
    }
}

// ---------------------------------------------------------------------------
// The printed report
// ---------------------------------------------------------------------------

fn print_engine(measured: &Measured) {
    println!("\n== {} ==", measured.preset.name);
    println!("  {}", measured.headline());
    println!("  {}", measured.preset.note);

    println!(
        "\n  {:<COLUMN$}{:>8}{:>12}{:>9}{:>9}{:>11}{:>12}  continuity",
        "profile", "length", "rpm", "peak", "rms", "real time", "synth core"
    );
    for run in &measured.runs {
        println!(
            "  {:<COLUMN$}{:>7.1}s{:>12}{:>9.3}{:>9.4}{:>10.0}x{:>11.2}%  {}",
            run.script,
            run.seconds,
            format!("{:.0}-{:.0}", run.speed.0, run.speed.1),
            run.continuity.peak,
            run.continuity.rms,
            run.cost.realtime_multiple(),
            run.cost.synth_core_load() * 100.0,
            if run.continuity.is_clean() {
                "clean"
            } else {
                "FAILED"
            },
        );
    }

    println!("\n  order levels [dBFS]");
    print!("  {:>7}", "order");
    for run in &measured.runs {
        print!("{:>COLUMN$}", run.script);
    }
    println!();
    for order in half_orders() {
        let firing = (order - measured.firing_order()).abs() < 1e-9;
        // The marker goes in front so the numbers themselves stay in a column.
        print!(
            "  {:>7}",
            format!("{}{}", if firing { "*" } else { " " }, trim(order))
        );
        for run in &measured.runs {
            print!("{:>COLUMN$}", cell(&run.orders, order));
        }
        println!();
    }
    println!("  * the firing order");

    for run in &measured.runs {
        if run.peaks.is_empty() {
            continue;
        }
        println!("\n  resonances on {} [Hz, dBFS, prominence]", run.script);
        for peak in &run.peaks {
            println!(
                "    {:>9.1}{:>10.1}{:>10.1}",
                peak.hz, peak.db, peak.prominence_db
            );
        }
    }
}

fn print_cpu_summary(all: &[Measured]) {
    println!("\n== cpu ==");
    println!("  audio seconds rendered per second of wall clock, and the share of");
    println!("  one core the synth alone would need to keep up in real time");
    print!("\n  {:<18}", "engine");
    if let Some(first) = all.first() {
        for run in &first.runs {
            print!("{:>COLUMN$}", run.script);
        }
    }
    println!("{:>14}", "synth core");
    for measured in all {
        print!("  {:<18}", measured.preset.name);
        for run in &measured.runs {
            print!(
                "{:>COLUMN$}",
                format!("{:.0}x", run.cost.realtime_multiple())
            );
        }
        let worst = measured
            .runs
            .iter()
            .map(|r| r.cost.synth_core_load())
            .fold(0.0f64, f64::max);
        println!("{:>13.2}%", worst * 100.0);
    }
}

// ---------------------------------------------------------------------------
// The recorded baseline
// ---------------------------------------------------------------------------

/// Writes the whole run into `docs/measurements`-shaped markdown.
fn write_markdown(dir: &Path, all: &[Measured]) -> Result<()> {
    fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;

    for measured in all {
        let path = dir.join(format!("{}.md", slug(measured.preset.name)));
        fs::write(&path, engine_markdown(measured))
            .with_context(|| format!("writing {}", path.display()))?;
    }

    let path = dir.join("README.md");
    fs::write(&path, index_markdown(all)).with_context(|| format!("writing {}", path.display()))?;
    println!("\nwrote {} files to {}", all.len() + 1, dir.display());
    Ok(())
}

fn engine_markdown(measured: &Measured) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "# {}\n", measured.preset.name);
    let _ = writeln!(out, "> {}\n", measured.preset.note);
    let _ = writeln!(out, "{}\n", measured.headline());
    let _ = writeln!(
        out,
        "Recorded by `cargo run --release --example measure -- --markdown \
         docs/measurements`. See [the index](README.md) for what the numbers \
         mean and what they are for.\n"
    );

    let _ = writeln!(out, "## Renders\n");
    let _ = writeln!(
        out,
        "| Profile | Length | rpm | Peak | RMS | Max step | Continuity | What it exposes |"
    );
    let _ = writeln!(out, "|---|---:|---:|---:|---:|---:|---|---|");
    for run in &measured.runs {
        let _ = writeln!(
            out,
            "| `{}` | {:.1} s | {:.0}–{:.0} | {:.3} | {:.4} | {:.3} | {} | {} |",
            run.script,
            run.seconds,
            run.speed.0,
            run.speed.1,
            run.continuity.peak,
            run.continuity.rms,
            run.continuity.max_slew,
            if run.continuity.is_clean() {
                "clean"
            } else {
                "**failed**"
            },
            run.note,
        );
    }

    let _ = writeln!(out, "\n## Order levels [dBFS]\n");
    let _ = writeln!(
        out,
        "A dash is an order the analysis window cannot separate from DC at that \
         engine speed, not an order that was silent.\n"
    );
    let _ = write!(out, "| Order |");
    for run in &measured.runs {
        let _ = write!(out, " `{}` |", run.script);
    }
    let _ = writeln!(out);
    let _ = write!(out, "|---:|");
    for _ in &measured.runs {
        let _ = write!(out, "---:|");
    }
    let _ = writeln!(out);
    for order in half_orders() {
        let firing = (order - measured.firing_order()).abs() < 1e-9;
        let label = if firing {
            format!("**{}**", trim(order))
        } else {
            trim(order)
        };
        let _ = write!(out, "| {label} |");
        for run in &measured.runs {
            let _ = write!(out, " {} |", cell(&run.orders, order));
        }
        let _ = writeln!(out);
    }
    let _ = writeln!(
        out,
        "\nThe firing order, **{}**, is in bold.",
        trim(measured.firing_order())
    );

    let _ = writeln!(out, "\n## Resonances\n");
    let _ = writeln!(
        out,
        "Peaks in the long-term average spectrum of the part of a render the \
         engine spent sweeping: what stayed still while every order moved past \
         it. These are the frequencies Stages 3 to 6 have to move when they \
         replace the plumbing.\n"
    );
    for run in &measured.runs {
        if run.peaks.is_empty() {
            continue;
        }
        let _ = writeln!(out, "### `{}`\n", run.script);
        let _ = writeln!(out, "| Hz | dBFS | Prominence [dB] |");
        let _ = writeln!(out, "|---:|---:|---:|");
        for peak in &run.peaks {
            let _ = writeln!(
                out,
                "| {:.1} | {:.1} | {:.1} |",
                peak.hz, peak.db, peak.prominence_db
            );
        }
        let _ = writeln!(out);
    }
    out
}

fn index_markdown(all: &[Measured]) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "# Measurements\n");
    let _ = writeln!(
        out,
        "The Stage 0 baseline from [the implementation \
         plan](../IMPLEMENTATION_PLAN.md): what every engine in the catalogue \
         sounds like, as numbers, before any of the later stages touch the \
         physics or the synth. Every stage after this one is measured against \
         these tables.\n"
    );
    let _ = writeln!(out, "Regenerate the whole directory with:\n");
    let _ = writeln!(
        out,
        "```bash\ncargo run --release --example measure -- --markdown \
         docs/measurements\n```\n"
    );

    let _ = writeln!(
        out,
        "[Calibration](calibration.md) is the other half of this directory: \
         these tables say what the catalogue sounds like, and that one says how \
         far it is from what its own crank and its own plumbing predict.\n"
    );

    let _ = writeln!(out, "## What the numbers are\n");
    let _ = writeln!(
        out,
        "**Order levels.** An engine order is a frequency in cycles per crank \
         revolution, so order `n` sits at `f = n * rpm / 60` and follows the \
         engine up and down the rev range. A four-stroke fires `N_cyl / 2` \
         times a revolution, so an inline-four's firing order is 2, a V8's is 4 \
         and a V12's is 6. Levels are dBFS on an amplitude reference: a \
         full-scale sine is 0 dB. Each figure is the level averaged in power \
         over every analysis frame of that render.\n"
    );
    let _ = writeln!(
        out,
        "**Resonances.** Peaks in the long-term average spectrum, picked by \
         prominence and interpolated between bins. A resonance is defined by \
         standing still while the orders move past it, so only the two sweeping \
         profiles get a table and only the part of them the engine spent moving \
         is averaged: a held speed leaves its own order lines in the average, at \
         one frequency, looking exactly like plumbing.\n"
    );
    let _ = writeln!(
        out,
        "That is a filter, not a proof. A sweep this length still cannot smear \
         the lowest orders across more than an analysis bin, so treat a peak as \
         a resonance only if `sweep_up` and `sweep_down` both put one at the \
         same frequency — they dwell at opposite ends of the rev range, so their \
         leftover order lines do not land in the same place.\n"
    );
    let _ = writeln!(
        out,
        "**Continuity.** Peak, RMS and the largest step between consecutive \
         samples — a pop is a step discontinuity — plus a check that no block of \
         256 frames came out silent.\n"
    );
    let _ = writeln!(
        out,
        "The analysis is an 8192-point Hann STFT at 48 kHz: a 5.86 Hz bin, and a \
         17.6 Hz floor under which an order cannot be told apart from DC. Orders \
         below it are recorded as a dash rather than as a number that would be \
         mostly leakage.\n"
    );

    let _ = writeln!(out, "## Engines\n");
    let _ = writeln!(
        out,
        "| Engine | Spec | Induction | Firing order | Top resonance | Clean |"
    );
    let _ = writeln!(out, "|---|---|---|---:|---:|---|");
    for measured in all {
        let _ = writeln!(
            out,
            "| [{}]({}.md) | {} | {} | {} | {} | {} |",
            measured.preset.name,
            slug(measured.preset.name),
            measured.preset.spec(),
            measured.preset.induction.label(),
            trim(measured.firing_order()),
            match measured.top_peak("sweep_up") {
                Some(peak) => format!("{:.0} Hz", peak.hz),
                None => "—".into(),
            },
            if measured.is_clean() { "yes" } else { "**no**" },
        );
    }

    let _ = writeln!(
        out,
        "\nEvery engine now breathes through its own plumbing, so this column no \
         longer reads the one muffler they all used to share and no longer lands \
         in a narrow band. It is the most prominent peak in the sweep average and \
         nothing more: a network of primaries, a collector and a silencer chain \
         has modes all the way up, and which of them stands tallest is a property \
         of that engine's pipes. Read the per-engine tables for the peaks in \
         order rather than this one number."
    );

    let _ = writeln!(out, "\n## CPU\n");
    let _ = writeln!(
        out,
        "Seconds of audio produced per second of wall clock, physics and synth \
         together, and the share of one core the synth alone would need to keep \
         up in real time. Appendix B of the plan holds that share under 5 % for \
         the largest preset.\n"
    );
    let _ = writeln!(
        out,
        "Unlike everything else here these figures are a property of the machine \
         that ran them, not of the build: compare them within one run, and \
         re-record them on the same machine when comparing across stages. This \
         set was taken on {} {}, release profile, at {:.0} kHz with the solver \
         at {:.0} Hz.\n",
        std::env::consts::ARCH,
        std::env::consts::OS,
        OFFLINE_RATE / 1e3,
        PHYSICS_HZ,
    );
    let _ = write!(out, "| Engine |");
    if let Some(first) = all.first() {
        for run in &first.runs {
            let _ = write!(out, " `{}` |", run.script);
        }
    }
    let _ = writeln!(out, " Synth core, worst |");
    let _ = write!(out, "|---|");
    if let Some(first) = all.first() {
        for _ in &first.runs {
            let _ = write!(out, "---:|");
        }
    }
    let _ = writeln!(out, "---:|");
    for measured in all {
        let _ = write!(out, "| {} |", measured.preset.name);
        for run in &measured.runs {
            let _ = write!(out, " {:.0}x |", run.cost.realtime_multiple());
        }
        let worst = measured
            .runs
            .iter()
            .map(|r| r.cost.synth_core_load())
            .fold(0.0f64, f64::max);
        let _ = writeln!(out, " {:.2} % |", worst * 100.0);
    }
    out
}

// ---------------------------------------------------------------------------

fn main() -> Result<()> {
    let mut out: Option<PathBuf> = Some(PathBuf::from("target/measurements"));
    let mut markdown: Option<PathBuf> = None;
    let mut only: Option<String> = None;
    let mut scale = 1.0f64;

    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--out" => out = args.next().map(PathBuf::from),
            "--no-wav" => out = None,
            "--markdown" => markdown = args.next().map(PathBuf::from),
            "--preset" => only = args.next(),
            "--scale" => scale = args.next().and_then(|v| v.parse().ok()).unwrap_or(scale),
            "--help" | "-h" => {
                println!(
                    "measure [--out DIR] [--no-wav] [--markdown DIR] \
                     [--preset NAME] [--scale F]"
                );
                return Ok(());
            }
            other => eprintln!("ignoring unknown argument: {other}"),
        }
    }
    let scale = scale.clamp(0.05, 10.0);

    let presets: Vec<EnginePreset> = EnginePreset::catalogue()
        .into_iter()
        .filter(|p| match &only {
            Some(name) => p.name.to_lowercase().contains(&name.to_lowercase()),
            None => true,
        })
        .collect();
    if presets.is_empty() {
        anyhow::bail!("no engine in the catalogue matches {only:?}");
    }

    if let Some(dir) = &out {
        fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
    }

    println!(
        "measuring {} engine{} through {} profiles at {}x length",
        presets.len(),
        if presets.len() == 1 { "" } else { "s" },
        script::catalogue(&presets[0]).len(),
        trim(scale),
    );
    if let Some(dir) = &out {
        println!("writing wavs to {}", dir.display());
    }

    let mut all = Vec::new();
    for preset in presets {
        let measured = measure(preset, scale, out.as_deref())?;
        print_engine(&measured);
        all.push(measured);
    }

    print_cpu_summary(&all);

    if let Some(dir) = &markdown {
        write_markdown(dir, &all)?;
    }

    let dirty: Vec<&str> = all
        .iter()
        .filter(|m| !m.is_clean())
        .map(|m| m.preset.name)
        .collect();
    if !dirty.is_empty() {
        anyhow::bail!("continuity checks failed on {}", dirty.join(", "));
    }
    Ok(())
}
