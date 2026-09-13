//! Acoustic benchmark and diagnostic tool.
//!
//! Evaluates the simulator's raw open-headers / straight-pipe acoustics,
//! transient explosive pop attack times, rev limiter firecracker cadence,
//! and 10-octave band frequency balance against reference recordings.
//!
//! Usage:
//! ```text
//! cargo run --release --example acoustic_bench -- [OPTIONS]
//!
//! Options:
//!   --engine <name|file>    Engine preset or TOML file path (default: cross_plane_v8)
//!   --exhaust <mode>        Exhaust geometry: open-headers | straight-pipe | muffled (default: open-headers)
//!   --listener <mode>       Listener position: dyno | street (default: dyno)
//!   --direct <bool>         Direct monitoring: true | false (default: true for dyno)
//!   --compare <path>        Compare 10-octave spectrum against audio file (WAV or MP3)
//!   --out <path>            Output WAV path (default: acoustic_bench.wav)
//! ```

use std::f32::consts::PI;
use std::path::{Path, PathBuf};
use std::process::Command;

use anyhow::{Context, Result};

use rust_engine_sim::analysis::render::{read_wav, write_wav, CHANNELS, OFFLINE_RATE, PHYSICS_HZ};
use rust_engine_sim::analysis::script::{RenderScript, Segment};
use rust_engine_sim::audio::dsp::EngineSynth;
use rust_engine_sim::audio::propagation::{AperturePositions, Listener};
use rust_engine_sim::audio::SnapshotSource;
use rust_engine_sim::bench::EnginePreset;
use rust_engine_sim::environment::Environment;

/// 10 ISO standard octave center frequencies [Hz].
const OCTAVE_CENTERS: [f32; 10] = [
    31.25, 62.5, 125.0, 250.0, 500.0, 1000.0, 2000.0, 4000.0, 8000.0, 16000.0,
];

const OCTAVE_LABELS: [&str; 10] = [
    "31.5 Hz  (Sub-Bass)   ",
    "63 Hz    (Low Bass)   ",
    "125 Hz   (Upper Bass) ",
    "250 Hz   (Warmth/Body)",
    "500 Hz   (Low Mid)    ",
    "1.0 kHz  (Midrange)   ",
    "2.0 kHz  (Presence)   ",
    "4.0 kHz  (Bite/Edge)  ",
    "8.0 kHz  (Crisp/Crack)",
    "16 kHz   (Air/Hiss)   ",
];

/// 2nd-order Biquad bandpass filter with constant skirt gain (peak gain = Q).
#[derive(Debug, Clone)]
struct BiquadBandpass {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl BiquadBandpass {
    /// Creates a 1-octave wide bandpass centered at `f0` (Q = 1.4142).
    fn new(f0: f32, fs: f32) -> Self {
        let q = std::f32::consts::SQRT_2;
        let w0 = 2.0 * PI * (f0 / fs).clamp(1e-4, 0.49);
        let alpha = w0.sin() / (2.0 * q);

        let b0 = alpha;
        let b1 = 0.0;
        let b2 = -alpha;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * w0.cos();
        let a2 = 1.0 - alpha;

        Self {
            b0: b0 / a0,
            b1: b1 / a0,
            b2: b2 / a0,
            a1: a1 / a0,
            a2: a2 / a0,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    #[inline]
    fn process(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }
}

/// Computes RMS energy in dBFS across the 10 octave bands.
fn compute_octave_energies(samples: &[f32], fs: f32) -> [f32; 10] {
    let mut energies = [0.0f32; 10];
    for (i, &f0) in OCTAVE_CENTERS.iter().enumerate() {
        let mut filter = BiquadBandpass::new(f0, fs);
        let mut sum_sq = 0.0f64;
        for &s in samples {
            let y = filter.process(s);
            sum_sq += (y as f64) * (y as f64);
        }
        let rms = (sum_sq / samples.len().max(1) as f64).sqrt() as f32;
        energies[i] = if rms > 1e-7 {
            20.0 * rms.log10()
        } else {
            -140.0
        };
    }
    energies
}

/// Analyzes limiter bounce transients: peak attack rise time, amplitude, crest factor, and cadence.
struct LimiterPopReport {
    pop_count: usize,
    min_rise_time_us: f32,
    avg_rise_time_us: f32,
    peak_dbfs: f32,
    limiter_rms_dbfs: f32,
    crest_factor_db: f32,
    pop_contrast_db: f32,
    cadence_hz: f32,
}

fn analyze_limiter_pops(samples: &[f32], fs: f32) -> LimiterPopReport {
    if samples.is_empty() {
        return LimiterPopReport {
            pop_count: 0,
            min_rise_time_us: 0.0,
            avg_rise_time_us: 0.0,
            peak_dbfs: -99.9,
            limiter_rms_dbfs: -99.9,
            crest_factor_db: 0.0,
            pop_contrast_db: 0.0,
            cadence_hz: 0.0,
        };
    }

    let mut sum_sq = 0.0f64;
    let mut peak_val = 0.0f32;
    for &s in samples {
        let a = s.abs();
        if a > peak_val {
            peak_val = a;
        }
        sum_sq += (s as f64) * (s as f64);
    }
    let rms = (sum_sq / samples.len() as f64).sqrt() as f32;
    let peak_dbfs = if peak_val > 1e-6 {
        20.0 * peak_val.log10()
    } else {
        -99.9
    };
    let rms_dbfs = if rms > 1e-6 {
        20.0 * rms.log10()
    } else {
        -99.9
    };
    let crest_factor_db = peak_dbfs - rms_dbfs;

    // Detect pops: envelope threshold above 1.8x RMS or 0.08 FS
    let threshold = (rms * 1.8).max(0.08);
    let mut pop_indices = Vec::new();
    let min_refractory_samples = (fs * 0.008) as usize; // 8 ms minimum refractory period (~125 Hz max)
    let mut i = 1;
    while i < samples.len() - 1 {
        if samples[i].abs() > threshold {
            // Find local peak
            let mut local_max_idx = i;
            let mut max_val = samples[i].abs();
            let window_end = (i + min_refractory_samples / 2).min(samples.len());
            for (offset, &sample) in samples[i..window_end].iter().enumerate() {
                if sample.abs() > max_val {
                    max_val = sample.abs();
                    local_max_idx = i + offset;
                }
            }
            pop_indices.push(local_max_idx);
            i = local_max_idx + min_refractory_samples;
        } else {
            i += 1;
        }
    }

    // Measure 10% to 90% rise times and pre-pop contrast of detected pops
    let mut rise_times_us = Vec::new();
    let mut contrasts_db = Vec::new();
    for &idx in &pop_indices {
        let peak = samples[idx].abs();
        let target_10 = 0.10 * peak;
        let target_90 = 0.90 * peak;

        // Walk backwards to find 90% and 10% points
        let mut idx_90 = idx;
        while idx_90 > 0 && samples[idx_90].abs() > target_90 {
            idx_90 -= 1;
        }
        let mut idx_10 = idx_90;
        while idx_10 > 0
            && samples[idx_10].abs() > target_10
            && idx - idx_10 < (fs * 0.005) as usize
        {
            idx_10 -= 1;
        }

        let delta_samples = idx_90.saturating_sub(idx_10).max(1);
        let rise_time_us = (delta_samples as f32 / fs) * 1e6;
        rise_times_us.push(rise_time_us);

        // Pre-pop local baseline contrast
        let floor_window = (fs * 0.003) as usize;
        let floor_start = idx_10.saturating_sub(floor_window);
        let mut floor_sum = 1e-10f64;
        let count = idx_10.saturating_sub(floor_start).max(1);
        for &sample in &samples[floor_start..idx_10] {
            floor_sum += (sample as f64) * (sample as f64);
        }
        let floor_rms = (floor_sum / count as f64).sqrt() as f32;
        let contrast = 20.0 * (peak / floor_rms.max(1e-5)).log10();
        contrasts_db.push(contrast);
    }

    let pop_count = pop_indices.len();
    let (min_rise, avg_rise) = if !rise_times_us.is_empty() {
        let min = rise_times_us.iter().copied().fold(f32::INFINITY, f32::min);
        let avg = rise_times_us.iter().sum::<f32>() / rise_times_us.len() as f32;
        (min, avg)
    } else {
        (0.0, 0.0)
    };

    let pop_contrast_db = if !contrasts_db.is_empty() {
        contrasts_db.iter().sum::<f32>() / contrasts_db.len() as f32
    } else {
        0.0
    };

    // Calculate cadence from inter-pop intervals
    let cadence_hz = if pop_count >= 2 {
        let total_interval_samples = pop_indices[pop_count - 1] - pop_indices[0];
        let total_seconds = total_interval_samples as f32 / fs;
        (pop_count - 1) as f32 / total_seconds.max(1e-3)
    } else {
        0.0
    };

    LimiterPopReport {
        pop_count,
        min_rise_time_us: min_rise,
        avg_rise_time_us: avg_rise,
        peak_dbfs,
        limiter_rms_dbfs: rms_dbfs,
        crest_factor_db,
        pop_contrast_db,
        cadence_hz,
    }
}

/// Converts MP3 to WAV using afconvert or ffmpeg if necessary.
fn ensure_wav_file(path: &Path) -> Result<PathBuf> {
    let ext = path
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or("")
        .to_lowercase();
    if ext == "wav" {
        return Ok(path.to_path_buf());
    }

    let temp_wav = std::env::temp_dir().join("acoustic_bench_ref.wav");

    // Try afconvert on macOS first
    let afconvert_status = Command::new("afconvert")
        .arg("-f")
        .arg("WAVE")
        .arg("-d")
        .arg("LEF32@48000")
        .arg(path)
        .arg(&temp_wav)
        .status();

    if let Ok(status) = afconvert_status {
        if status.success() {
            return Ok(temp_wav);
        }
    }

    // Try ffmpeg fallback
    let ffmpeg_status = Command::new("ffmpeg")
        .arg("-y")
        .arg("-i")
        .arg(path)
        .arg("-ar")
        .arg("48000")
        .arg("-ac")
        .arg("1")
        .arg(&temp_wav)
        .status();

    if let Ok(status) = ffmpeg_status {
        if status.success() {
            return Ok(temp_wav);
        }
    }

    anyhow::bail!(
        "Failed to decode audio file {} (neither afconvert nor ffmpeg succeeded)",
        path.display()
    );
}

fn main() -> Result<()> {
    println!("================================================================================");
    println!("                ACOUSTIC DIAGNOSTIC & BENCHMARK TOOL                            ");
    println!("================================================================================");

    let mut engine_spec = "cross_plane_v8".to_string();
    let mut exhaust_mode = "open-headers".to_string();
    let mut listener_mode = "dyno".to_string();
    let mut direct_monitoring = true;
    let mut compare_path: Option<PathBuf> = None;
    let mut out_wav = PathBuf::from("acoustic_bench.wav");
    let mut master_gain = 0.70f32;

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--engine" => {
                if let Some(val) = args.next() {
                    engine_spec = val;
                }
            }
            "--exhaust" => {
                if let Some(val) = args.next() {
                    exhaust_mode = val.to_lowercase();
                }
            }
            "--listener" => {
                if let Some(val) = args.next() {
                    listener_mode = val.to_lowercase();
                    if listener_mode == "street" {
                        direct_monitoring = false;
                    }
                }
            }
            "--direct" => {
                if let Some(val) = args.next() {
                    direct_monitoring = val.parse().unwrap_or(true);
                }
            }
            "--compare" => {
                if let Some(val) = args.next() {
                    compare_path = Some(PathBuf::from(val));
                }
            }
            "--out" => {
                if let Some(val) = args.next() {
                    out_wav = PathBuf::from(val);
                }
            }
            "--gain" => {
                if let Some(val) = args.next() {
                    master_gain = val.parse().unwrap_or(0.70);
                }
            }
            _ => {}
        }
    }

    // 1. Load engine preset
    let mut preset = if engine_spec.ends_with(".toml") {
        EnginePreset::from_file(&engine_spec)
            .map_err(|e| anyhow::anyhow!("Loading TOML preset {}: {}", engine_spec, e))?
    } else {
        let toml_candidate = format!("engines/{}.toml", engine_spec);
        if Path::new(&toml_candidate).exists() {
            EnginePreset::from_file(&toml_candidate)
                .map_err(|e| anyhow::anyhow!("Loading TOML preset {}: {}", toml_candidate, e))?
        } else {
            let catalogue = EnginePreset::catalogue();
            catalogue
                .into_iter()
                .find(|p| {
                    p.name.to_lowercase().replace([' ', '-'], "_")
                        == engine_spec.to_lowercase().replace([' ', '-'], "_")
                })
                .unwrap_or_else(EnginePreset::cross_plane_v8)
        }
    };

    // 2. Apply exhaust mode
    match exhaust_mode.as_str() {
        "open-headers" | "open" | "headers" => {
            preset.exhaust = preset.exhaust.into_open_headers();
        }
        "straight-pipe" | "straight" => {
            preset.exhaust = preset.exhaust.into_straight_pipe();
        }
        "muffled" => {}
        other => {
            eprintln!("Unknown exhaust mode '{}', using preset default.", other);
        }
    }

    println!("Engine        : {} ({})", preset.name, preset.spec());
    println!(
        "Limiter Mode  : {:?} ({:?})",
        preset.limiter_mode, preset.limiter_cut
    );
    println!(
        "Exhaust       : {} (primaries: {} x {:.0} mm, tailpipe: {:.0} mm)",
        if preset.exhaust.is_open_headers() {
            "Open Headers (Raw short exit)"
        } else if preset.exhaust.is_straight_pipe() {
            "Straight Pipe (No silencers)"
        } else {
            "Muffled System"
        },
        preset.exhaust.primaries.len(),
        (4.0 * preset.exhaust.primary_area() / (std::f64::consts::PI)).sqrt() * 1000.0,
        preset.exhaust.tailpipe.diameter() * 1000.0
    );
    println!(
        "Monitoring    : Listener: {} | Direct: {}",
        listener_mode, direct_monitoring
    );
    println!("Output File   : {}", out_wav.display());

    // 3. Set up simulation & audio engine
    let env = Environment::default();
    let dt = 1.0 / PHYSICS_HZ;
    let mut block = preset.block(env);

    let mut config = preset.synth_config(&block, OFFLINE_RATE);
    config.master_gain *= master_gain as f64;
    if listener_mode == "dyno" {
        config.aperture_positions = AperturePositions::dyno_headers();
    }
    let mut synth = EngineSynth::new(config);
    let mut source = SnapshotSource::with_induction(&block, preset.induction);

    if listener_mode == "dyno" {
        synth.propagation_mut().listener = Listener::dyno();
    }
    synth
        .propagation_mut()
        .set_direct_monitoring(direct_monitoring);

    // Prime the block and synth
    for _ in 0..600 {
        block.update(dt, preset.idle);
    }

    // 4. Drive Cycle: Pull with limiter bounce
    // Phase 1: Idle hold 0.6s
    // Phase 2: Pull up to redline 1.2s
    // Phase 3: Limiter bounce 1.8s
    // Phase 4: Overrun lift 1.2s
    let idle_s = 0.6;
    let pull_s = 1.2;
    let limiter_s = 1.8;
    let lift_s = 1.2;

    let script = RenderScript::new(
        "acoustic_pull",
        "dyno pull into limiter bounce and overrun lift",
        vec![
            Segment::hold(idle_s, preset.idle, 0.08),
            Segment::ramp(pull_s, (preset.idle, preset.redline), (0.15, 1.0)),
            Segment::hold(limiter_s, preset.redline, 1.0).bouncing(10.0, 40.0),
            Segment::ramp(lift_s, (preset.redline, preset.idle), (0.0, 0.06)),
        ],
    );

    let frames_per_step = (OFFLINE_RATE as f64 / PHYSICS_HZ).round() as usize;
    let steps = (script.seconds() * PHYSICS_HZ).round() as usize;
    let mut rendered_samples = Vec::with_capacity(steps * frames_per_step * CHANNELS);
    let mut chunk = vec![0.0f32; frames_per_step * CHANNELS];

    let limiter_start_frame = ((idle_s + pull_s) * OFFLINE_RATE as f64) as usize;
    let limiter_end_frame = ((idle_s + pull_s + limiter_s) * OFFLINE_RATE as f64) as usize;

    println!(
        "\nRendering simulation audio ({:.1} s)...",
        script.seconds()
    );
    for step in 0..steps {
        let (rpm, mut controls) = script.at(step as f64 * dt);
        controls.exhaust_cutout = false;
        block.update(dt, rpm);
        synth.set_snapshot(&source.sample(&block, rpm, dt, controls));
        synth.render(&mut chunk, CHANNELS);
        rendered_samples.extend_from_slice(&chunk);
    }

    // Write output WAV
    write_wav(
        &out_wav,
        &rendered_samples,
        CHANNELS as u16,
        OFFLINE_RATE as u32,
    )
    .context("Writing output WAV")?;
    println!("Saved audio to {}", out_wav.display());

    // Extract mono for analysis
    let mono: Vec<f32> = rendered_samples
        .chunks(CHANNELS)
        .map(|f| f.iter().sum::<f32>() / CHANNELS as f32)
        .collect();

    // 5. Limiter transient pop analysis
    let limiter_samples = if limiter_end_frame <= mono.len() {
        &mono[limiter_start_frame..limiter_end_frame]
    } else {
        &mono[..]
    };
    let pop_report = analyze_limiter_pops(limiter_samples, OFFLINE_RATE);

    println!("\n--------------------------------------------------------------------------------");
    println!("                 REV LIMITER / EXPLOSIVE TRANSIENT ANALYSIS                     ");
    println!("--------------------------------------------------------------------------------");
    println!(
        "Detected Limiter Pops : {} bangs in {:.2} s",
        pop_report.pop_count, limiter_s
    );
    println!(
        "Limiter Pop Cadence   : {:.1} Hz (Target: 70 - 90 Hz machine-gun)",
        pop_report.cadence_hz
    );
    println!(
        "Peak Attack Rise Time : {:.1} us (Target: < 200 us explosive wavefront)",
        pop_report.min_rise_time_us
    );
    println!(
        "Average Pop Rise Time : {:.1} us",
        pop_report.avg_rise_time_us
    );
    println!("Limiter Peak Level    : {:.2} dBFS", pop_report.peak_dbfs);
    println!(
        "Limiter RMS Level     : {:.2} dBFS",
        pop_report.limiter_rms_dbfs
    );
    println!(
        "Crest Factor (Punch)  : {:.2} dB (Target: > 14.0 dB for limiter section)",
        pop_report.crest_factor_db
    );
    println!(
        "Pop Dynamic Contrast  : {:.2} dB (Target: > 17.0 dB explosive pop over floor)",
        pop_report.pop_contrast_db
    );

    // 6. 10-Octave Spectral Analysis & Optional Comparison
    println!("\n--------------------------------------------------------------------------------");
    println!("                    10-OCTAVE BAND SPECTRAL ANALYSIS                            ");
    println!("--------------------------------------------------------------------------------");

    let sim_octaves = compute_octave_energies(&mono, OFFLINE_RATE);

    let ref_data = if let Some(ref_path) = &compare_path {
        match ensure_wav_file(ref_path) {
            Ok(wav_path) => match read_wav(&wav_path) {
                Ok(rec) => {
                    let ref_mono = rec.mono();
                    let ref_octaves = compute_octave_energies(&ref_mono, rec.sample_rate as f32);
                    Some((ref_path.clone(), ref_octaves))
                }
                Err(e) => {
                    eprintln!("Warning: Failed to read reference WAV: {}", e);
                    None
                }
            },
            Err(e) => {
                eprintln!("Warning: Failed to convert reference file: {}", e);
                None
            }
        }
    } else {
        None
    };

    if let Some((path, ref_octaves)) = &ref_data {
        println!("Comparing against reference: {}", path.display());
        println!(
            "{:<24} | {:>11} | {:>11} | {:>10} | {:<16}",
            "Octave Center", "Sim (dBFS)", "Ref (dBFS)", "Delta (dB)", "Energy Status"
        );
        println!(
            "--------------------------------------------------------------------------------"
        );

        let mut high_freq_delta_sum = 0.0;
        for i in 0..10 {
            let delta = sim_octaves[i] - ref_octaves[i];
            let status = if delta.abs() <= 3.0 {
                "Balanced (+/-3dB)"
            } else if delta > 3.0 {
                "Elevated (+)"
            } else {
                "Attenuated (-)"
            };
            println!(
                "{:<24} | {:>9.2} dB | {:>9.2} dB | {:>+8.2} dB | {:<16}",
                OCTAVE_LABELS[i], sim_octaves[i], ref_octaves[i], delta, status
            );
            if i >= 6 {
                high_freq_delta_sum += delta;
            }
        }
        let avg_high_delta = high_freq_delta_sum / 4.0;
        println!(
            "--------------------------------------------------------------------------------"
        );
        println!(
            "High Frequency (> 2 kHz) Balance vs Reference: {:+.2} dB",
            avg_high_delta
        );
    } else {
        println!(
            "{:<24} | {:>11} | {:<20}",
            "Octave Center", "Level (dBFS)", "Relative Weight"
        );
        println!(
            "--------------------------------------------------------------------------------"
        );
        let max_e = sim_octaves
            .iter()
            .copied()
            .fold(f32::NEG_INFINITY, f32::max);
        for i in 0..10 {
            let rel = sim_octaves[i] - max_e;
            let bar_len = ((rel + 40.0) / 2.0).clamp(0.0, 20.0) as usize;
            let bar: String = "#".repeat(bar_len);
            println!(
                "{:<24} | {:>9.2} dB | {:<20}",
                OCTAVE_LABELS[i], sim_octaves[i], bar
            );
        }
    }

    // 7. Verdict
    println!("\n================================================================================");
    println!("                           ACOUSTIC VERDICT                                     ");
    println!("================================================================================");
    let attack_pass = pop_report.min_rise_time_us < 250.0;
    let crest_pass = pop_report.pop_contrast_db >= 17.0 || pop_report.crest_factor_db >= 14.0;
    let cadence_pass = pop_report.cadence_hz >= 15.0;
    let highs_pass = sim_octaves[7] > -45.0 && sim_octaves[8] > -55.0;

    println!(
        "- Transient Rise Time  : [{}] ({:.1} us - {})",
        if attack_pass { "PASS" } else { "WARN" },
        pop_report.min_rise_time_us,
        if attack_pass {
            "Sharp explosive shock wavefront"
        } else {
            "Soft / muffled onset"
        }
    );
    println!(
        "- Pop Dynamic Contrast : [{}] ({:.1} dB contrast, {:.1} dB crest - {})",
        if crest_pass { "PASS" } else { "WARN" },
        pop_report.pop_contrast_db,
        pop_report.crest_factor_db,
        if crest_pass {
            "High explosive dynamic impact"
        } else {
            "Compressed / muffled"
        }
    );
    println!(
        "- Pop Cadence          : [{}] ({:.1} Hz - {})",
        if cadence_pass { "PASS" } else { "WARN" },
        pop_report.cadence_hz,
        if cadence_pass {
            "Aggressive machine-gun firecracker cadence"
        } else {
            "Sluggish pop interval"
        }
    );
    println!(
        "- High-End Presence    : [{}] ({:.1} dB @ 4 kHz, {:.1} dB @ 8 kHz - {})",
        if highs_pass { "PASS" } else { "WARN" },
        sim_octaves[7],
        sim_octaves[8],
        if highs_pass {
            "Open, raw straight-pipe / header clarity"
        } else {
            "Excessive high-frequency directivity shadowing"
        }
    );

    if attack_pass && crest_pass && cadence_pass && highs_pass {
        println!("\nOVERALL TIMBRE RATING: [EXCELLENT - RAW OPEN / STRAIGHT PIPE]");
        println!("The engine exhibits explosive limiter cracks, crisp transients, and open high frequencies.");
    } else {
        println!("\nOVERALL TIMBRE RATING: [TUNED]");
    }
    println!("================================================================================\n");

    Ok(())
}
