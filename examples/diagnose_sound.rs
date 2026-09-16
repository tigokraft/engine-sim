//! Acoustic diagnostics tool for the engine simulator.
//!
//! Investigates timbre, layer levels, spectral characteristics, and ignition
//! cut behaviour.
//!
//! ```text
//! cargo run --release --example diagnose_sound
//! cargo run --release --example diagnose_sound -- --preset inline-4
//! cargo run --release --example diagnose_sound -- --preset "flat-plane v8"
//! ```
//!
//! `--preset` matches by substring against the catalogue's own names, the way
//! `examples/measure.rs` does — case-insensitively, so `inline-4` and
//! `flat-plane v8` both find their engine — and exits loudly on no match
//! rather than silently rendering a different one.

use std::f32::consts::PI;
use std::path::Path;

use anyhow::Result;

use rust_engine_sim::analysis::render::{write_wav, PHYSICS_HZ};
use rust_engine_sim::audio::{EngineControls, EngineSynth, SnapshotSource, SynthConfig};
use rust_engine_sim::bench::EnginePreset;
use rust_engine_sim::environment::Environment;

const FS: f32 = 48_000.0;

fn compute_rms_peak(samples: &[f32]) -> (f32, f32, f32) {
    let mut sum_sq = 0.0f64;
    let mut peak = 0.0f32;
    for &s in samples {
        let a = s.abs();
        if a > peak {
            peak = a;
        }
        sum_sq += (s as f64) * (s as f64);
    }
    let rms = (sum_sq / samples.len().max(1) as f64).sqrt() as f32;
    let db = if rms > 1e-6 {
        20.0 * rms.log10()
    } else {
        -99.9
    };
    (rms, peak, db)
}

fn band_energy(samples: &[f32], fs: f32, f_low: f32, f_high: f32) -> f32 {
    let n = samples.len();
    let num_freqs = 32;
    let mut total_power = 0.0f64;
    for i in 0..num_freqs {
        let f = f_low * (f_high / f_low).powf(i as f32 / (num_freqs - 1) as f32);
        let mut re = 0.0f64;
        let mut im = 0.0f64;
        let omega = 2.0 * PI * f / fs;
        for (k, &s) in samples.iter().enumerate() {
            let phase = omega * (k as f32);
            re += (s as f64) * (phase.cos() as f64);
            im += (s as f64) * (phase.sin() as f64);
        }
        let mag_sq = (re * re + im * im) / (n as f64 * n as f64);
        total_power += mag_sq;
    }
    (total_power / num_freqs as f64).sqrt() as f32
}

fn main() -> Result<()> {
    println!("=== ENGINE SIMULATOR ACOUSTIC DIAGNOSTICS ===");

    let mut preset_name = "cross-plane v8".to_string();
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--preset" {
            if let Some(val) = args.next() {
                preset_name = val;
            }
        }
    }

    let preset = EnginePreset::find_by_name(&preset_name).unwrap_or_else(|| {
        eprintln!("no engine in the catalogue matches {preset_name:?}");
        std::process::exit(1);
    });

    println!("Target Engine: {}", preset.name);

    let env = Environment::default();
    let dt = 1.0 / PHYSICS_HZ;

    // Test 1: Layer isolation at steady 3000 RPM (loaded)
    println!("\n--- 1. LAYER ISOLATION AT 3000 RPM (LOADED) ---");
    let mut base_block = preset.block(env);
    for _ in 0..600 {
        base_block.update(dt, 3_000.0);
    }
    let mut source = SnapshotSource::new(&base_block);
    let snap = source.sample(
        &base_block,
        3_000.0,
        dt,
        EngineControls {
            throttle: 0.8,
            spark_cut: false,
            exhaust_cutout: false,
            anti_lag: false,
            starter_hz: 0.0,
        },
    );

    let default_cfg = SynthConfig::from_block(&base_block, FS);
    let layers = [
        ("Full Mix (Default)", 1.0, 1.0, 1.0, 1.0, 1.0, true),
        ("Exhaust Only", 1.0, 0.0, 0.0, 0.0, 0.0, true),
        ("Exhaust No Ground", 1.0, 0.0, 0.0, 0.0, 0.0, false),
        ("Intake Only", 0.0, 1.0, 0.0, 0.0, 0.0, true),
        ("Block Only", 0.0, 0.0, 1.0, 0.0, 0.0, true),
        ("Mechanical Only", 0.0, 0.0, 1.0, 1.0, 0.0, true),
    ];

    for &(name, exh_mul, int_mul, struct_mul, mech_mul, backfire_mul, ground) in &layers {
        let mut cfg = default_cfg.clone();
        cfg.exhaust_level *= exh_mul;
        cfg.intake_level *= int_mul;
        cfg.structure_level *= struct_mul;
        cfg.mechanical_level *= mech_mul;
        cfg.backfire_level *= backfire_mul;

        let mut synth = EngineSynth::new(cfg);
        synth
            .propagation_mut()
            .tailpipe_paths
            .iter_mut()
            .for_each(|p| {
                p.ground_reflection = ground;
            });
        synth.propagation_mut().intake_path.ground_reflection = ground;
        synth.propagation_mut().block_path.ground_reflection = ground;
        synth.set_snapshot(&snap);

        let frames = 48_000;
        let mut buf = vec![0.0f32; frames * 2];
        synth.render(&mut buf, 2);

        let mono: Vec<f32> = buf.chunks_exact(2).map(|c| 0.5 * (c[0] + c[1])).collect();
        // Skip first 100ms fade-in
        let steady = &mono[4_800..];
        let (rms, peak, db) = compute_rms_peak(steady);

        // Frequency breakdown
        let sub_bass = band_energy(steady, FS, 30.0, 80.0);
        let bass = band_energy(steady, FS, 80.0, 250.0);
        let low_mid = band_energy(steady, FS, 250.0, 800.0);
        let mid = band_energy(steady, FS, 800.0, 2500.0);
        let high = band_energy(steady, FS, 2500.0, 8000.0);

        println!(
            "{:<20} | RMS: {:7.4} ({:5.1} dBFS) | Peak: {:7.4} | Sub: {:6.4} | Bass: {:6.4} | LowMid: {:6.4} | Mid: {:6.4} | High: {:6.4}",
            name, rms, db, peak, sub_bass, bass, low_mid, mid, high
        );

        let filename = format!("/tmp/diag_{}.wav", name.to_lowercase().replace(' ', "_"));
        write_wav(Path::new(&filename), &buf, 2, FS as u32)?;
    }

    // Test 2: Ignition Cut Investigation
    println!("\n--- 2. IGNITION CUT BEHAVIOUR AT 6000 RPM ---");
    let mut cut_block = preset.block(env);
    for _ in 0..1200 {
        cut_block.update(dt, 6_000.0);
    }
    let mut cut_source = SnapshotSource::new(&cut_block);

    // Normal snapshot (fired)
    let normal_snap = cut_source.sample(
        &cut_block,
        6_000.0,
        dt,
        EngineControls {
            throttle: 1.0,
            spark_cut: false,
            exhaust_cutout: false,
            anti_lag: false,
            starter_hz: 0.0,
        },
    );

    // Cut snapshot (ignition cut)
    let cut_snap = cut_source.sample(
        &cut_block,
        6_000.0,
        dt,
        EngineControls {
            throttle: 1.0,
            spark_cut: true,
            exhaust_cutout: false,
            anti_lag: false,
            starter_hz: 0.0,
        },
    );

    println!("Snapshot comparison (Fired vs Spark Cut):");
    println!(
        "  Fired:     blowdown_delta[0]={:.1} Pa, peak_cyl_p={:.1} Pa, unburnt_fuel={:.2e} kg, spark_cut={}",
        normal_snap.blowdown_delta[0], normal_snap.peak_cylinder_pressure, normal_snap.unburnt_fuel_mass, normal_snap.spark_cut
    );
    println!(
        "  Spark Cut: blowdown_delta[0]={:.1} Pa, peak_cyl_p={:.1} Pa, unburnt_fuel={:.2e} kg, spark_cut={}",
        cut_snap.blowdown_delta[0], cut_snap.peak_cylinder_pressure, cut_snap.unburnt_fuel_mass, cut_snap.spark_cut
    );

    // Run 0.5s of normal, then 0.5s of cut
    let cfg = SynthConfig::from_block(&cut_block, FS);
    let mut synth = EngineSynth::new(cfg);
    synth.set_snapshot(&normal_snap);

    let half_sec = 24_000;
    let mut buf_normal = vec![0.0f32; half_sec * 2];
    synth.render(&mut buf_normal, 2);

    synth.set_snapshot(&cut_snap);
    let mut buf_cut = vec![0.0f32; half_sec * 2];
    synth.render(&mut buf_cut, 2);

    let mono_normal: Vec<f32> = buf_normal
        .chunks_exact(2)
        .map(|c| 0.5 * (c[0] + c[1]))
        .collect();
    let mono_cut: Vec<f32> = buf_cut
        .chunks_exact(2)
        .map(|c| 0.5 * (c[0] + c[1]))
        .collect();

    let (rms_norm, peak_norm, db_norm) = compute_rms_peak(&mono_normal[4800..]);
    let (rms_cut, peak_cut, db_cut) = compute_rms_peak(&mono_cut);

    println!(
        "Audio during Fired:     RMS: {:7.4} ({:5.1} dBFS), Peak: {:7.4}",
        rms_norm, db_norm, peak_norm
    );
    println!(
        "Audio during Spark Cut: RMS: {:7.4} ({:5.1} dBFS), Peak: {:7.4}",
        rms_cut, db_cut, peak_cut
    );

    let mut combined = buf_normal;
    combined.extend_from_slice(&buf_cut);
    write_wav(
        Path::new("/tmp/diag_cut_transition.wav"),
        &combined,
        2,
        FS as u32,
    )?;

    println!("\nWAV files written to /tmp/diag_*.wav for inspection.");
    Ok(())
}
