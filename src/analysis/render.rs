//! Offline rendering: the physics and the synth against a virtual clock.
//!
//! The same block, the same snapshots and the same synth the audio device
//! drives, stepped by a script instead of by a wall clock. Nothing here needs
//! audio hardware, which is what lets a measurement run in CI, and nothing here
//! reads the time except to report what the render cost.

use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::Path;
use std::time::Instant;

use anyhow::{Context, Result};

use crate::analysis::orders::RpmCurve;
use crate::analysis::script::RenderScript;
use crate::audio::dsp::EngineSynth;
use crate::audio::{Induction, SnapshotSource};
use crate::bench::EnginePreset;
use crate::environment::Environment;

/// Physics updates per second.
///
/// The rate the interactive simulator runs its solver at, kept here so an
/// offline render is stepped exactly as a live one is.
pub const PHYSICS_HZ: f64 = 240.0;

/// Sample rate an offline render works at [Hz].
///
/// Fixed rather than negotiated: a device that would only take 44.1 kHz would
/// move every resonance by 8 % relative to the analysis bins, and two
/// measurements taken on two machines have to be comparable.
pub const OFFLINE_RATE: f32 = 48_000.0;

/// Channels an offline render produces.
pub const CHANNELS: usize = 2;

/// Physics steps run before the first sample is taken.
///
/// The phase ring holds a whole cycle and starts full of ambient air, so a
/// snapshot taken immediately describes a cylinder that has never fired.
pub const PRIME_STEPS: usize = 600;

/// How much of the start of a render the continuity check ignores [s].
///
/// The synth fades in over 15 ms and the engine is legitimately near-silent
/// while it idles into the first analysis, so the opening half-second would
/// otherwise read as a dropout.
const CONTINUITY_LEAD_IN: f64 = 0.5;

// ---------------------------------------------------------------------------
// What to render
// ---------------------------------------------------------------------------

/// An engine, how it breathes, and what it is asked to do.
#[derive(Debug, Clone)]
pub struct RenderPlan<'a> {
    /// The engine out of the catalogue.
    pub preset: &'a EnginePreset,
    /// The drive cycle it is put through.
    pub script: &'a RenderScript,
    /// The induction fitted for this render.
    ///
    /// Defaults to the preset's own. It is separate because forced induction
    /// lives only in the audio path — the block is solved atmospherically
    /// either way — so a render can bolt a pair of turbos onto an atmospheric
    /// engine without the solver knowing.
    pub induction: Induction,
    /// Multiplier on the synth's master gain.
    pub gain: f64,
    /// Sample rate [Hz].
    pub sample_rate: f32,
    /// Ambient conditions the block is solved in.
    pub environment: Environment,
}

impl<'a> RenderPlan<'a> {
    /// A plan to run `preset` through `script` exactly as the catalogue has it.
    pub fn new(preset: &'a EnginePreset, script: &'a RenderScript) -> Self {
        Self {
            preset,
            script,
            induction: preset.induction,
            gain: 1.0,
            sample_rate: OFFLINE_RATE,
            environment: Environment::default(),
        }
    }

    /// Fits — or removes — an induction, overriding the preset's.
    pub fn with_induction(mut self, induction: Induction) -> Self {
        self.induction = induction;
        self
    }

    /// Scales the synth's master gain.
    pub fn with_gain(mut self, gain: f64) -> Self {
        self.gain = gain;
        self
    }

    /// Runs the physics and the synth through the whole script.
    ///
    /// Deterministic end to end: the block is primed for a fixed number of
    /// steps, the script is a function of elapsed time alone, and the synth's
    /// stochastic layers run off a fixed seed. Two calls with the same plan
    /// return the same samples, bit for bit.
    pub fn render(&self) -> Render {
        let dt = 1.0 / PHYSICS_HZ;
        let mut block = self.preset.block(self.environment);

        // Prime the phase ring so the first snapshot describes a running engine
        // rather than a cylinder full of ambient air.
        let idle = self.script.start_rpm();
        for _ in 0..PRIME_STEPS {
            block.update(dt, idle);
        }

        let mut config = self
            .preset
            .synth_config(&block, self.sample_rate)
            .with_induction(self.induction);
        config.master_gain *= self.gain;
        let mut synth = EngineSynth::new(config);
        // Built from the same induction the synth was, not from the preset's:
        // this one decides whether a shaft turns and that one decides whether
        // anything is listening to it, and they have to agree.
        let mut source = SnapshotSource::with_induction(&block, self.induction);

        let frames_per_step = (self.sample_rate as f64 / PHYSICS_HZ).round() as usize;
        // Rounded, not truncated: a script whose legs are scaled to a total
        // lands a hair either side of it in floating point, and truncating
        // would silently drop a whole physics step off the end of one render
        // and not off another it is supposed to be compared with.
        let steps = (self.script.seconds() * PHYSICS_HZ).round() as usize;
        let mut samples = Vec::with_capacity(steps * frames_per_step * CHANNELS);
        let mut chunk = vec![0.0f32; frames_per_step * CHANNELS];
        let mut speeds = Vec::with_capacity(steps + 1);

        let started = Instant::now();
        let mut synth_seconds = 0.0f64;

        for step in 0..steps {
            let (rpm, controls) = self.script.at(step as f64 * dt);
            speeds.push(rpm);

            block.update(dt, rpm);
            synth.set_snapshot(&source.sample(&block, rpm, dt, controls));

            let entered = Instant::now();
            synth.render(&mut chunk, CHANNELS);
            synth_seconds += entered.elapsed().as_secs_f64();

            samples.extend_from_slice(&chunk);
        }
        let wall_seconds = started.elapsed().as_secs_f64();

        let sample_rate = self.sample_rate as f64;
        let audio_seconds = samples.len() as f64 / (CHANNELS as f64 * sample_rate);
        Render {
            samples,
            channels: CHANNELS,
            sample_rate,
            rpm: RpmCurve::new(speeds, dt),
            cost: RenderCost {
                audio_seconds,
                wall_seconds,
                synth_seconds,
            },
        }
    }
}

// ---------------------------------------------------------------------------
// What comes back
// ---------------------------------------------------------------------------

/// What a render cost in wall clock.
///
/// Recorded now, before Stage 5 puts seventeen delay lines in the exhaust and
/// Stage 10d oversamples the nonlinear stages, so there is a number to compare
/// the cost of every later stage against.
#[derive(Debug, Clone, Copy)]
pub struct RenderCost {
    /// Audio produced [s].
    pub audio_seconds: f64,
    /// Wall clock for the physics and the synth together [s].
    pub wall_seconds: f64,
    /// Of which, inside the synth [s].
    pub synth_seconds: f64,
}

impl RenderCost {
    /// Audio seconds produced per second of wall clock, physics included.
    pub fn realtime_multiple(&self) -> f64 {
        self.audio_seconds / self.wall_seconds.max(1e-12)
    }

    /// The same figure for the synth alone.
    pub fn synth_realtime_multiple(&self) -> f64 {
        self.audio_seconds / self.synth_seconds.max(1e-12)
    }

    /// Fraction of one core the synth alone would need to keep up in real time.
    ///
    /// This is the budget Appendix B holds to 5 %: what the audio callback
    /// costs, with the solver that feeds it left out, because the solver runs
    /// on its own thread and has its own budget.
    pub fn synth_core_load(&self) -> f64 {
        self.synth_seconds / self.audio_seconds.max(1e-12)
    }
}

/// One offline render: the audio, the speed that produced it, and its cost.
#[derive(Debug, Clone)]
pub struct Render {
    /// Interleaved output, [`Self::channels`] samples per frame.
    pub samples: Vec<f32>,
    /// Channels in the interleave.
    pub channels: usize,
    /// Sample rate [Hz].
    pub sample_rate: f64,
    /// Speed the block was driven at, on the physics grid.
    pub rpm: RpmCurve,
    /// What the render cost.
    pub cost: RenderCost,
}

impl Render {
    /// Length of the audio [s].
    pub fn seconds(&self) -> f64 {
        self.samples.len() as f64 / (self.channels.max(1) as f64 * self.sample_rate)
    }

    /// The channels summed to mono, which is what the analysis reads.
    ///
    /// A mean rather than a sum: the exhaust is panned and the induction is
    /// centred, and summing would put 6 dB on whatever happens to be in both
    /// channels.
    pub fn mono(&self) -> Vec<f32> {
        let channels = self.channels.max(1);
        self.samples
            .chunks(channels)
            .map(|frame| frame.iter().sum::<f32>() / channels as f32)
            .collect()
    }

    /// Writes the render to a 32-bit float WAV.
    pub fn write_wav(&self, path: &Path) -> Result<()> {
        write_wav(
            path,
            &self.samples,
            self.channels as u16,
            self.sample_rate as u32,
        )
    }

    /// Measures what distinguishes clean output from a glitchy one.
    pub fn continuity(&self) -> Continuity {
        analyse(&self.samples, self.channels, self.sample_rate)
    }
}

// ---------------------------------------------------------------------------
// Continuity
// ---------------------------------------------------------------------------

/// Continuity report for a rendered buffer.
#[derive(Debug, Clone, Copy)]
pub struct Continuity {
    /// Largest absolute sample [full scale].
    pub peak: f32,
    /// Root-mean-square level [full scale].
    pub rms: f32,
    /// Largest step between consecutive samples of one channel.
    pub max_slew: f32,
    /// Blocks of 256 frames that came out silent after the lead-in.
    pub silent_blocks: usize,
    /// Samples that were not finite.
    pub non_finite: usize,
}

impl Continuity {
    /// Whether the buffer is free of dropouts, clipping and discontinuities.
    pub fn is_clean(&self) -> bool {
        self.non_finite == 0 && self.silent_blocks == 0 && self.peak <= 1.0 && self.max_slew < 0.5
    }
}

/// Measures the things that distinguish clean output from a glitchy one.
pub fn analyse(samples: &[f32], channels: usize, sample_rate: f64) -> Continuity {
    let channels = channels.max(1);
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

    // A dropout is a run of samples the synth failed to fill.
    let block = 256 * channels;
    let skip = (CONTINUITY_LEAD_IN * sample_rate) as usize * channels;
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

/// Writes 32-bit IEEE-float WAV, which keeps the signal bit-exact for analysis.
///
/// Float rather than the usual 16-bit integer because the point of the file is
/// to be measured: quantising to 16 bits would put a dither floor at -96 dBFS
/// over the top of the quiet orders these tables are supposed to be able to see.
pub fn write_wav(path: &Path, samples: &[f32], channels: u16, sample_rate: u32) -> Result<()> {
    let mut file =
        BufWriter::new(File::create(path).with_context(|| format!("creating {}", path.display()))?);
    let data_bytes = (samples.len() * 4) as u32;
    let block_align = channels * 4;

    file.write_all(b"RIFF")?;
    // 4 ("WAVE") + 8+18 (fmt) + 8+4 (fact) + 8+data
    file.write_all(&(4 + 26 + 12 + 8 + data_bytes).to_le_bytes())?;
    file.write_all(b"WAVE")?;

    // WAVE_FORMAT_IEEE_FLOAT requires the 18-byte fmt chunk and a fact chunk.
    file.write_all(b"fmt ")?;
    file.write_all(&18u32.to_le_bytes())?;
    file.write_all(&3u16.to_le_bytes())?; // IEEE float
    file.write_all(&channels.to_le_bytes())?;
    file.write_all(&sample_rate.to_le_bytes())?;
    file.write_all(&(sample_rate * block_align as u32).to_le_bytes())?;
    file.write_all(&block_align.to_le_bytes())?;
    file.write_all(&32u16.to_le_bytes())?;
    file.write_all(&0u16.to_le_bytes())?; // cbSize

    file.write_all(b"fact")?;
    file.write_all(&4u32.to_le_bytes())?;
    file.write_all(&(samples.len() as u32 / channels.max(1) as u32).to_le_bytes())?;

    file.write_all(b"data")?;
    file.write_all(&data_bytes.to_le_bytes())?;
    for sample in samples {
        file.write_all(&sample.to_le_bytes())?;
    }
    file.flush()?;
    Ok(())
}

// ---------------------------------------------------------------------------
// Reading a recording back in
// ---------------------------------------------------------------------------

/// Audio read off disk: a reference recording, or a render written earlier.
///
/// The counterpart of [`write_wav`], and the only way a measurement gets at
/// something this crate did not synthesise. A reference keeps the rate it was
/// recorded at rather than being resampled to [`OFFLINE_RATE`]: the analysis
/// takes a rate as an argument, and resampling a reference would put an
/// interpolation filter's own rolloff over the top of the noise-floor tilt the
/// comparison is trying to read.
#[derive(Debug, Clone)]
pub struct Recording {
    /// Interleaved samples, [`Self::channels`] per frame, full scale ±1.
    pub samples: Vec<f32>,
    /// Channels in the interleave.
    pub channels: usize,
    /// Sample rate [Hz].
    pub sample_rate: f64,
}

impl Recording {
    /// Frames in the recording.
    pub fn frames(&self) -> usize {
        self.samples.len() / self.channels.max(1)
    }

    /// Length of the recording [s].
    pub fn seconds(&self) -> f64 {
        self.frames() as f64 / self.sample_rate.max(f64::MIN_POSITIVE)
    }

    /// The channels averaged to mono, as [`Render::mono`] does it.
    ///
    /// The same convention on both sides of a comparison matters more than
    /// which convention it is: a sum would put 6 dB on whatever a recording
    /// happens to have in both channels and nothing on what it has in one.
    pub fn mono(&self) -> Vec<f32> {
        let channels = self.channels.max(1);
        self.samples
            .chunks(channels)
            .map(|frame| frame.iter().sum::<f32>() / channels as f32)
            .collect()
    }
}

/// Reads a WAV file: 8/16/24/32-bit PCM or 32/64-bit float, any rate.
///
/// Every one of those is something a reference recording actually arrives as —
/// a phone records 16-bit, a field recorder 24-bit, and anything that has been
/// through an editor comes back 32-bit float. Chunks are walked rather than
/// assumed to be in a fixed place, because a file out of a real editor carries
/// `LIST`, `id3 ` and `bext` chunks ahead of the audio.
pub fn read_wav(path: &Path) -> Result<Recording> {
    let bytes = fs::read(path).with_context(|| format!("reading {}", path.display()))?;
    if bytes.len() < 12 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        anyhow::bail!("{} is not a RIFF/WAVE file", path.display());
    }

    let u16_at = |i: usize| u16::from_le_bytes([bytes[i], bytes[i + 1]]);
    let u32_at =
        |i: usize| u32::from_le_bytes([bytes[i], bytes[i + 1], bytes[i + 2], bytes[i + 3]]);

    let mut format: Option<(u16, usize, f64, u16)> = None;
    let mut audio: Option<(usize, usize)> = None;

    // Chunks are `id[4] size[4] payload[size]`, each payload padded to an even
    // length. A truncated final chunk is taken for what is there rather than
    // rejected: a recording that stopped mid-write is still measurable.
    let mut at = 12usize;
    while at + 8 <= bytes.len() {
        let id = &bytes[at..at + 4];
        let size = u32_at(at + 4) as usize;
        let start = at + 8;
        let end = (start + size).min(bytes.len());

        if id == b"fmt " && end - start >= 16 {
            let mut tag = u16_at(start);
            let channels = u16_at(start + 2) as usize;
            let rate = u32_at(start + 4) as f64;
            let bits = u16_at(start + 14);
            // WAVE_FORMAT_EXTENSIBLE keeps the real format tag in the first two
            // bytes of its sub-format GUID.
            if tag == 0xFFFE && end - start >= 26 {
                tag = u16_at(start + 24);
            }
            format = Some((tag, channels, rate, bits));
        } else if id == b"data" {
            audio = Some((start, end));
        }

        at = start + size + (size & 1);
    }

    let (tag, channels, sample_rate, bits) =
        format.with_context(|| format!("{} has no fmt chunk", path.display()))?;
    let (start, end) = audio.with_context(|| format!("{} has no data chunk", path.display()))?;
    if channels == 0 || sample_rate <= 0.0 {
        anyhow::bail!(
            "{} declares {channels} channels at {sample_rate} Hz",
            path.display()
        );
    }

    let payload = &bytes[start..end];
    let samples: Vec<f32> = match (tag, bits) {
        // Unsigned, centred on 128, and the only PCM format that is.
        (1, 8) => payload
            .iter()
            .map(|&b| (b as f32 - 128.0) / 128.0)
            .collect(),
        (1, 16) => payload
            .chunks_exact(2)
            .map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32_768.0)
            .collect(),
        // Sign-extended into the top three bytes of an i32, then scaled by the
        // 24-bit full scale.
        (1, 24) => payload
            .chunks_exact(3)
            .map(|c| i32::from_le_bytes([0, c[0], c[1], c[2]]) as f32 / 2_147_483_648.0)
            .collect(),
        (1, 32) => payload
            .chunks_exact(4)
            .map(|c| i32::from_le_bytes([c[0], c[1], c[2], c[3]]) as f32 / 2_147_483_648.0)
            .collect(),
        (3, 32) => payload
            .chunks_exact(4)
            .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
            .collect(),
        (3, 64) => payload
            .chunks_exact(8)
            .map(|c| f64::from_le_bytes([c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7]]) as f32)
            .collect(),
        _ => anyhow::bail!(
            "{} is format {tag} at {bits} bits, which this harness cannot read",
            path.display()
        ),
    };

    // Whole frames only: half a frame at the end of a file is not a sample of
    // anything, and leaving it in would put the channels out of step.
    let frames = samples.len() / channels;
    Ok(Recording {
        samples: samples[..frames * channels].to_vec(),
        channels,
        sample_rate,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analysis::script;

    /// The claim the whole harness rests on: a rerun is the same run.
    ///
    /// If this ever fails, every order table in `docs/measurements/` becomes a
    /// record of the machine it was taken on rather than of the build.
    #[test]
    fn two_renders_of_one_script_are_bit_identical() {
        let preset = EnginePreset::inline_four();
        let script = script::sweep_up(&preset).scaled_to(1.5);

        let first = RenderPlan::new(&preset, &script).render();
        let second = RenderPlan::new(&preset, &script).render();

        assert_eq!(first.samples.len(), second.samples.len());
        assert!(!first.samples.is_empty(), "the render produced nothing");

        // Bit-for-bit, not within a tolerance: a difference of one ulp means
        // something in the chain is reading state it should not have.
        let differing = first
            .samples
            .iter()
            .zip(&second.samples)
            .position(|(a, b)| a.to_bits() != b.to_bits());
        assert_eq!(
            differing, None,
            "two renders diverged at sample {differing:?}"
        );
    }

    /// Every engine in the catalogue renders, and renders cleanly.
    #[test]
    fn every_preset_renders_without_a_dropout() {
        for preset in EnginePreset::catalogue() {
            let script = script::idle_hold(&preset).scaled_to(1.0);
            let render = RenderPlan::new(&preset, &script).render();
            let report = render.continuity();
            assert!(report.is_clean(), "{} rendered {report:?}", preset.name);
            assert!(report.rms > 0.0, "{} rendered silence", preset.name);
        }
    }

    /// A render is as long as the script says, and its speed curve with it.
    #[test]
    fn a_render_is_as_long_as_its_script() {
        let preset = EnginePreset::inline_four();
        let script = script::tip_in(&preset).scaled_to(2.0);
        let render = RenderPlan::new(&preset, &script).render();

        assert!((render.seconds() - 2.0).abs() < 1e-3);
        assert!((render.rpm.seconds() - 2.0).abs() < 0.01);
        assert!((render.rpm.at(0.0) - preset.idle).abs() < 1.0);
    }

    /// What the harness writes, the harness reads: a render survives the trip
    /// through a file with every sample bit-exact.
    ///
    /// The float WAV exists so that a measurement taken off a file is the same
    /// measurement as one taken off the buffer it came from. If this ever
    /// fails, every reading in `docs/measurements` taken from a written WAV is
    /// a reading of the file format.
    #[test]
    fn a_float_wav_round_trips_bit_for_bit() {
        let dir = std::env::temp_dir().join("engine-sim-wav-round-trip");
        let _ = fs::create_dir_all(&dir);
        let path = dir.join("float.wav");

        let written: Vec<f32> = (0..2_000)
            .map(|i| (i as f32 * 0.01).sin() * 0.9 * if i % 2 == 0 { 1.0 } else { -0.5 })
            .collect();
        write_wav(&path, &written, 2, 48_000).expect("writing");

        let read = read_wav(&path).expect("reading");
        assert_eq!(read.channels, 2);
        assert_eq!(read.sample_rate, 48_000.0);
        assert_eq!(read.frames(), 1_000);
        let differing = read
            .samples
            .iter()
            .zip(&written)
            .position(|(a, b)| a.to_bits() != b.to_bits());
        assert_eq!(differing, None, "the file changed sample {differing:?}");
        let _ = fs::remove_file(&path);
    }

    /// A reference recording arrives as 16-bit PCM at 44.1 kHz far more often
    /// than as anything else, and it has to read at the right level and rate.
    #[test]
    fn a_sixteen_bit_recording_reads_at_the_right_level() {
        let dir = std::env::temp_dir().join("engine-sim-wav-round-trip");
        let _ = fs::create_dir_all(&dir);
        let path = dir.join("pcm16.wav");

        // Hand-built, because the harness cannot write this format: one mono
        // frame at half scale, one at minus half scale, at 44.1 kHz.
        let mut file = Vec::new();
        file.extend_from_slice(b"RIFF");
        file.extend_from_slice(&(4u32 + 24 + 8 + 4).to_le_bytes());
        file.extend_from_slice(b"WAVE");
        file.extend_from_slice(b"fmt ");
        file.extend_from_slice(&16u32.to_le_bytes());
        file.extend_from_slice(&1u16.to_le_bytes()); // PCM
        file.extend_from_slice(&1u16.to_le_bytes()); // mono
        file.extend_from_slice(&44_100u32.to_le_bytes());
        file.extend_from_slice(&88_200u32.to_le_bytes());
        file.extend_from_slice(&2u16.to_le_bytes());
        file.extend_from_slice(&16u16.to_le_bytes());
        file.extend_from_slice(b"data");
        file.extend_from_slice(&4u32.to_le_bytes());
        file.extend_from_slice(&16_384i16.to_le_bytes());
        file.extend_from_slice(&(-16_384i16).to_le_bytes());
        fs::write(&path, &file).expect("writing");

        let read = read_wav(&path).expect("reading");
        assert_eq!(read.channels, 1);
        assert_eq!(read.sample_rate, 44_100.0);
        assert_eq!(read.samples, vec![0.5, -0.5]);
        let _ = fs::remove_file(&path);
    }

    /// Something that is not a WAV at all is refused, not read as noise.
    #[test]
    fn a_file_that_is_not_a_wav_is_refused() {
        let dir = std::env::temp_dir().join("engine-sim-wav-round-trip");
        let _ = fs::create_dir_all(&dir);
        let path = dir.join("not-a-wav.txt");
        fs::write(&path, b"this is not a recording of anything").expect("writing");
        assert!(read_wav(&path).is_err());
        let _ = fs::remove_file(&path);
    }

    /// Mono is the mean of the channels, not their sum.
    #[test]
    fn mono_averages_the_channels() {
        let render = Render {
            samples: vec![1.0, 0.0, 0.5, 0.5],
            channels: 2,
            sample_rate: OFFLINE_RATE as f64,
            rpm: RpmCurve::constant(1_000.0),
            cost: RenderCost {
                audio_seconds: 1.0,
                wall_seconds: 1.0,
                synth_seconds: 0.5,
            },
        };
        assert_eq!(render.mono(), vec![0.5, 0.5]);
        assert!((render.cost.synth_core_load() - 0.5).abs() < 1e-12);
    }
}
