//! Offline rendering: the physics and the synth against a virtual clock.
//!
//! The same block, the same snapshots and the same synth the audio device
//! drives, stepped by a script instead of by a wall clock. Nothing here needs
//! audio hardware, which is what lets a measurement run in CI, and nothing here
//! reads the time except to report what the render cost.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use anyhow::{Context, Result};

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
