//! The `cpal` output stream and the lock-free bridge into it.
//!
//! # The two-thread contract
//!
//! ```text
//!   physics thread                          audio callback (real-time)
//!   ──────────────                          ──────────────────────────
//!   EngineBlock::update()                   drain queue -> newest snapshot
//!        │                                  EngineSynth::render(buffer)
//!        └─ push(EngineSnapshot) ──[rtrb]──▶ (never blocks, never allocates)
//! ```
//!
//! The audio callback runs on a thread the operating system schedules with a
//! hard deadline: at 48 kHz with a 512-frame buffer it has 10.7 ms to fill the
//! buffer, every time, or the device plays whatever was left in memory — which
//! is heard as a click. Anything that can make it wait for an unbounded time is
//! therefore forbidden, and that rules out more than it first appears to:
//!
//! - **No mutex.** Not because locking is slow, but because the physics thread
//!   can be preempted while holding the lock, and then the audio thread waits on
//!   a thread that is not running. This is priority inversion, and it produces
//!   dropouts that are impossible to reproduce under a debugger.
//! - **No allocation.** `malloc` takes a lock internally, so it has the same
//!   problem, plus it can trip a page fault.
//! - **No channel that can block or allocate**, which excludes
//!   `std::sync::mpsc`.
//!
//! [`rtrb`] is a single-producer single-consumer queue over a buffer allocated
//! once at construction. Both ends are wait-free: `push` and `pop` are a load,
//! a store, and an atomic index update. Nothing in the callback can be made to
//! wait for the physics thread.
//!
//! # Falling behind is not a dropout
//!
//! The two threads run at unrelated rates and neither waits for the other, so
//! the queue does not stay balanced. Both directions are handled without a
//! glitch:
//!
//! - **Queue empty** (physics ran late): the callback keeps its previous state
//!   and the synth coasts. Firing continues on schedule because crank phase is
//!   integrated in the audio thread, not sent as events — see
//!   [`crate::audio::dsp`]. Counted as [`AudioStats::starved`], which is a
//!   diagnostic, not an error.
//! - **Queue full** (physics ran fast, or the device stalled): the *newest*
//!   snapshot is dropped by the producer. Dropping the new one rather than
//!   overwriting the old is deliberate — the consumer drains to the newest entry
//!   every callback anyway, so a full queue means the callback has not run, and
//!   in that case nothing that arrives now will be heard in time regardless.
//!
//! Because the callback drains to the newest snapshot rather than consuming one
//! per buffer, a physics thread that runs faster than the audio device simply
//! has its intermediate frames skipped, which is the correct behaviour: the
//! synth wants the current state, not a backlog of stale ones.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use anyhow::{anyhow, Context, Result};
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{
    BufferSize, Device, SampleFormat, SampleRate, StreamConfig, SupportedBufferSize,
    SupportedStreamConfig, SupportedStreamConfigRange,
};
use rtrb::{Consumer, Producer, RingBuffer};

use crate::audio::dsp::{EngineSnapshot, EngineSynth, SynthConfig};

/// The sample rate the engine is designed around [Hz].
pub const PREFERRED_SAMPLE_RATE: u32 = 48_000;

/// Target callback size [frames].
///
/// 512 frames is 10.7 ms at 48 kHz: short enough that a throttle input is heard
/// promptly, long enough that a scheduling hiccup on a loaded machine does not
/// immediately become an underrun.
pub const PREFERRED_BUFFER_FRAMES: u32 = 512;

/// How many snapshots the queue can hold.
///
/// Sized for roughly a second of physics frames, so a stalled callback has to be
/// badly stalled before the producer starts dropping. The memory cost is
/// negligible — a snapshot is a few dozen bytes — and it is all allocated up
/// front.
pub const SNAPSHOT_QUEUE_CAPACITY: usize = 256;

/// Mono samples the scope tap can hold before the callback stops writing to it.
///
/// A little over 170 ms at 48 kHz, which is ten frames at 60 fps — deep enough
/// that a reader missing a few frames still sees continuous audio, shallow
/// enough that what it reads is recent.
pub const SCOPE_CAPACITY: usize = 8_192;

/// Knobs for [`EngineAudio::start_with`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AudioSettings {
    /// Sample rate to request [Hz]; falls back to the device default.
    pub sample_rate: u32,
    /// Callback size to request [frames]; falls back to the device default.
    pub buffer_frames: u32,
    /// Whether to begin playing immediately.
    pub autostart: bool,
}

impl Default for AudioSettings {
    fn default() -> Self {
        Self {
            sample_rate: PREFERRED_SAMPLE_RATE,
            buffer_frames: PREFERRED_BUFFER_FRAMES,
            autostart: true,
        }
    }
}

/// What the device actually gave us, which may not be what was asked for.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StreamInfo {
    /// Human-readable device name.
    pub device: String,
    /// Negotiated sample rate [Hz].
    pub sample_rate: u32,
    /// Negotiated channel count.
    pub channels: u16,
    /// Fixed callback size if one was granted, `None` if the device chooses.
    pub buffer_frames: Option<u32>,
    /// Whether the requested sample rate was honoured.
    pub sample_rate_honoured: bool,
}

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------

/// Counters the audio thread publishes for the rest of the program.
///
/// Every field is a relaxed atomic. Relaxed ordering is correct here because
/// these are independent counters with no happens-before relationship to defend
/// — nothing else is published through them — and stronger ordering would put a
/// barrier on the hot path for no benefit.
#[derive(Debug, Default)]
pub struct AudioStats {
    /// Callbacks serviced since the stream opened.
    callbacks: AtomicU64,
    /// Frames written since the stream opened.
    frames: AtomicU64,
    /// Callbacks that found the snapshot queue empty and coasted.
    starved: AtomicU64,
    /// Errors reported by `cpal` on this stream.
    errors: AtomicU64,
    /// Set once the device has gone away for good.
    disconnected: AtomicBool,
    /// Peak magnitude of the most recent callback, as `f32` bits.
    peak_bits: AtomicU32,
}

impl AudioStats {
    /// Callbacks serviced.
    pub fn callbacks(&self) -> u64 {
        self.callbacks.load(Ordering::Relaxed)
    }

    /// Frames written.
    pub fn frames(&self) -> u64 {
        self.frames.load(Ordering::Relaxed)
    }

    /// Callbacks that ran without a fresh snapshot.
    ///
    /// A steady trickle is normal and inaudible — it just means the audio device
    /// asked for samples more often than the physics produced frames. A ratio
    /// approaching 1.0 means the physics thread has effectively stopped, and the
    /// engine note will be frozen at its last state.
    pub fn starved(&self) -> u64 {
        self.starved.load(Ordering::Relaxed)
    }

    /// Stream errors reported by the backend.
    pub fn errors(&self) -> u64 {
        self.errors.load(Ordering::Relaxed)
    }

    /// Whether the device has been lost.
    pub fn disconnected(&self) -> bool {
        self.disconnected.load(Ordering::Relaxed)
    }

    /// Peak magnitude of the last callback, `0..` [-].
    pub fn peak(&self) -> f32 {
        f32::from_bits(self.peak_bits.load(Ordering::Relaxed))
    }

    /// Fraction of callbacks that had no new physics to work with, `0..=1`.
    pub fn starvation_ratio(&self) -> f64 {
        let callbacks = self.callbacks();
        if callbacks == 0 {
            0.0
        } else {
            self.starved() as f64 / callbacks as f64
        }
    }
}

// ---------------------------------------------------------------------------
// Scope tap
// ---------------------------------------------------------------------------

/// The read end of a tap on what the device is actually playing.
///
/// A meter or spectrum analyser cannot be driven from the snapshot the physics
/// pushes *in*: the synth integrates crank phase itself, so the waveform leaving
/// the callback is not recoverable from the state that went in. The only honest
/// source for a display of the sound is the sound. This carries it back.
///
/// The callback writes a mono downmix through the same wait-free queue the
/// snapshots use, and drops a whole buffer rather than a partial one when the
/// reader has fallen behind — a torn write would show up as a discontinuity,
/// which a spectrum reads as broadband noise that is not in the signal.
///
/// The reader is under no obligation to keep up. Nothing in the audio path waits
/// on it, and a scope that is never drained costs the callback one integer
/// comparison per buffer.
#[derive(Debug)]
pub struct AudioScope {
    samples: Consumer<f32>,
    sample_rate: u32,
}

impl AudioScope {
    /// Sample rate of the tapped stream [Hz].
    pub fn sample_rate(&self) -> u32 {
        self.sample_rate
    }

    /// Samples waiting to be read.
    pub fn available(&self) -> usize {
        self.samples.slots()
    }

    /// Drains everything available into the back of `sink`, keeping at most
    /// `keep` samples, and returns how many new samples arrived.
    ///
    /// `sink` is a rolling window: the caller owns it across frames so an
    /// analyser always has a full block to work on even in a frame where the
    /// device happened to deliver nothing.
    pub fn drain_into(&mut self, sink: &mut Vec<f32>, keep: usize) -> usize {
        let mut read = 0;
        while let Ok(sample) = self.samples.pop() {
            sink.push(sample);
            read += 1;
        }
        if sink.len() > keep {
            // `drain` on a `Vec` shifts the tail down, which is a memmove of at
            // most `keep` floats — cheaper here than the bookkeeping a ring
            // buffer would need for a window this small.
            sink.drain(..sink.len() - keep);
        }
        read
    }
}

// ---------------------------------------------------------------------------
// Device negotiation
// ---------------------------------------------------------------------------

/// Picks the best output configuration from what a device advertises.
///
/// Preference order, strongest first:
///
/// 1. `f32` samples. The synth works in `f32` and the whole point of asking for
///    it is to hand the device exactly what is already in the buffer; letting
///    `cpal` convert would add a pass over every sample for nothing.
/// 2. The requested sample rate, if the range covers it.
/// 3. Stereo, so the exhaust banks can be imaged. More channels than two are
///    accepted but the extras are left silent.
///
/// Returns `None` when the device advertises no `f32` output at all, which is
/// the caller's cue to fall back to the device default.
pub fn select_config(
    ranges: impl IntoIterator<Item = SupportedStreamConfigRange>,
    sample_rate: u32,
) -> Option<SupportedStreamConfig> {
    let wanted = SampleRate(sample_rate);
    let mut best: Option<(u8, SupportedStreamConfig)> = None;

    for range in ranges {
        if range.sample_format() != SampleFormat::F32 {
            continue;
        }
        // Exact rate if it is in range, otherwise the closest endpoint.
        let (rate, exact) =
            if range.min_sample_rate() <= wanted && wanted <= range.max_sample_rate() {
                (wanted, true)
            } else if wanted < range.min_sample_rate() {
                (range.min_sample_rate(), false)
            } else {
                (range.max_sample_rate(), false)
            };

        let channels = range.channels();
        // Higher score wins: the rate matters more than the channel layout, so
        // it is weighted above it.
        let score = u8::from(exact) * 4
            + match channels {
                2 => 2,
                1 => 1,
                _ => 0,
            };

        let config = match range.try_with_sample_rate(rate) {
            Some(config) => config,
            // Unreachable given the clamping above, but the API is fallible and
            // the panicking variant is not worth the risk in a startup path.
            None => continue,
        };
        if best.as_ref().is_none_or(|(b, _)| score > *b) {
            best = Some((score, config));
        }
    }

    best.map(|(_, config)| config)
}

/// Resolves a requested callback size against what the device allows.
///
/// A fixed buffer size is worth asking for: it makes the callback's workload
/// constant, which makes an overrun a property of the machine rather than of
/// which buffer size the device happened to pick this time. When the backend
/// will not commit to a range, `BufferSize::Default` is the honest answer.
pub fn select_buffer_size(supported: &SupportedBufferSize, requested: u32) -> BufferSize {
    match supported {
        SupportedBufferSize::Range { min, max } if min <= max => {
            BufferSize::Fixed(requested.clamp(*min, *max))
        }
        _ => BufferSize::Default,
    }
}

// ---------------------------------------------------------------------------
// The stream
// ---------------------------------------------------------------------------

/// A running engine-audio output stream.
///
/// Holds the `cpal` stream alive — dropping this stops the audio. The stream
/// handle is not `Send` on every backend, so build this on the thread that will
/// feed it and keep it there; that is also the thread running the physics, so
/// there is no reason to move it.
pub struct EngineAudio {
    // Declared first so it is dropped first: the callback borrows the consumer
    // end of the queue, and stopping the stream before the queue goes away
    // avoids relying on `rtrb`'s abandonment handling during teardown.
    stream: cpal::Stream,
    producer: Producer<EngineSnapshot>,
    stats: Arc<AudioStats>,
    info: StreamInfo,
    scope: Option<AudioScope>,
    dropped: u64,
    playing: bool,
}

impl EngineAudio {
    /// Opens the default output device with default settings.
    ///
    /// `config.sample_rate` is ignored and replaced with whatever the device
    /// negotiates, so the synth is always built for the rate it will actually
    /// run at.
    pub fn start(config: SynthConfig) -> Result<Self> {
        Self::start_with(config, AudioSettings::default())
    }

    /// Opens the default output device with explicit settings.
    pub fn start_with(mut config: SynthConfig, settings: AudioSettings) -> Result<Self> {
        let host = cpal::default_host();
        let device = host
            .default_output_device()
            .ok_or_else(|| anyhow!("no default audio output device"))?;
        let name = device
            .name()
            .unwrap_or_else(|_| "<unnamed output device>".to_string());

        let supported = negotiate(&device, settings.sample_rate)
            .with_context(|| format!("no usable f32 output configuration on '{name}'"))?;

        let sample_rate = supported.sample_rate().0;
        let channels = supported.channels();

        // Build the synth for the rate the device agreed to, not the one that
        // was asked for. Every frequency in the synth is derived from this, so
        // falling back to 44.1 kHz retunes the whole engine correctly rather
        // than pitching it.
        config.sample_rate = sample_rate as f32;

        let stats = Arc::new(AudioStats::default());
        let mut buffer_size = select_buffer_size(supported.buffer_size(), settings.buffer_frames);

        // A device can advertise a buffer range and still refuse a size inside
        // it. Retry once letting the backend choose rather than failing to make
        // any sound at all. The queue is rebuilt each attempt because the
        // callback takes ownership of the consumer end.
        let (stream, producer, scope) = loop {
            let (producer, consumer) = RingBuffer::<EngineSnapshot>::new(SNAPSHOT_QUEUE_CAPACITY);
            let (scope_producer, scope_consumer) = RingBuffer::<f32>::new(SCOPE_CAPACITY);
            let stream_config = StreamConfig {
                channels,
                sample_rate: supported.sample_rate(),
                buffer_size,
            };
            let synth = EngineSynth::new(config.clone());

            match build_stream(
                &device,
                &stream_config,
                synth,
                consumer,
                scope_producer,
                stats.clone(),
            ) {
                Ok(stream) => break (stream, producer, scope_consumer),
                Err(error) if matches!(buffer_size, BufferSize::Fixed(_)) => {
                    buffer_size = BufferSize::Default;
                    let _ = error;
                }
                Err(error) => {
                    return Err(error)
                        .with_context(|| format!("failed to open an output stream on '{name}'"));
                }
            }
        };

        if settings.autostart {
            stream.play().context("failed to start the output stream")?;
        }

        Ok(Self {
            stream,
            producer,
            stats,
            info: StreamInfo {
                device: name,
                sample_rate,
                channels,
                buffer_frames: match buffer_size {
                    BufferSize::Fixed(n) => Some(n),
                    BufferSize::Default => None,
                },
                sample_rate_honoured: sample_rate == settings.sample_rate,
            },
            scope: Some(AudioScope {
                samples: scope,
                sample_rate,
            }),
            dropped: 0,
            playing: settings.autostart,
        })
    }

    /// What the device negotiated.
    pub fn info(&self) -> &StreamInfo {
        &self.info
    }

    /// Live counters from the audio thread.
    pub fn stats(&self) -> &Arc<AudioStats> {
        &self.stats
    }

    /// Snapshots dropped because the queue was full.
    pub fn dropped_snapshots(&self) -> u64 {
        self.dropped
    }

    /// Takes the tap on the rendered output, once.
    ///
    /// Returns `None` on any call after the first: the tap is a single-consumer
    /// queue, so there is exactly one reader to hand out. The scope is `Send`
    /// even where `cpal::Stream` is not, which is the point — it lets a display
    /// thread read the audio without owning the stream.
    pub fn take_scope(&mut self) -> Option<AudioScope> {
        self.scope.take()
    }

    /// Hands a physics frame to the audio thread.
    ///
    /// Wait-free: this never blocks, never allocates, and is safe to call from
    /// inside the simulation loop. Returns `false` if the queue was full and the
    /// snapshot was discarded — see the module docs for why discarding is the
    /// right response.
    pub fn push(&mut self, snapshot: EngineSnapshot) -> bool {
        match self.producer.push(snapshot) {
            Ok(()) => true,
            Err(_) => {
                self.dropped += 1;
                false
            }
        }
    }

    /// Starts or resumes playback.
    pub fn play(&mut self) -> Result<()> {
        self.stream.play().context("failed to resume audio")?;
        self.playing = true;
        Ok(())
    }

    /// Pauses playback, if the backend supports it.
    pub fn pause(&mut self) -> Result<()> {
        self.stream.pause().context("failed to pause audio")?;
        self.playing = false;
        Ok(())
    }

    /// Whether the stream is currently running.
    pub fn is_playing(&self) -> bool {
        self.playing
    }
}

/// Finds a workable configuration, preferring `f32` at the requested rate.
fn negotiate(device: &Device, sample_rate: u32) -> Result<SupportedStreamConfig> {
    let ranges = device
        .supported_output_configs()
        .context("could not enumerate output configurations")?;

    if let Some(config) = select_config(ranges, sample_rate) {
        return Ok(config);
    }

    // Nothing advertised `f32`. The default config is still worth trying,
    // because some backends under-report what they support.
    let default = device
        .default_output_config()
        .context("device has no default output configuration")?;
    if default.sample_format() == SampleFormat::F32 {
        Ok(default)
    } else {
        Err(anyhow!(
            "device offers no 32-bit float output (default format is {:?})",
            default.sample_format()
        ))
    }
}

/// Wires the synth and the queue into a `cpal` callback.
///
/// Everything the callback touches is moved into it here: the synth (with all
/// its buffers already allocated), the consumer end of the queue, and an `Arc`
/// to the counters. After this function returns, the callback allocates nothing
/// and shares nothing that could block.
fn build_stream(
    device: &Device,
    config: &StreamConfig,
    mut synth: EngineSynth,
    mut consumer: Consumer<EngineSnapshot>,
    mut scope: Producer<f32>,
    stats: Arc<AudioStats>,
) -> Result<cpal::Stream, cpal::BuildStreamError> {
    let channels = config.channels as usize;
    let error_stats = stats.clone();

    device.build_output_stream(
        config,
        move |output: &mut [f32], _: &cpal::OutputCallbackInfo| {
            // Drain to the newest snapshot. Consuming one per callback would let
            // the synth fall progressively further behind whenever the physics
            // runs faster than the device, and the lag would be unbounded.
            let mut fresh = false;
            while let Ok(snapshot) = consumer.pop() {
                synth.set_snapshot(&snapshot);
                fresh = true;
            }
            if !fresh {
                stats.starved.fetch_add(1, Ordering::Relaxed);
            }

            // Fills every sample unconditionally — see `EngineSynth::render`.
            synth.render(output, channels);

            let mut peak = 0.0f32;
            for sample in output.iter() {
                peak = peak.max(sample.abs());
            }

            // Mono downmix to the scope, all or nothing. `slots` is a single
            // atomic load, so a scope nobody is reading costs one comparison
            // per buffer and is then skipped entirely.
            let frames = output.len() / channels;
            if scope.slots() >= frames {
                for frame in output.chunks_exact(channels) {
                    let sum: f32 = frame.iter().sum();
                    // `push` cannot fail here: the capacity was just checked and
                    // this is the only producer.
                    let _ = scope.push(sum / channels as f32);
                }
            }

            // A plain store, not a compare-exchange maximum: a CAS loop has no
            // bounded completion time, which is exactly what a real-time
            // callback may not contain. Callers wanting a peak hold can take the
            // maximum on their side.
            stats.peak_bits.store(peak.to_bits(), Ordering::Relaxed);
            stats
                .frames
                .fetch_add((output.len() / channels) as u64, Ordering::Relaxed);
            stats.callbacks.fetch_add(1, Ordering::Relaxed);
        },
        move |error| {
            error_stats.errors.fetch_add(1, Ordering::Relaxed);
            if matches!(error, cpal::StreamError::DeviceNotAvailable) {
                error_stats.disconnected.store(true, Ordering::Relaxed);
            }
        },
        // No timeout: the backend blocks as long as it needs to. A timeout here
        // would surface as a stream error on a machine that is merely busy.
        None::<Duration>,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn range(
        channels: u16,
        min: u32,
        max: u32,
        format: SampleFormat,
    ) -> SupportedStreamConfigRange {
        SupportedStreamConfigRange::new(
            channels,
            SampleRate(min),
            SampleRate(max),
            SupportedBufferSize::Range { min: 64, max: 4096 },
            format,
        )
    }

    #[test]
    fn prefers_f32_stereo_at_the_requested_rate() {
        let chosen = select_config(
            vec![
                range(2, 44_100, 44_100, SampleFormat::F32),
                range(1, 8_000, 96_000, SampleFormat::F32),
                range(2, 8_000, 96_000, SampleFormat::F32),
                range(2, 8_000, 96_000, SampleFormat::I16),
            ],
            48_000,
        )
        .expect("a config should be selectable");

        assert_eq!(chosen.sample_rate(), SampleRate(48_000));
        assert_eq!(chosen.channels(), 2);
        assert_eq!(chosen.sample_format(), SampleFormat::F32);
    }

    #[test]
    fn exact_rate_beats_channel_count() {
        // Mono at 48k must win over stereo that cannot reach it: the synth
        // downmixes cleanly, but a wrong sample rate mistunes everything.
        let chosen = select_config(
            vec![
                range(2, 96_000, 192_000, SampleFormat::F32),
                range(1, 48_000, 48_000, SampleFormat::F32),
            ],
            48_000,
        )
        .unwrap();
        assert_eq!(chosen.sample_rate(), SampleRate(48_000));
        assert_eq!(chosen.channels(), 1);
    }

    #[test]
    fn falls_back_to_the_nearest_available_rate() {
        let chosen =
            select_config(vec![range(2, 88_200, 192_000, SampleFormat::F32)], 48_000).unwrap();
        // Clamped to the bottom of the range, not left unset.
        assert_eq!(chosen.sample_rate(), SampleRate(88_200));
    }

    #[test]
    fn rejects_devices_with_no_float_output() {
        assert!(select_config(
            vec![
                range(2, 8_000, 96_000, SampleFormat::I16),
                range(2, 8_000, 96_000, SampleFormat::U16),
            ],
            48_000,
        )
        .is_none());
        assert!(select_config(Vec::new(), 48_000).is_none());
    }

    #[test]
    fn buffer_size_is_clamped_into_the_supported_range() {
        let supported = SupportedBufferSize::Range { min: 128, max: 512 };
        assert_eq!(select_buffer_size(&supported, 512), BufferSize::Fixed(512));
        assert_eq!(select_buffer_size(&supported, 64), BufferSize::Fixed(128));
        assert_eq!(select_buffer_size(&supported, 8192), BufferSize::Fixed(512));
        assert_eq!(
            select_buffer_size(&SupportedBufferSize::Unknown, 512),
            BufferSize::Default
        );
        // A backend reporting a nonsense range must not produce a nonsense size.
        assert_eq!(
            select_buffer_size(&SupportedBufferSize::Range { min: 900, max: 100 }, 512),
            BufferSize::Default
        );
    }

    #[test]
    fn stats_start_clean_and_report_ratios() {
        let stats = AudioStats::default();
        assert_eq!(stats.callbacks(), 0);
        assert_eq!(stats.starvation_ratio(), 0.0);
        assert!(!stats.disconnected());

        stats.callbacks.store(100, Ordering::Relaxed);
        stats.starved.store(25, Ordering::Relaxed);
        assert!((stats.starvation_ratio() - 0.25).abs() < 1e-9);

        stats.peak_bits.store(0.5f32.to_bits(), Ordering::Relaxed);
        assert_eq!(stats.peak(), 0.5);
    }

    /// The queue behaviour the callback depends on, exercised without a device.
    #[test]
    fn queue_drains_to_the_newest_snapshot() {
        let (mut producer, mut consumer) = RingBuffer::<EngineSnapshot>::new(8);
        for rpm in [1_000.0, 2_000.0, 3_000.0] {
            assert!(producer
                .push(EngineSnapshot {
                    rpm,
                    ..Default::default()
                })
                .is_ok());
        }
        let mut newest = None;
        while let Ok(snapshot) = consumer.pop() {
            newest = Some(snapshot);
        }
        assert_eq!(newest.map(|s| s.rpm), Some(3_000.0));
        assert!(consumer.pop().is_err(), "queue should now be empty");
    }

    #[test]
    fn queue_drops_rather_than_blocking_when_full() {
        let (mut producer, _consumer) = RingBuffer::<EngineSnapshot>::new(4);
        let mut accepted = 0;
        let mut dropped = 0;
        for _ in 0..16 {
            if producer.push(EngineSnapshot::default()).is_ok() {
                accepted += 1;
            } else {
                dropped += 1;
            }
        }
        assert_eq!(accepted, 4);
        assert_eq!(dropped, 12);
    }
}
