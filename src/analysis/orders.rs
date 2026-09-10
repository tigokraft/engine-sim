//! Order tracking: how loud each engine order is, through a render.
//!
//! An *engine order* is a frequency expressed in cycles per crank revolution,
//! so order `n` sits at `f = n * rpm / 60` and follows the engine up and down
//! the rev range instead of staying put. It is the natural coordinate for
//! anything an engine does: a four-stroke fires `N_cyl / 2` times per
//! revolution, so a V8's firing order is 4, an inline-four's is 2 and a V12's
//! is 6, and the half-orders in between are what a cross-plane crank or an
//! uneven bank puts there.
//!
//! The measurement is a Hann-windowed STFT with the engine's speed curve read
//! alongside it: for each analysis frame, the speed at that instant says where
//! each order *should* be, and the level is read out of the spectrum there.
//! That is what makes an order table comparable between two builds whose rev
//! ranges do not line up sample for sample.
//!
//! # Levels
//!
//! Everything is reported in dBFS on an amplitude reference: a full-scale sine
//! reads `0 dB`, half scale reads `-6.02 dB`. The level of one order is
//! recovered by summing power across the window's main lobe and dividing by the
//! window's own energy, which makes the reading independent of where the
//! component happens to fall between two bins — a peak-bin reading would lose up
//! to 1.4 dB to scalloping, and that is the same size as the differences these
//! tables exist to detect.
//!
//! # What cannot be measured
//!
//! Two limits are real and are reported as absent data rather than hidden:
//!
//! - An order below [`Stft::resolution_floor`] cannot be told apart from DC by
//!   a window this long. Order 1 of an engine idling at 850 rpm is 14 Hz, and
//!   at a 8192-point window on 48 kHz the floor is 17.6 Hz, so it reads as
//!   nothing at all rather than as a number that is mostly leakage.
//! - During a sweep the order moves *while* the window is open. The reading
//!   answers that by integrating across the whole band the order swept through
//!   in that window rather than at one frequency, so a fast pull costs
//!   resolution but not level.

use std::f64::consts::PI;

/// Analysis window length [samples].
///
/// At 48 kHz this is 170 ms and a 5.86 Hz bin: long enough to resolve the low
/// orders of an idling engine, short enough that the speed does not run away
/// inside one window even on the hardest pull.
pub const WINDOW: usize = 8_192;

/// Hop between analysis frames, as a fraction of the window.
///
/// Four gives 75 % overlap, which is the point at which a Hann window's frames
/// sum flat and no transient can fall between two of them.
pub const OVERLAP: usize = 4;

/// Half-width of a Hann window's main lobe [bins].
///
/// A Hann window's transform is zero at ±2 bins from the component, so the
/// component's whole energy lives in the five bins either side of it. Summing
/// exactly that band is what makes the level independent of bin alignment.
const LOBE: isize = 2;

/// Level reported for silence [dB].
///
/// Finite so it can be averaged and printed like any other reading, and far
/// enough down that nothing an engine does is mistaken for it.
pub const SILENCE_DB: f64 = -200.0;

/// Amplitude as dBFS, with silence clamped to [`SILENCE_DB`].
pub fn db(amplitude: f64) -> f64 {
    if amplitude > 0.0 {
        (20.0 * amplitude.log10()).max(SILENCE_DB)
    } else {
        SILENCE_DB
    }
}

/// The orders a table reports, half-orders from 0.5 to 24.
///
/// Half-orders because half of what distinguishes one engine from another lives
/// between the integers: a cross-plane V8's uneven bank puts energy on the odd
/// half-orders that a flat-plane crank has nothing on. Twenty-four because that
/// is where the highest firing order of the catalogue's V12 lands its fourth
/// harmonic, above which the content is pipe resonance rather than firing.
pub fn half_orders() -> Vec<f64> {
    (1..=48).map(|n| n as f64 * 0.5).collect()
}

// ---------------------------------------------------------------------------
// Speed curve
// ---------------------------------------------------------------------------

/// Engine speed through a render, on a uniform time grid [rev/min].
///
/// Sampled at the physics rate rather than per audio frame: the solver is what
/// knows the speed, and a curve at 240 Hz is already far finer than the
/// analysis window it will be read through.
#[derive(Debug, Clone)]
pub struct RpmCurve {
    /// Speed at each grid point [rev/min].
    speeds: Vec<f64>,
    /// Grid spacing [s].
    dt: f64,
}

impl RpmCurve {
    /// A curve sampled every `dt` seconds.
    pub fn new(speeds: Vec<f64>, dt: f64) -> Self {
        debug_assert!(dt > 0.0, "a speed curve needs a positive grid spacing");
        Self {
            speeds,
            dt: dt.max(f64::MIN_POSITIVE),
        }
    }

    /// A curve that never changes, for a steady-state render or a test.
    pub fn constant(rpm: f64) -> Self {
        Self::new(vec![rpm], 1.0)
    }

    /// Length of the curve [s].
    pub fn seconds(&self) -> f64 {
        self.speeds.len().saturating_sub(1) as f64 * self.dt
    }

    /// Speed at time `t`, linearly interpolated and clamped at both ends.
    pub fn at(&self, t: f64) -> f64 {
        if self.speeds.is_empty() {
            return 0.0;
        }
        let x = (t / self.dt).clamp(0.0, (self.speeds.len() - 1) as f64);
        let i = x.floor() as usize;
        let Some(&next) = self.speeds.get(i + 1) else {
            return self.speeds[i];
        };
        let frac = x - i as f64;
        self.speeds[i] * (1.0 - frac) + next * frac
    }

    /// The slowest and fastest the engine turned between `t0` and `t1`.
    ///
    /// The endpoints are not enough on their own: a limiter bounce reverses
    /// several times inside one analysis window, and an order's band has to
    /// cover the whole excursion or the reading loses the part that was outside
    /// it.
    pub fn range(&self, t0: f64, t1: f64) -> (f64, f64) {
        let (t0, t1) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
        let mut lo = self.at(t0).min(self.at(t1));
        let mut hi = self.at(t0).max(self.at(t1));

        let first = (t0 / self.dt).ceil().max(0.0) as usize;
        let last = (t1 / self.dt).floor().max(0.0) as usize;
        for &speed in self
            .speeds
            .get(first..=last.min(self.speeds.len().saturating_sub(1)))
            .unwrap_or(&[])
        {
            lo = lo.min(speed);
            hi = hi.max(speed);
        }
        (lo, hi)
    }
}

// ---------------------------------------------------------------------------
// Transform
// ---------------------------------------------------------------------------

/// A Hann-windowed short-time Fourier transform, with its scratch buffers.
///
/// Solves in `f64` rather than reusing the `f32` transform behind the dashboard
/// spectrum: that one is sized for a display refresh and advances its twiddle
/// factors by repeated complex multiplication, which is invisible at 56 display
/// bands and is not what a measurement wants underneath it.
pub struct Stft {
    /// Window length [samples]; a power of two.
    size: usize,
    /// Hann coefficients.
    taper: Vec<f64>,
    /// `sum(w^2)`, the window's energy, for calibrating a level.
    taper_energy: f64,
    /// `exp(-2 pi i k / size)` for `k` in `0..size / 2`.
    twiddle: Vec<(f64, f64)>,
    /// Transform scratch.
    re: Vec<f64>,
    im: Vec<f64>,
    /// Power spectrum of the last block analysed, bins `0..=size / 2`.
    power: Vec<f64>,
    /// Sample rate [Hz].
    sample_rate: f64,
}

impl Stft {
    /// Builds a transform of `size` points, which must be a power of two.
    pub fn new(size: usize, sample_rate: f64) -> Self {
        assert!(
            size.is_power_of_two() && size >= 4,
            "window must be 2^n >= 4"
        );

        // Periodic rather than symmetric Hann: the periodic form is the one
        // whose overlapped frames sum flat, and the one whose transform is
        // exactly zero at ±2 bins.
        let taper: Vec<f64> = (0..size)
            .map(|n| 0.5 * (1.0 - (2.0 * PI * n as f64 / size as f64).cos()))
            .collect();
        let taper_energy = taper.iter().map(|w| w * w).sum();

        let twiddle = (0..size / 2)
            .map(|k| {
                let angle = -2.0 * PI * k as f64 / size as f64;
                (angle.cos(), angle.sin())
            })
            .collect();

        Self {
            size,
            taper,
            taper_energy,
            twiddle,
            re: vec![0.0; size],
            im: vec![0.0; size],
            power: vec![0.0; size / 2 + 1],
            sample_rate,
        }
    }

    /// Window length [samples].
    pub fn size(&self) -> usize {
        self.size
    }

    /// Samples between the start of one frame and the next.
    pub fn hop(&self) -> usize {
        self.size / OVERLAP
    }

    /// Width of one bin [Hz].
    pub fn bin_hz(&self) -> f64 {
        self.sample_rate / self.size as f64
    }

    /// Lowest frequency this window can separate from DC [Hz].
    ///
    /// A component's main lobe reaches [`LOBE`] bins either side of it, so the
    /// lowest one whose lobe clears DC entirely sits at `LOBE + 1` bins.
    pub fn resolution_floor(&self) -> f64 {
        (LOBE + 1) as f64 * self.bin_hz()
    }

    /// Windows and transforms one block, leaving its power spectrum in place.
    ///
    /// The block's mean is removed first. A DC term would put a main lobe over
    /// the bottom of the spectrum that has nothing to do with the engine, and
    /// the lowest orders are read from exactly there.
    pub fn analyse(&mut self, block: &[f32]) {
        debug_assert_eq!(block.len(), self.size);

        let mean = block.iter().map(|&s| s as f64).sum::<f64>() / self.size as f64;
        for (i, &sample) in block.iter().enumerate().take(self.size) {
            self.re[i] = (sample as f64 - mean) * self.taper[i];
            self.im[i] = 0.0;
        }

        fft(&mut self.re, &mut self.im, &self.twiddle);

        for (k, power) in self.power.iter_mut().enumerate() {
            *power = self.re[k] * self.re[k] + self.im[k] * self.im[k];
        }
    }

    /// Power spectrum of the last block analysed, bins `0..=size / 2`.
    pub fn power(&self) -> &[f64] {
        &self.power
    }

    /// Amplitude of a sinusoid at `hz`, or `None` if it sits under the floor.
    pub fn amplitude_at(&self, hz: f64) -> Option<f64> {
        self.amplitude_in_band(hz, hz)
    }

    /// Amplitude of a component that swept from `lo_hz` to `hi_hz` in this
    /// window, or `None` if the band sits under the resolution floor.
    ///
    /// The band is widened by the window's main lobe at both ends and the power
    /// in it summed, then divided by the window's energy. Summing power rather
    /// than reading the tallest bin is what makes the answer independent of bin
    /// alignment; the identity behind it is that a Hann window puts effectively
    /// all of a component's energy inside its own main lobe, so
    /// `sum|X|^2 = (A^2 / 4) * N * sum(w^2)`.
    pub fn amplitude_in_band(&self, lo_hz: f64, hi_hz: f64) -> Option<f64> {
        let (lo_hz, hi_hz) = if lo_hz <= hi_hz {
            (lo_hz, hi_hz)
        } else {
            (hi_hz, lo_hz)
        };
        if !lo_hz.is_finite() || !hi_hz.is_finite() || lo_hz < self.resolution_floor() {
            return None;
        }

        let bin = self.bin_hz();
        let first = ((lo_hz / bin).floor() as isize - LOBE).max(1) as usize;
        let last = (((hi_hz / bin).ceil() as isize + LOBE).max(1) as usize).min(self.size / 2);
        if first > last {
            return None;
        }

        let power: f64 = self.power[first..=last].iter().sum();
        Some(2.0 * (power / (self.size as f64 * self.taper_energy)).sqrt())
    }
}

/// In-place iterative radix-2 Cooley-Tukey FFT on split real/imaginary buffers.
///
/// `twiddle[k]` is `exp(-2 pi i k / n)`, read with a stride per stage rather
/// than advanced by multiplication, so no error accumulates across the block.
fn fft(re: &mut [f64], im: &mut [f64], twiddle: &[(f64, f64)]) {
    let n = re.len();
    debug_assert_eq!(n, im.len());
    debug_assert_eq!(n / 2, twiddle.len());

    // Bit-reversal permutation, by incrementing a reversed counter: the carry
    // propagates from the high bit down.
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }

    // Danielson-Lanczos: combine pairs of length-`len / 2` transforms.
    let mut len = 2;
    while len <= n {
        let stride = n / len;
        for start in (0..n).step_by(len) {
            for k in 0..len / 2 {
                let (wr, wi) = twiddle[k * stride];
                let (a, b) = (start + k, start + k + len / 2);
                let tr = re[b] * wr - im[b] * wi;
                let ti = re[b] * wi + im[b] * wr;
                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
            }
        }
        len <<= 1;
    }
}

// ---------------------------------------------------------------------------
// Order tracking
// ---------------------------------------------------------------------------

/// What one engine order did over a whole render.
#[derive(Debug, Clone, Copy)]
pub struct OrderLevel {
    /// The order, in cycles per crank revolution.
    pub order: f64,
    /// Level averaged over the frames it was resolvable in [dBFS].
    ///
    /// The average is taken in power and converted once, not taken over
    /// decibels: a mean of logarithms is not the level of anything.
    pub mean_db: f64,
    /// Level in the loudest single frame [dBFS].
    pub peak_db: f64,
    /// How many analysis frames the order sat above the resolution floor in.
    ///
    /// Zero means the order was never measurable — an idling engine's order 1
    /// is below what a window this long can separate from DC — and `mean_db` is
    /// then [`SILENCE_DB`] by convention rather than by measurement.
    pub frames: usize,
}

/// The order content of one render.
#[derive(Debug, Clone)]
pub struct OrderTable {
    /// One row per order requested, in the order they were requested.
    pub levels: Vec<OrderLevel>,
    /// Analysis frames the render was long enough for.
    pub frames: usize,
    /// Window the levels were read through [samples].
    pub window: usize,
    /// Lowest frequency that window could resolve [Hz].
    pub resolution_floor_hz: f64,
}

impl OrderTable {
    /// The row for `order`, if it was asked for.
    pub fn level(&self, order: f64) -> Option<&OrderLevel> {
        self.levels.iter().find(|l| (l.order - order).abs() < 1e-9)
    }

    /// The loudest order in the table, by mean level.
    pub fn loudest(&self) -> Option<&OrderLevel> {
        self.levels
            .iter()
            .filter(|l| l.frames > 0)
            .max_by(|a, b| a.mean_db.total_cmp(&b.mean_db))
    }
}

/// Reads the level of each order out of a rendered buffer.
///
/// `samples` is mono; `rpm` is the speed curve over the same span of time, and
/// is what turns a frequency into an order. Frames in which an order falls
/// under the resolution floor are left out of that order's average rather than
/// counted as silence, so a table taken at idle reports fewer frames on the low
/// orders instead of burying them.
pub fn track(samples: &[f32], sample_rate: f64, rpm: &RpmCurve, orders: &[f64]) -> OrderTable {
    let mut stft = Stft::new(WINDOW, sample_rate);
    let hop = stft.hop();

    let mut power_sum = vec![0.0f64; orders.len()];
    let mut peak = vec![0.0f64; orders.len()];
    let mut counted = vec![0usize; orders.len()];
    let mut frames = 0usize;

    let mut start = 0usize;
    while start + stft.size() <= samples.len() {
        stft.analyse(&samples[start..start + stft.size()]);

        // The speed the engine passed through while this window was open, so a
        // sweeping order is integrated across the band it swept rather than
        // sampled at one end of it.
        let t0 = start as f64 / sample_rate;
        let t1 = (start + stft.size()) as f64 / sample_rate;
        let (slow, fast) = rpm.range(t0, t1);

        for (i, &order) in orders.iter().enumerate() {
            let Some(amplitude) = stft.amplitude_in_band(order * slow / 60.0, order * fast / 60.0)
            else {
                continue;
            };
            power_sum[i] += amplitude * amplitude;
            peak[i] = peak[i].max(amplitude);
            counted[i] += 1;
        }

        frames += 1;
        start += hop;
    }

    let levels = orders
        .iter()
        .enumerate()
        .map(|(i, &order)| OrderLevel {
            order,
            mean_db: if counted[i] > 0 {
                db((power_sum[i] / counted[i] as f64).sqrt())
            } else {
                SILENCE_DB
            },
            peak_db: db(peak[i]),
            frames: counted[i],
        })
        .collect();

    OrderTable {
        levels,
        frames,
        window: stft.size(),
        resolution_floor_hz: stft.resolution_floor(),
    }
}
