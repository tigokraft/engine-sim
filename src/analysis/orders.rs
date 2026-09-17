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

/// How far either side of an order its own background is measured [orders].
///
/// A quarter of an order is half the way to the next half-order, so the two
/// bands either side of a line sit between it and its neighbours and measure
/// what is *under* it rather than what is next to it.
pub const BACKGROUND_OFFSET: f64 = 0.25;

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

    /// A linear sweep from `from` to `to` over `seconds`.
    ///
    /// Two points and no grid: linear interpolation between the endpoints *is*
    /// a linear sweep, exactly, so sampling it finer would only add rounding.
    /// This is the curve a reference recording gets when all that is known
    /// about it is where the pull started and where it ended.
    pub fn sweep(from: f64, to: f64, seconds: f64) -> Self {
        Self::new(vec![from, to], seconds.max(f64::MIN_POSITIVE))
    }

    /// A curve logged as `(time [s], speed [rev/min])` points, resampled onto a
    /// uniform grid of spacing `dt`.
    ///
    /// The honest shape for a real recording: a driver's pull is not linear, and
    /// a tachometer read off the video of one arrives as a handful of points at
    /// whatever instants they could be read at. Points need not be sorted, and
    /// anything before the first or after the last holds that endpoint — the
    /// same clamping [`Self::at`] does, for the same reason.
    pub fn from_points(points: &[(f64, f64)], dt: f64) -> Self {
        let dt = dt.max(f64::MIN_POSITIVE);
        let mut points: Vec<(f64, f64)> = points.to_vec();
        points.sort_by(|a, b| a.0.total_cmp(&b.0));
        let Some(&(first, _)) = points.first() else {
            return Self::constant(0.0);
        };
        let last = points.last().map_or(first, |p| p.0);

        let steps = ((last - first).max(0.0) / dt).round() as usize;
        let speeds = (0..=steps)
            .map(|i| {
                let t = first + i as f64 * dt;
                // The pair the instant falls between, or the nearer endpoint.
                let after = points.iter().position(|&(u, _)| u >= t);
                match after {
                    None => points[points.len() - 1].1,
                    Some(0) => points[0].1,
                    Some(j) => {
                        let (t0, r0) = points[j - 1];
                        let (t1, r1) = points[j];
                        let span = t1 - t0;
                        if span > 0.0 {
                            r0 + (r1 - r0) * (t - t0) / span
                        } else {
                            r1
                        }
                    }
                }
            })
            .collect();
        Self::new(speeds, dt)
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
    /// A block shorter than the window is zero-padded, which costs it level
    /// rather than leaving whatever the previous block put in the scratch.
    pub fn analyse(&mut self, block: &[f32]) {
        debug_assert_eq!(block.len(), self.size);

        let taken = block.len().min(self.size);
        let mean = block[..taken].iter().map(|&s| s as f64).sum::<f64>() / self.size as f64;
        for i in 0..self.size {
            let sample = block.get(i).map_or(0.0, |&s| s as f64 - mean);
            self.re[i] = sample * self.taper[i];
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
        self.amplitude_in_band_of(&self.power, lo_hz, hi_hz)
    }

    /// One bin's power on the same amplitude scale as a whole component.
    ///
    /// A single bin holds only the part of a component that landed in it, so
    /// this is not the level of anything on its own; it is the scale on which
    /// two neighbouring bins can be compared.
    pub fn bin_amplitude(&self, power: f64) -> f64 {
        2.0 * (power.max(0.0) / (self.size as f64 * self.taper_energy)).sqrt()
    }

    /// The same reading, taken from a power spectrum held elsewhere.
    ///
    /// A long-term average is the same transform accumulated over many frames,
    /// and has to be calibrated by the same window energy as a single one.
    pub fn amplitude_in_band_of(&self, power: &[f64], lo_hz: f64, hi_hz: f64) -> Option<f64> {
        let (lo_hz, hi_hz) = if lo_hz <= hi_hz {
            (lo_hz, hi_hz)
        } else {
            (hi_hz, lo_hz)
        };
        if !lo_hz.is_finite() || !hi_hz.is_finite() || lo_hz < self.resolution_floor() {
            return None;
        }

        let bin = self.bin_hz();
        let top = (self.size / 2).min(power.len().saturating_sub(1));
        let first = ((lo_hz / bin).floor() as isize - LOBE).max(1) as usize;
        let last = (((hi_hz / bin).ceil() as isize + LOBE).max(1) as usize).min(top);
        if first > last {
            return None;
        }

        let summed: f64 = power[first..=last].iter().sum();
        Some(2.0 * (summed / (self.size as f64 * self.taper_energy)).sqrt())
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
    ///
    /// This is the level *in the order's band*, which is the line plus whatever
    /// else is passing through it. On a sweep that matters: order 2.5 of an
    /// engine pulling from 850 to 7400 rpm drags its band across every
    /// resonance between 35 and 310 Hz, and reads them all, whether or not the
    /// crank drives that order at all. [`Self::line_db`] is the figure that
    /// does not.
    pub mean_db: f64,
    /// Level in the loudest single frame [dBFS].
    pub peak_db: f64,
    /// Level in the bands a quarter-order either side of it [dBFS].
    ///
    /// What is under the line: the broadband content and the tails of whatever
    /// resonances the order was sweeping through, measured through bands of the
    /// same width so the two readings are on one scale.
    pub background_db: f64,
    /// Level of the order *line*, with its own background taken out [dBFS].
    ///
    /// Power subtraction, not a decibel difference: the band holds the line and
    /// the background together, so the line alone is
    /// `sqrt(max(0, P_band - P_background))`. An order with no line in it comes
    /// back at [`SILENCE_DB`], which is the answer the crank's nulls deserve
    /// and the one a band level cannot give.
    pub line_db: f64,
    /// How many analysis frames the order sat above the resolution floor in.
    ///
    /// Zero means the order was never measurable — an idling engine's order 1
    /// is below what a window this long can separate from DC — and `mean_db` is
    /// then [`SILENCE_DB`] by convention rather than by measurement.
    pub frames: usize,
    /// Of those, how many the background could be measured in.
    ///
    /// Fewer, and sometimes none: at 850 rpm a quarter of an order is 3.5 Hz,
    /// well inside the window's own main lobe, so an idle has a band level but
    /// no line level.
    pub background_frames: usize,
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
    let mut background_sum = vec![0.0f64; orders.len()];
    let mut line_sum = vec![0.0f64; orders.len()];
    let mut background_counted = vec![0usize; orders.len()];
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

            // The background either side, but only where a quarter-order is
            // further from the line than the window's own main lobe. Below
            // that the two bands overlap the line they are supposed to measure
            // under, and the answer would be the line again.
            if BACKGROUND_OFFSET * slow / 60.0 < stft.resolution_floor() {
                continue;
            }
            let mut background = 0.0;
            let mut sides = 0;
            for offset in [-BACKGROUND_OFFSET, BACKGROUND_OFFSET] {
                let side = order + offset;
                if side < BACKGROUND_OFFSET {
                    continue;
                }
                if let Some(amplitude) =
                    stft.amplitude_in_band(side * slow / 60.0, side * fast / 60.0)
                {
                    background += amplitude * amplitude;
                    sides += 1;
                }
            }
            if sides == 0 {
                continue;
            }
            let background = background / sides as f64;
            background_sum[i] += background;
            line_sum[i] += (amplitude * amplitude - background).max(0.0);
            background_counted[i] += 1;
        }

        frames += 1;
        start += hop;
    }

    let levels = orders
        .iter()
        .enumerate()
        .map(|(i, &order)| {
            let mean = |sum: f64, count: usize| {
                if count > 0 {
                    db((sum / count as f64).sqrt())
                } else {
                    SILENCE_DB
                }
            };
            OrderLevel {
                order,
                mean_db: mean(power_sum[i], counted[i]),
                peak_db: db(peak[i]),
                background_db: mean(background_sum[i], background_counted[i]),
                line_db: mean(line_sum[i], background_counted[i]),
                frames: counted[i],
                background_frames: background_counted[i],
            }
        })
        .collect();

    OrderTable {
        levels,
        frames,
        window: stft.size(),
        resolution_floor_hz: stft.resolution_floor(),
    }
}

// ---------------------------------------------------------------------------
// Order balance
// ---------------------------------------------------------------------------

/// One order's level, measured against another order's [dB].
#[derive(Debug, Clone, Copy)]
pub struct BalanceLevel {
    /// The order, in cycles per crank revolution.
    pub order: f64,
    /// How far this order sits above or below the reference order [dB], or
    /// `None` where it was never resolvable.
    pub relative_db: Option<f64>,
}

/// An order table with one order taken as its reference: the *balance* between
/// orders, with the absolute level divided out.
///
/// This is the only form in which a recording and a render can be compared.
/// A recording arrives at whatever level a microphone, a preamp, a distance and
/// a mastering chain left it at, and a render arrives at whatever the preset's
/// master gain asks for; the difference between two of its orders survives all
/// of that and the absolute level of either survives none of it.
///
/// It is also what keeps the calibration loop physical, and that is the point
/// of it rather than a side effect. A gain constant — anywhere from the
/// excitation to the master fader — moves every order by the same number of
/// decibels, so it cancels out of every figure in this table and cannot close
/// a single disagreement in it. What *can* move one order relative to another
/// is geometry: a length, a volume, a radius, a reflection. When this table
/// disagrees with a reference, the fix is in the plumbing or it does not exist.
#[derive(Debug, Clone)]
pub struct Balance {
    /// The order everything else is quoted against.
    pub reference_order: f64,
    /// The absolute level of that order [dBFS], kept only so a report can say
    /// what was divided out.
    pub reference_db: f64,
    /// One row per order, in the order they were measured.
    pub levels: Vec<BalanceLevel>,
}

impl Balance {
    /// Builds a balance from levels already in decibels, relative to
    /// `reference_order`.
    ///
    /// `levels` are absolute — dBFS from a render, or the crank's own comb in
    /// dB — and `None` is an order that could not be measured, which is not the
    /// same as one that was silent.
    pub fn from_levels(reference_order: f64, levels: &[(f64, Option<f64>)]) -> Self {
        let reference_db = levels
            .iter()
            .find(|(order, _)| (order - reference_order).abs() < 1e-9)
            .and_then(|&(_, db)| db)
            .unwrap_or(SILENCE_DB);
        Self {
            reference_order,
            reference_db,
            levels: levels
                .iter()
                .map(|&(order, db)| BalanceLevel {
                    order,
                    relative_db: db.map(|db| db - reference_db),
                })
                .collect(),
        }
    }

    /// This order's level relative to the reference order [dB].
    pub fn at(&self, order: f64) -> Option<f64> {
        self.levels
            .iter()
            .find(|l| (l.order - order).abs() < 1e-9)
            .and_then(|l| l.relative_db)
    }

    /// The same balance restricted to the orders in `orders`.
    ///
    /// What a comparison uses to answer a question about part of the comb —
    /// the orders the crank actually drives, say, as against the ones it leaves
    /// empty, which are a different question and take a different test.
    pub fn only(&self, orders: &[f64]) -> Self {
        Self {
            reference_order: self.reference_order,
            reference_db: self.reference_db,
            levels: self
                .levels
                .iter()
                .filter(|l| orders.iter().any(|o| (o - l.order).abs() < 1e-9))
                .copied()
                .collect(),
        }
    }

    /// The loudest order in the balance, by level relative to the reference.
    pub fn loudest(&self) -> Option<&BalanceLevel> {
        self.levels
            .iter()
            .filter(|l| l.relative_db.is_some())
            .max_by(|a, b| {
                a.relative_db
                    .unwrap_or(SILENCE_DB)
                    .total_cmp(&b.relative_db.unwrap_or(SILENCE_DB))
            })
    }

    /// How far this balance sits from `reference`, order by order.
    ///
    /// Only orders resolvable on both sides are compared; an order missing from
    /// either is reported as uncompared rather than as agreement.
    pub fn against(&self, reference: &Balance) -> Comparison {
        let mut deltas = Vec::new();
        let mut missing = 0usize;
        for level in &self.levels {
            let Some(mine) = level.relative_db else {
                missing += 1;
                continue;
            };
            let Some(theirs) = reference.at(level.order) else {
                missing += 1;
                continue;
            };
            deltas.push(OrderDelta {
                order: level.order,
                reference_db: theirs,
                measured_db: mine,
                delta_db: mine - theirs,
            });
        }
        Comparison { deltas, missing }
    }
}

impl OrderTable {
    /// This table's *order lines* as a balance, each with its own background
    /// taken out.
    ///
    /// The form to compare against a firing pattern. A band level answers "how
    /// much energy went past this order", which on a sweep is largely a
    /// question about what resonances the band crossed; a line level answers
    /// "how much of it was on this order", which is the question a crank
    /// answers. An order with no line in it is absent here rather than
    /// reported at the level of its own background.
    pub fn line_balance(&self, reference_order: f64) -> Balance {
        let levels: Vec<(f64, Option<f64>)> = self
            .levels
            .iter()
            .map(|l| {
                (
                    l.order,
                    (l.background_frames > 0 && l.line_db > SILENCE_DB).then_some(l.line_db),
                )
            })
            .collect();
        Balance::from_levels(reference_order, &levels)
    }

    /// This table as a balance against one of its own orders.
    ///
    /// The firing order is the one to ask for: it is the order the engine
    /// drives hardest, it is resolvable over the widest speed range, and it is
    /// the one a listener hears as the note.
    pub fn balance(&self, reference_order: f64) -> Balance {
        let levels: Vec<(f64, Option<f64>)> = self
            .levels
            .iter()
            .map(|l| (l.order, if l.frames > 0 { Some(l.mean_db) } else { None }))
            .collect();
        Balance::from_levels(reference_order, &levels)
    }
}

/// One order's disagreement between a measurement and a reference [dB].
#[derive(Debug, Clone, Copy)]
pub struct OrderDelta {
    /// The order.
    pub order: f64,
    /// Where the reference put it, relative to the reference order [dB].
    pub reference_db: f64,
    /// Where the measurement put it, on the same footing [dB].
    pub measured_db: f64,
    /// Measured minus reference: positive where the measurement is too loud.
    pub delta_db: f64,
}

/// How far one order balance sits from another.
#[derive(Debug, Clone)]
pub struct Comparison {
    /// One row per order compared, in the order they were measured.
    pub deltas: Vec<OrderDelta>,
    /// Orders that could not be compared because one side could not resolve
    /// them.
    pub missing: usize,
}

impl Comparison {
    /// The largest disagreement on any order [dB].
    pub fn max_abs_db(&self) -> f64 {
        self.worst().map_or(0.0, |d| d.delta_db.abs())
    }

    /// Root-mean-square disagreement across the orders compared [dB].
    ///
    /// The headline figure, because a single order can disagree for a reason
    /// that is about that order — a mode sitting on it at one engine speed —
    /// while the whole comb sliding one way is about the model.
    pub fn rms_db(&self) -> f64 {
        if self.deltas.is_empty() {
            return 0.0;
        }
        let sum: f64 = self.deltas.iter().map(|d| d.delta_db * d.delta_db).sum();
        (sum / self.deltas.len() as f64).sqrt()
    }

    /// Mean disagreement across the orders compared [dB].
    ///
    /// A tilt in the comb rather than a scatter in it: if every order above the
    /// reference is down and every order below it is up, this stays near zero
    /// and [`Self::rms_db`] does not, and the two together say which it is.
    pub fn mean_db(&self) -> f64 {
        if self.deltas.is_empty() {
            return 0.0;
        }
        self.deltas.iter().map(|d| d.delta_db).sum::<f64>() / self.deltas.len() as f64
    }

    /// The disagreement on one order [dB], if it was compared at all.
    pub fn at(&self, order: f64) -> Option<f64> {
        self.deltas
            .iter()
            .find(|d| (d.order - order).abs() < 1e-9)
            .map(|d| d.delta_db)
    }

    /// The order that disagrees most.
    pub fn worst(&self) -> Option<&OrderDelta> {
        self.deltas
            .iter()
            .max_by(|a, b| a.delta_db.abs().total_cmp(&b.delta_db.abs()))
    }

    /// Whether every order compared agrees to within `tolerance_db`.
    pub fn within(&self, tolerance_db: f64) -> bool {
        !self.deltas.is_empty() && self.max_abs_db() <= tolerance_db
    }
}

// ---------------------------------------------------------------------------
// Resonances
// ---------------------------------------------------------------------------

/// Smallest rise above the surrounding spectrum that counts as a peak [dB].
///
/// An absolute floor under the adaptive test below, so that a very long and
/// very smooth average does not start reporting half-decibel ripples as modes.
pub const MIN_PROMINENCE_DB: f64 = 3.0;

/// How far above the average's own residual scatter a peak has to stand.
///
/// Averaging `M` periodograms leaves a chi-square scatter of roughly
/// `4.34 / sqrt(M)` dB in every bin, and over the few thousand bins of a
/// spectrum the largest excursion of pure noise lands a little under five of
/// those. Six is the first multiple that reports nothing at all from white
/// noise, and it means a peak found in a short render is as trustworthy as one
/// found in a long one without either having to know how long it was.
pub const PROMINENCE_SIGMA: f64 = 6.0;

/// A peak in a render's long-term average spectrum.
///
/// This is the shape the *plumbing* puts on the sound: a quarter-wave runner,
/// an expansion chamber, a Helmholtz volume and a block mode all sit at a fixed
/// frequency while the orders sweep past them, so they are what survives
/// averaging a whole rev range together.
#[derive(Debug, Clone, Copy)]
pub struct Peak {
    /// Centre frequency [Hz], interpolated between bins.
    pub hz: f64,
    /// Level of the component at that frequency [dBFS].
    pub db: f64,
    /// How far the peak stands above the higher of the two valleys around it
    /// [dB].
    ///
    /// Prominence rather than absolute level is what separates a resonance from
    /// a bump on the side of a louder one: a shoulder 40 dB up on the noise
    /// floor but only 1 dB above its own surroundings is not a mode.
    pub prominence_db: f64,
}

/// The long-term average spectrum of a render.
///
/// Welch's method: the power spectra of every overlapping frame, averaged.
/// Averaging power rather than a single long transform is what turns the
/// engine's swept orders into a smooth background and leaves the fixed
/// resonances standing up out of it.
pub struct AverageSpectrum {
    /// The transform that produced it, kept for its calibration.
    stft: Stft,
    /// Mean power per bin, `0..=size / 2`.
    power: Vec<f64>,
    /// Frames averaged.
    frames: usize,
}

impl AverageSpectrum {
    /// Averages every whole window in `samples`.
    pub fn of(samples: &[f32], sample_rate: f64) -> Self {
        let mut stft = Stft::new(WINDOW, sample_rate);
        let hop = stft.hop();
        let mut power = vec![0.0f64; stft.size() / 2 + 1];
        let mut frames = 0usize;

        let mut start = 0usize;
        while start + stft.size() <= samples.len() {
            stft.analyse(&samples[start..start + stft.size()]);
            for (sum, bin) in power.iter_mut().zip(stft.power()) {
                *sum += bin;
            }
            frames += 1;
            start += hop;
        }

        if frames > 0 {
            for bin in &mut power {
                *bin /= frames as f64;
            }
        }
        Self {
            stft,
            power,
            frames,
        }
    }

    /// Frames averaged; zero if the render was shorter than one window.
    pub fn frames(&self) -> usize {
        self.frames
    }

    /// Width of one bin [Hz].
    pub fn bin_hz(&self) -> f64 {
        self.stft.bin_hz()
    }

    /// Level of a component at `hz` [dBFS], on the same reference as an order.
    pub fn db_at(&self, hz: f64) -> Option<f64> {
        self.stft.amplitude_in_band_of(&self.power, hz, hz).map(db)
    }

    /// Per-bin level [dB], for finding peaks and measuring how far they stand up.
    ///
    /// Not the level of a *component* — a single bin holds only the part of one
    /// that landed in it — but on a fixed scale, which is all a comparison
    /// between neighbouring bins needs. Reported levels come from
    /// [`Self::db_at`], which sums the whole lobe.
    fn bin_db(&self, k: usize) -> f64 {
        db(self.stft.bin_amplitude(self.power[k]))
    }

    /// Scatter left in the average by the finite number of frames [dB].
    ///
    /// Estimated from the differences between bins a main lobe apart, through a
    /// median rather than a mean: a real resonance puts a large difference on
    /// its two flanks, and a median ignores the handful of bins that are on
    /// one. The gap has to clear the window's own main lobe, or the two bins
    /// share energy and their difference understates the scatter.
    ///
    /// A normal variable's median absolute value is `0.6745 sigma`, and the
    /// difference of two independent bins carries `sqrt(2)` times one bin's.
    pub fn scatter_db(&self) -> f64 {
        let bins = self.power.len();
        let first = (self.stft.resolution_floor() / self.bin_hz()).ceil() as usize;
        let gap = LOBE as usize + 1;
        if first + gap >= bins {
            return 0.0;
        }

        let curve: Vec<f64> = (first..bins).map(|k| self.bin_db(k)).collect();
        let mut diffs: Vec<f64> = curve
            .windows(gap + 1)
            .map(|w| (w[gap] - w[0]).abs())
            .collect();
        diffs.sort_by(f64::total_cmp);
        diffs[diffs.len() / 2] / 0.6745 / std::f64::consts::SQRT_2
    }

    /// Slope of the spectrum's own floor between `lo_hz` and `hi_hz`
    /// [dB/octave], or `None` if the band is too narrow to fit a line to.
    ///
    /// The third comparison metric, after order balance and resonance
    /// placement, and the one that says whether a synth is *shaped* like the
    /// thing it is copying. A real recording's floor falls away with frequency
    /// — viscothermal loss down the pipe goes as `sqrt(f)`, the radiating mouth
    /// rolls off above `ka = 1`, and the air between the car and the microphone
    /// takes the top off what is left. A model whose orders all land in the
    /// right place can still read as synthetic because its floor is flat.
    ///
    /// Taken as the median of each octave band rather than a fit to every bin:
    /// the floor is what is *left* when the resonances are taken out, and a
    /// median over an octave of bins is the cheapest honest way to take them
    /// out. A least-squares fit straight to the spectrum would be a fit to the
    /// tallest peaks in it.
    ///
    /// Gain-invariant, like a balance and for the same reason: a slope does not
    /// know what level it started from.
    pub fn tilt_db_per_octave(&self, lo_hz: f64, hi_hz: f64) -> Option<f64> {
        if self.frames == 0
            || !(lo_hz.is_finite() && hi_hz.is_finite())
            || lo_hz <= 0.0
            || hi_hz <= 2.0 * lo_hz
        {
            return None;
        }
        let bin = self.bin_hz();
        let top = ((hi_hz / bin).floor() as usize).min(self.power.len() - 1);

        // One point per octave, at the band's geometric centre.
        let mut points: Vec<(f64, f64)> = Vec::new();
        let mut low = lo_hz.max(self.stft.resolution_floor());
        while low * 2.0 <= hi_hz {
            let first = (low / bin).ceil() as usize;
            let last = ((2.0 * low / bin).floor() as usize).min(top);
            if last > first + 8 {
                let mut band: Vec<f64> = (first..=last).map(|k| self.bin_db(k)).collect();
                band.sort_by(f64::total_cmp);
                points.push((
                    (low * std::f64::consts::SQRT_2).log2(),
                    band[band.len() / 2],
                ));
            }
            low *= 2.0;
        }
        if points.len() < 3 {
            return None;
        }

        // Ordinary least squares through the octave medians.
        let n = points.len() as f64;
        let mean_x = points.iter().map(|p| p.0).sum::<f64>() / n;
        let mean_y = points.iter().map(|p| p.1).sum::<f64>() / n;
        let sxy: f64 = points
            .iter()
            .map(|(x, y)| (x - mean_x) * (y - mean_y))
            .sum();
        let sxx: f64 = points
            .iter()
            .map(|(x, _)| (x - mean_x) * (x - mean_x))
            .sum();
        (sxx > 0.0).then(|| sxy / sxx)
    }

    /// The `count` most prominent peaks, in frequency order.
    ///
    /// `min_prominence_db` is a floor: the test actually applied is the larger
    /// of it and [`PROMINENCE_SIGMA`] times [`Self::scatter_db`], so a short
    /// render is held to a higher bar than a long one.
    ///
    /// Frequencies are interpolated by fitting a parabola through the peak bin
    /// and its two neighbours in decibels, which is what recovers a resonance to
    /// a fraction of a bin instead of to the nearest one.
    pub fn peaks(&self, count: usize, min_prominence_db: f64) -> Vec<Peak> {
        if self.frames == 0 || count == 0 {
            return Vec::new();
        }

        let bins = self.power.len();
        let first = (self.stft.resolution_floor() / self.bin_hz()).ceil() as usize;
        if first + 1 >= bins {
            return Vec::new();
        }

        let threshold = min_prominence_db.max(PROMINENCE_SIGMA * self.scatter_db());
        let curve: Vec<f64> = (0..bins).map(|k| self.bin_db(k)).collect();
        let mut found = Vec::new();

        for k in (first.max(1))..bins - 1 {
            if curve[k] <= curve[k - 1] || curve[k] < curve[k + 1] {
                continue;
            }

            // Walk out to each side until the spectrum climbs back above this
            // peak; the deepest point reached on the way is that side's valley.
            let mut left = curve[k];
            for j in (first..k).rev() {
                if curve[j] > curve[k] {
                    break;
                }
                left = left.min(curve[j]);
            }
            let mut right = curve[k];
            for &value in curve.iter().take(bins).skip(k + 1) {
                if value > curve[k] {
                    break;
                }
                right = right.min(value);
            }

            let prominence = curve[k] - left.max(right);
            if prominence < threshold {
                continue;
            }

            // Quadratic interpolation on the decibel curve. The denominator is
            // the curvature at the peak and is negative there; a flat top would
            // make it zero, and then the bin centre is the best answer there is.
            let (a, b, c) = (curve[k - 1], curve[k], curve[k + 1]);
            let curvature = a - 2.0 * b + c;
            let offset = if curvature.abs() > 1e-12 {
                (0.5 * (a - c) / curvature).clamp(-0.5, 0.5)
            } else {
                0.0
            };
            let hz = (k as f64 + offset) * self.bin_hz();

            found.push(Peak {
                hz,
                db: self.db_at(hz).unwrap_or(SILENCE_DB),
                prominence_db: prominence,
            });
        }

        found.sort_by(|a, b| b.prominence_db.total_cmp(&a.prominence_db));
        found.truncate(count);
        found.sort_by(|a, b| a.hz.total_cmp(&b.hz));
        found
    }
}

/// The `count` strongest resonances in a rendered buffer, in frequency order.
///
/// The counterpart of [`track`]: that one follows what moves with the engine,
/// this one finds what stays put while it moves.
pub fn resonances(samples: &[f32], sample_rate: f64, count: usize) -> Vec<Peak> {
    AverageSpectrum::of(samples, sample_rate).peaks(count, MIN_PROMINENCE_DB)
}

// ---------------------------------------------------------------------------
// Broadband balance
// ---------------------------------------------------------------------------

/// ISO octave-band centre frequencies a fingerprint's spectral balance is read
/// at, 31.5 Hz through 16 kHz [Hz].
pub const OCTAVE_CENTERS_HZ: [f64; 10] = [
    31.5, 63.0, 125.0, 250.0, 500.0, 1_000.0, 2_000.0, 4_000.0, 8_000.0, 16_000.0,
];

/// Energy share per octave band, each in dB relative to the render's total
/// spectral energy.
///
/// Share, not absolute level, on the same principle as an order [`Balance`]:
/// a stage that changes overall loudness must not shift these numbers, only a
/// stage that moves energy between bands may. Read off the same Welch-averaged
/// power spectrum [`AverageSpectrum`] builds for a resonance search, so a band
/// edge that falls between two bins is settled the same way a peak's placement
/// is — from the power actually captured there, not from an analytic model of
/// where an octave "should" start.
///
/// [`SILENCE_DB`] for a band with no energy at all, and for the whole table
/// when the render is shorter than one analysis window.
///
/// This is the metric `order::Balance` cannot be: a balance is read only at
/// the fixed set of frequencies the firing order predicts, so it is blind to
/// energy that moved to a frequency no order sits at. An octave band asks
/// about the whole spectrum instead, in bands wide enough that a bin's exact
/// placement cannot be gamed.
pub fn octave_bands(samples: &[f32], sample_rate: f64) -> [f64; 10] {
    let mut shares = [SILENCE_DB; 10];
    let spectrum = AverageSpectrum::of(samples, sample_rate);
    if spectrum.frames == 0 {
        return shares;
    }

    let bin = spectrum.bin_hz();
    let total: f64 = spectrum.power.iter().sum();
    if total <= 0.0 {
        return shares;
    }

    for (share, &center) in shares.iter_mut().zip(OCTAVE_CENTERS_HZ.iter()) {
        let lo = center / std::f64::consts::SQRT_2;
        let hi = center * std::f64::consts::SQRT_2;
        let first = (lo / bin).ceil() as usize;
        let last = ((hi / bin).floor() as usize).min(spectrum.power.len().saturating_sub(1));
        if first > last {
            continue;
        }
        let band: f64 = spectrum.power[first..=last].iter().sum();
        // `db` turns an amplitude ratio into 20 log10 of it; a square root
        // turns the power ratio this band actually is into that amplitude
        // ratio, so the result is 10 log10(band / total) without a second
        // logarithm helper to keep in step with the one `db` already defines.
        *share = db((band / total).sqrt());
    }
    shares
}

/// Crest factor of a whole render, `20 log10(peak / rms)` [dB].
///
/// The one figure that tracks "hollow" when RMS does not: a signal whose
/// transients have flattened can hold its RMS level while its peaks fall
/// toward it, and a balance or a tilt reading — both taken from an *averaged*
/// spectrum — cannot see that at all.
pub fn crest_db(samples: &[f32]) -> f64 {
    if samples.is_empty() {
        return SILENCE_DB;
    }
    let mut peak = 0.0f64;
    let mut sum_sq = 0.0f64;
    for &s in samples {
        let a = (s as f64).abs();
        if a > peak {
            peak = a;
        }
        sum_sq += a * a;
    }
    let rms = (sum_sq / samples.len() as f64).sqrt();
    if rms <= 0.0 {
        return SILENCE_DB;
    }
    db(peak / rms)
}

// ---------------------------------------------------------------------------
// Resonance placement
// ---------------------------------------------------------------------------

/// How far either side of a prediction a peak is still the same mode [%].
///
/// A quarter of the frequency is about a third of an octave, which is wider
/// than any error a length or a temperature can plausibly account for and
/// narrower than the gap between a pipe's own harmonics — so a peak inside it
/// is the mode that was predicted and a peak outside it is a different mode.
/// Beyond it the honest answer is that the prediction was not found at all,
/// which is a different finding from having found it in the wrong place.
pub const PLACEMENT_WINDOW_PCT: f64 = 25.0;

/// Where a predicted resonance actually landed.
///
/// The second comparison metric: a length, a volume and a temperature each
/// predict a frequency, and this is whether the audio agrees. Unlike a level
/// this is not a matter of taste or of gain — a pipe either resonates where its
/// own geometry says it does or the geometry in the model is not the geometry
/// it is being credited with.
#[derive(Debug, Clone, Copy)]
pub struct Placement {
    /// Where the geometry says the mode is [Hz].
    pub predicted_hz: f64,
    /// The peak found nearest to it, inside the search window [Hz].
    pub measured_hz: Option<f64>,
    /// That peak's level [dBFS].
    pub db: Option<f64>,
    /// How far it stood above its surroundings [dB].
    pub prominence_db: Option<f64>,
}

impl Placement {
    /// Signed placement error [%], positive where the audio sits high.
    pub fn error_pct(&self) -> Option<f64> {
        let measured = self.measured_hz?;
        (self.predicted_hz > 0.0)
            .then(|| 100.0 * (measured - self.predicted_hz) / self.predicted_hz)
    }

    /// Whether the mode was found, and within `tolerance_pct` of its prediction.
    pub fn within(&self, tolerance_pct: f64) -> bool {
        self.error_pct().is_some_and(|e| e.abs() <= tolerance_pct)
    }

    /// The length that *would* put a quarter-wave mode where this one landed
    /// [m], given the length it was predicted from.
    ///
    /// The stage's one rule, as arithmetic: a mode 8 % low is a pipe 9 % longer
    /// than the one it was predicted from, and this says what that pipe is. It
    /// is the only
    /// honest way to answer a placement error, because the alternative — an
    /// equaliser that moves the peak without moving the pipe — leaves the
    /// delay, the reflection and every harmonic above it where they were.
    pub fn implied_length(&self, predicted_from: f64) -> Option<f64> {
        let measured = self.measured_hz?;
        (measured > 0.0).then(|| predicted_from * self.predicted_hz / measured)
    }
}

/// The peak nearest `predicted_hz`, if one lies within `window_pct` of it.
pub fn place(predicted_hz: f64, peaks: &[Peak], window_pct: f64) -> Placement {
    let window = predicted_hz * window_pct / 100.0;
    let nearest = peaks
        .iter()
        .filter(|p| (p.hz - predicted_hz).abs() <= window)
        .min_by(|a, b| {
            (a.hz - predicted_hz)
                .abs()
                .total_cmp(&(b.hz - predicted_hz).abs())
        });
    Placement {
        predicted_hz,
        measured_hz: nearest.map(|p| p.hz),
        db: nearest.map(|p| p.db),
        prominence_db: nearest.map(|p| p.prominence_db),
    }
}

// ---------------------------------------------------------------------------
// A measured reference
// ---------------------------------------------------------------------------

/// Everything a calibration reads out of one buffer of audio.
///
/// The point of the type is that both sides of a comparison go through it: a
/// reference recording of a real engine and an offline render of the synth are
/// order-tracked by the same transform, averaged by the same Welch method and
/// peak-picked by the same prominence test. A difference between the two is
/// then a difference between the engines and not between two analyses.
///
/// A recording brings its own sample rate and its own speed curve. The rate is
/// whatever it was recorded at, and the curve has to be supplied from outside —
/// nothing in the audio says how fast the engine was turning, and a curve
/// guessed from the audio would make the order table a restatement of the guess.
/// [`RpmCurve::sweep`] takes the two ends of a pull, [`RpmCurve::from_points`] a
/// tachometer read off it.
pub struct Reference {
    /// The level of every order asked for.
    pub orders: OrderTable,
    /// The long-term average, for resonance placement and the floor's tilt.
    pub spectrum: AverageSpectrum,
    /// Rate the audio was analysed at [Hz].
    pub sample_rate: f64,
    /// Length of the audio [s].
    pub seconds: f64,
    /// The slowest and fastest the engine turned over it [rev/min].
    pub speed: (f64, f64),
}

impl Reference {
    /// Order-tracks and averages one buffer against a supplied speed curve.
    pub fn extract(samples: &[f32], sample_rate: f64, rpm: &RpmCurve, orders: &[f64]) -> Self {
        let seconds = samples.len() as f64 / sample_rate.max(f64::MIN_POSITIVE);
        Self {
            orders: track(samples, sample_rate, rpm, orders),
            spectrum: AverageSpectrum::of(samples, sample_rate),
            sample_rate,
            seconds,
            speed: rpm.range(0.0, seconds),
        }
    }

    /// The order balance, against the order given — the firing order, normally.
    pub fn balance(&self, reference_order: f64) -> Balance {
        self.orders.balance(reference_order)
    }

    /// The `count` most prominent resonances, in frequency order.
    pub fn peaks(&self, count: usize) -> Vec<Peak> {
        self.spectrum.peaks(count, MIN_PROMINENCE_DB)
    }

    /// Slope of the noise floor between two frequencies [dB/octave].
    pub fn tilt_db_per_octave(&self, lo_hz: f64, hi_hz: f64) -> Option<f64> {
        self.spectrum.tilt_db_per_octave(lo_hz, hi_hz)
    }
}

// ---------------------------------------------------------------------------
// The reference the crank provides on its own
// ---------------------------------------------------------------------------

/// Highest order the crank's comb is a fair reference for, as a multiple of
/// the firing order.
///
/// A firing is not a Dirac impulse: blowdown lasts a valve event, and a valve
/// event is a fixed number of crank degrees whatever the engine speed, so its
/// envelope is fixed in *order* and it rolls the comb off from somewhere above
/// the firing order. Two octaves up is where the two disagree enough to matter
/// — an exhaust event of 90 crank degrees puts its first null around order
/// eight of a four — so the comb is quoted below it and an open question is
/// recorded above it rather than a number nobody should trust.
pub const CRANK_COMB_ORDERS: f64 = 2.0;

/// Relative amplitude of each order in a train of firings at crank angles
/// `offsets` [rad].
///
/// The crank's own order spectrum: `N` impulses in a 720-degree cycle, summed
/// with the phase each one has at that order. It is the one part of an engine's
/// order balance that owes nothing to the plumbing, the fuelling or the
/// synthesis — an inline-four's even 180-degree spacing puts energy on the even
/// orders and mathematically nothing anywhere else, and a cross-plane V8's
/// 90-180-270-180 bank leaves 1.5 standing 3.7 dB under its firing order. That
/// is what makes it usable as a reference: it is derived, not recorded, and it
/// cannot be tuned.
///
/// Order `n` is cycles per crank *revolution* and the offsets are crank
/// radians, so the phase of a firing at `theta` is `n * theta`.
pub fn firing_comb(offsets: &[f64], orders: &[f64]) -> Vec<f64> {
    orders
        .iter()
        .map(|&order| {
            let (mut re, mut im) = (0.0f64, 0.0f64);
            for &theta in offsets {
                re += (order * theta).cos();
                im -= (order * theta).sin();
            }
            re.hypot(im)
        })
        .collect()
}

/// The order balance the crank alone predicts, bank by bank.
///
/// Each bank is summed on its own and the banks added in power, because that is
/// what the plumbing does with them: a vee's banks fire into separate
/// collectors and radiate out of separate tailpipes, of different lengths, at
/// different places on the car. Summing the banks *coherently* instead would
/// assume the two paths are identical, and it would then predict that a
/// cross-plane V8 has no half-order content at all — the banks' 1.5 content is
/// in antiphase and would cancel exactly. It does not cancel, because the paths
/// differ, so this is the reference and its distance from a measurement is how
/// much the two banks did cancel.
pub fn crank_balance(banks: &[Vec<f64>], orders: &[f64], reference_order: f64) -> Balance {
    let mut power = vec![0.0f64; orders.len()];
    for bank in banks {
        for (sum, amplitude) in power.iter_mut().zip(firing_comb(bank, orders)) {
            *sum += amplitude * amplitude;
        }
    }

    let levels: Vec<(f64, Option<f64>)> = orders
        .iter()
        .zip(&power)
        .map(|(&order, &power)| (order, Some(db(power.sqrt()))))
        .collect();
    Balance::from_levels(reference_order, &levels)
}

/// Places a whole set of predictions, giving each peak to at most one of them.
///
/// Nearest-first: the prediction whose peak is the smallest *fractional*
/// distance away claims it, then the next, and a peak already claimed is no
/// longer on offer. Fractional rather than absolute because a 10 Hz error on a
/// 100 Hz mode is a different thing from a 10 Hz error on a 2 kHz one.
///
/// Without the exclusion a sparse spectrum flatters itself: an engine whose
/// audio has one peak near 400 Hz will report an intake runner, an exhaust
/// primary and a silencer pass band all landing there, three modes for one
/// peak, and the arithmetic mean of their errors will look like a measurement.
/// One of them is that peak and the other two were not found.
pub fn place_all(predicted: &[f64], peaks: &[Peak], window_pct: f64) -> Vec<Placement> {
    // Every candidate pairing inside the window, by fractional distance.
    let mut candidates: Vec<(f64, usize, usize)> = Vec::new();
    for (i, &hz) in predicted.iter().enumerate() {
        if !hz.is_finite() || hz <= 0.0 {
            continue;
        }
        for (j, peak) in peaks.iter().enumerate() {
            let error = (peak.hz - hz).abs() / hz;
            if error * 100.0 <= window_pct {
                candidates.push((error, i, j));
            }
        }
    }
    candidates.sort_by(|a, b| a.0.total_cmp(&b.0));

    let mut taken = vec![false; peaks.len()];
    let mut placements: Vec<Placement> = predicted
        .iter()
        .map(|&hz| Placement {
            predicted_hz: hz,
            measured_hz: None,
            db: None,
            prominence_db: None,
        })
        .collect();
    for (_, i, j) in candidates {
        if taken[j] || placements[i].measured_hz.is_some() {
            continue;
        }
        taken[j] = true;
        placements[i] = Placement {
            predicted_hz: predicted[i],
            measured_hz: Some(peaks[j].hz),
            db: Some(peaks[j].db),
            prominence_db: Some(peaks[j].prominence_db),
        };
    }
    placements
}

#[cfg(test)]
mod tests;
