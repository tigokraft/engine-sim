//! Audio-rate primitives for the exhaust, intake and turbo signal chains.
//!
//! Everything here is written for a real-time callback: no allocation after
//! construction, no locks, no branches that depend on how loud the signal is.
//! Buffers are sized once at build time and reused forever.
//!
//! Two conventions differ from the physics crate on purpose:
//!
//! - Samples and coefficients are `f32`, because that is what the output stream
//!   consumes and what keeps the delay lines in cache. State that integrates
//!   over long spans (crank phase) stays normalised to `[0, 1)` so the loss of
//!   mantissa against `f64` never accumulates.
//! - Frequencies are in Hz and delays in *samples*, since the sample rate is
//!   fixed for the life of the stream and converting once at the control rate is
//!   cheaper than converting per sample.
//!
//! The gas-dynamic helpers at the bottom ([`speed_of_sound`],
//! [`runner_delay_seconds`], [`helmholtz_frequency`]) are the bridge back to SI:
//! they take Kelvin and metres and hand back the seconds and Hz the filters
//! want.

use std::f32::consts::TAU;

use crate::audio::radiation::Mouth;

/// Anything below this magnitude is flushed to zero.
///
/// Recursive filters fed silence decay toward zero geometrically and spend a
/// long time in the denormal range on the way. Denormal arithmetic traps to
/// microcode on x86 and costs one to two orders of magnitude more per operation
/// than a normal multiply — enough, with a dozen filters running, to blow the
/// callback's deadline and produce the exact dropout this engine is supposed to
/// avoid. Flushing costs one compare.
const DENORMAL_FLOOR: f32 = 1e-25;

#[inline(always)]
fn flush(value: f32) -> f32 {
    if value.abs() < DENORMAL_FLOOR {
        0.0
    } else {
        value
    }
}

// ---------------------------------------------------------------------------
// Biquad
// ---------------------------------------------------------------------------

/// Normalised biquad coefficients (`a0` divided out).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BiquadCoeffs {
    pub b0: f32,
    pub b1: f32,
    pub b2: f32,
    pub a1: f32,
    pub a2: f32,
}

impl Default for BiquadCoeffs {
    /// Unity passthrough.
    fn default() -> Self {
        Self {
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
            a1: 0.0,
            a2: 0.0,
        }
    }
}

impl BiquadCoeffs {
    /// Shared prelude for the RBJ cookbook forms: `(cos w0, alpha)`.
    ///
    /// The centre frequency is clamped below Nyquist because the design
    /// equations are only valid there — a resonator asked for 30 kHz at a 48 kHz
    /// sample rate does not alias, it goes unstable and latches to infinity.
    fn prelude(sample_rate: f32, frequency: f32, q: f32) -> (f32, f32, f32) {
        let nyquist = 0.5 * sample_rate;
        let f0 = frequency.clamp(5.0, 0.45 * sample_rate).min(nyquist * 0.99);
        let q = q.clamp(0.05, 40.0);
        let w0 = TAU * f0 / sample_rate;
        let (sin_w0, cos_w0) = w0.sin_cos();
        (cos_w0, sin_w0 / (2.0 * q), w0)
    }

    /// Constant-peak-gain bandpass: 0 dB at `frequency`, -inf at DC and Nyquist.
    ///
    /// This is the muffler cavity form. The peak gain is independent of `q`, so
    /// re-tuning the resonance as the exhaust heats up changes the colour
    /// without changing the loudness.
    pub fn bandpass(sample_rate: f32, frequency: f32, q: f32) -> Self {
        let (cos_w0, alpha, _) = Self::prelude(sample_rate, frequency, q);
        let a0 = 1.0 + alpha;
        Self {
            b0: alpha / a0,
            b1: 0.0,
            b2: -alpha / a0,
            a1: -2.0 * cos_w0 / a0,
            a2: (1.0 - alpha) / a0,
        }
    }

    /// Second-order lowpass.
    pub fn lowpass(sample_rate: f32, frequency: f32, q: f32) -> Self {
        let (cos_w0, alpha, _) = Self::prelude(sample_rate, frequency, q);
        let a0 = 1.0 + alpha;
        let b = (1.0 - cos_w0) / 2.0;
        Self {
            b0: b / a0,
            b1: (1.0 - cos_w0) / a0,
            b2: b / a0,
            a1: -2.0 * cos_w0 / a0,
            a2: (1.0 - alpha) / a0,
        }
    }

    /// Second-order highpass.
    pub fn highpass(sample_rate: f32, frequency: f32, q: f32) -> Self {
        let (cos_w0, alpha, _) = Self::prelude(sample_rate, frequency, q);
        let a0 = 1.0 + alpha;
        let b = (1.0 + cos_w0) / 2.0;
        Self {
            b0: b / a0,
            b1: -(1.0 + cos_w0) / a0,
            b2: b / a0,
            a1: -2.0 * cos_w0 / a0,
            a2: (1.0 - alpha) / a0,
        }
    }

    /// Resonant peak of `gain_db` at `frequency`, unity elsewhere.
    pub fn peaking(sample_rate: f32, frequency: f32, q: f32, gain_db: f32) -> Self {
        let (cos_w0, alpha, _) = Self::prelude(sample_rate, frequency, q);
        let a = 10f32.powf(gain_db / 40.0);
        let a0 = 1.0 + alpha / a;
        Self {
            b0: (1.0 + alpha * a) / a0,
            b1: -2.0 * cos_w0 / a0,
            b2: (1.0 - alpha * a) / a0,
            a1: -2.0 * cos_w0 / a0,
            a2: (1.0 - alpha / a) / a0,
        }
    }
}

/// A biquad section in transposed direct form II.
///
/// TDF-II is the right choice for coefficients that move: its two state
/// variables hold a filtered version of the signal rather than raw input
/// history, so swapping coefficients mid-stream produces a smooth timbral shift
/// instead of the step discontinuity a direct-form-I section would emit. That is
/// what makes [`Biquad::set_coeffs`] safe to call at the control rate while
/// audio is flowing.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Biquad {
    coeffs: BiquadCoeffs,
    z1: f32,
    z2: f32,
}

impl Biquad {
    /// A section with the given response and cleared state.
    pub fn new(coeffs: BiquadCoeffs) -> Self {
        Self {
            coeffs,
            z1: 0.0,
            z2: 0.0,
        }
    }

    /// Retunes the section *without* touching its state, so no click.
    pub fn set_coeffs(&mut self, coeffs: BiquadCoeffs) {
        self.coeffs = coeffs;
    }

    /// Clears the delay elements.
    pub fn reset(&mut self) {
        self.z1 = 0.0;
        self.z2 = 0.0;
    }

    /// Filters one sample.
    #[inline(always)]
    pub fn process(&mut self, x: f32) -> f32 {
        let c = &self.coeffs;
        let y = c.b0 * x + self.z1;
        self.z1 = flush(c.b1 * x - c.a1 * y + self.z2);
        self.z2 = flush(c.b2 * x - c.a2 * y);
        y
    }
}

// ---------------------------------------------------------------------------
// One-pole helpers
// ---------------------------------------------------------------------------

/// First-order lowpass, `y += (x - y) * a`.
///
/// Used where a biquad would be overkill: tailpipe radiation loss, the
/// frequency-dependent loss inside a runner's reflection loop, and smoothing the
/// envelope of the turbo flutter.
#[derive(Debug, Clone, Copy, Default)]
pub struct OnePole {
    a: f32,
    z: f32,
}

impl OnePole {
    /// A lowpass with the given -3 dB cutoff [Hz].
    pub fn new(sample_rate: f32, cutoff: f32) -> Self {
        let mut filter = Self { a: 0.0, z: 0.0 };
        filter.set_cutoff(sample_rate, cutoff);
        filter
    }

    /// Retunes the cutoff [Hz], keeping state.
    pub fn set_cutoff(&mut self, sample_rate: f32, cutoff: f32) {
        let f = cutoff.clamp(1.0, 0.45 * sample_rate);
        // Exact pole placement rather than the usual `1 - exp` approximation,
        // so the response stays sane when the cutoff walks up near Nyquist.
        self.a = 1.0 - (-TAU * f / sample_rate).exp();
    }

    /// Filters one sample.
    #[inline(always)]
    pub fn process(&mut self, x: f32) -> f32 {
        self.z = flush(self.z + (x - self.z) * self.a);
        self.z
    }

    /// Current output without advancing.
    pub fn value(&self) -> f32 {
        self.z
    }

    /// The smoothing coefficient `a` currently in effect [-].
    ///
    /// The pole sits at `1 - a`. Anything building a shelf out of this filter
    /// needs it to place the shelf's zero.
    pub fn coefficient(&self) -> f32 {
        self.a
    }

    /// Delay the filter adds to a wave passing through it [samples].
    ///
    /// A one-pole is minimum phase, not zero phase: below its cutoff it lags by
    ///
    /// ```text
    /// tau = (1 - a) / a
    /// ```
    ///
    /// samples. That lag is real and it is small, but a filter sitting inside a
    /// resonant loop does not merely attenuate — it lengthens the loop, and the
    /// pipe comes out flat. Anything tuning a delay line around one of these has
    /// to subtract this.
    pub fn phase_delay_samples(&self) -> f32 {
        (1.0 - self.a) / self.a.max(1e-6)
    }

    /// Clears state.
    pub fn reset(&mut self) {
        self.z = 0.0;
    }
}

// ---------------------------------------------------------------------------
// First-order allpass
// ---------------------------------------------------------------------------

/// First-order allpass, `y[n] = a*x[n] + x[n-1] - a*y[n-1]`.
///
/// Unity magnitude at every frequency by construction — only phase moves,
/// turning over a full cycle around `break_hz`. That makes it the right
/// primitive for smearing a step-like edge in time without touching level or
/// spectrum: a lowpass would dull the edge and quiet it, this only changes
/// *when* each frequency in it arrives. Cascading several stages at
/// different break frequencies spreads that smear across the band instead of
/// concentrating it in one narrow region — see
/// [`crate::audio::waveguide::Turbine`], where a chain of these stands in for
/// the many differing path lengths through a turbine wheel's blade passages.
#[derive(Debug, Clone, Copy, Default)]
pub struct Allpass {
    a: f32,
    x1: f32,
    y1: f32,
}

impl Allpass {
    /// An allpass whose phase turnover centres on `break_hz` [Hz].
    pub fn new(sample_rate: f32, break_hz: f32) -> Self {
        let mut filter = Self::default();
        filter.set_break_frequency(sample_rate, break_hz);
        filter
    }

    /// Retunes the break frequency [Hz], keeping state.
    pub fn set_break_frequency(&mut self, sample_rate: f32, break_hz: f32) {
        let f = break_hz.clamp(1.0, 0.45 * sample_rate);
        let t = (std::f32::consts::PI * f / sample_rate).tan();
        self.a = (t - 1.0) / (t + 1.0);
    }

    /// Filters one sample.
    #[inline(always)]
    pub fn process(&mut self, x: f32) -> f32 {
        let y = flush(self.a * x + self.x1 - self.a * self.y1);
        self.x1 = x;
        self.y1 = y;
        y
    }

    /// Clears state.
    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.y1 = 0.0;
    }
}

/// DC blocker, `y[n] = x[n] - x[n-1] + R * y[n-1]`.
///
/// The blowdown excitation is a one-sided pressure pulse, so its mean is not
/// zero. Left in, that offset walks the master bus toward the clipper and eats
/// headroom that the pops need. `R = 0.9995` puts the corner near 4 Hz at
/// 48 kHz — well below anything the engine produces.
#[derive(Debug, Clone, Copy)]
pub struct DcBlocker {
    r: f32,
    x1: f32,
    y1: f32,
}

impl Default for DcBlocker {
    fn default() -> Self {
        Self {
            r: 0.9995,
            x1: 0.0,
            y1: 0.0,
        }
    }
}

impl DcBlocker {
    /// Filters one sample.
    #[inline(always)]
    pub fn process(&mut self, x: f32) -> f32 {
        let y = x - self.x1 + self.r * self.y1;
        self.x1 = x;
        self.y1 = flush(y);
        y
    }

    /// Clears state.
    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.y1 = 0.0;
    }
}

// ---------------------------------------------------------------------------
// Parameter smoothing
// ---------------------------------------------------------------------------

/// An exponentially glided parameter.
///
/// Physics snapshots arrive at the frame rate — a few hundred Hz at best, and
/// with no guarantee of even spacing. Feeding those steps straight into a gain
/// or a delay length is audible as zipper noise, and in the delay's case as a
/// hard click. Every control value the synth consumes goes through one of these
/// first, so the audio thread only ever sees a continuous signal.
#[derive(Debug, Clone, Copy)]
pub struct Smoothed {
    value: f32,
    target: f32,
    alpha: f32,
}

impl Smoothed {
    /// A parameter starting at `initial` with the given time constant [s].
    pub fn new(initial: f32, sample_rate: f32, time_constant: f32) -> Self {
        Self {
            value: initial,
            target: initial,
            alpha: 1.0 - (-1.0 / (time_constant.max(1e-5) * sample_rate)).exp(),
        }
    }

    /// Sets where the parameter is heading.
    #[inline(always)]
    pub fn set_target(&mut self, target: f32) {
        if target.is_finite() {
            self.target = target;
        }
    }

    /// Jumps straight to a value, skipping the glide. Construction-time only.
    pub fn snap(&mut self, value: f32) {
        self.value = value;
        self.target = value;
    }

    /// Advances one sample and returns the new value.
    #[inline(always)]
    pub fn next_value(&mut self) -> f32 {
        self.value += (self.target - self.value) * self.alpha;
        self.value
    }

    /// Current value without advancing.
    #[inline(always)]
    pub fn value(&self) -> f32 {
        self.value
    }

    /// Target value the smoother is gliding towards.
    #[inline(always)]
    pub fn target(&self) -> f32 {
        self.target
    }

    /// Advances by `n` samples at once, for control-rate use.
    #[inline]
    pub fn advance(&mut self, n: usize) -> f32 {
        for _ in 0..n {
            self.next_value();
        }
        self.value
    }
}

// ---------------------------------------------------------------------------
// Noise
// ---------------------------------------------------------------------------

/// Xorshift32 white-noise source.
///
/// Deterministic, allocation-free, and about four instructions per sample. The
/// engine needs a *lot* of noise — induction turbulence, the broadband half of
/// every blowdown pulse, compressor surge — so this runs on the hot path and is
/// deliberately not a cryptographic or high-quality generator.
#[derive(Debug, Clone, Copy)]
pub struct Noise {
    state: u32,
}

impl Default for Noise {
    fn default() -> Self {
        Self::new(0x2545_F491)
    }
}

impl Noise {
    /// A generator with the given non-zero seed.
    pub fn new(seed: u32) -> Self {
        Self {
            // Zero is the xorshift fixed point and would emit silence forever.
            state: if seed == 0 { 0x2545_F491 } else { seed },
        }
    }

    /// Next raw word.
    #[inline(always)]
    pub fn next_u32(&mut self) -> u32 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.state = x;
        x
    }

    /// Uniform white noise in `[-1, 1)`.
    #[inline(always)]
    pub fn next_bipolar(&mut self) -> f32 {
        // Take the top 24 bits: the low bits of xorshift are the weakest.
        let bits = self.next_u32() >> 8;
        (bits as f32) * (2.0 / 16_777_216.0) - 1.0
    }

    /// Uniform in `[0, 1)`.
    #[inline(always)]
    pub fn next_unit(&mut self) -> f32 {
        let bits = self.next_u32() >> 8;
        (bits as f32) * (1.0 / 16_777_216.0)
    }

    /// Approximately standard normal: zero mean, unit variance.
    ///
    /// Irwin-Hall with four terms rather than Box-Muller. Three properties
    /// matter here and none of them are distributional accuracy:
    ///
    /// - **Constant cost.** The polar form of Box-Muller rejects about 21 % of
    ///   its draws, so its worst case is unbounded. This is four xorshifts and
    ///   two arithmetic operations, every time.
    /// - **No transcendentals.** `ln` and `sqrt` are the expensive part of a
    ///   proper normal draw, and this is called from the firing path.
    /// - **Bounded support.** The sum of four uniforms cannot leave
    ///   `[-sqrt(12), +sqrt(12)]`, i.e. `+/-3.46 sigma`. For cycle-to-cycle
    ///   combustion variation that truncation is a feature: a true Gaussian has
    ///   a tail that would occasionally ask for a negative-amplitude pulse or a
    ///   firing offset past the next cylinder's, and neither is a thing an
    ///   engine does.
    ///
    /// The fourth moment is off — kurtosis is 2.4 against a normal's 3 — which
    /// is to say the tails are slightly light. Inaudible at the depths used.
    #[inline]
    pub fn next_gaussian(&mut self) -> f32 {
        // Var(U) = 1/12, so Var(sum of 4) = 1/3 and the scale is sqrt(3).
        let sum = self.next_unit() + self.next_unit() + self.next_unit() + self.next_unit();
        (sum - 2.0) * 1.732_050_8
    }
}

// ---------------------------------------------------------------------------
// Delay line
// ---------------------------------------------------------------------------

/// Power-of-two fractional delay line with first-order Thiran allpass interpolation.
///
/// The capacity is rounded up to a power of two so the wrap is a mask rather
/// than a modulo or a branch, which matters because the exhaust chain reads one
/// of these per bank per sample.
///
/// Thiran allpass fractional interpolation provides flat unity magnitude response
/// (|H| = 1.0) at all frequencies, eliminating the moving lowpass filtering and
/// breathing amplitude modulation associated with linear interpolation during
/// temperature glides.
#[derive(Debug, Clone)]
pub struct DelayLine {
    buffer: Vec<f32>,
    mask: usize,
    write: usize,
    allpass_x: f32,
    allpass_y: f32,
}

impl DelayLine {
    /// A line able to hold at least `max_delay` samples.
    pub fn with_max_delay(max_delay: usize) -> Self {
        let capacity = (max_delay + 4).next_power_of_two().max(8);
        Self {
            buffer: vec![0.0; capacity],
            mask: capacity - 1,
            write: 0,
            allpass_x: 0.0,
            allpass_y: 0.0,
        }
    }

    /// Longest delay that can be read back [samples].
    pub fn max_delay(&self) -> f32 {
        (self.buffer.len() - 2) as f32
    }

    /// Clears the line.
    pub fn reset(&mut self) {
        self.buffer.iter_mut().for_each(|s| *s = 0.0);
        self.write = 0;
        self.allpass_x = 0.0;
        self.allpass_y = 0.0;
    }

    /// Writes one sample at the head.
    #[inline(always)]
    pub fn push(&mut self, x: f32) {
        self.buffer[self.write] = flush(x);
        self.write = (self.write + 1) & self.mask;
    }

    /// Reads `delay` samples back from the head using linear interpolation (stateless).
    #[inline(always)]
    pub fn read_linear(&self, delay: f32) -> f32 {
        let d = delay.clamp(1.0, self.max_delay());
        let whole = d as usize;
        let frac = d - whole as f32;
        let len = self.buffer.len();
        let near = self.buffer[(self.write + len - whole) & self.mask];
        let far = self.buffer[(self.write + len - whole - 1) & self.mask];
        near + (far - near) * frac
    }

    /// Reads `delay` samples back from the head using 3rd-order Lagrange interpolation
    /// (stateless).
    ///
    /// The allpass read in [`DelayLine::read`] carries a filter state that is
    /// only meaningful while the delay glides, so it cannot be used by a reader
    /// whose position jumps — and a wave that has shocked has a read position
    /// that jumps, because that is what a shock is. Lagrange-3 has no memory to
    /// invalidate and is flat enough over the band that the jump costs no top.
    #[inline(always)]
    pub fn read_lagrange3(&self, delay: f32) -> f32 {
        let d = delay.clamp(2.0, self.max_delay() - 2.0);
        let n = d as usize;
        let t = d - n as f32;
        let len = self.buffer.len();
        let tap = |j: usize| self.buffer[(self.write + len - j) & self.mask];
        let s_m1 = tap(n - 1);
        let s_0 = tap(n);
        let s_1 = tap(n + 1);
        let s_2 = tap(n + 2);
        let l_m1 = -t * (t - 1.0) * (t - 2.0) / 6.0;
        let l_0 = (t + 1.0) * (t - 1.0) * (t - 2.0) / 2.0;
        let l_1 = -t * (t + 1.0) * (t - 2.0) / 2.0;
        let l_2 = t * (t + 1.0) * (t - 1.0) / 6.0;
        l_m1 * s_m1 + l_0 * s_0 + l_1 * s_1 + l_2 * s_2
    }

    /// Reads `delay` samples back from the head using 1st-order Thiran allpass interpolation.
    ///
    /// Preserves full high-frequency energy with constant unity magnitude response
    /// as delay moves continuously.
    #[inline(always)]
    pub fn read(&mut self, delay: f32) -> f32 {
        let d = delay.clamp(1.0, self.max_delay());
        if d < 1.5 {
            return self.read_linear(d);
        }
        let n_int = (d - 0.5).floor() as usize;
        let delta = d - n_int as f32;
        let a = (1.0 - delta) / (1.0 + delta);
        let len = self.buffer.len();
        let x = self.buffer[(self.write + len - n_int) & self.mask];
        let y = a * x + self.allpass_x - a * self.allpass_y;
        self.allpass_x = flush(x);
        self.allpass_y = flush(y);
        y
    }
}

// ---------------------------------------------------------------------------
// Gas dynamics
// ---------------------------------------------------------------------------

/// Speed of sound in an ideal gas, `c = sqrt(gamma * R * T)` [m/s].
///
/// This is the single quantity that ties the exhaust's *pitch* to its
/// *temperature*: a cold engine has a slower runner and a lower cavity
/// resonance, and both rise together as it warms up, because both are this
/// number divided by a fixed length.
#[inline]
pub fn speed_of_sound(gamma: f32, gas_constant: f32, temperature: f32) -> f32 {
    (gamma.max(1.01) * gas_constant.max(1.0) * temperature.max(1.0)).sqrt()
}

/// Acoustic transit time down an exhaust runner [s].
///
/// ```text
/// tau_delay = L_runner / sqrt(gamma * R * T_exhaust)
/// ```
#[inline]
pub fn runner_delay_seconds(
    runner_length: f32,
    gamma: f32,
    gas_constant: f32,
    temperature: f32,
) -> f32 {
    runner_length.max(0.0) / speed_of_sound(gamma, gas_constant, temperature)
}

/// Acoustic round-trip time of a runner, valve to collector and back [s].
///
/// This is the period of the pipe's own quarter-wave loop, and the natural
/// yardstick for asking whether a delay line is behaving as a resonator or as a
/// comb filter.
#[inline]
pub fn round_trip_seconds(
    runner_length: f32,
    gamma: f32,
    gas_constant: f32,
    temperature: f32,
) -> f32 {
    2.0 * runner_delay_seconds(runner_length, gamma, gas_constant, temperature)
}

/// Interval between successive firings into one exhaust bank [s].
///
/// ```text
/// tau_interval = (120 / RPM) / N_cylinders_in_bank
/// ```
///
/// The `120` is the four-stroke cycle: two revolutions per firing of any given
/// cylinder. A stopped engine returns [`f32::MAX`] rather than infinity, so the
/// ratios built from it stay finite.
#[inline]
pub fn firing_interval_seconds(rpm: f32, cylinders_in_bank: usize) -> f32 {
    // NaN included: a speed that is not a number never fires again either.
    if !rpm.is_finite() || rpm <= 1.0 {
        return f32::MAX;
    }
    let cycle_seconds = 120.0 / rpm;
    cycle_seconds / cylinders_in_bank.max(1) as f32
}

/// Helmholtz resonance of a muffler cavity [Hz].
///
/// ```text
/// f_muffler = (sqrt(gamma * R * T_exhaust) / 2*pi) * sqrt(A_neck / (V_chamber * L_neck))
/// ```
///
/// The neck length is *not* corrected for the end effect here. A real neck
/// radiates into the chamber and behaves acoustically longer than it is, by
/// roughly `0.6 * neck_radius` per open end; callers that want that should hand
/// in the corrected length, since the correction depends on how the neck is
/// flanged and this function has no way to know.
#[inline]
pub fn helmholtz_frequency(
    gamma: f32,
    gas_constant: f32,
    temperature: f32,
    neck_area: f32,
    chamber_volume: f32,
    neck_length: f32,
) -> f32 {
    let c = speed_of_sound(gamma, gas_constant, temperature);
    let geometry = neck_area.max(1e-9) / (chamber_volume.max(1e-9) * neck_length.max(1e-6));
    (c / TAU) * geometry.sqrt()
}

// ---------------------------------------------------------------------------
// Composite exhaust elements
// ---------------------------------------------------------------------------

pub use crate::physics::plumbing::MufflerGeometry;

/// The muffler: a Helmholtz bandpass in parallel with a through path, and the
/// tailpipe mouth both of them leave by.
///
/// A pure bandpass is what the cavity does, but it is not what a muffler
/// *sounds* like — all of the transient energy that gives an exhaust note its
/// edge lives outside the resonance. Real mufflers pass that energy through, so
/// what arrives at the tailpipe is a mix: `resonant_mix` of the cavity, the rest
/// straight through.
///
/// What leaves the tailpipe is then not that mixture but its radiated field.
/// The old model lowpassed here, which is the tilt backwards: inside the pipe
/// the low end dominates, outside it is the part that never got out. See
/// [`Mouth`] — below the mouth corner the radiated field rises at 6 dB/octave,
/// above it, it is flat.
///
/// The mouth's reflected wave is dropped rather than fed back, because there is
/// nothing here to feed it into: this muffler is a pair of lumped filters, not
/// a waveguide with a `p^-` to carry it. In the pipe network the same
/// termination loads the tailpipe properly.
#[derive(Debug, Clone)]
pub struct Muffler {
    geometry: MufflerGeometry,
    cavity: Biquad,
    mouth: Mouth,
    sample_rate: f32,
    centre_hz: f32,
}

impl Muffler {
    /// Builds a muffler discharging through a tailpipe of `tailpipe_radius`
    /// metres, and tunes it for `temperature` [K].
    ///
    /// `flanged` is whether the tailpipe exits through a panel or hangs free;
    /// it changes the end correction, not the radiation.
    pub fn new(
        sample_rate: f32,
        geometry: MufflerGeometry,
        tailpipe_radius: f32,
        flanged: bool,
        gamma: f32,
        gas_constant: f32,
        temperature: f32,
    ) -> Self {
        let mut muffler = Self {
            geometry,
            cavity: Biquad::default(),
            mouth: Mouth::new(
                sample_rate,
                tailpipe_radius,
                flanged,
                speed_of_sound(gamma, gas_constant, temperature),
            ),
            sample_rate,
            centre_hz: 0.0,
        };
        muffler.tune(gamma, gas_constant, temperature);
        muffler
    }

    /// Retunes the cavity and the mouth for the current exhaust temperature
    /// [K].
    ///
    /// Called at the control rate, not per sample: the transcendentals in the
    /// coefficient design are the expensive part of this whole module, and the
    /// temperature that drives them is already smoothed.
    pub fn tune(&mut self, gamma: f32, gas_constant: f32, temperature: f32) {
        self.mouth
            .tune(speed_of_sound(gamma, gas_constant, temperature));
        self.centre_hz = helmholtz_frequency(
            gamma,
            gas_constant,
            temperature,
            self.geometry.neck_area as f32,
            self.geometry.chamber_volume as f32,
            self.geometry.neck_length as f32,
        );
        self.cavity.set_coeffs(BiquadCoeffs::bandpass(
            self.sample_rate,
            self.centre_hz,
            self.geometry.q as f32,
        ));
    }

    /// Current cavity resonance [Hz].
    pub fn centre_frequency(&self) -> f32 {
        self.centre_hz
    }

    /// Frequency above which the tailpipe radiates rather than reflects [Hz].
    pub fn mouth_corner_hz(&self) -> f32 {
        self.mouth.corner_hz()
    }

    /// Radiates one sample: in at the muffler inlet, out at the tailpipe mouth.
    #[inline(always)]
    pub fn process(&mut self, x: f32) -> f32 {
        let mix = self.geometry.resonant_mix as f32;
        let resonant = self.cavity.process(x);
        let at_tailpipe = x * (1.0 - mix) + resonant * mix;
        let (_reflected, radiated) = self.mouth.step(at_tailpipe);
        radiated
    }

    /// Clears state.
    pub fn reset(&mut self) {
        self.cavity.reset();
        self.mouth.reset();
    }
}

/// An exhaust runner: a transit delay closed into a reflecting pipe.
///
/// The requirement is the transit time `tau = L / c`, but a bare delay is
/// inaudible — it shifts the pulse in time and changes nothing else. What makes
/// a header sound like a header is that the pulse reaches the collector, finds
/// an area discontinuity, and sends part of itself back to the valve, where it
/// reflects again. So the delay is wrapped in a loop:
///
/// ```text
/// y[n]      = line.read(tau)
/// line.push(excitation[n] - reflection * lowpass(y[n]))
/// ```
///
/// The end that closes the loop is a real mouth — [`Mouth`], from
/// [`radiation`](crate::audio::radiation) — rather than a bare sign flip. It
/// carries the pressure-release inversion, which is what puts the pipe's
/// fundamental at a *quarter* wave rather than a half; it carries the
/// frequency-dependent `|R|`, so the top of each echo leaves through the
/// collector instead of coming back; and it lengthens the pipe by its end
/// correction, so the fundamental sits at `c / (4 (L + delta))`. The lowpass in
/// the return path is separately wall and viscous loss, and between the two of
/// them the loop is a contraction at every frequency, so the resonance cannot
/// run away no matter how the temperature modulates the delay.
#[derive(Debug, Clone)]
pub struct ExhaustRunner {
    line: DelayLine,
    mouth: Mouth,
    loss: OnePole,
    delay_samples: Smoothed,
    /// Collector reflection the geometry alone implies, before damping.
    nominal_reflection: f32,
    /// Reflection actually in the loop, glided so a damping change never steps.
    reflection: Smoothed,
    /// Current damping, `0` for the bare pipe and `1` for fully damped [-].
    damping: f32,
    /// Physical length plus the mouth's end correction [m].
    acoustic_length: f32,
    sample_rate: f32,
}

/// Return-path loss cutoff of an undamped runner [Hz].
pub const RUNNER_LOSS_CUTOFF_HZ: f32 = 2_400.0;

/// Return-path loss cutoff of a fully damped runner [Hz].
///
/// Damping is not just a smaller reflection coefficient. A pipe that is
/// swallowing its own echoes swallows the top of them first — wall and viscous
/// loss both rise with frequency — so the damped runner is darker as well as
/// quieter, which is what keeps the idle from merely going soft while staying
/// just as metallic.
pub const RUNNER_DAMPED_CUTOFF_HZ: f32 = 700.0;

/// Fraction of the nominal reflection that survives full damping [-].
///
/// Not zero: even at idle the collector is a real area discontinuity and does
/// send something back. It is small enough that the loop's ringdown falls below
/// one firing interval, which is what stops the comb from forming.
pub const RUNNER_DAMPED_REFLECTION: f32 = 0.35;

impl ExhaustRunner {
    /// A runner of `length` metres and internal `radius` metres, tuned for
    /// `temperature` [K].
    ///
    /// `reflection` is the magnitude of the collector reflection coefficient and
    /// is clamped below unity; at 1.0 the loop would be lossless at DC and the
    /// pipe would never stop ringing.
    ///
    /// The radius is not decoration: it sets both the end correction that
    /// lengthens the pipe and the corner above which the mouth stops sending
    /// echoes back. A wide primary is flatter and darker than a narrow one of
    /// the same length.
    pub fn new(
        sample_rate: f32,
        length: f32,
        radius: f32,
        reflection: f32,
        gamma: f32,
        gas_constant: f32,
        temperature: f32,
    ) -> Self {
        // The collector end is taken unflanged: a primary running into a merge
        // cone is a pipe stopping in open space, not one cut into a wall.
        let mouth = Mouth::new(
            sample_rate,
            radius,
            false,
            speed_of_sound(gamma, gas_constant, temperature),
        );
        let acoustic_length = length + mouth.end_correction();
        // Size for the coldest gas the runner will ever see. Sound travels
        // slowest when cold, so that is the longest the delay can get; 250 K is
        // comfortably below any exhaust that has ever fired.
        let longest =
            runner_delay_seconds(acoustic_length, gamma, gas_constant, 250.0) * sample_rate;
        let initial =
            runner_delay_seconds(acoustic_length, gamma, gas_constant, temperature) * sample_rate;
        let nominal_reflection = reflection.clamp(0.0, 0.92);
        Self {
            line: DelayLine::with_max_delay(longest.ceil() as usize + 8),
            mouth,
            loss: OnePole::new(sample_rate, RUNNER_LOSS_CUTOFF_HZ),
            // 40 ms of glide: fast enough to track a throttle transient, slow
            // enough that the interpolator never audibly pitches.
            delay_samples: Smoothed::new(initial.max(1.0), sample_rate, 0.040),
            nominal_reflection,
            // 60 ms: damping tracks engine speed, which is the slowest control
            // the runner has, and a reflection coefficient that moved in steps
            // would push a discontinuity straight into the feedback path.
            reflection: Smoothed::new(nominal_reflection, sample_rate, 0.060),
            damping: 0.0,
            acoustic_length,
            sample_rate,
        }
    }

    /// Retunes the transit delay and the mouth for the current exhaust
    /// temperature [K].
    pub fn tune(&mut self, gamma: f32, gas_constant: f32, temperature: f32) {
        let samples = runner_delay_seconds(self.acoustic_length, gamma, gas_constant, temperature)
            * self.sample_rate;
        self.delay_samples
            .set_target(samples.clamp(1.0, self.line.max_delay()));
        self.mouth
            .tune(speed_of_sound(gamma, gas_constant, temperature));
    }

    /// Length the pipe resonates as, physical plus end correction [m].
    pub fn acoustic_length(&self) -> f32 {
        self.acoustic_length
    }

    /// Transit delay currently in use [samples].
    pub fn delay_samples(&self) -> f32 {
        self.delay_samples.value()
    }

    /// Acoustic round-trip time of the loop as currently tuned [s].
    ///
    /// This is the `tau_pulse` of the Transit-Time Decision Rule, read back from
    /// the delay the runner is actually using rather than recomputed from
    /// temperature — so it follows the glide instead of leading it.
    pub fn round_trip_seconds(&self) -> f32 {
        2.0 * self.delay_samples.value() / self.sample_rate
    }

    /// Scales the feedback loop down, `0` for the bare pipe and `1` for fully
    /// damped.
    ///
    /// Drive this at the control rate. Both the
    /// reflection magnitude and the return path's cutoff move together; see
    /// [`RUNNER_DAMPED_REFLECTION`] and [`RUNNER_DAMPED_CUTOFF_HZ`] for why the
    /// second one is not optional.
    ///
    /// The reflection is glided rather than assigned, so this is safe to call
    /// while audio is flowing. The loss cutoff is assigned directly, which is
    /// also click-free: a one-pole's state *is* its output, so changing the
    /// coefficient changes how fast it moves, never where it is.
    pub fn set_damping(&mut self, damping: f32) {
        let d = damping.clamp(0.0, 1.0);
        self.damping = d;
        self.reflection
            .set_target(self.nominal_reflection * (1.0 - (1.0 - RUNNER_DAMPED_REFLECTION) * d));
        self.loss.set_cutoff(
            self.sample_rate,
            RUNNER_LOSS_CUTOFF_HZ + (RUNNER_DAMPED_CUTOFF_HZ - RUNNER_LOSS_CUTOFF_HZ) * d,
        );
    }

    /// Damping currently applied, `0..=1` [-].
    pub fn damping(&self) -> f32 {
        self.damping
    }

    /// Reflection magnitude currently in the loop [-].
    pub fn reflection(&self) -> f32 {
        self.reflection.value()
    }

    /// Pipe fundamental, `c / 4L` expressed from the current delay [Hz].
    pub fn fundamental(&self) -> f32 {
        self.sample_rate / (4.0 * self.delay_samples.value().max(1.0))
    }

    /// Injects one sample of valve-end excitation and returns the collector end.
    #[inline(always)]
    pub fn process(&mut self, excitation: f32) -> f32 {
        let delay = self.delay_samples.next_value();
        let reflection = self.reflection.next_value();
        let out = self.line.read(delay);
        // The mouth carries the inversion, so what comes back is *added*: the
        // sign that used to sit in this expression is now the physics of a
        // pressure-release end rather than a minus somebody typed.
        let returned = self.loss.process(self.mouth.reflect(out));
        self.line.push(excitation + reflection * returned);
        out
    }

    /// Clears state.
    pub fn reset(&mut self) {
        self.line.reset();
        self.mouth.reset();
        self.loss.reset();
    }
}

// ---------------------------------------------------------------------------
// Engine block resonance
// ---------------------------------------------------------------------------
//
// The block's first mode, and nothing else. What is built on top of it — the
// bending, pan and bore-wall families, and the three excitations that drive
// them — lives in [`crate::audio::structure`], because a radiating body is not
// a filter.

/// Dressed block mass whose first rumble mode sits at [`BLOCK_REFERENCE_HZ`] [kg].
///
/// A 4-litre iron-decked V8 short block with pan, covers and accessories hung on
/// it lands near this.
pub const BLOCK_REFERENCE_MASS: f32 = 180.0;

/// First structural rumble mode of a block of [`BLOCK_REFERENCE_MASS`] [Hz].
pub const BLOCK_REFERENCE_HZ: f32 = 80.0;

/// Lowest block resonance the model will produce [Hz].
pub const BLOCK_MIN_HZ: f32 = 60.0;

/// Highest block resonance the model will produce [Hz].
pub const BLOCK_MAX_HZ: f32 = 120.0;

/// First structural mode of a block of the given dressed mass [Hz].
///
/// A block is a stiff lump on soft mounts driven by its own combustion, and its
/// lowest global mode goes as any mass on a spring does:
///
/// ```text
/// f = f_ref * sqrt(m_ref / m)
/// ```
///
/// Stiffness is held fixed across the range on the grounds that an alloy block
/// and an iron one of the same displacement differ far more in density than in
/// section — which is why the light one rings higher rather than merely quieter.
/// Clamped to `[BLOCK_MIN_HZ, BLOCK_MAX_HZ]`: outside that band the mode is no
/// longer the one this filter is modelling.
#[inline]
pub fn block_resonance_hz(dressed_mass: f32) -> f32 {
    let mass = dressed_mass.max(1.0);
    (BLOCK_REFERENCE_HZ * (BLOCK_REFERENCE_MASS / mass).sqrt()).clamp(BLOCK_MIN_HZ, BLOCK_MAX_HZ)
}

// ---------------------------------------------------------------------------
// Modal resonator bank
// ---------------------------------------------------------------------------

/// A compact modal resonator bank representing the acoustic modes of a casing,
/// head, or block excited by mechanical impacts.
///
/// Each mode is a constant-peak-gain bandpass [`Biquad`] section with its own
/// resonant frequency, Q factor, and relative mix weight.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModalBank {
    modes: [Biquad; 2],
    weights: [f32; 2],
    count: usize,
}

impl Default for ModalBank {
    fn default() -> Self {
        Self {
            modes: [Biquad::default(); 2],
            weights: [0.0; 2],
            count: 0,
        }
    }
}

impl ModalBank {
    /// Builds a modal bank with a single resonant mode.
    pub fn single(sample_rate: f32, frequency: f32, q: f32) -> Self {
        Self {
            modes: [
                Biquad::new(BiquadCoeffs::bandpass(sample_rate, frequency, q)),
                Biquad::default(),
            ],
            weights: [1.0, 0.0],
            count: 1,
        }
    }

    /// Builds a modal bank with two resonant modes.
    pub fn dual(sample_rate: f32, mode1: (f32, f32, f32), mode2: (f32, f32, f32)) -> Self {
        Self {
            modes: [
                Biquad::new(BiquadCoeffs::bandpass(sample_rate, mode1.0, mode1.1)),
                Biquad::new(BiquadCoeffs::bandpass(sample_rate, mode2.0, mode2.1)),
            ],
            weights: [mode1.2, mode2.2],
            count: 2,
        }
    }

    /// Number of active resonant modes in the bank.
    pub fn mode_count(&self) -> usize {
        self.count
    }

    /// Retunes one mode's resonant frequency and Q without resetting filter state.
    pub fn retune(&mut self, sample_rate: f32, index: usize, frequency: f32, q: f32) {
        if index < self.count {
            self.modes[index].set_coeffs(BiquadCoeffs::bandpass(sample_rate, frequency, q));
        }
    }

    /// Clears internal state of all modes.
    pub fn reset(&mut self) {
        for i in 0..self.count {
            self.modes[i].reset();
        }
    }

    /// Filters one excitation sample through the parallel modes.
    #[inline(always)]
    pub fn process(&mut self, x: f32) -> f32 {
        let mut out = 0.0f32;
        for i in 0..self.count {
            out += self.modes[i].process(x) * self.weights[i];
        }
        out
    }
}

// ---------------------------------------------------------------------------
// Output conditioning
// ---------------------------------------------------------------------------

/// Odd-symmetric soft clipper, a Padé approximation of `tanh`.
///
/// The last line of defence before the sample leaves for the device. Backfires
/// and a full-load V8 both peak well above the sum of their steady-state parts,
/// and hard clipping those transients is precisely the "pop" this engine must
/// not produce: a discontinuity in the waveform's *derivative* is audible as a
/// click even when the sample values themselves stay in range. This is smooth
/// everywhere, so overload turns into harmonic distortion rather than a snap.
#[inline(always)]
pub fn soft_clip(x: f32) -> f32 {
    // Beyond +/-3 the rational form turns back on itself, so saturate first.
    let x = x.clamp(-3.0, 3.0);
    let x2 = x * x;
    (x * (27.0 + x2) / (27.0 + 9.0 * x2)).clamp(-1.0, 1.0)
}

// ---------------------------------------------------------------------------
// 2x Oversampled Clipper
// ---------------------------------------------------------------------------

/// 1st-order allpass section for polyphase half-band IIR filters.
#[derive(Debug, Clone, Copy)]
struct AllpassSection {
    a: f32,
    x1: f32,
    y1: f32,
}

impl AllpassSection {
    fn new(a: f32) -> Self {
        Self {
            a,
            x1: 0.0,
            y1: 0.0,
        }
    }

    #[inline(always)]
    fn process(&mut self, x: f32) -> f32 {
        let y = self.a * (x - self.y1) + self.x1;
        self.x1 = flush(x);
        self.y1 = flush(y);
        y
    }

    fn reset(&mut self) {
        self.x1 = 0.0;
        self.y1 = 0.0;
    }
}

/// 2x oversampled soft clipper using a 5th-order elliptic polyphase IIR half-band filter.
///
/// Running non-linear saturation on fast transients at 48 kHz produces harmonics that
/// fold over the Nyquist frequency into the audible band as aliasing distortion.
///
/// By upsampling 2x to 96 kHz before evaluating the Padé soft clipper, the high-order
/// harmonics land well below the 48 kHz Nyquist limit and are filtered out by the
/// decimation half-band lowpass before downsampling back to 1x.
#[derive(Debug, Clone, Copy)]
pub struct OversampledClipper {
    up0: AllpassSection,
    up1: AllpassSection,
    down0: AllpassSection,
    down1: AllpassSection,
    down_delay: f32,
}

impl OversampledClipper {
    /// Constructs a new 2x oversampled clipper with half-band filter state cleared.
    pub fn new() -> Self {
        Self {
            up0: AllpassSection::new(0.14134867),
            up1: AllpassSection::new(0.5899948),
            down0: AllpassSection::new(0.14134867),
            down1: AllpassSection::new(0.5899948),
            down_delay: 0.0,
        }
    }

    /// Resets all internal filter states to zero.
    pub fn reset(&mut self) {
        self.up0.reset();
        self.up1.reset();
        self.down0.reset();
        self.down1.reset();
        self.down_delay = 0.0;
    }

    /// Processes one sample at 1x sample rate with 2x oversampled soft clipping.
    #[inline(always)]
    pub fn process(&mut self, x: f32) -> f32 {
        // 1x -> 2x upsampling polyphase branches
        let u0 = self.up0.process(x);
        let u1 = self.up1.process(x);

        // Evaluate nonlinearity at 2x rate
        let w0 = soft_clip(u0);
        let w1 = soft_clip(u1);

        // 2x -> 1x decimation polyphase branches
        let d0 = self.down0.process(w0);
        let d1 = self.down_delay;
        self.down_delay = self.down1.process(w1);

        (0.5 * (d0 + d1)).clamp(-1.0, 1.0)
    }
}

impl Default for OversampledClipper {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests;
