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

/// Hermite ramp from 0 at `lo` to 1 at `hi`, flat outside.
///
/// Used wherever a control law has to hand over between two regimes. A linear
/// ramp would leave a slope discontinuity at each end, and on a parameter that
/// is being swept slowly — engine speed crossing a damping threshold, say —
/// that corner is audible as a moment where the timbre changes direction.
#[inline]
fn smoothstep(lo: f32, hi: f32, x: f32) -> f32 {
    if hi <= lo {
        return if x < lo { 0.0 } else { 1.0 };
    }
    let t = ((x - lo) / (hi - lo)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
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

    /// Clears state.
    pub fn reset(&mut self) {
        self.z = 0.0;
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

/// Power-of-two fractional delay line with linear interpolation.
///
/// The capacity is rounded up to a power of two so the wrap is a mask rather
/// than a modulo or a branch, which matters because the exhaust chain reads one
/// of these per bank per sample.
///
/// Linear interpolation costs a mild lowpass at long delays. That is the correct
/// error to accept here: an allpass interpolator has better magnitude response
/// but its own state, and a moving allpass rings when the delay length changes —
/// which is exactly what happens every time the exhaust temperature moves.
#[derive(Debug, Clone)]
pub struct DelayLine {
    buffer: Vec<f32>,
    mask: usize,
    write: usize,
}

impl DelayLine {
    /// A line able to hold at least `max_delay` samples.
    pub fn with_max_delay(max_delay: usize) -> Self {
        let capacity = (max_delay + 4).next_power_of_two().max(8);
        Self {
            buffer: vec![0.0; capacity],
            mask: capacity - 1,
            write: 0,
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
    }

    /// Writes one sample at the head.
    #[inline(always)]
    pub fn push(&mut self, x: f32) {
        self.buffer[self.write] = flush(x);
        self.write = (self.write + 1) & self.mask;
    }

    /// Reads `delay` samples back from the head, interpolating.
    ///
    /// A delay of 1.0 is the sample just written. The request is clamped into
    /// the line rather than wrapping: a too-long delay is a tuning error, and
    /// silently aliasing it to a short one hides that, while wrapping would read
    /// the future.
    #[inline(always)]
    pub fn read(&self, delay: f32) -> f32 {
        let d = delay.clamp(1.0, self.max_delay());
        let whole = d as usize;
        let frac = d - whole as f32;
        let len = self.buffer.len();
        let near = self.buffer[(self.write + len - whole) & self.mask];
        let far = self.buffer[(self.write + len - whole - 1) & self.mask];
        near + (far - near) * frac
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
/// comb filter — see [`waveguide_damping`].
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

/// Ratio below which a runner loop is continuously re-excited [-].
///
/// Under four round trips per firing interval the next pulse arrives before the
/// previous one has finished bouncing, and the pipe is a driven resonator.
pub const WAVEGUIDE_COUPLED_RATIO: f32 = 4.0;

/// Ratio above which a runner loop is fully isolated between firings [-].
pub const WAVEGUIDE_ISOLATED_RATIO: f32 = 22.0;

/// The Transit-Time Decision Rule: how hard to damp a runner at this speed.
///
/// # The artefact this exists to remove
///
/// A delay line closed into a reflecting loop is a comb filter. That is the
/// point of it — the comb *is* the pipe's harmonic series, and when a fresh
/// blowdown pulse arrives every few round trips, what the listener hears is a
/// pipe being played: each pulse re-excites the loop before the last has decayed
/// and the comb reads as the note's own resonance.
///
/// At idle the arithmetic changes. A V8 bank at 800 rpm fires every 37 ms, and a
/// 0.45 m runner on hot gas round-trips in about 1.5 ms — twenty-four bounces
/// between excitations. The loop spends almost all of its time with no pulse in
/// it, ringing down on whatever broadband noise is present, and a static comb
/// filter sitting on a noise floor is exactly the recipe for flanging. It is the
/// single most recognisable "synthetic" artefact in a procedural engine: a
/// metallic, hollow, faintly pitched sheen under an idle that should be dry.
///
/// A real runner does not do this, because a real runner is much lossier than a
/// lossless delay: at idle the gas is slow, cool and barely moving, the boundary
/// layer is thick relative to the bore, and the collector is a poor reflector
/// for a wave that arrives with almost no amplitude behind it.
///
/// # The rule
///
/// ```text
/// ratio   = tau_interval / tau_pulse
/// damping = smoothstep(WAVEGUIDE_COUPLED_RATIO, WAVEGUIDE_ISOLATED_RATIO, ratio)
/// ```
///
/// where `tau_interval` is [`firing_interval_seconds`] and `tau_pulse` is the
/// [`round_trip_seconds`] of the runner. Because `tau_interval` falls with
/// engine speed while `tau_pulse` depends only on length and gas temperature,
/// the ratio is large at idle and small at the limiter — so damping comes on
/// where the artefact lives and gets out of the way where the resonance is
/// wanted. Returns `0` (no extra damping) to `1` (fully damped).
#[inline]
pub fn waveguide_damping(tau_interval: f32, tau_pulse: f32) -> f32 {
    // A degenerate or non-finite pipe gets full damping: whatever it is doing,
    // it is not modelling a resonance, and the safe failure is a quiet one.
    if !tau_pulse.is_finite() || tau_pulse <= 0.0 || !tau_interval.is_finite() {
        return 1.0;
    }
    let ratio = tau_interval / tau_pulse;
    smoothstep(WAVEGUIDE_COUPLED_RATIO, WAVEGUIDE_ISOLATED_RATIO, ratio)
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

/// The muffler: a Helmholtz bandpass in parallel with a damped through path.
///
/// A pure bandpass is what the cavity does, but it is not what a muffler
/// *sounds* like — all of the transient energy that gives an exhaust note its
/// edge lives outside the resonance. Real mufflers pass that energy through with
/// loss, so the output here is a mix: `resonant_mix` of the cavity, the rest
/// lowpassed by the tailpipe.
#[derive(Debug, Clone)]
pub struct Muffler {
    geometry: MufflerGeometry,
    cavity: Biquad,
    tailpipe: OnePole,
    sample_rate: f32,
    centre_hz: f32,
}

impl Muffler {
    /// Builds a muffler and tunes it for `temperature` [K].
    pub fn new(
        sample_rate: f32,
        geometry: MufflerGeometry,
        gamma: f32,
        gas_constant: f32,
        temperature: f32,
    ) -> Self {
        let mut muffler = Self {
            geometry,
            cavity: Biquad::default(),
            tailpipe: OnePole::new(sample_rate, geometry.tailpipe_cutoff as f32),
            sample_rate,
            centre_hz: 0.0,
        };
        muffler.tune(gamma, gas_constant, temperature);
        muffler
    }

    /// Retunes the cavity for the current exhaust temperature [K].
    ///
    /// Called at the control rate, not per sample: the transcendentals in the
    /// coefficient design are the expensive part of this whole module, and the
    /// temperature that drives them is already smoothed.
    pub fn tune(&mut self, gamma: f32, gas_constant: f32, temperature: f32) {
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

    /// Filters one sample.
    #[inline(always)]
    pub fn process(&mut self, x: f32) -> f32 {
        let mix = self.geometry.resonant_mix as f32;
        let resonant = self.cavity.process(x);
        let through = self.tailpipe.process(x);
        through * (1.0 - mix) + resonant * mix
    }

    /// Clears state.
    pub fn reset(&mut self) {
        self.cavity.reset();
        self.tailpipe.reset();
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
    /// Drive this from [`waveguide_damping`] at the control rate. Both the
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

/// Speed at or below which block rumble is at full level [rev/min].
pub const BLOCK_RUMBLE_FLOOR_RPM: f32 = 700.0;

/// Speed at which block rumble has tapered away entirely [rev/min].
pub const BLOCK_RUMBLE_TAPER_RPM: f32 = 4_000.0;

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

/// The structural rumble of the block and oil pan, as a speed-dependent EQ.
///
/// # What is being modelled
///
/// Not everything an engine radiates leaves through the tailpipe. Each firing
/// hammers the block, and the block — together with the sump full of oil bolted
/// under it — answers at its own low modes. That is the component a listener
/// reads as *size*: the difference between a large engine and a small one at
/// idle is far more this than it is the exhaust note, which at 850 rpm is a
/// series of discrete cracks with very little between them.
///
/// # Why the gain falls with speed
///
/// The block is being driven by an impulse train whose fundamental climbs
/// straight through the resonance and out the other side. At idle a V8's firing
/// frequency is around 57 Hz, sitting right in the mode and pumping it once per
/// cylinder; by 4000 rpm it is at 267 Hz and the block is being driven well
/// above resonance, where a mass-dominated structure barely responds. The
/// radiated exhaust power has meanwhile climbed by an order of magnitude and
/// buries what is left. So the gain is scheduled to be highest at idle and gone
/// by [`BLOCK_RUMBLE_TAPER_RPM`] — which is also, conveniently, exactly the
/// schedule that stops the low end from turning to mud under load.
#[derive(Debug, Clone)]
pub struct BlockResonator {
    /// One section per output channel; both carry identical coefficients.
    sections: [Biquad; 2],
    sample_rate: f32,
    centre_hz: f32,
    q: f32,
    max_gain_db: f32,
    gain_db: f32,
}

impl BlockResonator {
    /// A resonator for a block of `dressed_mass` kilograms.
    ///
    /// `max_gain_db` is the boost at [`BLOCK_RUMBLE_FLOOR_RPM`] and below.
    pub fn new(sample_rate: f32, dressed_mass: f32, q: f32, max_gain_db: f32) -> Self {
        let mut resonator = Self {
            sections: [Biquad::default(); 2],
            sample_rate,
            centre_hz: block_resonance_hz(dressed_mass),
            q: q.clamp(0.3, 12.0),
            max_gain_db: max_gain_db.max(0.0),
            gain_db: 0.0,
        };
        resonator.tune(0.0);
        resonator
    }

    /// Retunes for a new block mass [kg]. Coefficients follow on the next
    /// [`BlockResonator::tune`].
    pub fn set_mass(&mut self, dressed_mass: f32) {
        self.centre_hz = block_resonance_hz(dressed_mass);
    }

    /// Rebuilds the peak for the current engine speed [rev/min].
    ///
    /// Call at the control rate with an already-smoothed speed: the peaking
    /// design runs a `powf` and two trigonometric functions, and the whole point
    /// of the control block is to keep those off the per-sample path.
    pub fn tune(&mut self, rpm: f32) {
        let taper = smoothstep(BLOCK_RUMBLE_FLOOR_RPM, BLOCK_RUMBLE_TAPER_RPM, rpm);
        self.gain_db = self.max_gain_db * (1.0 - taper);
        let coeffs = BiquadCoeffs::peaking(self.sample_rate, self.centre_hz, self.q, self.gain_db);
        for section in self.sections.iter_mut() {
            section.set_coeffs(coeffs);
        }
    }

    /// Centre of the peak [Hz].
    pub fn centre_frequency(&self) -> f32 {
        self.centre_hz
    }

    /// Boost currently applied at the centre [dB].
    pub fn gain_db(&self) -> f32 {
        self.gain_db
    }

    /// Filters one sample of one channel.
    ///
    /// Channels above the second fold onto the last section. The block is a
    /// single physical object radiating a wavelength of four metres at 80 Hz, so
    /// there is nothing to image: both channels get the same response, and the
    /// separate state exists only because they carry different signals.
    #[inline(always)]
    pub fn process(&mut self, channel: usize, x: f32) -> f32 {
        self.sections[channel.min(self.sections.len() - 1)].process(x)
    }

    /// Clears state.
    pub fn reset(&mut self) {
        self.sections.iter_mut().for_each(Biquad::reset);
    }
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
    x * (27.0 + x2) / (27.0 + 9.0 * x2)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f32, b: f32, tol: f32) {
        assert!((a - b).abs() <= tol, "expected {b}, got {a} (tol {tol})");
    }

    /// Runs an impulse through a filter and returns the peak magnitude of the
    /// response at `frequency`, measured by quadrature correlation.
    fn magnitude_at(coeffs: BiquadCoeffs, sample_rate: f32, frequency: f32) -> f32 {
        let mut filter = Biquad::new(coeffs);
        let n = 8192;
        // Discard the transient, then correlate the steady state.
        let (mut re, mut im) = (0.0f32, 0.0f32);
        for i in 0..n {
            let t = i as f32 / sample_rate;
            let y = filter.process((TAU * frequency * t).sin());
            if i >= n / 2 {
                let phase = TAU * frequency * t;
                re += y * phase.sin();
                im += y * phase.cos();
            }
        }
        let half = (n / 2) as f32;
        2.0 * (re * re + im * im).sqrt() / half
    }

    #[test]
    fn bandpass_peaks_at_unity_on_centre() {
        let fs = 48_000.0;
        let coeffs = BiquadCoeffs::bandpass(fs, 120.0, 2.0);
        approx(magnitude_at(coeffs, fs, 120.0), 1.0, 0.02);
        // And rolls off either side.
        assert!(magnitude_at(coeffs, fs, 20.0) < 0.3);
        assert!(magnitude_at(coeffs, fs, 1200.0) < 0.3);
    }

    #[test]
    fn bandpass_gain_is_independent_of_q() {
        let fs = 48_000.0;
        for q in [0.5, 2.0, 8.0] {
            let coeffs = BiquadCoeffs::bandpass(fs, 200.0, q);
            approx(magnitude_at(coeffs, fs, 200.0), 1.0, 0.03);
        }
    }

    #[test]
    fn biquad_stays_stable_above_nyquist_request() {
        let mut filter = Biquad::new(BiquadCoeffs::bandpass(48_000.0, 96_000.0, 20.0));
        let mut noise = Noise::default();
        for _ in 0..48_000 {
            let y = filter.process(noise.next_bipolar());
            assert!(y.is_finite() && y.abs() < 100.0, "unstable: {y}");
        }
    }

    #[test]
    fn delay_line_reads_back_what_went_in() {
        let mut line = DelayLine::with_max_delay(64);
        for i in 0..32 {
            line.push(i as f32);
        }
        // Delay of 1 is the most recent sample.
        approx(line.read(1.0), 31.0, 1e-6);
        approx(line.read(8.0), 24.0, 1e-6);
        // Halfway between samples 24 and 23.
        approx(line.read(8.5), 23.5, 1e-6);
    }

    #[test]
    fn delay_line_clamps_instead_of_reading_the_future() {
        let mut line = DelayLine::with_max_delay(16);
        for _ in 0..64 {
            line.push(1.0);
        }
        assert!(line.read(0.0).is_finite());
        assert!(line.read(1e9).is_finite());
    }

    #[test]
    fn speed_of_sound_matches_hand_calculation() {
        // Air at 293 K: sqrt(1.4 * 287 * 293) = 343.1 m/s.
        approx(speed_of_sound(1.4, 287.0, 293.0), 343.1, 1.0);
        // Exhaust at 900 K is much faster, which is why hot pipes ring higher.
        assert!(speed_of_sound(1.33, 287.0, 900.0) > 580.0);
    }

    #[test]
    fn runner_delay_shortens_as_gas_heats() {
        let cold = runner_delay_seconds(0.45, 1.33, 287.0, 400.0);
        let hot = runner_delay_seconds(0.45, 1.33, 287.0, 1100.0);
        assert!(hot < cold, "hot gas must transit faster");
        // 0.45 m at 400 K: c = 390 m/s, tau = 1.15 ms.
        approx(cold, 0.45 / 390.6, 5e-5);
    }

    #[test]
    fn helmholtz_matches_closed_form() {
        let (gamma, r, t) = (1.33f32, 287.0f32, 900.0f32);
        let (area, volume, neck) = (2.0e-3f32, 8.0e-3f32, 0.10f32);
        let c = speed_of_sound(gamma, r, t);
        let expected = (c / TAU) * (area / (volume * neck)).sqrt();
        approx(
            helmholtz_frequency(gamma, r, t, area, volume, neck),
            expected,
            1e-3,
        );
        // Sanity: a road muffler sits in the low hundreds of Hz.
        assert!((50.0..600.0).contains(&expected), "unphysical: {expected}");
    }

    #[test]
    fn runner_loop_is_stable_under_delay_modulation() {
        let mut runner = ExhaustRunner::new(48_000.0, 0.5, 0.019, 0.9, 1.33, 287.0, 900.0);
        let mut noise = Noise::default();
        for i in 0..96_000 {
            // Sweep the temperature across the whole plausible range while
            // driving it hard.
            if i % 64 == 0 {
                let t = 400.0 + 700.0 * ((i as f32 / 4800.0).sin() * 0.5 + 0.5);
                runner.tune(1.33, 287.0, t);
            }
            let y = runner.process(noise.next_bipolar());
            assert!(y.is_finite() && y.abs() < 50.0, "runner blew up: {y}");
        }
    }

    #[test]
    fn smoothed_glides_without_overshoot() {
        let mut p = Smoothed::new(0.0, 48_000.0, 0.010);
        p.set_target(1.0);
        let mut previous = 0.0;
        for _ in 0..4_800 {
            let v = p.next_value();
            assert!(v >= previous - 1e-9 && v <= 1.0, "overshoot: {v}");
            previous = v;
        }
        // One time constant is 63 %; 100 ms is ten of them.
        approx(p.value(), 1.0, 1e-3);
    }

    #[test]
    fn smoothed_ignores_non_finite_targets() {
        let mut p = Smoothed::new(0.5, 48_000.0, 0.01);
        p.set_target(f32::NAN);
        p.set_target(f32::INFINITY);
        for _ in 0..1000 {
            assert!(p.next_value().is_finite());
        }
    }

    #[test]
    fn dc_blocker_removes_offset_but_keeps_signal() {
        let mut blocker = DcBlocker::default();
        let mut last = 0.0;
        for i in 0..48_000 {
            last = blocker.process(1.0 + 0.5 * (TAU * 200.0 * i as f32 / 48_000.0).sin());
        }
        assert!(last.abs() < 0.55, "offset survived: {last}");
        // The 200 Hz component must still be there.
        let mut peak: f32 = 0.0;
        for i in 48_000..49_000 {
            let y = blocker.process(1.0 + 0.5 * (TAU * 200.0 * i as f32 / 48_000.0).sin());
            peak = peak.max(y.abs());
        }
        approx(peak, 0.5, 0.05);
    }

    #[test]
    fn soft_clip_is_monotone_and_bounded() {
        let mut previous = f32::NEG_INFINITY;
        let mut x = -20.0;
        while x <= 20.0 {
            let y = soft_clip(x);
            assert!(y.abs() <= 1.05, "unbounded at {x}: {y}");
            assert!(y >= previous - 1e-6, "non-monotone at {x}");
            previous = y;
            x += 0.01;
        }
        // Near-unity gain in the linear region, so quiet signals pass clean.
        approx(soft_clip(0.1), 0.1, 2e-3);
        approx(soft_clip(0.0), 0.0, 1e-9);
    }

    #[test]
    fn noise_is_zero_mean_and_bounded() {
        let mut noise = Noise::new(12345);
        let mut sum = 0.0f64;
        for _ in 0..200_000 {
            let v = noise.next_bipolar();
            assert!((-1.0..1.0).contains(&v));
            sum += v as f64;
        }
        assert!(
            (sum / 200_000.0).abs() < 0.01,
            "biased: {}",
            sum / 200_000.0
        );
    }

    #[test]
    fn gaussian_draws_are_standard_normal_and_bounded() {
        let mut noise = Noise::new(4242);
        let n = 400_000;
        let (mut sum, mut sum_sq) = (0.0f64, 0.0f64);
        let mut extreme = 0.0f32;
        for _ in 0..n {
            let v = noise.next_gaussian();
            sum += v as f64;
            sum_sq += (v as f64) * (v as f64);
            extreme = extreme.max(v.abs());
        }
        let mean = sum / n as f64;
        let variance = sum_sq / n as f64 - mean * mean;
        assert!(mean.abs() < 0.01, "biased: {mean}");
        assert!((variance - 1.0).abs() < 0.02, "variance {variance}, want 1");
        // Irwin-Hall with four terms cannot leave +/-sqrt(12).
        assert!(extreme <= 12.0f32.sqrt(), "left its support: {extreme}");
        assert!(extreme > 2.5, "suspiciously narrow: {extreme}");
    }

    #[test]
    fn firing_interval_follows_the_four_stroke_law() {
        // A V8 bank of four at 800 rpm: a 150 ms cycle shared by four cylinders.
        approx(firing_interval_seconds(800.0, 4), 0.0375, 1e-6);
        // Twice the speed, half the interval.
        approx(firing_interval_seconds(1_600.0, 4), 0.01875, 1e-6);
        // A stopped engine never fires again.
        assert_eq!(firing_interval_seconds(0.0, 4), f32::MAX);
    }

    #[test]
    fn transit_time_rule_damps_idle_and_frees_the_top_end() {
        // A 0.45 m runner on 900 K exhaust: about 1.55 ms round trip.
        let tau_pulse = round_trip_seconds(0.45, 1.33, 287.0, 900.0);
        assert!(
            (0.0012..0.0020).contains(&tau_pulse),
            "unphysical round trip: {tau_pulse}"
        );

        let damping_at = |rpm: f32| waveguide_damping(firing_interval_seconds(rpm, 4), tau_pulse);

        // Idle: two dozen round trips between firings, so the loop is isolated
        // and would otherwise stand up a static comb on the noise floor.
        assert!(
            damping_at(800.0) > 0.9,
            "idle undamped: {}",
            damping_at(800.0)
        );
        // The limiter: pulses arrive faster than the pipe can ring out, which is
        // the regime the resonance is *wanted* in.
        assert!(
            damping_at(7_000.0) < 0.1,
            "top end over-damped: {}",
            damping_at(7_000.0)
        );

        // And it is monotone in between — no speed where damping goes the wrong
        // way, which would be heard as the note brightening as it slowed.
        let mut previous = f32::MAX;
        let mut rpm = 600.0;
        while rpm <= 8_000.0 {
            let d = damping_at(rpm);
            assert!(d <= previous + 1e-6, "damping rose with speed at {rpm} rpm");
            previous = d;
            rpm += 50.0;
        }

        // A stopped engine is fully damped rather than dividing by zero.
        assert_eq!(waveguide_damping(f32::MAX, tau_pulse), 1.0);
        assert_eq!(waveguide_damping(0.01, 0.0), 1.0);
    }

    #[test]
    fn damping_shortens_the_runner_ringdown() {
        // Kick the loop once and measure how long it takes to fall 40 dB.
        let ringdown = |damping: f32| {
            let mut runner = ExhaustRunner::new(48_000.0, 0.45, 0.019, 0.85, 1.33, 287.0, 900.0);
            runner.set_damping(damping);
            // Let the glided reflection reach its target before the kick.
            for _ in 0..12_000 {
                runner.process(0.0);
            }
            let mut peak = runner.process(1.0).abs();
            let mut last_loud = 0usize;
            for i in 1..48_000 {
                let y = runner.process(0.0).abs();
                peak = peak.max(y);
                if y > peak * 0.01 {
                    last_loud = i;
                }
            }
            last_loud
        };

        let open = ringdown(0.0);
        let damped = ringdown(1.0);
        assert!(open > 0, "the undamped pipe did not ring at all");
        assert!(
            damped * 3 < open,
            "damping barely shortened the ringdown: {damped} vs {open} samples"
        );

        // The damped loop must still die inside one idle firing interval —
        // that is the whole point of the rule.
        let idle_interval = (firing_interval_seconds(800.0, 4) * 48_000.0) as usize;
        assert!(
            damped < idle_interval,
            "still ringing when the next pulse lands: {damped} vs {idle_interval}"
        );
    }

    /// Peak-to-trough ripple of a runner's magnitude response over the band
    /// where the comb lives [dB].
    ///
    /// This is a direct measurement of the flanging artefact: a comb filter is
    /// exactly a response with deep regular ripple, and how deep it is, is how
    /// audible the flange is.
    fn comb_ripple_db(damping: f32) -> f32 {
        let fs = 48_000.0;
        let (mut lo, mut hi) = (f32::MAX, 0.0f32);
        let mut f = 200.0f32;
        while f <= 2_000.0 {
            let mut runner = ExhaustRunner::new(fs, 0.45, 0.019, 0.55, 1.33, 287.0, 700.0);
            runner.set_damping(damping);
            // Settle the glided reflection, then the tone's own transient.
            for _ in 0..12_000 {
                runner.process(0.0);
            }
            let (mut re, mut im) = (0.0f64, 0.0f64);
            for i in 0..12_000 {
                let phase = TAU * f * i as f32 / fs;
                let y = runner.process(phase.sin());
                if i >= 6_000 {
                    re += y as f64 * phase.sin() as f64;
                    im += y as f64 * phase.cos() as f64;
                }
            }
            let m = (2.0 * (re * re + im * im).sqrt() / 6_000.0) as f32;
            lo = lo.min(m);
            hi = hi.max(m);
            f += 10.0;
        }
        20.0 * (hi / lo.max(1e-9)).log10()
    }

    #[test]
    fn damping_flattens_the_comb() {
        // The artefact, measured: an undamped runner has a deep regular ripple
        // across the whole midrange, which on broadband excitation is heard as
        // flanging. Damping it is what the Transit-Time Decision Rule buys.
        let open = comb_ripple_db(0.0);
        let damped = comb_ripple_db(1.0);
        assert!(
            open > 8.0,
            "the undamped pipe barely combs at all: {open:.1} dB"
        );
        assert!(
            damped < 0.45 * open,
            "damping left the comb standing: {damped:.1} dB vs {open:.1} dB"
        );
        // But it must not flatten it away entirely — the pipe still has to be a
        // pipe, or the exhaust loses its character along with its flange.
        assert!(damped > 1.0, "damped into a plain delay: {damped:.1} dB");
    }

    #[test]
    fn damping_lowers_reflection_and_darkens_the_return_path() {
        let mut runner = ExhaustRunner::new(48_000.0, 0.45, 0.019, 0.80, 1.33, 287.0, 900.0);
        let nominal = runner.reflection();
        approx(nominal, 0.80, 1e-6);
        assert_eq!(runner.damping(), 0.0);

        runner.set_damping(1.0);
        assert_eq!(runner.damping(), 1.0);
        // Glided, not stepped: it has not arrived yet.
        assert!(runner.reflection() > 0.7, "reflection jumped");
        for _ in 0..48_000 {
            runner.process(0.0);
        }
        approx(runner.reflection(), 0.80 * RUNNER_DAMPED_REFLECTION, 1e-3);

        // Never unstable, at any damping, under a hard drive.
        let mut noise = Noise::new(11);
        for step in 0..24_000 {
            if step % 64 == 0 {
                runner.set_damping((step as f32 / 24_000.0).sin().abs());
            }
            let y = runner.process(noise.next_bipolar());
            assert!(y.is_finite() && y.abs() < 50.0, "blew up: {y}");
        }
    }

    #[test]
    fn block_resonance_falls_with_mass() {
        // The reference block sits on the reference frequency by definition.
        approx(
            block_resonance_hz(BLOCK_REFERENCE_MASS),
            BLOCK_REFERENCE_HZ,
            1e-3,
        );
        // f ~ 1/sqrt(m): four times the mass is half the frequency, before the
        // clamp catches it.
        let heavy = block_resonance_hz(BLOCK_REFERENCE_MASS * 4.0);
        assert_eq!(heavy, BLOCK_MIN_HZ, "should have clamped to the floor");
        approx(
            block_resonance_hz(BLOCK_REFERENCE_MASS * 1.4),
            BLOCK_REFERENCE_HZ / 1.4f32.sqrt(),
            0.5,
        );
        // A light alloy four rings high, a big iron V8 rings low, and both stay
        // inside the band the model claims.
        for mass in [40.0, 90.0, 180.0, 260.0, 400.0] {
            let f = block_resonance_hz(mass);
            assert!(
                (BLOCK_MIN_HZ..=BLOCK_MAX_HZ).contains(&f),
                "{mass} kg -> {f} Hz"
            );
        }
        assert!(block_resonance_hz(90.0) > block_resonance_hz(260.0));
        // Degenerate input must not divide by zero.
        assert!(block_resonance_hz(0.0).is_finite());
    }

    #[test]
    fn block_resonator_boosts_at_idle_and_steps_aside_at_speed() {
        let fs = 48_000.0;
        let mut resonator = BlockResonator::new(fs, BLOCK_REFERENCE_MASS, 1.1, 9.0);

        resonator.tune(700.0);
        approx(resonator.gain_db(), 9.0, 1e-3);
        let centre = resonator.centre_frequency();
        approx(centre, 80.0, 1e-3);

        // The peak really is a peak, and really is 9 dB.
        let at = |resonator: &BlockResonator, f: f32| {
            let coeffs =
                BiquadCoeffs::peaking(fs, resonator.centre_frequency(), 1.1, resonator.gain_db());
            magnitude_at(coeffs, fs, f)
        };
        // Loose tolerance on the centre: `magnitude_at` correlates over 4096
        // samples, which is only about seven cycles of 80 Hz, so the estimator
        // itself carries a couple of percent here.
        approx(at(&resonator, centre), 10f32.powf(9.0 / 20.0), 0.10);
        // A peak, not a shelf: back to unity well above it. Probed high rather
        // than low because `magnitude_at` needs several cycles inside its
        // correlation window and there are not many of those below 80 Hz.
        approx(at(&resonator, 640.0), 1.0, 0.05);
        approx(at(&resonator, 4_000.0), 1.0, 0.05);
        // And it really is monotone down the skirt.
        assert!(at(&resonator, 160.0) < at(&resonator, centre));
        assert!(at(&resonator, 640.0) < at(&resonator, 160.0));

        // And it tapers away with engine speed.
        resonator.tune(2_000.0);
        let mid = resonator.gain_db();
        resonator.tune(BLOCK_RUMBLE_TAPER_RPM);
        assert!(
            (0.0..1e-3).contains(&resonator.gain_db()),
            "still boosting at speed"
        );
        assert!(mid > 0.0 && mid < 9.0, "no taper in between: {mid}");

        // Mass moves the peak, and both channels stay in step.
        resonator.set_mass(400.0);
        resonator.tune(700.0);
        approx(resonator.centre_frequency(), BLOCK_MIN_HZ, 1e-3);
        let mut noise = Noise::new(3);
        for _ in 0..48_000 {
            let x = noise.next_bipolar();
            let (l, r) = (resonator.process(0, x), resonator.process(1, x));
            assert!(l.is_finite() && (l - r).abs() < 1e-6, "channels diverged");
        }
    }

    #[test]
    fn muffler_tracks_temperature() {
        let mut muffler = Muffler::new(48_000.0, MufflerGeometry::default(), 1.33, 287.0, 500.0);
        let cold = muffler.centre_frequency();
        muffler.tune(1.33, 287.0, 1100.0);
        let hot = muffler.centre_frequency();
        // f ~ c ~ sqrt(T), so a 2.2x temperature ratio is a 1.48x pitch ratio.
        approx(hot / cold, (1100.0f32 / 500.0).sqrt(), 0.01);
    }

    #[test]
    fn modal_bank_resonates_at_designed_modes() {
        let fs = 48_000.0;
        let bank = ModalBank::dual(fs, (1000.0, 4.0, 0.7), (3000.0, 4.0, 0.3));
        assert_eq!(bank.mode_count(), 2);

        // Correlate response to pure tones at mode 1, mode 2, and an off-resonance frequency.
        let response_at = |freq: f32| {
            let mut b = bank;
            let n = 8192;
            let (mut re, mut im) = (0.0f32, 0.0f32);
            for i in 0..n {
                let t = i as f32 / fs;
                let y = b.process((TAU * freq * t).sin());
                if i >= n / 2 {
                    let phase = TAU * freq * t;
                    re += y * phase.sin();
                    im += y * phase.cos();
                }
            }
            let half = (n / 2) as f32;
            2.0 * (re * re + im * im).sqrt() / half
        };

        let resp_1k = response_at(1000.0);
        let resp_3k = response_at(3000.0);
        let resp_off = response_at(200.0);

        // Peak response near the individual mode weights.
        approx(resp_1k, 0.7, 0.05);
        approx(resp_3k, 0.3, 0.05);
        assert!(
            resp_off < 0.15,
            "off-resonance response too high: {resp_off}"
        );
    }
}
