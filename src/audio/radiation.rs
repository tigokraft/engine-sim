//! What leaves the pipe: reflection, end correction and the radiated field.
//!
//! Every open end in an exhaust or intake system does three things at once, and
//! all three follow from one number — the mouth radius $a$.
//!
//! 1. **It reflects.** Long waves cannot see a hole a few centimetres across, so
//!    almost all of a low-frequency wave turns around and goes back down the
//!    pipe, inverted, because the mouth is very nearly a pressure release. Short
//!    waves walk straight out. The changeover sits at $ka = 1$, that is
//!    $$f_c = \frac{c}{2 \pi a}$$
//! 2. **It is longer than it looks.** The air just outside the mouth is dragged
//!    along by the wave inside it, so the pipe resonates as though it were
//!    $\delta$ metres longer: [`UNFLANGED_END_CORRECTION`] times the radius for
//!    a plain cut pipe, [`FLANGED_END_CORRECTION`] for one ending in a baffle.
//! 3. **It radiates.** What is not reflected leaves as sound, and a mouth small
//!    against the wavelength is a monopole: the far field goes as the *rate of
//!    change* of the volume it is pumping, so the radiated spectrum rises at
//!    +6 dB/octave below $f_c$ and flattens above it.
//!
//! The third point is the one that a lowpass on the tailpipe gets exactly
//! backwards. Inside the pipe the low end dominates; outside, it is the part
//! that never got out. Tilting the wrong way is the loudest single tell that a
//! note was synthesised rather than recorded, and it costs one filter to fix.

use crate::audio::filters::OnePole;

/// End correction of a plain, unbaffled open pipe, as a multiple of the radius [-].
///
/// Levine and Schwinger's exact result for the unflanged circular pipe in the
/// long-wave limit.
pub const UNFLANGED_END_CORRECTION: f64 = 0.6133;

/// End correction of a pipe ending in an infinite baffle, as a multiple of the
/// radius [-].
///
/// $8 / (3 \pi)$. A flanged mouth loads more air, so it is acoustically longer
/// than an unflanged one of the same bore — a tailpipe cut flush into a
/// bumper valance sits flat rather than sharp against one hanging free.
pub const FLANGED_END_CORRECTION: f64 = 0.8216;

/// First cross-mode cut-on of a circular duct, in units of the mouth corner [-].
///
/// A duct carries a plane wave alone only up to $ka = 1.8412$, the first zero of
/// $J_1'$; above it the (1,0) mode propagates, the field across the bore stops
/// being uniform, and a one-dimensional waveguide has nothing left to say. That
/// ceiling is $1.8412 / (2 \pi a) \cdot c$ — the mouth corner times this
/// number, since both are the same $ka$ measured at different values.
///
/// The radiated path is rolled off there. Not as a tone control: it is the edge
/// of the model's own validity, and above it the excitation the synth is
/// carrying — broadband noise generated at the port — is an extrapolation
/// rather than a prediction. Leaving it flat to Nyquist puts white noise
/// straight out of the tailpipe, which is neither what a pipe does nor what one
/// sounds like.
pub const DUCT_CUT_ON: f32 = 1.8412;

/// Mouth radius the radiated level is expressed against [m].
///
/// A 60 mm tailpipe. The synth carries pressure normalised to
/// [`REFERENCE_BLOWDOWN`](crate::audio::dsp::REFERENCE_BLOWDOWN) rather than in
/// Pascals, so the geometric part of the radiation gain is likewise quoted
/// relative to a reference mouth at [`REFERENCE_DISTANCE`]: a wider pipe or a
/// closer listener is louder in the same proportion the monopole law says, and
/// the reference case comes out at unity instead of at some number of
/// microbars.
pub const REFERENCE_MOUTH_RADIUS: f32 = 0.030;

/// Distance from the mouth the radiated level is expressed at [m].
pub const REFERENCE_DISTANCE: f32 = 1.0;

/// Acoustic end correction $\delta$ of an open pipe [m].
///
/// Add it to the physical length before the transit delay is computed: a pipe
/// of length $L$ resonates at $c / (4 (L + \delta))$, not at $c / 4L$.
#[inline]
pub fn end_correction(radius: f64, flanged: bool) -> f64 {
    let factor = if flanged {
        FLANGED_END_CORRECTION
    } else {
        UNFLANGED_END_CORRECTION
    };
    factor * radius.max(0.0)
}

/// Frequency at which the mouth stops reflecting and starts radiating [Hz]:
///
/// ```text
/// f_c = c / (2 pi a)      (ka = 1)
/// ```
///
/// Below it the mouth is a mirror, above it a window. Note that it moves with
/// the gas: a hot pipe radiates from a corner a third higher than a cold one,
/// which is part of why an exhaust brightens as it warms.
#[inline]
pub fn corner_hz(radius: f32, speed_of_sound: f32) -> f32 {
    speed_of_sound / (std::f32::consts::TAU * radius.max(1e-4))
}

/// Geometric part of the radiation gain, relative to a reference mouth [-].
///
/// A source small against the wavelength radiates the rate of change of the
/// volume it pumps, $p(r) = \rho \dot{U} / (4 \pi r)$, and at an open end
/// $U = 2 A p^+ / (\rho c)$. The differentiation is already carried by the
/// transmission — see [`Mouth::step`] — which brings a factor $\omega_c = c/a$
/// with it, so what is left of the geometry is
///
/// ```text
/// A * omega_c / (2 pi r c) = (pi a^2) / (2 pi r a) = a / (2 r)
/// ```
///
/// Radius over distance, and nothing else: the area is in there, divided by the
/// radius the corner frequency brought along. Quoted against
/// [`REFERENCE_MOUTH_RADIUS`] at [`REFERENCE_DISTANCE`], for the reason given
/// there.
#[inline]
pub fn radiation_gain(radius: f32, distance: f32) -> f32 {
    (radius.max(0.0) / distance.max(1e-3)) * (REFERENCE_DISTANCE / REFERENCE_MOUTH_RADIUS)
}

// ---------------------------------------------------------------------------
// The mouth
// ---------------------------------------------------------------------------

/// An open end: the reflection back down the pipe and the sound that escapes.
///
/// # The reflection
///
/// The Levine–Schwinger result for an unflanged pipe is a transcendental
/// function of $ka$, and no one has to evaluate it per sample to get its
/// behaviour: $|R| \to 1$ below $ka = 1$, falling monotonically to zero above
/// it, with a phase that lags. A first-order fit does all of that,
///
/// ```text
/// R(s) = -1 / (1 + s / omega_c)
/// ```
///
/// a one-pole lowpass with the sign of the pressure-release boundary in front
/// of it. At DC it is exactly $-1$, which is what makes a pipe open at one end
/// resonate at a quarter wave rather than a half; by $f_c$ it is down 3 dB, and
/// an octave above that the mouth is letting most of the wave out instead of
/// sending it back.
///
/// Being exactly unity at DC, the mouth alone is not a contraction: a loop
/// closed on nothing but this would ring forever at zero frequency. Every
/// caller must supply the wall loss and the junction's own reflection
/// magnitude, both of which are strictly below one — see
/// [`ExhaustRunner`](crate::audio::filters::ExhaustRunner), which does.
///
/// # What escapes
///
/// Whatever does not come back has left, so the pressure handed to the outside
/// world is the transmitted part $(1 + R) p^+$. Written out, that fit is
///
/// ```text
/// 1 + R(s) = (s / omega_c) / (1 + s / omega_c)
/// ```
///
/// — a differentiator below the corner and unity above it. This is not a
/// coincidence to be improved on by differentiating a second time: the monopole
/// $\mathrm{d}/\mathrm{d}t$ and the mouth's transmission are the same filter,
/// because the same $ka$ that decides how much gets out decides how efficiently
/// what got out couples to the air. Hence +6 dB/octave below $f_c$, flat above,
/// and a low end that is weak *outside* the pipe however strong it is inside.
#[derive(Debug, Clone, Copy)]
pub struct Mouth {
    radius: f32,
    flanged: bool,
    sample_rate: f32,
    corner_hz: f32,
    gain: f32,
    reflection: OnePole,
    plane_wave: OnePole,
}

impl Mouth {
    /// A mouth of radius `radius` [m] in gas carrying sound at `speed_of_sound`
    /// [m/s].
    ///
    /// `flanged` says whether the end is baffled, which changes nothing but how
    /// much air it drags along — see [`Mouth::end_correction`].
    pub fn new(sample_rate: f32, radius: f32, flanged: bool, speed_of_sound: f32) -> Self {
        let radius = radius.max(1e-4);
        let corner = corner_hz(radius, speed_of_sound);
        Self {
            radius,
            flanged,
            sample_rate,
            corner_hz: corner,
            gain: radiation_gain(radius, REFERENCE_DISTANCE),
            reflection: OnePole::new(sample_rate, corner),
            plane_wave: OnePole::new(sample_rate, DUCT_CUT_ON * corner),
        }
    }

    /// The same mouth heard from `distance` metres away instead of from
    /// [`REFERENCE_DISTANCE`].
    pub fn at_distance(mut self, distance: f32) -> Self {
        self.gain = radiation_gain(self.radius, distance);
        self
    }

    /// Retunes the corner for the current speed of sound [m/s].
    ///
    /// Called at the control rate alongside every other gas-dependent
    /// coefficient. Click-free: a one-pole's state is its output, so moving the
    /// coefficient changes how fast it travels, never where it is.
    pub fn tune(&mut self, speed_of_sound: f32) {
        self.corner_hz = corner_hz(self.radius, speed_of_sound);
        self.reflection.set_cutoff(self.sample_rate, self.corner_hz);
        self.plane_wave
            .set_cutoff(self.sample_rate, DUCT_CUT_ON * self.corner_hz);
    }

    /// Mouth radius [m].
    pub fn radius(&self) -> f32 {
        self.radius
    }

    /// Radiation corner as currently tuned [Hz].
    pub fn corner_hz(&self) -> f32 {
        self.corner_hz
    }

    /// Frequency above which the duct is no longer one-dimensional [Hz].
    pub fn cut_on_hz(&self) -> f32 {
        DUCT_CUT_ON * self.corner_hz
    }

    /// Length the pipe has to be lengthened by to account for this mouth [m].
    ///
    /// The air outside the mouth moves with the wave inside it, and that slug
    /// of air is part of the resonator: a pipe of length $L$ open at one end
    /// stands its fundamental at $c / (4 (L + \delta))$. On a 45 mm primary
    /// that is 14 mm, which is a percent of the length and therefore a percent
    /// of the pitch — small, but it is a systematic flat, and it is free.
    pub fn end_correction(&self) -> f32 {
        end_correction(self.radius as f64, self.flanged) as f32
    }

    /// Mouth area [m^2].
    pub fn area(&self) -> f32 {
        std::f32::consts::PI * self.radius * self.radius
    }

    /// Radiation gain currently in effect [-].
    pub fn gain(&self) -> f32 {
        self.gain
    }

    /// The wave that turns around and goes back down the pipe, given the one
    /// arriving at the mouth.
    #[inline(always)]
    pub fn reflect(&mut self, incident: f32) -> f32 {
        -self.reflection.process(incident)
    }

    /// Both sides of the mouth at once, given the wave arriving at it.
    ///
    /// Returns `(reflected, radiated)`: the wave sent back down the pipe, and
    /// the sound that leaves it. The second is the first added to the incident
    /// wave — the transmission $(1 + R) p^+$ of the type docs — scaled by
    /// [`radiation_gain`] and rolled off above the duct's own cut-on, for the
    /// reason given at [`DUCT_CUT_ON`].
    #[inline(always)]
    pub fn step(&mut self, incident: f32) -> (f32, f32) {
        let reflected = self.reflect(incident);
        let transmitted = (incident + reflected) * self.gain;
        (reflected, self.plane_wave.process(transmitted))
    }

    /// Clears the filter state.
    pub fn reset(&mut self) {
        self.reflection.reset();
        self.plane_wave.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::audio::filters::DelayLine;
    use std::f32::consts::TAU;

    const FS: f32 = 48_000.0;

    /// Ambient air, for tests that care about a number rather than about hot
    /// gas [m/s].
    const C: f32 = 343.0;

    /// Steady-state magnitude of a system driven by a sine at `hz` [-].
    ///
    /// Settles for a quarter of a second before it starts correlating, so what
    /// comes back is the response and not the transient on the way to it.
    fn magnitude_at(
        hz: f32,
        settle: usize,
        measure: usize,
        mut step: impl FnMut(f32) -> f32,
    ) -> f32 {
        let (mut re, mut im) = (0.0f64, 0.0f64);
        for i in 0..settle + measure {
            let phase = TAU * hz * i as f32 / FS;
            let y = step(phase.sin());
            if i >= settle {
                re += y as f64 * phase.sin() as f64;
                im += y as f64 * phase.cos() as f64;
            }
        }
        (2.0 * (re * re + im * im).sqrt() / measure as f64) as f32
    }

    fn db(ratio: f32) -> f32 {
        20.0 * ratio.max(1e-12).log10()
    }

    #[test]
    fn reflection_falls_monotonically_through_ka_one() {
        let radius = 0.030;
        let corner = corner_hz(radius, C);
        let mut previous = f32::MAX;
        for octave in [-3i32, -2, -1, 0, 1, 2, 3] {
            let f = corner * 2.0f32.powi(octave);
            let mut mouth = Mouth::new(FS, radius, false, C);
            let magnitude = magnitude_at(f, 4_000, 8_000, |x| mouth.reflect(x));
            assert!(
                magnitude < previous,
                "|R| rose at {f:.0} Hz: {magnitude:.4} after {previous:.4}"
            );
            previous = magnitude;
            // Three octaves below the corner the mouth is a mirror; three
            // above, a window.
            match octave {
                -3 => assert!(magnitude > 0.98, "leaky at ka = 1/8: {magnitude:.4}"),
                0 => approx(magnitude, 1.0 / 2.0f32.sqrt(), 0.02),
                3 => assert!(
                    magnitude < 0.15,
                    "still reflecting at ka = 8: {magnitude:.4}"
                ),
                _ => {}
            }
        }
    }

    #[test]
    fn end_correction_lowers_the_pipe_fundamental() {
        // A pipe closed at one end and open at the other, built from nothing
        // but a round-trip delay and the mouth: no wall loss, no collector, so
        // the only thing that can move the resonance is the end condition.
        // The 0.95 is that missing wall loss, flat with frequency so it cannot
        // shift the peak, present so the loop settles inside the measurement.
        let length = 0.60;
        let radius = 0.020;
        let delta = end_correction(radius as f64, false) as f32;
        let round_trip = |acoustic: f32| 2.0 * acoustic / C * FS;

        let mut mouth = Mouth::new(FS, radius, false, C);
        let mut line = DelayLine::with_max_delay(round_trip(length + delta).ceil() as usize + 8);
        let delay = round_trip(length + delta);
        let mut resonance = |hz: f32| {
            mouth.reset();
            line.reset();
            magnitude_at(hz, 24_000, 24_000, |x| {
                let out = line.read(delay);
                line.push(x + 0.95 * mouth.reflect(out));
                out
            })
        };

        let uncorrected = C / (4.0 * length);
        let corrected = C / (4.0 * (length + delta));
        let mut peak = (0.0f32, 0.0f32);
        let mut hz = uncorrected * 0.85;
        while hz <= uncorrected * 1.05 {
            let m = resonance(hz);
            if m > peak.1 {
                peak = (hz, m);
            }
            hz += 0.1;
        }
        let measured = peak.0;

        // The claim: the pipe stands its fundamental where the corrected length
        // says, not where a tape measure says. Measured against both, the
        // corrected prediction is the better fit — and it must be, because the
        // air outside the mouth is part of the resonator.
        assert!(
            (measured - corrected).abs() < (measured - uncorrected).abs(),
            "measured {measured:.2} Hz is nearer c/4L ({uncorrected:.2}) than \
             c/4(L+delta) ({corrected:.2})"
        );
        // Within two percent of the corrected prediction. It sits a shade
        // below even that: the reflection fit lags as well as attenuates, and
        // that lag is more length again.
        assert!(
            (measured - corrected).abs() / corrected < 0.02,
            "measured {measured:.2} Hz against {corrected:.2} Hz"
        );
    }

    #[test]
    fn radiated_tilt_is_six_db_per_octave() {
        let radius = 0.030;
        let corner = corner_hz(radius, C);
        let radiated = |hz: f32| {
            let mut mouth = Mouth::new(FS, radius, false, C);
            magnitude_at(hz, 4_000, 8_000, |x| mouth.step(x).1)
        };

        // Well below the corner the mouth is a differentiator: an octave up is
        // 6 dB up, because a monopole radiates the rate of change of what it
        // pumps and not the amount.
        let low = radiated(corner / 32.0);
        let low_octave_up = radiated(corner / 16.0);
        approx(db(low_octave_up) - db(low), 6.02, 0.3);

        // Above it, everything gets out and the tilt is gone. Measured
        // between the corner and the duct's cut-on, which is where the flat
        // stretch lives: 0.58 of an octave, over which a differentiator would
        // still be climbing 3.5 dB.
        let flat_low = radiated(corner * 1.2);
        let flat_high = radiated(corner * 1.8);
        assert!(
            (db(flat_high) - db(flat_low)).abs() < 1.0,
            "still tilting above the corner: {:.2} dB",
            db(flat_high) - db(flat_low)
        );
    }

    #[test]
    fn a_wider_mouth_radiates_more_low_end() {
        let narrow = 0.025;
        let wide = 2.0 * narrow;
        // Doubling the radius drops the corner by an octave: ka = 1 arrives at
        // half the frequency.
        approx(corner_hz(wide, C), corner_hz(narrow, C) / 2.0, 1e-3);

        // A decade below either corner, where both are still mirrors, the wide
        // mouth is 12 dB louder: 6 dB for a corner an octave lower, and 6 dB
        // again for a mouth twice the size.
        //
        // To within a decibel, not to within a tenth. The reflection is a
        // sampled one-pole placed by its pole rather than by its -3 dB point,
        // and that costs a fraction of a decibel of transmission at the bottom
        // — more for the narrow mouth, whose corner is a larger fraction of the
        // sample rate, which is why the measured difference runs a little over
        // the analytic one.
        let hz = corner_hz(wide, C) / 10.0;
        let level = |radius: f32| {
            let mut mouth = Mouth::new(FS, radius, false, C);
            magnitude_at(hz, 4_000, 8_000, |x| mouth.step(x).1)
        };
        approx(db(level(wide)) - db(level(narrow)), 12.04, 1.0);
    }

    fn approx(value: f32, expected: f32, tolerance: f32) {
        assert!(
            (value - expected).abs() <= tolerance,
            "{value} is not within {tolerance} of {expected}"
        );
    }
}
