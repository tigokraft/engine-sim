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
    }

    /// Mouth radius [m].
    pub fn radius(&self) -> f32 {
        self.radius
    }

    /// Radiation corner as currently tuned [Hz].
    pub fn corner_hz(&self) -> f32 {
        self.corner_hz
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
    /// [`radiation_gain`].
    #[inline(always)]
    pub fn step(&mut self, incident: f32) -> (f32, f32) {
        let reflected = self.reflect(incident);
        (reflected, (incident + reflected) * self.gain)
    }

    /// Clears the filter state.
    pub fn reset(&mut self) {
        self.reflection.reset();
    }
}
