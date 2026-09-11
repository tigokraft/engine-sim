//! Acoustic propagation from engine apertures to a listener.
//!
//! Real engine sound does not leave the car from a single point or a static stereo
//! pan. Different parts of the engine radiate from different physical locations,
//! through openings of different sizes, facing in different directions:
//!
//! - **Tailpipes** sit at the rear bumper (or side exits), pointed backward or
//!   downward, radiating high pressure pulses through small exhaust mouths.
//! - **Intake mouth** (snorkel or trumpets) sits at the front grille or under the
//!   hood, pointed into oncoming air, radiating intake rarefaction and throttle hiss.
//! - **Engine block** sits between the front or rear wheels, vibrating as a large
//!   structural casting, radiating combustion knock, mechanical clatter, and piston slap.
//!
//! Because these apertures are metres apart on a vehicle chassis:
//! 1. **Path delays** differ by several milliseconds, creating an audible comb filter
//!    when their sounds combine at the listener.
//! 2. **Inverse-square law** attenuates each aperture by $1/r$ according to its distance.
//! 3. **Air absorption** filters high frequencies over distance.
//! 4. **Aperture directivity** radiates as an omnidirectional monopole below $ka = 1$
//!    ($f_c = c / (2 \pi a)$) and beams forward along its facing normal above $ka = 1$,
//!    attenuating high frequencies off-axis.
//! 5. **Ground reflection** produces acoustic interference between the direct path and the
//!    ground-reflected bounce, creating characteristic cancellation notches.
//! 6. **Per-aperture Doppler** shifts the intake and exhaust frequencies differently
//!    during a pass-by, because the front and rear of the vehicle pass the observer
//!    at different moments.

/// Speed of sound in ambient air at reference conditions (20 °C, 101.3 kPa) [m/s].
pub const SPEED_OF_SOUND_AIR: f32 = 343.2;

/// Standard listener ear separation for interaural time and level differences [m].
pub const LISTENER_EAR_SPACING: f32 = 0.18;

/// Reference distance for 1/r geometric attenuation [m].
pub const REFERENCE_PROPAGATION_DISTANCE: f32 = 1.0;
