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

use std::f32::consts::PI;

use crate::audio::filters::DelayLine;

/// Speed of sound in ambient air at reference conditions (20 °C, 101.3 kPa) [m/s].
pub const SPEED_OF_SOUND_AIR: f32 = 343.2;

/// Standard listener ear separation for interaural time and level differences [m].
pub const LISTENER_EAR_SPACING: f32 = 0.18;

/// Reference distance for 1/r geometric attenuation [m].
pub const REFERENCE_PROPAGATION_DISTANCE: f32 = 1.0;

/// Maximum propagation distance supported by internal delay lines [m].
pub const MAX_PROPAGATION_DISTANCE: f32 = 40.0;

/// Normalizes a 3D vector to unit length.
#[inline]
pub fn normalize(v: [f32; 3]) -> [f32; 3] {
    let norm = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if norm > 1e-6 {
        [v[0] / norm, v[1] / norm, v[2] / norm]
    } else {
        [0.0, 1.0, 0.0]
    }
}

/// Dot product between two 3D vectors.
#[inline]
pub fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Euclidean distance between two 3D points [m].
#[inline]
pub fn distance(a: [f32; 3], b: [f32; 3]) -> f32 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Propagation delay in seconds for a given path length [s].
#[inline]
pub fn path_delay_seconds(distance: f32, c: f32) -> f32 {
    distance / c.max(1.0)
}

/// Propagation delay in samples for a given path length [samples].
#[inline]
pub fn path_delay_samples(distance: f32, c: f32, sample_rate: f32) -> f32 {
    path_delay_seconds(distance, c) * sample_rate
}

/// An acoustic radiating aperture on the engine / vehicle.
///
/// Every source that couples to the outside air does so through an aperture with
/// a physical location, radiating cross-sectional area, and outward normal facing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aperture {
    /// 3D position in vehicle coordinates [m] (X right, Y forward, Z up).
    pub position: [f32; 3],
    /// Radiating cross-sectional area [m^2].
    pub area: f32,
    /// Facing normal vector pointing outward from the opening (unit vector).
    pub facing: [f32; 3],
}

impl Aperture {
    /// Creates a new aperture with a position, area, and facing vector.
    pub fn new(position: [f32; 3], area: f32, facing: [f32; 3]) -> Self {
        Self {
            position,
            area: area.max(1e-6),
            facing: normalize(facing),
        }
    }

    /// Equivalent radius $a = \sqrt{A / \pi}$ of a circular aperture of this area [m].
    #[inline]
    pub fn radius(&self) -> f32 {
        (self.area / PI).sqrt()
    }

    /// Corner frequency $f_c = c / (2 \pi a)$ where $ka = 1$ [Hz].
    ///
    /// Below this frequency the aperture is smaller than the wavelength and acts
    /// as an omnidirectional monopole. Above this frequency radiation beams
    /// forward along the aperture's facing normal.
    #[inline]
    pub fn corner_hz(&self, c: f32) -> f32 {
        c / (2.0 * PI * self.radius().max(1e-4))
    }

    /// 3D Euclidean distance from this aperture to a target point [m].
    #[inline]
    pub fn distance_to(&self, target: [f32; 3]) -> f32 {
        distance(self.position, target)
    }

    /// Normalized direction unit vector pointing from this aperture towards `target`.
    #[inline]
    pub fn direction_to(&self, target: [f32; 3]) -> [f32; 3] {
        let dx = target[0] - self.position[0];
        let dy = target[1] - self.position[1];
        let dz = target[2] - self.position[2];
        normalize([dx, dy, dz])
    }

    /// Angle $\theta \in [0, \pi]$ between the aperture facing and the direction to `target` [rad].
    ///
    /// $\theta = 0$ is on-axis (directly in front of the opening);
    /// $\theta = \pi$ is 180° off-axis (behind the aperture).
    #[inline]
    pub fn angle_to(&self, target: [f32; 3]) -> f32 {
        let dir = self.direction_to(target);
        let cos_theta = dot(self.facing, dir).clamp(-1.0, 1.0);
        cos_theta.acos()
    }
}

/// Listener position and ear geometry in vehicle coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Listener {
    /// Position of the listener's head center [m] (X right, Y forward, Z up).
    pub position: [f32; 3],
    /// Ear-to-ear spacing along the lateral axis [m].
    pub ear_spacing: f32,
}

impl Default for Listener {
    /// Default spectator position 3.5 m behind the vehicle, 1.2 m above ground.
    fn default() -> Self {
        Self {
            position: [0.0, -3.5, 1.2],
            ear_spacing: LISTENER_EAR_SPACING,
        }
    }
}

impl Listener {
    /// Creates a new listener at `position` with standard ear separation.
    pub fn new(position: [f32; 3]) -> Self {
        Self {
            position,
            ear_spacing: LISTENER_EAR_SPACING,
        }
    }

    /// Left and right ear 3D positions `(left, right)` in world/vehicle coordinates [m].
    #[inline]
    pub fn ears(&self) -> ([f32; 3], [f32; 3]) {
        let half = self.ear_spacing * 0.5;
        let left = [self.position[0] - half, self.position[1], self.position[2]];
        let right = [self.position[0] + half, self.position[1], self.position[2]];
        (left, right)
    }
}

/// Propagation path from one aperture to the listener's ears.
///
/// Owns its delay lines sized for [`MAX_PROPAGATION_DISTANCE`].
#[derive(Debug, Clone)]
pub struct AperturePath {
    pub aperture: Aperture,
    sample_rate: f32,
    left_delay: DelayLine,
    right_delay: DelayLine,
}

impl AperturePath {
    /// Creates a new path for `aperture` at the given sample rate.
    pub fn new(aperture: Aperture, sample_rate: f32) -> Self {
        let max_samples =
            (MAX_PROPAGATION_DISTANCE / SPEED_OF_SOUND_AIR * sample_rate) as usize + 64;
        Self {
            aperture,
            sample_rate,
            left_delay: DelayLine::with_max_delay(max_samples),
            right_delay: DelayLine::with_max_delay(max_samples),
        }
    }

    /// Clears internal delay lines.
    pub fn reset(&mut self) {
        self.left_delay.reset();
        self.right_delay.reset();
    }

    /// Delays one input sample by the physical path length to each ear.
    ///
    /// Returns stereo `(left, right)` delayed samples.
    #[inline]
    pub fn step_delay(&mut self, sample: f32, listener: &Listener) -> (f32, f32) {
        let (left_ear, right_ear) = listener.ears();
        let r_left = self.aperture.distance_to(left_ear);
        let r_right = self.aperture.distance_to(right_ear);

        let d_left = (1.0 + path_delay_samples(r_left, SPEED_OF_SOUND_AIR, self.sample_rate))
            .clamp(1.0, self.left_delay.max_delay());
        let d_right = (1.0 + path_delay_samples(r_right, SPEED_OF_SOUND_AIR, self.sample_rate))
            .clamp(1.0, self.right_delay.max_delay());

        self.left_delay.push(sample);
        self.right_delay.push(sample);

        (self.left_delay.read(d_left), self.right_delay.read(d_right))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aperture_geometry_and_angles() {
        let tailpipe = Aperture::new([0.0, -2.0, 0.3], PI * 0.03 * 0.03, [0.0, -1.0, 0.0]);
        assert!((tailpipe.radius() - 0.03).abs() < 1e-4);

        // A listener directly behind the tailpipe is on-axis (angle ~ 0).
        let behind = [0.0, -5.0, 0.3];
        assert!(tailpipe.angle_to(behind) < 1e-4);
        assert!((tailpipe.distance_to(behind) - 3.0).abs() < 1e-4);

        // A listener directly in front is 180 degrees off-axis (angle ~ PI).
        let in_front = [0.0, 2.0, 0.3];
        assert!((tailpipe.angle_to(in_front) - PI).abs() < 1e-4);

        // A listener alongside (lateral) is 90 degrees off-axis (angle ~ PI/2).
        let alongside = [3.0, -2.0, 0.3];
        assert!((tailpipe.angle_to(alongside) - PI * 0.5).abs() < 1e-4);
    }

    #[test]
    fn listener_ears_are_separated_laterally() {
        let listener = Listener::new([0.0, -3.0, 1.2]);
        let (l, r) = listener.ears();
        assert!((r[0] - l[0] - LISTENER_EAR_SPACING).abs() < 1e-5);
        assert_eq!(l[1], -3.0);
        assert_eq!(r[1], -3.0);
        assert_eq!(l[2], 1.2);
        assert_eq!(r[2], 1.2);
    }

    #[test]
    fn aperture_delays_match_r_over_c_and_sum_shows_comb() {
        let sample_rate = 48_000.0;
        let c = SPEED_OF_SOUND_AIR; // 343.2 m/s

        // Test 1: Impulse delay matches r / c.
        // Choose distance 3.432 m -> delay = 3.432 / 343.2 = 0.010 s = 480 samples.
        let dist = 3.432;
        let aperture = Aperture::new([0.0, 0.0, 1.0], 0.01, [0.0, -1.0, 0.0]);
        let listener = Listener {
            position: [0.0, -dist, 1.0],
            ear_spacing: 0.0, // Monoaural for exact sample count assertion
        };
        let mut path = AperturePath::new(aperture, sample_rate);

        let expected_samples = (dist / c * sample_rate).round() as usize; // 480 samples
        let mut peak_sample = 0;
        let mut peak_val = 0.0f32;

        for n in 0..1000 {
            let input = if n == 0 { 1.0 } else { 0.0 };
            let (l, _) = path.step_delay(input, &listener);
            if l.abs() > peak_val {
                peak_val = l.abs();
                peak_sample = n;
            }
        }
        assert_eq!(
            peak_sample, expected_samples,
            "Impulse should arrive at exactly r / c samples"
        );

        // Test 2: Two apertures with path difference produce expected comb cancellation.
        // Aperture A at [0, 0, 1], distance 5 m.
        // Aperture B at [0, 1, 1], distance 6 m.
        // Path difference = 1.0 m -> delay difference = 1.0 / 343.2 s.
        // Comb notch frequency: f_notch = c / (2 * delta_r) = 343.2 / 2 = 171.6 Hz.
        // Comb peak frequency: f_peak = c / delta_r = 343.2 Hz.
        let ap_a = Aperture::new([0.0, 0.0, 1.0], 0.01, [0.0, -1.0, 0.0]);
        let ap_b = Aperture::new([0.0, 1.0, 1.0], 0.01, [0.0, -1.0, 0.0]);
        let listener_comb = Listener {
            position: [0.0, -5.0, 1.0],
            ear_spacing: 0.0,
        };

        let measure_response_at = |freq: f32| -> f32 {
            let mut path_a = AperturePath::new(ap_a, sample_rate);
            let mut path_b = AperturePath::new(ap_b, sample_rate);
            let mut max_sum = 0.0f32;
            for n in 0..4000 {
                let t = n as f32 / sample_rate;
                let sig = (2.0 * PI * freq * t).sin();
                let (la, _) = path_a.step_delay(sig, &listener_comb);
                let (lb, _) = path_b.step_delay(sig, &listener_comb);
                if n > 2000 {
                    let sum = (la + lb).abs();
                    if sum > max_sum {
                        max_sum = sum;
                    }
                }
            }
            max_sum
        };

        let f_notch = c / 2.0; // 171.6 Hz
        let f_peak = c; // 343.2 Hz
        let level_at_notch = measure_response_at(f_notch);
        let level_at_peak = measure_response_at(f_peak);

        assert!(
            level_at_notch < 0.05,
            "Comb notch at f = c / (2 * delta_r) should cancel, got {}",
            level_at_notch
        );
        assert!(
            level_at_peak > 1.90,
            "Comb peak at f = c / delta_r should reinforce to ~2.0, got {}",
            level_at_peak
        );
    }
}
