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

use crate::audio::filters::{DelayLine, OnePole};

/// Speed of sound in ambient air at reference conditions (20 °C, 101.3 kPa) [m/s].
pub const SPEED_OF_SOUND_AIR: f32 = 343.2;

/// Standard listener ear separation for interaural time and level differences [m].
pub const LISTENER_EAR_SPACING: f32 = 0.18;

/// Reference distance for 1/r geometric attenuation [m].
pub const REFERENCE_PROPAGATION_DISTANCE: f32 = 1.0;

/// Maximum propagation distance supported by internal delay lines [m].
pub const MAX_PROPAGATION_DISTANCE: f32 = 40.0;

/// Base cutoff frequency for air absorption at reference distance (1 m) [Hz].
pub const AIR_ABSORPTION_BASE_HZ: f32 = 20_000.0;

/// Air absorption rolloff rate per metre of propagation distance [1/m].
pub const AIR_ABSORPTION_RATE: f32 = 0.05;

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

/// High-frequency cutoff for atmospheric air absorption over distance $r$ [Hz].
///
/// Models the progressive high-frequency attenuation of sound in air (ISO 9613-1).
#[inline]
pub fn air_absorption_cutoff(distance: f32) -> f32 {
    let excess = (distance - REFERENCE_PROPAGATION_DISTANCE).max(0.0);
    AIR_ABSORPTION_BASE_HZ / (1.0 + AIR_ABSORPTION_RATE * excess)
}

/// Directivity cutoff frequency for an aperture of corner frequency $f_c$ at angle $\theta$ [Hz].
///
/// An open mouth is a monopole below $ka = 1$ ($f \le f_c$) and beams along its facing
/// normal above $ka = 1$. Off-axis listeners receive full low-end but progressively
/// lose high frequencies.
#[inline]
pub fn directivity_cutoff(angle_rad: f32, corner_hz: f32, sample_rate: f32) -> f32 {
    let max_cutoff = 0.45 * sample_rate;
    if corner_hz >= max_cutoff {
        return max_cutoff;
    }
    let half_angle = angle_rad.clamp(0.0, PI) * 0.5;
    let s = half_angle.sin();
    if s < 1e-3 {
        max_cutoff
    } else {
        (corner_hz / s).clamp(corner_hz, max_cutoff)
    }
}

/// Geometric distance attenuation factor $r_0 / r$ [-].
///
/// Follows the inverse-square law for sound intensity, which corresponds to
/// $1/r$ for acoustic pressure in the free field.
#[inline]
pub fn distance_attenuation(distance: f32) -> f32 {
    REFERENCE_PROPAGATION_DISTANCE / distance.max(0.1)
}

/// Default pressure reflection coefficient of hard ground (asphalt/road) [-].
pub const GROUND_REFLECTION_COEFF: f32 = 0.8;

/// Ground bounce path difference between reflected and direct paths [m]:
///
/// ```text
/// delta = sqrt((h_s + h_r)^2 + d^2) - sqrt((h_s - h_r)^2 + d^2)
/// ```
#[inline]
pub fn ground_path_difference(source_pos: [f32; 3], receiver_pos: [f32; 3]) -> f32 {
    let hs = source_pos[2].max(0.01);
    let hr = receiver_pos[2].max(0.01);
    let dx = source_pos[0] - receiver_pos[0];
    let dy = source_pos[1] - receiver_pos[1];
    let d2 = dx * dx + dy * dy;

    let r_direct = ((hs - hr) * (hs - hr) + d2).sqrt();
    let r_reflected = ((hs + hr) * (hs + hr) + d2).sqrt();

    (r_reflected - r_direct).max(0.0)
}

/// Interference cancellation notch frequency predicted by ground reflection [Hz]:
///
/// ```text
/// f_notch = c / (2 * delta)
/// ```
#[inline]
pub fn ground_notch_hz(delta_r: f32, c: f32) -> f32 {
    c / (2.0 * delta_r.max(1e-4))
}

/// Doppler pitch shift factor $c / (c - v_r)$ [-]:
///
/// ```text
/// f' = f * c / (c - v_r)
/// ```
///
/// where $v_r$ is the radial velocity of the source towards the observer [m/s].
#[inline]
pub fn doppler_factor(v_r: f32, c: f32) -> f32 {
    let clamped = v_r.clamp(-0.8 * c, 0.8 * c);
    c / (c - clamped)
}

/// Relative radial velocity of a source at `source_pos` moving with `velocity`
/// towards `target_pos` [m/s].
///
/// Positive when approaching the observer, negative when receding.
#[inline]
pub fn aperture_radial_velocity(
    source_pos: [f32; 3],
    velocity: [f32; 3],
    target_pos: [f32; 3],
) -> f32 {
    let dx = target_pos[0] - source_pos[0];
    let dy = target_pos[1] - source_pos[1];
    let dz = target_pos[2] - source_pos[2];
    let dir = normalize([dx, dy, dz]);
    dot(velocity, dir)
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
    /// Default listener position 1.8 m behind vehicle center, 1.2 m above ground.
    fn default() -> Self {
        Self {
            position: [1.0, -0.5, 1.2],
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
/// Owns its delay lines sized for [`MAX_PROPAGATION_DISTANCE`] and air absorption filters.
#[derive(Debug, Clone)]
pub struct AperturePath {
    pub aperture: Aperture,
    sample_rate: f32,
    left_delay: DelayLine,
    right_delay: DelayLine,
    left_air: OnePole,
    right_air: OnePole,
    left_directivity: OnePole,
    right_directivity: OnePole,
    left_ground: DelayLine,
    right_ground: DelayLine,
    pub ground_reflection: bool,
    /// Velocity vector of the aperture [m/s] (X right, Y forward, Z up).
    pub velocity: [f32; 3],
}

impl AperturePath {
    /// Creates a new path for `aperture` at the given sample rate.
    pub fn new(aperture: Aperture, sample_rate: f32) -> Self {
        let max_samples =
            (MAX_PROPAGATION_DISTANCE / SPEED_OF_SOUND_AIR * sample_rate) as usize + 64;
        let max_ground_samples = (10.0 / SPEED_OF_SOUND_AIR * sample_rate) as usize + 64;
        Self {
            aperture,
            sample_rate,
            left_delay: DelayLine::with_max_delay(max_samples),
            right_delay: DelayLine::with_max_delay(max_samples),
            left_air: OnePole::new(sample_rate, AIR_ABSORPTION_BASE_HZ),
            right_air: OnePole::new(sample_rate, AIR_ABSORPTION_BASE_HZ),
            left_directivity: OnePole::new(sample_rate, 0.45 * sample_rate),
            right_directivity: OnePole::new(sample_rate, 0.45 * sample_rate),
            left_ground: DelayLine::with_max_delay(max_ground_samples),
            right_ground: DelayLine::with_max_delay(max_ground_samples),
            ground_reflection: true,
            velocity: [0.0, 0.0, 0.0],
        }
    }

    /// Sets whether ground reflection interference is modelled.
    pub fn with_ground_reflection(mut self, enabled: bool) -> Self {
        self.ground_reflection = enabled;
        self
    }

    /// Sets the velocity vector of the aperture [m/s].
    pub fn with_velocity(mut self, velocity: [f32; 3]) -> Self {
        self.velocity = velocity;
        self
    }

    /// Advances aperture position by `dt` seconds according to its velocity vector.
    #[inline]
    pub fn update_motion(&mut self, dt: f32) {
        self.aperture.position[0] += self.velocity[0] * dt;
        self.aperture.position[1] += self.velocity[1] * dt;
        self.aperture.position[2] += self.velocity[2] * dt;
    }

    /// Returns the Doppler factor of this aperture towards a receiver position [-].
    #[inline]
    pub fn doppler_factor_to(&self, receiver_pos: [f32; 3]) -> f32 {
        let vr = aperture_radial_velocity(self.aperture.position, self.velocity, receiver_pos);
        doppler_factor(vr, SPEED_OF_SOUND_AIR)
    }

    pub fn reset(&mut self) {
        self.left_delay.reset();
        self.right_delay.reset();
        self.left_air.reset();
        self.right_air.reset();
        self.left_directivity.reset();
        self.right_directivity.reset();
        self.left_ground.reset();
        self.right_ground.reset();
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

    /// Propagates sample through ground reflection, path delay, directivity, air absorption, and distance attenuation.
    #[inline]
    pub fn step_propagated(&mut self, sample: f32, listener: &Listener) -> (f32, f32) {
        let (left_ear, right_ear) = listener.ears();
        let r_left = self.aperture.distance_to(left_ear);
        let r_right = self.aperture.distance_to(right_ear);
        let angle_left = self.aperture.angle_to(left_ear);
        let angle_right = self.aperture.angle_to(right_ear);

        let (ground_left, ground_right) =
            if self.ground_reflection && self.aperture.position[2] > 0.0 && left_ear[2] > 0.0 {
                let delta_r_left = ground_path_difference(self.aperture.position, left_ear);
                let delta_r_right = ground_path_difference(self.aperture.position, right_ear);

                let d_ground_left = (1.0
                    + path_delay_samples(delta_r_left, SPEED_OF_SOUND_AIR, self.sample_rate))
                .clamp(1.0, self.left_ground.max_delay());
                let d_ground_right = (1.0
                    + path_delay_samples(delta_r_right, SPEED_OF_SOUND_AIR, self.sample_rate))
                .clamp(1.0, self.right_ground.max_delay());

                self.left_ground.push(sample);
                self.right_ground.push(sample);

                (
                    sample + GROUND_REFLECTION_COEFF * self.left_ground.read(d_ground_left),
                    sample + GROUND_REFLECTION_COEFF * self.right_ground.read(d_ground_right),
                )
            } else {
                (sample, sample)
            };

        let d_left = (1.0 + path_delay_samples(r_left, SPEED_OF_SOUND_AIR, self.sample_rate))
            .clamp(1.0, self.left_delay.max_delay());
        let d_right = (1.0 + path_delay_samples(r_right, SPEED_OF_SOUND_AIR, self.sample_rate))
            .clamp(1.0, self.right_delay.max_delay());

        self.left_delay.push(ground_left);
        self.right_delay.push(ground_right);

        let del_left = self.left_delay.read(d_left);
        let del_right = self.right_delay.read(d_right);

        let fc = self.aperture.corner_hz(SPEED_OF_SOUND_AIR);
        self.left_directivity.set_cutoff(
            self.sample_rate,
            directivity_cutoff(angle_left, fc, self.sample_rate),
        );
        self.right_directivity.set_cutoff(
            self.sample_rate,
            directivity_cutoff(angle_right, fc, self.sample_rate),
        );

        let dir_left = self.left_directivity.process(del_left);
        let dir_right = self.right_directivity.process(del_right);

        self.left_air
            .set_cutoff(self.sample_rate, air_absorption_cutoff(r_left));
        self.right_air
            .set_cutoff(self.sample_rate, air_absorption_cutoff(r_right));

        let left = self.left_air.process(dir_left) * distance_attenuation(r_left);
        let right = self.right_air.process(dir_right) * distance_attenuation(r_right);

        (left, right)
    }

    /// Backwards-compatible alias for [`Self::step_propagated`].
    #[inline]
    pub fn step_attenuated(&mut self, sample: f32, listener: &Listener) -> (f32, f32) {
        self.step_propagated(sample, listener)
    }
}

/// Physical radiating aperture locations on the vehicle chassis [m] (X right, Y forward, Z up).
#[derive(Debug, Clone, PartialEq)]
pub struct AperturePositions {
    /// 3D position of each exhaust tailpipe [m].
    pub tailpipes: Vec<[f64; 3]>,
    /// 3D position of intake mouth (snorkel / airbox inlet / trumpets) [m].
    pub intake: [f64; 3],
    /// 3D position of engine block center of mass [m].
    pub block: [f64; 3],
}

impl AperturePositions {
    /// Standard front-engine single-exhaust layout (e.g. inline-4, inline-6).
    pub fn front_engine_single() -> Self {
        Self {
            tailpipes: vec![[0.35, -2.2, 0.35]],
            intake: [0.2, 1.3, 0.65],
            block: [0.0, 0.8, 0.5],
        }
    }

    /// Standard front-engine dual-exhaust layout (e.g. cross-plane V8, twin-turbo V8, V12).
    pub fn front_engine_dual() -> Self {
        Self {
            tailpipes: vec![[-0.45, -2.4, 0.35], [0.45, -2.4, 0.35]],
            intake: [0.0, 1.4, 0.65],
            block: [0.0, 0.8, 0.5],
        }
    }

    /// Mid-engine dual-exhaust layout (e.g. flat-plane V8, V10).
    pub fn mid_engine_dual() -> Self {
        Self {
            tailpipes: vec![[-0.35, -2.1, 0.45], [0.35, -2.1, 0.45]],
            intake: [0.0, 0.25, 0.75],
            block: [0.0, -0.4, 0.45],
        }
    }

    /// Front-mid rotary layout (e.g. RX-7).
    pub fn rotary() -> Self {
        Self {
            tailpipes: vec![[0.40, -2.0, 0.32]],
            intake: [-0.2, 1.1, 0.60],
            block: [0.0, 0.5, 0.40],
        }
    }
}

impl Default for AperturePositions {
    fn default() -> Self {
        Self::front_engine_dual()
    }
}

/// Acoustic propagation model placing vehicle radiator apertures in 3D space relative to a listener.
#[derive(Debug, Clone)]
pub struct PropagationModel {
    /// Listener position and ear geometry.
    pub listener: Listener,
    /// Acoustic paths from each exhaust tailpipe.
    pub tailpipe_paths: Vec<AperturePath>,
    /// Acoustic path from intake mouth.
    pub intake_path: AperturePath,
    /// Acoustic path from engine block structure.
    pub block_path: AperturePath,
}

impl PropagationModel {
    /// Creates a new propagation model from apertures and a listener at `sample_rate`.
    pub fn new(
        listener: Listener,
        tailpipes: Vec<Aperture>,
        intake: Aperture,
        block: Aperture,
        sample_rate: f32,
    ) -> Self {
        let tailpipe_paths = tailpipes
            .into_iter()
            .map(|ap| AperturePath::new(ap, sample_rate))
            .collect();
        let intake_path = AperturePath::new(intake, sample_rate);
        let block_path = AperturePath::new(block, sample_rate);
        Self {
            listener,
            tailpipe_paths,
            intake_path,
            block_path,
        }
    }

    /// Resets all internal delay lines and filters.
    pub fn reset(&mut self) {
        for path in &mut self.tailpipe_paths {
            path.reset();
        }
        self.intake_path.reset();
        self.block_path.reset();
    }

    /// Sets common vehicle velocity vector [m/s] for all engine apertures.
    pub fn set_velocity(&mut self, velocity: [f32; 3]) {
        for path in &mut self.tailpipe_paths {
            path.velocity = velocity;
        }
        self.intake_path.velocity = velocity;
        self.block_path.velocity = velocity;
    }

    /// Advances the position of all apertures by `dt` seconds according to their velocities.
    pub fn update_motion(&mut self, dt: f32) {
        for path in &mut self.tailpipe_paths {
            path.update_motion(dt);
        }
        self.intake_path.update_motion(dt);
        self.block_path.update_motion(dt);
    }

    /// Propagates sound from all apertures to the listener's ears.
    ///
    /// - `tailpipe_pressures`: Radiated pressure from each tailpipe [Pa].
    /// - `intake_pressure`: Radiated pressure from intake opening [Pa].
    /// - `block_pressure`: Radiated structural sound from engine block [Pa].
    ///
    /// Returns stereo `(left, right)` acoustic pressure at listener's ears.
    #[inline]
    pub fn step(
        &mut self,
        tailpipe_pressures: &[f32],
        intake_pressure: f32,
        block_pressure: f32,
    ) -> (f32, f32) {
        let mut left = 0.0f32;
        let mut right = 0.0f32;

        let num_paths = self.tailpipe_paths.len().max(1);
        for (i, path) in self.tailpipe_paths.iter_mut().enumerate() {
            let mut sample = 0.0f32;
            for (bank_idx, &p) in tailpipe_pressures.iter().enumerate() {
                if bank_idx % num_paths == i {
                    sample += p;
                }
            }
            let (l, r) = path.step_propagated(sample, &self.listener);
            left += l;
            right += r;
        }

        let (in_l, in_r) = self
            .intake_path
            .step_propagated(intake_pressure, &self.listener);
        left += in_l;
        right += in_r;

        let (bl_l, bl_r) = self
            .block_path
            .step_propagated(block_pressure, &self.listener);
        left += bl_l;
        right += bl_r;

        (left, right)
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
        let dist = 3.432;
        let aperture = Aperture::new([0.0, 0.0, 1.0], 0.01, [0.0, -1.0, 0.0]);
        let listener = Listener {
            position: [0.0, -dist, 1.0],
            ear_spacing: 0.0, // Monoaural for exact sample count assertion
        };
        let mut path = AperturePath::new(aperture, sample_rate).with_ground_reflection(false);

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

    #[test]
    fn total_level_falls_as_one_over_r_with_distance() {
        let sample_rate = 48_000.0;
        let aperture = Aperture::new([0.0, 0.0, 1.0], 0.01, [0.0, -1.0, 0.0]);

        // Measure low-frequency sine amplitude at 1m, 2m, and 4m distance.
        // At 80 Hz, air absorption is negligible, isolating the geometric 1/r law.
        let freq = 80.0;
        let distances = [1.0f32, 2.0, 4.0];
        let mut amplitudes = Vec::new();

        for &dist in &distances {
            let listener = Listener {
                position: [0.0, -dist, 1.0],
                ear_spacing: 0.0,
            };
            let mut path = AperturePath::new(aperture, sample_rate).with_ground_reflection(false);
            let mut max_val = 0.0f32;
            for n in 0..3000 {
                let t = n as f32 / sample_rate;
                let sig = (2.0 * PI * freq * t).sin();
                let (l, _) = path.step_attenuated(sig, &listener);
                if n > 1500 && l.abs() > max_val {
                    max_val = l.abs();
                }
            }
            amplitudes.push(max_val);
        }

        assert!(
            (amplitudes[0] - 1.0).abs() < 0.02,
            "Expected ~1.0 at 1m, got {}",
            amplitudes[0]
        );
        assert!(
            (amplitudes[1] - 0.5).abs() < 0.02,
            "Expected ~0.5 at 2m, got {}",
            amplitudes[1]
        );
        assert!(
            (amplitudes[2] - 0.25).abs() < 0.02,
            "Expected ~0.25 at 4m, got {}",
            amplitudes[2]
        );

        let ratio_1_2 = amplitudes[0] / amplitudes[1];
        assert!(
            (ratio_1_2 - 2.0).abs() < 0.05,
            "Level should halve when doubling distance: got ratio {}",
            ratio_1_2
        );

        let ratio_1_4 = amplitudes[0] / amplitudes[2];
        assert!(
            (ratio_1_4 - 4.0).abs() < 0.10,
            "Level should quarter when quadrupling distance: got ratio {}",
            ratio_1_4
        );
    }

    #[test]
    fn air_absorption_attenuates_high_frequencies_over_distance() {
        let sample_rate = 48_000.0;
        let aperture = Aperture::new([0.0, 0.0, 1.0], 0.01, [0.0, -1.0, 0.0]);
        let dist = 20.0;
        let listener = Listener {
            position: [0.0, -dist, 1.0],
            ear_spacing: 0.0,
        };

        let measure_response = |freq: f32| -> f32 {
            let mut path = AperturePath::new(aperture, sample_rate).with_ground_reflection(false);
            let mut max_val = 0.0f32;
            for n in 0..4000 {
                let t = n as f32 / sample_rate;
                let sig = (2.0 * PI * freq * t).sin();
                let (l, _) = path.step_attenuated(sig, &listener);
                if n > 2000 && l.abs() > max_val {
                    max_val = l.abs();
                }
            }
            // Normalize by 1/r gain so we only observe air absorption
            max_val / distance_attenuation(dist)
        };

        let low_gain = measure_response(100.0);
        let high_gain = measure_response(10_000.0);

        assert!(
            (low_gain - 1.0).abs() < 0.05,
            "Low frequencies should pass unattenuated by air absorption: got {}",
            low_gain
        );
        assert!(
            high_gain < 0.85,
            "10 kHz should be measurably attenuated at 20 m: got {}",
            high_gain
        );
    }

    #[test]
    fn moving_the_listener_behind_the_car_attenuates_the_tailpipes_high_end_and_not_its_low_end() {
        let sample_rate = 48_000.0;
        // Tailpipe pointed downward (0, 0, -1) or transversely, common on road vehicles.
        // A listener behind the car is off-axis relative to the downward aperture opening.
        let tailpipe = Aperture::new([0.0, -2.0, 0.3], PI * 0.03 * 0.03, [0.0, 0.0, -1.0]);
        let dist = 4.0;
        let behind = Listener {
            position: [0.0, -dist - 2.0, 0.3],
            ear_spacing: 0.0,
        };
        // Directly on-axis under the opening at the exact same distance:
        let on_axis = Listener {
            position: [0.0, -2.0, 0.3 - dist],
            ear_spacing: 0.0,
        };

        let measure_gain = |listener: &Listener, freq: f32| -> f32 {
            let mut path = AperturePath::new(tailpipe, sample_rate).with_ground_reflection(false);
            let mut max_val = 0.0f32;
            for n in 0..4000 {
                let t = n as f32 / sample_rate;
                let sig = (2.0 * PI * freq * t).sin();
                let (l, _) = path.step_propagated(sig, listener);
                if n > 2000 && l.abs() > max_val {
                    max_val = l.abs();
                }
            }
            max_val
        };

        // Low frequency (60 Hz), well below ka = 1 (corner ~ 1820 Hz):
        let low_on_axis = measure_gain(&on_axis, 60.0);
        let low_behind = measure_gain(&behind, 60.0);
        assert!(
            (low_behind - low_on_axis).abs() / low_on_axis < 0.05,
            "Low frequencies must be omnidirectional (monopole): behind={}, on_axis={}",
            low_behind,
            low_on_axis
        );

        // High frequency (8000 Hz), well above ka = 1:
        let high_on_axis = measure_gain(&on_axis, 8000.0);
        let high_behind = measure_gain(&behind, 8000.0);
        assert!(
            high_behind < 0.55 * high_on_axis,
            "High frequencies must beam along the aperture normal and attenuate off-axis: behind={}, on_axis={}",
            high_behind,
            high_on_axis
        );
    }

    #[test]
    fn ground_path_difference_formula_analytic_check() {
        // Source height hs = 0.35 m, receiver height hr = 1.2 m, horizontal dist d = 5.0 m
        let source = [0.0, 0.0, 0.35];
        let receiver = [3.0, 4.0, 1.2]; // d^2 = 3^2 + 4^2 = 25
        let delta = ground_path_difference(source, receiver);

        // Analytic:
        // r_direct = sqrt((0.35 - 1.2)^2 + 25) = sqrt((-0.85)^2 + 25) = sqrt(0.7225 + 25) = sqrt(25.7225)
        // r_reflect = sqrt((0.35 + 1.2)^2 + 25) = sqrt((1.55)^2 + 25) = sqrt(2.4025 + 25) = sqrt(27.4025)
        let expected_direct = (0.85f32 * 0.85 + 25.0).sqrt();
        let expected_reflected = (1.55f32 * 1.55 + 25.0).sqrt();
        let expected_delta = expected_reflected - expected_direct;

        assert!(
            (delta - expected_delta).abs() < 1e-5,
            "Path difference must match sqrt((hs+hr)^2+d^2) - sqrt((hs-hr)^2+d^2)"
        );

        let f_notch = ground_notch_hz(delta, SPEED_OF_SOUND_AIR);
        let expected_f_notch = SPEED_OF_SOUND_AIR / (2.0 * expected_delta);
        assert!((f_notch - expected_f_notch).abs() < 1e-3);
    }

    #[test]
    fn doppler_shift_matches_radial_velocity_formula() {
        let c = SPEED_OF_SOUND_AIR;
        let f = 1000.0f32;

        // Stationary: vr = 0 -> f' = f
        let factor_zero = doppler_factor(0.0, c);
        assert!((factor_zero - 1.0).abs() < 1e-6);

        // Approaching at 34.32 m/s (Mach 0.1): vr = +34.32
        // f' = f * c / (c - vr) = 1000 * 343.2 / (343.2 - 34.32) = 1000 * 10/9 = 1111.11 Hz
        let vr_approach = 0.1 * c;
        let factor_approach = doppler_factor(vr_approach, c);
        let expected_approach = c / (c - vr_approach);
        assert!((factor_approach - expected_approach).abs() < 1e-5);
        let f_prime_approach = f * factor_approach;
        assert!((f_prime_approach - 1111.1111).abs() < 1e-2);

        // Receding at 34.32 m/s (Mach 0.1): vr = -34.32
        // f' = f * c / (c - vr) = 1000 * 343.2 / (343.2 + 34.32) = 1000 * 10/11 = 909.09 Hz
        let vr_recede = -0.1 * c;
        let factor_recede = doppler_factor(vr_recede, c);
        let expected_recede = c / (c - vr_recede);
        assert!((factor_recede - expected_recede).abs() < 1e-5);
        let f_prime_recede = f * factor_recede;
        assert!((f_prime_recede - 909.0909).abs() < 1e-2);
    }

    #[test]
    fn pass_by_shifts_tailpipe_and_intake_separately() {
        // Vehicle travelling in +Y direction at 25 m/s (90 km/h)
        let velocity = [0.0, 25.0, 0.0];
        // Observer at roadside: [5.0, 0.0, 1.2]
        let observer = [5.0, 0.0, 1.2];

        // Vehicle center is at y = 0.0 at this instant.
        // Intake mouth is at vehicle front: y = +1.8 m
        // Tailpipe is at vehicle rear: y = -2.2 m
        let intake = Aperture::new([0.0, 1.8, 0.6], 0.01, [0.0, 1.0, 0.0]);
        let tailpipe = Aperture::new([0.0, -2.2, 0.3], 0.01, [0.0, -1.0, 0.0]);

        let intake_path = AperturePath::new(intake, 48_000.0).with_velocity(velocity);
        let tailpipe_path = AperturePath::new(tailpipe, 48_000.0).with_velocity(velocity);

        let intake_doppler = intake_path.doppler_factor_to(observer);
        let tailpipe_doppler = tailpipe_path.doppler_factor_to(observer);

        // The intake mouth has already passed y = 0 (is at y = +1.8), so it is moving away from the observer!
        // Radial velocity is negative, Doppler factor < 1.0 (shifted down).
        assert!(
            intake_doppler < 1.0,
            "Intake mouth having passed the observer must be pitch shifted down: got {}",
            intake_doppler
        );

        // The tailpipe has NOT yet reached y = 0 (is at y = -2.2), so it is still approaching the observer!
        // Radial velocity is positive, Doppler factor > 1.0 (shifted up).
        assert!(
            tailpipe_doppler > 1.0,
            "Tailpipe still approaching the observer must be pitch shifted up: got {}",
            tailpipe_doppler
        );

        // They must differ measurably:
        assert!(
            tailpipe_doppler > intake_doppler + 0.05,
            "Intake and tailpipe must have distinctly separate Doppler shifts during pass-by"
        );
    }

    #[test]
    fn propagation_model_propagates_all_apertures_to_stereo() {
        let sample_rate = 48_000.0;
        let listener = Listener::new([0.0, -3.5, 1.2]);
        let tailpipes = vec![
            Aperture::new([-0.4, -2.4, 0.35], 0.003, [0.0, -1.0, 0.0]),
            Aperture::new([0.4, -2.4, 0.35], 0.003, [0.0, -1.0, 0.0]),
        ];
        let intake = Aperture::new([0.0, 1.5, 0.65], 0.004, [0.0, 1.0, 0.0]);
        let block = Aperture::new([0.0, 0.8, 0.50], 0.20, [0.0, 0.0, 1.0]);

        let mut model = PropagationModel::new(listener, tailpipes, intake, block, sample_rate);

        // Feed impulse to tailpipe 0
        let (l, r) = model.step(&[1.0, 0.0], 0.0, 0.0);
        // Direct propagation will arrive after path delay, so initial sample is zero
        assert_eq!((l, r), (0.0, 0.0));

        // Step through until pulses arrive and verify finite output
        let mut sum_energy = 0.0f32;
        for _ in 0..1000 {
            let (out_l, out_r) = model.step(&[0.0, 0.0], 0.0, 0.0);
            assert!(out_l.is_finite() && out_r.is_finite());
            sum_energy += out_l * out_l + out_r * out_r;
        }
        assert!(
            sum_energy > 1e-6,
            "Acoustic energy must arrive at the listener"
        );
    }
}
