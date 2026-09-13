//! Slider-crank kinematics and the zero-dimensional cylinder state vector.
//!
//! The cylinder carries the state vector
//!
//! ```text
//! X_cyl = [theta, T, m, x_b]^T
//! ```
//!
//! with crank angle `theta` [rad], bulk gas temperature `T` [K], trapped mass
//! `m` [kg] and burned mass fraction `x_b` [-]. Pressure is deliberately *not*
//! a state: it is closed algebraically from the ideal gas law every time it is
//! needed, which keeps the integrator from drifting off the equation of state.

use std::f64::consts::PI;

/// One full four-stroke cycle [rad].
pub const CYCLE_ANGLE: f64 = 4.0 * PI;

/// Fixed geometry of a single cylinder. All lengths in metres, volumes in m^3.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CylinderGeometry {
    /// Cylinder bore diameter [m].
    pub bore: f64,
    /// Piston stroke [m].
    pub stroke: f64,
    /// Connecting-rod length, centre to centre [m].
    pub rod_length: f64,
    /// Geometric compression ratio `(V_d + V_c) / V_c` [-].
    pub compression_ratio: f64,
    /// Reciprocating mass: piston, rings, pin and about a third of the rod's
    /// own mass, all of it lumped at the wrist pin the way every slider-crank
    /// inertia treatment does [kg].
    ///
    /// Defaults from bore alone (see [`default_reciprocating_mass`]) so no
    /// preset has to name it, and every caller that never sets it explicitly
    /// still gets a physically reasonable slug rather than a zero that quietly
    /// switches the inertia force and torque off.
    pub reciprocating_mass: f64,
}

impl Default for CylinderGeometry {
    /// A ~500 cc naturally aspirated sport-bike slug: 86 x 86 mm, 11.5:1.
    fn default() -> Self {
        Self::new(0.086, 0.086, 0.1345, 11.5)
    }
}

/// Reciprocating mass from bore alone [kg].
///
/// `m ~ 620 * B^3` puts a 94 mm bore near 0.51 kg, the right neighbourhood for
/// an aluminium piston plus rings, pin and a rod's-worth share — light enough
/// that nobody has measured a real slug this way, heavy enough that the
/// inertia force it produces sits in the right octave against the combustion
/// drive it shares a mix with.
pub fn default_reciprocating_mass(bore: f64) -> f64 {
    620.0 * bore.powi(3)
}

impl CylinderGeometry {
    /// Builds a geometry, clamping each parameter into a physically meaningful
    /// range so a bad CLI argument degrades instead of producing NaNs.
    ///
    /// The rod is forced to stay longer than the crank radius: `l <= r` has no
    /// slider-crank solution and would put a negative argument under the square
    /// root in [`CylinderGeometry::piston_position`]. Reciprocating mass is not
    /// a parameter here — it defaults from `bore` via
    /// [`default_reciprocating_mass`]; override it with
    /// [`CylinderGeometry::with_reciprocating_mass`] when a preset wants to
    /// name one explicitly.
    pub fn new(bore: f64, stroke: f64, rod_length: f64, compression_ratio: f64) -> Self {
        let bore = bore.max(1e-4);
        let stroke = stroke.max(1e-4);
        let crank_radius = stroke / 2.0;
        Self {
            bore,
            stroke,
            // 1.05 r is already an absurdly short rod; real engines run 1.5-2.2 r.
            rod_length: rod_length.max(crank_radius * 1.05),
            compression_ratio: compression_ratio.max(1.05),
            reciprocating_mass: default_reciprocating_mass(bore),
        }
    }

    /// Overrides the reciprocating mass, clamping negative input to zero.
    ///
    /// Zero is a legitimate value, not a clamp floor to avoid: it is what
    /// makes the whole inertia-force and inertia-torque path a no-op, which is
    /// what every fingerprint recorded before this mass existed relies on.
    pub fn with_reciprocating_mass(mut self, mass: f64) -> Self {
        self.reciprocating_mass = mass.max(0.0);
        self
    }

    /// Crank radius `r = S / 2` [m].
    pub fn crank_radius(&self) -> f64 {
        self.stroke / 2.0
    }

    /// Rod ratio `l / r` [-]; drives how asymmetric the piston motion is.
    pub fn rod_ratio(&self) -> f64 {
        self.rod_length / self.crank_radius()
    }

    /// Piston crown area `A = pi/4 * B^2` [m^2].
    pub fn piston_area(&self) -> f64 {
        PI * self.bore * self.bore / 4.0
    }

    /// Swept (displacement) volume `V_d = A * S` [m^3].
    pub fn displacement(&self) -> f64 {
        self.piston_area() * self.stroke
    }

    /// Clearance volume at TDC, `V_c = V_d / (CR - 1)` [m^3].
    pub fn clearance_volume(&self) -> f64 {
        self.displacement() / (self.compression_ratio - 1.0)
    }

    /// Total volume at BDC, `V_c + V_d` [m^3].
    pub fn max_volume(&self) -> f64 {
        self.clearance_volume() + self.displacement()
    }

    /// Piston displacement from TDC [m].
    ///
    /// ```text
    /// x(theta) = r * (1 - cos theta) + l - sqrt(l^2 - r^2 sin^2 theta)
    /// ```
    ///
    /// `theta = 0` is TDC, `theta = pi` is BDC. The expression is 2*pi periodic,
    /// so the four-stroke phase lives in the cycle angle, not in the geometry.
    pub fn piston_position(&self, theta: f64) -> f64 {
        let r = self.crank_radius();
        let l = self.rod_length;
        let s = theta.sin();
        // l > r by construction, so the radicand is strictly positive.
        let radical = (l * l - r * r * s * s).max(0.0).sqrt();
        r * (1.0 - theta.cos()) + l - radical
    }

    /// Instantaneous cylinder volume `V(theta) = V_c + A * x(theta)` [m^3].
    pub fn volume(&self, theta: f64) -> f64 {
        self.clearance_volume() + self.piston_area() * self.piston_position(theta)
    }

    /// Volume with the zero-volume singularity floor applied [m^3].
    ///
    /// `V_safe(theta) = max(V_clearance, V(theta))`. Rounding in the sqrt can put
    /// `V(theta)` a few ULP under `V_c` near TDC; every division by volume goes
    /// through here so the pressure closure can never blow up.
    pub fn safe_volume(&self, theta: f64) -> f64 {
        self.volume(theta).max(self.clearance_volume())
    }

    /// Rate of volume change with crank angle [m^3/rad].
    ///
    /// ```text
    /// dV/dtheta = A * r * (sin theta + r sin theta cos theta / sqrt(l^2 - r^2 sin^2 theta))
    /// ```
    pub fn dvolume_dtheta(&self, theta: f64) -> f64 {
        let r = self.crank_radius();
        let l = self.rod_length;
        let (s, c) = theta.sin_cos();
        let radical = (l * l - r * r * s * s).max(1e-12).sqrt();
        self.piston_area() * r * (s + r * s * c / radical)
    }

    /// First derivative of piston displacement with crank angle [m/rad].
    ///
    /// ```text
    /// x'(theta) = r sin theta + r^2 sin theta cos theta / sqrt(l^2 - r^2 sin^2 theta)
    /// ```
    ///
    /// The exact derivative of [`CylinderGeometry::piston_position`], not the
    /// small-angle `sin theta + (r/2l) sin 2theta` approximation — the closed
    /// form is no harder to evaluate. It is the same expression already inside
    /// [`CylinderGeometry::dvolume_dtheta`] (which is just `A * x'(theta)`) and
    /// [`CylinderGeometry::piston_velocity`] (`omega * x'(theta)`); it is
    /// broken out here because the inertia torque below needs `x'` and `x''`
    /// together rather than folded into a volume rate or a velocity.
    pub fn dposition_dtheta(&self, theta: f64) -> f64 {
        let r = self.crank_radius();
        let l = self.rod_length;
        let (s, c) = theta.sin_cos();
        let radical = (l * l - r * r * s * s).max(1e-12).sqrt();
        r * (s + r * s * c / radical)
    }

    /// Second derivative of piston displacement with crank angle [m/rad^2].
    ///
    /// Differentiating `x'(theta) = r sin theta + r^2 sin theta cos theta / R`
    /// with `R = sqrt(l^2 - r^2 sin^2 theta)` and, from the same expansion,
    /// `dR/dtheta = -r^2 sin theta cos theta / R`, gives the closed form
    ///
    /// ```text
    /// x''(theta) = r cos theta + r^2 cos(2 theta) / R
    ///              + r^4 sin^2 theta cos^2 theta / R^3
    /// ```
    ///
    /// Exact, not the `cos theta + (r/l) cos 2theta` two-term approximation:
    /// the closed form for `x` is already sitting in this file, so there is no
    /// reason to reach for the truncated series that approximates it.
    pub fn d2position_dtheta2(&self, theta: f64) -> f64 {
        let r = self.crank_radius();
        let l = self.rod_length;
        let (s, c) = theta.sin_cos();
        let r2 = r * r;
        let radical = (l * l - r2 * s * s).max(1e-12).sqrt();
        let cos2theta = c * c - s * s;
        r * c + r2 * cos2theta / radical + r2 * r2 * s * s * c * c / radical.powi(3)
    }

    /// Piston velocity at a given shaft speed [m/s].
    pub fn piston_velocity(&self, theta: f64, omega: f64) -> f64 {
        let r = self.crank_radius();
        let l = self.rod_length;
        let (s, c) = theta.sin_cos();
        let radical = (l * l - r * r * s * s).max(1e-12).sqrt();
        omega * r * (s + r * s * c / radical)
    }

    /// Mean piston speed `2 * S * n` for an engine speed in RPM [m/s].
    pub fn mean_piston_speed(&self, rpm: f64) -> f64 {
        2.0 * self.stroke * rpm / 60.0
    }

    /// Reciprocating inertia torque reflected onto the crank [N*m].
    ///
    /// ```text
    /// tau_i(theta) = -m * omega^2 * x''(theta) * x'(theta)
    /// ```
    ///
    /// Piston acceleration at near-constant crank speed is `x_ddot = omega^2 *
    /// x''(theta)` — the angular-acceleration (`alpha`) term is second order in
    /// an already small quantity, and is left out. The inertia *force* along
    /// the bore is `-m * x_ddot`; by the virtual-work relation between a force
    /// along the slider and the torque it reflects onto the crank
    /// (`tau * omega = F * x_dot`, `x_dot = omega * x'(theta)`), that force
    /// reflects as `F * x'(theta)`, which is this expression. It carries no
    /// bank angle and no sum over cylinders — it acts on the crank the engine
    /// already has, not on the case bolted around it — which is what keeps it
    /// a separate mechanism from the shaking force that shakes the block.
    ///
    /// Zero net work over a full revolution: `x'' * x'` is `d/dtheta[(x')^2 /
    /// 2]`, the derivative of a periodic function, so it integrates to zero
    /// over any whole number of periods. It ripples the crank speed; it does
    /// not add or remove energy from it.
    pub fn inertia_torque(&self, theta: f64, omega: f64) -> f64 {
        -self.reciprocating_mass
            * omega
            * omega
            * self.d2position_dtheta2(theta)
            * self.dposition_dtheta(theta)
    }

    /// Reciprocating shaking force along the cylinder's own bore axis [N].
    ///
    /// ```text
    /// F_i(theta) = -m * omega^2 * x''(theta)
    /// ```
    ///
    /// The reaction the accelerating piston (and the share of the rod lumped
    /// with it) presses back into the block through the cylinder wall and main
    /// bearings, at the same near-constant-speed approximation as
    /// [`CylinderGeometry::inertia_torque`]. This is the mechanism that shakes
    /// the case the engine is bolted to; it says nothing about where this
    /// cylinder's axis points or how it phases against any other cylinder's —
    /// resolving several of these onto the block's own axes, at their own
    /// crank phase and bank angle, is what
    /// [`crate::physics::engine_block::FiringOrder::shaking_force`] does.
    pub fn inertia_force(&self, theta: f64, omega: f64) -> f64 {
        -self.reciprocating_mass * omega * omega * self.d2position_dtheta2(theta)
    }
}

/// Working-gas properties, blended between fresh charge and burned products.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GasProperties {
    /// Specific gas constant of the unburned charge [J/(kg*K)].
    pub r_unburned: f64,
    /// Specific gas constant of the burned products [J/(kg*K)].
    pub r_burned: f64,
    /// Ratio of specific heats of the unburned charge [-].
    pub gamma_unburned: f64,
    /// Ratio of specific heats of the burned products [-].
    pub gamma_burned: f64,
}

impl Default for GasProperties {
    /// Stoichiometric gasoline-air charge burning to CO2/H2O products.
    fn default() -> Self {
        Self {
            r_unburned: 287.0,
            r_burned: 291.0,
            gamma_unburned: 1.35,
            gamma_burned: 1.26,
        }
    }
}

impl GasProperties {
    /// Adjusts burned gas properties for air-fuel ratio.
    ///
    /// Rich mixtures produce more CO, H2, and unburnt hydrocarbons with lower
    /// gamma_burned. Lean mixtures with excess air raise gamma_burned towards air.
    pub fn for_afr(afr: f64) -> Self {
        let stoich = 14.7;
        let phi = stoich / afr.clamp(8.0, 25.0);
        let gamma_burned = (1.26 - 0.06 * (phi - 1.0)).clamp(1.20, 1.32);
        let r_burned = (291.0 + 8.0 * (phi - 1.0)).clamp(280.0, 310.0);
        Self {
            r_unburned: 287.0,
            r_burned,
            gamma_unburned: 1.35,
            gamma_burned,
        }
    }

    /// Specific gas constant at a burned mass fraction, linearly blended.
    pub fn r_specific(&self, burned_fraction: f64) -> f64 {
        let x = burned_fraction.clamp(0.0, 1.0);
        self.r_unburned * (1.0 - x) + self.r_burned * x
    }

    /// Ratio of specific heats at a burned mass fraction, linearly blended.
    pub fn gamma(&self, burned_fraction: f64) -> f64 {
        let x = burned_fraction.clamp(0.0, 1.0);
        self.gamma_unburned * (1.0 - x) + self.gamma_burned * x
    }

    /// Constant-volume specific heat `c_v = R / (gamma - 1)` [J/(kg*K)].
    pub fn cv(&self, burned_fraction: f64) -> f64 {
        self.r_specific(burned_fraction) / (self.gamma(burned_fraction) - 1.0)
    }

    /// Constant-pressure specific heat `c_p = gamma * c_v` [J/(kg*K)].
    pub fn cp(&self, burned_fraction: f64) -> f64 {
        self.gamma(burned_fraction) * self.cv(burned_fraction)
    }
}

/// The four strokes of the cycle, keyed off the 720-degree crank phase.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stroke {
    /// TDC -> BDC, inlet valve open.
    Intake,
    /// BDC -> TDC, both valves shut.
    Compression,
    /// TDC -> BDC, expansion after ignition.
    Power,
    /// BDC -> TDC, exhaust valve open.
    Exhaust,
}

/// State vector `X_cyl = [theta, T, m, x_b]^T` for one cylinder.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CylinderState {
    /// Crank angle within the 720-degree cycle, `0` at intake TDC [rad].
    pub theta: f64,
    /// Bulk gas temperature [K].
    pub temperature: f64,
    /// Trapped gas mass [kg].
    pub mass: f64,
    /// Burned mass fraction, 0 = fresh charge, 1 = fully burned [-].
    pub burned_fraction: f64,
}

impl CylinderState {
    /// Seeds a cylinder at intake TDC filled with ambient charge.
    pub fn at_ambient(
        geometry: &CylinderGeometry,
        gas: &GasProperties,
        pressure: f64,
        temperature: f64,
    ) -> Self {
        let volume = geometry.safe_volume(0.0);
        Self {
            theta: 0.0,
            temperature,
            mass: pressure * volume / (gas.r_unburned * temperature),
            burned_fraction: 0.0,
        }
    }

    /// Advances the crank angle, wrapping into `[0, 4*pi)`.
    pub fn advance(&mut self, dtheta: f64) {
        self.theta = wrap_cycle(self.theta + dtheta);
    }

    /// Keeps the state inside its physical envelope after an integration step.
    pub fn sanitize(&mut self) {
        self.theta = wrap_cycle(self.theta);
        self.temperature = self.temperature.clamp(1.0, 6000.0);
        self.mass = self.mass.max(1e-12);
        self.burned_fraction = self.burned_fraction.clamp(0.0, 1.0);
    }

    /// Current volume, singularity-protected [m^3].
    pub fn volume(&self, geometry: &CylinderGeometry) -> f64 {
        geometry.safe_volume(self.theta)
    }

    /// Algebraic pressure closure from the ideal gas law [Pa].
    ///
    /// ```text
    /// P = m * R_spec(x_b) * T / V_safe(theta)
    /// ```
    ///
    /// `R_spec` tracks the burned fraction, so the charge smoothly takes on the
    /// properties of its products as the flame front consumes it.
    pub fn pressure(&self, geometry: &CylinderGeometry, gas: &GasProperties) -> f64 {
        self.mass * gas.r_specific(self.burned_fraction) * self.temperature
            / geometry.safe_volume(self.theta)
    }

    /// Gas density [kg/m^3].
    pub fn density(&self, geometry: &CylinderGeometry) -> f64 {
        self.mass / geometry.safe_volume(self.theta)
    }

    /// Which of the four strokes the cylinder is in.
    pub fn stroke(&self) -> Stroke {
        match (wrap_cycle(self.theta) / PI) as u8 {
            0 => Stroke::Intake,
            1 => Stroke::Compression,
            2 => Stroke::Power,
            _ => Stroke::Exhaust,
        }
    }
}

/// Wraps a crank angle into `[0, 4*pi)`.
pub fn wrap_cycle(theta: f64) -> f64 {
    let wrapped = theta.rem_euclid(CYCLE_ANGLE);
    // rem_euclid can return exactly CYCLE_ANGLE for tiny negative inputs.
    if wrapped >= CYCLE_ANGLE {
        0.0
    } else {
        wrapped
    }
}

/// Converts degrees to radians.
pub fn deg(degrees: f64) -> f64 {
    degrees * PI / 180.0
}

#[cfg(test)]
mod tests {
    use super::*;

    const GEOM: fn() -> CylinderGeometry = CylinderGeometry::default;

    fn approx(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() <= tol, "expected {b}, got {a} (tol {tol})");
    }

    #[test]
    fn tdc_volume_is_clearance_volume() {
        let g = GEOM();
        approx(g.volume(0.0), g.clearance_volume(), 1e-15);
        approx(g.piston_position(0.0), 0.0, 1e-15);
    }

    #[test]
    fn bdc_volume_is_clearance_plus_displacement() {
        let g = GEOM();
        approx(g.volume(PI), g.max_volume(), 1e-15);
        approx(g.piston_position(PI), g.stroke, 1e-15);
    }

    #[test]
    fn compression_ratio_is_recovered_from_geometry() {
        let g = CylinderGeometry::new(0.081, 0.0862, 0.144, 12.5);
        approx(g.max_volume() / g.volume(0.0), 12.5, 1e-12);
    }

    #[test]
    fn displacement_matches_swept_cylinder() {
        // 86 x 86 mm single: pi/4 * 86^2 * 86 mm^3 ~= 499.6 cc.
        let g = GEOM();
        approx(g.displacement() * 1e6, 499.56, 0.05);
    }

    /// Walks the whole 720-degree four-stroke cycle and checks the geometric
    /// expansion/compression pattern of every stroke.
    #[test]
    fn volume_expands_and_contracts_over_the_full_720_degree_cycle() {
        let g = GEOM();
        let steps = 7200; // 0.1 degree resolution

        for i in 1..=steps {
            let theta = CYCLE_ANGLE * i as f64 / steps as f64;
            let v = g.volume(theta);
            assert!(
                v >= g.clearance_volume() - 1e-15 && v <= g.max_volume() + 1e-15,
                "volume left [Vc, Vc+Vd] at {:.1} deg: {v}",
                theta.to_degrees()
            );
        }

        // Intake and power strokes expand monotonically; compression and
        // exhaust contract monotonically.
        for (start_deg, expanding) in [(0.0, true), (180.0, false), (360.0, true), (540.0, false)] {
            let mut last = g.volume(deg(start_deg));
            for i in 1..=1800 {
                let theta = deg(start_deg + 180.0 * i as f64 / 1800.0);
                let v = g.volume(theta);
                if expanding {
                    assert!(v > last, "stroke from {start_deg} deg should expand");
                } else {
                    assert!(v < last, "stroke from {start_deg} deg should contract");
                }
                last = v;
            }
        }

        // A full cycle returns the piston exactly where it started.
        approx(g.volume(CYCLE_ANGLE), g.volume(0.0), 1e-15);
        // And each half-turn lands on an extremum.
        approx(g.volume(deg(360.0)), g.clearance_volume(), 1e-15);
        approx(g.volume(deg(540.0)), g.max_volume(), 1e-15);
    }

    #[test]
    fn volume_is_symmetric_about_bdc() {
        let g = GEOM();
        for d in [10.0, 45.0, 90.0, 150.0] {
            approx(g.volume(deg(d)), g.volume(deg(360.0 - d)), 1e-15);
        }
    }

    #[test]
    fn dvolume_dtheta_matches_numerical_derivative() {
        let g = GEOM();
        let h = 1e-6;
        for d in [1.0, 30.0, 90.0, 179.0, 181.0, 270.0, 359.0] {
            let theta = deg(d);
            let numeric = (g.volume(theta + h) - g.volume(theta - h)) / (2.0 * h);
            approx(g.dvolume_dtheta(theta), numeric, 1e-9);
        }
        // dV/dtheta vanishes at both dead centres.
        approx(g.dvolume_dtheta(0.0), 0.0, 1e-15);
        approx(g.dvolume_dtheta(PI), 0.0, 1e-15);
    }

    #[test]
    fn position_derivatives_match_central_differences_across_a_revolution() {
        let g = GEOM();
        let h = 1e-4;
        for i in 0..360 {
            let theta = deg(i as f64) + 0.001; // offset off the dead centres
            let numeric_first =
                (g.piston_position(theta + h) - g.piston_position(theta - h)) / (2.0 * h);
            approx(g.dposition_dtheta(theta), numeric_first, 1e-6);

            let numeric_second = (g.piston_position(theta + h) - 2.0 * g.piston_position(theta)
                + g.piston_position(theta - h))
                / (h * h);
            approx(g.d2position_dtheta2(theta), numeric_second, 1e-6);
        }
    }

    #[test]
    fn rod_ratio_skews_peak_velocity_before_mid_stroke() {
        // With a finite rod the piston hits peak speed before 90 degrees ATDC.
        let g = GEOM();
        let mut peak_deg = 0.0;
        let mut peak = 0.0;
        for i in 0..=1800 {
            let d = 180.0 * i as f64 / 1800.0;
            let v = g.piston_velocity(deg(d), 1.0).abs();
            if v > peak {
                peak = v;
                peak_deg = d;
            }
        }
        assert!(
            (70.0..90.0).contains(&peak_deg),
            "peak piston speed at {peak_deg} deg, expected just before mid-stroke"
        );
    }

    #[test]
    fn safe_volume_never_falls_below_clearance() {
        let g = CylinderGeometry::new(0.086, 0.086, 0.1345, 11.5);
        for i in 0..=720 {
            let theta = deg(i as f64);
            assert!(g.safe_volume(theta) >= g.clearance_volume());
            assert!(g.safe_volume(theta).is_finite());
        }
        // Even a degenerate 1:1 rod-to-crank request stays finite.
        let degenerate = CylinderGeometry::new(0.086, 0.086, 0.001, 11.5);
        for i in 0..=720 {
            assert!(degenerate.safe_volume(deg(i as f64)).is_finite());
        }
    }

    #[test]
    fn pressure_closure_follows_ideal_gas_law() {
        let g = GEOM();
        let gas = GasProperties::default();
        let state = CylinderState::at_ambient(&g, &gas, 101_325.0, 300.0);
        approx(state.pressure(&g, &gas), 101_325.0, 1e-6);

        // Isentropic-ish check: squeeze the same mass to TDC and the pressure
        // must rise by at least the compression ratio (isothermal lower bound).
        let mut compressed = state;
        compressed.theta = 0.0;
        let mut at_bdc = state;
        at_bdc.theta = PI;
        let ratio = compressed.pressure(&g, &gas) / at_bdc.pressure(&g, &gas);
        approx(ratio, g.compression_ratio, 1e-9);
    }

    #[test]
    fn specific_gas_constant_tracks_burned_fraction() {
        let gas = GasProperties::default();
        approx(gas.r_specific(0.0), gas.r_unburned, 1e-12);
        approx(gas.r_specific(1.0), gas.r_burned, 1e-12);
        approx(
            gas.r_specific(0.5),
            0.5 * (gas.r_unburned + gas.r_burned),
            1e-12,
        );
        // Out-of-range fractions clamp instead of extrapolating.
        approx(gas.r_specific(-3.0), gas.r_unburned, 1e-12);
        approx(gas.r_specific(7.0), gas.r_burned, 1e-12);
        // Mayer's relation must hold for the blended mixture.
        approx(gas.cp(0.3) - gas.cv(0.3), gas.r_specific(0.3), 1e-9);
    }

    #[test]
    fn stroke_phase_maps_to_the_720_degree_cycle() {
        let s = |d: f64| {
            CylinderState {
                theta: deg(d),
                temperature: 300.0,
                mass: 1e-4,
                burned_fraction: 0.0,
            }
            .stroke()
        };
        assert_eq!(s(10.0), Stroke::Intake);
        assert_eq!(s(200.0), Stroke::Compression);
        assert_eq!(s(400.0), Stroke::Power);
        assert_eq!(s(700.0), Stroke::Exhaust);
        assert_eq!(s(730.0), Stroke::Intake, "cycle must wrap at 720 degrees");
    }

    #[test]
    fn advance_wraps_and_sanitize_clamps() {
        let mut st = CylinderState {
            theta: deg(710.0),
            temperature: -50.0,
            mass: -1.0,
            burned_fraction: 1.7,
        };
        st.advance(deg(20.0));
        approx(st.theta.to_degrees(), 10.0, 1e-9);
        st.sanitize();
        assert!(st.temperature >= 1.0 && st.mass > 0.0);
        approx(st.burned_fraction, 1.0, 1e-12);
        approx(wrap_cycle(-deg(90.0)).to_degrees(), 630.0, 1e-9);
    }
}
