//! The vehicle: a gearbox, the road it pushes back with, and the clutch that
//! couples the crank to both — including through the moment it cannot.
//!
//! [`crate::bench::Driveline`] integrates a bare flywheel against a load. Until
//! now that load was a quadratic in engine speed and nothing else, so the
//! engine had exactly one load line regardless of gear, road, or grade. This
//! module gives it a real one: [`Gearbox`] picks a ratio, a road load prices
//! the road at that ratio, and a clutch is the limited-torque coupling between
//! the two, which is what a standing start and a shift both are.

use serde::{Deserialize, Serialize};

/// Standard gravity [m/s^2].
pub const STANDARD_GRAVITY: f64 = 9.806_65;

// ---------------------------------------------------------------------------
// Gearbox
// ---------------------------------------------------------------------------

/// Which gear the box is in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Gear {
    /// No drive path: the crank spins free of the road entirely. This is the
    /// gear that makes today's free-revving behaviour a special case of a
    /// gearbox rather than a separate code path — see [`Gearbox::overall_ratio`].
    Neutral,
    /// A numbered gear, one-indexed the way a shift pattern is printed, into
    /// [`Gearbox::ratios`].
    Engaged(usize),
}

/// A ratio list plus a final drive, and the gear currently selected.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Gearbox {
    /// Gear ratios, first gear first: engine speed divided by gearbox output
    /// speed, before the final drive.
    pub ratios: Vec<f64>,
    /// Final drive (differential) ratio, applied after whichever gear ratio.
    pub final_drive: f64,
    /// Currently selected gear.
    pub gear: Gear,
}

impl Gearbox {
    /// Builds a gearbox in neutral from a ratio list and a final drive.
    pub fn new(ratios: Vec<f64>, final_drive: f64) -> Self {
        Self {
            ratios,
            final_drive,
            gear: Gear::Neutral,
        }
    }

    /// A generic close-ratio six-speed behind a 3.90 final drive.
    ///
    /// Not tuned per engine — Stage M1 is about the architecture existing at
    /// all, not about fitting a gearbox to each car in the catalogue.
    pub fn generic_six_speed() -> Self {
        Self::new(vec![3.36, 2.09, 1.47, 1.14, 1.00, 0.85], 3.90)
    }

    /// Overall ratio (engine turns per output turn) for the selected gear, or
    /// `None` in neutral — there is no ratio because there is no drive path.
    pub fn overall_ratio(&self) -> Option<f64> {
        match self.gear {
            Gear::Neutral => None,
            Gear::Engaged(g) => self
                .ratios
                .get(g.checked_sub(1)?)
                .map(|r| r * self.final_drive),
        }
    }
}

// ---------------------------------------------------------------------------
// Road load
// ---------------------------------------------------------------------------

/// Rolling resistance, aerodynamic drag, and grade: what the road costs.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RoadLoad {
    /// Vehicle mass [kg].
    pub vehicle_mass: f64,
    /// Rolling resistance coefficient [-].
    pub rolling_resistance: f64,
    /// Drag area, `C_d * A` [m^2].
    pub drag_area: f64,
    /// Driven wheel radius [m].
    pub wheel_radius: f64,
    /// Road grade angle, positive uphill [rad].
    pub grade: f64,
}

impl RoadLoad {
    /// A generic 1.5 tonne road car on a level road.
    pub fn generic_road_car() -> Self {
        Self {
            vehicle_mass: 1_500.0,
            rolling_resistance: 0.012,
            drag_area: 0.30 * 2.2,
            wheel_radius: 0.32,
            grade: 0.0,
        }
    }

    /// Force the road pushes back with at a given road speed [N].
    ///
    /// ```text
    /// F = m g C_rr + 0.5 rho C_d A v^2 + m g sin(grade)
    /// ```
    pub fn force(&self, speed_mps: f64, air_density: f64) -> f64 {
        let rolling = self.vehicle_mass * STANDARD_GRAVITY * self.rolling_resistance;
        let aero = 0.5 * air_density * self.drag_area * speed_mps * speed_mps;
        let grade = self.vehicle_mass * STANDARD_GRAVITY * self.grade.sin();
        rolling + aero + grade
    }

    /// That force reflected to the crank as a torque, through an overall
    /// ratio (engine turns per output turn) [N m].
    ///
    /// Power is conserved across the gearbox: `T_wheel * omega_wheel =
    /// T_crank * omega_crank`, and `omega_crank = omega_wheel * ratio`, so
    /// `T_crank = T_wheel / ratio = F * r / ratio`.
    pub fn crank_torque(&self, speed_mps: f64, air_density: f64, overall_ratio: f64) -> f64 {
        self.force(speed_mps, air_density) * self.wheel_radius / overall_ratio.max(1e-6)
    }

    /// Road speed a crank speed implies through an overall ratio [m/s].
    pub fn road_speed(&self, crank_omega: f64, overall_ratio: f64) -> f64 {
        crank_omega * self.wheel_radius / overall_ratio.max(1e-6)
    }

    /// The vehicle's translational inertia, reflected to the crank through an
    /// overall ratio, as an equivalent rotational inertia [kg m^2].
    ///
    /// `KE = 1/2 m v^2 = 1/2 m (omega_crank r / ratio)^2`, so the equivalent
    /// inertia seen at the crank is `m (r / ratio)^2`.
    pub fn reflected_inertia(&self, overall_ratio: f64) -> f64 {
        self.vehicle_mass * (self.wheel_radius / overall_ratio.max(1e-6)).powi(2)
    }
}

// ---------------------------------------------------------------------------
// Clutch
// ---------------------------------------------------------------------------

/// State the clutch is transmitting torque in this frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClutchState {
    /// No capacity engaged: nothing crosses the interface.
    Open,
    /// Speeds differ, or the locked torque would exceed capacity: it is
    /// dragging the two sides together rather than moving with them.
    Slipping,
    /// Both sides turn together and drive torque fits inside capacity.
    Locked,
}

/// A friction clutch with a torque capacity.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Clutch {
    /// Maximum torque it can transmit fully engaged [N m].
    pub torque_capacity: f64,
    /// Pedal position: 0 open, 1 fully engaged [-].
    pub engagement: f64,
}

impl Clutch {
    /// Builds a fully-engaged clutch with a given torque capacity.
    pub fn new(torque_capacity: f64) -> Self {
        Self {
            torque_capacity,
            engagement: 1.0,
        }
    }

    /// A generic road car's clutch: enough capacity for a mid-size engine.
    pub fn generic_road_car() -> Self {
        Self::new(450.0)
    }

    /// Torque capacity at the current pedal position [N m].
    pub fn capacity(&self) -> f64 {
        (self.torque_capacity * self.engagement.clamp(0.0, 1.0)).max(0.0)
    }

    /// Torque transmitted while slipping, signed with the slip direction
    /// (engine omega minus driven omega) [N m]. Friction drags the slower
    /// side up and the faster side down, so the sign follows the slip.
    pub fn slipping_torque(&self, slip_omega: f64) -> f64 {
        self.capacity() * slip_omega.signum()
    }

    /// Power dissipated as heat while slipping [W], always `>= 0`.
    ///
    /// This is the clutch's whole energy balance: the engine side loses
    /// `slipping_torque * omega_engine` and the driven side gains
    /// `slipping_torque * omega_driven`; the difference between those two is
    /// exactly `slipping_torque * slip_omega`, which is this quantity. It
    /// leaves as heat and is never negative, so slip can only ever destroy
    /// kinetic energy relative to the locked case, never create it.
    pub fn dissipated_power(&self, slip_omega: f64) -> f64 {
        self.slipping_torque(slip_omega) * slip_omega
    }

    /// Whether the clutch would stay locked, given the slip speed between the
    /// two sides and the torque locking them together would need to carry.
    pub fn state(&self, slip_omega: f64, required_lock_torque: f64) -> ClutchState {
        if self.capacity() <= 0.0 {
            ClutchState::Open
        } else if slip_omega.abs() < 1e-2 && required_lock_torque.abs() <= self.capacity() {
            ClutchState::Locked
        } else {
            ClutchState::Slipping
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() <= tol, "expected {b}, got {a} (tol {tol})");
    }

    #[test]
    fn neutral_has_no_overall_ratio() {
        let gearbox = Gearbox::generic_six_speed();
        assert_eq!(gearbox.overall_ratio(), None);
    }

    #[test]
    fn overall_ratio_is_gear_times_final_drive() {
        let mut gearbox = Gearbox::new(vec![3.36, 2.09, 1.47, 1.14, 1.00, 0.85], 3.90);
        gearbox.gear = Gear::Engaged(1);
        approx(gearbox.overall_ratio().unwrap(), 3.36 * 3.90, 1e-9);
        gearbox.gear = Gear::Engaged(6);
        approx(gearbox.overall_ratio().unwrap(), 0.85 * 3.90, 1e-9);
    }

    #[test]
    fn a_gear_past_the_ratio_list_has_no_overall_ratio() {
        let mut gearbox = Gearbox::new(vec![3.0, 2.0], 4.0);
        gearbox.gear = Gear::Engaged(3);
        assert_eq!(gearbox.overall_ratio(), None);
        gearbox.gear = Gear::Engaged(0);
        assert_eq!(gearbox.overall_ratio(), None);
    }

    // -- RoadLoad -----------------------------------------------------------

    #[test]
    fn road_force_matches_hand_computation() {
        // 1500 kg, Crr 0.012, Cd*A 0.66, at 30 m/s, sea-level-ish air.
        let road = RoadLoad {
            vehicle_mass: 1_500.0,
            rolling_resistance: 0.012,
            drag_area: 0.66,
            wheel_radius: 0.32,
            grade: 0.0,
        };
        let air_density = 1.2041;
        let rolling = 1_500.0 * STANDARD_GRAVITY * 0.012;
        let aero = 0.5 * air_density * 0.66 * 30.0 * 30.0;
        approx(road.force(30.0, air_density), rolling + aero, 1e-6);
    }

    #[test]
    fn grade_adds_mg_sin_theta() {
        let level = RoadLoad {
            grade: 0.0,
            ..RoadLoad::generic_road_car()
        };
        let uphill = RoadLoad {
            grade: 0.05, // ~5 % grade, small angle
            ..RoadLoad::generic_road_car()
        };
        let air_density = 1.2041;
        let expected_extra = uphill.vehicle_mass * STANDARD_GRAVITY * 0.05_f64.sin();
        approx(
            uphill.force(20.0, air_density) - level.force(20.0, air_density),
            expected_extra,
            1e-6,
        );
    }

    #[test]
    fn steady_state_in_a_gear_balances_road_load_against_brake_torque() {
        // Hand-computed case: a 1500 kg car, Crr 0.012, Cd*A 0.66, wheel
        // radius 0.32 m, in a gear with overall ratio 10.0, holding 25 m/s.
        let road = RoadLoad {
            vehicle_mass: 1_500.0,
            rolling_resistance: 0.012,
            drag_area: 0.66,
            wheel_radius: 0.32,
            grade: 0.0,
        };
        let overall_ratio = 10.0;
        let speed_mps = 25.0;
        let air_density = 1.2041;

        // F = m g Crr + 0.5 rho Cd*A v^2
        let expected_force =
            1_500.0 * STANDARD_GRAVITY * 0.012 + 0.5 * air_density * 0.66 * 25.0 * 25.0;
        approx(road.force(speed_mps, air_density), expected_force, 1e-6);

        // T_crank = F * r / ratio, and that is exactly the brake torque a
        // flywheel at this speed needs to hold steady (alpha = 0): drive
        // torque in, road torque reflected back out, nothing left over to
        // accelerate against.
        let expected_crank_torque = expected_force * road.wheel_radius / overall_ratio;
        let crank_torque = road.crank_torque(speed_mps, air_density, overall_ratio);
        approx(crank_torque, expected_crank_torque, 1e-6);

        let crank_omega = speed_mps * overall_ratio / road.wheel_radius;
        approx(road.road_speed(crank_omega, overall_ratio), speed_mps, 1e-9);
        let alpha = (crank_torque - road.crank_torque(speed_mps, air_density, overall_ratio))
            / 1.0 /* any positive inertia */;
        approx(alpha, 0.0, 1e-12);
    }

    #[test]
    fn two_gears_at_the_same_engine_speed_imply_different_road_speed_and_torque() {
        let road = RoadLoad::generic_road_car();
        let air_density = 1.2041;
        let engine_omega = 300.0; // rad/s, ~2865 rpm
        let low_gear_ratio = 12.0;
        let high_gear_ratio = 6.0;

        let low_gear_speed = road.road_speed(engine_omega, low_gear_ratio);
        let high_gear_speed = road.road_speed(engine_omega, high_gear_ratio);
        assert!(high_gear_speed > low_gear_speed);

        let low_gear_torque = road.crank_torque(low_gear_speed, air_density, low_gear_ratio);
        let high_gear_torque = road.crank_torque(high_gear_speed, air_density, high_gear_ratio);
        assert!(
            (low_gear_torque - high_gear_torque).abs() > 1e-6,
            "gears must load the crank differently at the same engine speed"
        );
    }

    #[test]
    fn reflected_inertia_matches_kinetic_energy_equivalence() {
        let road = RoadLoad::generic_road_car();
        let overall_ratio = 8.0;
        let crank_omega = 250.0;
        let road_speed = road.road_speed(crank_omega, overall_ratio);

        let vehicle_ke = 0.5 * road.vehicle_mass * road_speed * road_speed;
        let equivalent_ke = 0.5 * road.reflected_inertia(overall_ratio) * crank_omega * crank_omega;
        approx(vehicle_ke, equivalent_ke, 1e-6);
    }

    // -- Clutch ---------------------------------------------------------

    #[test]
    fn open_clutch_transmits_nothing() {
        let clutch = Clutch {
            torque_capacity: 400.0,
            engagement: 0.0,
        };
        assert_eq!(clutch.capacity(), 0.0);
        assert_eq!(clutch.state(50.0, 100.0), ClutchState::Open);
    }

    #[test]
    fn matched_speed_within_capacity_locks() {
        let clutch = Clutch::new(400.0);
        assert_eq!(clutch.state(0.0, 200.0), ClutchState::Locked);
    }

    #[test]
    fn torque_beyond_capacity_slips_even_at_matched_speed() {
        let clutch = Clutch::new(400.0);
        assert_eq!(clutch.state(0.0, 500.0), ClutchState::Slipping);
    }

    #[test]
    fn speed_difference_slips_regardless_of_torque() {
        let clutch = Clutch::new(400.0);
        assert_eq!(clutch.state(50.0, 10.0), ClutchState::Slipping);
    }

    #[test]
    fn slip_dissipates_the_torque_difference_and_never_creates_energy() {
        let clutch = Clutch::new(300.0);
        for slip_omega in [-200.0, -1.0, 1.0, 50.0, 200.0] {
            let transmitted = clutch.slipping_torque(slip_omega);
            // Signed with the slip: drags the driven side toward the engine.
            assert_eq!(transmitted.signum(), slip_omega.signum());
            approx(transmitted.abs(), clutch.capacity(), 1e-9);

            let dissipated = clutch.dissipated_power(slip_omega);
            assert!(
                dissipated >= 0.0,
                "slip must dissipate energy, never create it: {dissipated}"
            );
            approx(dissipated, transmitted * slip_omega, 1e-9);
        }
    }

    #[test]
    fn zero_slip_dissipates_nothing() {
        let clutch = Clutch::new(300.0);
        approx(clutch.dissipated_power(0.0), 0.0, 1e-9);
    }
}
