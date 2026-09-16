//! Turbocharger turbine map: reduced flow and efficiency from corrected
//! speed and expansion ratio.
//!
//! Read the same way the compressor map in [`crate::physics::compressor`]
//! is: a small number of honest `(expansion ratio, reduced flow,
//! efficiency)` points per speed line, interpolated piecewise-linear rather
//! than fitted, because the interesting behaviour is at the edges. Turbine
//! maps are far flatter than compressor maps — flow saturates as it
//! chokes — so fewer points suffice, but the representation is the same.
//!
//! See `docs/TURBO_PLAN.md`'s TB2.

use std::f64::consts::PI;

use crate::physics::compressor;
use crate::physics::thermodynamics::PortState;

/// One measured point on a turbine speed line: expansion ratio against
/// reduced flow and efficiency at that ratio.
#[derive(Debug, Clone, Copy)]
pub struct TurbineMapPoint {
    /// Total-to-static expansion ratio, `p_upstream / p_downstream` [-].
    pub expansion_ratio: f64,
    /// Reduced mass flow, in the same `m sqrt(T/T_ref) / (P/P_ref)` form as
    /// [`crate::physics::compressor::corrected_flow`] [kg/s].
    pub reduced_flow: f64,
    /// Isentropic efficiency, `0.0..=1.0` [-].
    pub efficiency: f64,
}

/// One constant-corrected-speed curve of the turbine map.
///
/// Points are stored ascending by expansion ratio. Unlike a compressor speed
/// line, flow *rises* with expansion ratio rather than falling — a turbine
/// passes more mass the harder it is pushed, up to the point it chokes and
/// the line goes flat.
#[derive(Debug, Clone)]
pub struct TurbineSpeedLine {
    corrected_speed: f64,
    points: Vec<TurbineMapPoint>,
}

impl TurbineSpeedLine {
    /// Builds a speed line from at least two points.
    ///
    /// Panics if fewer than two points are given, or if reduced flow does not
    /// rise monotonically with expansion ratio — a turbine line that falls
    /// again is not a turbine map, it is bad data.
    pub fn new(corrected_speed: f64, mut points: Vec<TurbineMapPoint>) -> Self {
        points.sort_by(|a, b| {
            a.expansion_ratio
                .partial_cmp(&b.expansion_ratio)
                .expect("expansion ratio must not be NaN")
        });
        assert!(
            points.len() >= 2,
            "a turbine speed line needs at least two points"
        );
        for pair in points.windows(2) {
            assert!(
                pair[1].reduced_flow >= pair[0].reduced_flow,
                "turbine speed line at N_corr={corrected_speed} does not rise monotonically \
                 with expansion ratio: {:?} then {:?}",
                pair[0],
                pair[1]
            );
        }
        Self {
            corrected_speed,
            points,
        }
    }

    /// Corrected speed this curve was measured at [rpm].
    pub fn corrected_speed(&self) -> f64 {
        self.corrected_speed
    }

    /// Lowest expansion ratio this line has data for [-].
    pub fn min_expansion_ratio(&self) -> f64 {
        self.points.first().expect("validated non-empty").expansion_ratio
    }

    /// Highest expansion ratio this line has data for [-].
    pub fn max_expansion_ratio(&self) -> f64 {
        self.points.last().expect("validated non-empty").expansion_ratio
    }

    /// Reduced flow and efficiency at an expansion ratio, piecewise-linear
    /// between the nearest two measured points and clamped to the line's own
    /// endpoints.
    fn interpolate_at(&self, expansion_ratio: f64) -> (f64, f64) {
        let clamped = expansion_ratio.clamp(self.min_expansion_ratio(), self.max_expansion_ratio());
        let idx = self
            .points
            .windows(2)
            .position(|pair| clamped <= pair[1].expansion_ratio)
            .unwrap_or(self.points.len() - 2);
        let (lo, hi) = (self.points[idx], self.points[idx + 1]);
        let span = hi.expansion_ratio - lo.expansion_ratio;
        let t = if span > 0.0 {
            (clamped - lo.expansion_ratio) / span
        } else {
            0.0
        };
        (
            lerp(lo.reduced_flow, hi.reduced_flow, t),
            lerp(lo.efficiency, hi.efficiency, t),
        )
    }
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

/// A reduced flow and efficiency read off the turbine map.
#[derive(Debug, Clone, Copy)]
pub struct TurbineReading {
    pub reduced_flow: f64,
    pub efficiency: f64,
}

/// A turbine map: reduced flow and efficiency against corrected speed and
/// expansion ratio.
#[derive(Debug, Clone)]
pub struct TurbineMap {
    lines: Vec<TurbineSpeedLine>,
}

impl TurbineMap {
    /// Builds a map from at least two speed lines.
    ///
    /// Panics if fewer than two lines are given — a single speed line cannot
    /// be interpolated in the speed direction at all.
    pub fn new(mut lines: Vec<TurbineSpeedLine>) -> Self {
        lines.sort_by(|a, b| a.corrected_speed.partial_cmp(&b.corrected_speed).unwrap());
        assert!(lines.len() >= 2, "a turbine map needs at least two speed lines");
        Self { lines }
    }

    /// Lowest and highest corrected speed the map has data for [rpm].
    pub fn speed_range(&self) -> (f64, f64) {
        (
            self.lines.first().unwrap().corrected_speed,
            self.lines.last().unwrap().corrected_speed,
        )
    }

    /// Reads reduced flow and efficiency at a corrected operating point.
    ///
    /// Interpolates in speed between the two bracketing lines at the same
    /// expansion ratio on each. Panics if `corrected_speed` falls outside the
    /// map's speed lines, for the same reason
    /// [`crate::physics::compressor::CompressorMap::evaluate`] does: a map is
    /// a measured device and does not extrapolate past its fastest or
    /// slowest line.
    pub fn evaluate(&self, corrected_speed: f64, expansion_ratio: f64) -> TurbineReading {
        let (lo, hi) = self.bracket(corrected_speed);
        let t = (corrected_speed - lo.corrected_speed) / (hi.corrected_speed - lo.corrected_speed);
        let (flow_lo, eff_lo) = lo.interpolate_at(expansion_ratio);
        let (flow_hi, eff_hi) = hi.interpolate_at(expansion_ratio);
        TurbineReading {
            reduced_flow: lerp(flow_lo, flow_hi, t),
            efficiency: lerp(eff_lo, eff_hi, t),
        }
    }

    fn bracket(&self, corrected_speed: f64) -> (&TurbineSpeedLine, &TurbineSpeedLine) {
        let (min, max) = self.speed_range();
        assert!(
            corrected_speed >= min && corrected_speed <= max,
            "corrected speed {corrected_speed} outside turbine map range {min}..={max}: a map \
             does not extrapolate past its fastest or slowest speed line"
        );
        let idx = self
            .lines
            .windows(2)
            .position(|pair| corrected_speed <= pair[1].corrected_speed)
            .unwrap_or(self.lines.len() - 2);
        (&self.lines[idx], &self.lines[idx + 1])
    }
}

/// Converts a reduced flow back to an actual mass flow at real inlet
/// conditions — the inverse of [`compressor::corrected_flow`].
fn reduced_flow_to_mass_flow(reduced_flow: f64, inlet_temperature: f64, inlet_pressure: f64) -> f64 {
    reduced_flow * (inlet_pressure / compressor::P_REF) / (inlet_temperature / compressor::T_REF).sqrt()
}

/// Isentropic specific work extracted per unit mass, as a fraction of the
/// ideal enthalpy drop across `expansion_ratio` [-].
fn specific_extraction(expansion_ratio: f64, gamma: f64, efficiency: f64) -> f64 {
    let exponent = (gamma - 1.0) / gamma;
    let ideal = 1.0 - (1.0 / expansion_ratio.max(1.0)).powf(exponent);
    efficiency * ideal.max(0.0)
}

/// Mass flow, efficiency and expansion ratio the turbine map reads at a
/// given exhaust manifold state.
///
/// `corrected_speed` is clamped to the map's own range before the lookup: a
/// stalled or just-spooling shaft sits below every line the map has, and
/// reading the slowest line's data for it is an honest nearest-neighbour
/// read, not an extrapolation past measured data the way exceeding the map
/// on the high side would be.
pub fn turbine_operating_point(
    upstream: &PortState,
    downstream_pressure: f64,
    map: &TurbineMap,
    corrected_speed: f64,
) -> (f64, f64, f64) {
    let expansion_ratio = (upstream.pressure / downstream_pressure.max(1.0)).max(1.0);
    let (speed_lo, speed_hi) = map.speed_range();
    let reading = map.evaluate(corrected_speed.clamp(speed_lo, speed_hi), expansion_ratio);
    let mass_flow = reduced_flow_to_mass_flow(reading.reduced_flow, upstream.temperature, upstream.pressure);
    (mass_flow, reading.efficiency, expansion_ratio)
}

/// Instantaneous turbine shaft power extracted from the exhaust enthalpy
/// drop [W].
///
/// Takes the manifold's *instantaneous* state, not a cycle-averaged one —
/// see [`turbine_operating_point`]. Feeding this the mean of a pulse train
/// instead of the pulse itself throws away real energy: mass flow and
/// pressure ratio both rise together on a blowdown pulse, so their product
/// integrated over the pulse exceeds the product of their means.
pub fn turbine_power(
    upstream: &PortState,
    downstream_pressure: f64,
    map: &TurbineMap,
    corrected_speed: f64,
) -> f64 {
    let (mass_flow, efficiency, expansion_ratio) =
        turbine_operating_point(upstream, downstream_pressure, map, corrected_speed);
    mass_flow * upstream.cp() * upstream.temperature * specific_extraction(expansion_ratio, upstream.gamma, efficiency)
}

/// Turbocharger bearing cartridge, which sets how much shaft power the
/// bearing itself loses to drag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BearingType {
    /// Full-film journal bearing: cheap, and the higher-drag default.
    Journal,
    /// Ball bearing cartridge: measurably lower drag, which is the entire
    /// reason people buy one.
    BallBearing,
}

impl BearingType {
    /// Viscous drag loss coefficient, `P_bearing = coefficient * w^2` [W/(rad/s)^2].
    fn viscous_loss_coefficient(self) -> f64 {
        match self {
            BearingType::Journal => 1.5e-5,
            BearingType::BallBearing => 5.0e-6,
        }
    }
}

/// The rotating inertia a turbine and compressor wheel share, driven by
/// turbine power and loaded by compressor power and bearing drag.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TurboShaft {
    /// Polar inertia of wheel, shaft and compressor together [kg m^2].
    pub inertia: f64,
    /// Mechanical transmission efficiency between turbine and compressor wheels [-].
    pub mechanical_efficiency: f64,
    /// Bearing cartridge fitted.
    pub bearing: BearingType,
    /// Shaft angular speed [rad/s].
    omega: f64,
    /// Set once the shaft has been clamped at the map's overspeed limit.
    overspeed: bool,
}

impl TurboShaft {
    /// A shaft at rest.
    pub fn new(inertia: f64, mechanical_efficiency: f64, bearing: BearingType) -> Self {
        Self {
            inertia,
            mechanical_efficiency,
            bearing,
            omega: 0.0,
            overspeed: false,
        }
    }

    /// Shaft speed [rpm].
    pub fn shaft_rpm(&self) -> f64 {
        self.omega * 60.0 / (2.0 * PI)
    }

    /// Whether the shaft was clamped at the turbine map's overspeed limit on
    /// the most recent [`Self::advance`].
    pub fn is_overspeed(&self) -> bool {
        self.overspeed
    }

    /// Advances the shaft by `dt` under a turbine power, a compressor power
    /// draw, and this shaft's own bearing drag.
    ///
    /// Integrated in kinetic energy rather than angular speed directly. The
    /// textbook torque form `I dw/dt = (P_t eta - P_c) / w - P_bearing(w)` is
    /// singular at `w = 0`, which sends a stalled shaft's first step to
    /// infinity. Writing the same balance as
    /// `d(0.5 I w^2)/dt = P_t eta - P_c - P_bearing(w)` removes the division
    /// entirely and is exactly the same physics, just integrated in the
    /// variable that is actually smooth at rest.
    pub fn integrate(&mut self, dt: f64, turbine_power: f64, compressor_power: f64) -> f64 {
        let bearing_power = self.bearing.viscous_loss_coefficient() * self.omega * self.omega;
        let net_power = turbine_power * self.mechanical_efficiency - compressor_power - bearing_power;
        let energy = 0.5 * self.inertia * self.omega * self.omega;
        let next_energy = (energy + net_power * dt).max(0.0);
        self.omega = (2.0 * next_energy / self.inertia).sqrt();
        self.shaft_rpm()
    }

    /// Advances the shaft from the instantaneous exhaust manifold state,
    /// reading turbine power off `turbine_map` rather than taking it as a
    /// precomputed input — see [`turbine_power`].
    pub fn advance(
        &mut self,
        dt: f64,
        turbine_upstream: &PortState,
        turbine_downstream_pressure: f64,
        turbine_map: &TurbineMap,
        compressor_power: f64,
    ) -> f64 {
        let corrected_speed = compressor::corrected_speed(self.shaft_rpm(), turbine_upstream.temperature);
        let power = turbine_power(turbine_upstream, turbine_downstream_pressure, turbine_map, corrected_speed);
        self.integrate(dt, power, compressor_power);

        let (_, max_corrected) = turbine_map.speed_range();
        let max_shaft_rpm = max_corrected * (turbine_upstream.temperature / compressor::T_REF).sqrt();
        if self.shaft_rpm() > max_shaft_rpm {
            self.omega = max_shaft_rpm * 2.0 * PI / 60.0;
            self.overspeed = true;
        } else {
            self.overspeed = false;
        }

        self.shaft_rpm()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(expansion_ratio: f64, reduced_flow: f64, efficiency: f64) -> TurbineMapPoint {
        TurbineMapPoint {
            expansion_ratio,
            reduced_flow,
            efficiency,
        }
    }

    fn line(corrected_speed: f64, points: &[(f64, f64, f64)]) -> TurbineSpeedLine {
        TurbineSpeedLine::new(
            corrected_speed,
            points
                .iter()
                .map(|&(pr, flow, eff)| point(pr, flow, eff))
                .collect(),
        )
    }

    pub(super) fn sample_map() -> TurbineMap {
        TurbineMap::new(vec![
            line(
                80_000.0,
                &[
                    (1.0, 0.045, 0.60),
                    (1.5, 0.055, 0.68),
                    (2.0, 0.060, 0.65),
                    (3.0, 0.063, 0.58),
                    (4.0, 0.064, 0.50),
                ],
            ),
            line(
                140_000.0,
                &[
                    (1.0, 0.050, 0.62),
                    (1.5, 0.062, 0.72),
                    (2.0, 0.068, 0.70),
                    (3.0, 0.072, 0.62),
                    (4.0, 0.073, 0.53),
                ],
            ),
        ])
    }

    #[test]
    fn reduced_flow_rises_monotonically_along_a_speed_line() {
        let map = sample_map();
        let mut previous = 0.0;
        let mut pr = 1.05;
        while pr < 3.95 {
            let reading = map.evaluate(80_000.0, pr);
            assert!(
                reading.reduced_flow >= previous - 1e-9,
                "reduced flow fell from {previous} to {} at PR {pr}",
                reading.reduced_flow
            );
            previous = reading.reduced_flow;
            pr += 0.05;
        }
    }

    #[test]
    fn interpolation_does_not_overshoot_between_speed_lines() {
        let map = sample_map();
        let lo = map.evaluate(80_000.0, 2.0).reduced_flow;
        let hi = map.evaluate(140_000.0, 2.0).reduced_flow;
        let mid = map.evaluate(110_000.0, 2.0).reduced_flow;
        let (low, high) = if lo < hi { (lo, hi) } else { (hi, lo) };
        assert!(
            mid >= low - 1e-9 && mid <= high + 1e-9,
            "midpoint reduced flow {mid} overshot the bracket {low}..{high}"
        );
    }

    #[test]
    #[should_panic(expected = "outside turbine map range")]
    fn a_speed_outside_the_map_panics_rather_than_extrapolating() {
        sample_map().evaluate(200_000.0, 2.0);
    }

    fn port(pressure: f64, temperature: f64) -> PortState {
        PortState {
            pressure,
            temperature,
            gas_constant: 287.0,
            gamma: 1.33,
            burned_fraction: 1.0,
        }
    }

    #[test]
    fn pulse_fed_power_exceeds_mean_flow_fed_power_at_the_same_average_mass_flow() {
        let map = sample_map();
        let corrected_speed = 100_000.0;
        let downstream_pressure = 100_000.0;

        let high = port(260_000.0, 1100.0);
        let low = port(100_000.0, 1100.0);
        let mean = port(180_000.0, 1100.0);

        let (m_high, _, _) = turbine_operating_point(&high, downstream_pressure, &map, corrected_speed);
        let (m_low, _, _) = turbine_operating_point(&low, downstream_pressure, &map, corrected_speed);
        let power_high = turbine_power(&high, downstream_pressure, &map, corrected_speed);
        let power_low = turbine_power(&low, downstream_pressure, &map, corrected_speed);
        let pulse_average_power = (power_high + power_low) / 2.0;

        let average_mass_flow = (m_high + m_low) / 2.0;
        let (_, eff_mean, pr_mean) = turbine_operating_point(&mean, downstream_pressure, &map, corrected_speed);
        let mean_flow_power =
            average_mass_flow * mean.cp() * mean.temperature * specific_extraction(pr_mean, mean.gamma, eff_mean);

        assert!(
            pulse_average_power > mean_flow_power,
            "pulse-average power {pulse_average_power} did not exceed mean-flow power \
             {mean_flow_power} at the same average mass flow {average_mass_flow}"
        );
    }

    #[test]
    fn shaft_speed_is_bounded_by_the_maps_overspeed_limit_and_it_is_reported() {
        let map = sample_map();
        let mut shaft = TurboShaft::new(1e-5, 0.97, BearingType::BallBearing);
        let upstream = port(400_000.0, 1100.0);
        let dt = 1.0 / 480.0;

        let (_, max_corrected) = map.speed_range();
        let max_shaft_rpm = max_corrected * (upstream.temperature / compressor::T_REF).sqrt();

        for _ in 0..50_000 {
            shaft.advance(dt, &upstream, 100_000.0, &map, 1000.0);
        }

        assert!(
            shaft.shaft_rpm() <= max_shaft_rpm + 1e-6,
            "shaft speed {} exceeded the map's overspeed limit {}",
            shaft.shaft_rpm(),
            max_shaft_rpm
        );
        assert!(shaft.is_overspeed(), "shaft at its clamp did not report overspeed");
    }

    #[test]
    fn shaft_accelerates_when_turbine_power_exceeds_the_load_and_not_otherwise() {
        let mut shaft = TurboShaft::new(3e-5, 0.97, BearingType::Journal);
        shaft.integrate(1.0 / 480.0, 7000.0, 3000.0);
        assert!(shaft.shaft_rpm() > 0.0, "shaft did not accelerate under net positive power");

        let mut stalled = TurboShaft::new(3e-5, 0.97, BearingType::Journal);
        stalled.integrate(1.0 / 480.0, 2000.0, 3000.0);
        assert_eq!(
            stalled.shaft_rpm(),
            0.0,
            "shaft accelerated with turbine power below the load it must overcome"
        );
    }

    fn time_to_reach(target_rpm: f64, inertia: f64, bearing: BearingType) -> f64 {
        let mut shaft = TurboShaft::new(inertia, 0.97, bearing);
        let dt = 1.0 / 480.0;
        let mut t = 0.0;
        while shaft.shaft_rpm() < target_rpm && t < 10.0 {
            shaft.integrate(dt, 7000.0, 3000.0);
            t += dt;
        }
        assert!(t < 10.0, "shaft never reached {target_rpm} rpm");
        t
    }

    #[test]
    fn a_ball_bearing_spools_faster_than_a_journal_at_the_same_inertia() {
        let journal = time_to_reach(60_000.0, 3e-5, BearingType::Journal);
        let ball = time_to_reach(60_000.0, 3e-5, BearingType::BallBearing);
        assert!(
            ball < journal,
            "ball bearing ({ball}s) did not spool faster than journal ({journal}s)"
        );
    }

    #[test]
    fn steady_state_matches_a_hand_computed_power_balance() {
        let turbine_power = 7000.0;
        let compressor_power = 3000.0;
        let mechanical_efficiency = 0.97;
        let bearing = BearingType::Journal;

        let mut shaft = TurboShaft::new(3e-5, mechanical_efficiency, bearing);
        let dt = 1.0 / 480.0;
        for _ in 0..200_000 {
            shaft.integrate(dt, turbine_power, compressor_power);
        }

        let net_power = turbine_power * mechanical_efficiency - compressor_power;
        let expected_omega = (net_power / bearing.viscous_loss_coefficient()).sqrt();
        let expected_rpm = expected_omega * 60.0 / (2.0 * PI);

        let relative_error = (shaft.shaft_rpm() - expected_rpm).abs() / expected_rpm;
        assert!(
            relative_error < 1e-3,
            "integrated steady state {} rpm did not match hand-computed {} rpm",
            shaft.shaft_rpm(),
            expected_rpm
        );
    }
}
