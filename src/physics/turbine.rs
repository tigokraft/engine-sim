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
use crate::physics::thermodynamics::{flow_function, PortState};

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
        self.points
            .first()
            .expect("validated non-empty")
            .expansion_ratio
    }

    /// Highest expansion ratio this line has data for [-].
    pub fn max_expansion_ratio(&self) -> f64 {
        self.points
            .last()
            .expect("validated non-empty")
            .expansion_ratio
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
        assert!(
            lines.len() >= 2,
            "a turbine map needs at least two speed lines"
        );
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

impl TurbineMap {
    /// A stock turbine map sized to match [`crate::physics::compressor::CompressorMap::stock`]'s
    /// frame — a preset can say "small single" once and get a matched pair.
    ///
    /// Turbine maps are far flatter than compressor maps, so the same three
    /// points per line used there are still more than the interesting
    /// behaviour needs.
    pub fn stock(frame: compressor::FrameSize) -> Self {
        use compressor::FrameSize;
        match frame {
            FrameSize::Small => small_frame_map(),
            FrameSize::Medium => medium_frame_map(),
            FrameSize::Large => large_frame_map(),
        }
    }
}

fn turbine_line(corrected_speed: f64, points: &[(f64, f64, f64)]) -> TurbineSpeedLine {
    TurbineSpeedLine::new(
        corrected_speed,
        points
            .iter()
            .map(
                |&(expansion_ratio, reduced_flow, efficiency)| TurbineMapPoint {
                    expansion_ratio,
                    reduced_flow,
                    efficiency,
                },
            )
            .collect(),
    )
}

fn small_frame_map() -> TurbineMap {
    TurbineMap::new(vec![
        turbine_line(
            50_000.0,
            &[(1.0, 0.020, 0.48), (1.6, 0.060, 0.68), (2.6, 0.095, 0.58)],
        ),
        turbine_line(
            150_000.0,
            &[(1.0, 0.030, 0.52), (2.0, 0.100, 0.74), (3.2, 0.150, 0.60)],
        ),
    ])
}

fn medium_frame_map() -> TurbineMap {
    TurbineMap::new(vec![
        turbine_line(
            40_000.0,
            &[(1.0, 0.045, 0.50), (1.6, 0.130, 0.70), (2.6, 0.205, 0.60)],
        ),
        turbine_line(
            125_000.0,
            &[(1.0, 0.065, 0.54), (2.0, 0.215, 0.76), (3.2, 0.320, 0.62)],
        ),
    ])
}

fn large_frame_map() -> TurbineMap {
    TurbineMap::new(vec![
        turbine_line(
            30_000.0,
            &[(1.0, 0.085, 0.52), (1.6, 0.245, 0.71), (2.6, 0.385, 0.61)],
        ),
        turbine_line(
            100_000.0,
            &[(1.0, 0.120, 0.55), (2.0, 0.400, 0.77), (3.2, 0.590, 0.63)],
        ),
    ])
}

/// Converts a reduced flow back to an actual mass flow at real inlet
/// conditions — the inverse of [`compressor::corrected_flow`].
fn reduced_flow_to_mass_flow(
    reduced_flow: f64,
    inlet_temperature: f64,
    inlet_pressure: f64,
) -> f64 {
    reduced_flow * (inlet_pressure / compressor::P_REF)
        / (inlet_temperature / compressor::T_REF).sqrt()
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
    let mass_flow = reduced_flow_to_mass_flow(
        reading.reduced_flow,
        upstream.temperature,
        upstream.pressure,
    );
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
    mass_flow
        * upstream.cp()
        * upstream.temperature
        * specific_extraction(expansion_ratio, upstream.gamma, efficiency)
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

/// Turbine inlet temperature limit typical of an automotive single-scroll
/// housing (~950 C) [K]. Not enforced here; reported so a later stage can
/// gate anti-lag and overboost behaviour against it.
pub const TURBINE_INLET_TEMPERATURE_LIMIT: f64 = 1223.15;

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
    /// Most recent turbine inlet temperature seen by [`Self::advance`] [K].
    turbine_inlet_temperature: f64,
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
            turbine_inlet_temperature: compressor::T_REF,
            overspeed: false,
        }
    }

    /// Shaft speed [rpm].
    pub fn shaft_rpm(&self) -> f64 {
        self.omega * 60.0 / (2.0 * PI)
    }

    /// Most recent turbine inlet temperature seen by [`Self::advance`] [K].
    pub fn turbine_inlet_temperature(&self) -> f64 {
        self.turbine_inlet_temperature
    }

    /// Whether the most recent inlet temperature is over
    /// [`TURBINE_INLET_TEMPERATURE_LIMIT`].
    pub fn is_over_temperature_limit(&self) -> bool {
        self.turbine_inlet_temperature > TURBINE_INLET_TEMPERATURE_LIMIT
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
        let net_power =
            turbine_power * self.mechanical_efficiency - compressor_power - bearing_power;
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
        self.turbine_inlet_temperature = turbine_upstream.temperature;

        let corrected_speed =
            compressor::corrected_speed(self.shaft_rpm(), turbine_upstream.temperature);
        let power = turbine_power(
            turbine_upstream,
            turbine_downstream_pressure,
            turbine_map,
            corrected_speed,
        );
        self.integrate(dt, power, compressor_power);

        let (_, max_corrected) = turbine_map.speed_range();
        let max_shaft_rpm =
            max_corrected * (turbine_upstream.temperature / compressor::T_REF).sqrt();
        if self.shaft_rpm() > max_shaft_rpm {
            self.omega = max_shaft_rpm * 2.0 * PI / 60.0;
            self.overspeed = true;
        } else {
            self.overspeed = false;
        }

        self.shaft_rpm()
    }

    /// Advances the shaft from two separate exhaust inlets feeding a
    /// twin-scroll housing, and returns each inlet's own mass flow through
    /// the wheel — see [`Self::advance`], which this generalizes to more
    /// than one simultaneous inlet.
    ///
    /// The map is evaluated separately against each inlet's own
    /// instantaneous state at the shared shaft speed and the two resulting
    /// powers are summed, rather than evaluating once against a
    /// flux-weighted mean of the two states. [`turbine_power`]'s own doc
    /// explains why an instantaneous evaluation keeps pulse energy a mean
    /// throws away; the same reasoning applies across two inlets held
    /// simultaneously apart in separate scrolls as it does across one
    /// inlet's own time history, and is the entire acoustic point of
    /// dividing the manifold in the first place — see
    /// `docs/TURBO_PLAN.md`'s TB5.
    pub fn advance_scrolls(
        &mut self,
        dt: f64,
        inlets: &[PortState; 2],
        turbine_downstream_pressure: f64,
        turbine_map: &TurbineMap,
        compressor_power: f64,
    ) -> [f64; 2] {
        self.turbine_inlet_temperature = inlets[0].temperature.max(inlets[1].temperature);

        let corrected_speed =
            compressor::corrected_speed(self.shaft_rpm(), self.turbine_inlet_temperature);
        let total_power: f64 = inlets
            .iter()
            .map(|inlet| {
                turbine_power(
                    inlet,
                    turbine_downstream_pressure,
                    turbine_map,
                    corrected_speed,
                )
            })
            .sum();
        self.integrate(dt, total_power, compressor_power);

        let (_, max_corrected) = turbine_map.speed_range();
        let max_shaft_rpm =
            max_corrected * (self.turbine_inlet_temperature / compressor::T_REF).sqrt();
        if self.shaft_rpm() > max_shaft_rpm {
            self.omega = max_shaft_rpm * 2.0 * PI / 60.0;
            self.overspeed = true;
        } else {
            self.overspeed = false;
        }

        let mut flows = [0.0; 2];
        for (i, inlet) in inlets.iter().enumerate() {
            let (mass_flow, _, _) = turbine_operating_point(
                inlet,
                turbine_downstream_pressure,
                turbine_map,
                corrected_speed,
            );
            flows[i] = mass_flow;
        }
        flows
    }
}

/// Where a wastegate's bypassed flow ends up.
///
/// See `docs/TURBO_PLAN.md`'s TB4: the two fitments sound entirely
/// different, because they are entirely different acoustic paths, not a
/// voicing choice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WastegateFitment {
    /// Dumps into the downpipe after the turbine, silenced with the rest of
    /// the exhaust.
    Internal,
    /// Dumps to atmosphere through its own screamer pipe — a second,
    /// unsilenced radiating aperture upstream of everything else.
    External,
}

/// A wastegate: a real bypass around the turbine, with a flow area, a spring
/// preload, and whatever bias a [`BoostController`] adds on top of it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Wastegate {
    pub fitment: WastegateFitment,
    /// Fully open bypass flow area [m^2].
    pub max_flow_area: f64,
    /// Manifold gauge pressure at which the spring alone starts to lift the
    /// valve, with no actuator bias applied [Pa].
    pub spring_preload: f64,
    /// Gauge pressure span from just-cracked to fully open [Pa].
    pub opening_span: f64,
}

impl Wastegate {
    /// Open fraction, `0..=1`, at a manifold gauge pressure once any
    /// actuator bias has already been subtracted from the spring's own
    /// threshold — see [`BoostController::update`].
    pub fn open_fraction(&self, gauge_pressure: f64, effective_threshold: f64) -> f64 {
        ((gauge_pressure - effective_threshold) / self.opening_span.max(1.0)).clamp(0.0, 1.0)
    }

    /// Mass flow bypassed around the turbine wheel [kg/s].
    pub fn mass_flow(
        &self,
        upstream: &PortState,
        downstream_pressure: f64,
        effective_threshold: f64,
    ) -> f64 {
        let gauge_pressure = upstream.pressure - downstream_pressure;
        let area = self.max_flow_area * self.open_fraction(gauge_pressure, effective_threshold);
        if area <= 0.0 {
            return 0.0;
        }
        let p_up = upstream.pressure.max(1.0);
        let p_down = downstream_pressure.max(1.0);
        if p_down >= p_up {
            return 0.0;
        }
        area * p_up / (upstream.gas_constant * upstream.temperature.max(1.0)).sqrt()
            * flow_function(p_down / p_up, upstream.gamma)
    }
}

/// Closed-loop boost control: a target pressure ratio and a controller with
/// real authority limits, biasing a [`Wastegate`]'s effective spring
/// threshold rather than commanding its position directly — an actuator
/// pushes on the same diaphragm the spring does, it does not replace it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoostController {
    /// Pressure ratio this loop tries to hold [-].
    pub target_pressure_ratio: f64,
    /// Proportional gain, pressure bias per unit of pressure-ratio error
    /// [Pa].
    pub gain: f64,
    /// Largest threshold reduction the actuator can command [Pa].
    pub authority: f64,
    /// First-order response time of the actuator itself [s]. Real actuators
    /// do not move instantly, and this lag is what lets a fast tip-in
    /// overshoot the target before the loop catches up — the overshoot is a
    /// property of the loop, not something added on top of it.
    pub actuator_lag: f64,
    /// Current threshold reduction the actuator is holding [Pa].
    bias: f64,
}

impl BoostController {
    /// A controller at rest, with no actuator bias yet applied.
    pub fn new(target_pressure_ratio: f64, gain: f64, authority: f64, actuator_lag: f64) -> Self {
        Self {
            target_pressure_ratio,
            gain,
            authority,
            actuator_lag,
            bias: 0.0,
        }
    }

    /// Current actuator bias [Pa]: the amount subtracted from the
    /// wastegate's spring threshold.
    pub fn bias(&self) -> f64 {
        self.bias
    }

    /// Advances the actuator by `dt` toward the bias this frame's measured
    /// pressure ratio calls for, and returns the new bias.
    pub fn update(&mut self, dt: f64, measured_pressure_ratio: f64) -> f64 {
        let error = measured_pressure_ratio - self.target_pressure_ratio;
        let desired = (self.gain * error).clamp(-self.authority, self.authority);
        let alpha = if self.actuator_lag > 1e-6 {
            1.0 - (-dt.max(0.0) / self.actuator_lag).exp()
        } else {
            1.0
        };
        self.bias += (desired - self.bias) * alpha;
        self.bias
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

        let (m_high, _, _) =
            turbine_operating_point(&high, downstream_pressure, &map, corrected_speed);
        let (m_low, _, _) =
            turbine_operating_point(&low, downstream_pressure, &map, corrected_speed);
        let power_high = turbine_power(&high, downstream_pressure, &map, corrected_speed);
        let power_low = turbine_power(&low, downstream_pressure, &map, corrected_speed);
        let pulse_average_power = (power_high + power_low) / 2.0;

        let average_mass_flow = (m_high + m_low) / 2.0;
        let (_, eff_mean, pr_mean) =
            turbine_operating_point(&mean, downstream_pressure, &map, corrected_speed);
        let mean_flow_power = average_mass_flow
            * mean.cp()
            * mean.temperature
            * specific_extraction(pr_mean, mean.gamma, eff_mean);

        assert!(
            pulse_average_power > mean_flow_power,
            "pulse-average power {pulse_average_power} did not exceed mean-flow power \
             {mean_flow_power} at the same average mass flow {average_mass_flow}"
        );
    }

    #[test]
    fn twin_scroll_inlets_spin_the_shaft_faster_than_a_merged_mean_inlet() {
        // The same correlation the pulse-fed test above measures in time —
        // mass flow and pressure rising together — also holds between two
        // scrolls held simultaneously apart: two separated inlets deliver
        // more shaft power than one inlet flux-weighted down to their mean
        // pressure, at the same average flow. This is TB5's whole acoustic
        // case for dividing the manifold in the first place.
        let map = sample_map();
        let downstream_pressure = 100_000.0;
        let dt = 1.0 / 480.0;

        let high = port(260_000.0, 1100.0);
        let low = port(100_000.0, 1100.0);
        let mean = port(180_000.0, 1100.0);

        let mut divided = TurboShaft::new(1e-5, 0.97, BearingType::BallBearing);
        divided.advance_scrolls(dt, &[high, low], downstream_pressure, &map, 0.0);

        // A merged single inlet at the mean pressure, driven by the same
        // corrected speed (both shafts start at rest) so the comparison
        // isolates the map's response to two separated pressures against
        // one averaged one, rather than an artifact of integration order.
        let mut merged = TurboShaft::new(1e-5, 0.97, BearingType::BallBearing);
        let corrected_speed = compressor::corrected_speed(0.0, mean.temperature);
        let power = 2.0 * turbine_power(&mean, downstream_pressure, &map, corrected_speed);
        merged.integrate(dt, power, 0.0);

        assert!(
            divided.shaft_rpm() > merged.shaft_rpm(),
            "twin-scroll inlets should spin the shaft faster than a merged \
             mean inlet at the same average flow: divided={:.0} rpm, merged={:.0} rpm",
            divided.shaft_rpm(),
            merged.shaft_rpm()
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
        assert!(
            shaft.is_overspeed(),
            "shaft at its clamp did not report overspeed"
        );
    }

    #[test]
    fn turbine_inlet_temperature_is_reported_and_checked_against_its_limit() {
        let map = sample_map();
        let mut shaft = TurboShaft::new(3e-5, 0.97, BearingType::Journal);

        let cool = port(200_000.0, 900.0);
        shaft.advance(1.0 / 480.0, &cool, 100_000.0, &map, 1000.0);
        assert_eq!(shaft.turbine_inlet_temperature(), 900.0);
        assert!(!shaft.is_over_temperature_limit());

        let scorching = port(200_000.0, 1400.0);
        shaft.advance(1.0 / 480.0, &scorching, 100_000.0, &map, 1000.0);
        assert_eq!(shaft.turbine_inlet_temperature(), 1400.0);
        assert!(shaft.is_over_temperature_limit());
    }

    #[test]
    fn shaft_accelerates_when_turbine_power_exceeds_the_load_and_not_otherwise() {
        let mut shaft = TurboShaft::new(3e-5, 0.97, BearingType::Journal);
        shaft.integrate(1.0 / 480.0, 7000.0, 3000.0);
        assert!(
            shaft.shaft_rpm() > 0.0,
            "shaft did not accelerate under net positive power"
        );

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
    fn a_larger_wheel_inertia_lags_a_given_step_longer() {
        let small = time_to_reach(60_000.0, 1e-5, BearingType::Journal);
        let large = time_to_reach(60_000.0, 8e-5, BearingType::Journal);
        assert!(
            large > small,
            "larger inertia ({large}s) did not lag longer than smaller inertia ({small}s)"
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

    #[test]
    fn stock_maps_span_more_flow_as_frame_grows() {
        use compressor::FrameSize;
        let small = TurbineMap::stock(FrameSize::Small);
        let medium = TurbineMap::stock(FrameSize::Medium);
        let large = TurbineMap::stock(FrameSize::Large);
        let max_flow = |map: &TurbineMap| map.evaluate(map.speed_range().1, 3.2).reduced_flow;
        assert!(max_flow(&medium) > max_flow(&small));
        assert!(max_flow(&large) > max_flow(&medium));
    }

    fn wastegate() -> Wastegate {
        Wastegate {
            fitment: WastegateFitment::Internal,
            max_flow_area: 8.0e-4,
            spring_preload: 0.7e5,
            opening_span: 0.3e5,
        }
    }

    fn hot_exhaust(pressure: f64) -> PortState {
        PortState {
            pressure,
            temperature: 950.0,
            gas_constant: 290.0,
            gamma: 1.32,
            burned_fraction: 1.0,
        }
    }

    #[test]
    fn a_wastegate_stays_shut_below_its_spring_preload() {
        let wg = wastegate();
        assert_eq!(wg.open_fraction(0.0, wg.spring_preload), 0.0);
        assert_eq!(
            wg.open_fraction(wg.spring_preload * 0.5, wg.spring_preload),
            0.0
        );
    }

    #[test]
    fn a_wastegate_opens_across_its_spring_span_and_flows_more_open() {
        let wg = wastegate();
        let mid = wg.spring_preload + wg.opening_span * 0.5;
        let full = wg.spring_preload + wg.opening_span * 2.0;
        assert!((wg.open_fraction(mid, wg.spring_preload) - 0.5).abs() < 1e-9);
        assert_eq!(wg.open_fraction(full, wg.spring_preload), 1.0);

        let upstream = hot_exhaust(2.0e5);
        let downstream = 1.0e5;
        let half_open = wg.mass_flow(
            &upstream,
            downstream,
            wg.spring_preload + wg.opening_span * 1.5,
        );
        let fully_open = wg.mass_flow(&upstream, downstream, wg.spring_preload);
        assert!(fully_open > half_open);
    }

    #[test]
    fn an_actuator_bias_lowers_the_effective_opening_threshold() {
        let wg = wastegate();
        let gauge = wg.spring_preload - 0.1e5;
        // Below the bare spring threshold, shut...
        assert_eq!(wg.open_fraction(gauge, wg.spring_preload), 0.0);
        // ...but a controller bias lowering the threshold opens it early.
        let biased_threshold = wg.spring_preload - 0.2e5;
        assert!(wg.open_fraction(gauge, biased_threshold) > 0.0);
    }

    #[test]
    fn boost_controller_reduces_the_threshold_once_over_target() {
        let mut ctl = BoostController::new(1.8, 2.0e5, 0.5e5, 0.05);
        for _ in 0..2_000 {
            ctl.update(1.0 / 480.0, 2.0);
        }
        assert!(
            ctl.bias() > 0.0,
            "over target must produce a positive (threshold-lowering) bias"
        );

        let mut relieved = BoostController::new(1.8, 2.0e5, 0.5e5, 0.05);
        for _ in 0..2_000 {
            relieved.update(1.0 / 480.0, 1.8);
        }
        assert!(
            relieved.bias().abs() < 1.0,
            "on target, the bias must settle back near zero: {}",
            relieved.bias()
        );
    }

    #[test]
    fn boost_controller_bias_is_authority_limited() {
        let mut ctl = BoostController::new(1.0, 10.0e5, 0.3e5, 0.02);
        for _ in 0..2_000 {
            ctl.update(1.0 / 480.0, 5.0);
        }
        assert!((ctl.bias() - ctl.authority).abs() < 1.0);
    }

    #[test]
    fn boost_controller_actuator_lags_a_step_change() {
        let mut ctl = BoostController::new(1.0, 5.0e5, 1.0e5, 0.2);
        ctl.update(1.0 / 480.0, 2.0);
        assert!(
            ctl.bias() < ctl.authority * 0.5,
            "a real actuator cannot jump straight to full bias in one small step: {}",
            ctl.bias()
        );
    }
}
