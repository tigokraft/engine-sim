//! Turbocharger compressor map: pressure ratio and efficiency from corrected
//! speed and corrected flow.
//!
//! Real compressor maps are not analytic surfaces — the interesting behaviour
//! lives at the surge and choke edges, which is exactly where a fitted curve
//! is worst. This module ships small collections of honest `(flow, pressure
//! ratio, efficiency)` points per speed line and interpolates between them,
//! the way a manufacturer's map is actually read.
//!
//! Everything on the map is in *corrected* quantities, because that is the
//! only form in which a map is portable across inlet conditions:
//!
//! ```text
//! N_corr = N / sqrt(T_in / T_ref)
//! m_corr = m sqrt(T_in / T_ref) / (P_in / P_ref)
//! ```

/// SAE reference temperature for corrected quantities [K].
pub const T_REF: f64 = 288.15;
/// SAE reference pressure for corrected quantities [Pa].
pub const P_REF: f64 = 101_325.0;

/// Corrected shaft speed, `N / sqrt(T_in / T_ref)` [rpm].
pub fn corrected_speed(shaft_rpm: f64, inlet_temperature: f64) -> f64 {
    shaft_rpm / (inlet_temperature / T_REF).sqrt()
}

/// Corrected mass flow, `m sqrt(T_in / T_ref) / (P_in / P_ref)` [kg/s].
pub fn corrected_flow(mass_flow: f64, inlet_temperature: f64, inlet_pressure: f64) -> f64 {
    mass_flow * (inlet_temperature / T_REF).sqrt() / (inlet_pressure / P_REF)
}

/// One measured point on a speed line: flow against pressure ratio and
/// isentropic efficiency at that flow.
#[derive(Debug, Clone, Copy)]
pub struct MapPoint {
    /// Corrected mass flow [kg/s].
    pub flow: f64,
    /// Total-to-total pressure ratio [-].
    pub pressure_ratio: f64,
    /// Isentropic efficiency, `0.0..=1.0` [-].
    pub efficiency: f64,
}

/// One constant-corrected-speed curve of the map.
///
/// Points are stored ascending by flow. The first point is the surge line at
/// this speed; the last is the choke line. Real maps are shipped as a small
/// number of honest points, not a fitted curve — the interesting behaviour is
/// at the edges, where a fit is worst.
#[derive(Debug, Clone)]
pub struct SpeedLine {
    corrected_speed: f64,
    points: Vec<MapPoint>,
}

impl SpeedLine {
    /// Builds a speed line from at least a surge and a choke point.
    ///
    /// Panics if fewer than two points are given, or if pressure ratio does
    /// not fall monotonically from surge to choke — a speed line that rises
    /// again is not a compressor map, it is bad data.
    pub fn new(corrected_speed: f64, mut points: Vec<MapPoint>) -> Self {
        points.sort_by(|a, b| a.flow.partial_cmp(&b.flow).expect("flow must not be NaN"));
        assert!(
            points.len() >= 2,
            "a speed line needs at least a surge and a choke point"
        );
        for pair in points.windows(2) {
            assert!(
                pair[1].pressure_ratio <= pair[0].pressure_ratio,
                "speed line at N_corr={corrected_speed} does not fall monotonically: \
                 {:?} then {:?}",
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

    /// Corrected flow at the surge boundary [kg/s].
    pub fn surge_flow(&self) -> f64 {
        self.points.first().expect("validated non-empty").flow
    }

    /// Corrected flow at the choke boundary [kg/s].
    pub fn choke_flow(&self) -> f64 {
        self.points.last().expect("validated non-empty").flow
    }

    /// Pressure ratio and efficiency at a corrected flow, piecewise-linear
    /// between the nearest two measured points and clamped to the line's own
    /// endpoints.
    fn interpolate_at(&self, flow: f64) -> (f64, f64) {
        let clamped = flow.clamp(self.surge_flow(), self.choke_flow());
        let idx = self
            .points
            .windows(2)
            .position(|pair| clamped <= pair[1].flow)
            .unwrap_or(self.points.len() - 2);
        let (lo, hi) = (self.points[idx], self.points[idx + 1]);
        let span = hi.flow - lo.flow;
        let t = if span > 0.0 {
            (clamped - lo.flow) / span
        } else {
            0.0
        };
        (
            lerp(lo.pressure_ratio, hi.pressure_ratio, t),
            lerp(lo.efficiency, hi.efficiency, t),
        )
    }

    /// Pressure ratio and efficiency at a normalized position between surge
    /// (`0.0`) and choke (`1.0`).
    fn interpolate_at_fraction(&self, fraction: f64) -> (f64, f64) {
        let flow = lerp(self.surge_flow(), self.choke_flow(), fraction);
        self.interpolate_at(flow)
    }
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

/// Where a queried point falls relative to the map's surge and choke
/// boundaries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapRegion {
    /// Left of the surge line: the wheel is unloaded and flow will reverse.
    Surge,
    /// Between surge and choke: the map's honest operating range.
    Operating,
    /// Right of the choke line: flow has gone sonic and pressure ratio has
    /// collapsed.
    Choke,
}

/// A pressure ratio and efficiency read off the map, with the region they
/// were read from.
#[derive(Debug, Clone, Copy)]
pub struct MapReading {
    pub region: MapRegion,
    pub pressure_ratio: f64,
    pub efficiency: f64,
}

/// A compressor map: pressure ratio and efficiency against corrected speed
/// and corrected flow, with surge and choke boundaries read off the data
/// rather than typed as constants.
#[derive(Debug, Clone)]
pub struct CompressorMap {
    lines: Vec<SpeedLine>,
}

impl CompressorMap {
    /// Builds a map from at least two speed lines.
    ///
    /// Panics if fewer than two lines are given — a single speed line cannot
    /// be interpolated in the speed direction at all.
    pub fn new(mut lines: Vec<SpeedLine>) -> Self {
        lines.sort_by(|a, b| a.corrected_speed.partial_cmp(&b.corrected_speed).unwrap());
        assert!(
            lines.len() >= 2,
            "a compressor map needs at least two speed lines"
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

    /// Reads pressure ratio, efficiency, and surge/choke region at a
    /// corrected operating point.
    ///
    /// Interpolates in speed between the two bracketing lines at the same
    /// normalized position between each line's own surge and choke flow, so
    /// two lines with different flow ranges do not overshoot each other. The
    /// surge and choke boundaries are themselves interpolated from the map's
    /// own speed lines, not read from a constant.
    ///
    /// Panics if `corrected_speed` falls outside the map's speed lines: a map
    /// is a measured device, and extrapolating past its fastest or slowest
    /// speed line invents data nobody measured.
    pub fn evaluate(&self, corrected_speed: f64, corrected_flow: f64) -> MapReading {
        let (lo, hi) = self.bracket(corrected_speed);
        let t = (corrected_speed - lo.corrected_speed) / (hi.corrected_speed - lo.corrected_speed);

        let surge_bound = lerp(lo.surge_flow(), hi.surge_flow(), t);
        let choke_bound = lerp(lo.choke_flow(), hi.choke_flow(), t);
        let region = if corrected_flow < surge_bound {
            MapRegion::Surge
        } else if corrected_flow > choke_bound {
            MapRegion::Choke
        } else {
            MapRegion::Operating
        };

        let fraction =
            ((corrected_flow - surge_bound) / (choke_bound - surge_bound)).clamp(0.0, 1.0);
        let (pr_lo, eff_lo) = lo.interpolate_at_fraction(fraction);
        let (pr_hi, eff_hi) = hi.interpolate_at_fraction(fraction);

        MapReading {
            region,
            pressure_ratio: lerp(pr_lo, pr_hi, t),
            efficiency: lerp(eff_lo, eff_hi, t),
        }
    }

    /// The corrected flow that would produce a given pressure ratio at a
    /// corrected speed, found by bisection against [`Self::evaluate`].
    ///
    /// The map is naturally organised the other way round — flow in,
    /// pressure ratio out — because that is how a manufacturer measures it.
    /// But in the running engine, flow is what the downstream system (the
    /// throttle and the cylinders behind it) actually sets, and pressure
    /// ratio is what results, so the inverse read is the one the physics
    /// needs. Pressure ratio falls monotonically with flow on every speed
    /// line, so bisection converges to the unique answer, clamping to the
    /// surge or choke boundary if `pressure_ratio` is not achievable at all.
    pub fn flow_for_pressure_ratio(&self, corrected_speed: f64, pressure_ratio: f64) -> (f64, MapReading) {
        let speed = corrected_speed.clamp(self.speed_range().0, self.speed_range().1);
        let (lo_line, hi_line) = self.bracket(speed);
        let mut lo = 0.0f64;
        // Bounding the search at the bracketing lines' own choke flow, rather
        // than an arbitrary large number, matters past the choke boundary:
        // `evaluate` clamps there, so *every* flow past choke reads back the
        // same plateaued pressure ratio, and an unreachably low target would
        // otherwise walk the search out to whatever the bound was instead of
        // stopping at the choke point the map actually measured.
        let mut hi = lo_line.choke_flow().max(hi_line.choke_flow());
        for _ in 0..32 {
            let mid = 0.5 * (lo + hi);
            if self.evaluate(speed, mid).pressure_ratio > pressure_ratio {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let flow = 0.5 * (lo + hi);
        (flow, self.evaluate(speed, flow))
    }

    fn bracket(&self, corrected_speed: f64) -> (&SpeedLine, &SpeedLine) {
        let (min, max) = self.speed_range();
        assert!(
            corrected_speed >= min && corrected_speed <= max,
            "corrected speed {corrected_speed} outside map range {min}..={max}: a map \
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

/// Discharge temperature from the isentropic compression relation.
///
/// ```text
/// T_out = T_in (1 + (PR^((gamma-1)/gamma) - 1) / eta)
/// ```
///
/// A lower efficiency turns more of the compression work into heat rather
/// than pressure rise, so it gives a hotter charge at the same pressure
/// ratio — a map without efficiency gives free power.
pub fn discharge_temperature(
    inlet_temperature: f64,
    pressure_ratio: f64,
    efficiency: f64,
    gamma: f64,
) -> f64 {
    let exponent = (gamma - 1.0) / gamma;
    inlet_temperature * (1.0 + (pressure_ratio.powf(exponent) - 1.0) / efficiency)
}

/// Shaft power the compressor draws to produce `pressure_ratio` at
/// `mass_flow` [W].
///
/// The enthalpy rise the gas actually receives, `m cp (T_out - T_in)`, with
/// `T_out` from [`discharge_temperature`] — a map without efficiency gives
/// free power, so the shaft must pay for exactly the temperature rise the
/// map's efficiency predicts, not the isentropic minimum.
pub fn compressor_power(
    mass_flow: f64,
    inlet_temperature: f64,
    pressure_ratio: f64,
    efficiency: f64,
    gas_constant: f64,
    gamma: f64,
) -> f64 {
    let cp = gamma * gas_constant / (gamma - 1.0);
    let outlet_temperature = discharge_temperature(inlet_temperature, pressure_ratio, efficiency, gamma);
    mass_flow.max(0.0) * cp * (outlet_temperature - inlet_temperature)
}

/// Turbocharger frame size, used to pick one of the stock maps below.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameSize {
    Small,
    Medium,
    Large,
}

impl CompressorMap {
    /// A stock map for the given frame size.
    ///
    /// These are representative small/medium/large single-turbo maps, sized
    /// so a preset can say "small single" or "large single" without typing
    /// its own speed lines.
    pub fn stock(frame: FrameSize) -> Self {
        match frame {
            FrameSize::Small => small_frame_map(),
            FrameSize::Medium => medium_frame_map(),
            FrameSize::Large => large_frame_map(),
        }
    }
}

fn line(corrected_speed: f64, points: &[(f64, f64, f64)]) -> SpeedLine {
    SpeedLine::new(
        corrected_speed,
        points
            .iter()
            .map(|&(flow, pressure_ratio, efficiency)| MapPoint {
                flow,
                pressure_ratio,
                efficiency,
            })
            .collect(),
    )
}

fn small_frame_map() -> CompressorMap {
    CompressorMap::new(vec![
        line(60_000.0, &[(0.020, 1.45, 0.62), (0.045, 1.35, 0.74), (0.065, 1.05, 0.58)]),
        line(90_000.0, &[(0.035, 1.95, 0.66), (0.065, 1.80, 0.77), (0.090, 1.30, 0.60)]),
        line(120_000.0, &[(0.050, 2.55, 0.65), (0.085, 2.35, 0.76), (0.115, 1.55, 0.58)]),
        line(150_000.0, &[(0.060, 3.05, 0.60), (0.100, 2.75, 0.72), (0.135, 1.70, 0.54)]),
    ])
}

fn medium_frame_map() -> CompressorMap {
    CompressorMap::new(vec![
        line(50_000.0, &[(0.045, 1.40, 0.63), (0.090, 1.30, 0.75), (0.130, 1.05, 0.59)]),
        line(75_000.0, &[(0.070, 1.90, 0.67), (0.130, 1.75, 0.78), (0.180, 1.30, 0.61)]),
        line(100_000.0, &[(0.095, 2.50, 0.66), (0.170, 2.30, 0.77), (0.230, 1.55, 0.59)]),
        line(125_000.0, &[(0.115, 3.00, 0.61), (0.200, 2.70, 0.73), (0.270, 1.70, 0.55)]),
    ])
}

fn large_frame_map() -> CompressorMap {
    CompressorMap::new(vec![
        line(40_000.0, &[(0.090, 1.35, 0.64), (0.180, 1.25, 0.76), (0.260, 1.05, 0.60)]),
        line(60_000.0, &[(0.140, 1.85, 0.68), (0.260, 1.70, 0.79), (0.360, 1.30, 0.62)]),
        line(80_000.0, &[(0.190, 2.45, 0.67), (0.340, 2.25, 0.78), (0.460, 1.55, 0.60)]),
        line(100_000.0, &[(0.230, 2.95, 0.62), (0.400, 2.65, 0.74), (0.540, 1.70, 0.56)]),
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() <= tol, "expected {b}, got {a} (tol {tol})");
    }

    #[test]
    fn corrected_speed_is_inlet_invariant() {
        // The same physical shaft speed scaled to a hotter inlet must land
        // back on the same corrected speed once the scaling is undone.
        let reference = corrected_speed(80_000.0, T_REF);
        let hotter_shaft_rpm = 80_000.0 * (310.0 / T_REF).sqrt();
        let scaled = corrected_speed(hotter_shaft_rpm, 310.0);
        approx_eq(scaled, reference, 1e-6);
    }

    #[test]
    fn corrected_flow_is_inlet_invariant() {
        let reference = corrected_flow(0.05, T_REF, P_REF);
        // Solve corrected_flow(m, 310, 110_000) == reference for m.
        let scaled_mass = reference * (110_000.0 / P_REF) / (310.0 / T_REF).sqrt();
        let scaled = corrected_flow(scaled_mass, 310.0, 110_000.0);
        approx_eq(scaled, reference, 1e-6);
    }

    fn point(flow: f64, pressure_ratio: f64, efficiency: f64) -> MapPoint {
        MapPoint {
            flow,
            pressure_ratio,
            efficiency,
        }
    }

    fn sample_map() -> CompressorMap {
        CompressorMap::new(vec![
            SpeedLine::new(
                60_000.0,
                vec![point(0.02, 1.5, 0.6), point(0.04, 1.4, 0.72), point(0.06, 1.1, 0.55)],
            ),
            SpeedLine::new(
                90_000.0,
                vec![point(0.03, 2.0, 0.62), point(0.06, 1.85, 0.75), point(0.09, 1.35, 0.58)],
            ),
            SpeedLine::new(
                120_000.0,
                vec![point(0.04, 2.5, 0.6), point(0.08, 2.3, 0.74), point(0.12, 1.6, 0.56)],
            ),
        ])
    }

    #[test]
    fn interpolation_is_monotone_along_a_speed_line() {
        let map = sample_map();
        let mut previous = f64::INFINITY;
        let mut flow = 0.021;
        while flow < 0.059 {
            let reading = map.evaluate(60_000.0, flow);
            assert!(
                reading.pressure_ratio <= previous + 1e-9,
                "pressure ratio rose from {previous} to {} at flow {flow}",
                reading.pressure_ratio
            );
            previous = reading.pressure_ratio;
            flow += 0.001;
        }
    }

    #[test]
    fn interpolation_does_not_overshoot_between_speed_lines() {
        let map = sample_map();
        let lo = map.evaluate(60_000.0, 0.03).pressure_ratio;
        let hi = map.evaluate(90_000.0, 0.03).pressure_ratio;
        let mid = map.evaluate(75_000.0, 0.03).pressure_ratio;
        let (low, high) = if lo < hi { (lo, hi) } else { (hi, lo) };
        assert!(
            mid >= low - 1e-9 && mid <= high + 1e-9,
            "midpoint pressure ratio {mid} overshot the bracket {low}..{high}"
        );
    }

    #[test]
    #[should_panic(expected = "outside map range")]
    fn a_speed_outside_the_map_panics_rather_than_extrapolating() {
        sample_map().evaluate(200_000.0, 0.04);
    }

    #[test]
    fn a_point_left_of_surge_reports_surging() {
        let reading = sample_map().evaluate(90_000.0, 0.01);
        assert_eq!(reading.region, MapRegion::Surge);
    }

    #[test]
    fn a_point_right_of_choke_reports_choked() {
        let reading = sample_map().evaluate(90_000.0, 0.5);
        assert_eq!(reading.region, MapRegion::Choke);
    }

    #[test]
    fn boundaries_come_from_the_map_not_a_constant() {
        // Two maps that disagree only about where surge starts must disagree
        // about whether the same point is surging.
        let narrow = CompressorMap::new(vec![
            SpeedLine::new(60_000.0, vec![point(0.02, 1.5, 0.6), point(0.06, 1.1, 0.55)]),
            SpeedLine::new(90_000.0, vec![point(0.03, 2.0, 0.62), point(0.09, 1.35, 0.58)]),
        ]);
        let wide = CompressorMap::new(vec![
            SpeedLine::new(60_000.0, vec![point(0.005, 1.5, 0.6), point(0.06, 1.1, 0.55)]),
            SpeedLine::new(90_000.0, vec![point(0.010, 2.0, 0.62), point(0.09, 1.35, 0.58)]),
        ]);
        assert_eq!(narrow.evaluate(75_000.0, 0.012).region, MapRegion::Surge);
        assert_eq!(wide.evaluate(75_000.0, 0.012).region, MapRegion::Operating);
    }

    fn approx(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() <= tol, "expected {b}, got {a} (tol {tol})");
    }

    #[test]
    fn discharge_temperature_follows_isentropic_relation() {
        let t_out = discharge_temperature(300.0, 2.0, 0.7, 1.4);
        let expected = 300.0 * (1.0 + (2.0_f64.powf(1.0 / 3.5) - 1.0) / 0.7);
        approx(t_out, expected, 1e-9);
    }

    #[test]
    fn lower_efficiency_gives_a_hotter_charge_at_the_same_pressure_ratio() {
        let efficient = discharge_temperature(300.0, 2.2, 0.78, 1.4);
        let inefficient = discharge_temperature(300.0, 2.2, 0.55, 1.4);
        assert!(inefficient > efficient);
    }

    #[test]
    fn stock_maps_span_a_wider_flow_range_as_frame_grows() {
        let small = CompressorMap::stock(FrameSize::Small);
        let medium = CompressorMap::stock(FrameSize::Medium);
        let large = CompressorMap::stock(FrameSize::Large);
        let choke = |map: &CompressorMap| map.lines.last().unwrap().choke_flow();
        assert!(choke(&medium) > choke(&small));
        assert!(choke(&large) > choke(&medium));
    }

    #[test]
    fn a_stock_map_reports_an_operating_point_at_its_own_mid_speed_line() {
        for frame in [FrameSize::Small, FrameSize::Medium, FrameSize::Large] {
            let map = CompressorMap::stock(frame);
            let mid_line = &map.lines[map.lines.len() / 2];
            let mid_flow = (mid_line.surge_flow() + mid_line.choke_flow()) / 2.0;
            let reading = map.evaluate(mid_line.corrected_speed(), mid_flow);
            assert_eq!(reading.region, MapRegion::Operating);
        }
    }

    #[test]
    fn flow_for_pressure_ratio_inverts_evaluate() {
        let map = sample_map();
        let forward = map.evaluate(90_000.0, 0.05);
        let (flow, reading) = map.flow_for_pressure_ratio(90_000.0, forward.pressure_ratio);
        approx(flow, 0.05, 1e-3);
        approx(reading.pressure_ratio, forward.pressure_ratio, 1e-3);
    }

    #[test]
    fn flow_for_pressure_ratio_clamps_at_the_map_edges() {
        let map = sample_map();
        // Nothing on this speed line reaches a pressure ratio of 10; the
        // inverse read must land at the surge point, not somewhere invented.
        let (_, reading) = map.flow_for_pressure_ratio(90_000.0, 10.0);
        assert_eq!(reading.region, MapRegion::Surge);
    }

    #[test]
    fn compressor_power_is_zero_at_unity_pressure_ratio() {
        let power = compressor_power(0.05, 300.0, 1.0, 0.7, 287.0, 1.4);
        approx(power, 0.0, 1e-6);
    }

    #[test]
    fn compressor_power_rises_with_pressure_ratio() {
        let low = compressor_power(0.05, 300.0, 1.5, 0.7, 287.0, 1.4);
        let high = compressor_power(0.05, 300.0, 2.5, 0.7, 287.0, 1.4);
        assert!(high > low);
    }
}
