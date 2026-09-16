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

#[cfg(test)]
mod tests {
    use super::*;

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
}
