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
}
