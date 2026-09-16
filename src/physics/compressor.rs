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
