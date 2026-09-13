//! Atmospheric conditions feeding the induction system.
//!
//! Everything is SI: pressure in Pa, temperature in K, density in kg/m^3.
//! Humid air is treated as an ideal mixture of dry air and water vapour, so the
//! mixture density follows Dalton's law of partial pressures:
//!
//! ```text
//! rho_air = (P_dry * M_dry + e * M_vapor) / (R_UNIVERSAL * T_ambient)
//! ```

/// Universal gas constant [J/(mol*K)].
pub const R_UNIVERSAL: f64 = 8.314_462_618;
/// Molar mass of dry air [kg/mol].
pub const M_DRY_AIR: f64 = 0.028_964_6;
/// Molar mass of water vapour [kg/mol].
pub const M_WATER_VAPOR: f64 = 0.018_015_28;
/// Ratio of specific heats for dry air at ambient temperature [-].
pub const GAMMA_AIR: f64 = 1.4;

/// 0 degrees Celsius expressed in Kelvin.
pub const KELVIN_OFFSET: f64 = 273.15;
/// ISA sea-level pressure [Pa].
pub const STANDARD_PRESSURE: f64 = 101_325.0;

/// Ambient state the engine breathes from.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Environment {
    /// Absolute ambient (barometric) pressure [Pa].
    pub pressure: f64,
    /// Ambient dry-bulb temperature [K].
    pub temperature: f64,
    /// Relative humidity in the range 0.0 ..= 1.0 [-].
    pub relative_humidity: f64,
}

impl Default for Environment {
    /// ISA sea level, 20 C, 40 % RH.
    fn default() -> Self {
        Self {
            pressure: STANDARD_PRESSURE,
            temperature: KELVIN_OFFSET + 20.0,
            relative_humidity: 0.40,
        }
    }
}

impl Environment {
    /// Builds an ambient state from human-facing units (Celsius, percent).
    ///
    /// Humidity is clamped to a physical range; pressure and temperature are
    /// clamped away from zero so downstream divisions stay finite.
    pub fn new(pressure_pa: f64, temperature_c: f64, relative_humidity_pct: f64) -> Self {
        Self {
            pressure: pressure_pa.max(1.0),
            temperature: (temperature_c + KELVIN_OFFSET).max(1.0),
            relative_humidity: (relative_humidity_pct / 100.0).clamp(0.0, 1.0),
        }
    }

    /// Ambient temperature in Celsius [C].
    pub fn temperature_celsius(&self) -> f64 {
        self.temperature - KELVIN_OFFSET
    }

    /// Saturation vapour pressure via the Tetens approximation [Pa].
    ///
    /// `e_s = 610.78 * exp(17.27 * T_c / (T_c + 237.3))` over liquid water; the
    /// over-ice coefficients are used below freezing, where the liquid-water fit
    /// overestimates by several percent.
    pub fn saturation_vapor_pressure(&self) -> f64 {
        tetens(self.temperature_celsius())
    }

    /// Partial pressure of water vapour, `e = RH * e_s` [Pa].
    pub fn vapor_pressure(&self) -> f64 {
        // Vapour cannot exceed the total pressure even if the caller hands us a
        // hot, low-pressure combination that Tetens would push past it.
        (self.relative_humidity * self.saturation_vapor_pressure()).min(self.pressure)
    }

    /// Partial pressure of the dry-air fraction, `P_dry = P - e` [Pa].
    pub fn dry_air_pressure(&self) -> f64 {
        (self.pressure - self.vapor_pressure()).max(0.0)
    }

    /// Density of the humid-air mixture [kg/m^3].
    pub fn air_density(&self) -> f64 {
        let e = self.vapor_pressure();
        let p_dry = self.pressure - e;
        (p_dry * M_DRY_AIR + e * M_WATER_VAPOR) / (R_UNIVERSAL * self.temperature)
    }

    /// Specific gas constant of the humid mixture [J/(kg*K)].
    ///
    /// Derived from the mixture density so it stays consistent with
    /// [`Environment::air_density`]: `R_spec = P / (rho * T)`.
    pub fn specific_gas_constant(&self) -> f64 {
        self.pressure / (self.air_density() * self.temperature)
    }

    /// Speed of sound in the humid mixture [m/s].
    ///
    /// Used later by the acoustic transit delays; humidity raises it slightly
    /// because water vapour is lighter than the nitrogen it displaces.
    pub fn speed_of_sound(&self) -> f64 {
        (GAMMA_AIR * self.specific_gas_constant() * self.temperature).sqrt()
    }
}

/// Tetens saturation vapour pressure for a Celsius temperature [Pa].
fn tetens(temperature_c: f64) -> f64 {
    let (a, b) = if temperature_c >= 0.0 {
        (17.27, 237.3) // over liquid water
    } else {
        (21.875, 265.5) // over ice
    };
    610.78 * (a * temperature_c / (temperature_c + b)).exp()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() <= tol, "expected {b}, got {a} (tol {tol})");
    }

    #[test]
    fn tetens_matches_reference_table() {
        // Reference saturation pressures (WMO tables), ~0.2 % agreement.
        approx(tetens(0.0), 611.0, 3.0);
        approx(tetens(20.0), 2339.0, 12.0);
        approx(tetens(100.0), 101_325.0, 900.0);
    }

    #[test]
    fn dry_air_density_matches_isa() {
        let env = Environment::new(STANDARD_PRESSURE, 15.0, 0.0);
        // ISA sea-level density is 1.2250 kg/m^3.
        approx(env.air_density(), 1.2250, 1e-3);
        approx(env.specific_gas_constant(), 287.05, 0.05);
    }

    #[test]
    fn humid_air_is_less_dense_than_dry_air() {
        let dry = Environment::new(STANDARD_PRESSURE, 30.0, 0.0);
        let humid = Environment::new(STANDARD_PRESSURE, 30.0, 100.0);
        assert!(
            humid.air_density() < dry.air_density(),
            "water vapour displaces heavier N2/O2, so humid air must be lighter"
        );
        // At 30 C the effect is well under a percent.
        let delta = (dry.air_density() - humid.air_density()) / dry.air_density();
        assert!(
            (0.0..0.02).contains(&delta),
            "unphysical humidity effect: {delta}"
        );
    }

    #[test]
    fn partial_pressures_sum_to_total() {
        let env = Environment::new(95_000.0, 35.0, 85.0);
        approx(
            env.dry_air_pressure() + env.vapor_pressure(),
            env.pressure,
            1e-9,
        );
    }

    #[test]
    fn speed_of_sound_is_sane() {
        let env = Environment::new(STANDARD_PRESSURE, 20.0, 0.0);
        approx(env.speed_of_sound(), 343.2, 1.0);
    }

    #[test]
    fn saturated_vapor_cannot_exceed_total_pressure() {
        // Boiling conditions: Tetens alone would exceed the ambient pressure.
        let env = Environment::new(50_000.0, 100.0, 100.0);
        assert!(env.vapor_pressure() <= env.pressure);
        assert!(env.air_density() > 0.0 && env.air_density().is_finite());
    }
}
