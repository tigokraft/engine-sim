//! Physical geometry for engine induction and exhaust plumbing.
//!
//! # Acoustic geometry vs tuning knobs
//!
//! In this simulator, acoustics are derived from physical geometry rather than
//! chosen by ear. A pipe section has a physical length, cross-sectional area,
//! and wall temperature. Reflections, wave travel delays, cavity resonances, and
//! viscothermal losses follow directly from transmission line and junction
//! scattering theory:
//!
//! - A junction of characteristic admittances $Y_i = A_i / (\rho c)$ scatters an
//!   incident wave $p_i^+$ according to the acoustic pressure junction condition:
//!   $$p_J = \frac{2 \sum Y_i p_i^+}{\sum Y_i}$$
//!   For a single primary entering a collector of outlet area $A_2$, the
//!   pressure reflection coefficient is:
//!   $$r = \frac{A_1 - A_2}{A_1 + A_2}$$
//! - A primary pipe closed at the exhaust valve and open at the collector
//!   resonates at quarter-wave intervals:
//!   $$f_n = (2n - 1) \frac{c}{4 L}$$
//! - Wave round-trip transit time along a pipe of length $L$ is:
//!   $$\tau = \frac{2 L}{c}$$

use std::f64::consts::PI;

/// Dimensions of a Helmholtz muffler chamber [SI].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MufflerGeometry {
    /// Cross-sectional area of the neck [m^2].
    pub neck_area: f64,
    /// Volume of the resonating chamber [m^3].
    pub chamber_volume: f64,
    /// Effective length of the neck [m].
    pub neck_length: f64,
    /// Sharpness of the cavity resonance [-].
    pub q: f64,
    /// How much of the output comes from the resonant path, `0..=1` [-].
    pub resonant_mix: f64,
}

impl Default for MufflerGeometry {
    /// A road-car rear muffler: roughly 8 litres of chamber behind a 50 mm neck,
    /// which lands the cavity near 120 Hz on hot gas.
    fn default() -> Self {
        Self {
            neck_area: PI * 0.025f64.powi(2),
            chamber_volume: 8.0e-3,
            neck_length: 0.10,
            q: 1.6,
            resonant_mix: 0.62,
        }
    }
}

/// A contiguous cylindrical pipe section [SI].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PipeSection {
    /// Centerline length of the pipe section [m].
    pub length: f64,
    /// Internal cross-sectional area [m^2].
    pub area: f64,
    /// Wall temperature for viscothermal boundary condition [K].
    pub wall_temperature: f64,
}

impl PipeSection {
    /// Creates a pipe section from length, cross-sectional area, and wall temperature.
    pub const fn new(length: f64, area: f64, wall_temperature: f64) -> Self {
        Self {
            length,
            area,
            wall_temperature,
        }
    }

    /// Creates a cylindrical pipe section from length and internal bore diameter [m].
    pub fn from_diameter(length: f64, diameter: f64, wall_temperature: f64) -> Self {
        let radius = diameter * 0.5;
        Self {
            length,
            area: PI * radius * radius,
            wall_temperature,
        }
    }

    /// Internal bore diameter [m].
    pub fn diameter(&self) -> f64 {
        2.0 * (self.area / PI).sqrt()
    }

    /// Internal volume of this section [m^3].
    pub fn volume(&self) -> f64 {
        self.length * self.area
    }
}

/// Collector junction where multiple primary exhaust runners converge.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Collector {
    /// Number of inlet runners feeding into the collector.
    pub inlets: usize,
    /// Cross-sectional area of the collector outlet [m^2].
    pub outlet_area: f64,
    /// Length of the converging taper transition [m].
    pub taper_length: f64,
}

impl Collector {
    /// Creates a new collector junction specification.
    pub const fn new(inlets: usize, outlet_area: f64, taper_length: f64) -> Self {
        Self {
            inlets,
            outlet_area,
            taper_length,
        }
    }

    /// Creates a collector junction from inlet count, outlet diameter [m], and taper length [m].
    pub fn from_diameter(inlets: usize, outlet_diameter: f64, taper_length: f64) -> Self {
        let radius = outlet_diameter * 0.5;
        Self {
            inlets,
            outlet_area: PI * radius * radius,
            taper_length,
        }
    }
}

/// Acoustic crossover connecting exhaust banks.
#[derive(Debug, Clone, PartialEq)]
pub enum Crossover {
    /// Independent dual exhaust system with no bank link.
    None,
    /// Perpendicular balance tube between banks.
    HPipe {
        /// Position along the system from the collector [m].
        position: f64,
        /// Cross-sectional area of the balance tube [m^2].
        area: f64,
    },
    /// Merged crossing junction where pulses share volume.
    XPipe {
        /// Position along the system from the collector [m].
        position: f64,
    },
    /// 180-degree bundle crossover pairing cylinders firing 360 deg apart.
    Balance180,
}

/// Silencing element fitted in an exhaust system.
#[derive(Debug, Clone, PartialEq)]
pub enum Silencer {
    /// Open pipe with no expansion or damping device fitted.
    Straight,
    /// Reactive resonator chamber with a tuned neck.
    Helmholtz(MufflerGeometry),
    /// Sudden cross-sectional area expansion chamber.
    ExpansionChamber {
        /// Length of the expansion cavity [m].
        length: f64,
        /// Ratio of chamber cross-sectional area to pipe area $A_{\text{chamber}} / A_{\text{pipe}}$ [-].
        area_ratio: f64,
        /// Number of expansion stages in series.
        stages: usize,
    },
    /// Straight-through packed perforated core silencer with resistive packing.
    Absorptive {
        /// Length of the packed silencer body [m].
        length: f64,
        /// Core flow area [m^2].
        area: f64,
        /// Viscothermal and fibrous acoustic attenuation per metre [dB/m].
        loss_db_per_m: f64,
    },
    /// Side-branch quarter-wave destructive interference stub (drone killer).
    QuarterWaveStub {
        /// Length of the closed stub [m].
        length: f64,
        /// Cross-sectional area of the stub pipe [m^2].
        area: f64,
    },
}

/// Complete geometric description of an engine's exhaust system [SI].
#[derive(Debug, Clone, PartialEq)]
pub struct ExhaustSystem {
    /// Primary exhaust runners from cylinder ports to collector.
    pub primaries: Vec<PipeSection>,
    /// Collector junction uniting primary runners.
    pub collector: Collector,
    /// Secondary pipes downstream of the primary collector.
    pub secondary: Vec<PipeSection>,
    /// Bank crossover junction.
    pub crossover: Crossover,
    /// Silencer elements in order from upstream to downstream.
    pub silencers: Vec<Silencer>,
    /// Final tailpipe section discharging to atmosphere.
    pub tailpipe: PipeSection,
    /// Whether the tailpipe exit features an acoustic flange (baffle reflection boundary).
    pub tailpipe_flanged: bool,
}

/// Layout and sizing of the engine throttle mechanism.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ThrottleLayout {
    /// Single central throttle body feeding a common plenum.
    Single {
        /// Throttle bore diameter [m].
        bore: f64,
    },
    /// Individual throttle bodies (ITBs) at each intake runner mouth.
    IndividualBodies {
        /// Throttle bore diameter per cylinder [m].
        bore: f64,
    },
}

/// Complete geometric description of an engine's intake system [SI].
#[derive(Debug, Clone, PartialEq)]
pub struct IntakeSystem {
    /// Intake runners from plenum/ambient to cylinder intake valves.
    pub runners: Vec<PipeSection>,
    /// Volume of the central intake plenum chamber [m^3].
    pub plenum_volume: f64,
    /// Throttle sizing and arrangement.
    pub throttle: ThrottleLayout,
    /// Upstream airbox volume/filter duct, if fitted.
    pub airbox: Option<PipeSection>,
    /// Air inlet snorkel / feed tract from exterior atmosphere, if fitted.
    pub snorkel: Option<PipeSection>,
    /// Whether runner entries feature flared velocity stack trumpets.
    pub trumpet_flanged: bool,
}

impl ExhaustSystem {
    /// Area of a primary runner [m^2].
    ///
    /// When primaries vary slightly in area, this returns their arithmetic mean.
    pub fn primary_area(&self) -> f64 {
        if self.primaries.is_empty() {
            0.0
        } else {
            self.primaries.iter().map(|p| p.area).sum::<f64>() / self.primaries.len() as f64
        }
    }

    /// Characteristic centerline length of primary runners [m].
    pub fn primary_length(&self) -> f64 {
        if self.primaries.is_empty() {
            0.0
        } else {
            self.primaries.iter().map(|p| p.length).sum::<f64>() / self.primaries.len() as f64
        }
    }

    /// Mean primary length of the cylinders that actually feed one bank [m].
    ///
    /// Bank membership comes from the firing table, not from position in the
    /// primary list. A cross-plane V8's left bank is cylinders 1, 3, 4 and 8;
    /// averaging the first four entries instead describes an engine nobody
    /// built, and on an unequal-length header the two answers differ by enough
    /// to move the tuned peak.
    pub fn primary_length_for_cylinders(&self, cylinders: &[usize]) -> f64 {
        if self.primaries.is_empty() {
            return 0.0;
        }
        let mut total = 0.0;
        let mut count = 0usize;
        for &c in cylinders {
            if let Some(pipe) = self.primaries.get(c) {
                total += pipe.length;
                count += 1;
            }
        }
        if count == 0 {
            self.primary_length()
        } else {
            total / count as f64
        }
    }

    /// Acoustic reflection coefficient at the collector junction:
    /// $$r = \frac{A_{\text{primary}} - A_{\text{outlet}}}{A_{\text{primary}} + A_{\text{outlet}}}$$
    ///
    /// For an expansion into a larger collector outlet ($A_{\text{outlet}} > A_{\text{primary}}$),
    /// this value is negative, corresponding to the expected acoustic phase inversion at an open end.
    pub fn collector_reflection(&self) -> f64 {
        let a1 = self.primary_area();
        let a2 = self.collector.outlet_area;
        if a1 + a2 > 0.0 {
            (a1 - a2) / (a1 + a2)
        } else {
            0.0
        }
    }

    /// Quarter-wave fundamental resonance frequency of the primary runner [Hz]:
    /// $$f_0 = \frac{c}{4 L}$$
    pub fn primary_quarter_wave_hz(&self, speed_of_sound: f64) -> f64 {
        let l = self.primary_length();
        if l > 0.0 {
            speed_of_sound / (4.0 * l)
        } else {
            0.0
        }
    }

    /// Round-trip wave propagation delay along the primary runner [s]:
    /// $$\tau = \frac{2 L}{c}$$
    pub fn primary_round_trip_seconds(&self, speed_of_sound: f64) -> f64 {
        let l = self.primary_length();
        if speed_of_sound > 0.0 {
            2.0 * l / speed_of_sound
        } else {
            0.0
        }
    }

    /// Derives effective Helmholtz/muffler cavity geometry from the silencer list.
    pub fn muffler_geometry(&self) -> MufflerGeometry {
        for s in &self.silencers {
            match s {
                Silencer::Helmholtz(geo) => return *geo,
                Silencer::ExpansionChamber {
                    length,
                    area_ratio,
                    stages,
                } => {
                    let a_pipe = self.primary_area().max(1e-4);
                    let st = (*stages).max(1) as f64;
                    return MufflerGeometry {
                        neck_area: a_pipe,
                        chamber_volume: length * a_pipe * area_ratio * st,
                        neck_length: (length / (2.0 * st)).max(0.04),
                        q: 1.2 + 0.3 * st,
                        resonant_mix: (0.35 + 0.10 * st).clamp(0.0, 0.85),
                    };
                }
                Silencer::Absorptive {
                    length,
                    area,
                    loss_db_per_m: _,
                } => {
                    return MufflerGeometry {
                        neck_area: *area,
                        chamber_volume: length * area * 2.5,
                        neck_length: length * 0.2,
                        q: 0.8,
                        resonant_mix: 0.20,
                    };
                }
                Silencer::QuarterWaveStub { length, area } => {
                    return MufflerGeometry {
                        neck_area: *area,
                        chamber_volume: length * area,
                        neck_length: 0.05,
                        q: 2.2,
                        resonant_mix: 0.35,
                    };
                }
                Silencer::Straight => continue,
            }
        }
        // Straight-through or no silencer: bypass resonance
        MufflerGeometry {
            neck_area: 0.0,
            chamber_volume: 0.0,
            neck_length: 0.0,
            q: 1.0,
            resonant_mix: 0.0,
        }
    }
}

impl IntakeSystem {
    /// Mean centerline length of the intake runners [m].
    pub fn runner_length(&self) -> f64 {
        if self.runners.is_empty() {
            0.0
        } else {
            self.runners.iter().map(|p| p.length).sum::<f64>() / self.runners.len() as f64
        }
    }

    /// Mean cross-sectional area of the intake runners [m^2].
    pub fn runner_area(&self) -> f64 {
        if self.runners.is_empty() {
            0.0
        } else {
            self.runners.iter().map(|p| p.area).sum::<f64>() / self.runners.len() as f64
        }
    }

    /// Quarter-wave ram resonance frequency of an intake runner [Hz]:
    /// $$f_0 = \frac{c}{4 L}$$
    pub fn runner_quarter_wave_hz(&self, speed_of_sound: f64) -> f64 {
        let l = self.runner_length();
        if l > 0.0 {
            speed_of_sound / (4.0 * l)
        } else {
            0.0
        }
    }

    /// Helmholtz resonance frequency of the intake plenum and runners [Hz].
    pub fn helmholtz_resonance_hz(&self, speed_of_sound: f64) -> Option<f64> {
        if self.plenum_volume <= 0.0 || self.runners.is_empty() {
            return None;
        }
        let total_area: f64 = self.runners.iter().map(|p| p.area).sum();
        let l_eff = self.runner_length() + 0.6 * (self.runner_area() / PI).sqrt();
        if l_eff > 0.0 {
            let f =
                (speed_of_sound / (2.0 * PI)) * (total_area / (self.plenum_volume * l_eff)).sqrt();
            Some(f)
        } else {
            None
        }
    }

    /// Default intake system geometry for a given cylinder count.
    pub fn default_for_cylinders(cylinders: usize) -> Self {
        let n = cylinders.max(1);
        Self {
            runners: vec![PipeSection::from_diameter(0.30, 0.042, 310.0); n],
            plenum_volume: 0.5e-3 * n as f64,
            throttle: ThrottleLayout::Single {
                bore: (0.040 + 0.005 * n as f64).min(0.085),
            },
            airbox: Some(PipeSection::from_diameter(0.20, 0.070, 300.0)),
            snorkel: Some(PipeSection::from_diameter(0.35, 0.065, 300.0)),
            trumpet_flanged: true,
        }
    }
}

impl ExhaustSystem {
    /// Default exhaust system geometry for a given cylinder count and bank count.
    pub fn default_for_cylinders(cylinders: usize, banks: usize) -> Self {
        let n = cylinders.max(1);
        let b = banks.max(1);
        let per_bank = (n / b).max(1);
        let primary = PipeSection::from_diameter(0.45, 0.040, 850.0);
        let collector_outlet_d = match per_bank {
            1 | 2 => 0.050,
            3 | 4 => 0.060,
            5 | 6 => 0.065,
            _ => 0.070,
        };
        Self {
            primaries: vec![primary; n],
            collector: Collector::from_diameter(per_bank, collector_outlet_d, 0.15),
            secondary: vec![],
            crossover: if b > 1 {
                Crossover::HPipe {
                    position: 0.80,
                    area: PI * 0.022 * 0.022,
                }
            } else {
                Crossover::None
            },
            silencers: vec![Silencer::Helmholtz(MufflerGeometry::default())],
            tailpipe: PipeSection::from_diameter(1.2, collector_outlet_d, 600.0),
            tailpipe_flanged: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn collector_reflection_matches_area_ratio() {
        let primary = PipeSection::from_diameter(0.40, 0.038, 800.0);
        let collector = Collector::from_diameter(4, 0.054, 0.12);
        let exhaust = ExhaustSystem {
            primaries: vec![primary; 4],
            collector,
            secondary: vec![],
            crossover: Crossover::None,
            silencers: vec![Silencer::Straight],
            tailpipe: PipeSection::from_diameter(1.0, 0.054, 600.0),
            tailpipe_flanged: false,
        };

        let a1 = primary.area;
        let a2 = collector.outlet_area;
        let expected = (a1 - a2) / (a1 + a2);
        let actual = exhaust.collector_reflection();
        assert!((actual - expected).abs() < 1e-12);
        assert!(actual < 0.0, "expansion into collector inverts phase");
    }

    #[test]
    fn primary_quarter_wave_resonates_at_c_over_four_l() {
        let length = 0.42;
        let primary = PipeSection::from_diameter(length, 0.041, 800.0);
        let exhaust = ExhaustSystem {
            primaries: vec![primary; 8],
            collector: Collector::from_diameter(4, 0.065, 0.15),
            secondary: vec![],
            crossover: Crossover::None,
            silencers: vec![],
            tailpipe: PipeSection::from_diameter(1.0, 0.065, 600.0),
            tailpipe_flanged: false,
        };

        let c = 550.0; // speed of sound on hot exhaust gas
        let expected = c / (4.0 * length);
        assert!((exhaust.primary_quarter_wave_hz(c) - expected).abs() < 1e-12);
    }

    #[test]
    fn primary_round_trip_matches_two_l_over_c() {
        let length = 0.55;
        let primary = PipeSection::from_diameter(length, 0.044, 800.0);
        let exhaust = ExhaustSystem {
            primaries: vec![primary; 8],
            collector: Collector::from_diameter(4, 0.060, 0.15),
            secondary: vec![],
            crossover: Crossover::None,
            silencers: vec![],
            tailpipe: PipeSection::from_diameter(1.0, 0.060, 600.0),
            tailpipe_flanged: false,
        };

        let c = 580.0;
        let expected = 2.0 * length / c;
        assert!((exhaust.primary_round_trip_seconds(c) - expected).abs() < 1e-12);
    }
}
