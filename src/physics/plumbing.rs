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
    /// Tailpipe radiation cutoff [Hz].
    pub tailpipe_cutoff: f64,
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
            tailpipe_cutoff: 3_200.0,
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
