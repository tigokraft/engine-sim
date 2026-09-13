//! Serialisable engine configuration for standalone TOML files.
//!
//! Every engine in the catalogue can be represented as an independent, human-editable
//! TOML file in `engines/<name>.toml`. This decouples the engine specifications from
//! Rust code and allows sound designers and enthusiasts to tweak displacements,
//! valve events, pipe lengths, silencer configurations, rev limiter strategies,
//! and acoustic aperture positions without recompiling.

use std::f64::consts::PI;
use std::fs;
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::audio::{
    AperturePositions, BlowOffVoicing, CentrifugalVoicing, ImpulsiveSpec, Induction,
    MechanicalSpec, RootsVoicing, SourceRate, TurboModel, TurboVoicing, WastegateVoicing,
};
use crate::bench::EnginePreset;
use crate::physics::control::{LimiterCut, LimiterMode};
use crate::physics::cylinder::{
    default_float_rpm, default_reciprocating_mass, deg, CylinderGeometry,
};
use crate::physics::engine_block::{CylinderIndex, FiringOrder};
use crate::physics::plumbing::{
    Collector, Crossover, ExhaustSystem, IntakeSystem, MufflerGeometry, PipeSection, Silencer,
    ThrottleLayout,
};
use crate::physics::thermodynamics::{
    CylinderModel, DieselCombustion, HeatRelease, ValveEvent, ValveTrain, WiebeProfile, DIESEL_LHV,
};

/// Root engine configuration matching the structure of `engines/*.toml`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EngineConfig {
    pub identity: IdentityConfig,
    pub block: BlockConfig,
    pub cylinder: CylinderConfig,
    pub combustion: CombustionConfig,
    pub firing: FiringConfig,
    pub exhaust: ExhaustConfig,
    pub intake: IntakeConfig,
    pub induction: InductionConfig,
    #[serde(default)]
    pub mechanical: MechanicalConfig,
    pub acoustics: AperturesConfig,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IdentityConfig {
    pub name: String,
    pub note: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BlockConfig {
    pub mass: f64,
    pub bore_spacing: f64,
    pub inertia: f64,
    pub redline: f64,
    pub idle: f64,
    pub load: [f64; 3],
    #[serde(default)]
    pub anti_lag: bool,
    #[serde(default = "default_limiter_mode")]
    pub limiter_mode: LimiterMode,
    #[serde(default = "default_limiter_cut")]
    pub limiter_cut: LimiterCut,
    /// Valve float threshold speed override [rev/min]; absent lets it default
    /// from redline via [`default_float_rpm`].
    #[serde(default)]
    pub float_rpm: Option<f64>,
}

fn default_limiter_mode() -> LimiterMode {
    LimiterMode::RotatingStutter
}

fn default_limiter_cut() -> LimiterCut {
    LimiterCut::Spark
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CylinderConfig {
    pub bore: f64,
    pub stroke: f64,
    pub rod_length: f64,
    pub compression_ratio: f64,
    pub intake_open_deg: f64,
    pub intake_duration_deg: f64,
    pub intake_lift: f64,
    pub intake_diameter: f64,
    #[serde(default = "default_intake_discharge")]
    pub intake_discharge_coeff: f64,
    pub exhaust_open_deg: f64,
    pub exhaust_duration_deg: f64,
    pub exhaust_lift: f64,
    pub exhaust_diameter: f64,
    #[serde(default = "default_exhaust_discharge")]
    pub exhaust_discharge_coeff: f64,
    /// Reciprocating mass override [kg]; absent lets [`CylinderGeometry::new`]
    /// default it from `bore`, which is why no existing engine file has to
    /// name one.
    #[serde(default)]
    pub reciprocating_mass: Option<f64>,
    /// Valve float threshold speed override [rev/min]; absent lets it default
    /// from redline via [`default_float_rpm`].
    #[serde(default)]
    pub float_rpm: Option<f64>,
}

fn default_intake_discharge() -> f64 {
    0.65
}

fn default_exhaust_discharge() -> f64 {
    0.60
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum CombustionConfig {
    Spark {
        spark_angle_deg: f64,
        duration_deg: f64,
        efficiency_parameter: f64,
        form_factor: f64,
        combustion_efficiency: f64,
        #[serde(default = "default_fuel_lhv")]
        fuel_lhv: f64,
        #[serde(default = "default_gasoline_afr")]
        air_fuel_ratio: f64,
    },
    Compression {
        injection_angle_deg: f64,
        injection_duration_deg: f64,
        premixed_duration_deg: f64,
        diffusion_duration_deg: f64,
        premixed_form_factor: f64,
        diffusion_form_factor: f64,
        efficiency_parameter: f64,
        combustion_efficiency: f64,
        #[serde(default = "default_diesel_lhv")]
        fuel_lhv: f64,
        #[serde(default = "default_diesel_afr")]
        air_fuel_ratio: f64,
    },
}

fn default_fuel_lhv() -> f64 {
    44.0e6
}

fn default_gasoline_afr() -> f64 {
    14.7
}

fn default_diesel_lhv() -> f64 {
    DIESEL_LHV
}

fn default_diesel_afr() -> f64 {
    22.0
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FiringConfig {
    pub sequence: Vec<u8>,
    pub banks: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PipeConfig {
    pub length: f64,
    pub diameter: f64,
    pub temperature: f64,
}

impl PipeConfig {
    pub fn from_section(p: &PipeSection) -> Self {
        Self {
            length: p.length,
            diameter: p.diameter(),
            temperature: p.wall_temperature,
        }
    }

    pub fn to_section(&self) -> PipeSection {
        PipeSection::from_diameter(self.length, self.diameter, self.temperature)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CollectorConfig {
    pub inlets: usize,
    pub outlet_diameter: f64,
    pub taper_length: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum CrossoverConfig {
    None,
    XPipe { position: f64 },
    HPipe { position: f64, diameter: f64 },
    Balance180,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum SilencerConfig {
    Straight,
    Helmholtz {
        chamber_volume: f64,
        neck_diameter: f64,
        neck_length: f64,
        q: f64,
        resonant_mix: f64,
    },
    ExpansionChamber {
        length: f64,
        area_ratio: f64,
        stages: usize,
    },
    Absorptive {
        length: f64,
        diameter: f64,
        packing_thickness: f64,
        packing_absorption: f64,
    },
    QuarterWaveStub {
        length: f64,
        diameter: f64,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExhaustConfig {
    pub primaries: Vec<PipeConfig>,
    pub collector: CollectorConfig,
    pub crossover: CrossoverConfig,
    pub silencers: Vec<SilencerConfig>,
    pub tailpipe: PipeConfig,
    #[serde(default)]
    pub tailpipe_flanged: bool,
    /// On-disk key stays `cutout`: every `engines/*.toml` file already has
    /// one, and it means fitment there exactly as it does on the Rust side.
    #[serde(default, rename = "cutout")]
    pub cutout_fitted: bool,
    /// Optional mode override: `"muffled"`, `"straight_pipe"`, or `"open_headers"`.
    #[serde(default)]
    pub mode: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum ThrottleConfig {
    Single { bore: f64 },
    IndividualBodies { bore: f64 },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct IntakeConfig {
    pub runners: Vec<PipeConfig>,
    pub plenum_volume: f64,
    pub throttle: ThrottleConfig,
    pub airbox: Option<PipeConfig>,
    pub snorkel: Option<PipeConfig>,
    #[serde(default)]
    pub trumpet_flanged: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum InductionConfig {
    NaturallyAspirated,
    Turbocharged {
        max_shaft_rpm: f64,
        reference_engine_rpm: f64,
        spool_up: f64,
        spool_down: f64,
        order: f64,
        reference_rpm: f64,
        level: f64,
        #[serde(default)]
        blow_off: Option<BlowOffConfig>,
        #[serde(default)]
        wastegate: Option<WastegateConfig>,
    },
    RootsSupercharged {
        belt_ratio: f64,
        lobes: usize,
        level: f64,
    },
    CentrifugalSupercharged {
        gear_ratio: f64,
        order: f64,
        level: f64,
        #[serde(default)]
        blow_off: Option<BlowOffConfig>,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BlowOffConfig {
    pub threshold: f32,
    pub decay_time: f32,
    pub center_hz: f32,
    pub level: f32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WastegateConfig {
    pub flutter_hz: f32,
    pub rattle_hz: f32,
    pub threshold: f32,
    pub level: f32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct MechanicalConfig {
    #[serde(default)]
    pub float_rpm: Option<f64>,
    #[serde(default)]
    pub intake_valve: Option<ImpulsiveConfig>,
    #[serde(default)]
    pub exhaust_valve: Option<ImpulsiveConfig>,
    #[serde(default)]
    pub piston_slap: Option<ImpulsiveConfig>,
    #[serde(default)]
    pub injector: Option<ImpulsiveConfig>,
    #[serde(default)]
    pub timing_chain: Option<ImpulsiveConfig>,
    #[serde(default)]
    pub gear_whine: Option<ImpulsiveConfig>,
    #[serde(default)]
    pub accessory: Option<ImpulsiveConfig>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ImpulsiveConfig {
    pub timing: String, // "per_cylinder" or "crank_order"
    pub order: f32,
    pub level: f32,
}

impl ImpulsiveConfig {
    pub fn from_spec(spec: &ImpulsiveSpec) -> Self {
        let (timing, order) = match spec.rate {
            SourceRate::PerCylinder => ("per_cylinder".to_string(), 0.0),
            SourceRate::Order(o) => ("crank_order".to_string(), o),
        };
        Self {
            timing,
            order,
            level: spec.level,
        }
    }

    pub fn to_spec(&self) -> ImpulsiveSpec {
        let rate = match self.timing.as_str() {
            "crank_order" => SourceRate::Order(self.order),
            _ => SourceRate::PerCylinder,
        };
        ImpulsiveSpec {
            rate,
            level: self.level,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AperturesConfig {
    pub tailpipes: Vec<[f64; 3]>,
    pub intake: [f64; 3],
    pub block: [f64; 3],
}

impl EngineConfig {
    /// Converts a compiled [`EnginePreset`] into an [`EngineConfig`].
    pub fn from_preset(preset: &EnginePreset) -> Self {
        let identity = IdentityConfig {
            name: preset.name.to_string(),
            note: preset.note.to_string(),
        };

        let block = BlockConfig {
            mass: preset.block_mass,
            bore_spacing: preset.bore_spacing,
            inertia: preset.inertia,
            redline: preset.redline,
            idle: preset.idle,
            load: [preset.load.0, preset.load.1, preset.load.2],
            anti_lag: preset.anti_lag,
            limiter_mode: preset.limiter_mode,
            limiter_cut: preset.limiter_cut,
            float_rpm: None,
        };

        let cyl_geom = &preset.model.geometry;
        let valves = &preset.model.valves;
        let cylinder = CylinderConfig {
            bore: cyl_geom.bore,
            stroke: cyl_geom.stroke,
            rod_length: cyl_geom.rod_length,
            compression_ratio: cyl_geom.compression_ratio,
            intake_open_deg: valves.intake.open_angle.to_degrees(),
            intake_duration_deg: valves.intake.duration.to_degrees(),
            intake_lift: valves.intake.max_lift,
            intake_diameter: valves.intake.diameter,
            intake_discharge_coeff: valves.intake.discharge_coefficient,
            exhaust_open_deg: valves.exhaust.open_angle.to_degrees(),
            exhaust_duration_deg: valves.exhaust.duration.to_degrees(),
            exhaust_lift: valves.exhaust.max_lift,
            exhaust_diameter: valves.exhaust.diameter,
            exhaust_discharge_coeff: valves.exhaust.discharge_coefficient,
            // Only named when it differs meaningfully from what `bore` alone
            // would give, so round-tripping a preset that never set one does
            // not clutter its file with a value that was always implicit. A
            // relative tolerance rather than exact equality, because a bore
            // that reached this geometry through a compile-time constant and
            // one recomputed here at runtime can round `powi(3)` to different
            // last bits of the same number.
            reciprocating_mass: {
                let default_mass = default_reciprocating_mass(cyl_geom.bore);
                let differs = (cyl_geom.reciprocating_mass - default_mass).abs()
                    > 1e-9 * default_mass.max(1e-12);
                differs.then_some(cyl_geom.reciprocating_mass)
            },
            float_rpm: {
                let default_float = default_float_rpm(preset.redline);
                let differs =
                    (preset.float_rpm - default_float).abs() > 1e-9 * default_float.max(1e-12);
                differs.then_some(preset.float_rpm)
            },
        };

        let combustion = match &preset.model.combustion {
            HeatRelease::Spark(w) => CombustionConfig::Spark {
                spark_angle_deg: w.spark_angle.to_degrees(),
                duration_deg: w.duration.to_degrees(),
                efficiency_parameter: w.efficiency_parameter,
                form_factor: w.form_factor,
                combustion_efficiency: w.combustion_efficiency,
                fuel_lhv: preset.model.fuel_lhv,
                air_fuel_ratio: preset.model.air_fuel_ratio,
            },
            HeatRelease::Compression(d) => CombustionConfig::Compression {
                injection_angle_deg: d.injection_angle.to_degrees(),
                injection_duration_deg: d.injection_duration.to_degrees(),
                premixed_duration_deg: d.premixed_duration.to_degrees(),
                diffusion_duration_deg: d.diffusion_duration.to_degrees(),
                premixed_form_factor: d.premixed_form_factor,
                diffusion_form_factor: d.diffusion_form_factor,
                efficiency_parameter: d.efficiency_parameter,
                combustion_efficiency: d.combustion_efficiency,
                fuel_lhv: preset.model.fuel_lhv,
                air_fuel_ratio: preset.model.air_fuel_ratio,
            },
        };

        let firing = FiringConfig {
            sequence: preset.firing.sequence.clone(),
            banks: preset
                .firing
                .sequence
                .iter()
                .map(|&num| {
                    preset
                        .firing
                        .cylinders
                        .iter()
                        .find(|c| c.number == num)
                        .map_or(0, |c| c.bank)
                })
                .collect(),
        };

        let primaries = preset
            .exhaust
            .primaries
            .iter()
            .map(PipeConfig::from_section)
            .collect();
        let collector = CollectorConfig {
            inlets: preset.exhaust.collector.inlets,
            outlet_diameter: preset.exhaust.collector.outlet_diameter(),
            taper_length: preset.exhaust.collector.taper_length,
        };
        let crossover = match &preset.exhaust.crossover {
            Crossover::None => CrossoverConfig::None,
            Crossover::XPipe { position } => CrossoverConfig::XPipe {
                position: *position,
            },
            Crossover::HPipe { position, area } => CrossoverConfig::HPipe {
                position: *position,
                diameter: (4.0 * area / PI).sqrt(),
            },
            Crossover::Balance180 => CrossoverConfig::Balance180,
        };
        let silencers = preset
            .exhaust
            .silencers
            .iter()
            .map(|s| match s {
                Silencer::Straight => SilencerConfig::Straight,
                Silencer::Helmholtz(geom) => SilencerConfig::Helmholtz {
                    chamber_volume: geom.chamber_volume,
                    neck_diameter: (4.0 * geom.neck_area / PI).sqrt(),
                    neck_length: geom.neck_length,
                    q: geom.q,
                    resonant_mix: geom.resonant_mix,
                },
                Silencer::ExpansionChamber {
                    length,
                    area_ratio,
                    stages,
                } => SilencerConfig::ExpansionChamber {
                    length: *length,
                    area_ratio: *area_ratio,
                    stages: *stages,
                },
                Silencer::Absorptive {
                    length,
                    area,
                    packing_thickness,
                    packing_absorption,
                } => SilencerConfig::Absorptive {
                    length: *length,
                    diameter: (4.0 * area / PI).sqrt(),
                    packing_thickness: *packing_thickness,
                    packing_absorption: *packing_absorption,
                },
                Silencer::QuarterWaveStub { length, area } => SilencerConfig::QuarterWaveStub {
                    length: *length,
                    diameter: (4.0 * area / PI).sqrt(),
                },
            })
            .collect();
        let tailpipe = PipeConfig::from_section(&preset.exhaust.tailpipe);
        let mode = if preset.exhaust.is_open_headers() {
            Some("open_headers".to_string())
        } else if preset.exhaust.is_straight_pipe() {
            Some("straight_pipe".to_string())
        } else {
            None
        };

        let exhaust = ExhaustConfig {
            primaries,
            collector,
            crossover,
            silencers,
            tailpipe,
            tailpipe_flanged: preset.exhaust.tailpipe_flanged,
            cutout_fitted: preset.exhaust.cutout_fitted,
            mode,
        };

        let runners = preset
            .intake
            .runners
            .iter()
            .map(PipeConfig::from_section)
            .collect();
        let throttle = match preset.intake.throttle {
            ThrottleLayout::Single { bore } => ThrottleConfig::Single { bore },
            ThrottleLayout::IndividualBodies { bore } => ThrottleConfig::IndividualBodies { bore },
        };
        let airbox = preset.intake.airbox.as_ref().map(PipeConfig::from_section);
        let snorkel = preset.intake.snorkel.as_ref().map(PipeConfig::from_section);

        let intake = IntakeConfig {
            runners,
            plenum_volume: preset.intake.plenum_volume,
            throttle,
            airbox,
            snorkel,
            trumpet_flanged: preset.intake.trumpet_flanged,
        };

        let induction = match preset.induction {
            Induction::NaturallyAspirated => InductionConfig::NaturallyAspirated,
            Induction::Turbocharged {
                shaft,
                voice,
                blow_off,
                wastegate,
            } => InductionConfig::Turbocharged {
                max_shaft_rpm: shaft.max_shaft_rpm,
                reference_engine_rpm: shaft.reference_engine_rpm,
                spool_up: shaft.spool_up,
                spool_down: shaft.spool_down,
                order: voice.order,
                reference_rpm: voice.reference_rpm,
                level: voice.level,
                blow_off: blow_off.map(|b| BlowOffConfig {
                    threshold: b.threshold,
                    decay_time: b.decay_time,
                    center_hz: b.center_hz,
                    level: b.level,
                }),
                wastegate: wastegate.map(|w| WastegateConfig {
                    flutter_hz: w.flutter_hz,
                    rattle_hz: w.rattle_hz,
                    threshold: w.threshold,
                    level: w.level,
                }),
            },
            Induction::RootsSupercharged { voice } => InductionConfig::RootsSupercharged {
                belt_ratio: voice.belt_ratio,
                lobes: voice.lobes,
                level: voice.level,
            },
            Induction::CentrifugalSupercharged { voice, blow_off } => {
                InductionConfig::CentrifugalSupercharged {
                    gear_ratio: voice.gear_ratio,
                    order: voice.order,
                    level: voice.level,
                    blow_off: blow_off.map(|b| BlowOffConfig {
                        threshold: b.threshold,
                        decay_time: b.decay_time,
                        center_hz: b.center_hz,
                        level: b.level,
                    }),
                }
            }
        };

        let mechanical = MechanicalConfig {
            float_rpm: None,
            intake_valve: preset
                .mechanical
                .intake_valve
                .as_ref()
                .map(ImpulsiveConfig::from_spec),
            exhaust_valve: preset
                .mechanical
                .exhaust_valve
                .as_ref()
                .map(ImpulsiveConfig::from_spec),
            piston_slap: preset
                .mechanical
                .piston_slap
                .as_ref()
                .map(ImpulsiveConfig::from_spec),
            injector: preset
                .mechanical
                .injector
                .as_ref()
                .map(ImpulsiveConfig::from_spec),
            timing_chain: preset
                .mechanical
                .timing_chain
                .as_ref()
                .map(ImpulsiveConfig::from_spec),
            gear_whine: preset
                .mechanical
                .gear_whine
                .as_ref()
                .map(ImpulsiveConfig::from_spec),
            accessory: preset
                .mechanical
                .accessory
                .as_ref()
                .map(ImpulsiveConfig::from_spec),
        };

        let acoustics = AperturesConfig {
            tailpipes: preset.aperture_positions.tailpipes.clone(),
            intake: preset.aperture_positions.intake,
            block: preset.aperture_positions.block,
        };

        Self {
            identity,
            block,
            cylinder,
            combustion,
            firing,
            exhaust,
            intake,
            induction,
            mechanical,
            acoustics,
        }
    }

    /// Converts this [`EngineConfig`] into a runnable [`EnginePreset`].
    pub fn to_preset(&self) -> EnginePreset {
        let name: &'static str = Box::leak(self.identity.name.clone().into_boxed_str());
        let note: &'static str = Box::leak(self.identity.note.clone().into_boxed_str());

        let mut geometry = CylinderGeometry::new(
            self.cylinder.bore,
            self.cylinder.stroke,
            self.cylinder.rod_length,
            self.cylinder.compression_ratio,
        );
        if let Some(mass) = self.cylinder.reciprocating_mass {
            geometry = geometry.with_reciprocating_mass(mass);
        }

        let valves = ValveTrain {
            intake: ValveEvent::new(
                deg(self.cylinder.intake_open_deg),
                deg(self.cylinder.intake_duration_deg),
                self.cylinder.intake_lift,
                self.cylinder.intake_diameter,
                self.cylinder.intake_discharge_coeff,
            ),
            exhaust: ValveEvent::new(
                deg(self.cylinder.exhaust_open_deg),
                deg(self.cylinder.exhaust_duration_deg),
                self.cylinder.exhaust_lift,
                self.cylinder.exhaust_diameter,
                self.cylinder.exhaust_discharge_coeff,
            ),
        };

        let (combustion, fuel_lhv, air_fuel_ratio) = match &self.combustion {
            CombustionConfig::Spark {
                spark_angle_deg,
                duration_deg,
                efficiency_parameter,
                form_factor,
                combustion_efficiency,
                fuel_lhv,
                air_fuel_ratio,
            } => (
                HeatRelease::Spark(WiebeProfile::new(
                    deg(*spark_angle_deg),
                    deg(*duration_deg),
                    *efficiency_parameter,
                    *form_factor,
                    *combustion_efficiency,
                )),
                *fuel_lhv,
                *air_fuel_ratio,
            ),
            CombustionConfig::Compression {
                injection_angle_deg,
                injection_duration_deg,
                premixed_duration_deg,
                diffusion_duration_deg,
                premixed_form_factor,
                diffusion_form_factor,
                efficiency_parameter,
                combustion_efficiency,
                fuel_lhv,
                air_fuel_ratio,
            } => (
                HeatRelease::Compression(DieselCombustion {
                    injection_angle: deg(*injection_angle_deg),
                    injection_duration: deg(*injection_duration_deg),
                    premixed_duration: deg(*premixed_duration_deg),
                    diffusion_duration: deg(*diffusion_duration_deg),
                    premixed_form_factor: *premixed_form_factor,
                    diffusion_form_factor: *diffusion_form_factor,
                    efficiency_parameter: *efficiency_parameter,
                    combustion_efficiency: *combustion_efficiency,
                    delay: crate::physics::thermodynamics::IgnitionDelay::default(),
                }),
                *fuel_lhv,
                *air_fuel_ratio,
            ),
        };

        let model = CylinderModel {
            geometry,
            valves,
            combustion,
            fuel_lhv,
            air_fuel_ratio,
            ..CylinderModel::default()
        };

        // Firing order reconstruction
        let n = self.firing.sequence.len().max(1);
        let interval = crate::physics::cylinder::CYCLE_ANGLE / n as f64;
        let banks_map = &self.firing.banks;
        let mut cylinders: Vec<CylinderIndex> = self
            .firing
            .sequence
            .iter()
            .enumerate()
            .map(|(slot, &number)| {
                let bank = if slot < banks_map.len() {
                    banks_map[slot]
                } else {
                    0
                };
                CylinderIndex {
                    number,
                    bank,
                    firing_offset: interval * slot as f64,
                }
            })
            .collect();
        cylinders.sort_by_key(|c| c.number);
        let firing = FiringOrder {
            cylinders,
            interval,
            sequence: self.firing.sequence.clone(),
        };

        let primaries: Vec<PipeSection> = self
            .exhaust
            .primaries
            .iter()
            .map(PipeConfig::to_section)
            .collect();
        let collector = Collector::from_diameter(
            self.exhaust.collector.inlets,
            self.exhaust.collector.outlet_diameter,
            self.exhaust.collector.taper_length,
        );
        let crossover = match &self.exhaust.crossover {
            CrossoverConfig::None => Crossover::None,
            CrossoverConfig::XPipe { position } => Crossover::XPipe {
                position: *position,
            },
            CrossoverConfig::HPipe { position, diameter } => Crossover::HPipe {
                position: *position,
                area: PI * (diameter * 0.5).powi(2),
            },
            CrossoverConfig::Balance180 => Crossover::Balance180,
        };
        let silencers: Vec<Silencer> = self
            .exhaust
            .silencers
            .iter()
            .map(|s| match s {
                SilencerConfig::Straight => Silencer::Straight,
                SilencerConfig::Helmholtz {
                    chamber_volume,
                    neck_diameter,
                    neck_length,
                    q,
                    resonant_mix,
                } => Silencer::Helmholtz(MufflerGeometry {
                    chamber_volume: *chamber_volume,
                    neck_area: PI * (neck_diameter * 0.5).powi(2),
                    neck_length: *neck_length,
                    q: *q,
                    resonant_mix: *resonant_mix,
                }),
                SilencerConfig::ExpansionChamber {
                    length,
                    area_ratio,
                    stages,
                } => Silencer::ExpansionChamber {
                    length: *length,
                    area_ratio: *area_ratio,
                    stages: *stages,
                },
                SilencerConfig::Absorptive {
                    length,
                    diameter,
                    packing_thickness,
                    packing_absorption,
                } => Silencer::Absorptive {
                    length: *length,
                    area: PI * (diameter * 0.5).powi(2),
                    packing_thickness: *packing_thickness,
                    packing_absorption: *packing_absorption,
                },
                SilencerConfig::QuarterWaveStub { length, diameter } => Silencer::QuarterWaveStub {
                    length: *length,
                    area: PI * (diameter * 0.5).powi(2),
                },
            })
            .collect();

        let tailpipe = PipeSection::from_diameter(
            self.exhaust.tailpipe.length,
            self.exhaust.tailpipe.diameter,
            self.exhaust.tailpipe.temperature,
        );

        let mut exhaust = ExhaustSystem {
            primaries,
            collector,
            secondary: Vec::new(),
            crossover,
            silencers,
            tailpipe,
            tailpipe_flanged: self.exhaust.tailpipe_flanged,
            cutout_fitted: self.exhaust.cutout_fitted,
        };

        if let Some(mode) = &self.exhaust.mode {
            match mode.as_str() {
                "open_headers" => exhaust = exhaust.into_open_headers(),
                "straight_pipe" => exhaust = exhaust.into_straight_pipe(),
                _ => {}
            }
        }

        let runners: Vec<PipeSection> = self
            .intake
            .runners
            .iter()
            .map(PipeConfig::to_section)
            .collect();
        let throttle = match self.intake.throttle {
            ThrottleConfig::Single { bore } => ThrottleLayout::Single { bore },
            ThrottleConfig::IndividualBodies { bore } => ThrottleLayout::IndividualBodies { bore },
        };
        let airbox = self.intake.airbox.as_ref().map(PipeConfig::to_section);
        let snorkel = self.intake.snorkel.as_ref().map(PipeConfig::to_section);

        let intake = IntakeSystem {
            runners,
            plenum_volume: self.intake.plenum_volume,
            throttle,
            airbox,
            snorkel,
            trumpet_flanged: self.intake.trumpet_flanged,
        };

        let induction = match &self.induction {
            InductionConfig::NaturallyAspirated => Induction::NaturallyAspirated,
            InductionConfig::Turbocharged {
                max_shaft_rpm,
                reference_engine_rpm,
                spool_up,
                spool_down,
                order,
                reference_rpm,
                level,
                blow_off,
                wastegate,
            } => Induction::Turbocharged {
                shaft: TurboModel {
                    max_shaft_rpm: *max_shaft_rpm,
                    reference_engine_rpm: *reference_engine_rpm,
                    spool_up: *spool_up,
                    spool_down: *spool_down,
                    shaft_rpm: 0.0,
                },
                voice: TurboVoicing {
                    order: *order,
                    reference_rpm: *reference_rpm,
                    level: *level,
                },
                blow_off: blow_off.as_ref().map(|b| BlowOffVoicing {
                    threshold: b.threshold,
                    decay_time: b.decay_time,
                    center_hz: b.center_hz,
                    level: b.level,
                }),
                wastegate: wastegate.as_ref().map(|w| WastegateVoicing {
                    flutter_hz: w.flutter_hz,
                    rattle_hz: w.rattle_hz,
                    threshold: w.threshold,
                    level: w.level,
                }),
            },
            InductionConfig::RootsSupercharged {
                belt_ratio,
                lobes,
                level,
            } => Induction::RootsSupercharged {
                voice: RootsVoicing {
                    belt_ratio: *belt_ratio,
                    lobes: *lobes,
                    level: *level,
                },
            },
            InductionConfig::CentrifugalSupercharged {
                gear_ratio,
                order,
                level,
                blow_off,
            } => Induction::CentrifugalSupercharged {
                voice: CentrifugalVoicing {
                    gear_ratio: *gear_ratio,
                    order: *order,
                    level: *level,
                },
                blow_off: blow_off.as_ref().map(|b| BlowOffVoicing {
                    threshold: b.threshold,
                    decay_time: b.decay_time,
                    center_hz: b.center_hz,
                    level: b.level,
                }),
            },
        };

        let mechanical = MechanicalSpec {
            intake_valve: self
                .mechanical
                .intake_valve
                .as_ref()
                .map(ImpulsiveConfig::to_spec),
            exhaust_valve: self
                .mechanical
                .exhaust_valve
                .as_ref()
                .map(ImpulsiveConfig::to_spec),
            piston_slap: self
                .mechanical
                .piston_slap
                .as_ref()
                .map(ImpulsiveConfig::to_spec),
            injector: self
                .mechanical
                .injector
                .as_ref()
                .map(ImpulsiveConfig::to_spec),
            timing_chain: self
                .mechanical
                .timing_chain
                .as_ref()
                .map(ImpulsiveConfig::to_spec),
            gear_whine: self
                .mechanical
                .gear_whine
                .as_ref()
                .map(ImpulsiveConfig::to_spec),
            accessory: self
                .mechanical
                .accessory
                .as_ref()
                .map(ImpulsiveConfig::to_spec),
            float_rpm: None,
        };

        let aperture_positions = AperturePositions {
            tailpipes: self.acoustics.tailpipes.clone(),
            intake: self.acoustics.intake,
            block: self.acoustics.block,
        };

        EnginePreset {
            name,
            note,
            model,
            firing,
            induction,
            mechanical,
            exhaust,
            intake,
            block_mass: self.block.mass,
            bore_spacing: self.block.bore_spacing,
            redline: self.block.redline,
            float_rpm: self
                .cylinder
                .float_rpm
                .or(self.block.float_rpm)
                .or(self.mechanical.float_rpm)
                .unwrap_or_else(|| default_float_rpm(self.block.redline)),
            idle: self.block.idle,
            inertia: self.block.inertia,
            load: (self.block.load[0], self.block.load[1], self.block.load[2]),
            aperture_positions,
            anti_lag: self.block.anti_lag,
            limiter_mode: self.block.limiter_mode,
            limiter_cut: self.block.limiter_cut,
        }
    }
}

impl EnginePreset {
    /// Deserializes an [`EnginePreset`] from a TOML string.
    pub fn from_toml(s: &str) -> Result<Self, toml::de::Error> {
        let config: EngineConfig = toml::from_str(s)?;
        Ok(config.to_preset())
    }

    /// Serializes this [`EnginePreset`] into a formatted TOML string.
    pub fn to_toml(&self) -> Result<String, toml::ser::Error> {
        let config = EngineConfig::from_preset(self);
        toml::to_string_pretty(&config)
    }

    /// Reads and parses an engine preset from a TOML file.
    pub fn from_file(path: impl AsRef<Path>) -> Result<Self, Box<dyn std::error::Error>> {
        let path_ref = path.as_ref();
        let content = match fs::read_to_string(path_ref) {
            Ok(c) => c,
            Err(e) => {
                if path_ref.is_relative() {
                    if let Ok(manifest) = std::env::var("CARGO_MANIFEST_DIR") {
                        let full = Path::new(&manifest).join(path_ref);
                        fs::read_to_string(full)?
                    } else {
                        return Err(Box::new(e));
                    }
                } else {
                    return Err(Box::new(e));
                }
            }
        };
        Ok(Self::from_toml(&content)?)
    }

    /// Saves this engine preset to a TOML file.
    pub fn to_file(&self, path: impl AsRef<Path>) -> Result<(), Box<dyn std::error::Error>> {
        let s = self.to_toml()?;
        fs::write(path, s)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn export_and_validate_all_catalogue_toml_files() {
        let presets = [
            ("engines/inline_4.toml", EnginePreset::inline_four()),
            (
                "engines/cross_plane_v8.toml",
                EnginePreset::cross_plane_v8(),
            ),
            ("engines/flat_plane_v8.toml", EnginePreset::flat_plane_v8()),
            ("engines/v10.toml", EnginePreset::v10()),
            ("engines/v12.toml", EnginePreset::v12()),
            (
                "engines/2_rotor_wankel.toml",
                EnginePreset::two_rotor_wankel(),
            ),
            (
                "engines/turbo_inline_4.toml",
                EnginePreset::turbo_inline_four(),
            ),
            ("engines/twin_turbo_v8.toml", EnginePreset::twin_turbo_v8()),
            (
                "engines/turbo_inline_6.toml",
                EnginePreset::turbo_inline_six(),
            ),
            (
                "engines/turbodiesel_i4.toml",
                EnginePreset::turbo_diesel_four(),
            ),
            ("engines/big_single.toml", EnginePreset::big_single()),
            ("engines/gt3_cup_992.toml", EnginePreset::gt3_cup_992()),
            ("engines/gt3_cup_997.toml", EnginePreset::gt3_cup_997()),
            ("engines/amg_gt3.toml", EnginePreset::amg_gt3()),
            (
                "engines/ferrari_458_gt3.toml",
                EnginePreset::ferrari_458_gt3(),
            ),
            ("engines/r8_lms_gt3.toml", EnginePreset::r8_lms_gt3()),
        ];

        for (path, preset) in &presets {
            preset
                .to_file(path)
                .expect("failed to write preset to file");
            let loaded = EnginePreset::from_file(path).expect("failed to read preset from file");
            assert_eq!(loaded.name, preset.name);
            assert_eq!(loaded.firing.len(), preset.firing.len());
            assert_eq!(loaded.redline, preset.redline);
        }
    }

    #[test]
    fn engine_config_roundtrip_all_catalogue_presets() {
        for preset in EnginePreset::catalogue() {
            let toml_str = preset.to_toml().expect("failed to serialize preset");
            let restored =
                EnginePreset::from_toml(&toml_str).expect("failed to deserialize preset");

            assert_eq!(restored.name, preset.name);
            assert_eq!(restored.firing.len(), preset.firing.len());
            assert_eq!(restored.redline, preset.redline);
            assert!((restored.float_rpm - preset.float_rpm).abs() < 1e-6);
            assert_eq!(restored.limiter_mode, preset.limiter_mode);
            assert_eq!(restored.limiter_cut, preset.limiter_cut);
            assert!(
                (restored.model.geometry.displacement() - preset.model.geometry.displacement())
                    .abs()
                    < 1e-9
            );
        }
    }

    #[test]
    fn engine_config_float_rpm_override_roundtrips() {
        let mut preset = EnginePreset::inline_four();
        preset.float_rpm = 8_200.0;
        let toml_str = preset.to_toml().expect("failed to serialize preset");
        assert!(toml_str.contains("float_rpm = 8200.0"));
        let restored = EnginePreset::from_toml(&toml_str).expect("failed to deserialize preset");
        assert_eq!(restored.float_rpm, 8_200.0);
    }

    #[test]
    fn engine_config_open_headers_mode_override() {
        let preset = EnginePreset::inline_four();
        let mut config = EngineConfig::from_preset(&preset);
        config.exhaust.mode = Some("open_headers".to_string());

        let built = config.to_preset();
        assert!(built.exhaust.is_open_headers());
        assert!(built.exhaust.silencers.is_empty());
        assert!(built.exhaust.cutout_fitted);
    }
}
