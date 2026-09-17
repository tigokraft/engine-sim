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
use crate::physics::control::{
    IdleGovernor, LimiterCut, LimiterMode, IDLE_ACTUATOR_LAG, IDLE_GOVERNOR_INTEGRAL,
    IDLE_GOVERNOR_PROPORTIONAL,
};
use crate::physics::cylinder::{
    default_float_rpm, default_reciprocating_mass, deg, CylinderGeometry,
};
use crate::physics::engine_block::{CylinderIndex, FiringOrder};
use crate::physics::plumbing::{
    Collector, Crossover, ExhaustSystem, IntakeSystem, MufflerGeometry, PipeSection, Silencer,
    ThrottleLayout, TurbineGeometry,
};
use crate::physics::thermodynamics::{
    CylinderModel, DieselCombustion, HeatRelease, ValveEvent, ValveTrain, WiebeProfile, DIESEL_LHV,
    FASTEST_RAMP,
};
use crate::physics::vehicle::{Clutch, Gearbox, RoadLoad};

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
    #[serde(default)]
    pub driveline: DrivelineConfig,
}

/// Default gearbox, road load and clutch this engine is driven against.
///
/// Not per-engine tuned — see [`Gearbox::generic_six_speed`] and
/// [`RoadLoad::generic_road_car`]. `#[serde(default)]` on its home field in
/// [`EngineConfig`] means a hand-edited TOML from before Stage M1 still parses.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DrivelineConfig {
    pub gearbox: Gearbox,
    pub road_load: RoadLoad,
    pub clutch: Clutch,
}

impl Default for DrivelineConfig {
    fn default() -> Self {
        Self {
            gearbox: Gearbox::generic_six_speed(),
            road_load: RoadLoad::generic_road_car(),
            clutch: Clutch::generic_road_car(),
        }
    }
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
    /// Idle governor gains, lag and travel limit; absent on every field keeps
    /// the stock [`IdleGovernor::default`] this engine would otherwise fall
    /// back to.
    #[serde(default)]
    pub governor: IdleGovernorConfig,
}

fn default_limiter_mode() -> LimiterMode {
    LimiterMode::RotatingStutter
}

fn default_limiter_cut() -> LimiterCut {
    LimiterCut::Spark
}

/// Serialisable [`IdleGovernor`] tuning: PI gains, actuator lag and travel
/// limit, without the runtime state a live governor also carries.
///
/// Every field defaults to the stock gains an engine file written before
/// per-preset governors already assumed, so none of them has to name one.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct IdleGovernorConfig {
    #[serde(default = "default_governor_proportional")]
    pub proportional: f64,
    #[serde(default = "default_governor_integral")]
    pub integral: f64,
    #[serde(default = "default_governor_actuator_lag")]
    pub actuator_lag: f64,
    #[serde(default = "default_governor_authority")]
    pub authority: f64,
}

impl Default for IdleGovernorConfig {
    fn default() -> Self {
        Self {
            proportional: default_governor_proportional(),
            integral: default_governor_integral(),
            actuator_lag: default_governor_actuator_lag(),
            authority: default_governor_authority(),
        }
    }
}

fn default_governor_proportional() -> f64 {
    IDLE_GOVERNOR_PROPORTIONAL
}

fn default_governor_integral() -> f64 {
    IDLE_GOVERNOR_INTEGRAL
}

fn default_governor_actuator_lag() -> f64 {
    IDLE_ACTUATOR_LAG
}

fn default_governor_authority() -> f64 {
    1.0
}

impl IdleGovernorConfig {
    fn to_governor(self) -> IdleGovernor {
        IdleGovernor::tuned(
            self.proportional,
            self.integral,
            self.actuator_lag,
            self.authority,
        )
    }

    fn from_governor(governor: &IdleGovernor) -> Self {
        Self {
            proportional: governor.proportional,
            integral: governor.integral,
            actuator_lag: governor.actuator_lag,
            authority: governor.authority,
        }
    }
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
    /// How aggressively the cam lobes ramp, `0` a stock hydraulic profile and
    /// `1` the fastest flank a valvetrain survives [-].
    ///
    /// Absent is `0`, the raised cosine every existing engine file was written
    /// against, so none of them has to name one. See
    /// [`ValveTrain::with_aggressiveness`].
    #[serde(default)]
    pub cam_aggressiveness: f64,
    /// Raw ramp fraction override, past what [`Self::cam_aggressiveness`] can
    /// express [-].
    ///
    /// `cam_aggressiveness` is normalised against the valvetrain survival
    /// limit `FASTEST_RAMP`, because every poppet cam in the catalogue is
    /// bound by it. A rotary port is not a poppet cam and has no spring to
    /// survive, so [`crate::physics::rotor`]'s named port profiles can ramp
    /// past that limit — and a value `cam_aggressiveness` cannot represent
    /// would otherwise round-trip through a TOML file rounded up to it. This
    /// is the escape hatch: present only when the ramp fraction the preset
    /// was built with is out of `cam_aggressiveness`'s range, absent for
    /// every ordinary poppet-valved engine.
    #[serde(default)]
    pub port_ramp_fraction: Option<f64>,
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
    TwoPlug {
        leading_spark_angle_deg: f64,
        leading_duration_deg: f64,
        leading_efficiency_parameter: f64,
        leading_form_factor: f64,
        leading_combustion_efficiency: f64,
        trailing_delay_deg: f64,
        trailing_duration_deg: f64,
        trailing_efficiency_parameter: f64,
        trailing_form_factor: f64,
        trailing_combustion_efficiency: f64,
        trailing_share: f64,
        #[serde(default = "default_fuel_lhv")]
        fuel_lhv: f64,
        #[serde(default = "default_gasoline_afr")]
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
    /// Turbine housing, absent on every naturally aspirated engine and on
    /// every turbo preset until it is given one.
    #[serde(default)]
    pub turbine: Option<TurbineConfig>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TurbineConfig {
    pub housing_ar: f64,
    pub blade_count: u32,
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
            governor: IdleGovernorConfig::from_governor(&preset.idle_governor),
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
            cam_aggressiveness: valves.aggressiveness(),
            // Only present when the ramp is past what `cam_aggressiveness`
            // can express at all — see the field's own doc comment. Intake
            // and exhaust are built to the same ramp fraction by every named
            // port profile, so the mean recovers it exactly; nothing in the
            // catalogue's poppet-valved engines ever reaches this branch.
            port_ramp_fraction: {
                let mean_ramp = 0.5 * (valves.intake.ramp_fraction + valves.exhaust.ramp_fraction);
                (mean_ramp < FASTEST_RAMP).then_some(mean_ramp)
            },
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
            HeatRelease::TwoPlug(t) => CombustionConfig::TwoPlug {
                leading_spark_angle_deg: t.leading.spark_angle.to_degrees(),
                leading_duration_deg: t.leading.duration.to_degrees(),
                leading_efficiency_parameter: t.leading.efficiency_parameter,
                leading_form_factor: t.leading.form_factor,
                leading_combustion_efficiency: t.leading.combustion_efficiency,
                trailing_delay_deg: t.trailing_delay().to_degrees(),
                trailing_duration_deg: t.trailing.duration.to_degrees(),
                trailing_efficiency_parameter: t.trailing.efficiency_parameter,
                trailing_form_factor: t.trailing.form_factor,
                trailing_combustion_efficiency: t.trailing.combustion_efficiency,
                trailing_share: t.trailing_share,
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

        let turbine = preset.exhaust.turbine.map(|t| TurbineConfig {
            housing_ar: t.housing_ar,
            blade_count: t.blade_count,
        });

        let exhaust = ExhaustConfig {
            primaries,
            collector,
            crossover,
            silencers,
            tailpipe,
            tailpipe_flanged: preset.exhaust.tailpipe_flanged,
            cutout_fitted: preset.exhaust.cutout_fitted,
            mode,
            turbine,
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

        let driveline = DrivelineConfig {
            gearbox: preset.gearbox.clone(),
            road_load: preset.road_load,
            clutch: preset.clutch,
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
            driveline,
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
        }
        .with_aggressiveness(self.cylinder.cam_aggressiveness);
        let valves = match self.cylinder.port_ramp_fraction {
            Some(ramp) => ValveTrain {
                intake: valves.intake.with_port_ramp_fraction(ramp),
                exhaust: valves.exhaust.with_port_ramp_fraction(ramp),
            },
            None => valves,
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
            CombustionConfig::TwoPlug {
                leading_spark_angle_deg,
                leading_duration_deg,
                leading_efficiency_parameter,
                leading_form_factor,
                leading_combustion_efficiency,
                trailing_delay_deg,
                trailing_duration_deg,
                trailing_efficiency_parameter,
                trailing_form_factor,
                trailing_combustion_efficiency,
                trailing_share,
                fuel_lhv,
                air_fuel_ratio,
            } => {
                let leading = WiebeProfile::new(
                    deg(*leading_spark_angle_deg),
                    deg(*leading_duration_deg),
                    *leading_efficiency_parameter,
                    *leading_form_factor,
                    *leading_combustion_efficiency,
                );
                let mut two_plug = crate::physics::thermodynamics::TwoPlugCombustion::new(
                    leading,
                    deg(*trailing_delay_deg),
                    *trailing_share,
                );
                two_plug.trailing.duration = deg(*trailing_duration_deg);
                two_plug.trailing.efficiency_parameter = *trailing_efficiency_parameter;
                two_plug.trailing.form_factor = *trailing_form_factor;
                two_plug.trailing.combustion_efficiency = *trailing_combustion_efficiency;
                (HeatRelease::TwoPlug(two_plug), *fuel_lhv, *air_fuel_ratio)
            }
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

        let turbine = self.exhaust.turbine.as_ref().map(|t| TurbineGeometry {
            housing_ar: t.housing_ar,
            blade_count: t.blade_count,
        });

        let mut exhaust = ExhaustSystem {
            primaries,
            collector,
            secondary: Vec::new(),
            crossover,
            silencers,
            tailpipe,
            tailpipe_flanged: self.exhaust.tailpipe_flanged,
            cutout_fitted: self.exhaust.cutout_fitted,
            turbine,
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
            idle_governor: self.block.governor.to_governor(),
            inertia: self.block.inertia,
            load: (self.block.load[0], self.block.load[1], self.block.load[2]),
            aperture_positions,
            anti_lag: self.block.anti_lag,
            limiter_mode: self.block.limiter_mode,
            limiter_cut: self.block.limiter_cut,
            gearbox: self.driveline.gearbox.clone(),
            road_load: self.driveline.road_load,
            clutch: self.driveline.clutch,
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
mod tests;
