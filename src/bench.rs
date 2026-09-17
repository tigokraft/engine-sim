//! The test bench around the engine.
//!
//! [`EngineBlock`] solves a cycle for a speed it is *given*. Two things have to
//! exist outside it before a person can drive one from a keyboard:
//!
//! - **A catalogue** ([`EnginePreset`]) — the block is a cylinder model plus a
//!   firing order, and picking a V12 means picking both together, along with the
//!   redline and flywheel that belong with it.
//! - **A flywheel** ([`Driveline`]) — something for the solved torque to
//!   accelerate, so the engine finds its own speed instead of being told one.
//!
//! Neither is physics the block is missing; both are the rig it is bolted to.
//!
//! # A caveat on the throttle
//!
//! The 0D block has no throttle plate. The pedal here scales the torque the
//! block solves, which is enough to make the engine drivable and to give the
//! audio path a throttle position, but it is a bench control rather than a
//! manifold-pressure model. Everything downstream of it — pressure at EVO,
//! exhaust temperature, induction flow, the manifold pressure the dashboard
//! displays — is the solver's own output and is not scaled here.

pub mod config;
pub use config::EngineConfig;

use std::f64::consts::PI;

use crate::audio::{
    AperturePositions, BlowOffVoicing, EngineControls, ImpulsiveSpec, Induction, MechanicalSpec,
    SnapshotSource, SynthConfig, WastegateVoicing,
};
use crate::environment::Environment;
use crate::physics::control::{IdleGovernor, IdleHunt, LimiterCut, LimiterMode, Starter};
use crate::physics::cylinder::{default_float_rpm, deg, CylinderGeometry};
use crate::physics::engine_block::{EngineBlock, FiringOrder};
use crate::physics::plumbing::{
    Collector, Crossover, ExhaustSystem, IntakeSystem, PipeSection, Silencer, ThrottleLayout,
    TurbineGeometry,
};
use crate::physics::thermodynamics::{
    CylinderModel, DieselCombustion, HeatRelease, TwoPlugCombustion, ValveEvent, ValveTrain,
    WiebeProfile, DIESEL_LHV,
};
use crate::physics::vehicle::{Clutch, ClutchState, Gearbox, RoadLoad};

/// Speed below which the engine has stalled [rev/min].
pub const STALL_RPM: f64 = 400.0;

/// Fractional lift in the idle target on a stone-cold engine [-].
///
/// See [`Driveline::cold_idle_rise`], which is where it is used and why.
pub const COLD_IDLE_RISE: f64 = 0.60;

/// Pumping mean effective pressure with the throttle shut [Pa].
///
/// A closed throttle plate is a brake: the piston draws its intake stroke
/// against a partial vacuum and gives none of that work back, which is what
/// engine braking *is*. The block does not model it, because it has no throttle
/// plate to shut — so it is applied here, as the torque a given PMEP costs a
/// four-stroke over its displacement:
///
/// ```text
/// tau_pumping = PMEP * V_displaced / (4 * pi)
/// ```
///
/// The `4 pi` is two crank revolutions per cycle. Half a bar is the usual figure
/// for a petrol engine on a fully shut throttle, and it scales with
/// displacement, so a 6.5 litre V12 brakes harder than a 2.0 litre four without
/// anything being tuned per engine.
pub const CLOSED_THROTTLE_PMEP: f64 = 0.55e5;

/// Pedal opening the idle bypass is worth, at full bypass travel [-].
///
/// The bypass is a hole beside the plate, so the free-revving flywheel — which
/// bills its drag against how far the pedal is down — has to be told how much
/// pedal that hole is equivalent to. A third of a percent of bore area against
/// a plate that swings to a hundred is not a third of a percent of pedal,
/// because the plate's first few degrees uncover almost nothing; measured on
/// the air it passes, full bypass travel is worth about three tenths of pedal,
/// which is what the governor's clamp always was.
pub const IDLE_BYPASS_PEDAL_AUTHORITY: f64 = 0.30;

/// Pedal travel under which the engine counts as idling [-].
///
/// A real throttle's first percent or two is plate clearance rather than
/// pedal, and an idle is what the engine does when the driver is asking for
/// nothing. Above this the driver is driving and the governor is a passenger.
pub const IDLE_PEDAL_THRESHOLD: f64 = 0.02;

// ---------------------------------------------------------------------------
// Catalogue
// ---------------------------------------------------------------------------

/// One selectable engine: what it is, and what it takes to drive it.
#[derive(Debug, Clone)]
pub struct EnginePreset {
    /// Display name, e.g. `"Cross-plane V8"`.
    pub name: &'static str,
    /// One line on what makes it sound the way it does.
    pub note: &'static str,
    /// The cylinder the block solves.
    pub model: CylinderModel,
    /// Firing order and bank layout.
    pub firing: FiringOrder,
    /// How it breathes — and therefore whether it whistles.
    ///
    /// The block is solved atmospherically whichever this is; see
    /// [`Induction`] for why forced induction lives in the audio path only.
    pub induction: Induction,
    /// Mechanical noise rig specification: which sources exist, their orders and levels.
    pub mechanical: MechanicalSpec,
    /// Exhaust system geometry: primaries, collector, crossover, and silencers.
    pub exhaust: ExhaustSystem,
    /// Intake system geometry: runners, plenum, throttle, airbox, and snorkel.
    pub intake: IntakeSystem,
    /// Dressed mass of the engine block [kg].
    ///
    /// Sets where the structural rumble sits: a heavy iron block rings low and
    /// an alloy one rings high.
    pub block_mass: f64,
    /// Distance between adjacent bore centres [m].
    ///
    /// The one piece of block geometry a cylinder model cannot imply. It sets
    /// how long the block is, and therefore where its bending and pan modes sit,
    /// and it sets how much metal is left between one bore and the next, which
    /// is what the bore-wall modes ring on. Production practice is 1.15 to 1.30
    /// bores; a siamesed high-output engine goes tighter and reads brighter and
    /// thinner for it. See [`crate::audio::structure`].
    pub bore_spacing: f64,
    /// Speed at which ignition is cut [rev/min].
    pub redline: f64,
    /// Speed at which valve lifters separate from the cam profile [rev/min].
    pub float_rpm: f64,
    /// Speed the idle governor holds [rev/min].
    pub idle: f64,
    /// The idle governor's own gains, lag and authority.
    ///
    /// A governor is calibrated on the engine it ships with: gains that hold
    /// a mild cam's idle steady will overshoot into a lope, or fail to
    /// recover from a WOT lift at all, on a plant with more cam overlap and
    /// less flywheel to damp it. See [`IdleGovernor`].
    pub idle_governor: IdleGovernor,
    /// Rotating inertia of crank, rods and flywheel [kg m^2].
    pub inertia: f64,
    /// Constant, linear and quadratic load coefficients for the dyno brake.
    ///
    /// `load = a + b * omega + c * omega^2` [N m], standing in for accessories,
    /// bearing drag and windage. The quadratic term is what gives the engine a
    /// natural free-revving ceiling; it has to stay under the torque the block
    /// makes at the redline, or the engine plateaus below it and never bounces
    /// off the limiter.
    pub load: (f64, f64, f64),
    /// Physical radiating aperture locations on the vehicle chassis [m].
    pub aperture_positions: AperturePositions,
    /// Whether anti-lag is enabled on lift for this engine.
    pub anti_lag: bool,
    /// Rev limiter intervention mode.
    pub limiter_mode: LimiterMode,
    /// Rev limiter cut mechanism.
    pub limiter_cut: LimiterCut,
    /// Default gearbox: ratios and final drive, starting in neutral.
    ///
    /// Not tuned per engine — see [`Gearbox::generic_six_speed`].
    pub gearbox: Gearbox,
    /// Default road load this engine is driven against.
    pub road_load: RoadLoad,
    /// Default clutch coupling the crank to it.
    pub clutch: Clutch,
}

impl EnginePreset {
    /// Total swept volume [m^3].
    pub fn displacement(&self) -> f64 {
        self.model.geometry.displacement() * self.firing.len() as f64
    }

    /// Builds a block on this preset in a given environment.
    pub fn block(&self, environment: Environment) -> EngineBlock {
        let mut block = EngineBlock::new(self.model, self.firing.clone(), environment);
        block.exhaust = self.exhaust.clone();
        block.intake_system = self.intake.clone();
        block.set_block_mass(self.block_mass);
        block.rebuild_exhaust_banks();
        block.rebuild_intake_throttle();
        block.ecu.redline = self.redline;
        block.ecu.anti_lag = self.anti_lag;
        block.ecu.limiter_mode = self.limiter_mode;
        block.ecu.limiter_cut_type = self.limiter_cut;
        block
    }

    /// The snapshot source for this engine, with its turbo shaft if it has one.
    pub fn snapshot_source(&self, block: &EngineBlock) -> SnapshotSource {
        SnapshotSource::with_induction(block, self.induction)
    }

    /// The synth configuration for this engine, voiced for its induction and mechanical spec.
    pub fn synth_config(&self, block: &EngineBlock, sample_rate: f32) -> SynthConfig {
        let mut mechanical = self.mechanical;
        if mechanical.float_rpm.is_none() {
            mechanical.float_rpm = Some(self.float_rpm as f32);
        }
        // Valvetrain noise is the valve landing, and what a valve lands with
        // is the velocity the closing flank gave it. That is exactly
        // [`ValveEvent::ramp_rate`], so the seating impulse scales with it and
        // nothing else has to be said per engine: a solid roller is loud
        // because of its lobe, not because a preset says it is. A stock cam
        // rates 1 and leaves every level exactly where it was.
        let valves = &block.model.valves;
        if let Some(spec) = mechanical.intake_valve.as_mut() {
            spec.level *= valves.intake.ramp_rate() as f32;
        }
        if let Some(spec) = mechanical.exhaust_valve.as_mut() {
            spec.level *= valves.exhaust.ramp_rate() as f32;
        }
        let mut config = SynthConfig::from_block(block, sample_rate)
            .with_induction(self.induction)
            .with_mechanical(mechanical);
        config.structure = config.structure.with_bore_spacing(self.bore_spacing);
        config.aperture_positions = self.aperture_positions.clone();
        config
    }

    /// Whether a turbo is fitted.
    pub fn is_turbocharged(&self) -> bool {
        matches!(self.induction, Induction::Turbocharged { .. })
    }

    /// Whether any compressor is fitted (turbo or supercharger).
    pub fn is_forced(&self) -> bool {
        self.induction.is_forced()
    }

    /// A short spec line: `"4.0 L * 8 cyl * 11.5:1"`.
    pub fn spec(&self) -> String {
        format!(
            "{:.1} L  {} cyl  {:.1}:1",
            self.displacement() * 1e3,
            self.firing.len(),
            self.model.geometry.compression_ratio,
        )
    }

    /// The engines the dashboard offers, in key order.
    ///
    /// Atmospheric first, then the three that are turbocharged, so the list
    /// reads as a group of engines rather than a random assortment — and so
    /// the two inline-fours and the two V8s sit near enough to each other to
    /// hear what a compressor does to a note.
    pub fn catalogue() -> Vec<EnginePreset> {
        type PresetSource = (&'static str, fn() -> EnginePreset);
        let engine_files: [PresetSource; 17] = [
            ("engines/inline_4.toml", Self::inline_four),
            ("engines/cross_plane_v8.toml", Self::cross_plane_v8),
            (
                "engines/big_cam_chopping_v8.toml",
                Self::big_cam_chopping_v8,
            ),
            ("engines/flat_plane_v8.toml", Self::flat_plane_v8),
            ("engines/v10.toml", Self::v10),
            ("engines/v12.toml", Self::v12),
            ("engines/2_rotor_wankel.toml", Self::two_rotor_wankel),
            ("engines/turbo_inline_4.toml", Self::turbo_inline_four),
            ("engines/twin_turbo_v8.toml", Self::twin_turbo_v8),
            ("engines/turbo_inline_6.toml", Self::turbo_inline_six),
            ("engines/turbodiesel_i4.toml", Self::turbo_diesel_four),
            ("engines/big_single.toml", Self::big_single),
            ("engines/gt3_cup_992.toml", Self::gt3_cup_992),
            ("engines/gt3_cup_997.toml", Self::gt3_cup_997),
            ("engines/amg_gt3.toml", Self::amg_gt3),
            ("engines/ferrari_458_gt3.toml", Self::ferrari_458_gt3),
            ("engines/r8_lms_gt3.toml", Self::r8_lms_gt3),
        ];

        engine_files
            .into_iter()
            .map(|(path, fallback)| Self::from_file(path).unwrap_or_else(|_| fallback()))
            .collect()
    }

    /// Finds the catalogue preset whose name contains `query`, case-insensitively.
    ///
    /// The same substring rule `examples/measure.rs` filters the catalogue
    /// with, exposed once so every caller matching a preset by name agrees on
    /// what a name means, rather than each parsing it its own way — which is
    /// how `examples/diagnose_sound.rs` used to fall silently through to the
    /// cross-plane V8 on an unrecognised `--preset`. Returns `None` rather
    /// than a default on no match, so a typo is a loud failure instead of a
    /// render of the wrong engine.
    pub fn find_by_name(query: &str) -> Option<EnginePreset> {
        let query = query.to_lowercase();
        Self::catalogue()
            .into_iter()
            .find(|p| p.name.to_lowercase().contains(&query))
    }

    /// Curated collection of GT3 Cup and GT3 class race engines.
    ///
    /// Includes Porsche 911 GT3 Cup (Type 992 & Type 997.2 Mezger), Mercedes-AMG GT3 6.2L V8,
    /// Ferrari 458 Italia GT3 4.5L Flat-Plane V8, and Audi R8 LMS GT3 5.2L V10.
    pub fn gt3_cup_collection() -> Vec<EnginePreset> {
        type PresetSource = (&'static str, fn() -> EnginePreset);
        let race_files: [PresetSource; 5] = [
            ("engines/gt3_cup_992.toml", Self::gt3_cup_992),
            ("engines/gt3_cup_997.toml", Self::gt3_cup_997),
            ("engines/amg_gt3.toml", Self::amg_gt3),
            ("engines/ferrari_458_gt3.toml", Self::ferrari_458_gt3),
            ("engines/r8_lms_gt3.toml", Self::r8_lms_gt3),
        ];

        race_files
            .into_iter()
            .map(|(path, fallback)| Self::from_file(path).unwrap_or_else(|_| fallback()))
            .collect()
    }

    /// 2.0 litre inline four.
    pub fn inline_four() -> Self {
        Self {
            name: "Inline-4",
            note: "Even 180 deg firing on one bank: a hard, plain four-cylinder bark.",
            model: CylinderModel {
                geometry: CylinderGeometry::new(0.086, 0.086, 0.1345, 11.5),
                valves: ValveTrain {
                    intake: ValveEvent::new(deg(696.0), deg(248.0), 0.0110, 0.0390, 0.67),
                    exhaust: ValveEvent::new(deg(500.0), deg(244.0), 0.0100, 0.0330, 0.64),
                }
                .with_aggressiveness(0.35),
                combustion: HeatRelease::Spark(WiebeProfile::new(
                    deg(338.0),
                    deg(54.0),
                    5.0,
                    2.0,
                    0.97,
                )),
                ..CylinderModel::default()
            },
            firing: FiringOrder::inline_four(),
            induction: Induction::NaturallyAspirated,
            mechanical: MechanicalSpec::default(),
            exhaust: ExhaustSystem {
                primaries: vec![PipeSection::from_diameter(0.40, 0.038, 850.0); 4],
                collector: Collector::from_diameter(4, 0.054, 0.12),
                secondary: vec![],
                crossover: Crossover::None,
                silencers: vec![Silencer::ExpansionChamber {
                    length: 0.45,
                    area_ratio: 4.5,
                    stages: 2,
                }],
                tailpipe: PipeSection::from_diameter(1.2, 0.054, 600.0),
                tailpipe_flanged: false,
                cutout_fitted: false,
                turbine: None,
            },
            intake: IntakeSystem {
                runners: vec![PipeSection::from_diameter(0.26, 0.044, 310.0); 4],
                plenum_volume: 2.5e-3,
                throttle: ThrottleLayout::Single { bore: 0.064 },
                airbox: Some(PipeSection::from_diameter(0.15, 0.070, 300.0)),
                snorkel: Some(PipeSection::from_diameter(0.30, 0.065, 300.0)),
                trumpet_flanged: true,
            },
            block_mass: 110.0,
            // A production four on 96 mm centres: 5 mm of iron between the bores.
            bore_spacing: 0.096,
            redline: 7_400.0,
            float_rpm: default_float_rpm(7_400.0),
            idle: 850.0,
            inertia: 0.22,
            load: (3.0, 0.010, 7.0e-5),
            gearbox: Gearbox::generic_six_speed(),
            road_load: RoadLoad::generic_road_car(),
            clutch: Clutch::generic_road_car(),
            aperture_positions: AperturePositions::front_engine_single(),
            anti_lag: false,
            limiter_mode: LimiterMode::RotatingStutter,
            limiter_cut: LimiterCut::Spark,
            idle_governor: IdleGovernor::default(),
        }
    }

    /// 5.0 litre cross-plane V8.
    pub fn cross_plane_v8() -> Self {
        Self {
            name: "Cross-plane V8",
            note: "90-180-270-180 gaps on each bank: the offbeat American burble.",
            model: CylinderModel {
                geometry: CylinderGeometry::new(0.094, 0.0895, 0.1500, 11.0),
                valves: ValveTrain {
                    intake: ValveEvent::new(deg(700.0), deg(240.0), 0.0100, 0.0370, 0.65),
                    exhaust: ValveEvent::new(deg(500.0), deg(240.0), 0.0090, 0.0310, 0.60),
                },
                ..CylinderModel::default()
            },
            firing: FiringOrder::cross_plane_v8(),
            induction: Induction::roots(),
            mechanical: MechanicalSpec::default(),
            exhaust: ExhaustSystem {
                primaries: vec![
                    PipeSection::from_diameter(0.52, 0.044, 850.0),
                    PipeSection::from_diameter(0.57, 0.044, 850.0),
                    PipeSection::from_diameter(0.54, 0.044, 850.0),
                    PipeSection::from_diameter(0.57, 0.044, 850.0),
                    PipeSection::from_diameter(0.53, 0.044, 850.0),
                    PipeSection::from_diameter(0.58, 0.044, 850.0),
                    PipeSection::from_diameter(0.55, 0.044, 850.0),
                    PipeSection::from_diameter(0.54, 0.044, 850.0),
                ],
                collector: Collector::from_diameter(4, 0.060, 0.15),
                secondary: vec![],
                crossover: Crossover::HPipe {
                    position: 0.85,
                    area: PI * 0.025 * 0.025,
                },
                silencers: vec![Silencer::Straight],
                tailpipe: PipeSection::from_diameter(1.5, 0.060, 600.0),
                tailpipe_flanged: false,
                cutout_fitted: true,
                turbine: None,
            },
            intake: IntakeSystem {
                runners: vec![PipeSection::from_diameter(0.38, 0.044, 310.0); 8],
                plenum_volume: 4.8e-3,
                throttle: ThrottleLayout::Single { bore: 0.075 },
                airbox: Some(PipeSection::from_diameter(0.20, 0.080, 300.0)),
                snorkel: Some(PipeSection::from_diameter(0.40, 0.075, 300.0)),
                trumpet_flanged: true,
            },
            block_mass: 210.0,
            // The small-block V8's 4.40 inch bore centres, and its 9 mm of deck iron.
            bore_spacing: 0.1118,
            redline: 7_000.0,
            float_rpm: default_float_rpm(7_000.0),
            idle: 750.0,
            inertia: 0.45,
            load: (6.0, 0.020, 1.3e-4),
            gearbox: Gearbox::generic_six_speed(),
            road_load: RoadLoad::generic_road_car(),
            clutch: Clutch::generic_road_car(),
            aperture_positions: AperturePositions::front_engine_dual(),
            anti_lag: false,
            limiter_mode: LimiterMode::RotatingStutter,
            limiter_cut: LimiterCut::Spark,
            idle_governor: IdleGovernor::default(),
        }
    }

    /// 7.0 litre big cam cross-plane V8.
    ///
    /// A high-compression, naturally aspirated 427 cu in American pushrod V8
    /// with a radical solid-roller camshaft: 82 degrees of overlap at the
    /// same lobe separation and near-maximal flank speed drive the idle
    /// governor deep into its limit cycle (docs/measurements/mechanism-m4.md),
    /// vented through open, unsilenced 4-into-1 headers for the raw bark.
    pub fn big_cam_chopping_v8() -> Self {
        Self {
            name: "Big Cam Chopping V8",
            note: "82 deg cam overlap, aggressive solid roller ramps and open headers: violent idle chop and raw bark.",
            model: CylinderModel {
                geometry: CylinderGeometry::new(0.1048, 0.1016, 0.1540, 11.5),
                valves: ValveTrain {
                    intake: ValveEvent::new(deg(675.0), deg(292.0), 0.0140, 0.0500, 0.68),
                    exhaust: ValveEvent::new(deg(465.0), deg(292.0), 0.0135, 0.0390, 0.65),
                }
                .with_aggressiveness(0.75),
                combustion: HeatRelease::Spark(WiebeProfile::new(
                    deg(338.0),
                    deg(52.0),
                    5.0,
                    2.0,
                    0.97,
                )),
                ..CylinderModel::default()
            },
            firing: FiringOrder::cross_plane_v8(),
            induction: Induction::NaturallyAspirated,
            mechanical: MechanicalSpec {
                intake_valve: Some(ImpulsiveSpec::per_cylinder(0.55)),
                exhaust_valve: Some(ImpulsiveSpec::per_cylinder(0.55)),
                piston_slap: Some(ImpulsiveSpec::per_cylinder(0.40)),
                timing_chain: Some(ImpulsiveSpec::order(19.0, 0.22)),
                ..MechanicalSpec::default()
            },
            exhaust: ExhaustSystem {
                primaries: vec![
                    PipeSection::from_diameter(0.54, 0.048, 880.0),
                    PipeSection::from_diameter(0.58, 0.048, 880.0),
                    PipeSection::from_diameter(0.55, 0.048, 880.0),
                    PipeSection::from_diameter(0.57, 0.048, 880.0),
                    PipeSection::from_diameter(0.53, 0.048, 880.0),
                    PipeSection::from_diameter(0.58, 0.048, 880.0),
                    PipeSection::from_diameter(0.56, 0.048, 880.0),
                    PipeSection::from_diameter(0.54, 0.048, 880.0),
                ],
                collector: Collector::from_diameter(4, 0.076, 0.20),
                secondary: vec![],
                // No crossover and no silencer: bare 4-into-1 open headers, a
                // short stub off the collector outlet standing in for the
                // exhaust flange rather than any muffled tailpipe run.
                crossover: Crossover::None,
                silencers: vec![],
                tailpipe: PipeSection::from_diameter(0.06, 0.076, 800.0),
                tailpipe_flanged: false,
                cutout_fitted: true,
                turbine: None,
            },
            intake: IntakeSystem {
                runners: vec![PipeSection::from_diameter(0.28, 0.048, 310.0); 8],
                plenum_volume: 5.2e-3,
                throttle: ThrottleLayout::Single { bore: 0.092 },
                airbox: Some(PipeSection::from_diameter(0.15, 0.110, 300.0)),
                snorkel: None,
                trumpet_flanged: true,
            },
            block_mass: 230.0,
            bore_spacing: 0.1118,
            redline: 7_000.0,
            float_rpm: default_float_rpm(7_000.0),
            idle: 900.0,
            inertia: 0.42,
            load: (5.0, 0.015, 7.0e-5),
            gearbox: Gearbox::generic_six_speed(),
            road_load: RoadLoad::generic_road_car(),
            clutch: Clutch::generic_road_car(),
            aperture_positions: AperturePositions::front_engine_dual(),
            anti_lag: false,
            limiter_mode: LimiterMode::RotatingStutter,
            limiter_cut: LimiterCut::Spark,
            idle_governor: IdleGovernor::default(),
        }
    }

    /// 4.5 litre flat-plane V8.
    pub fn flat_plane_v8() -> Self {
        Self {
            name: "Flat-plane V8",
            note: "Even 180 deg on both banks: two inline-fours sharing a crank.",
            model: CylinderModel {
                geometry: CylinderGeometry::new(0.094, 0.0810, 0.1420, 12.5),
                valves: ValveTrain {
                    intake: ValveEvent::new(deg(702.0), deg(258.0), 0.0120, 0.0400, 0.68),
                    exhaust: ValveEvent::new(deg(492.0), deg(252.0), 0.0110, 0.0340, 0.65),
                }
                .with_aggressiveness(0.40),
                // A shorter burn: flat-plane vees are built to rev, and a race
                // chamber lights faster than a road one.
                combustion: HeatRelease::Spark(WiebeProfile::new(
                    deg(342.0),
                    deg(52.0),
                    5.0,
                    2.1,
                    0.97,
                )),
                ..CylinderModel::default()
            },
            firing: FiringOrder::flat_plane_v8(),
            induction: Induction::NaturallyAspirated,
            mechanical: MechanicalSpec {
                timing_chain: Some(ImpulsiveSpec::order(24.0, 0.22)),
                gear_whine: Some(ImpulsiveSpec::order(36.0, 0.25)),
                ..MechanicalSpec::default()
            },
            exhaust: ExhaustSystem {
                primaries: vec![PipeSection::from_diameter(0.42, 0.041, 880.0); 8],
                collector: Collector::from_diameter(4, 0.065, 0.18),
                secondary: vec![],
                crossover: Crossover::XPipe { position: 0.80 },
                // `Silencer::Straight` appended nothing to the chain, so this
                // preset was acoustically a straight pipe in muffled mode by
                // construction — no muffled mode to compare against at all.
                // Same chamber V10 carries, on a similar-diameter primary.
                silencers: vec![Silencer::ExpansionChamber {
                    length: 0.40,
                    area_ratio: 3.5,
                    stages: 2,
                }],
                tailpipe: PipeSection::from_diameter(0.9, 0.065, 650.0),
                tailpipe_flanged: false,
                cutout_fitted: false,
                turbine: None,
            },
            intake: IntakeSystem {
                runners: vec![PipeSection::from_diameter(0.18, 0.048, 310.0); 8],
                plenum_volume: 0.0,
                throttle: ThrottleLayout::IndividualBodies { bore: 0.048 },
                airbox: None,
                snorkel: None,
                trumpet_flanged: false,
            },
            block_mass: 180.0,
            // Tighter than the cross-plane and shorter for it, which is half of why a
            // flat-plane sounds so much harder.
            bore_spacing: 0.104,
            redline: 8_600.0,
            float_rpm: default_float_rpm(8_600.0),
            idle: 900.0,
            inertia: 0.30,
            load: (5.0, 0.014, 6.5e-5),
            gearbox: Gearbox::generic_six_speed(),
            road_load: RoadLoad::generic_road_car(),
            clutch: Clutch::generic_road_car(),
            aperture_positions: AperturePositions::mid_engine_dual(),
            anti_lag: false,
            limiter_mode: LimiterMode::RotatingStutter,
            limiter_cut: LimiterCut::Spark,
            idle_governor: IdleGovernor::default(),
        }
    }

    /// 5.2 litre 90-degree V10.
    pub fn v10() -> Self {
        Self {
            name: "V10",
            note: "72 deg firing, unevenly split across the banks: metallic and hard.",
            model: CylinderModel {
                geometry: CylinderGeometry::new(0.0845, 0.0928, 0.1540, 12.7),
                valves: ValveTrain {
                    intake: ValveEvent::new(deg(704.0), deg(274.0), 0.0120, 0.0385, 0.68),
                    exhaust: ValveEvent::new(deg(486.0), deg(268.0), 0.0110, 0.0325, 0.65),
                }
                .with_aggressiveness(0.60),
                combustion: HeatRelease::Spark(WiebeProfile::new(
                    deg(342.0),
                    deg(50.0),
                    5.0,
                    2.1,
                    0.98,
                )),
                ..CylinderModel::default()
            },
            firing: FiringOrder::v10(),
            induction: Induction::centrifugal().with_blow_off(BlowOffVoicing::default()),
            mechanical: MechanicalSpec {
                gear_whine: Some(ImpulsiveSpec::order(35.0, 0.28)),
                ..MechanicalSpec::default()
            },
            exhaust: ExhaustSystem {
                primaries: vec![
                    PipeSection::from_diameter(0.34, 0.040, 880.0),
                    PipeSection::from_diameter(0.37, 0.040, 880.0),
                    PipeSection::from_diameter(0.35, 0.040, 880.0),
                    PipeSection::from_diameter(0.38, 0.040, 880.0),
                    PipeSection::from_diameter(0.36, 0.040, 880.0),
                    PipeSection::from_diameter(0.34, 0.040, 880.0),
                    PipeSection::from_diameter(0.37, 0.040, 880.0),
                    PipeSection::from_diameter(0.35, 0.040, 880.0),
                    PipeSection::from_diameter(0.38, 0.040, 880.0),
                    PipeSection::from_diameter(0.36, 0.040, 880.0),
                ],
                collector: Collector::from_diameter(5, 0.062, 0.14),
                secondary: vec![],
                crossover: Crossover::XPipe { position: 0.75 },
                silencers: vec![Silencer::ExpansionChamber {
                    length: 0.40,
                    area_ratio: 3.5,
                    stages: 2,
                }],
                tailpipe: PipeSection::from_diameter(1.1, 0.062, 650.0),
                tailpipe_flanged: false,
                cutout_fitted: false,
                turbine: None,
            },
            intake: IntakeSystem {
                runners: vec![PipeSection::from_diameter(0.22, 0.046, 310.0); 10],
                plenum_volume: 0.0,
                throttle: ThrottleLayout::IndividualBodies { bore: 0.050 },
                airbox: None,
                snorkel: None,
                trumpet_flanged: true,
            },
            block_mass: 220.0,
            // Siamesed: 2.75 mm of wall, which is about as thin as a block is cast.
            bore_spacing: 0.09,
            redline: 8_500.0,
            float_rpm: default_float_rpm(8_500.0),
            idle: 900.0,
            inertia: 0.40,
            load: (6.0, 0.017, 8.0e-5),
            gearbox: Gearbox::generic_six_speed(),
            road_load: RoadLoad::generic_road_car(),
            clutch: Clutch::generic_road_car(),
            aperture_positions: AperturePositions {
                tailpipes: vec![[-0.30, -2.1, 0.45], [0.30, -2.1, 0.45]],
                intake: [0.0, 0.30, 0.75],
                block: [0.0, -0.4, 0.45],
            },
            anti_lag: false,
            limiter_mode: LimiterMode::RotatingStutter,
            limiter_cut: LimiterCut::Spark,
            idle_governor: IdleGovernor::default(),
        }
    }

    /// 6.5 litre 60-degree V12.
    pub fn v12() -> Self {
        Self {
            name: "V12",
            note: "60 deg firing, even on both banks: no beat left to hear, only pitch.",
            model: CylinderModel {
                geometry: CylinderGeometry::new(0.095, 0.0764, 0.1380, 11.8),
                valves: ValveTrain {
                    intake: ValveEvent::new(deg(706.0), deg(278.0), 0.0124, 0.0405, 0.68),
                    exhaust: ValveEvent::new(deg(484.0), deg(272.0), 0.0114, 0.0340, 0.65),
                }
                .with_aggressiveness(0.65),
                combustion: HeatRelease::Spark(WiebeProfile::new(
                    deg(344.0),
                    deg(48.0),
                    5.0,
                    2.1,
                    0.98,
                )),
                ..CylinderModel::default()
            },
            firing: FiringOrder::v12(),
            induction: Induction::NaturallyAspirated,
            mechanical: MechanicalSpec {
                timing_chain: Some(ImpulsiveSpec::order(24.0, 0.20)),
                ..MechanicalSpec::default()
            },
            exhaust: ExhaustSystem {
                primaries: vec![PipeSection::from_diameter(0.30, 0.035, 900.0); 12],
                collector: Collector::from_diameter(6, 0.055, 0.15),
                secondary: vec![],
                crossover: Crossover::None,
                silencers: vec![Silencer::ExpansionChamber {
                    length: 0.35,
                    area_ratio: 3.0,
                    stages: 1,
                }],
                tailpipe: PipeSection::from_diameter(1.0, 0.055, 650.0),
                tailpipe_flanged: false,
                cutout_fitted: false,
                turbine: None,
            },
            intake: IntakeSystem {
                runners: vec![PipeSection::from_diameter(0.14, 0.042, 310.0); 12],
                plenum_volume: 0.0,
                throttle: ThrottleLayout::IndividualBodies { bore: 0.045 },
                airbox: None,
                snorkel: None,
                trumpet_flanged: true,
            },
            block_mass: 260.0,
            // Twelve of these make the longest block in the catalogue.
            bore_spacing: 0.1045,
            redline: 8_500.0,
            float_rpm: default_float_rpm(8_500.0),
            idle: 800.0,
            inertia: 0.48,
            load: (7.0, 0.020, 1.2e-4),
            gearbox: Gearbox::generic_six_speed(),
            road_load: RoadLoad::generic_road_car(),
            clutch: Clutch::generic_road_car(),
            aperture_positions: AperturePositions {
                tailpipes: vec![[-0.45, -2.5, 0.35], [0.45, -2.5, 0.35]],
                intake: [0.0, 1.6, 0.65],
                block: [0.0, 0.7, 0.5],
            },
            anti_lag: false,
            limiter_mode: LimiterMode::RotatingStutter,
            limiter_cut: LimiterCut::Spark,
            idle_governor: IdleGovernor::default(),
        }
    }

    /// Two-rotor Wankel, approximated on the reciprocating solver.
    ///
    /// See [`FiringOrder::two_rotor_wankel`] for what the four phase slots mean.
    /// The chamber here is 654 cc — a 13B's — and four of them per 720 degrees
    /// puts 2.6 litres through the cycle, which is the familiar result that a
    /// 1.3 litre rotary breathes like a 2.6 litre four-stroke.
    ///
    /// Two deliberate distortions of the cylinder model stand in for the
    /// epitrochoid the solver cannot describe, and one thing is not a
    /// distortion at all:
    ///
    /// - **A very long rod** (l/r = 12) flattens the slider-crank motion towards
    ///   the sinusoid a rotor's volume curve is closer to. Stage M5 of
    ///   `docs/MECHANISM_PLAN.md` looked at replacing this with a first-class
    ///   `RotorGeometry` and chose not to: the long rod is already within a
    ///   percent of the sinusoid, and re-deriving the same curve from an
    ///   honest epitrochoid would have spent the stage on geometry nobody can
    ///   hear. See [`crate::physics::rotor::RotorGeometry`] for what *is* now a
    ///   type instead of a comment — the 3:1 eccentric shaft ratio.
    /// - **A stretched Wiebe** (90 degrees, started early) stands in for the
    ///   long, thin, moving chamber, which burns slowly and is still burning
    ///   when the port uncovers. That is why a rotary's exhaust is so hot and
    ///   why it pops so readily on a cut.
    /// - **A low compression ratio** (10:1) is a distortion; the ports are not.
    ///   [`crate::physics::rotor::peripheral_port`] is a real area-versus-angle
    ///   port profile, opened far faster than the other three named profiles
    ///   in [`crate::physics::rotor`] — and its overlap is *why* this engine
    ///   fails to hold the Stage M4 idle governor while a side-ported profile
    ///   at the same target does not, not a number asserted to make it sound
    ///   right. See that module's doc comments for how far this preset's
    ///   reversion model actually tolerates pushing duration and opening rate
    ///   together before combustion stops sustaining itself at any throttle.
    pub fn two_rotor_wankel() -> Self {
        Self {
            name: "2-Rotor Wankel",
            note: "Four firings per cycle from two rotors: no valvetrain, no beat, just buzz.",
            model: CylinderModel {
                // 654 cc per chamber, with a rod long enough to be nearly
                // sinusoidal. See the doc comment above.
                geometry: CylinderGeometry::new(0.1050, 0.0755, 0.4530, 10.0),
                // Leading plug fires at the same angle the single-Wiebe
                // version of this preset always spark advance the ECU
                // schedules; the trailing plug follows twelve degrees later,
                // closer to the exhaust port, and accounts for a smaller
                // share of the charge — the leading flame has already
                // started consuming it by the time the trailing kernel
                // lights. See `physics::thermodynamics::TwoPlugCombustion`.
                combustion: HeatRelease::TwoPlug(TwoPlugCombustion::new(
                    WiebeProfile::new(deg(335.0), deg(90.0), 5.0, 1.6, 0.94),
                    deg(12.0),
                    0.35,
                )),
                valves: crate::physics::rotor::peripheral_port(),
                ..CylinderModel::default()
            },
            firing: FiringOrder::two_rotor_wankel(),
            induction: Induction::NaturallyAspirated,
            mechanical: MechanicalSpec::rotary(),
            exhaust: ExhaustSystem {
                primaries: vec![PipeSection::from_diameter(0.60, 0.048, 950.0); 2],
                collector: Collector::from_diameter(2, 0.060, 0.10),
                secondary: vec![],
                crossover: Crossover::None,
                silencers: vec![Silencer::Absorptive {
                    length: 0.50,
                    area: PI * 0.030 * 0.030,
                    packing_thickness: 0.025,
                    packing_absorption: 0.80,
                }],
                tailpipe: PipeSection::from_diameter(1.0, 0.060, 700.0),
                tailpipe_flanged: false,
                cutout_fitted: false,
                turbine: None,
            },
            intake: IntakeSystem {
                runners: vec![PipeSection::from_diameter(0.20, 0.052, 320.0); 2],
                plenum_volume: 0.0,
                throttle: ThrottleLayout::IndividualBodies { bore: 0.055 },
                airbox: None,
                snorkel: None,
                trumpet_flanged: false,
            },
            block_mass: 95.0,
            // Rotor housing pitch rather than a bore pitch — two housings and the
            // intermediate plate between them.
            bore_spacing: 0.115,
            // Lower than the peripheral port's own duration/ramp/area search
            // could push clear of by TB3's real reversion and choking
            // losses; see `physics::rotor::peripheral_port`'s doc comment.
            redline: 8_200.0,
            float_rpm: default_float_rpm(8_200.0),
            idle: 950.0,
            inertia: 0.20,
            load: (4.0, 0.013, 7.0e-5),
            gearbox: Gearbox::generic_six_speed(),
            road_load: RoadLoad::generic_road_car(),
            clutch: Clutch::generic_road_car(),
            aperture_positions: AperturePositions::rotary(),
            anti_lag: false,
            limiter_mode: LimiterMode::RotatingStutter,
            limiter_cut: LimiterCut::Spark,
            idle_governor: IdleGovernor::default(),
        }
    }

    /// 2.0 litre turbocharged inline four.
    ///
    /// The same architecture as [`EnginePreset::inline_four`] with the
    /// compression dropped from 11.5 to 9.6 — which is what boost costs an
    /// engine, and what makes a turbo four sound flatter and duller between
    /// firings than the atmospheric version of itself. The whistle over the top
    /// and the chirp on every lift are the compensation.
    pub fn turbo_inline_four() -> Self {
        Self {
            name: "Turbo Inline-4",
            note: "Even 180 deg firing under a small fast single: bark, whistle, flutter.",
            model: CylinderModel {
                geometry: CylinderGeometry::new(0.086, 0.086, 0.1345, 9.6),
                valves: ValveTrain {
                    intake: ValveEvent::new(deg(706.0), deg(242.0), 0.0105, 0.0380, 0.67),
                    exhaust: ValveEvent::new(deg(496.0), deg(238.0), 0.0098, 0.0330, 0.64),
                }
                .with_aggressiveness(0.30),
                combustion: HeatRelease::Spark(WiebeProfile::new(
                    deg(342.0),
                    deg(55.0),
                    5.0,
                    2.0,
                    0.97,
                )),
                ..CylinderModel::default()
            },
            firing: FiringOrder::inline_four(),
            induction: Induction::small_single().with_blow_off(BlowOffVoicing::default()),
            mechanical: MechanicalSpec::default(),
            exhaust: ExhaustSystem {
                primaries: vec![PipeSection::from_diameter(0.35, 0.038, 900.0); 4],
                collector: Collector::from_diameter(4, 0.050, 0.10),
                secondary: vec![],
                crossover: Crossover::None,
                silencers: vec![Silencer::ExpansionChamber {
                    length: 0.40,
                    area_ratio: 4.0,
                    stages: 1,
                }],
                tailpipe: PipeSection::from_diameter(1.2, 0.060, 600.0),
                tailpipe_flanged: false,
                cutout_fitted: false,
                turbine: Some(TurbineGeometry {
                    housing_ar: 0.63,
                    blade_count: 11,
                }),
            },
            intake: IntakeSystem {
                runners: vec![PipeSection::from_diameter(0.24, 0.042, 310.0); 4],
                plenum_volume: 2.5e-3,
                throttle: ThrottleLayout::Single { bore: 0.065 },
                airbox: Some(PipeSection::from_diameter(0.20, 0.065, 310.0)),
                snorkel: Some(PipeSection::from_diameter(0.35, 0.065, 300.0)),
                trumpet_flanged: true,
            },
            block_mass: 125.0,
            // The same architecture as the atmospheric four, per the note above.
            bore_spacing: 0.096,
            redline: 6_900.0,
            float_rpm: default_float_rpm(6_900.0),
            idle: 820.0,
            inertia: 0.24,
            load: (2.6, 0.009, 5.5e-5),
            gearbox: Gearbox::generic_six_speed(),
            road_load: RoadLoad::generic_road_car(),
            clutch: Clutch::generic_road_car(),
            aperture_positions: AperturePositions {
                tailpipes: vec![[0.35, -2.2, 0.35]],
                intake: [0.2, 1.4, 0.65],
                block: [0.0, 0.8, 0.5],
            },
            anti_lag: true,
            limiter_mode: LimiterMode::RotatingStutter,
            limiter_cut: LimiterCut::Spark,
            idle_governor: IdleGovernor::default(),
        }
    }

    /// 4.0 litre twin-turbo cross-plane V8.
    ///
    /// A hot-vee layout: both turbines sit inside the banks, close to the
    /// ports, which is why twins this size spool early. Acoustically the
    /// turbines are also two restrictions in the way of the blowdown pulse, so
    /// the cross-plane burble is still there but softened — the offbeat is
    /// audible as rhythm more than as crack.
    pub fn twin_turbo_v8() -> Self {
        Self {
            name: "Twin-turbo V8",
            note: "Hot-vee twins over the 90-180-270-180 burble: offbeat, but muffled.",
            model: CylinderModel {
                geometry: CylinderGeometry::new(0.0830, 0.0920, 0.1480, 10.0),
                valves: ValveTrain {
                    intake: ValveEvent::new(deg(704.0), deg(244.0), 0.0106, 0.0370, 0.67),
                    exhaust: ValveEvent::new(deg(494.0), deg(240.0), 0.0098, 0.0315, 0.64),
                }
                .with_aggressiveness(0.35),
                combustion: HeatRelease::Spark(WiebeProfile::new(
                    deg(342.0),
                    deg(52.0),
                    5.0,
                    2.0,
                    0.97,
                )),
                ..CylinderModel::default()
            },
            firing: FiringOrder::cross_plane_v8(),
            induction: Induction::twin().with_wastegate(WastegateVoicing::default()),
            mechanical: MechanicalSpec::default(),
            exhaust: ExhaustSystem {
                primaries: vec![
                    PipeSection::from_diameter(0.43, 0.044, 900.0),
                    PipeSection::from_diameter(0.47, 0.044, 900.0),
                    PipeSection::from_diameter(0.44, 0.044, 900.0),
                    PipeSection::from_diameter(0.46, 0.044, 900.0),
                    PipeSection::from_diameter(0.43, 0.044, 900.0),
                    PipeSection::from_diameter(0.47, 0.044, 900.0),
                    PipeSection::from_diameter(0.44, 0.044, 900.0),
                    PipeSection::from_diameter(0.46, 0.044, 900.0),
                ],
                collector: Collector::from_diameter(4, 0.058, 0.12),
                secondary: vec![],
                crossover: Crossover::HPipe {
                    position: 0.70,
                    area: PI * 0.022 * 0.022,
                },
                silencers: vec![Silencer::ExpansionChamber {
                    length: 0.55,
                    area_ratio: 4.5,
                    stages: 2,
                }],
                tailpipe: PipeSection::from_diameter(1.4, 0.065, 600.0),
                tailpipe_flanged: false,
                cutout_fitted: true,
                turbine: Some(TurbineGeometry {
                    housing_ar: 0.62,
                    blade_count: 9,
                }),
            },
            intake: IntakeSystem {
                runners: vec![PipeSection::from_diameter(0.26, 0.044, 310.0); 8],
                plenum_volume: 4.5e-3,
                throttle: ThrottleLayout::Single { bore: 0.080 },
                airbox: Some(PipeSection::from_diameter(0.25, 0.080, 310.0)),
                snorkel: Some(PipeSection::from_diameter(0.40, 0.075, 300.0)),
                trumpet_flanged: true,
            },
            block_mass: 235.0,
            // A short-stroke turbo V8 on tight centres.
            bore_spacing: 0.09,
            redline: 7_100.0,
            float_rpm: default_float_rpm(7_100.0),
            idle: 760.0,
            inertia: 0.44,
            load: (5.0, 0.016, 9.0e-5),
            gearbox: Gearbox::generic_six_speed(),
            road_load: RoadLoad::generic_road_car(),
            clutch: Clutch::generic_road_car(),
            aperture_positions: AperturePositions::front_engine_dual(),
            anti_lag: false,
            limiter_mode: LimiterMode::RotatingStutter,
            limiter_cut: LimiterCut::Spark,
            idle_governor: IdleGovernor::default(),
        }
    }

    /// 660 cc big single.
    ///
    /// One cylinder, one firing every two revolutions, and a flywheel light
    /// enough that the engine visibly changes speed inside its own cycle. That
    /// last part is the whole point of the preset: the crank ripple integrated
    /// at audio rate in Stage 1c is present on every engine here, but on a V12
    /// it is a couple of tenths of a percent and on this it is several percent
    /// at idle — the firing interval breathes, the pulse train is not periodic,
    /// and what comes out is the lope a thumper has and a four does not.
    ///
    /// Everything else follows from having one of everything: a single long
    /// primary with no collector to merge into, one throttle body on a stub
    /// runner with no plenum to smooth it, and forty-five kilos of engine to
    /// hang it all on, which rings at the top of the block band rather than the
    /// bottom.
    pub fn big_single() -> Self {
        Self {
            name: "Big Single",
            note: "One firing every two turns: the crank itself is the rhythm.",
            model: CylinderModel {
                // Oversquare and short-stroke, the way a modern thumper is
                // built so that it can rev at all.
                geometry: CylinderGeometry::new(0.1000, 0.0840, 0.1450, 12.0),
                valves: ValveTrain {
                    intake: ValveEvent::new(deg(695.0), deg(255.0), 0.0115, 0.042, 0.66),
                    exhaust: ValveEvent::new(deg(485.0), deg(250.0), 0.0105, 0.036, 0.62),
                },
                ..CylinderModel::default()
            },
            firing: FiringOrder::single(),
            induction: Induction::NaturallyAspirated,
            mechanical: MechanicalSpec {
                // No timing chain long enough to whirr and no gear train: a
                // single runs a short chain and a balance shaft, and the
                // balancer is the one thing on it that sings.
                timing_chain: None,
                gear_whine: Some(ImpulsiveSpec::order(2.0, 0.22)),
                piston_slap: Some(ImpulsiveSpec::per_cylinder(0.70)),
                ..MechanicalSpec::default()
            },
            exhaust: ExhaustSystem {
                // A long single primary straight into a megaphone: with one
                // cylinder there is nothing to collect, so the only tuning
                // available is the length of the one pipe.
                primaries: vec![PipeSection::from_diameter(0.62, 0.042, 880.0)],
                collector: Collector::from_diameter(1, 0.048, 0.16),
                secondary: vec![],
                crossover: Crossover::None,
                silencers: vec![Silencer::Absorptive {
                    length: 0.36,
                    area: PI * 0.024 * 0.024,
                    packing_thickness: 0.022,
                    packing_absorption: 0.75,
                }],
                tailpipe: PipeSection::from_diameter(0.35, 0.048, 620.0),
                tailpipe_flanged: false,
                cutout_fitted: false,
                turbine: None,
            },
            intake: IntakeSystem {
                runners: vec![PipeSection::from_diameter(0.18, 0.048, 310.0)],
                plenum_volume: 0.0,
                throttle: ThrottleLayout::IndividualBodies { bore: 0.046 },
                airbox: Some(PipeSection::from_diameter(0.16, 0.075, 300.0)),
                snorkel: None,
                trumpet_flanged: true,
            },
            block_mass: 45.0,
            // There is no second bore to be spaced from, so this is the width
            // of the one barrel and its jacket rather than a centre distance.
            bore_spacing: 0.125,
            // TB3's real reversion and wall heat in the intake plenum land
            // this single's charge a good deal hotter at high rpm than the
            // old blind plenum ever let it get — at this 12:1 compression it
            // is a real, self-limiting knock ceiling around 6900 rpm, not a
            // number picked to make a test pass.
            redline: 6_700.0,
            float_rpm: default_float_rpm(6_700.0),
            idle: 1_250.0,
            // A tenth of a kilogram metre squared, which is a light flywheel on
            // a heavy piston: exactly the combination that ripples.
            inertia: 0.10,
            load: (1.0, 0.004, 2.2e-5),
            gearbox: Gearbox::generic_six_speed(),
            road_load: RoadLoad::generic_road_car(),
            clutch: Clutch::generic_road_car(),
            aperture_positions: AperturePositions {
                tailpipes: vec![[0.20, -1.0, 0.30]],
                intake: [-0.15, 0.5, 0.70],
                block: [0.0, 0.2, 0.45],
            },
            anti_lag: false,
            limiter_mode: LimiterMode::HardCut,
            limiter_cut: LimiterCut::Spark,
            idle_governor: IdleGovernor::default(),
        }
    }

    /// 2.0 litre turbocharged inline-four diesel.
    ///
    /// The one engine in the catalogue that is not heard through its exhaust.
    /// Everything about it points the same way: nineteen to one compresses the
    /// air until it lights the spray on its own, which puts a premixed spike
    /// into the first crank degree of the burn; the spike is a hammer blow on a
    /// hundred and ninety kilos of cast iron, which answers at its own modes;
    /// and a turbine sits in the exhaust between the port and the tailpipe,
    /// swallowing most of what would otherwise have got out that way. What is
    /// left is clatter — structure-borne, wideband, and loudest at idle where
    /// the delay is longest. See [`crate::audio::structure`] for the path it
    /// takes, and [`DieselCombustion`] for where the spike comes from.
    ///
    /// Nineteen to one rather than the sixteen a modern common-rail engine
    /// runs, for the same reason the module docs give about the throttle: the
    /// block is solved on atmospheric air. A production turbodiesel gets its
    /// autoignition temperature partly from boost, and an engine modelled
    /// without the boost has to get all of it from the squeeze — which is
    /// exactly the compression ratio the pre-turbo diesels ran, for exactly
    /// that reason.
    pub fn turbo_diesel_four() -> Self {
        Self {
            name: "Turbodiesel I4",
            note: "No spark at all: a premixed spike, an iron block, and clatter.",
            model: CylinderModel {
                // Undersquare, as every diesel is: a long stroke gives the
                // torque and keeps the piston speed down where the fuel has
                // time to find its oxygen.
                geometry: CylinderGeometry::new(0.0830, 0.0920, 0.1470, 21.5),
                combustion: HeatRelease::Compression(DieselCombustion::default()),
                valves: ValveTrain {
                    // Ten degrees of overlap against the petrol four's forty:
                    // a diesel is pumping against a turbine, has no fuel in its
                    // intake charge to lose out of the exhaust, and would pull
                    // exhaust back in if the two events met. IVO 10 BTDC, IVC
                    // 25 ABDC; EVO 45 BBDC, EVC at top dead centre. The ports
                    // are swirl-biased rather than flow-biased, which is what
                    // the lower discharge coefficients are.
                    intake: ValveEvent::new(deg(710.0), deg(215.0), 0.0090, 0.035, 0.60),
                    exhaust: ValveEvent::new(deg(495.0), deg(225.0), 0.0085, 0.031, 0.57),
                }
                .with_aggressiveness(0.20),
                fuel_lhv: DIESEL_LHV,
                // Lean everywhere, because there is no throttle plate and the
                // fuel is the only thing being metered. Twenty-two to one is a
                // diesel at a decent load; it never goes rich.
                air_fuel_ratio: 22.0,
                ..CylinderModel::default()
            },
            firing: FiringOrder::inline_four(),
            induction: Induction::variable_geometry(),
            mechanical: MechanicalSpec::diesel(),
            exhaust: ExhaustSystem {
                // A short cast log into the turbine, not a set of tuned
                // primaries: on a diesel the exhaust manifold's job is to keep
                // the pulse energy hot and get it to the wheel.
                primaries: vec![PipeSection::from_diameter(0.30, 0.036, 750.0); 4],
                // The turbine housing, which is where four primaries on a
                // turbodiesel actually merge. The network has no turbine in it
                // — the solver is atmospheric and says so — so what stands in
                // for one here is the housing's *volume*, as the area the merge
                // opens into: seventy-two millimetres over a hundred and ten is
                // 0.45 litres, which is a real VGT housing for a two-litre
                // engine. That is not the whole of what a turbine does to a
                // blowdown pulse, since most of what it takes it takes as shaft
                // work, but an area step that large takes the pulse out of the
                // downstream pipe by reflecting it, which is the part of the
                // answer this network can represent honestly.
                collector: Collector::from_diameter(4, 0.072, 0.11),
                secondary: vec![],
                crossover: Crossover::None,
                silencers: vec![
                    // The aftertreatment can: an oxidation catalyst, a
                    // wall-flow particulate filter and an SCR brick, nearly a
                    // metre of ceramic monolith in one shell. Nothing on a
                    // petrol car is remotely like it, and it is most of why a
                    // modern diesel's tailpipe is the quietest part of it — a
                    // wall-flow filter makes the gas pass *through* a porous
                    // wall, which is a deep resistive layer by construction.
                    Silencer::Absorptive {
                        length: 0.55,
                        area: PI * 0.028 * 0.028,
                        packing_thickness: 0.030,
                        packing_absorption: 0.80,
                    },
                    Silencer::ExpansionChamber {
                        length: 0.50,
                        area_ratio: 5.5,
                        stages: 2,
                    },
                ],
                tailpipe: PipeSection::from_diameter(1.3, 0.055, 450.0),
                tailpipe_flanged: false,
                cutout_fitted: false,
                turbine: None,
            },
            intake: IntakeSystem {
                runners: vec![PipeSection::from_diameter(0.22, 0.040, 320.0); 4],
                plenum_volume: 2.8e-3,
                // The plate exists for shutdown and for EGR, and is wide open
                // at every load the engine is actually driven at.
                throttle: ThrottleLayout::Single { bore: 0.058 },
                airbox: Some(PipeSection::from_diameter(0.25, 0.075, 300.0)),
                snorkel: Some(PipeSection::from_diameter(0.45, 0.070, 300.0)),
                trumpet_flanged: false,
            },
            // Cast iron, and a deck thick enough to hold nineteen to one down.
            block_mass: 190.0,
            // An 88 mm centre on an 83 mm bore: 2.5 mm of iron, siamesed hard,
            // which is what a diesel four's short block costs.
            bore_spacing: 0.0885,
            redline: 5_000.0,
            float_rpm: default_float_rpm(5_000.0),
            idle: 800.0,
            inertia: 0.40,
            load: (2.2, 0.008, 4.0e-5),
            gearbox: Gearbox::generic_six_speed(),
            road_load: RoadLoad::generic_road_car(),
            clutch: Clutch::generic_road_car(),
            aperture_positions: AperturePositions::front_engine_single(),
            anti_lag: false,
            limiter_mode: LimiterMode::HardCut,
            limiter_cut: LimiterCut::Fuel,
            idle_governor: IdleGovernor::default(),
        }
    }

    /// 3.0 litre inline six on one large turbo.
    ///
    /// Six evenly spaced firings into a single collector leave no gap anywhere
    /// in the cycle, so the exhaust note is the most continuous in the
    /// catalogue — which is exactly the note a big lazy single sits best on
    /// top of. The wheel is sized for the top end rather than for response, so
    /// it arrives late, leaves slowly, and has the most stored energy to dump
    /// back through the compressor when the throttle shuts.
    pub fn turbo_inline_six() -> Self {
        Self {
            name: "Turbo Inline-6",
            note: "Even 120 deg firing, no gaps at all, and a big lazy single over it.",
            model: CylinderModel {
                geometry: CylinderGeometry::new(0.0840, 0.0900, 0.1450, 9.2),
                valves: ValveTrain {
                    intake: ValveEvent::new(deg(698.0), deg(246.0), 0.0105, 0.0350, 0.67),
                    exhaust: ValveEvent::new(deg(496.0), deg(242.0), 0.0098, 0.0305, 0.64),
                }
                .with_aggressiveness(0.35),
                combustion: HeatRelease::Spark(WiebeProfile::new(
                    deg(342.0),
                    deg(52.0),
                    5.0,
                    2.1,
                    0.98,
                )),
                ..CylinderModel::default()
            },
            firing: FiringOrder::inline_six(),
            induction: Induction::large_single()
                .with_blow_off(BlowOffVoicing::default())
                .with_wastegate(WastegateVoicing::default()),
            mechanical: MechanicalSpec::default(),
            exhaust: ExhaustSystem {
                primaries: vec![PipeSection::from_diameter(0.48, 0.042, 900.0); 6],
                collector: Collector::from_diameter(6, 0.064, 0.14),
                secondary: vec![],
                crossover: Crossover::None,
                silencers: vec![Silencer::Absorptive {
                    length: 0.60,
                    area: PI * 0.032 * 0.032,
                    packing_thickness: 0.035,
                    packing_absorption: 0.80,
                }],
                tailpipe: PipeSection::from_diameter(1.6, 0.070, 600.0),
                tailpipe_flanged: false,
                cutout_fitted: false,
                turbine: Some(TurbineGeometry {
                    housing_ar: 0.82,
                    blade_count: 10,
                }),
            },
            intake: IntakeSystem {
                runners: vec![PipeSection::from_diameter(0.30, 0.044, 310.0); 6],
                plenum_volume: 3.5e-3,
                throttle: ThrottleLayout::Single { bore: 0.075 },
                airbox: Some(PipeSection::from_diameter(0.20, 0.075, 310.0)),
                snorkel: Some(PipeSection::from_diameter(0.35, 0.070, 300.0)),
                trumpet_flanged: true,
            },
            block_mass: 195.0,
            // Ninety-one millimetres, the inline-six figure, and a long block with it.
            bore_spacing: 0.091,
            redline: 7_200.0,
            float_rpm: default_float_rpm(7_200.0),
            idle: 780.0,
            inertia: 0.33,
            load: (3.6, 0.012, 7.0e-5),
            gearbox: Gearbox::generic_six_speed(),
            road_load: RoadLoad::generic_road_car(),
            clutch: Clutch::generic_road_car(),
            aperture_positions: AperturePositions {
                tailpipes: vec![[0.35, -2.3, 0.35]],
                intake: [0.25, 1.4, 0.65],
                block: [0.0, 0.7, 0.5],
            },
            anti_lag: false,
            limiter_mode: LimiterMode::RotatingStutter,
            limiter_cut: LimiterCut::Spark,
            idle_governor: IdleGovernor::default(),
        }
    }

    /// 4.0 litre Porsche 911 GT3 Cup (Type 992) flat-six race engine.
    ///
    /// Screaming naturally aspirated boxer-six revving to 8750 rpm with
    /// individual throttle bodies, stiff valve train with solid lifters, tuned
    /// race headers, lightweight single-mass flywheel, and 6-speed sequential
    /// straight-cut dog-ring gear whine.
    pub fn gt3_cup_992() -> Self {
        Self {
            name: "Porsche 911 GT3 Cup (992)",
            note: "4.0L flat-six at 8750 rpm: screaming boxer harmonics and sequential gear whine.",
            model: CylinderModel {
                geometry: CylinderGeometry::new(0.1020, 0.0815, 0.1300, 13.3),
                valves: ValveTrain {
                    intake: ValveEvent::new(deg(705.0), deg(275.0), 0.0125, 0.0405, 0.68),
                    exhaust: ValveEvent::new(deg(485.0), deg(270.0), 0.0115, 0.0345, 0.65),
                },
                combustion: HeatRelease::Spark(WiebeProfile::new(
                    deg(344.0),
                    deg(50.0),
                    5.0,
                    2.1,
                    0.98,
                )),
                ..CylinderModel::default()
            },
            firing: FiringOrder::boxer_six(),
            induction: Induction::NaturallyAspirated,
            mechanical: MechanicalSpec {
                gear_whine: Some(ImpulsiveSpec::order(34.0, 0.32)),
                intake_valve: Some(ImpulsiveSpec::per_cylinder(0.55)),
                exhaust_valve: Some(ImpulsiveSpec::per_cylinder(0.55)),
                timing_chain: Some(ImpulsiveSpec::order(24.0, 0.20)),
                ..MechanicalSpec::default()
            },
            exhaust: ExhaustSystem {
                primaries: vec![PipeSection::from_diameter(0.38, 0.045, 900.0); 6],
                collector: Collector::from_diameter(3, 0.062, 0.15),
                secondary: vec![],
                crossover: Crossover::None,
                silencers: vec![Silencer::Straight],
                tailpipe: PipeSection::from_diameter(0.75, 0.062, 680.0),
                tailpipe_flanged: true,
                cutout_fitted: true,
                turbine: None,
            },
            intake: IntakeSystem {
                runners: vec![PipeSection::from_diameter(0.16, 0.048, 310.0); 6],
                plenum_volume: 3.5e-3,
                throttle: ThrottleLayout::IndividualBodies { bore: 0.050 },
                airbox: Some(PipeSection::from_diameter(0.12, 0.085, 300.0)),
                snorkel: Some(PipeSection::from_diameter(0.20, 0.080, 300.0)),
                trumpet_flanged: true,
            },
            block_mass: 145.0,
            bore_spacing: 0.118,
            redline: 8_750.0,
            float_rpm: default_float_rpm(8_750.0),
            idle: 1_100.0,
            inertia: 0.14,
            load: (4.0, 0.012, 4.5e-5),
            gearbox: Gearbox::generic_six_speed(),
            road_load: RoadLoad::generic_road_car(),
            clutch: Clutch::generic_road_car(),
            aperture_positions: AperturePositions::rear_engine_dual(),
            anti_lag: false,
            limiter_mode: LimiterMode::RotatingStutter,
            limiter_cut: LimiterCut::Spark,
            idle_governor: IdleGovernor::default(),
        }
    }

    /// 3.8 litre Porsche 911 GT3 Cup (Type 997.2) Mezger flat-six race engine.
    ///
    /// The legendary dry-sump Mezger race block with titanium connecting rods,
    /// solid lifter valvetrain chatter, equal-length race headers, and a searing
    /// 8500 rpm redline rooted in Porsche's 1998 Le Mans-winning 911 GT1.
    pub fn gt3_cup_997() -> Self {
        Self {
            name: "Porsche 911 GT3 Cup (997.2)",
            note: "3.8L Mezger flat-six at 8500 rpm: dry-sump mechanical clatter and GT1 lineage.",
            model: CylinderModel {
                geometry: CylinderGeometry::new(0.1027, 0.0764, 0.1300, 12.6),
                valves: ValveTrain {
                    intake: ValveEvent::new(deg(704.0), deg(270.0), 0.0122, 0.0400, 0.67),
                    exhaust: ValveEvent::new(deg(486.0), deg(265.0), 0.0112, 0.0340, 0.64),
                }
                .with_aggressiveness(0.70),
                combustion: HeatRelease::Spark(WiebeProfile::new(
                    deg(343.0),
                    deg(51.0),
                    5.0,
                    2.1,
                    0.98,
                )),
                ..CylinderModel::default()
            },
            firing: FiringOrder::boxer_six(),
            induction: Induction::NaturallyAspirated,
            mechanical: MechanicalSpec {
                gear_whine: Some(ImpulsiveSpec::order(32.0, 0.30)),
                intake_valve: Some(ImpulsiveSpec::per_cylinder(0.60)),
                exhaust_valve: Some(ImpulsiveSpec::per_cylinder(0.60)),
                timing_chain: Some(ImpulsiveSpec::order(24.0, 0.25)),
                ..MechanicalSpec::default()
            },
            exhaust: ExhaustSystem {
                primaries: vec![PipeSection::from_diameter(0.40, 0.043, 890.0); 6],
                collector: Collector::from_diameter(3, 0.055, 0.16),
                secondary: vec![],
                crossover: Crossover::None,
                silencers: vec![Silencer::Straight],
                tailpipe: PipeSection::from_diameter(0.80, 0.055, 660.0),
                tailpipe_flanged: true,
                cutout_fitted: true,
                turbine: None,
            },
            intake: IntakeSystem {
                runners: vec![PipeSection::from_diameter(0.18, 0.046, 310.0); 6],
                plenum_volume: 3.2e-3,
                throttle: ThrottleLayout::IndividualBodies { bore: 0.048 },
                airbox: Some(PipeSection::from_diameter(0.14, 0.080, 300.0)),
                snorkel: Some(PipeSection::from_diameter(0.22, 0.075, 300.0)),
                trumpet_flanged: true,
            },
            block_mass: 155.0,
            bore_spacing: 0.118,
            redline: 8_500.0,
            float_rpm: default_float_rpm(8_500.0),
            idle: 1_150.0,
            inertia: 0.16,
            load: (4.2, 0.013, 5.0e-5),
            gearbox: Gearbox::generic_six_speed(),
            road_load: RoadLoad::generic_road_car(),
            clutch: Clutch::generic_road_car(),
            aperture_positions: AperturePositions::rear_engine_dual(),
            anti_lag: false,
            limiter_mode: LimiterMode::RotatingStutter,
            limiter_cut: LimiterCut::Spark,
            idle_governor: IdleGovernor::default(),
        }
    }

    /// 6.2 litre Mercedes-AMG GT3 naturally aspirated cross-plane V8 race engine.
    ///
    /// The high-displacement M159 race engine with dry-sump lubrication and
    /// open side-exit race exhaust headers dumping ahead of the doors, delivering
    /// visceral cross-plane V8 thunder.
    pub fn amg_gt3() -> Self {
        Self {
            name: "Mercedes-AMG GT3",
            note: "6.2L M159 cross-plane V8: earth-shaking low-frequency thunder through open side-pipes.",
            model: CylinderModel {
                geometry: CylinderGeometry::new(0.1022, 0.0946, 0.1530, 12.0),
                valves: ValveTrain {
                    intake: ValveEvent::new(deg(702.0), deg(268.0), 0.0120, 0.0420, 0.68),
                    exhaust: ValveEvent::new(deg(488.0), deg(264.0), 0.0110, 0.0360, 0.65),
                }
                .with_aggressiveness(0.65),
                combustion: HeatRelease::Spark(WiebeProfile::new(
                    deg(342.0),
                    deg(52.0),
                    5.0,
                    2.0,
                    0.98,
                )),
                ..CylinderModel::default()
            },
            firing: FiringOrder::cross_plane_v8(),
            induction: Induction::NaturallyAspirated,
            mechanical: MechanicalSpec {
                gear_whine: Some(ImpulsiveSpec::order(36.0, 0.26)),
                timing_chain: Some(ImpulsiveSpec::order(24.0, 0.22)),
                piston_slap: Some(ImpulsiveSpec::per_cylinder(0.45)),
                ..MechanicalSpec::default()
            },
            exhaust: ExhaustSystem {
                primaries: vec![PipeSection::from_diameter(0.50, 0.046, 920.0); 8],
                collector: Collector::from_diameter(4, 0.066, 0.16),
                secondary: vec![],
                crossover: Crossover::None,
                silencers: vec![Silencer::Straight],
                tailpipe: PipeSection::from_diameter(0.60, 0.066, 720.0),
                tailpipe_flanged: false,
                cutout_fitted: true,
                turbine: None,
            },
            intake: IntakeSystem {
                runners: vec![PipeSection::from_diameter(0.24, 0.048, 310.0); 8],
                plenum_volume: 5.0e-3,
                throttle: ThrottleLayout::Single { bore: 0.082 },
                airbox: Some(PipeSection::from_diameter(0.18, 0.090, 300.0)),
                snorkel: Some(PipeSection::from_diameter(0.35, 0.085, 300.0)),
                trumpet_flanged: true,
            },
            block_mass: 205.0,
            bore_spacing: 0.1118,
            redline: 7_500.0,
            float_rpm: default_float_rpm(7_500.0),
            idle: 950.0,
            inertia: 0.24,
            load: (5.5, 0.018, 1.1e-4),
            gearbox: Gearbox::generic_six_speed(),
            road_load: RoadLoad::generic_road_car(),
            clutch: Clutch::generic_road_car(),
            aperture_positions: AperturePositions {
                tailpipes: vec![[-0.95, -0.40, 0.28], [0.95, -0.40, 0.28]],
                intake: [0.0, 1.40, 0.65],
                block: [0.0, 0.70, 0.45],
            },
            anti_lag: false,
            limiter_mode: LimiterMode::RotatingStutter,
            limiter_cut: LimiterCut::Spark,
            idle_governor: IdleGovernor::default(),
        }
    }

    /// 4.5 litre Ferrari 458 Italia GT3 naturally aspirated flat-plane V8 race engine.
    ///
    /// Screaming to 9200 rpm with individual velocity stacks, 4-into-1 race
    /// extractors with an X-pipe crossover, and lightweight internal components,
    /// producing a pure high-frequency tenor howl.
    pub fn ferrari_458_gt3() -> Self {
        Self {
            name: "Ferrari 458 Italia GT3",
            note: "4.5L flat-plane V8 screaming to 9200 rpm: tuned 4-into-1 race extractors and pure tenor howl.",
            model: CylinderModel {
                geometry: CylinderGeometry::new(0.0940, 0.0810, 0.1420, 13.0),
                valves: ValveTrain {
                    intake: ValveEvent::new(deg(708.0), deg(278.0), 0.0128, 0.0410, 0.69),
                    exhaust: ValveEvent::new(deg(482.0), deg(274.0), 0.0118, 0.0350, 0.66),
                }
                .with_aggressiveness(0.75),
                combustion: HeatRelease::Spark(WiebeProfile::new(
                    deg(345.0),
                    deg(48.0),
                    5.0,
                    2.1,
                    0.98,
                )),
                ..CylinderModel::default()
            },
            firing: FiringOrder::flat_plane_v8(),
            induction: Induction::NaturallyAspirated,
            mechanical: MechanicalSpec {
                gear_whine: Some(ImpulsiveSpec::order(38.0, 0.28)),
                timing_chain: Some(ImpulsiveSpec::order(24.0, 0.22)),
                intake_valve: Some(ImpulsiveSpec::per_cylinder(0.55)),
                ..MechanicalSpec::default()
            },
            exhaust: ExhaustSystem {
                primaries: vec![PipeSection::from_diameter(0.40, 0.042, 910.0); 8],
                collector: Collector::from_diameter(4, 0.068, 0.18),
                secondary: vec![],
                crossover: Crossover::XPipe { position: 0.70 },
                silencers: vec![Silencer::Straight],
                tailpipe: PipeSection::from_diameter(0.70, 0.068, 700.0),
                tailpipe_flanged: true,
                cutout_fitted: true,
                turbine: None,
            },
            intake: IntakeSystem {
                runners: vec![PipeSection::from_diameter(0.15, 0.048, 310.0); 8],
                plenum_volume: 0.0,
                throttle: ThrottleLayout::IndividualBodies { bore: 0.050 },
                airbox: None,
                snorkel: None,
                trumpet_flanged: false,
            },
            block_mass: 170.0,
            bore_spacing: 0.104,
            redline: 8_950.0,
            float_rpm: default_float_rpm(8_950.0),
            idle: 1_100.0,
            inertia: 0.17,
            load: (4.5, 0.013, 4.8e-5),
            gearbox: Gearbox::generic_six_speed(),
            road_load: RoadLoad::generic_road_car(),
            clutch: Clutch::generic_road_car(),
            aperture_positions: AperturePositions::mid_engine_dual(),
            anti_lag: false,
            limiter_mode: LimiterMode::RotatingStutter,
            limiter_cut: LimiterCut::Spark,
            idle_governor: IdleGovernor::default(),
        }
    }

    /// 5.2 litre Audi R8 LMS GT3 naturally aspirated 90-degree V10 race engine.
    ///
    /// Uneven-bank 72-degree firing pattern with individual throttles and
    /// short, high-mounted race extractors screaming to 8800 rpm, paired with
    /// intense straight-cut transmission whine.
    pub fn r8_lms_gt3() -> Self {
        Self {
            name: "Audi R8 LMS GT3",
            note: "5.2L 90 deg V10 at 8800 rpm: uneven-bank acoustic fire and straight-cut race gear scream.",
            model: CylinderModel {
                geometry: CylinderGeometry::new(0.0845, 0.0928, 0.1540, 12.5),
                valves: ValveTrain {
                    intake: ValveEvent::new(deg(706.0), deg(274.0), 0.0125, 0.0380, 0.68),
                    exhaust: ValveEvent::new(deg(484.0), deg(270.0), 0.0115, 0.0325, 0.65),
                }
                .with_aggressiveness(0.70),
                combustion: HeatRelease::Spark(WiebeProfile::new(
                    deg(344.0),
                    deg(49.0),
                    5.0,
                    2.1,
                    0.98,
                )),
                ..CylinderModel::default()
            },
            firing: FiringOrder::v10(),
            induction: Induction::NaturallyAspirated,
            mechanical: MechanicalSpec {
                gear_whine: Some(ImpulsiveSpec::order(36.0, 0.30)),
                timing_chain: Some(ImpulsiveSpec::order(24.0, 0.20)),
                intake_valve: Some(ImpulsiveSpec::per_cylinder(0.50)),
                ..MechanicalSpec::default()
            },
            exhaust: ExhaustSystem {
                primaries: vec![PipeSection::from_diameter(0.35, 0.038, 910.0); 10],
                collector: Collector::from_diameter(5, 0.062, 0.15),
                secondary: vec![],
                crossover: Crossover::XPipe { position: 0.65 },
                silencers: vec![Silencer::Straight],
                tailpipe: PipeSection::from_diameter(0.80, 0.062, 700.0),
                tailpipe_flanged: true,
                cutout_fitted: true,
                turbine: None,
            },
            intake: IntakeSystem {
                runners: vec![PipeSection::from_diameter(0.18, 0.046, 310.0); 10],
                plenum_volume: 0.0,
                throttle: ThrottleLayout::IndividualBodies { bore: 0.050 },
                airbox: None,
                snorkel: None,
                trumpet_flanged: true,
            },
            block_mass: 210.0,
            bore_spacing: 0.090,
            redline: 8_800.0,
            float_rpm: default_float_rpm(8_800.0),
            idle: 1_050.0,
            inertia: 0.19,
            load: (4.8, 0.014, 5.5e-5),
            gearbox: Gearbox::generic_six_speed(),
            road_load: RoadLoad::generic_road_car(),
            clutch: Clutch::generic_road_car(),
            aperture_positions: AperturePositions::mid_engine_dual(),
            anti_lag: false,
            limiter_mode: LimiterMode::RotatingStutter,
            limiter_cut: LimiterCut::Spark,
            idle_governor: IdleGovernor::default(),
        }
    }
}

// ---------------------------------------------------------------------------
// Dyno modes and run tracking
// ---------------------------------------------------------------------------

/// How the engine is loaded on the dyno bench.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DynoMode {
    /// Natural free-revving against flywheel inertia and drag load.
    FreeRev,
    /// Isochronous speed hold: absorber balances engine torque to maintain target speed.
    RpmHold { target_rpm: f64 },
    /// Automated wide-open-throttle sweep from start to redline at controlled acceleration.
    SweepPull { start_rpm: f64, rate_rpm_s: f64 },
    /// Electric motoring: dyno spins engine with ignition cut to measure friction and pumping.
    Motoring { target_rpm: f64 },
}

/// One measured sample on a dyno pull.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DynoPoint {
    /// Engine speed [rev/min].
    pub rpm: f64,
    /// Brake torque [N m].
    pub torque: f64,
    /// Brake power [kW].
    pub power_kw: f64,
}

/// A recorded dyno pull trace with peak values and atmospheric correction.
#[derive(Debug, Clone, PartialEq)]
pub struct DynoRun {
    /// Recorded points in speed order.
    pub points: Vec<DynoPoint>,
    /// Peak brake torque measured during the pull.
    pub peak_torque: Option<DynoPoint>,
    /// Peak brake power measured during the pull.
    pub peak_power: Option<DynoPoint>,
    /// SAE J1349 atmospheric power correction factor [-].
    pub sae_correction: f64,
}

impl Default for DynoRun {
    fn default() -> Self {
        Self::new(1.0)
    }
}

impl DynoRun {
    /// Builds a new empty run with a given SAE correction factor.
    pub fn new(sae_correction: f64) -> Self {
        Self {
            points: Vec::new(),
            peak_torque: None,
            peak_power: None,
            sae_correction,
        }
    }

    /// Records a measurement sample.
    pub fn record(&mut self, rpm: f64, torque: f64) {
        if !rpm.is_finite() || !torque.is_finite() || rpm <= 0.0 {
            return;
        }
        let power_kw = torque * rpm * PI / 30.0 / 1_000.0;
        let point = DynoPoint {
            rpm,
            torque,
            power_kw,
        };
        if self.peak_torque.is_none_or(|p| torque > p.torque) {
            self.peak_torque = Some(point);
        }
        if self.peak_power.is_none_or(|p| power_kw > p.power_kw) {
            self.peak_power = Some(point);
        }
        self.points.push(point);
    }
}

/// Computes the SAE J1349 net power atmospheric correction factor.
///
/// Standard reference conditions: 25 °C (298.15 K), 99.0 kPa dry air pressure.
/// Formula: CF = 1.18 * (99.0 / P_dry_kpa) * sqrt(T_amb_k / 298.15) - 0.18
pub fn sae_j1349_correction(ambient_pa: f64, ambient_k: f64) -> f64 {
    let p_dry_kpa = (ambient_pa / 1_000.0).max(50.0);
    let t_k = ambient_k.max(200.0);
    (1.18 * (99.0 / p_dry_kpa) * (t_k / 298.15).sqrt() - 0.18).clamp(0.80, 1.30)
}

// ---------------------------------------------------------------------------
// Driveline
// ---------------------------------------------------------------------------

/// A flywheel with a dyno brake on it, driven by the block's torque.
///
/// This is what closes the loop. The block is asked for torque at the speed the
/// flywheel is currently turning, that torque less the brake load accelerates
/// the flywheel, and the new speed goes back into the block on the next frame.
/// The engine therefore finds its own speed, and the note follows from that
/// rather than from a scripted ramp.
#[derive(Debug, Clone)]
pub struct Driveline {
    /// Crankshaft speed [rev/min].
    pub rpm: f64,
    /// Throttle actually applied, after pedal travel [-].
    pub throttle: f64,
    /// Where the pedal is being asked to go [-].
    pub throttle_target: f64,
    /// Whether the driver is holding the ignition cut.
    pub manual_cut: bool,
    /// Holds the idle speed by opening a bypass past the throttle plate.
    pub idle_governor: IdleGovernor,
    /// Period and depth of the idle limit cycle, when there is one.
    pub idle_hunt: IdleHunt,
    /// Speed at which the limiter cuts [rev/min].
    pub redline: f64,
    /// Speed the governor holds once the engine is warm [rev/min].
    pub idle: f64,
    /// How far above the warm idle the governor holds a stone-cold engine [-].
    ///
    /// A cold engine is given a fast idle for reasons that are all real: the oil
    /// is thick enough that the same throttle leaves less net torque, the fuel
    /// that condenses on a cold port never burns, and the catalyst has to be lit
    /// before it does anything. Six tenths puts an 850 rpm idle at 1360 cold,
    /// which is where a production engine actually sits on a winter morning, and
    /// it decays with [`EngineThermal::cold_fraction`] rather than on a timer.
    pub cold_idle_rise: f64,
    /// Rotating inertia [kg m^2].
    pub inertia: f64,
    /// Brake load coefficients, see [`EnginePreset::load`].
    ///
    /// Accessories, bearing drag and windage — the engine's own internal
    /// resistance, independent of whatever the wheels are asking for. This
    /// used to be the whole of the load; since [`Gearbox`] and [`RoadLoad`]
    /// it is only this term, and the dyno's absorber curve besides.
    pub load: (f64, f64, f64),
    /// Engine braking at a fully shut throttle [N m], see
    /// [`CLOSED_THROTTLE_PMEP`].
    pub pumping: f64,
    /// Gearbox: ratios, final drive, and the gear currently selected.
    pub gearbox: Gearbox,
    /// The road the driven wheels push against.
    pub road_load: RoadLoad,
    /// Couples the crank to the driven side when a gear is selected.
    pub clutch: Clutch,
    /// Driven-side speed, referred to the crank through the current overall
    /// ratio so it is directly comparable to `rpm` [rad/s]. Meaningless in
    /// neutral, where nothing couples to it.
    pub vehicle_omega: f64,
    /// What the clutch did last frame; for telemetry.
    pub clutch_state: ClutchState,
    /// Brake torque the block last reported at this speed [N m].
    pub torque: f64,
    /// Whether the exhaust cutout is currently open.
    ///
    /// Live state, not fitment — defaults closed regardless of whether this
    /// preset's [`ExhaustSystem::cutout_fitted`](crate::physics::plumbing::ExhaustSystem::cutout_fitted)
    /// is set, so a car with a cutout valve does not drive around with it open.
    pub exhaust_cutout: bool,
    /// Whether anti-lag is enabled on lift.
    pub anti_lag: bool,
    /// The starter motor, and whether it is currently meshed.
    ///
    /// Out, and never sized, on a driveline built by [`Driveline::new`] — that
    /// is an engine already running, and a motor nothing ever asks for torque
    /// from does not need specifying. [`Driveline::cranking`] is where one is
    /// chosen for the engine it has to turn.
    pub starter: Starter,
    /// Active dyno loading mode.
    pub dyno_mode: DynoMode,
    /// Opposing torque applied by the dyno absorber [N m].
    pub dyno_absorber_torque: f64,
    /// Speed error integral for the isochronous governor [rad].
    pub dyno_hold_integral: f64,
    /// Active dyno pull being recorded, if any.
    pub active_pull: Option<DynoRun>,
    /// Completed dyno pull held for display and comparison.
    pub last_pull: Option<DynoRun>,
}

impl Driveline {
    /// Builds a driveline for a preset, idling.
    pub fn new(preset: &EnginePreset) -> Self {
        Self {
            rpm: preset.idle,
            throttle: 0.0,
            throttle_target: 0.0,
            manual_cut: false,
            idle_governor: preset.idle_governor,
            idle_hunt: IdleHunt::default(),
            redline: preset.redline,
            idle: preset.idle,
            cold_idle_rise: COLD_IDLE_RISE,
            inertia: preset.inertia,
            load: preset.load,
            pumping: CLOSED_THROTTLE_PMEP * preset.displacement() / (4.0 * PI),
            torque: 0.0,
            gearbox: preset.gearbox.clone(),
            road_load: preset.road_load,
            clutch: preset.clutch,
            vehicle_omega: 0.0,
            clutch_state: ClutchState::Open,
            // Closed regardless of whether this preset has a cutout fitted —
            // fitment and state are different things, and a car does not
            // drive around with its cutout open by default just because it
            // has one.
            exhaust_cutout: false,
            anti_lag: preset.anti_lag,
            starter: Starter::default(),
            dyno_mode: DynoMode::FreeRev,
            dyno_absorber_torque: 0.0,
            dyno_hold_integral: 0.0,
            active_pull: None,
            last_pull: None,
        }
    }

    /// A stopped engine with the key at START.
    ///
    /// The only entry point that begins below [`STALL_RPM`], because it is the
    /// only one where being below it is not a stall: an engine at rest with the
    /// starter meshed is on its way up, not on its way out. Everything from
    /// there is the starter against the engine's own compression — see
    /// [`Driveline::crank`].
    pub fn cranking(preset: &EnginePreset, block: &mut EngineBlock) -> Self {
        // A crank has to have something to crank against, and the phase ring is
        // where that lives. Seeded here rather than left to the caller because
        // forgetting it is silent: the starter spins the engine up against no
        // compression at all and the whole sequence is over before the first
        // cell is written.
        block.prime_ring(Self::SEED_RPM);
        let mut driveline = Self::new(preset);
        driveline.rpm = 0.0;
        driveline.starter = Starter::for_engine(
            block.total_displacement(),
            block.peak_motored_resistance(),
            preset.inertia,
        );
        driveline.starter.engage();
        driveline
    }

    /// Speed the seed cycle in [`EngineBlock::prime_ring`] is solved at [rev/min].
    ///
    /// Roughly where a starter settles, so the charge the ring is seeded with
    /// is the one the first compression stroke is about to meet rather than a
    /// cycle from some speed the engine is not at.
    pub const SEED_RPM: f64 = 200.0;

    /// Speed the governor is holding for a given engine [rev/min].
    ///
    /// Not a constant: it is the warm idle lifted by how cold the block still
    /// is, so a cold start idles fast and comes down as the engine warms rather
    /// than stepping when a timer runs out.
    pub fn idle_target(&self, block: &EngineBlock) -> f64 {
        self.idle * (1.0 + self.cold_idle_rise * block.thermal.cold_fraction())
    }

    /// Road speed implied by the driven side, or `0` in neutral, where
    /// nothing couples the crank to a road at all [m/s].
    pub fn vehicle_speed_mps(&self) -> f64 {
        match self.gearbox.overall_ratio() {
            Some(ratio) => self.road_load.road_speed(self.vehicle_omega, ratio),
            None => 0.0,
        }
    }

    /// Whether the limiter is cutting, as distinct from the driver.
    pub fn on_the_limiter(&self) -> bool {
        self.rpm >= self.redline
    }

    /// Whether ignition is cut this frame, by the limiter or by the driver.
    pub fn cutting(&self) -> bool {
        self.manual_cut || self.on_the_limiter()
    }

    /// Whether drive torque is cut this frame by manual cut or by the ECU rev limiter.
    pub fn is_cutting(&self, block: &EngineBlock) -> bool {
        self.manual_cut
            || block.ecu.active_cut != LimiterCut::None
            || (self.rpm >= self.redline && block.ecu.limiter_cut_type != LimiterCut::None)
    }

    /// Moves the pedal, clamped to its travel.
    pub fn nudge_throttle(&mut self, delta: f64) {
        self.throttle_target = (self.throttle_target + delta).clamp(0.0, 1.0);
    }

    /// Toggles isochronous RPM hold at current engine speed.
    pub fn toggle_rpm_hold(&mut self) {
        match self.dyno_mode {
            DynoMode::RpmHold { .. } => {
                self.dyno_mode = DynoMode::FreeRev;
                self.dyno_absorber_torque = 0.0;
            }
            _ => {
                let target = (self.rpm / 100.0).round() * 100.0;
                self.dyno_mode = DynoMode::RpmHold {
                    target_rpm: target.clamp(self.idle, self.redline),
                };
            }
        }
    }

    /// Nudges held RPM target by delta if RPM hold is active.
    pub fn nudge_held_rpm(&mut self, delta: f64) {
        if let DynoMode::RpmHold { target_rpm } = self.dyno_mode {
            self.dyno_mode = DynoMode::RpmHold {
                target_rpm: (target_rpm + delta).clamp(self.idle, self.redline),
            };
        }
    }

    /// Triggers an automated wide-open-throttle sweep pull to redline.
    pub fn trigger_sweep_pull(&mut self) {
        self.active_pull = None;
        let start_rpm = (self.idle * 1.5).max(1800.0).min(self.redline - 500.0);
        self.dyno_mode = DynoMode::SweepPull {
            start_rpm,
            rate_rpm_s: 300.0,
        };
    }

    /// Advances the flywheel one frame from the block's solved torque.
    pub fn update(&mut self, block: &mut EngineBlock, dt: f64) {
        // A meshed starter overrides every loading mode: there is no dyno
        // absorber, no gear and no governor on an engine that is not running
        // yet, and the pedal does nothing a driver can hear.
        if self.starter.engaged {
            block.ecu.cranking = true;
            self.crank(block, dt);
            return;
        }
        block.ecu.cranking = false;
        match self.dyno_mode {
            DynoMode::FreeRev => {
                self.dyno_absorber_torque = 0.0;
                self.dyno_hold_integral = 0.0;

                // Pedal travel. A step input would be both unrealistic and a parameter
                // discontinuity for the audio thread to chase.
                let slew = 1.0 - (-dt / 0.12).exp();
                self.throttle += (self.throttle_target - self.throttle) * slew;

                // Idle governor: enough air to hold the idle speed, and no
                // more. This is what stops the engine stalling the moment the
                // pedal comes up. The speed it holds is the engine's own, and
                // a cold one is held faster. It commands the bypass, which is
                // a hole in the manifold rather than a number multiplying a
                // torque, so what it does reaches the cylinder as charge —
                // which is the only reason the loop through reversion and
                // burn completeness closes at all. See [`IdleGovernor`].
                let target = self.idle_target(block);
                let bypass = self.idle_governor.update(target, self.rpm, dt);
                block.idle_bypass = bypass;
                // Only meaningful while the governor is the thing holding the
                // speed. A driver on the pedal swings the engine far harder
                // than any lope, and calling that a limit cycle would be a
                // measurement of the driver.
                if self.throttle < IDLE_PEDAL_THRESHOLD {
                    self.idle_hunt.observe(self.rpm, dt);
                } else {
                    self.idle_hunt.reset();
                }
                let effective = self.throttle.max(bypass * IDLE_BYPASS_PEDAL_AUTHORITY);

                match self.gearbox.overall_ratio() {
                    None => {
                        // Neutral: no drive path, so this is the free-revving
                        // flywheel against its own drag curve — the special
                        // case [`Gearbox::overall_ratio`] promises. The pedal
                        // swings the plate, exactly as it does in gear, and
                        // the torque the block reports is therefore already
                        // throttled; see the note in
                        // [`Driveline::update_in_gear`] on why scaling it a
                        // second time would derate it twice. Until this stage
                        // it *was* scaled a second time, because in neutral
                        // the block never saw the pedal at all — and an engine
                        // whose cylinder never sees a shut throttle has no
                        // manifold vacuum, no reversion and no idle to lope.
                        block.throttle = self.throttle;
                        self.torque = block.mean_brake_torque(self.rpm);
                        let drive = if self.is_cutting(block) {
                            0.0
                        } else {
                            self.torque
                        };

                        // Accessories, then bearing drag, then windage, then the throttle
                        // plate. Only the last of these depends on the pedal: it is the whole
                        // of engine braking, and without it a lift from the limiter takes the
                        // best part of a minute to come back to idle.
                        let (a, b, c) = self.load;
                        let omega = self.rpm * PI / 30.0;
                        let load =
                            a + b * omega + c * omega * omega + (1.0 - effective) * self.pumping;

                        let alpha = (drive - load) / self.inertia.max(1e-3);
                        let omega = (omega + alpha * dt).max(STALL_RPM * PI / 30.0);
                        self.rpm = omega * 30.0 / PI;
                    }
                    Some(ratio) => self.update_in_gear(block, dt, ratio, effective),
                }
            }
            DynoMode::RpmHold { target_rpm } => {
                block.ecu.motoring = false;
                let slew = 1.0 - (-dt / 0.12).exp();
                self.throttle += (self.throttle_target - self.throttle) * slew;

                let effective = self.throttle;
                block.throttle = effective;
                self.torque = block.mean_brake_torque(self.rpm);
                let drive = if self.is_cutting(block) {
                    0.0
                } else {
                    self.torque
                };

                let (a, b, c) = self.load;
                let omega = self.rpm * PI / 30.0;
                let natural_load =
                    a + b * omega + c * omega * omega + (1.0 - effective) * self.pumping;

                // Closed-loop PI absorber:
                let target_omega = target_rpm * PI / 30.0;
                let error = omega - target_omega;
                self.dyno_hold_integral = (self.dyno_hold_integral + error * dt).clamp(-50.0, 50.0);

                let kp = self.inertia.max(0.1) * 35.0;
                let ki = self.inertia.max(0.1) * 70.0;
                let absorber =
                    (drive - natural_load + kp * error + ki * self.dyno_hold_integral).max(0.0);
                self.dyno_absorber_torque = absorber;

                let total_load = natural_load + absorber;
                let alpha = (drive - total_load) / self.inertia.max(1e-3);
                let new_omega = (omega + alpha * dt).max(STALL_RPM * PI / 30.0);
                self.rpm = new_omega * 30.0 / PI;
            }
            DynoMode::SweepPull {
                start_rpm,
                rate_rpm_s,
            } => {
                block.ecu.motoring = false;
                self.dyno_hold_integral = 0.0;

                if self.active_pull.is_none() {
                    let cf = sae_j1349_correction(
                        block.environment.pressure,
                        block.environment.temperature,
                    );
                    self.active_pull = Some(DynoRun::new(cf));
                }

                if self.rpm < start_rpm {
                    self.throttle_target = 0.40;
                    let slew = 1.0 - (-dt / 0.10).exp();
                    self.throttle += (self.throttle_target - self.throttle) * slew;
                    block.throttle = self.throttle;
                    self.torque = block.mean_brake_torque(self.rpm);
                    let drive = if self.is_cutting(block) {
                        0.0
                    } else {
                        self.torque
                    };
                    let (a, b, c) = self.load;
                    let omega = self.rpm * PI / 30.0;
                    let natural_load =
                        a + b * omega + c * omega * omega + (1.0 - self.throttle) * self.pumping;
                    // Dyno drive assist ensures the engine reaches test speed even if
                    // cold, stalled, or running aggressive race ports at low speed.
                    let alpha = ((drive - natural_load) / self.inertia.max(1e-3)).max(50.0);
                    let new_omega = (omega + alpha * dt).max(STALL_RPM * PI / 30.0);
                    self.rpm = new_omega * 30.0 / PI;
                    self.dyno_absorber_torque = 0.0;
                } else {
                    self.throttle = 1.0;
                    self.throttle_target = 1.0;
                    block.throttle = 1.0;
                    self.torque = block.mean_brake_torque(self.rpm);

                    if let Some(pull) = self.active_pull.as_mut() {
                        let should_record = match pull.points.last() {
                            Some(last) => (self.rpm - last.rpm) >= 50.0,
                            None => true,
                        };
                        if should_record {
                            pull.record(self.rpm, self.torque);
                        }
                    }

                    let omega_step = rate_rpm_s * (PI / 30.0) * dt;
                    let omega = self.rpm * PI / 30.0 + omega_step;
                    self.rpm = omega * 30.0 / PI;

                    let (a, b, c) = self.load;
                    let natural_load = a + b * omega + c * omega * omega;
                    let inertial_torque = self.inertia * (rate_rpm_s * PI / 30.0);
                    self.dyno_absorber_torque =
                        (self.torque - natural_load - inertial_torque).max(0.0);

                    if self.rpm >= self.redline {
                        if let Some(mut pull) = self.active_pull.take() {
                            pull.record(self.rpm, self.torque);
                            self.last_pull = Some(pull);
                        }
                        self.dyno_mode = DynoMode::FreeRev;
                        self.throttle = 0.0;
                        self.throttle_target = 0.0;
                        block.throttle = 0.0;
                        self.dyno_absorber_torque = 0.0;
                    }
                }
            }
            DynoMode::Motoring { target_rpm } => {
                self.dyno_hold_integral = 0.0;
                self.throttle = 0.0;
                self.throttle_target = 0.0;
                block.throttle = 0.0;
                block.ecu.motoring = true;

                let rate = 800.0 * dt;
                if self.rpm < target_rpm {
                    self.rpm = (self.rpm + rate).min(target_rpm);
                } else {
                    self.rpm = (self.rpm - rate).max(target_rpm);
                }

                let peak_p = block.ring.peak_pressure();
                let mps = block.geometry().mean_piston_speed(self.rpm);
                let fmep = block
                    .friction
                    .fmep(peak_p, mps, block.thermal.oil_temperature());
                let friction_tau = fmep * block.total_displacement() / (4.0 * PI);
                let pumping_tau = self.pumping;
                let motoring_tau = friction_tau + pumping_tau;

                self.torque = -motoring_tau;
                self.dyno_absorber_torque = -motoring_tau;
            }
        }
    }

    /// Advances the crank one frame under the starter.
    ///
    /// Four torques and nothing else: the motor, the engine's own gas torque at
    /// the angle the crank has actually reached, the friction the cold oil is
    /// charging for, and the accessories. No governor, because a starter motor
    /// is not a speed controller; no [`STALL_RPM`] floor, because an engine
    /// being cranked is below it by definition and clamping there would erase
    /// the whole of the compression stroke.
    ///
    /// Everything audible about cranking falls out of the second term. It
    /// swings tens of newton metres either side of zero every compression
    /// stroke, the motor's own curve gives back more torque the more it is
    /// slowed, and the crank therefore lurches over each compression at a rate
    /// set by the firing interval. Nothing in here knows that is a sound.
    fn crank(&mut self, block: &mut EngineBlock, dt: f64) {
        let omega = self.rpm * PI / 30.0;
        let gas = block.instantaneous_indicated_torque();
        let friction = block.friction.torque(
            block.ring.peak_pressure(),
            block.geometry().mean_piston_speed(self.rpm),
            block.total_displacement(),
            block.thermal.oil_temperature(),
        );
        let (a, b, c) = self.load;
        let accessories = a + b * omega + c * omega * omega;

        let net = self.starter.torque(self.rpm) + gas - friction - accessories;
        let omega = (omega + net / self.inertia.max(1e-3) * dt).max(0.0);
        self.rpm = omega * 30.0 / PI;
        self.torque = gas;
        self.starter.update(self.rpm, dt);
    }

    /// Advances engine and driven-side speed one frame with a gear engaged.
    ///
    /// Two rotating masses — the crank and the vehicle, the latter reflected
    /// to the crank through `overall_ratio` — coupled by a clutch of limited
    /// capacity. Locked is tried first: if the torque that would take to hold
    /// them together fits inside the clutch's capacity, they move as one
    /// combined inertia. If it does not, or the two sides are not at the same
    /// speed to begin with, the clutch instead transmits its capacity signed
    /// with the slip direction, and the two sides integrate independently —
    /// which is what makes a standing start and a shift both work without a
    /// separate mode for either.
    fn update_in_gear(
        &mut self,
        block: &mut EngineBlock,
        dt: f64,
        overall_ratio: f64,
        effective: f64,
    ) {
        // The pedal now actually restricts what the cylinder can trap, so the
        // block's own torque curve already carries the pedal's effect — see
        // `EngineBlock::update_manifolds`. That replaces the synthetic
        // `0.05 + 0.95 * effective` scaling neutral driving still needs
        // (the block there never sees the pedal at all); scaling a torque
        // that is already throttle-restricted a second time would derate it
        // twice for one pedal position. Takes effect on the block's next
        // `update`, same one-frame lag as every other quantity read here off
        // last cycle's ring.
        block.throttle = effective;

        self.torque = block.mean_brake_torque(self.rpm);
        let drive = if self.is_cutting(block) {
            0.0
        } else {
            self.torque
        };

        let (a, b, c) = self.load;
        let engine_omega = self.rpm * PI / 30.0;
        let internal_load = a
            + b * engine_omega
            + c * engine_omega * engine_omega
            + (1.0 - effective) * self.pumping;

        let i_engine = self.inertia.max(1e-3);
        let i_reflected = self.road_load.reflected_inertia(overall_ratio).max(1e-6);
        let vehicle_speed = self.road_load.road_speed(self.vehicle_omega, overall_ratio);
        let road_torque = self.road_load.crank_torque(
            vehicle_speed,
            block.environment.air_density(),
            overall_ratio,
        );

        let capacity = self.clutch.capacity();
        let slip = engine_omega - self.vehicle_omega;

        // Try locked: one combined inertia, and the torque the clutch would
        // need to carry to keep it that way.
        let i_total = i_engine + i_reflected;
        let alpha_locked = (drive - internal_load - road_torque) / i_total;
        let lock_torque_needed = drive - internal_load - i_engine * alpha_locked;

        let (alpha_engine, alpha_vehicle) =
            if capacity > 0.0 && slip.abs() < 1e-2 && lock_torque_needed.abs() <= capacity {
                self.clutch_state = ClutchState::Locked;
                (alpha_locked, alpha_locked)
            } else if capacity <= 0.0 {
                self.clutch_state = ClutchState::Open;
                (
                    (drive - internal_load) / i_engine,
                    -road_torque / i_reflected,
                )
            } else {
                self.clutch_state = ClutchState::Slipping;
                let transmitted = self.clutch.slipping_torque(slip);
                (
                    (drive - internal_load - transmitted) / i_engine,
                    (transmitted - road_torque) / i_reflected,
                )
            };

        let new_engine_omega = (engine_omega + alpha_engine * dt).max(STALL_RPM * PI / 30.0);
        self.rpm = new_engine_omega * 30.0 / PI;
        self.vehicle_omega = (self.vehicle_omega + alpha_vehicle * dt).max(0.0);
    }

    /// What the audio path is being asked for this frame.
    pub fn controls(&self) -> EngineControls {
        EngineControls {
            throttle: self.throttle.clamp(0.0, 1.0),
            spark_cut: self.cutting() || matches!(self.dyno_mode, DynoMode::Motoring { .. }),
            exhaust_cutout: self.exhaust_cutout,
            anti_lag: self.anti_lag,
            // Zero once the pinion is out, which is what stops the whine: the
            // starter is not faded down when the engine catches, it is thrown
            // out of the gear it was singing with.
            starter_hz: self.starter.whine_hz(self.rpm),
        }
    }
}

#[cfg(test)]
mod tests;
