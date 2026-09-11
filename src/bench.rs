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

use std::f64::consts::PI;

use crate::audio::{
    AperturePositions, EngineControls, ImpulsiveSpec, Induction, MechanicalSpec, SnapshotSource,
    SynthConfig,
};
use crate::environment::Environment;
use crate::physics::cylinder::{deg, CylinderGeometry};
use crate::physics::engine_block::{EngineBlock, FiringOrder};
use crate::physics::plumbing::{
    Collector, Crossover, ExhaustSystem, IntakeSystem, PipeSection, Silencer, ThrottleLayout,
};
use crate::physics::thermodynamics::{CylinderModel, ValveEvent, ValveTrain, WiebeProfile};

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
    /// Speed the idle governor holds [rev/min].
    pub idle: f64,
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
        block.ecu.redline = self.redline;
        block
    }

    /// The snapshot source for this engine, with its turbo shaft if it has one.
    pub fn snapshot_source(&self, block: &EngineBlock) -> SnapshotSource {
        SnapshotSource::with_induction(block, self.induction)
    }

    /// The synth configuration for this engine, voiced for its induction and mechanical spec.
    pub fn synth_config(&self, block: &EngineBlock, sample_rate: f32) -> SynthConfig {
        let mut config = SynthConfig::from_block(block, sample_rate)
            .with_induction(self.induction)
            .with_mechanical(self.mechanical);
        config.structure = config.structure.with_bore_spacing(self.bore_spacing);
        config.aperture_positions = self.aperture_positions.clone();
        config
    }

    /// Whether a turbo is fitted.
    pub fn is_turbocharged(&self) -> bool {
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
        vec![
            Self::inline_four(),
            Self::cross_plane_v8(),
            Self::flat_plane_v8(),
            Self::v10(),
            Self::v12(),
            Self::two_rotor_wankel(),
            Self::turbo_inline_four(),
            Self::twin_turbo_v8(),
            Self::turbo_inline_six(),
        ]
    }

    /// 2.0 litre inline four.
    pub fn inline_four() -> Self {
        Self {
            name: "Inline-4",
            note: "Even 180 deg firing on one bank: a hard, plain four-cylinder bark.",
            model: CylinderModel {
                geometry: CylinderGeometry::new(0.086, 0.086, 0.1345, 11.5),
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
            },
            intake: IntakeSystem {
                runners: vec![PipeSection::from_diameter(0.28, 0.042, 310.0); 4],
                plenum_volume: 2.2e-3,
                throttle: ThrottleLayout::Single { bore: 0.060 },
                airbox: Some(PipeSection::from_diameter(0.15, 0.070, 300.0)),
                snorkel: Some(PipeSection::from_diameter(0.30, 0.065, 300.0)),
                trumpet_flanged: true,
            },
            block_mass: 110.0,
            // A production four on 96 mm centres: 5 mm of iron between the bores.
            bore_spacing: 0.096,
            redline: 7_400.0,
            idle: 850.0,
            inertia: 0.22,
            load: (3.0, 0.010, 7.0e-5),
            aperture_positions: AperturePositions::front_engine_single(),
        }
    }

    /// 5.0 litre cross-plane V8.
    pub fn cross_plane_v8() -> Self {
        Self {
            name: "Cross-plane V8",
            note: "90-180-270-180 gaps on each bank: the offbeat American burble.",
            model: CylinderModel {
                geometry: CylinderGeometry::new(0.094, 0.0895, 0.1500, 11.0),
                ..CylinderModel::default()
            },
            firing: FiringOrder::cross_plane_v8(),
            induction: Induction::NaturallyAspirated,
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
                silencers: vec![Silencer::ExpansionChamber {
                    length: 0.65,
                    area_ratio: 6.0,
                    stages: 2,
                }],
                tailpipe: PipeSection::from_diameter(1.5, 0.060, 600.0),
                tailpipe_flanged: false,
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
            idle: 750.0,
            inertia: 0.45,
            load: (6.0, 0.020, 1.3e-4),
            aperture_positions: AperturePositions::front_engine_dual(),
        }
    }

    /// 4.5 litre flat-plane V8.
    pub fn flat_plane_v8() -> Self {
        Self {
            name: "Flat-plane V8",
            note: "Even 180 deg on both banks: two inline-fours sharing a crank.",
            model: CylinderModel {
                geometry: CylinderGeometry::new(0.094, 0.0810, 0.1420, 12.5),
                // A shorter burn: flat-plane vees are built to rev, and a race
                // chamber lights faster than a road one.
                wiebe: WiebeProfile::new(deg(342.0), deg(52.0), 5.0, 2.1, 0.97),
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
                silencers: vec![Silencer::Straight],
                tailpipe: PipeSection::from_diameter(0.9, 0.065, 650.0),
                tailpipe_flanged: false,
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
            idle: 900.0,
            inertia: 0.30,
            load: (5.0, 0.014, 6.5e-5),
            aperture_positions: AperturePositions::mid_engine_dual(),
        }
    }

    /// 5.2 litre 90-degree V10.
    pub fn v10() -> Self {
        Self {
            name: "V10",
            note: "72 deg firing, unevenly split across the banks: metallic and hard.",
            model: CylinderModel {
                geometry: CylinderGeometry::new(0.0845, 0.0928, 0.1540, 12.7),
                wiebe: WiebeProfile::new(deg(344.0), deg(54.0), 5.0, 2.0, 0.97),
                ..CylinderModel::default()
            },
            firing: FiringOrder::v10(),
            induction: Induction::NaturallyAspirated,
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
            idle: 900.0,
            inertia: 0.40,
            load: (6.0, 0.017, 8.0e-5),
            aperture_positions: AperturePositions {
                tailpipes: vec![[-0.30, -2.1, 0.45], [0.30, -2.1, 0.45]],
                intake: [0.0, 0.30, 0.75],
                block: [0.0, -0.4, 0.45],
            },
        }
    }

    /// 6.5 litre 60-degree V12.
    pub fn v12() -> Self {
        Self {
            name: "V12",
            note: "60 deg firing, even on both banks: no beat left to hear, only pitch.",
            model: CylinderModel {
                geometry: CylinderGeometry::new(0.095, 0.0764, 0.1380, 11.8),
                wiebe: WiebeProfile::new(deg(344.0), deg(50.0), 5.0, 2.1, 0.97),
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
            idle: 800.0,
            inertia: 0.48,
            load: (7.0, 0.020, 1.2e-4),
            aperture_positions: AperturePositions {
                tailpipes: vec![[-0.45, -2.5, 0.35], [0.45, -2.5, 0.35]],
                intake: [0.0, 1.6, 0.65],
                block: [0.0, 0.7, 0.5],
            },
        }
    }

    /// Two-rotor Wankel, approximated on the reciprocating solver.
    ///
    /// See [`FiringOrder::two_rotor_wankel`] for what the four phase slots mean.
    /// The chamber here is 654 cc — a 13B's — and four of them per 720 degrees
    /// puts 2.6 litres through the cycle, which is the familiar result that a
    /// 1.3 litre rotary breathes like a 2.6 litre four-stroke.
    ///
    /// Three deliberate distortions of the cylinder model stand in for the
    /// epitrochoid the solver cannot describe:
    ///
    /// - **A very long rod** (l/r = 12) flattens the slider-crank motion towards
    ///   the sinusoid a rotor's volume curve is closer to.
    /// - **A stretched Wiebe** (90 degrees, started early) stands in for the
    ///   long, thin, moving chamber, which burns slowly and is still burning
    ///   when the port uncovers. That is why a rotary's exhaust is so hot and
    ///   why it pops so readily on a cut.
    /// - **A low compression ratio** (10:1) and wide ports with enormous overlap
    ///   stand in for peripheral porting, which has no valves to shut.
    pub fn two_rotor_wankel() -> Self {
        Self {
            name: "2-Rotor Wankel",
            note: "Four firings per cycle from two rotors: no valvetrain, no beat, just buzz.",
            model: CylinderModel {
                // 654 cc per chamber, with a rod long enough to be nearly
                // sinusoidal. See the doc comment above.
                geometry: CylinderGeometry::new(0.1050, 0.0755, 0.4530, 10.0),
                wiebe: WiebeProfile::new(deg(335.0), deg(90.0), 5.0, 1.6, 0.94),
                valves: ValveTrain {
                    intake: ValveEvent::new(deg(680.0), deg(280.0), 0.014, 0.048, 0.70),
                    exhaust: ValveEvent::new(deg(480.0), deg(280.0), 0.013, 0.042, 0.68),
                },
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
            redline: 8_800.0,
            idle: 950.0,
            inertia: 0.20,
            load: (4.0, 0.013, 7.0e-5),
            aperture_positions: AperturePositions::rotary(),
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
                ..CylinderModel::default()
            },
            firing: FiringOrder::inline_four(),
            induction: Induction::small_single(),
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
            idle: 820.0,
            inertia: 0.24,
            load: (2.6, 0.009, 5.5e-5),
            aperture_positions: AperturePositions {
                tailpipes: vec![[0.35, -2.2, 0.35]],
                intake: [0.2, 1.4, 0.65],
                block: [0.0, 0.8, 0.5],
            },
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
                ..CylinderModel::default()
            },
            firing: FiringOrder::cross_plane_v8(),
            induction: Induction::twin(),
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
            idle: 760.0,
            inertia: 0.44,
            load: (5.0, 0.016, 9.0e-5),
            aperture_positions: AperturePositions::front_engine_dual(),
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
                ..CylinderModel::default()
            },
            firing: FiringOrder::inline_six(),
            induction: Induction::large_single(),
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
            idle: 780.0,
            inertia: 0.33,
            load: (3.6, 0.012, 7.0e-5),
            aperture_positions: AperturePositions {
                tailpipes: vec![[0.35, -2.3, 0.35]],
                intake: [0.25, 1.4, 0.65],
                block: [0.0, 0.7, 0.5],
            },
        }
    }
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
    pub load: (f64, f64, f64),
    /// Engine braking at a fully shut throttle [N m], see
    /// [`CLOSED_THROTTLE_PMEP`].
    pub pumping: f64,
    /// Brake torque the block last reported at this speed [N m].
    pub torque: f64,
}

impl Driveline {
    /// Builds a driveline for a preset, idling.
    pub fn new(preset: &EnginePreset) -> Self {
        Self {
            rpm: preset.idle,
            throttle: 0.0,
            throttle_target: 0.0,
            manual_cut: false,
            redline: preset.redline,
            idle: preset.idle,
            cold_idle_rise: COLD_IDLE_RISE,
            inertia: preset.inertia,
            load: preset.load,
            pumping: CLOSED_THROTTLE_PMEP * preset.displacement() / (4.0 * PI),
            torque: 0.0,
        }
    }

    /// Speed the governor is holding for a given engine [rev/min].
    ///
    /// Not a constant: it is the warm idle lifted by how cold the block still
    /// is, so a cold start idles fast and comes down as the engine warms rather
    /// than stepping when a timer runs out.
    pub fn idle_target(&self, block: &EngineBlock) -> f64 {
        self.idle * (1.0 + self.cold_idle_rise * block.thermal.cold_fraction())
    }

    /// Whether the limiter is cutting, as distinct from the driver.
    pub fn on_the_limiter(&self) -> bool {
        self.rpm >= self.redline
    }

    /// Whether ignition is cut this frame, by the limiter or by the driver.
    pub fn cutting(&self) -> bool {
        self.manual_cut || self.on_the_limiter()
    }

    /// Moves the pedal, clamped to its travel.
    pub fn nudge_throttle(&mut self, delta: f64) {
        self.throttle_target = (self.throttle_target + delta).clamp(0.0, 1.0);
    }

    /// Advances the flywheel one frame from the block's solved torque.
    pub fn update(&mut self, block: &EngineBlock, dt: f64) {
        // Pedal travel. A step input would be both unrealistic and a parameter
        // discontinuity for the audio thread to chase.
        let slew = 1.0 - (-dt / 0.12).exp();
        self.throttle += (self.throttle_target - self.throttle) * slew;

        // Idle governor: enough throttle to hold the idle speed, and no more.
        // This is what stops the engine stalling the moment the pedal comes up.
        // The speed it holds is the engine's own, and a cold one is held faster.
        let target = self.idle_target(block);
        let governor = ((target + 60.0 - self.rpm) / 500.0).clamp(0.0, 0.30);
        let effective = self.throttle.max(governor);

        // The block solves torque for a speed, not for a throttle, so the pedal
        // is applied here. See the module docs.
        self.torque = block.mean_brake_torque(self.rpm);
        let drive = if self.cutting() {
            0.0
        } else {
            self.torque * (0.05 + 0.95 * effective)
        };

        // Accessories, then bearing drag, then windage, then the throttle
        // plate. Only the last of these depends on the pedal: it is the whole
        // of engine braking, and without it a lift from the limiter takes the
        // best part of a minute to come back to idle.
        let (a, b, c) = self.load;
        let omega = self.rpm * PI / 30.0;
        let load = a + b * omega + c * omega * omega + (1.0 - effective) * self.pumping;

        let alpha = (drive - load) / self.inertia.max(1e-3);
        let omega = (omega + alpha * dt).max(STALL_RPM * PI / 30.0);
        self.rpm = omega * 30.0 / PI;
    }

    /// What the audio path is being asked for this frame.
    pub fn controls(&self) -> EngineControls {
        EngineControls {
            throttle: self.throttle.clamp(0.0, 1.0),
            spark_cut: self.cutting(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Runs a preset at a fixed speed until its phase ring has filled.
    fn primed(preset: &EnginePreset, rpm: f64) -> EngineBlock {
        let mut block = preset.block(Environment::default());
        for _ in 0..900 {
            block.update(1.0 / 240.0, rpm);
        }
        block
    }

    #[test]
    fn every_preset_is_physically_sane() {
        for preset in EnginePreset::catalogue() {
            let displacement = preset.displacement() * 1e3;
            assert!(
                (0.5..8.0).contains(&displacement),
                "{}: implausible displacement {displacement:.2} L",
                preset.name
            );
            assert!(
                (5_000.0..9_000.0).contains(&preset.redline),
                "{}: redline {} leaves the dashboard's 0-9000 needle no travel",
                preset.name,
                preset.redline
            );
            assert!(
                preset.idle > STALL_RPM,
                "{}: idles below stall",
                preset.name
            );
            // A rod shorter than the crank radius has no slider-crank solution;
            // `CylinderGeometry::new` clamps it, so check the clamp was never
            // needed rather than that it worked.
            assert!(
                preset.model.geometry.rod_ratio() > 1.5,
                "{}: rod ratio {:.2} is not a real engine",
                preset.name,
                preset.model.geometry.rod_ratio()
            );
        }
    }

    #[test]
    fn every_preset_makes_torque_and_stays_finite() {
        for preset in EnginePreset::catalogue() {
            let block = primed(&preset, 4_000.0);
            let torque = block.mean_brake_torque(4_000.0);
            assert!(
                torque.is_finite() && torque > 0.0,
                "{} makes no torque at 4000 rpm: {torque:.1} N m",
                preset.name
            );
            let peak = block.ring.peak_pressure();
            assert!(
                (5e5..3e7).contains(&peak),
                "{}: implausible peak pressure {:.1} bar",
                preset.name,
                peak / 1e5
            );
        }
    }

    #[test]
    fn every_preset_free_revs_past_its_own_limiter() {
        // Reaching the limiter is not enough: the engine has to be able to
        // *overshoot* it, or it hovers just underneath, the cut chatters on and
        // off, and the limiter bounce — with the backfires that come with it —
        // never really happens. This measures the free-revving ceiling with the
        // limiter taken out, which is the headroom the brake load leaves.
        for preset in EnginePreset::catalogue() {
            let mut block = preset.block(Environment::default());
            let mut driveline = Driveline::new(&preset);
            driveline.redline = f64::INFINITY;
            driveline.throttle_target = 1.0;
            let dt = 1.0 / 240.0;

            for _ in 0..(240 * 25) {
                driveline.update(&block, dt);
                block.update(dt, driveline.rpm);
            }
            let margin = driveline.rpm - preset.redline;
            assert!(
                margin > 150.0,
                "{} tops out at {:.0} rpm, only {margin:.0} over its {:.0} rpm \
                 limiter — too little to bounce off it",
                preset.name,
                driveline.rpm,
                preset.redline,
            );
        }
    }

    #[test]
    fn every_preset_can_reach_its_own_limiter() {
        // The brake load is tuned per preset, and getting it wrong is silent:
        // the engine simply plateaus below the redline and the limiter, the
        // backfires and the whole top end of the dashboard become unreachable.
        for preset in EnginePreset::catalogue() {
            let mut block = preset.block(Environment::default());
            let mut driveline = Driveline::new(&preset);
            driveline.throttle_target = 1.0;
            let dt = 1.0 / 240.0;

            let mut reached = false;
            for _ in 0..(240 * 20) {
                driveline.update(&block, dt);
                block.update(dt, driveline.rpm);
                reached |= driveline.on_the_limiter();
            }
            assert!(
                reached,
                "{} never reached its {:.0} rpm limiter (stalled at {:.0})",
                preset.name, preset.redline, driveline.rpm
            );
        }
    }

    #[test]
    fn a_shut_throttle_falls_back_to_idle_without_stalling() {
        let preset = EnginePreset::cross_plane_v8();
        let mut block = preset.block(Environment::default());
        let mut driveline = Driveline::new(&preset);
        let dt = 1.0 / 240.0;

        driveline.throttle_target = 1.0;
        for _ in 0..(240 * 6) {
            driveline.update(&block, dt);
            block.update(dt, driveline.rpm);
        }
        assert!(driveline.rpm > 3_000.0, "never pulled away from idle");

        driveline.throttle_target = 0.0;
        for _ in 0..(240 * 15) {
            driveline.update(&block, dt);
            block.update(dt, driveline.rpm);
        }
        assert!(
            driveline.rpm > STALL_RPM,
            "the governor let it stall at {:.0} rpm",
            driveline.rpm
        );
        assert!(
            driveline.rpm < preset.idle + 400.0,
            "never came back down to idle: {:.0} rpm",
            driveline.rpm
        );
    }

    #[test]
    fn a_cold_engine_idles_above_a_warm_one_and_converges() {
        let preset = EnginePreset::cross_plane_v8();
        let dt = 1.0 / 120.0;

        let mut block = preset.block(Environment::default());
        block.cold_start();
        let mut driveline = Driveline::new(&preset);

        // Ten seconds of a cold start: the governor is holding a fast idle.
        let idle_for = |block: &mut EngineBlock, driveline: &mut Driveline, seconds: f64| {
            for _ in 0..(seconds / dt) as usize {
                driveline.update(block, dt);
                block.update(dt, driveline.rpm);
            }
            driveline.rpm
        };

        let cold = idle_for(&mut block, &mut driveline, 10.0);
        assert!(
            cold > preset.idle * 1.25,
            "a stone-cold engine idled at {cold:.0} rpm against a warm {:.0}",
            preset.idle
        );

        // And it comes back down on its own, on the block's temperature rather
        // than on a timer: no throttle input anywhere in this test.
        let warm = idle_for(&mut block, &mut driveline, 120.0);
        assert!(
            warm < cold,
            "the fast idle never came down: {warm:.0} rpm after two minutes"
        );
        assert!(
            (warm - preset.idle).abs() < 120.0,
            "converged to {warm:.0} rpm rather than the {:.0} rpm it idles at warm",
            preset.idle
        );
        assert!(
            driveline.throttle_target == 0.0,
            "the test drove the throttle instead of letting the governor do it"
        );
    }

    #[test]
    fn a_presets_shaft_and_its_voice_always_agree() {
        // The two halves of a turbo are wired up separately — one into the
        // snapshot source, one into the synth — and disagreeing is silent in
        // both directions: a shaft nobody listens to, or a voice with nothing
        // driving it, both of which just sound like a missing turbo.
        let mut turbocharged = 0;
        for preset in EnginePreset::catalogue() {
            let block = preset.block(Environment::default());
            let fitted = preset.is_turbocharged();
            assert_eq!(
                preset.snapshot_source(&block).has_turbo(),
                fitted,
                "{}: the shaft disagrees with the preset",
                preset.name
            );
            assert_eq!(
                preset.synth_config(&block, 48_000.0).turbo.is_some(),
                fitted,
                "{}: the voice disagrees with the preset",
                preset.name
            );
            turbocharged += usize::from(fitted);
        }
        // Both kinds have to be represented, or the distinction is untested in
        // every other test in this file.
        assert!(turbocharged > 0, "no turbocharged engine in the catalogue");
        assert!(
            turbocharged < EnginePreset::catalogue().len(),
            "every engine is turbocharged"
        );
    }

    #[test]
    fn the_wankel_breathes_like_a_2_6_litre_four_stroke() {
        // The mapping in `FiringOrder::two_rotor_wankel` is only meaningful if
        // the displacement per cycle comes out right; a chamber sized as though
        // it were a piston would quietly halve the engine.
        let wankel = EnginePreset::two_rotor_wankel();
        let chamber = wankel.model.geometry.displacement() * 1e6;
        assert!(
            (600.0..700.0).contains(&chamber),
            "chamber is {chamber:.0} cc, not a 654 cc rotor face"
        );
        assert!((2.5..2.7).contains(&(wankel.displacement() * 1e3)));
    }

    #[test]
    fn every_presets_derived_collector_reflection_differs_and_matches_area_ratio() {
        let catalogue = EnginePreset::catalogue();
        let mut reflections = Vec::new();

        for preset in &catalogue {
            let a_primary = preset.exhaust.primary_area();
            let a_outlet = preset.exhaust.collector.outlet_area;
            let expected = (a_primary - a_outlet) / (a_primary + a_outlet);
            let derived = preset.exhaust.collector_reflection();

            assert!(
                (derived - expected).abs() < 1e-12,
                "{}: derived reflection {derived} != expected {expected}",
                preset.name
            );
            assert!(
                derived < 0.0,
                "{}: primary expands into larger collector, reflection must invert phase",
                preset.name
            );
            reflections.push((preset.name, derived));
        }

        // Assert all presets derive distinct reflection coefficients.
        for i in 0..reflections.len() {
            for j in (i + 1)..reflections.len() {
                let (name_a, r_a) = reflections[i];
                let (name_b, r_b) = reflections[j];
                assert!(
                    (r_a - r_b).abs() > 1e-4,
                    "{name_a} and {name_b} share collector reflection {r_a:.4}"
                );
            }
        }
    }

    #[test]
    fn primary_quarter_wave_fundamental_lands_on_hot_gas() {
        // Hot exhaust gas: gamma = 1.33, R = 287 J/(kg K), T = 900 K
        let c = (1.33 * 287.0 * 900.0_f64).sqrt();

        for preset in EnginePreset::catalogue() {
            let length = preset.exhaust.primary_length();
            let expected = c / (4.0 * length);
            let derived = preset.exhaust.primary_quarter_wave_hz(c);

            assert!(
                (derived - expected).abs() < 1e-9,
                "{}: quarter-wave {derived} Hz != expected {expected} Hz",
                preset.name
            );
            // Sane fundamental frequency bounds for physical exhaust primaries (0.30 - 0.60 m)
            assert!(
                (200.0..600.0).contains(&derived),
                "{}: quarter-wave {derived} Hz is out of plausible range",
                preset.name
            );
        }
    }

    #[test]
    fn every_preset_is_cast_on_production_bore_centres() {
        use crate::audio::structure::MIN_BORE_WALL;

        for preset in EnginePreset::catalogue() {
            let bore = preset.model.geometry.bore;
            let ratio = preset.bore_spacing / bore;
            assert!(
                (1.05..=1.35).contains(&ratio),
                "{}: {:.3} bores between centres is not a block anyone casts",
                preset.name,
                ratio
            );
            // And the wall the bore-wall modes ring on is real metal, not the
            // model's floor: a spacing that had to be clamped is a spacing that
            // was not describing this engine.
            let wall = 0.5 * (preset.bore_spacing - bore);
            assert!(
                wall >= MIN_BORE_WALL as f64,
                "{}: {:.1} mm of wall between the bores",
                preset.name,
                wall * 1e3
            );
        }
    }

    #[test]
    fn no_two_presets_share_the_same_derived_acoustic_constants() {
        use crate::audio::filters::block_resonance_hz;

        let catalogue = EnginePreset::catalogue();
        let c = (1.33 * 287.0 * 900.0_f64).sqrt();

        for i in 0..catalogue.len() {
            for j in (i + 1)..catalogue.len() {
                let a = &catalogue[i];
                let b = &catalogue[j];

                let r_a = a.exhaust.collector_reflection();
                let r_b = b.exhaust.collector_reflection();
                let f_qw_a = a.exhaust.primary_quarter_wave_hz(c);
                let f_qw_b = b.exhaust.primary_quarter_wave_hz(c);
                let tau_rt_a = a.exhaust.primary_round_trip_seconds(c);
                let tau_rt_b = b.exhaust.primary_round_trip_seconds(c);
                let f_block_a = block_resonance_hz(a.block_mass as f32);
                let f_block_b = block_resonance_hz(b.block_mass as f32);

                let identical = (r_a - r_b).abs() < 1e-6
                    && (f_qw_a - f_qw_b).abs() < 1e-3
                    && (tau_rt_a - tau_rt_b).abs() < 1e-6
                    && (f_block_a - f_block_b).abs() < 1e-3;

                assert!(
                    !identical,
                    "regression: {} and {} share identical derived acoustics",
                    a.name, b.name
                );
            }
        }
    }

    #[test]
    fn physics_pipe_and_audio_path_round_trip_times_agree() {
        use crate::audio::dsp::EngineSynth;
        use crate::audio::filters::round_trip_seconds;

        let fs = 48_000.0;
        let env = Environment::default();

        for preset in EnginePreset::catalogue() {
            let block = preset.block(env);
            let config = preset.synth_config(&block, fs);
            let synth = EngineSynth::new(config);

            let gamma = block.model.gas.gamma_burned;
            let gas_constant = block.model.gas.r_burned;
            let temperature = 900.0;
            let c = (gamma * gas_constant * temperature).sqrt();

            for (bank_idx, manifold) in block.exhaust_banks.iter().enumerate() {
                let phys_round_trip = manifold.pipe.transit_time(c);
                let runner_length = preset
                    .exhaust
                    .primary_length_for_cylinders(&block.firing.cylinders_on_bank(bank_idx as u8));

                // Direct analytical calculation in audio filters:
                let audio_filter_round_trip = round_trip_seconds(
                    runner_length as f32,
                    gamma as f32,
                    gas_constant as f32,
                    temperature as f32,
                ) as f64;

                let diff_filter_samples =
                    (phys_round_trip - audio_filter_round_trip).abs() * fs as f64;
                assert!(
                    diff_filter_samples < 1.0,
                    "{}, bank {}: physics round trip ({:.6} s) and audio filter ({:.6} s) disagree by {:.3} samples",
                    preset.name,
                    bank_idx,
                    phys_round_trip,
                    audio_filter_round_trip,
                    diff_filter_samples
                );

                // Initialized audio synth runner agreement at default ambient
                // conditions. No end correction here: a primary in the network
                // stops at the collector junction, which scatters off an area
                // ratio, and only a pipe stopping in open air is lengthened by
                // the slug of gas outside its mouth.
                let c_ambient = (1.33 * 287.0 * 300.0_f64).sqrt();
                let phys_ambient_round_trip = 2.0 * runner_length / c_ambient;
                let audio_synth_round_trip = synth.runner_round_trip_seconds(bank_idx) as f64;
                let diff_synth_samples =
                    (phys_ambient_round_trip - audio_synth_round_trip).abs() * fs as f64;
                assert!(
                    diff_synth_samples < 1.0,
                    "{}, bank {}: physics round trip ({:.6} s) and synth runner ({:.6} s) disagree by {:.3} samples",
                    preset.name,
                    bank_idx,
                    phys_ambient_round_trip,
                    audio_synth_round_trip,
                    diff_synth_samples
                );
            }
        }
    }

    #[test]
    fn every_preset_has_valid_aperture_positions() {
        for preset in EnginePreset::catalogue() {
            assert!(
                !preset.aperture_positions.tailpipes.is_empty(),
                "{}: has no tailpipes configured",
                preset.name
            );
            for (i, tp) in preset.aperture_positions.tailpipes.iter().enumerate() {
                assert!(
                    tp[2] > 0.0,
                    "{}: tailpipe {i} height <= 0 ({:?})",
                    preset.name,
                    tp
                );
            }
            assert!(
                preset.aperture_positions.intake[2] > 0.0,
                "{}: intake height <= 0 ({:?})",
                preset.name,
                preset.aperture_positions.intake
            );
            assert!(
                preset.aperture_positions.block[2] > 0.0,
                "{}: block height <= 0 ({:?})",
                preset.name,
                preset.aperture_positions.block
            );
        }
    }
}
