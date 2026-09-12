//! 1D digital waveguide network for engine intake and induction acoustics.
//!
//! # Intake acoustics
//!
//! Sound in the induction tract travels through digital waveguides:
//! bidirectional delay lines carrying forward-travelling pressure waves $p^+$
//! (toward the atmosphere / mouth) and backward-travelling pressure waves $p^-$
//! (toward the intake valves).
//!
//! Topology:
//! ```text
//! intake valve -> port -> runner -> plenum junction (N runners) -> throttle
//!     -> airbox -> snorkel -> mouth radiation -> listener
//! ```
//!
//! For engines fitted with individual throttle bodies (ITBs), the runners vent
//! directly to atmosphere through velocity stack mouths with no shared plenum.
//!
//! Processing runs in `f32` at the audio sample rate with zero allocation
//! in the audio callback.

use crate::audio::filters::speed_of_sound;
use crate::audio::radiation::Mouth;
use crate::audio::waveguide::{ScatteringJunction, ValveTermination, WaveguidePipe, SMOOTH_WALL};
use crate::physics::plumbing::{IntakeSystem, ThrottleLayout};

/// Reference ambient temperature for intake air [K].
pub const INTAKE_AMBIENT_TEMPERATURE_K: f32 = 300.0;

/// Specific heat ratio of fresh air [-].
pub const INTAKE_AIR_GAMMA: f32 = 1.40;

/// Specific gas constant of fresh air [J/(kg K)].
pub const INTAKE_GAS_CONSTANT: f32 = 287.0;

// ---------------------------------------------------------------------------
// The air filter
// ---------------------------------------------------------------------------

/// Specific airflow resistance of automotive filter medium [Pa s / m].
///
/// The quantity a resistive sheet is characterised by: the pressure it drops
/// per unit of velocity *through the medium*. Cellulose engine-intake paper is
/// a dense one, several hundred to a couple of thousand rayls; acoustic
/// resistive screens, made for the job, sit at the bottom of that range.
///
/// It is a viscous resistance, so it is flat with frequency, and that is the
/// point of it: unlike every other element in the tract it dissipates rather
/// than reflects, and it does so at every frequency the tract can ring at.
pub const FILTER_MEDIUM_RAYLS: f32 = 1_000.0;

/// How much medium a pleated element folds into its face area [-].
///
/// A 40 mm deep pleat on a 10 mm pitch is eight times the face, and the depth
/// is what an airbox is mostly made of. It divides the medium's resistance,
/// because the acoustic velocity through the paper is lower than the velocity
/// in the duct by the same factor — which is exactly why a filter can drop so
/// little pressure and still be a filter.
pub const FILTER_PLEAT_RATIO: f32 = 10.0;

/// Resistance an element presents across the duct it sits in, normalised [-].
///
/// $\zeta = \frac{R_s}{n \rho c}$, the medium's resistance referred to the
/// duct and divided by the characteristic impedance of the air in it.
#[inline]
pub fn filter_resistance(temperature: f32, gas_constant: f32, speed_of_sound: f32) -> f32 {
    let rho = 101_325.0 / (gas_constant.max(1.0) * temperature.max(1.0));
    FILTER_MEDIUM_RAYLS / (FILTER_PLEAT_RATIO * rho * speed_of_sound).max(1.0)
}

/// A resistive sheet across a duct: the air filter element.
///
/// # What it is for
///
/// Everything else in an intake tract is reactive. A runner reflects, a plenum
/// reflects, a snorkel mouth reflects almost everything below its radiation
/// corner, and the junctions between them conserve energy exactly. Put those
/// together with a valve that is shut for three quarters of the cycle and the
/// tract is very nearly a lossless resonator — which is audible as a hollow
/// midrange comb sitting over the whole engine, and is not what an induction
/// system sounds like, because every induction system has a filter in it.
///
/// # The element
///
/// A sheet of specific flow resistance $R_s$ drops pressure in proportion to
/// the velocity through it and passes that velocity unchanged:
///
/// ```text
/// p1 - p2 = R_s u,        u1 = u2
/// ```
///
/// In travelling waves, with $\zeta = R_s / \rho c$, that is a symmetric
/// two-port
///
/// ```text
/// p1- = r p1+ + t p2-
/// p2+ = t p1+ + r p2-
/// r = zeta / (2 + zeta),   t = 2 / (2 + zeta)
/// ```
///
/// which is transparent at $\zeta = 0$ and a rigid wall as $\zeta \to \infty$.
/// It is not lossless at either end of that range but the middle: `r + t = 1`
/// in pressure, so `r^2 + t^2 < 1` in power, and what is missing went into the
/// paper as heat. That is the whole reason it is here.
#[derive(Debug, Clone, Copy, Default)]
pub struct ResistiveSheet {
    reflection: f32,
    transmission: f32,
}

impl ResistiveSheet {
    /// A sheet of normalised resistance `zeta`.
    pub fn new(zeta: f32) -> Self {
        let z = zeta.max(0.0);
        Self {
            reflection: z / (2.0 + z),
            transmission: 2.0 / (2.0 + z),
        }
    }

    /// Retunes for the current gas.
    pub fn tune(&mut self, temperature: f32, gas_constant: f32, speed_of_sound: f32) {
        *self = Self::new(filter_resistance(temperature, gas_constant, speed_of_sound));
    }

    /// Fraction of an incident wave sent back [-].
    pub fn reflection(&self) -> f32 {
        self.reflection
    }

    /// Fraction of an incident wave passed through [-].
    pub fn transmission(&self) -> f32 {
        self.transmission
    }

    /// Scatters the two waves arriving at the sheet.
    ///
    /// Given `p_in_plus` arriving from the engine side and `p_out_minus`
    /// arriving from the atmosphere side, returns `(p_in_minus, p_out_plus)`:
    /// what goes back towards the engine, and what continues outwards.
    #[inline(always)]
    pub fn scatter(&self, p_in_plus: f32, p_out_minus: f32) -> (f32, f32) {
        (
            self.reflection * p_in_plus + self.transmission * p_out_minus,
            self.transmission * p_in_plus + self.reflection * p_out_minus,
        )
    }
}

/// Induction rarefaction acoustic pressure pulse at valve opening [Pa]:
///
/// $$p' = -\rho c u = -\frac{c \dot{m}}{A}$$
///
/// As the piston descends with the intake valve open, it pulls air into the cylinder,
/// launching a negative pressure (rarefaction) wave up the runner toward the mouth.
#[inline]
pub fn induction_rarefaction_pa(mass_flow: f32, runner_area: f32, speed_of_sound: f32) -> f32 {
    if runner_area <= 1e-7 {
        0.0
    } else {
        -speed_of_sound * mass_flow.max(0.0) / runner_area
    }
}

/// Acoustic water hammer pressure pulse from intake valve closing slam [Pa]:
///
/// $$\Delta p_{\text{slam}} = \frac{L}{A} \max\left(0, -\frac{d\dot{m}}{dt}\right)$$
///
/// At intake valve closing (IVC), the inertia of the moving air column in the runner
/// slams into the shutting valve, converting kinetic energy into a sharp positive
/// pressure wavefront that travels up the runner and radiates from the mouth.
#[inline]
pub fn valve_slam_pa(d_mdot_dt: f32, runner_length: f32, runner_area: f32) -> f32 {
    if runner_area <= 1e-7 {
        0.0
    } else {
        (runner_length.max(0.0) / runner_area) * (-d_mdot_dt).max(0.0)
    }
}

/// Dipole orifice noise amplitude scaling from normalized flow velocity [-]:
///
/// $$p_{\text{rms}} \propto u^3$$
///
/// Aeroacoustic dipole sound generated by turbulent flow through an orifice radiates
/// acoustic power scaling with the sixth power of velocity ($W \propto u^6$).
/// The corresponding acoustic pressure amplitude scales with the cube of velocity ($u^3$).
#[inline]
pub fn dipole_orifice_amplitude(flow_ratio: f32) -> f32 {
    let u = flow_ratio.max(0.0);
    u * u * u
}

/// Calculates the geometric open area of a throttle body [m^2] from pedal position `0..=1`.
///
/// Follows the physical throttle plate geometry:
/// $$A_{\text{open}}(\theta) = A_{\text{bore}} \cdot \left[1 - \cos\left(\theta \frac{\pi}{2}\right)\right] + A_{\text{leak}}$$
/// where $A_{\text{leak}} = 0.008 \cdot A_{\text{bore}}$ represents idle bypass and edge leakage.
#[inline]
pub fn throttle_area(bore_diameter: f64, throttle_position: f32) -> f32 {
    let t = (throttle_position as f64).clamp(0.0, 1.0);
    let radius = bore_diameter.max(1e-4) * 0.5;
    let bore_area = std::f64::consts::PI * radius * radius;
    let leak_area = bore_area * 0.008;
    let swept_area = bore_area * (1.0 - (t * std::f64::consts::FRAC_PI_2).cos());
    ((swept_area + leak_area).min(bore_area)) as f32
}

/// Computes acoustic scattering across a throttle area restriction $A_{\text{th}}$.
///
/// Models an orifice of area $A_{\text{th}}$ connecting duct 1 ($A_1$) and duct 2 ($A_2$).
/// Acoustic series resistance of the restriction:
///
/// $$Z_{\text{rest}} = \max\left(0, \frac{1}{A_{\text{th}}} - \frac{1}{\min(A_1, A_2)}\right)$$
///
/// Acoustic volume velocity through the restriction:
///
/// $$U = \frac{2 (p_1^+ - p_2^+)}{\frac{1}{A_1} + \frac{1}{A_2} + 2 Z_{\text{rest}}}$$
///
/// Scattered waves returning into each duct:
///
/// $$p_1^- = p_1^+ - \frac{U}{A_1}$$
/// $$p_2^- = p_2^+ + \frac{U}{A_2}$$
///
/// When the throttle is shut ($A_{\text{th}} \approx A_{\text{leak}} \to 0$), $Z_{\text{rest}} \to \infty$,
/// driving $U \to 0$, $p_1^- \to p_1^+$, $p_2^- \to p_2^+$ (rigid reflections, transmission vanishes).
/// When wide open ($A_{\text{th}} \ge \min(A_1, A_2)$), $Z_{\text{rest}} = 0$, giving unimpeded transmission.
#[inline]
pub fn scatter_throttle_restriction(
    p1_plus: f32,
    p2_plus: f32,
    area1: f32,
    area2: f32,
    throttle_area: f32,
) -> (f32, f32) {
    let a1 = area1.max(1e-7);
    let a2 = area2.max(1e-7);
    let a_th = throttle_area.max(1e-9);
    let min_a = a1.min(a2);
    let z_rest = (1.0 / a_th - 1.0 / min_a).max(0.0);
    let denom = (1.0 / a1) + (1.0 / a2) + 2.0 * z_rest;
    let u = (2.0 * (p1_plus - p2_plus)) / denom;
    let p1_minus = p1_plus - (u / a1);
    let p2_minus = p2_plus + (u / a2);
    (p1_minus, p2_minus)
}

/// 1D digital waveguide network representing the complete intake system.
#[derive(Debug, Clone)]
pub struct IntakeNetwork {
    /// Intake runners (one per cylinder).
    runners: Vec<WaveguidePipe>,
    /// Valve boundary conditions at the cylinder ports.
    valves: Vec<ValveTermination>,
    /// Junction where all runners meet the central plenum cavity.
    plenum_junction: Option<ScatteringJunction>,
    /// Central plenum cavity duct.
    plenum_pipe: Option<WaveguidePipe>,
    /// Upstream airbox duct, if fitted.
    airbox_pipe: Option<WaveguidePipe>,
    /// The element in that airbox. A box has a filter in it; that is what a box
    /// is for, and it is the only thing in the tract that dissipates.
    filter: Option<ResistiveSheet>,
    /// Air inlet snorkel duct, if fitted.
    snorkel_pipe: Option<WaveguidePipe>,
    /// Mouth radiation termination for single-throttle systems.
    single_mouth: Option<Mouth>,
    /// Mouth radiation terminations for ITB systems (one per runner).
    itb_mouths: Vec<Mouth>,

    // Preallocated buffers for real-time processing
    runner_in_0: Vec<f32>,
    runner_to_plenum: Vec<f32>,
    plenum_scatter_in: Vec<f32>,
    plenum_scatter_out: Vec<f32>,
    plenum_to_downstream: f32,
    downstream_to_plenum: f32,
    itb_mouth_refl: Vec<f32>,

    /// Number of cylinders.
    cylinder_count: usize,
    /// Sample rate [Hz].
    sample_rate: f32,
    /// Current throttle position, `0..=1` [-].
    throttle: f32,
    /// Throttle bore diameter [m].
    throttle_bore: f64,
    /// Plenum cavity cross-sectional area [m^2].
    plenum_area: f32,
    /// Downstream duct cross-sectional area [m^2].
    downstream_area: f32,
    /// Throttle arrangement and geometry.
    layout: ThrottleLayout,
}

impl IntakeNetwork {
    /// Constructs a new intake waveguide network from an [`IntakeSystem`] description.
    pub fn new(intake: &IntakeSystem, cylinder_count: usize, sample_rate: f32) -> Self {
        let n_cyl = cylinder_count.max(1);
        let gamma = INTAKE_AIR_GAMMA;
        let r = INTAKE_GAS_CONSTANT;
        let temp = INTAKE_AMBIENT_TEMPERATURE_K;
        let c = speed_of_sound(gamma, r, temp);

        let is_itb = matches!(intake.throttle, ThrottleLayout::IndividualBodies { .. });

        // 1. Runners and valve terminations
        let mut runners = Vec::with_capacity(n_cyl);
        let mut valves = Vec::with_capacity(n_cyl);
        let mut itb_mouths = Vec::with_capacity(if is_itb { n_cyl } else { 0 });

        for i in 0..n_cyl {
            let spec = if !intake.runners.is_empty() {
                intake.runners[i % intake.runners.len()]
            } else {
                crate::physics::plumbing::PipeSection::from_diameter(0.30, 0.042, 310.0)
            };

            if is_itb {
                let radius = (spec.area / std::f64::consts::PI).sqrt() as f32;
                let mouth = Mouth::new(sample_rate, radius, intake.trumpet_flanged, c);
                let eff_len = spec.length + mouth.end_correction() as f64;
                let mut pipe = WaveguidePipe::new(eff_len, spec.area, sample_rate, gamma, r, temp);
                // Cast, cold, and drawing a column rather than venting a jet:
                // the textbook boundary layer, not the exhaust's rough one.
                pipe.set_wall_enhancement(SMOOTH_WALL);
                pipe.set_boundary_phase_delay(mouth.phase_delay_samples());
                runners.push(pipe);
                itb_mouths.push(mouth);
            } else {
                let mut pipe =
                    WaveguidePipe::new(spec.length, spec.area, sample_rate, gamma, r, temp);
                pipe.set_wall_enhancement(SMOOTH_WALL);
                runners.push(pipe);
            }
            valves.push(ValveTermination::new(spec.area));
        }

        // 2. Single throttle path (plenum, airbox, snorkel, mouth)
        let (plenum_junction, plenum_pipe, airbox_pipe, filter, snorkel_pipe, single_mouth) =
            if is_itb {
                (None, None, None, None, None, None)
            } else {
                // Plenum cavity: length 0.20 m, area = volume / length
                let plenum_len = 0.20f64;
                let plenum_area = (intake.plenum_volume / plenum_len).max(1e-5);
                let mut plenum_pipe =
                    WaveguidePipe::new(plenum_len, plenum_area, sample_rate, gamma, r, temp);
                plenum_pipe.set_wall_enhancement(SMOOTH_WALL);

                // Scattering junction: N runners + 1 plenum pipe
                let mut junction_areas = Vec::with_capacity(n_cyl + 1);
                for r in &runners {
                    junction_areas.push(r.area() as f64);
                }
                junction_areas.push(plenum_area);
                let plenum_junction = ScatteringJunction::from_areas(&junction_areas);

                // Airbox
                let filter = intake
                    .airbox
                    .map(|_| ResistiveSheet::new(filter_resistance(temp, r, c)));
                let airbox_pipe = intake.airbox.map(|box_spec| {
                    let mut pipe = WaveguidePipe::new(
                        box_spec.length,
                        box_spec.area,
                        sample_rate,
                        gamma,
                        r,
                        temp,
                    );
                    pipe.set_wall_enhancement(SMOOTH_WALL);
                    pipe
                });

                // Snorkel
                let snorkel_pipe = intake.snorkel.map(|snork_spec| {
                    let mut pipe = WaveguidePipe::new(
                        snork_spec.length,
                        snork_spec.area,
                        sample_rate,
                        gamma,
                        r,
                        temp,
                    );
                    pipe.set_wall_enhancement(SMOOTH_WALL);
                    pipe
                });

                // Exit mouth
                let exit_spec = intake.snorkel.or(intake.airbox).unwrap_or_else(|| {
                    crate::physics::plumbing::PipeSection::from_diameter(0.20, 0.070, 300.0)
                });
                let mouth_radius = (exit_spec.area / std::f64::consts::PI).sqrt() as f32;
                let mouth = Mouth::new(sample_rate, mouth_radius, intake.trumpet_flanged, c);

                (
                    Some(plenum_junction),
                    Some(plenum_pipe),
                    airbox_pipe,
                    filter,
                    snorkel_pipe,
                    Some(mouth),
                )
            };

        let runner_in_0 = vec![0.0; n_cyl];
        let runner_to_plenum = vec![0.0; n_cyl];
        let plenum_scatter_in = vec![0.0; n_cyl + 1];
        let plenum_scatter_out = vec![0.0; n_cyl + 1];
        let itb_mouth_refl = vec![0.0; if is_itb { n_cyl } else { 0 }];

        let throttle_bore = match intake.throttle {
            ThrottleLayout::Single { bore } => bore,
            ThrottleLayout::IndividualBodies { bore } => bore,
        };
        let plenum_len = 0.20f64;
        let plenum_area = if is_itb {
            0.0
        } else {
            (intake.plenum_volume / plenum_len).max(1e-5) as f32
        };
        let downstream_area = if is_itb {
            0.0
        } else {
            intake
                .airbox
                .or(intake.snorkel)
                .map(|s| s.area as f32)
                .unwrap_or(0.00385)
        };

        Self {
            runners,
            valves,
            plenum_junction,
            plenum_pipe,
            airbox_pipe,
            filter,
            snorkel_pipe,
            single_mouth,
            itb_mouths,
            runner_in_0,
            runner_to_plenum,
            plenum_scatter_in,
            plenum_scatter_out,
            plenum_to_downstream: 0.0,
            downstream_to_plenum: 0.0,
            itb_mouth_refl,
            cylinder_count: n_cyl,
            sample_rate,
            throttle: 0.0,
            throttle_bore,
            plenum_area,
            downstream_area,
            layout: intake.throttle,
        }
    }

    /// Retunes propagation delays and losses for current gas state.
    pub fn tune(&mut self, gamma: f32, gas_constant: f32, temperature: f32) {
        let c = speed_of_sound(gamma, gas_constant, temperature);
        for p in &mut self.runners {
            p.tune(gamma, gas_constant, temperature);
        }
        if let Some(p) = &mut self.plenum_pipe {
            p.tune(gamma, gas_constant, temperature);
        }
        if let Some(p) = &mut self.airbox_pipe {
            p.tune(gamma, gas_constant, temperature);
        }
        if let Some(p) = &mut self.snorkel_pipe {
            p.tune(gamma, gas_constant, temperature);
        }
        if let Some(m) = &mut self.single_mouth {
            m.tune(c);
        }
        for m in &mut self.itb_mouths {
            m.tune(c);
        }
    }

    /// Updates valve effective flow areas for all cylinders.
    pub fn set_valve_areas(&mut self, valve_areas: &[f64]) {
        for (v, &area) in self.valves.iter_mut().zip(valve_areas.iter()) {
            v.set_effective_area(area);
        }
    }

    /// Updates the throttle position, `0.0..=1.0` [-].
    pub fn set_throttle(&mut self, throttle: f32) {
        self.throttle = throttle.clamp(0.0, 1.0);
    }

    /// Number of cylinders in this intake system.
    pub fn cylinder_count(&self) -> usize {
        self.cylinder_count
    }

    /// Audio sample rate [Hz].
    pub fn sample_rate(&self) -> f32 {
        self.sample_rate
    }

    /// Throttle arrangement and geometry.
    pub fn layout(&self) -> &ThrottleLayout {
        &self.layout
    }

    /// Steps the intake waveguide network by one audio sample.
    ///
    /// - `excitations`: slice of acoustic pressure excitations per cylinder [Pa].
    ///
    /// Returns the radiated acoustic pressure [Pa].
    #[inline(always)]
    pub fn step(&mut self, excitations: &[f32]) -> f32 {
        let n_cyl = self.runners.len();

        if !self.itb_mouths.is_empty() {
            // Individual throttle bodies (ITBs)
            let a_th = throttle_area(self.throttle_bore, self.throttle);
            let mut total_rad = 0.0f32;
            for i in 0..n_cyl {
                let (p_at_valve, p_at_mouth) = self.runners[i].read_outputs();
                let excit = if i < excitations.len() {
                    excitations[i]
                } else {
                    0.0
                };
                let p_in_valve = self.valves[i].step(excit, p_at_valve);
                let runner_area = self.runners[i].area();
                let prev_refl = self.itb_mouth_refl[i];
                let (p_back_to_runner, p_to_mouth) = scatter_throttle_restriction(
                    p_at_mouth,
                    prev_refl,
                    runner_area,
                    runner_area,
                    a_th,
                );
                let (new_refl, p_rad) = self.itb_mouths[i].step(p_to_mouth);
                self.itb_mouth_refl[i] = new_refl;
                self.runners[i].push_inputs(p_in_valve, p_back_to_runner);
                total_rad += p_rad;
            }
            total_rad / (n_cyl as f32).sqrt().max(1.0)
        } else {
            // Single throttle with plenum
            // 1. Read runner outputs at valve and plenum boundaries
            for i in 0..n_cyl {
                let (p_at_valve, p_at_plenum) = self.runners[i].read_outputs();
                let excit = if i < excitations.len() {
                    excitations[i]
                } else {
                    0.0
                };
                self.runner_in_0[i] = self.valves[i].step(excit, p_at_valve);
                self.runner_to_plenum[i] = p_at_plenum;
            }

            // 2. Read plenum cavity outputs
            let (p_plenum_at_runners, p_plenum_at_exit) = match &mut self.plenum_pipe {
                Some(p) => p.read_outputs(),
                None => (0.0, 0.0),
            };

            // 3. Scatter at plenum junction: N runners + plenum cavity
            if let Some(junc) = &self.plenum_junction {
                for i in 0..n_cyl {
                    self.plenum_scatter_in[i] = self.runner_to_plenum[i];
                }
                self.plenum_scatter_in[n_cyl] = p_plenum_at_runners;

                junc.scatter(&self.plenum_scatter_in, &mut self.plenum_scatter_out);

                for i in 0..n_cyl {
                    let p_back_to_runner = self.plenum_scatter_out[i];
                    self.runners[i].push_inputs(self.runner_in_0[i], p_back_to_runner);
                }
            }

            // 4. Scatter through throttle restriction between plenum exit and downstream duct
            let a_th = throttle_area(self.throttle_bore, self.throttle);
            let (p_back_to_plenum, p_to_downstream) = scatter_throttle_restriction(
                p_plenum_at_exit,
                self.downstream_to_plenum,
                self.plenum_area,
                self.downstream_area,
                a_th,
            );
            self.plenum_to_downstream = p_to_downstream;

            // 5. Downstream chain: filter -> airbox -> snorkel -> mouth.
            //
            // Every element's downstream input is its neighbour's upstream
            // output *from this sample*, which is why both ducts are read
            // before either is pushed. Handing each of them one shared value
            // instead — which is what this did — feeds the snorkel its own
            // backward wave and leaves the mouth's reflection with nowhere to
            // go, so the one end of the tract that is open to the world never
            // reaches the rest of it.
            let (box_up, box_down) = match &mut self.airbox_pipe {
                Some(p) => p.read_outputs(),
                None => (0.0, 0.0),
            };
            let (snorkel_up, snorkel_down) = match &mut self.snorkel_pipe {
                Some(p) => p.read_outputs(),
                None => (0.0, 0.0),
            };

            // The mouth is fed by whatever duct is last in the chain, and with
            // no ducts at all by the throttle itself.
            let to_mouth = if self.snorkel_pipe.is_some() {
                snorkel_down
            } else if self.airbox_pipe.is_some() {
                box_down
            } else {
                p_to_downstream
            };
            let (mouth_reflection, radiated) = match &mut self.single_mouth {
                Some(mouth) => mouth.step(to_mouth),
                None => (0.0, 0.0),
            };

            // Walking back up: what waits on the far side of each interface.
            let past_airbox = if self.snorkel_pipe.is_some() {
                snorkel_up
            } else {
                mouth_reflection
            };
            let past_filter = if self.airbox_pipe.is_some() {
                box_up
            } else {
                past_airbox
            };

            // The element sits on the clean side of the box, so it is the first
            // thing a wave leaving the throttle meets on its way out and the
            // last thing a wave from the mouth meets on its way in — which is
            // why the tract pays for it twice per round trip.
            let (back_to_plenum, into_airbox) = match &self.filter {
                Some(filter) => filter.scatter(p_to_downstream, past_filter),
                None => (past_filter, p_to_downstream),
            };
            self.downstream_to_plenum = back_to_plenum;

            if let Some(airbox) = &mut self.airbox_pipe {
                airbox.push_inputs(into_airbox, past_airbox);
            }
            if let Some(snorkel) = &mut self.snorkel_pipe {
                let into_snorkel = if self.airbox_pipe.is_some() {
                    box_down
                } else {
                    into_airbox
                };
                snorkel.push_inputs(into_snorkel, mouth_reflection);
            }

            let p_into_plenum_0 = self.plenum_scatter_out[n_cyl];
            if let Some(plenum) = &mut self.plenum_pipe {
                plenum.push_inputs(p_into_plenum_0, p_back_to_plenum);
            }

            radiated
        }
    }

    /// Whether an air filter element is fitted.
    pub fn has_filter(&self) -> bool {
        self.filter.is_some()
    }

    /// Resets all internal delay lines and filters.
    pub fn reset(&mut self) {
        for r in &mut self.runners {
            r.reset();
        }
        for v in &mut self.valves {
            v.set_effective_area(0.0);
        }
        if let Some(p) = &mut self.plenum_pipe {
            p.reset();
        }
        if let Some(p) = &mut self.airbox_pipe {
            p.reset();
        }
        if let Some(p) = &mut self.snorkel_pipe {
            p.reset();
        }
        if let Some(m) = &mut self.single_mouth {
            m.reset();
        }
        for m in &mut self.itb_mouths {
            m.reset();
        }
        self.runner_in_0.fill(0.0);
        self.runner_to_plenum.fill(0.0);
        self.plenum_scatter_in.fill(0.0);
        self.plenum_scatter_out.fill(0.0);
        self.plenum_to_downstream = 0.0;
        self.downstream_to_plenum = 0.0;
        self.itb_mouth_refl.fill(0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn magnitude_at(signal: &[f32], frequency: f32, sample_rate: f32) -> f32 {
        let (mut re, mut im) = (0.0f64, 0.0f64);
        for (i, &x) in signal.iter().enumerate() {
            let phase = std::f32::consts::TAU * frequency * i as f32 / sample_rate;
            re += x as f64 * phase.sin() as f64;
            im += x as f64 * phase.cos() as f64;
        }
        (2.0 * (re * re + im * im).sqrt() / signal.len() as f64) as f32
    }

    #[test]
    fn intake_note_is_pitched_at_firing_order() {
        // An engine's intake does not hiss: the cylinders draw sequentially,
        // and the resulting rarefactions and valve closures produce a strongly
        // pitched sound whose fundamental lands at the engine firing order
        // f_firing = (N_cyl / 2) * (rpm / 60) = (N_cyl / 120) * rpm.
        let fs = 48_000.0f32;
        let system = IntakeSystem::default_for_cylinders(4);
        let mut network = IntakeNetwork::new(&system, 4, fs);
        network.set_throttle(0.8);

        // 3000 rpm -> 50 rev/sec. For a 4-stroke 4-cylinder, firing order is 2nd order (100 Hz).
        let rpm = 3_000.0f32;
        let f_crank = rpm / 60.0;
        let f_firing = 4.0 / 2.0 * f_crank; // 100 Hz
        let period_samples = (fs / f_firing) as usize; // 480 samples per firing

        let total_samples = 48_000;
        let mut out = Vec::with_capacity(total_samples);

        for n in 0..total_samples {
            let mut excits = [0.0f32; 4];
            let cyl = (n / period_samples) % 4;
            let phase_in_event = n % period_samples;
            if phase_in_event < 120 {
                let flow = 0.05 * (std::f32::consts::PI * phase_in_event as f32 / 120.0).sin();
                let d_flow = if phase_in_event > 100 {
                    -0.05 * fs / 20.0
                } else {
                    0.0
                };
                excits[cyl] = induction_rarefaction_pa(flow, 0.0014, 343.0)
                    + valve_slam_pa(d_flow, 0.25, 0.0014);
            }
            out.push(network.step(&excits));
        }

        let at_firing = magnitude_at(&out[4_800..], f_firing, fs);
        let off1 = magnitude_at(&out[4_800..], f_firing * 0.61, fs);
        let off2 = magnitude_at(&out[4_800..], f_firing * 1.43, fs);
        let off = 0.5 * (off1 + off2);

        assert!(
            at_firing > 4.0 * off,
            "intake note must be pitched at firing order (100 Hz): firing={at_firing:.3e} vs off={off:.3e}"
        );
    }

    #[test]
    fn plenum_ram_peak_lands_at_the_helmholtz_frequency() {
        // In engine acoustics, plenum ram tuning occurs when the intake runner
        // (acting as an acoustic inertance M = rho * L / A) resonates against
        // the plenum chamber volume (acting as an acoustic compliance C = V / (rho * c^2)).
        // During induction with the intake valve open and the throttle closing or restricting
        // the cavity, this Helmholtz resonator mode peaks at:
        //   f_H = (c / 2*pi) * sqrt(A_runner / (V_plenum * L_runner))
        //
        // Here we construct an IntakeNetwork with known geometry, open the valve
        // termination (effective area > pipe area, r -> -1), shut the throttle
        // (rigid cavity termination), inject an acoustic impulse, and measure the
        // resonant frequency inside the runner.
        let runner_len = 0.25f64;
        let runner_diam = 0.042f64;
        let runner_area = std::f64::consts::PI * 0.25 * runner_diam * runner_diam;
        let plenum_vol = 0.0035f64; // 3.5 L
        let runners = vec![crate::physics::PipeSection::new(
            runner_len,
            runner_area,
            300.0,
        )];
        let system = IntakeSystem {
            runners,
            plenum_volume: plenum_vol,
            throttle: ThrottleLayout::Single { bore: 0.065 },
            airbox: None,
            snorkel: None,
            trumpet_flanged: false,
        };
        let fs = 48_000.0;
        let mut net = IntakeNetwork::new(&system, 1, fs);
        net.set_throttle(0.0);
        net.set_valve_areas(&[runner_area * 10.0]);

        let c = speed_of_sound(
            INTAKE_AIR_GAMMA,
            INTAKE_GAS_CONSTANT,
            INTAKE_AMBIENT_TEMPERATURE_K,
        );
        let f_helmholtz = (c / std::f32::consts::TAU)
            * ((runner_area as f32) / (plenum_vol as f32 * runner_len as f32)).sqrt();

        let mut runner_waves = Vec::with_capacity(48_000);
        for i in 0..48_000 {
            let excit = if i == 0 { 1_000.0 } else { 0.0 };
            net.step(&[excit]);
            runner_waves.push(net.runner_to_plenum[0]);
        }

        let mut max_mag = 0.0f32;
        let mut peak_f = 0.0f32;
        for f in 40..110 {
            let mag = magnitude_at(&runner_waves, f as f32, fs);
            if mag > max_mag {
                max_mag = mag;
                peak_f = f as f32;
            }
        }

        let error_pct = (peak_f - f_helmholtz).abs() / f_helmholtz * 100.0;
        assert!(
            error_pct < 4.0,
            "Helmholtz ram peak at {peak_f:.1} Hz must match theoretical {f_helmholtz:.1} Hz within 4% (got {error_pct:.2}%)"
        );
    }

    #[test]
    fn itbs_are_brighter_than_a_single_throttle() {
        // Individual throttle bodies (ITBs) radiate directly through short velocity
        // stacks to atmosphere with no plenum or airbox compliance. A single-throttle
        // plenum/airbox system acts as an acoustic lowpass filter, attenuating high-frequency
        // valve-closing transients.
        // Therefore, ITBs produce measurably more high-frequency / high-order content
        // than a single-throttle setup on the same engine.
        let fs = 48_000.0f32;

        let itb_system = IntakeSystem {
            runners: vec![crate::physics::PipeSection::new(0.18, 0.0015, 300.0); 4],
            plenum_volume: 0.0,
            throttle: ThrottleLayout::IndividualBodies { bore: 0.045 },
            airbox: None,
            snorkel: None,
            trumpet_flanged: true,
        };

        let single_system = IntakeSystem {
            runners: vec![crate::physics::PipeSection::new(0.25, 0.0015, 300.0); 4],
            plenum_volume: 0.0035,
            throttle: ThrottleLayout::Single { bore: 0.065 },
            airbox: Some(crate::physics::PipeSection::new(0.20, 0.015, 300.0)),
            snorkel: Some(crate::physics::PipeSection::new(0.30, 0.003, 300.0)),
            trumpet_flanged: false,
        };

        let mut net_itb = IntakeNetwork::new(&itb_system, 4, fs);
        let mut net_single = IntakeNetwork::new(&single_system, 4, fs);

        net_itb.set_throttle(1.0);
        net_single.set_throttle(1.0);

        let rpm = 3_000.0f32;
        let f_firing = 4.0 / 2.0 * (rpm / 60.0); // 100 Hz
        let period = (fs / f_firing) as usize; // 480 samples

        let n_samples = 48_000;
        let mut out_itb = Vec::with_capacity(n_samples);
        let mut out_single = Vec::with_capacity(n_samples);

        for n in 0..n_samples {
            let mut excits = [0.0f32; 4];
            let cyl = (n / period) % 4;
            let phase = n % period;
            if phase < 120 {
                let flow = 0.05 * (std::f32::consts::PI * phase as f32 / 120.0).sin();
                let d_flow = if phase > 100 { -0.05 * fs / 20.0 } else { 0.0 };
                excits[cyl] = induction_rarefaction_pa(flow, 0.0015, 343.0)
                    + valve_slam_pa(d_flow, 0.20, 0.0015);
            }
            out_itb.push(net_itb.step(&excits));
            out_single.push(net_single.step(&excits));
        }

        let steady = 4_800..n_samples;
        let itb_high = magnitude_at(&out_itb[steady.clone()], 3_000.0, fs);
        let itb_low = magnitude_at(&out_itb[steady.clone()], f_firing, fs);
        let itb_ratio = itb_high / itb_low.max(1e-6);

        let single_high = magnitude_at(&out_single[steady.clone()], 3_000.0, fs);
        let single_low = magnitude_at(&out_single[steady], f_firing, fs);
        let single_ratio = single_high / single_low.max(1e-6);

        assert!(
            itb_ratio > 1.5 * single_ratio,
            "ITBs must be brighter than single throttle: ITB ratio={itb_ratio:.4} vs single ratio={single_ratio:.4}"
        );
    }

    #[test]
    fn intake_network_constructs_and_runs_stable() {
        let system = IntakeSystem::default_for_cylinders(4);
        let mut network = IntakeNetwork::new(&system, 4, 48_000.0);
        network.tune(
            INTAKE_AIR_GAMMA,
            INTAKE_GAS_CONSTANT,
            INTAKE_AMBIENT_TEMPERATURE_K,
        );

        let excitations = [100.0, 0.0, 0.0, 0.0];
        for _ in 0..1000 {
            let rad = network.step(&excitations);
            assert!(rad.is_finite());
        }
    }

    #[test]
    fn a_resistive_sheet_absorbs_what_it_neither_passes_nor_returns() {
        // The element is the only thing in an intake tract that dissipates.
        // Reflection and transmission sum to one in pressure by construction,
        // so what it costs shows up in power: `r^2 + t^2 < 1`, and the
        // difference went into the paper as heat.
        for zeta in [0.0f32, 0.05, 0.25, 1.0, 4.0] {
            let sheet = ResistiveSheet::new(zeta);
            let (r, t) = (sheet.reflection(), sheet.transmission());
            assert!((r + t - 1.0).abs() < 1e-6, "zeta {zeta}: r + t = {}", r + t);
            let absorbed = 1.0 - (r * r + t * t);
            if zeta == 0.0 {
                assert!(
                    absorbed.abs() < 1e-6,
                    "a transparent sheet absorbed {absorbed}"
                );
            } else {
                assert!(absorbed > 0.0, "zeta {zeta} absorbed nothing");
            }
            // The scatter is symmetric: the element does not care which side a
            // wave arrives from.
            let (back, through) = sheet.scatter(1.0, 0.0);
            let (back_r, through_r) = sheet.scatter(0.0, 1.0);
            assert!((back - through_r).abs() < 1e-6);
            assert!((through - back_r).abs() < 1e-6);
        }

        // The limits: transparent at zero, a rigid wall at infinity.
        let open = ResistiveSheet::new(0.0);
        assert!((open.transmission() - 1.0).abs() < 1e-6);
        let blocked = ResistiveSheet::new(1.0e6);
        assert!(blocked.reflection() > 0.999);
    }

    #[test]
    fn a_fitted_filter_damps_the_tract_and_a_bare_trumpet_does_not() {
        // An airbox has an element in it; that is what a box is for. A set of
        // individual throttle bodies has bare stacks and gets none, which is
        // also why they are the louder installation.
        let with_box = crate::bench::EnginePreset::inline_four().intake;
        assert!(with_box.airbox.is_some());
        let net = IntakeNetwork::new(&with_box, 4, 48_000.0);
        assert!(net.has_filter(), "an airbox was fitted without an element");

        let mut bare = with_box.clone();
        bare.airbox = None;
        bare.snorkel = None;
        bare.throttle = ThrottleLayout::IndividualBodies { bore: 0.048 };
        let net = IntakeNetwork::new(&bare, 4, 48_000.0);
        assert!(!net.has_filter(), "bare stacks were given a filter");
    }

    #[test]
    fn induction_rarefaction_matches_physical_formula() {
        let mdot = 0.045f32;
        let area = 0.0015f32;
        let c = 340.0f32;
        let p_prime = induction_rarefaction_pa(mdot, area, c);
        assert!((p_prime - (-c * mdot / area)).abs() < 1e-4);
        assert!(p_prime < 0.0, "rarefaction must be negative pressure");

        // Zero mass flow produces no pressure pulse
        assert_eq!(induction_rarefaction_pa(0.0, area, c), 0.0);
    }

    #[test]
    fn valve_slam_scales_with_flow_derivative_and_runner_length() {
        let d_mdot = -15.0f32;
        let l1 = 0.25f32;
        let l2 = 0.50f32;
        let area = 0.0014f32;

        let p1 = valve_slam_pa(d_mdot, l1, area);
        let p2 = valve_slam_pa(d_mdot, l2, area);

        assert!(
            p1 > 0.0,
            "water hammer slam must be a positive pressure surge"
        );
        assert!(
            (p2 - 2.0 * p1).abs() < 1e-4,
            "doubling runner length must double water hammer amplitude"
        );

        let p_double_derivative = valve_slam_pa(2.0 * d_mdot, l1, area);
        assert!(
            (p_double_derivative - 2.0 * p1).abs() < 1e-4,
            "doubling flow shutoff rate must double water hammer amplitude"
        );

        // Opening valve (flow increasing) produces zero slam pulse
        assert_eq!(valve_slam_pa(15.0, l1, area), 0.0);
    }

    #[test]
    fn dipole_orifice_noise_scales_as_flow_cubed() {
        let u1 = 0.5f32;
        let u2 = 1.0f32;
        let amp1 = dipole_orifice_amplitude(u1);
        let amp2 = dipole_orifice_amplitude(u2);
        assert!(
            (amp2 - 8.0 * amp1).abs() < 1e-4,
            "doubling flow velocity must scale dipole amplitude by 2^3 = 8"
        );
        assert_eq!(dipole_orifice_amplitude(0.0), 0.0);
    }

    #[test]
    fn throttle_area_sweeps_from_leak_to_bore() {
        let bore = 0.070;
        let bore_area = (std::f64::consts::PI * (bore * 0.5) * (bore * 0.5)) as f32;
        let leak_area = bore_area * 0.008;

        let a_shut = throttle_area(bore, 0.0);
        let a_open = throttle_area(bore, 1.0);

        assert!((a_shut - leak_area).abs() < 1e-6);
        assert!((a_open - bore_area).abs() < 1e-6);

        // Monotonic with pedal position
        let mut prev = a_shut;
        for i in 1..=10 {
            let a = throttle_area(bore, i as f32 * 0.1);
            assert!(a >= prev, "throttle area must increase monotonically");
            prev = a;
        }
    }

    #[test]
    fn throttle_restriction_is_passive_and_conserves_continuity() {
        let area = 0.002f32;
        let a_open = area;
        let a_shut = area * 0.008;

        // Wide open throttle: transmission is high, reflection is minimal
        let (refl_open, trans_open) = scatter_throttle_restriction(100.0, 0.0, area, area, a_open);
        assert!(
            (trans_open - 100.0).abs() < 1.0,
            "open throttle must transmit wave: got {trans_open}"
        );
        assert!(
            refl_open.abs() < 1.0,
            "open throttle reflection must vanish: got {refl_open}"
        );

        // Shut throttle: reflection is nearly total rigid (+1.0), transmission is blocked
        let (refl_shut, trans_shut) = scatter_throttle_restriction(100.0, 0.0, area, area, a_shut);
        assert!(
            refl_shut > 90.0,
            "shut throttle must reflect incident wave: got {refl_shut}"
        );
        assert!(
            trans_shut < 10.0,
            "shut throttle must block transmission: got {trans_shut}"
        );

        // Passive: power out <= power in
        let p_in = 100.0 * 100.0 * area;
        let p_out_open = (refl_open * refl_open + trans_open * trans_open) * area;
        let p_out_shut = (refl_shut * refl_shut + trans_shut * trans_shut) * area;
        assert!(p_out_open <= p_in + 1e-3);
        assert!(p_out_shut <= p_in + 1e-3);
    }

    #[test]
    fn closing_the_throttle_attenuates_mouth_output_through_area_term() {
        let system = IntakeSystem::default_for_cylinders(4);
        let mut network_open = IntakeNetwork::new(&system, 4, 48_000.0);
        let mut network_shut = IntakeNetwork::new(&system, 4, 48_000.0);

        network_open.set_throttle(1.0);
        network_shut.set_throttle(0.0);

        let mut energy_open = 0.0f32;
        let mut energy_shut = 0.0f32;

        // Drive with periodic suction rarefaction pulses
        for n in 0..2_400 {
            let excit = if n % 240 == 0 { -5_000.0 } else { 0.0 };
            let excits = [excit, 0.0, 0.0, 0.0];

            let out_open = network_open.step(&excits);
            let out_shut = network_shut.step(&excits);

            energy_open += out_open * out_open;
            energy_shut += out_shut * out_shut;
        }

        let rms_open = (energy_open / 2_400.0).sqrt();
        let rms_shut = (energy_shut / 2_400.0).sqrt();

        assert!(rms_open > 0.0, "open throttle must radiate sound");
        assert!(
            rms_shut < 0.20 * rms_open,
            "shutting the throttle must attenuate mouth output by at least 5x: open={rms_open}, shut={rms_shut}"
        );
    }
}
