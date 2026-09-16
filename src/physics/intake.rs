//! Throttle body and the 0D lumped intake plenum.
//!
//! The induction side is split into two pieces that meet at one state variable
//! pair. Upstream, the throttle plate is a compressible orifice whose effective
//! area is set by pedal position. Downstream, the manifold is an open
//! thermodynamic control volume that the cylinders draw from:
//!
//! ```text
//! m_dot_th   = C_d A(TPS) * P_up / sqrt(R T_up) * Psi(P_man / P_up)
//! dm_man/dt  = m_dot_th - sum_i m_dot_v,i
//! d(m c_v T)/dt = m_dot_th c_p T_up - sum_i m_dot_v,i c_p T_v,i - Q_dot_wall
//! P_man      = m_man R T_man / V_man
//! ```
//!
//! The last line is the whole reason this module exists as a control volume
//! rather than a lookup: pressure is not a state, it is an *algebraic closure*
//! on the two states, so it is re-evaluated at every RK4 stage from that
//! stage's mass and temperature. The throttle flow is a steep function of
//! `P_man` — the closure is what couples the plate to the plenum, and freezing
//! it across a step is what makes an explicit plenum overshoot ambient and
//! silently supercharge a naturally aspirated engine.
//!
//! [`IntakePlenum::port_state`] hands the resulting pressure and temperature
//! back as a [`PortState`], which is exactly the boundary condition
//! [`port_flux`](super::thermodynamics) already solves each intake valve
//! against. The plenum is therefore the dynamic upstream condition for the
//! cylinders, and the cylinders are the `sum_i m_dot_v,i` draw on the plenum.
//!
//! Sign convention throughout: mass flows are positive in the direction the
//! name implies — `m_dot_th` positive *into* the plenum, a [`ValveDraw`]
//! positive *out of* it. Both reverse cleanly, because reverse flow is the same
//! orifice equation with the two sides exchanged.

use std::f64::consts::{FRAC_PI_2, PI};

use crate::audio::{
    BlowOffVoicing, CentrifugalVoicing, RootsVoicing, TurboModel, TurboVoicing, WastegateVoicing,
};
use crate::environment::Environment;
use crate::physics::compressor::{self, CompressorMap};
use crate::physics::cylinder::GasProperties;
use crate::physics::engine_block::Plenum;
use crate::physics::thermodynamics::{flow_function, PortConditions, PortState};
use crate::physics::turbine::{BearingType, BoostController, TurbineMap, TurboShaft, Wastegate};

/// Smallest mass the plenum is allowed to hold [kg].
///
/// The energy equation divides by `m c_v`, so the floor is what keeps a plenum
/// that has been pumped flat from producing an infinite temperature rate.
const MASS_FLOOR: f64 = 1e-9;

/// Coldest and hottest the charge is allowed to get [K].
const TEMPERATURE_BOUNDS: (f64, f64) = (1.0, 4000.0);

/// How hard a compressor pushed past its own surge boundary reverses flow,
/// per unit of pressure-ratio overshoot [-]. See
/// [`ForcedInduction::advance_intake`]'s surge branch.
const SURGE_REVERSAL_GAIN: f64 = 3.0;

/// Largest number of internal steps one `advance` will take.
///
/// Only reached for frame deltas far longer than the simulator ever hands over;
/// it exists so a pathological `dt` costs bounded time instead of hanging.
const MAX_SUBSTEPS: usize = 1024;

/// Fraction of the fastest local time constant one internal step may span.
///
/// RK4 on the plenum is stable well past this, but accuracy is what matters
/// here: the fill transient *is* the thing being measured.
const STABILITY_FRACTION: f64 = 0.25;

/// Pressure-ratio band below unity over which `Psi` is linearised [-].
const LINEAR_BAND: f64 = 0.002;

/// Isentropic flow function, linearised over the last [`LINEAR_BAND`] of
/// pressure ratio before equilibrium.
///
/// Away from equilibrium this is exactly
/// [`flow_function`](super::thermodynamics::flow_function) — choking included.
/// The change is confined to `P_man > 0.998 P_up`, and it is there for two
/// reasons that happen to agree.
///
/// The physical one: `Psi ~ sqrt(2 (1 - P_r))` as the ratio approaches one, and
/// a square-root in the *pressure difference* is the inviscid Bernoulli limit.
/// A real bore does not obey it as the difference vanishes — the throat goes
/// low-Mach and viscous, where flow is linear in `dP`, not in its root.
///
/// The numerical one: that square root has an infinite derivative at
/// equilibrium, `d(m_dot)/dP -> -inf` as `P_man -> P_up`. No explicit step,
/// however small, can land on the equilibrium — a step of `h` leaves a residual
/// depression of order `(k h / 2)^2`, which at a millisecond frame is over a
/// kilopascal of phantom vacuum. That is a 1 % torque error parked at wide-open
/// throttle, and worse, it is large enough to invert the *ordering* of MAP
/// against pedal over the last tenth of travel, where the true spread is under
/// 100 Pa. Linearising the last 0.2 % of ratio makes the Jacobian finite, so the
/// plenum settles on `P_man = P_up` exactly rather than hovering below it.
fn throttle_flow_function(pressure_ratio: f64, gamma: f64) -> f64 {
    let pr = pressure_ratio.clamp(0.0, 1.0);
    let knee = 1.0 - LINEAR_BAND;
    if pr <= knee {
        flow_function(pr, gamma)
    } else {
        // Continuous at the knee, and exactly zero at `P_r = 1`.
        flow_function(knee, gamma) * (1.0 - pr) / LINEAR_BAND
    }
}

// ---------------------------------------------------------------------------
// Throttle body
// ---------------------------------------------------------------------------

/// The throttle plate, as a variable-area compressible orifice.
///
/// Area comes from a butterfly approximation,
///
/// ```text
/// A_open(TPS) = pi D^2 / 4 * (1 - cos(TPS * pi/2))
/// ```
///
/// which sweeps the plate from shut at `TPS = 0` to the full bore at
/// `TPS = 1`. It is worth preferring over a linear ramp for the same reason
/// [`ValveEvent::lift`](super::thermodynamics::ValveEvent::lift) is a raised
/// cosine: the derivative vanishes at the seat, so the first few percent of
/// pedal open a little area rather than a lot. That is what makes a real
/// throttle driveable off idle, and it is where a linear map feels like a
/// switch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThrottleBody {
    /// Throttle bore diameter [m].
    pub bore: f64,
    /// Discharge coefficient of the bore [-].
    pub discharge_coefficient: f64,
    /// Area still open with the plate shut, as a fraction of the bore [-].
    ///
    /// A shut throttle is never sealed: plate clearance plus whatever idle
    /// bypass the engine uses. Without it a closed throttle pulls the manifold
    /// down towards absolute vacuum and the engine cannot idle at all, because
    /// the only air path is a valve that is shut.
    pub leak_area_fraction: f64,
}

impl Default for ThrottleBody {
    /// A single 70 mm bore, which suits a naturally aspirated 4 litre V8.
    fn default() -> Self {
        Self {
            bore: 0.070,
            discharge_coefficient: 0.90,
            leak_area_fraction: 0.008,
        }
    }
}

impl ThrottleBody {
    /// Builds a throttle body, clamping to a physical envelope.
    pub fn new(bore: f64, discharge_coefficient: f64, leak_area_fraction: f64) -> Self {
        Self {
            bore: bore.max(1e-4),
            discharge_coefficient: discharge_coefficient.clamp(0.0, 1.0),
            leak_area_fraction: leak_area_fraction.clamp(0.0, 1.0),
        }
    }

    /// Full cross-section of the bore [m^2].
    pub fn bore_area(&self) -> f64 {
        PI * self.bore * self.bore / 4.0
    }

    /// Geometric area uncovered by the plate at a pedal position [m^2].
    pub fn open_area(&self, tps: f64) -> f64 {
        let tps = tps.clamp(0.0, 1.0);
        let bore = self.bore_area();
        let swept = bore * (1.0 - (tps * FRAC_PI_2).cos());
        // The bypass is in parallel with the plate, so it adds; the bore is
        // still the hard limit on what can get through.
        (swept + bore * self.leak_area_fraction).min(bore)
    }

    /// Effective flow area `C_d * A_open(TPS)` [m^2].
    ///
    /// The discharge coefficient is folded in here, matching
    /// [`ValveEvent::effective_area`](super::thermodynamics::ValveEvent::effective_area),
    /// so callers multiply an area by a flow function and nothing else.
    pub fn effective_area(&self, tps: f64) -> f64 {
        self.discharge_coefficient * self.open_area(tps)
    }

    /// Pressure ratio at which the throat goes sonic [-].
    ///
    /// ```text
    /// P_crit / P_up = (2 / (gamma + 1)) ^ (gamma / (gamma - 1))
    /// ```
    pub fn critical_pressure_ratio(gamma: f64) -> f64 {
        let g = gamma.clamp(1.01, 1.99);
        (2.0 / (g + 1.0)).powf(g / (g - 1.0))
    }

    /// Whether the plate is choked for a given manifold and upstream pressure.
    pub fn is_choked(&self, upstream: &PortState, manifold_pressure: f64) -> bool {
        let p_up = upstream.pressure.max(1.0);
        let p_man = manifold_pressure.max(0.0);
        p_man / p_up <= Self::critical_pressure_ratio(upstream.gamma)
    }

    /// Mass flow through the plate [kg/s], positive into the manifold.
    ///
    /// ```text
    /// m_dot = C_d A(TPS) * P_up / sqrt(R T_up) * Psi(P_man / P_up)
    /// ```
    ///
    /// Choking is enforced inside
    /// [`flow_function`](super::thermodynamics::flow_function), which holds
    /// `Psi` at its sonic plateau once the ratio drops below critical: past that
    /// point the throat cannot hear the manifold, so pulling more vacuum buys no
    /// more air. That plateau is the airflow ceiling a wide-open throttle runs
    /// into, and it is why the flow saturates instead of growing without bound.
    ///
    /// A manifold above its upstream — boost bleeding back, or a big intake
    /// pulse reaching the plate — simply exchanges the two sides and comes back
    /// negative.
    pub fn mass_flow(&self, tps: f64, upstream: &PortState, manifold: &PortState) -> f64 {
        let area = self.effective_area(tps);
        if area <= 0.0 {
            return 0.0;
        }
        let p_up = upstream.pressure.max(1.0);
        let p_man = manifold.pressure.max(1.0);

        if p_up >= p_man {
            area * p_up / (upstream.gas_constant * upstream.temperature.max(1.0)).sqrt()
                * throttle_flow_function(p_man / p_up, upstream.gamma)
        } else {
            -area * p_man / (manifold.gas_constant * manifold.temperature.max(1.0)).sqrt()
                * throttle_flow_function(p_up / p_man, manifold.gamma)
        }
    }
}

// ---------------------------------------------------------------------------
// Valve draw
// ---------------------------------------------------------------------------

/// One cylinder's intake valve, as seen from the plenum.
///
/// This is the `m_dot_v,i` term. It is deliberately not a cylinder: the plenum
/// does not need to know what is on the other side of the valve, only how much
/// mass crossed and how hot it was.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ValveDraw {
    /// Mass flow [kg/s], positive *out of* the plenum and into the cylinder.
    pub mass_flow: f64,
    /// Temperature of the stream when it is flowing *back into* the plenum [K].
    ///
    /// Ignored while `mass_flow` is positive, because gas leaving the plenum
    /// leaves at the plenum's own temperature — a control volume has one
    /// temperature by definition. It matters on reversion during valve overlap,
    /// when hot residual is pushed back up the runner and genuinely heats the
    /// manifold.
    pub temperature: f64,
}

impl ValveDraw {
    /// A draw of `mass_flow` out of the plenum, reverting at `temperature`.
    pub fn new(mass_flow: f64, temperature: f64) -> Self {
        Self {
            mass_flow,
            temperature: temperature.max(TEMPERATURE_BOUNDS.0),
        }
    }
}

// ---------------------------------------------------------------------------
// Intake plenum
// ---------------------------------------------------------------------------

/// The intake manifold as a fixed-volume, well-stirred open control volume.
///
/// State is `[m_man, T_man]`. Pressure is not a state — see the module docs.
///
/// Integrated with RK4 over both states at once, sub-stepped internally so a
/// wall-clock frame that is long compared with the manifold's own filling time
/// still resolves the transient. A plenum this size empties and fills in a few
/// milliseconds, which is shorter than an audio frame, so the sub-stepping is
/// not optional.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IntakePlenum {
    /// Plenum volume `V_man` [m^3].
    pub volume: f64,
    /// The plate feeding it.
    pub throttle: ThrottleBody,
    /// Trapped mass `m_man` [kg].
    pub mass: f64,
    /// Bulk charge temperature `T_man` [K].
    pub temperature: f64,
    /// Specific gas constant of the charge [J/(kg K)].
    pub gas_constant: f64,
    /// Ratio of specific heats of the charge [-].
    pub gamma: f64,
    /// Manifold wall temperature [K].
    pub wall_temperature: f64,
    /// Wall heat transfer conductance `h A` [W/K].
    ///
    /// Drives the `Q_dot_wall` term. On a heat-soaked manifold this runs
    /// *backwards* — the wall warms the charge, thinning it, which is a real
    /// and unhelpful loss of volumetric efficiency rather than a rounding error.
    pub wall_conductance: f64,
}

impl IntakePlenum {
    /// A plenum filled to a given pressure and temperature.
    pub fn new(
        volume: f64,
        throttle: ThrottleBody,
        pressure: f64,
        temperature: f64,
        gas_constant: f64,
        gamma: f64,
    ) -> Self {
        let volume = volume.max(1e-6);
        let temperature = temperature.clamp(TEMPERATURE_BOUNDS.0, TEMPERATURE_BOUNDS.1);
        let gas_constant = gas_constant.max(1.0);
        let gamma = gamma.clamp(1.01, 1.99);
        Self {
            volume,
            throttle,
            mass: (pressure.max(0.0) * volume / (gas_constant * temperature)).max(MASS_FLOOR),
            temperature,
            gas_constant,
            gamma,
            wall_temperature: temperature,
            wall_conductance: 0.0,
        }
    }

    /// A plenum sitting at ambient, breathing the unburned-charge gas.
    ///
    /// The wall starts warm and mildly conductive: a manifold bolted to a
    /// running engine is not at air temperature.
    pub fn at_ambient(
        volume: f64,
        throttle: ThrottleBody,
        env: &Environment,
        gas: &GasProperties,
    ) -> Self {
        let mut plenum = Self::new(
            volume,
            throttle,
            env.pressure,
            env.temperature,
            gas.r_unburned,
            gas.gamma_unburned,
        );
        plenum.wall_temperature = env.temperature + 40.0;
        plenum.wall_conductance = 5.0;
        plenum
    }

    /// Ambient air ahead of the plate, as an upstream boundary condition.
    ///
    /// Reuses the intake half of
    /// [`PortConditions::from_environment`](super::thermodynamics::PortConditions::from_environment)
    /// so the airbox and the port agree on what ambient air is.
    pub fn ambient_upstream(env: &Environment, gas: &GasProperties) -> PortState {
        PortConditions::from_environment(env, gas).intake
    }

    /// Manifold absolute pressure `P_man = m R T / V` [Pa].
    pub fn pressure(&self) -> f64 {
        self.mass * self.gas_constant * self.temperature / self.volume
    }

    /// Charge density [kg/m^3].
    pub fn density(&self) -> f64 {
        self.mass / self.volume
    }

    /// Constant-volume specific heat `c_v = R / (gamma - 1)` [J/(kg K)].
    pub fn cv(&self) -> f64 {
        self.gas_constant / (self.gamma - 1.0)
    }

    /// Constant-pressure specific heat `c_p = gamma * c_v` [J/(kg K)].
    pub fn cp(&self) -> f64 {
        self.gamma * self.cv()
    }

    /// Speed of sound in the charge [m/s].
    pub fn speed_of_sound(&self) -> f64 {
        (self.gamma * self.gas_constant * self.temperature).sqrt()
    }

    /// Manifold pressure relative to ambient [Pa]; negative is vacuum.
    pub fn gauge_pressure(&self, ambient: f64) -> f64 {
        self.pressure() - ambient
    }

    /// The boundary condition the intake valves see.
    ///
    /// This is the hand-off named in the module docs: drop it into
    /// [`PortConditions::intake`](super::thermodynamics::PortConditions) and
    /// every cylinder solves its valve against the live manifold pressure
    /// instead of against ambient.
    pub fn port_state(&self) -> PortState {
        PortState {
            pressure: self.pressure(),
            temperature: self.temperature,
            gas_constant: self.gas_constant,
            gamma: self.gamma,
            // Fresh charge: nothing in the manifold has burned. Reverted
            // residual is accounted for in the energy balance, not by
            // pretending the plenum is full of exhaust.
            burned_fraction: 0.0,
        }
    }

    /// Mass flow through the throttle at the current state [kg/s].
    pub fn throttle_flow(&self, tps: f64, upstream: &PortState) -> f64 {
        self.throttle.mass_flow(tps, upstream, &self.port_state())
    }

    /// Heat leaving the charge to the wall `Q_dot_wall` [W].
    ///
    /// Positive out of the gas, matching the sign it carries in the energy
    /// equation. A wall hotter than the charge makes it negative.
    pub fn wall_heat_flow(&self) -> f64 {
        self.wall_conductance.max(0.0) * (self.temperature - self.wall_temperature)
    }

    /// Sums the valve terms at a given charge temperature.
    ///
    /// Returns `(net mass out [kg/s], net enthalpy out [W])`.
    fn valve_balance(&self, temperature: f64, draws: &[ValveDraw]) -> (f64, f64) {
        let cp = self.cp();
        let mut mass_out = 0.0;
        let mut enthalpy_out = 0.0;
        for draw in draws {
            if !draw.mass_flow.is_finite() {
                continue;
            }
            mass_out += draw.mass_flow;
            // Outflow leaves at the plenum's temperature; reversion arrives at
            // the cylinder's. One plenum `c_p` is used for both, because the
            // control volume tracks a single gas composition — the same
            // simplification the exhaust
            // [`Plenum`](super::engine_block::Plenum) makes.
            let stream = if draw.mass_flow >= 0.0 {
                temperature
            } else {
                draw.temperature.max(TEMPERATURE_BOUNDS.0)
            };
            enthalpy_out += draw.mass_flow * cp * stream;
        }
        (mass_out, enthalpy_out)
    }

    /// `(dm/dt, dT/dt)` at an arbitrary `(mass, temperature)`.
    ///
    /// Taking the state as arguments rather than reading `self` is what lets the
    /// RK4 stages evaluate at their own intermediate states — and with it, the
    /// pressure closure `P = m R T / V` is recomputed here, once per stage, so
    /// the throttle flow always sees the pressure that belongs to the stage.
    ///
    /// The temperature rate is the given energy equation with the product rule
    /// applied and `c_v` held constant:
    ///
    /// ```text
    /// d(m c_v T)/dt = c_v (T dm/dt + m dT/dt)
    /// =>   dT/dt = [ m_dot_th c_p T_up - sum_i m_dot_v,i c_p T_v,i
    ///                - Q_dot_wall - c_v T dm/dt ] / (m c_v)
    /// ```
    fn derivatives(
        &self,
        mass: f64,
        temperature: f64,
        tps: f64,
        upstream: &PortState,
        draws: &[ValveDraw],
    ) -> (f64, f64) {
        let mass = mass.max(MASS_FLOOR);
        let temperature = temperature.clamp(TEMPERATURE_BOUNDS.0, TEMPERATURE_BOUNDS.1);
        let cv = self.cv();
        let cp = self.cp();

        // Algebraic pressure closure, re-evaluated for this stage's state.
        let stage = PortState {
            pressure: mass * self.gas_constant * temperature / self.volume,
            temperature,
            gas_constant: self.gas_constant,
            gamma: self.gamma,
            burned_fraction: 0.0,
        };

        let throttle_flow = self.throttle.mass_flow(tps, upstream, &stage);
        let (valve_out, valve_enthalpy_out) = self.valve_balance(temperature, draws);

        let dmass = throttle_flow - valve_out;

        // Air pushed back out of the plate leaves at the plenum's temperature,
        // not at the airbox's.
        let throttle_enthalpy_in = if throttle_flow >= 0.0 {
            throttle_flow * cp * upstream.temperature.max(TEMPERATURE_BOUNDS.0)
        } else {
            throttle_flow * cp * temperature
        };

        let wall_loss = self.wall_conductance.max(0.0) * (temperature - self.wall_temperature);

        let dtemperature =
            (throttle_enthalpy_in - valve_enthalpy_out - wall_loss - cv * temperature * dmass)
                / (mass * cv);

        (dmass, dtemperature)
    }

    /// Shortest time constant the state is currently moving on [s].
    ///
    /// The filling mode is measured, not guessed: one extra derivative
    /// evaluation gives `d(dm/dt)/dm` by finite difference, and its reciprocal
    /// is the local relaxation time of the mass channel. A closed-form estimate
    /// is the tempting alternative and it is wrong in the case that matters —
    /// `V / (A c)` is the *choked* fill time, some tens of times longer than the
    /// mode that survives once the plate is barely restricting and `Psi` has
    /// gone steep. Sizing steps off the closed form leaves the last kilopascal
    /// of the fill unresolved, which is precisely the region a wide-open
    /// throttle lives in.
    ///
    /// The temperature channel is probed the same way, and it is not optional.
    /// A choked plate is *insensitive to manifold pressure* by definition, so
    /// while the throat is sonic the mass channel reports no restoring force
    /// whatsoever — and on a nearly empty plenum the energy equation is
    /// meanwhile moving on a timescale of microseconds, because `dT/dt` carries
    /// a `1 / (m c_v)`. Sizing steps off the mass channel alone lets a frame
    /// step straight over the charge heating and land hundreds of kelvin out.
    ///
    /// A residence time `m / (throughput)` backs both up without differencing
    /// anything, which is what covers the finite differences landing on the
    /// kink where `Psi` is linearised.
    fn relaxation_time(&self, tps: f64, upstream: &PortState, draws: &[ValveDraw]) -> f64 {
        let mass = self.mass.max(MASS_FLOOR);
        let mut tau = f64::INFINITY;

        let temperature = self.temperature;
        let base = self.derivatives(mass, temperature, tps, upstream, draws);
        let mut rate: f64 = 0.0;

        // How hard the pressure closure pushes back on further filling.
        let probe_mass = mass * 1e-6;
        let perturbed = self.derivatives(mass + probe_mass, temperature, tps, upstream, draws);
        rate = rate.max(((perturbed.0 - base.0) / probe_mass).abs());

        // Charge exchange and wall heating, both of which act through `dT/dt`.
        let probe_temperature = temperature * 1e-6;
        let perturbed =
            self.derivatives(mass, temperature + probe_temperature, tps, upstream, draws);
        rate = rate.max(((perturbed.1 - base.1) / probe_temperature).abs());

        if rate.is_finite() && rate > 1e-12 {
            tau = tau.min(1.0 / rate);
        }

        // Undifferenced backstop: how long the volume takes to exchange its own
        // contents, counting both directions of every stream.
        let throughput: f64 = self.throttle_flow(tps, upstream).abs()
            + draws
                .iter()
                .filter(|d| d.mass_flow.is_finite())
                .map(|d| d.mass_flow.abs())
                .sum::<f64>();
        if throughput > 1e-12 {
            tau = tau.min(mass / throughput);
        }

        tau
    }

    /// Advances the control volume by `dt` under a throttle position and a set
    /// of valve draws.
    ///
    /// The draws are held constant across the internal steps: they are the
    /// cylinder solve's answer for this sub-step, and the cylinders are
    /// integrated on their own crank-angle grid. That is the quasi-steady
    /// coupling the 0D model is built on — the plenum reacts to the flow the
    /// cylinders just took, and the cylinders next see the pressure that left
    /// the plenum with.
    pub fn advance(&mut self, dt: f64, tps: f64, upstream: &PortState, draws: &[ValveDraw]) {
        if !(dt.is_finite() && dt > 0.0) {
            return;
        }
        let tps = if tps.is_finite() {
            tps.clamp(0.0, 1.0)
        } else {
            0.0
        };

        // Re-planned every internal step rather than once per frame. The
        // relaxation time moves by orders of magnitude *during* a throttle snap
        // — that is what a snap is — so a single plan sized on the state at
        // frame entry resolves the beginning of the transient and not the end.
        let floor = dt / MAX_SUBSTEPS as f64;
        let mut remaining = dt;

        while remaining > 0.0 {
            let tau = self.relaxation_time(tps, upstream, draws);
            let mut h = if tau.is_finite() && tau > 0.0 {
                STABILITY_FRACTION * tau
            } else {
                remaining
            };
            // The floor bounds the loop at `MAX_SUBSTEPS` iterations and so
            // bounds the cost of one frame; the clip lands the last step exactly
            // on `dt` instead of overshooting the frame.
            h = h.max(floor).min(remaining);
            self.rk4(h, tps, upstream, draws);
            remaining -= h;
        }
    }

    /// One classical RK4 step over `[m, T]`.
    fn rk4(&mut self, h: f64, tps: f64, upstream: &PortState, draws: &[ValveDraw]) {
        let (m0, t0) = (self.mass, self.temperature);

        let k1 = self.derivatives(m0, t0, tps, upstream, draws);
        let k2 = self.derivatives(
            m0 + 0.5 * h * k1.0,
            t0 + 0.5 * h * k1.1,
            tps,
            upstream,
            draws,
        );
        let k3 = self.derivatives(
            m0 + 0.5 * h * k2.0,
            t0 + 0.5 * h * k2.1,
            tps,
            upstream,
            draws,
        );
        let k4 = self.derivatives(m0 + h * k3.0, t0 + h * k3.1, tps, upstream, draws);

        let weight = |a: f64, b: f64, c: f64, d: f64| h / 6.0 * (a + 2.0 * b + 2.0 * c + d);

        let mass = m0 + weight(k1.0, k2.0, k3.0, k4.0);
        let temperature = t0 + weight(k1.1, k2.1, k3.1, k4.1);

        // A diverged step is worse than a stalled one: hand back the state the
        // step started from rather than poisoning the pressure closure, and with
        // it every cylinder downstream, with a NaN.
        if mass.is_finite() && temperature.is_finite() {
            self.mass = mass.max(MASS_FLOOR);
            self.temperature = temperature.clamp(TEMPERATURE_BOUNDS.0, TEMPERATURE_BOUNDS.1);
        }
    }
}

/// A charge-air intercooler: a heat exchanger and a pressure drop between the
/// compressor discharge and the throttle.
///
/// Its core volume is not modelled as a plenum of its own — see
/// [`ForcedInduction::new`] — it is added straight into the shared charge pipe
/// capacitance, which is why a bigger core changes throttle response as well
/// as temperature.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Intercooler {
    /// Core volume, added to the charge pipe's own [m^3].
    pub core_volume: f64,
    /// Fraction of the temperature rise above ambient removed, `0.0..=1.0` [-].
    pub effectiveness: f64,
    /// Pressure drop coefficient across the core, `dP = k m_dot^2` [Pa / (kg/s)^2].
    pub loss_coefficient: f64,
}

impl Intercooler {
    /// Discharge temperature after the core, cooled a fraction of the way
    /// back to ambient.
    pub fn cooled_temperature(&self, inlet_temperature: f64, ambient_temperature: f64) -> f64 {
        inlet_temperature
            - self.effectiveness.clamp(0.0, 1.0) * (inlet_temperature - ambient_temperature)
    }

    /// Pressure lost crossing the core at a given mass flow [Pa].
    pub fn pressure_drop(&self, mass_flow: f64) -> f64 {
        self.loss_coefficient.max(0.0) * mass_flow * mass_flow
    }
}

/// Where a blow-off valve's vented flow goes.
///
/// See `docs/TURBO_PLAN.md`'s TB4: in this lumped model both fitments relieve
/// the charge pipe identically — the distinction here is what a listener
/// hears, atmospheric being a second radiating aperture and recirculating
/// venting almost silently back ahead of the compressor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlowOffFitment {
    /// Vents outside — the classic release, and another aperture.
    Atmospheric,
    /// Vents back ahead of the compressor: nearly silent, and the flow keeps
    /// the wheel loaded rather than leaving the system.
    Recirculating,
}

/// A blow-off / dump valve: a real vent from the charge pipe, referenced
/// against the intake manifold pressure the way a real diaphragm is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BlowOffValve {
    pub fitment: BlowOffFitment,
    /// Fully open vent flow area [m^2].
    pub max_flow_area: f64,
    /// Charge-pipe-minus-manifold differential at which the spring starts to
    /// crack the valve [Pa].
    pub spring_preload: f64,
    /// Differential span from just-cracked to fully open [Pa].
    pub opening_span: f64,
}

impl BlowOffValve {
    /// Open fraction, `0..=1`, from how far the charge pipe sits above the
    /// intake manifold past the spring's own preload — a shut throttle
    /// leaves the manifold in vacuum, which is what actually opens a real
    /// valve on lift rather than boost pressure alone.
    pub fn open_fraction(&self, charge_pipe_pressure: f64, manifold_pressure: f64) -> f64 {
        ((charge_pipe_pressure - manifold_pressure - self.spring_preload)
            / self.opening_span.max(1.0))
        .clamp(0.0, 1.0)
    }

    /// Mass flow vented from the charge pipe [kg/s].
    pub fn mass_flow(
        &self,
        charge_pipe: &PortState,
        downstream_pressure: f64,
        manifold_pressure: f64,
    ) -> f64 {
        let area = self.max_flow_area * self.open_fraction(charge_pipe.pressure, manifold_pressure);
        if area <= 0.0 {
            return 0.0;
        }
        let p_up = charge_pipe.pressure.max(1.0);
        let p_down = downstream_pressure.max(1.0);
        if p_down >= p_up {
            return 0.0;
        }
        area * p_up / (charge_pipe.gas_constant * charge_pipe.temperature.max(1.0)).sqrt()
            * flow_function(p_down / p_up, charge_pipe.gamma)
    }
}

/// Real turbocharger hardware: the compressor and turbine maps and the shaft
/// they share, plus the charge pipe capacitance between the compressor and
/// the throttle.
///
/// This is what closes the loop TB1 and TB2 leave open — see
/// `docs/TURBO_PLAN.md`'s TB3. Before this, [`crate::physics::compressor`] and
/// [`crate::physics::turbine`] describe a compressor and a turbine that
/// nothing in the running engine ever calls.
#[derive(Debug, Clone)]
pub struct ForcedInduction {
    pub compressor_map: CompressorMap,
    pub turbine_map: TurbineMap,
    pub shaft: TurboShaft,
    /// Capacitance between the compressor discharge and the throttle plate,
    /// its volume already including any intercooler core.
    pub charge_pipe: Plenum,
    pub intercooler: Option<Intercooler>,
    pub wastegate: Option<Wastegate>,
    pub boost_controller: Option<BoostController>,
    pub blow_off: Option<BlowOffValve>,
    /// Shaft power the compressor drew on the most recent
    /// [`Self::advance_intake`] [W], held here so [`Self::advance_exhaust`]
    /// can load the shaft with it without recomputing the compressor's
    /// operating point a second time.
    compressor_power: f64,
}

impl ForcedInduction {
    /// Builds the hardware at rest, with the charge pipe filled to ambient.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        compressor_map: CompressorMap,
        turbine_map: TurbineMap,
        inertia: f64,
        mechanical_efficiency: f64,
        bearing: BearingType,
        charge_pipe_volume: f64,
        intercooler: Option<Intercooler>,
        env: &Environment,
        gas: &GasProperties,
    ) -> Self {
        let volume = charge_pipe_volume + intercooler.map(|ic| ic.core_volume).unwrap_or(0.0);
        Self {
            compressor_map,
            turbine_map,
            shaft: TurboShaft::new(inertia, mechanical_efficiency, bearing),
            charge_pipe: Plenum::new(
                volume,
                env.pressure,
                env.temperature,
                gas.r_unburned,
                gas.gamma_unburned,
            ),
            intercooler,
            wastegate: None,
            boost_controller: None,
            blow_off: None,
            compressor_power: 0.0,
        }
    }

    /// Fits a wastegate to bypass the turbine.
    pub fn with_wastegate(mut self, wastegate: Wastegate) -> Self {
        self.wastegate = Some(wastegate);
        self
    }

    /// Fits the closed-loop controller that biases the wastegate's threshold.
    pub fn with_boost_controller(mut self, controller: BoostController) -> Self {
        self.boost_controller = Some(controller);
        self
    }

    /// Fits a blow-off valve to vent the charge pipe.
    pub fn with_blow_off(mut self, valve: BlowOffValve) -> Self {
        self.blow_off = Some(valve);
        self
    }

    /// The charge pipe's current [`PortState`], without advancing anything.
    ///
    /// Used to get a representative throttle-flow estimate for this frame's
    /// [`Self::advance_intake`] call before it runs — the same lagged-state
    /// pattern [`ExhaustManifold::tailpipe_flow`](super::engine_block::ExhaustManifold::tailpipe_flow)
    /// already uses.
    pub fn upstream_port_state(&self) -> PortState {
        PortState {
            pressure: self.charge_pipe.pressure(),
            temperature: self.charge_pipe.temperature,
            gas_constant: self.charge_pipe.gas_constant,
            gamma: self.charge_pipe.gamma,
            burned_fraction: 0.0,
        }
    }

    /// Advances the compressor and charge pipe by `dt`, given the mass flow
    /// currently leaving the pipe through the throttle, and returns the
    /// [`PortState`] the throttle should read as its upstream this frame.
    ///
    /// The compressor's own flow is read off the map at the pipe's *current*
    /// pressure ratio via [`CompressorMap::flow_for_pressure_ratio`] — flow is
    /// the map's dependent axis, but pressure is the quantity the plenum
    /// actually holds, so the inverse read is the one this boundary needs.
    /// That flow need not match what the throttle is drawing this same
    /// instant, and the mismatch is exactly what gives the pipe real
    /// capacitance: shut the throttle and the compressor, still turning,
    /// keeps supplying its surge-line flow while the throttle's leak area
    /// draws far less, so the pipe pressure moves — the rate set by
    /// [`Plenum::volume`].
    /// `manifold_pressure` is the intake manifold's own pressure, downstream
    /// of the throttle — needed only to reference a fitted [`BlowOffValve`]
    /// against, since a real one opens on manifold vacuum as much as on
    /// charge pipe pressure.
    pub fn advance_intake(
        &mut self,
        dt: f64,
        throttle_flow: f64,
        manifold_pressure: f64,
        env: &Environment,
        gas: &GasProperties,
    ) -> PortState {
        let ambient = IntakePlenum::ambient_upstream(env, gas);
        let pressure_ratio = (self.charge_pipe.pressure() / ambient.pressure.max(1.0)).max(1.0);
        let corrected_speed =
            compressor::corrected_speed(self.shaft.shaft_rpm(), ambient.temperature);
        let (corrected_flow, reading) = self
            .compressor_map
            .flow_for_pressure_ratio(corrected_speed, pressure_ratio);

        // The map cannot make this pressure ratio forward at all at this
        // speed — `flow_for_pressure_ratio` has clamped to the surge
        // boundary rather than inventing negative data of its own (a map is
        // a measured device). The system is not obliged to stop there: a
        // real wheel asked for more head than it can produce surges, and
        // flow reverses. Scaling the reversal against this speed's own surge
        // flow, rather than a constant, is what lets the resulting
        // oscillation's amplitude and period come from the map and the
        // charge pipe's own capacitance instead of being tuned by hand.
        let mass_flow = if reading.region == compressor::MapRegion::Surge {
            let achievable = reading.pressure_ratio.max(1.0);
            let overshoot = (pressure_ratio / achievable - 1.0).max(0.0);
            let scale = self.compressor_map.surge_flow_at(corrected_speed);
            let reversed_corrected = -SURGE_REVERSAL_GAIN * overshoot * scale;
            reversed_corrected * (ambient.pressure / compressor::P_REF)
                / (ambient.temperature / compressor::T_REF).sqrt()
        } else {
            (corrected_flow * (ambient.pressure / compressor::P_REF)
                / (ambient.temperature / compressor::T_REF).sqrt())
            .max(0.0)
        };

        let discharge_temperature = compressor::discharge_temperature(
            ambient.temperature,
            reading.pressure_ratio,
            reading.efficiency.max(0.05),
            ambient.gamma,
        );
        let inflow_temperature = match &self.intercooler {
            Some(ic) => ic.cooled_temperature(discharge_temperature, ambient.temperature),
            None => discharge_temperature,
        };

        self.compressor_power = compressor::compressor_power(
            mass_flow.max(0.0),
            ambient.temperature,
            reading.pressure_ratio,
            reading.efficiency.max(0.05),
            ambient.gas_constant,
            ambient.gamma,
        );

        if let Some(controller) = &mut self.boost_controller {
            controller.update(dt, pressure_ratio);
        }

        let charge_pipe_state = self.upstream_port_state();
        let blow_off_flow = self
            .blow_off
            .as_ref()
            .map(|bov| bov.mass_flow(&charge_pipe_state, ambient.pressure, manifold_pressure))
            .unwrap_or(0.0);

        self.charge_pipe.integrate(
            dt,
            mass_flow,
            inflow_temperature,
            throttle_flow.max(0.0) + blow_off_flow,
        );

        let mut pressure = self.charge_pipe.pressure();
        if let Some(ic) = &self.intercooler {
            pressure -= ic.pressure_drop(throttle_flow.max(0.0));
        }
        PortState {
            pressure: pressure.max(1e3),
            temperature: self.charge_pipe.temperature,
            gas_constant: self.charge_pipe.gas_constant,
            gamma: self.charge_pipe.gamma,
            burned_fraction: 0.0,
        }
    }

    /// Advances the shaft from the actual exhaust manifold state, and returns
    /// the total mass flow that should leave the exhaust collector this frame
    /// — through the turbine wheel, plus whatever a fitted wastegate bypasses
    /// around it — in place of a plain vent to atmosphere, now that a wheel
    /// sits in the way of it.
    ///
    /// Must be called after [`Self::advance_intake`] on the same frame: the
    /// shaft's power balance needs that call's compressor power draw, and a
    /// fitted wastegate needs that call's boost controller update.
    pub fn advance_exhaust(
        &mut self,
        dt: f64,
        turbine_upstream: &PortState,
        downstream_pressure: f64,
    ) -> f64 {
        let corrected_speed =
            compressor::corrected_speed(self.shaft.shaft_rpm(), turbine_upstream.temperature);
        self.shaft.advance(
            dt,
            turbine_upstream,
            downstream_pressure,
            &self.turbine_map,
            self.compressor_power,
        );
        let (speed_lo, speed_hi) = self.turbine_map.speed_range();
        let (turbine_flow, _, _) = crate::physics::turbine::turbine_operating_point(
            turbine_upstream,
            downstream_pressure,
            &self.turbine_map,
            corrected_speed.clamp(speed_lo, speed_hi),
        );

        let wastegate_flow = match &self.wastegate {
            Some(wg) => {
                let bias = self
                    .boost_controller
                    .as_ref()
                    .map(|c| c.bias())
                    .unwrap_or(0.0);
                let effective_threshold = wg.spring_preload - bias;
                wg.mass_flow(turbine_upstream, downstream_pressure, effective_threshold)
            }
            None => 0.0,
        };

        turbine_flow + wastegate_flow
    }
}

/// How an engine breathes.
///
/// A naturally aspirated engine has nothing here; a forced-induction one now
/// carries real physics — see [`ForcedInduction`] — as well as the voicing
/// that says what the hardware sounds like. Moved here from `crate::audio`
/// by `docs/TURBO_PLAN.md`'s TB3, once boost stopped being purely a sound:
/// the shaft and voicing fields below are still TB0-era audio proxies
/// (`TurboModel`, `TurboVoicing`, ...), and stay that way until TB6 rewires
/// them onto the real [`ForcedInduction::shaft`] this module now runs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Induction {
    /// No compressor: no whistle, no surge, nothing between the firings.
    NaturallyAspirated,
    /// A turbocharger, with the shaft that drives the sound and its voicing.
    Turbocharged {
        /// The shaft, spooling with throttle and speed.
        shaft: TurboModel,
        /// What that shaft is heard as.
        voice: TurboVoicing,
        /// Optional blow-off / dump valve fitted to the charge pipe.
        blow_off: Option<BlowOffVoicing>,
        /// Optional wastegate chatter voicing under high boost.
        wastegate: Option<WastegateVoicing>,
    },
    /// A Roots or twin-screw positive displacement supercharger, crank-order locked.
    RootsSupercharged {
        /// What the supercharger sounds like.
        voice: RootsVoicing,
    },
    /// A centrifugal supercharger, shaft-order whistled but belt-locked to the crank.
    CentrifugalSupercharged {
        /// What the supercharger sounds like.
        voice: CentrifugalVoicing,
        /// Optional blow-off / dump valve fitted to the charge pipe.
        blow_off: Option<BlowOffVoicing>,
    },
}

impl Induction {
    /// A small, fast-spooling single on a four-cylinder.
    ///
    /// Low wheel inertia is the whole character: it is up before the driver has
    /// finished asking, and its small wheel turns fast enough to whistle higher
    /// than anything else here. This is what people mean when they say "turbo".
    pub const fn small_single() -> Self {
        Self::Turbocharged {
            shaft: TurboModel {
                max_shaft_rpm: 190_000.0,
                reference_engine_rpm: 6_900.0,
                spool_up: 0.42,
                spool_down: 0.28,
                shaft_rpm: 0.0,
            },
            voice: TurboVoicing {
                order: 2.3,
                reference_rpm: 160_000.0,
                level: 0.028,
            },
            blow_off: None,
            wastegate: None,
        }
    }

    /// A pair of mid-sized turbos, one per bank or one per three cylinders.
    ///
    /// Splitting the flow across two wheels halves what each has to swallow, so
    /// twins spool nearly as readily as a small single while carrying an engine
    /// twice the size — heard as a whistle that arrives early but never quite
    /// dominates the note underneath it.
    pub const fn twin() -> Self {
        Self::Turbocharged {
            shaft: TurboModel {
                max_shaft_rpm: 172_000.0,
                reference_engine_rpm: 7_100.0,
                spool_up: 0.50,
                spool_down: 0.32,
                shaft_rpm: 0.0,
            },
            voice: TurboVoicing {
                order: 1.9,
                reference_rpm: 145_000.0,
                level: 0.020,
            },
            blow_off: None,
            wastegate: None,
        }
    }

    /// One large turbo sized for top end rather than response.
    ///
    /// A big wheel has real rotational inertia, so it is slow to answer the
    /// throttle and slow to give the speed back; it also turns more slowly for
    /// the same air, which drops the tone. The result is the laggy, low, heavy
    /// whistle of a single-turbo conversion. It is mixed the loudest of the
    /// three, which is a voicing choice rather than anything the shaft model
    /// derives: a wheel this size is the loudest thing on the engine, and its
    /// lower tone can carry that level without becoming shrill.
    pub const fn large_single() -> Self {
        Self::Turbocharged {
            shaft: TurboModel {
                max_shaft_rpm: 138_000.0,
                reference_engine_rpm: 7_200.0,
                spool_up: 0.95,
                spool_down: 0.55,
                shaft_rpm: 0.0,
            },
            voice: TurboVoicing {
                order: 1.5,
                reference_rpm: 116_000.0,
                level: 0.031,
            },
            blow_off: None,
            wastegate: None,
        }
    }

    /// The variable-geometry turbine a diesel wears.
    ///
    /// Vanes that swivel shut at low speed keep the turbine's effective area
    /// small exactly where a fixed wheel would still be waiting for gas, so a
    /// VGT is on boost from just above idle and has no wastegate at all — the
    /// vanes are the wastegate. It also has no blow-off valve, because there is
    /// no throttle plate for the charge to slam into on a lift: a diesel's
    /// inlet tract is open from the filter to the valve at every load it ever
    /// sees. The engine under it turns to five thousand rather than seven, so
    /// the shaft reference is lower and the whistle sits lower with it.
    pub const fn variable_geometry() -> Self {
        Self::Turbocharged {
            shaft: TurboModel {
                max_shaft_rpm: 165_000.0,
                reference_engine_rpm: 4_600.0,
                spool_up: 0.30,
                spool_down: 0.42,
                shaft_rpm: 0.0,
            },
            voice: TurboVoicing {
                order: 2.0,
                reference_rpm: 132_000.0,
                level: 0.026,
            },
            blow_off: None,
            wastegate: None,
        }
    }

    /// A twin-screw / Roots-type positive displacement supercharger.
    ///
    /// Driven directly by belt from the crankshaft, its whine frequency is
    /// locked to crank speed times pulley ratio times rotor lobe count with zero
    /// spool lag.
    pub const fn roots() -> Self {
        Self::RootsSupercharged {
            voice: RootsVoicing {
                belt_ratio: 2.1,
                lobes: 4,
                level: 0.030,
            },
        }
    }

    /// A centrifugal supercharger.
    ///
    /// Driven by internal step-up planetary gear transmission and belt from the
    /// crankshaft, its impeller speed is belt-locked to crank speed with zero
    /// spool lag, producing high-frequency shaft-order compressor whistle.
    pub const fn centrifugal() -> Self {
        Self::CentrifugalSupercharged {
            voice: CentrifugalVoicing {
                gear_ratio: 9.2,
                order: 1.8,
                level: 0.024,
            },
            blow_off: None,
        }
    }

    /// Equips a blow-off / dump valve to this forced-induction configuration.
    pub const fn with_blow_off(self, bov: BlowOffVoicing) -> Self {
        match self {
            Self::Turbocharged {
                shaft,
                voice,
                blow_off: _,
                wastegate,
            } => Self::Turbocharged {
                shaft,
                voice,
                blow_off: Some(bov),
                wastegate,
            },
            Self::CentrifugalSupercharged { voice, blow_off: _ } => Self::CentrifugalSupercharged {
                voice,
                blow_off: Some(bov),
            },
            other => other,
        }
    }

    /// Equips wastegate chatter to this turbocharged configuration.
    pub const fn with_wastegate(self, wg: WastegateVoicing) -> Self {
        match self {
            Self::Turbocharged {
                shaft,
                voice,
                blow_off,
                wastegate: _,
            } => Self::Turbocharged {
                shaft,
                voice,
                blow_off,
                wastegate: Some(wg),
            },
            other => other,
        }
    }

    /// Whether a compressor is fitted at all.
    pub fn is_forced(&self) -> bool {
        matches!(
            self,
            Self::Turbocharged { .. }
                | Self::RootsSupercharged { .. }
                | Self::CentrifugalSupercharged { .. }
        )
    }

    /// The shaft to run, if there is one.
    pub fn shaft(&self) -> Option<TurboModel> {
        match *self {
            Self::NaturallyAspirated
            | Self::RootsSupercharged { .. }
            | Self::CentrifugalSupercharged { .. } => None,
            Self::Turbocharged { shaft, .. } => Some(shaft),
        }
    }

    /// The voicing to mix, if there is one.
    pub fn voice(&self) -> Option<TurboVoicing> {
        match *self {
            Self::NaturallyAspirated
            | Self::RootsSupercharged { .. }
            | Self::CentrifugalSupercharged { .. } => None,
            Self::Turbocharged { voice, .. } => Some(voice),
        }
    }

    /// The Roots supercharger voicing, if one is fitted.
    pub fn roots_voice(&self) -> Option<RootsVoicing> {
        match *self {
            Self::RootsSupercharged { voice } => Some(voice),
            _ => None,
        }
    }

    /// The centrifugal supercharger voicing, if one is fitted.
    pub fn centrifugal_voice(&self) -> Option<CentrifugalVoicing> {
        match *self {
            Self::CentrifugalSupercharged { voice, .. } => Some(voice),
            _ => None,
        }
    }

    /// The blow-off valve voicing, if one is fitted.
    pub fn blow_off_voice(&self) -> Option<BlowOffVoicing> {
        match *self {
            Self::Turbocharged { blow_off, .. } => blow_off,
            Self::CentrifugalSupercharged { blow_off, .. } => blow_off,
            _ => None,
        }
    }

    /// The wastegate chatter voicing, if one is fitted.
    pub fn wastegate_voice(&self) -> Option<WastegateVoicing> {
        match *self {
            Self::Turbocharged { wastegate, .. } => wastegate,
            _ => None,
        }
    }

    /// A two-word label for a dashboard.
    pub fn label(&self) -> &'static str {
        match self {
            Self::NaturallyAspirated => "naturally aspirated",
            Self::Turbocharged { .. } => "turbocharged",
            Self::RootsSupercharged { .. } => "roots supercharged",
            Self::CentrifugalSupercharged { .. } => "centrifugal supercharged",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() <= tol, "expected {b}, got {a} (tol {tol})");
    }

    fn gas() -> GasProperties {
        GasProperties::default()
    }

    fn env() -> Environment {
        Environment::default()
    }

    /// A 4.0 litre V8's worth of plenum on the default 70 mm throttle.
    fn plenum() -> IntakePlenum {
        IntakePlenum::at_ambient(3.2e-3, ThrottleBody::default(), &env(), &gas())
    }

    fn upstream() -> PortState {
        IntakePlenum::ambient_upstream(&env(), &gas())
    }

    /// The mass flow a bank of cylinders takes at `rpm`, as a volumetric pump.
    ///
    /// A real cylinder swallows a *volume* per cycle and gets whatever mass the
    /// manifold density puts in it, so the draw has to fall with manifold
    /// pressure. A fixed mass draw would happily pump the plenum to absolute
    /// vacuum, which is exactly the error this coupling exists to avoid.
    fn pumped_draw(plenum: &IntakePlenum, displacement: f64, rpm: f64, ve: f64) -> ValveDraw {
        let volume_rate = displacement * (rpm / 60.0) * 0.5 * ve;
        ValveDraw::new(plenum.density() * volume_rate, 900.0)
    }

    #[test]
    fn critical_pressure_ratio_matches_the_isentropic_formula() {
        for gamma in [1.26, 1.35, 1.4] {
            let expected = (2.0 / (gamma + 1.0f64)).powf(gamma / (gamma - 1.0));
            approx(
                ThrottleBody::critical_pressure_ratio(gamma),
                expected,
                1e-12,
            );
        }
        // Air at the charge gamma chokes just above half an atmosphere.
        approx(ThrottleBody::critical_pressure_ratio(1.35), 0.5367, 1e-3);
    }

    #[test]
    fn throttle_area_sweeps_from_the_leak_to_the_full_bore() {
        let body = ThrottleBody::default();
        let bore = body.bore_area();

        approx(body.open_area(0.0), bore * body.leak_area_fraction, 1e-12);
        approx(body.open_area(1.0), bore, 1e-12);
        approx(
            body.effective_area(1.0),
            body.discharge_coefficient * bore,
            1e-12,
        );

        // Monotonic in pedal, and gentle where it needs to be: the first 10 %
        // of travel must open far less area than the last 10 %.
        let mut previous = body.open_area(0.0);
        for i in 1..=100 {
            let area = body.open_area(i as f64 / 100.0);
            assert!(area >= previous, "area went backwards at TPS {i}%");
            previous = area;
        }
        let first = body.open_area(0.1) - body.open_area(0.0);
        let last = body.open_area(1.0) - body.open_area(0.9);
        assert!(
            first < last * 0.5,
            "tip-in is too aggressive: {first} vs {last}"
        );
    }

    #[test]
    fn throttle_area_is_clamped_outside_the_pedal_range() {
        let body = ThrottleBody::default();
        approx(body.open_area(-1.0), body.open_area(0.0), 1e-12);
        approx(body.open_area(7.0), body.open_area(1.0), 1e-12);
    }

    #[test]
    fn throttle_flow_chokes_below_the_critical_pressure_ratio() {
        let body = ThrottleBody::default();
        let up = upstream();
        let critical = ThrottleBody::critical_pressure_ratio(up.gamma);

        let at = |ratio: f64| {
            let mut manifold = up;
            manifold.pressure = up.pressure * ratio;
            (
                body.mass_flow(1.0, &up, &manifold),
                body.is_choked(&up, manifold.pressure),
            )
        };

        let (deep, deep_choked) = at(0.05);
        let (mid, mid_choked) = at(0.30);
        let (edge, edge_choked) = at(critical * 0.999);
        assert!(deep_choked && mid_choked && edge_choked);

        // The defining property: below critical the throat cannot hear the
        // manifold, so pulling more vacuum buys no more air.
        approx(mid, deep, deep.abs() * 1e-9);
        approx(edge, deep, deep.abs() * 1e-6);

        // And it equals the analytic sonic plateau.
        let g: f64 = up.gamma;
        let psi = g.sqrt() * (2.0 / (g + 1.0)).powf((g + 1.0) / (2.0 * (g - 1.0)));
        let expected = body.effective_area(1.0) * up.pressure
            / (up.gas_constant * up.temperature).sqrt()
            * psi;
        approx(deep, expected, expected * 1e-9);

        // Just above critical it must already be falling away from the plateau.
        let (subsonic, subsonic_choked) = at(critical * 1.05);
        assert!(!subsonic_choked);
        assert!(subsonic < deep, "subsonic flow should be below the plateau");
    }

    #[test]
    fn throttle_flow_vanishes_with_no_pressure_difference() {
        let body = ThrottleBody::default();
        let up = upstream();
        approx(body.mass_flow(1.0, &up, &up), 0.0, 1e-12);
    }

    #[test]
    fn throttle_flow_reverses_when_the_manifold_is_above_the_airbox() {
        let body = ThrottleBody::default();
        let up = upstream();
        let mut boosted = up;
        boosted.pressure = up.pressure * 1.8;
        assert!(
            body.mass_flow(1.0, &up, &boosted) < 0.0,
            "flow must run back out of the plate"
        );
    }

    #[test]
    fn a_shut_throttle_still_passes_the_leak_but_no_more() {
        let body = ThrottleBody::default();
        let up = upstream();
        let mut manifold = up;
        manifold.pressure = 20_000.0;

        let shut = body.mass_flow(0.0, &up, &manifold);
        let open = body.mass_flow(1.0, &up, &manifold);
        assert!(shut > 0.0, "a shut plate is not a seal");
        assert!(
            shut < open * 0.02,
            "the leak is doing too much work: {shut} vs {open}"
        );

        let sealed = ThrottleBody::new(0.070, 0.9, 0.0);
        approx(sealed.mass_flow(0.0, &up, &manifold), 0.0, 1e-12);
    }

    #[test]
    fn the_ideal_gas_closure_holds_after_every_step() {
        let mut p = plenum();
        let up = upstream();
        for i in 0..200 {
            let tps = (i as f64 / 200.0).sin().abs();
            let draw = pumped_draw(&p, 4.0e-3, 3000.0, 0.85);
            p.advance(1.0e-3, tps, &up, &[draw]);
            approx(
                p.pressure(),
                p.mass * p.gas_constant * p.temperature / p.volume,
                1e-9,
            );
            assert!(p.pressure().is_finite() && p.pressure() > 0.0);
        }
    }

    #[test]
    fn an_open_throttle_fills_to_ambient_and_stops_there() {
        let mut p = IntakePlenum::new(
            3.2e-3,
            ThrottleBody::default(),
            20_000.0,
            env().temperature,
            gas().r_unburned,
            gas().gamma_unburned,
        );
        let up = upstream();

        for _ in 0..2_000 {
            p.advance(1.0e-3, 1.0, &up, &[]);
        }

        // Equalises, and critically does not sail past: an unclamped explicit
        // plenum overshoots here and quietly supercharges the engine.
        approx(p.pressure(), up.pressure, up.pressure * 1e-4);
        assert!(
            p.pressure() <= up.pressure * (1.0 + 1e-6),
            "a throttle cannot supercharge: {} Pa",
            p.pressure()
        );
    }

    /// Independent check on the energy equation against a closed-form result.
    ///
    /// Charging a rigid, adiabatic, initially evacuated vessel from a large
    /// reservoir is the textbook transient-filling problem, and its answer does
    /// not depend on the orifice at all:
    ///
    /// ```text
    /// u_final = h_reservoir   =>   c_v T_final = c_p T_up   =>   T_final = gamma T_up
    /// ```
    ///
    /// Every incoming parcel arrives with its flow work included, and with no
    /// wall and no outflow there is nowhere for that work to go but internal
    /// energy. If the `c_p` on the inflow term were mistakenly a `c_v`, or the
    /// `c_v T dm/dt` product-rule term were dropped, this lands on `T_up`
    /// instead and the error would be invisible in a steady-state test.
    #[test]
    fn adiabatic_filling_of_an_evacuated_plenum_lands_on_gamma_times_upstream() {
        let gas = gas();
        let up = upstream();
        let mut p = IntakePlenum::new(
            3.2e-3,
            ThrottleBody::default(),
            up.pressure * 1e-4,
            up.temperature,
            gas.r_unburned,
            gas.gamma_unburned,
        );
        p.wall_conductance = 0.0;

        for _ in 0..5_000 {
            p.advance(1.0e-3, 1.0, &up, &[]);
        }

        let expected = p.gamma * up.temperature;
        // The residual gap is the charge that was already in there at ambient
        // temperature, which is 0.01 % of the final mass.
        approx(p.temperature, expected, expected * 2e-3);
        approx(p.pressure(), up.pressure, up.pressure * 1e-4);
    }

    #[test]
    fn a_shut_throttle_under_draw_pulls_a_deep_vacuum() {
        let mut p = plenum();
        let up = upstream();

        for _ in 0..3_000 {
            let draw = pumped_draw(&p, 4.0e-3, 2000.0, 0.85);
            p.advance(1.0e-3, 0.0, &up, &[draw]);
        }

        let map = p.pressure();
        assert!(
            map < up.pressure * 0.5,
            "a shut throttle at 2000 rpm should be a strong vacuum, got {map} Pa"
        );
        assert!(map > 0.0 && map.is_finite());
        assert!(
            p.gauge_pressure(up.pressure) < 0.0,
            "gauge pressure must read vacuum"
        );
    }

    /// The headline transient: 0 % to 100 % pedal in one frame.
    #[test]
    fn a_throttle_snap_decays_the_manifold_vacuum_and_stabilizes() {
        let mut p = plenum();
        let up = upstream();
        let ambient = up.pressure;
        let rpm = 2000.0;
        let displacement = 4.0e-3;
        let frame = 1.0e-3;

        // Settle on a shut throttle first, so the snap starts from real vacuum.
        for _ in 0..3_000 {
            let draw = pumped_draw(&p, displacement, rpm, 0.85);
            p.advance(frame, 0.0, &up, &[draw]);
        }
        let idle_map = p.pressure();
        assert!(
            idle_map < ambient * 0.5,
            "expected vacuum before the snap, got {idle_map} Pa"
        );

        // Snap to wide open and watch the vacuum collapse.
        let mut trace = Vec::new();
        let mut peak_charge_temperature = p.temperature;
        for _ in 0..600 {
            let draw = pumped_draw(&p, displacement, rpm, 0.85);
            p.advance(frame, 1.0, &up, &[draw]);
            trace.push(p.pressure());
            peak_charge_temperature = peak_charge_temperature.max(p.temperature);
        }

        // 1. The vacuum decays monotonically for as long as it is decaying.
        //
        //    The plenum has two relaxation modes and they are decades apart: the
        //    mass channel equalises with the airbox in a few milliseconds, while
        //    the charge temperature — spiked by the fill itself — bleeds off to
        //    the wall over hundreds. Once the plate has stopped flowing, the
        //    still-cooling charge drops `P = m R T / V` by a pascal or so before
        //    the plate tops the mass back up. So monotonicity is asserted where
        //    it is physically required, over the fill, and the settled tail is
        //    checked separately below. Asserting it over both would be
        //    asserting that the manifold has one state, and it has two.
        let fill_ceiling = ambient * 0.995;
        let mut filling = trace
            .iter()
            .position(|&p| p >= fill_ceiling)
            .expect("the manifold never filled");
        filling = filling.max(1);
        for pair in trace[..=filling].windows(2) {
            assert!(
                pair[1] > pair[0],
                "manifold pressure fell during the fill: {pair:?}"
            );
        }
        assert!(
            trace[0] > idle_map,
            "the first frame after the snap should already be filling"
        );

        // The fill is a few milliseconds, not a few hundred: this is a plenum
        // being equalised through a 70 mm bore, not a slow leak.
        assert!(
            filling < 20,
            "a wide-open 70 mm bore should fill 3.2 litres in milliseconds, took {filling} ms"
        );

        // 2. It gets most of the way there fast. A 3.2 litre plenum behind a
        //    70 mm bore has a filling time of a few milliseconds, so 50 ms of
        //    pedal is many time constants.
        let at_50ms = trace[49];
        assert!(
            at_50ms > idle_map + 0.9 * (ambient - idle_map),
            "fill is far too slow: {at_50ms} Pa after 50 ms"
        );

        // 3. It stabilises, and at a pressure just below ambient — the residual
        //    depression is the pumping loss across a wide-open plate, which is
        //    small but must not be zero.
        let settled = *trace.last().unwrap();
        let previous = trace[trace.len() - 51];
        assert!(
            (settled - previous).abs() < ambient * 1e-4,
            "still moving at the end of the pull: {previous} -> {settled} Pa"
        );
        assert!(
            settled > ambient * 0.9,
            "wide-open throttle should be near ambient, got {settled} Pa"
        );
        assert!(
            settled < ambient,
            "a naturally aspirated manifold cannot exceed ambient, got {settled} Pa"
        );

        // 4. Past the fill, nothing rings: the tail is a slow thermal drift of
        //    a few pascals, not an oscillation about ambient. An explicit
        //    plenum stepped past its filling time constant fails here.
        let tail = &trace[filling..];
        let (lo, hi) = tail
            .iter()
            .fold((f64::MAX, f64::MIN), |(lo, hi), &p| (lo.min(p), hi.max(p)));
        assert!(
            hi - lo < ambient * 1e-3,
            "the settled manifold is ringing: {lo} .. {hi} Pa"
        );

        // 5. The gauge agrees: vacuum has decayed to almost nothing.
        assert!(p.gauge_pressure(ambient) < 0.0);
        assert!(p.gauge_pressure(ambient) > -0.1 * ambient);

        // 6. Filling a near-vacuum is irreversible, so the charge arrives hot:
        //    the flow work done pushing air into the plenum shows up as
        //    temperature, heading for `gamma * T_up` in the limit of filling
        //    from nothing. A plenum that fills isothermally has lost that term.
        let limit = p.gamma * up.temperature;
        assert!(
            peak_charge_temperature > up.temperature + 40.0,
            "the fill should have heated the charge, peaked at {peak_charge_temperature} K"
        );
        assert!(
            peak_charge_temperature < limit,
            "charge cannot exceed the filling limit {limit} K, got {peak_charge_temperature} K"
        );
        // And it sheds that heat afterwards, back towards the wall.
        assert!(
            p.temperature < peak_charge_temperature - 20.0,
            "the hot charge should have relaxed, sitting at {} K",
            p.temperature
        );
    }

    #[test]
    fn a_throttle_lift_restores_the_vacuum() {
        let mut p = plenum();
        let up = upstream();
        let draws = |p: &IntakePlenum| [pumped_draw(p, 4.0e-3, 2500.0, 0.85)];

        for _ in 0..1_000 {
            p.advance(1.0e-3, 1.0, &up, &draws(&p));
        }
        let wide_open = p.pressure();

        for _ in 0..1_000 {
            p.advance(1.0e-3, 0.0, &up, &draws(&p));
        }
        let lifted = p.pressure();

        assert!(
            lifted < wide_open * 0.5,
            "lifting off must re-establish vacuum: {wide_open} -> {lifted} Pa"
        );
    }

    #[test]
    fn manifold_pressure_rises_monotonically_with_pedal() {
        let up = upstream();
        let mut previous = 0.0;

        for step in 0..=10 {
            let tps = step as f64 / 10.0;
            let mut p = plenum();
            for _ in 0..2_000 {
                let draw = pumped_draw(&p, 4.0e-3, 2500.0, 0.85);
                p.advance(1.0e-3, tps, &up, &[draw]);
            }
            let map = p.pressure();
            assert!(
                map > previous,
                "MAP did not rise from TPS {tps}: {previous} -> {map} Pa"
            );
            previous = map;
        }
        assert!(previous < up.pressure, "still cannot exceed ambient");
    }

    #[test]
    fn reversion_from_a_hot_cylinder_warms_the_plenum() {
        let mut p = plenum();
        // Isolate the valve term from the wall term.
        p.wall_conductance = 0.0;
        let up = upstream();
        let before = p.temperature;

        // Overlap reversion: mass pushed back up the runner at exhaust
        // temperature, with the plate shut so nothing cool dilutes it.
        let reversion = ValveDraw::new(-2.0e-3, 1_100.0);
        for _ in 0..50 {
            p.advance(1.0e-4, 0.0, &up, &[reversion]);
        }

        assert!(
            p.temperature > before + 1.0,
            "hot reversion must heat the charge: {before} -> {} K",
            p.temperature
        );
        assert!(p.pressure() > up.pressure * 0.9, "and pressurise it");
    }

    #[test]
    fn a_heat_soaked_wall_warms_the_charge_and_thins_it() {
        let up = upstream();
        let draw_rate = |p: &IntakePlenum| [pumped_draw(p, 4.0e-3, 2500.0, 0.85)];

        let mut cold = plenum();
        cold.wall_temperature = env().temperature;
        cold.wall_conductance = 5.0;

        let mut soaked = plenum();
        soaked.wall_temperature = env().temperature + 80.0;
        soaked.wall_conductance = 5.0;

        for _ in 0..2_000 {
            cold.advance(1.0e-3, 0.6, &up, &draw_rate(&cold));
            soaked.advance(1.0e-3, 0.6, &up, &draw_rate(&soaked));
        }

        assert!(
            soaked.temperature > cold.temperature + 1.0,
            "the hot wall must raise charge temperature: {} vs {} K",
            soaked.temperature,
            cold.temperature
        );
        assert!(
            soaked.density() < cold.density(),
            "warmer charge at a similar pressure must be less dense"
        );
    }

    #[test]
    fn a_coarse_frame_reaches_the_same_steady_state_as_a_fine_one() {
        let up = upstream();
        let settle = |frame: f64, frames: usize| {
            let mut p = plenum();
            for _ in 0..frames {
                let draw = pumped_draw(&p, 4.0e-3, 2500.0, 0.85);
                p.advance(frame, 0.45, &up, &[draw]);
            }
            p.pressure()
        };

        // Same 2 s of simulated time at 0.1 ms and at 20 ms per frame. The
        // internal sub-stepping is what makes these agree; a single RK4 step
        // per 20 ms frame spans several filling time constants.
        let fine = settle(1.0e-4, 20_000);
        let coarse = settle(2.0e-2, 100);
        approx(coarse, fine, fine * 5e-3);
    }

    #[test]
    fn the_plenum_survives_nonsense_inputs() {
        let mut p = plenum();
        let up = upstream();
        let reference = p.pressure();

        for dt in [0.0, -1.0e-3, f64::NAN, f64::INFINITY] {
            p.advance(dt, 1.0, &up, &[]);
            approx(p.pressure(), reference, 1e-9);
        }

        // A NaN pedal is treated as shut, and a NaN draw is ignored rather
        // than laundered into the state.
        p.advance(1.0e-3, f64::NAN, &up, &[ValveDraw::new(f64::NAN, 300.0)]);
        assert!(p.pressure().is_finite() && p.pressure() > 0.0);
        assert!(p.temperature.is_finite() && p.temperature > 0.0);

        // Absurd geometry is clamped at construction, not left to divide by zero.
        let tiny = IntakePlenum::new(0.0, ThrottleBody::new(0.0, 5.0, -1.0), 0.0, 0.0, 0.0, 0.0);
        assert!(tiny.volume > 0.0);
        assert!(tiny.mass >= MASS_FLOOR);
        assert!(tiny.pressure().is_finite());
    }

    #[test]
    fn a_pumped_dry_plenum_does_not_produce_a_singular_temperature() {
        let mut p = plenum();
        let up = upstream();
        // Sealed plate, absurd draw: the control volume is being emptied faster
        // than anything can refill it.
        p.throttle = ThrottleBody::new(0.070, 0.9, 0.0);
        for _ in 0..1_000 {
            p.advance(1.0e-3, 0.0, &up, &[ValveDraw::new(1.0, 900.0)]);
        }
        assert!(p.mass >= MASS_FLOOR);
        assert!(p.temperature.is_finite());
        assert!((TEMPERATURE_BOUNDS.0..=TEMPERATURE_BOUNDS.1).contains(&p.temperature));
        assert!(p.pressure().is_finite() && p.pressure() >= 0.0);
    }

    #[test]
    fn the_gas_property_accessors_agree_with_their_definitions() {
        let p = plenum();
        approx(p.cv(), p.gas_constant / (p.gamma - 1.0), 1e-12);
        approx(p.cp(), p.gamma * p.cv(), 1e-12);
        // Mayer's relation, as a cross-check that c_p and c_v are consistent.
        approx(p.cp() - p.cv(), p.gas_constant, 1e-9);
        approx(
            p.speed_of_sound(),
            (p.gamma * p.gas_constant * p.temperature).sqrt(),
            1e-12,
        );
        approx(p.density(), p.mass / p.volume, 1e-12);
        // ~340 m/s in ambient charge air.
        assert!((300.0..380.0).contains(&p.speed_of_sound()));
    }

    #[test]
    fn the_throttle_flow_helper_matches_the_orifice_it_wraps() {
        let mut p = plenum();
        let up = upstream();
        for _ in 0..500 {
            let draw = pumped_draw(&p, 4.0e-3, 3000.0, 0.85);
            p.advance(1.0e-3, 0.3, &up, &[draw]);
        }
        // The convenience wrapper must solve the same orifice against the same
        // closure the RK4 stages use, or telemetry and physics disagree.
        approx(
            p.throttle_flow(0.3, &up),
            p.throttle.mass_flow(0.3, &up, &p.port_state()),
            1e-15,
        );
        assert!(p.throttle_flow(0.3, &up) > 0.0, "should be drawing air in");
        assert!(
            p.throttle_flow(1.0, &up) > p.throttle_flow(0.3, &up),
            "more pedal at the same manifold state must pass more air"
        );
    }

    #[test]
    fn wall_heat_flow_is_signed_out_of_the_charge() {
        let mut p = plenum();
        p.wall_conductance = 5.0;

        // Charge hotter than the wall: heat leaves the gas, so positive.
        p.wall_temperature = 300.0;
        p.temperature = 400.0;
        approx(p.wall_heat_flow(), 5.0 * 100.0, 1e-9);

        // Heat-soaked wall: the sign flips and the charge is being warmed.
        p.wall_temperature = 400.0;
        p.temperature = 300.0;
        approx(p.wall_heat_flow(), -5.0 * 100.0, 1e-9);

        // In equilibrium, and with no conductance, it is exactly nothing.
        p.temperature = 400.0;
        approx(p.wall_heat_flow(), 0.0, 1e-12);
        p.wall_conductance = 0.0;
        p.temperature = 300.0;
        approx(p.wall_heat_flow(), 0.0, 1e-12);
    }

    #[test]
    fn the_port_state_is_the_boundary_condition_the_valves_see() {
        let mut p = plenum();
        let up = upstream();
        for _ in 0..1_000 {
            let draw = pumped_draw(&p, 4.0e-3, 3000.0, 0.85);
            p.advance(1.0e-3, 0.25, &up, &[draw]);
        }

        let port = p.port_state();
        approx(port.pressure, p.pressure(), 1e-9);
        approx(port.temperature, p.temperature, 1e-12);
        approx(port.gas_constant, p.gas_constant, 1e-12);
        approx(port.gamma, p.gamma, 1e-12);
        approx(port.burned_fraction, 0.0, 1e-12);

        // It must be a *throttled* condition, not ambient: that difference is
        // the entire point of feeding it to the cylinders.
        assert!(port.pressure < up.pressure * 0.9);
        approx(port.cp(), p.cp(), 1e-9);
    }

    fn turbine_map() -> TurbineMap {
        use crate::physics::turbine::{TurbineMapPoint, TurbineSpeedLine};
        let point = |expansion_ratio: f64, reduced_flow: f64, efficiency: f64| TurbineMapPoint {
            expansion_ratio,
            reduced_flow,
            efficiency,
        };
        TurbineMap::new(vec![
            TurbineSpeedLine::new(
                40_000.0,
                vec![
                    point(1.0, 0.015, 0.45),
                    point(1.5, 0.045, 0.66),
                    point(2.5, 0.075, 0.58),
                ],
            ),
            TurbineSpeedLine::new(
                140_000.0,
                vec![
                    point(1.0, 0.025, 0.50),
                    point(1.8, 0.085, 0.72),
                    point(3.0, 0.135, 0.60),
                ],
            ),
        ])
    }

    fn forced_induction() -> ForcedInduction {
        ForcedInduction::new(
            crate::physics::compressor::CompressorMap::stock(
                crate::physics::compressor::FrameSize::Small,
            ),
            turbine_map(),
            6.0e-5,
            0.98,
            BearingType::Journal,
            1.5e-3,
            None,
            &env(),
            &gas(),
        )
    }

    /// Spins the shaft up over `seconds` under a hot, high pressure-ratio
    /// exhaust and a low downstream pressure, the way a boosted engine would.
    fn spool_up(fi: &mut ForcedInduction, seconds: f64) {
        let hot_upstream = PortState {
            pressure: 250_000.0,
            temperature: 1000.0,
            gas_constant: gas().r_burned,
            gamma: gas().gamma_burned,
            burned_fraction: 1.0,
        };
        let dt = 1.0e-3;
        let mut t = 0.0;
        while t < seconds {
            fi.advance_exhaust(dt, &hot_upstream, env().pressure);
            t += dt;
        }
    }

    /// Runs `advance_intake` at a fixed throttle draw until the charge pipe's
    /// capacitance has settled, and returns the resulting pressure.
    fn settled_charge_pipe_pressure(fi: &mut ForcedInduction, throttle_flow: f64) -> f64 {
        let mut pressure = fi.charge_pipe.pressure();
        for _ in 0..2_000 {
            pressure = fi
                .advance_intake(1.0e-3, throttle_flow, env().pressure, &env(), &gas())
                .pressure;
        }
        pressure
    }

    #[test]
    fn a_spun_up_shaft_raises_the_charge_pipe_further_above_ambient() {
        // Even a stalled shaft reads as the map's slowest cataloged speed
        // line — the same nearest-neighbour convention
        // `turbine::turbine_operating_point` documents — so "at rest" already
        // carries a small pressure ratio. What must still hold is that a
        // genuinely spun-up shaft settles well past that floor.
        let mut fi = forced_induction();
        let at_rest = settled_charge_pipe_pressure(&mut fi, 0.02);

        let mut spun = forced_induction();
        spool_up(&mut spun, 2.0);
        assert!(
            spun.shaft.shaft_rpm() > 1_000.0,
            "shaft failed to spool: {}",
            spun.shaft.shaft_rpm()
        );

        let boosted = settled_charge_pipe_pressure(&mut spun, 0.02);
        assert!(
            boosted > at_rest * 1.1,
            "spinning the shaft must settle the charge pipe well past its stalled reading: \
             at rest {at_rest}, boosted {boosted}"
        );
    }

    #[test]
    fn an_intercooler_lowers_the_charge_temperature_at_the_same_operating_point() {
        let mut hot = forced_induction();
        let mut cooled = ForcedInduction {
            intercooler: Some(Intercooler {
                core_volume: 1.0e-3,
                effectiveness: 0.7,
                loss_coefficient: 0.0,
            }),
            ..forced_induction()
        };

        spool_up(&mut hot, 2.0);
        spool_up(&mut cooled, 2.0);
        // Match the shafts exactly so the comparison isolates the intercooler.
        cooled.shaft = hot.shaft;

        let hot_state = hot.advance_intake(1.0e-3, 0.03, env().pressure, &env(), &gas());
        let cooled_state = cooled.advance_intake(1.0e-3, 0.03, env().pressure, &env(), &gas());
        assert!(
            cooled_state.temperature < hot_state.temperature,
            "intercooled charge ({}) must be cooler than bare ({})",
            cooled_state.temperature,
            hot_state.temperature
        );
    }

    #[test]
    fn compressor_power_rises_with_shaft_speed() {
        let mut idle = forced_induction();
        idle.advance_intake(1.0e-3, 0.02, env().pressure, &env(), &gas());
        let idle_power = idle.compressor_power;

        let mut spun = forced_induction();
        spool_up(&mut spun, 2.0);
        spun.advance_intake(1.0e-3, 0.02, env().pressure, &env(), &gas());
        assert!(
            spun.compressor_power > idle_power,
            "spinning the shaft must draw more compressor power: idle {idle_power}, spun {}",
            spun.compressor_power
        );
    }

    #[test]
    fn a_real_expansion_ratio_spools_the_shaft_and_a_flat_one_does_not() {
        let mut spooling = forced_induction();
        spool_up(&mut spooling, 1.0);
        assert!(spooling.shaft.shaft_rpm() > 100.0);

        let mut idle = forced_induction();
        let flat = PortState {
            pressure: env().pressure,
            temperature: env().temperature,
            gas_constant: gas().r_burned,
            gamma: gas().gamma_burned,
            burned_fraction: 1.0,
        };
        for _ in 0..1_000 {
            idle.advance_exhaust(1.0e-3, &flat, env().pressure);
        }
        assert!(idle.shaft.shaft_rpm() < 1.0);
    }
}
