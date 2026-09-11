//! Open-system thermodynamics: the RK4 solver that actually moves the cylinder
//! state vector forward in time.
//!
//! [`cylinder`](super::cylinder) supplies the geometry and the algebraic
//! pressure closure; this module supplies the derivatives and the integrator
//! that consumes them. The integrated vector is
//!
//! ```text
//! Y = [T, m, x_b, I_knock]^T
//! ```
//!
//! with crank angle carried analytically (`theta(t) = theta_0 + omega t`) rather
//! than integrated, so the phase can never drift away from the shaft.
//!
//! The temperature derivative is the open-system first law for a well-stirred
//! variable-volume control volume:
//!
//! ```text
//! m c_v dT/dt = Q_dot_comb - Q_dot_wall - P dV/dt + sum_j m_dot_j (h_j - u)
//! dm/dt       = sum_j m_dot_j
//! ```
//!
//! where `u = c_v T` is the specific internal energy of the cylinder contents
//! and `h_j` the specific enthalpy of stream `j` evaluated at *its own* upstream
//! state. Writing the flux term as `m_dot_j (h_j - u)` rather than `m_dot_j h_j`
//! folds the `u dm/dt` bookkeeping in, which is what keeps a pure blowdown
//! (no heat, no work) isentropic to round-off.
//!
//! Everything is SI: metres, kilograms, seconds, Kelvin, Pascals, radians.

use std::f64::consts::PI;

use crate::environment::Environment;
use crate::physics::cylinder::{
    deg, wrap_cycle, CylinderGeometry, CylinderState, GasProperties, CYCLE_ANGLE,
};

/// Lower heating value of pump gasoline [J/kg].
pub const GASOLINE_LHV: f64 = 44.0e6;
/// Stoichiometric air/fuel mass ratio for gasoline [-].
pub const STOICH_AFR: f64 = 14.7;
/// Standard atmosphere, used to non-dimensionalise the knock correlation [Pa].
pub const ATMOSPHERE: f64 = 101_325.0;

/// Smallest mass we will divide by, so an emptied cylinder cannot produce an
/// infinite `dT/dt` [kg].
const MASS_FLOOR: f64 = 1e-9;

// ---------------------------------------------------------------------------
// Combustion: Wiebe heat release
// ---------------------------------------------------------------------------

/// Single-zone Wiebe burn profile.
///
/// ```text
/// x_b(theta) = 1 - exp[-a * ((theta - theta_0) / delta_theta)^(n + 1)]
/// ```
///
/// `a` sets how much of the charge is consumed by the end of the nominal
/// duration (`a = 5` leaves 0.7 % unburned, the usual convention) and `n` sets
/// the skew of the rate curve.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WiebeProfile {
    /// Start of heat release (spark, plus flame development delay) [rad, cycle coords].
    pub spark_angle: f64,
    /// Nominal burn duration `delta_theta` [rad].
    pub duration: f64,
    /// Wiebe efficiency parameter `a` [-].
    pub efficiency_parameter: f64,
    /// Wiebe form factor `n` [-].
    pub form_factor: f64,
    /// Fraction of the fuel's chemical energy that actually shows up as heat [-].
    pub combustion_efficiency: f64,
}

impl Default for WiebeProfile {
    /// 20 degrees BTDC spark, 60 degree burn, `a = 5`, `n = 2`.
    ///
    /// Compression TDC sits at 360 degrees in cycle coordinates, so 20 BTDC is
    /// 340 degrees.
    fn default() -> Self {
        Self::new(deg(340.0), deg(60.0), 5.0, 2.0, 0.97)
    }
}

impl WiebeProfile {
    /// Builds a profile, clamping the shape parameters into the range where the
    /// exponent and its derivative are both finite and monotone.
    pub fn new(
        spark_angle: f64,
        duration: f64,
        efficiency_parameter: f64,
        form_factor: f64,
        combustion_efficiency: f64,
    ) -> Self {
        Self {
            spark_angle: wrap_cycle(spark_angle),
            // A burn shorter than a tenth of a degree is a delta function the
            // integrator cannot resolve at any usable substep.
            duration: duration.clamp(deg(0.1), CYCLE_ANGLE),
            efficiency_parameter: efficiency_parameter.clamp(0.1, 20.0),
            // n < 0 puts a singularity at theta_0; n > 6 is numerically inert.
            form_factor: form_factor.clamp(0.0, 6.0),
            combustion_efficiency: combustion_efficiency.clamp(0.0, 1.0),
        }
    }

    /// Angle since spark, wrapped into `[0, 720)` [rad].
    ///
    /// Beyond `duration` the burn is over; the caller decides what that means.
    fn phase(&self, theta: f64) -> f64 {
        wrap_cycle(theta - self.spark_angle)
    }

    /// True while heat release is actively happening.
    pub fn is_burning(&self, theta: f64) -> bool {
        self.phase(theta) < self.duration
    }

    /// Analytic burned mass fraction `x_b(theta)` [-].
    ///
    /// Only meaningful from spark to the end of the burn: the solver integrates
    /// `dx_b/dtheta` into the state vector instead of sampling this, because the
    /// state also has to survive dilution by fresh charge and the reset at IVC.
    /// Outside the burn window this reports a fully burned charge.
    pub fn burned_fraction(&self, theta: f64) -> f64 {
        let phase = self.phase(theta);
        if phase >= self.duration {
            return 1.0;
        }
        let tau = phase / self.duration;
        1.0 - (-self.efficiency_parameter * tau.powf(self.form_factor + 1.0)).exp()
    }

    /// Burn rate with respect to crank angle [1/rad].
    ///
    /// ```text
    /// dx_b/dtheta = a (n+1) / dtheta * tau^n * exp(-a tau^(n+1)),  tau = (theta - theta_0)/dtheta
    /// ```
    pub fn dburned_dtheta(&self, theta: f64) -> f64 {
        let phase = self.phase(theta);
        if phase >= self.duration {
            return 0.0;
        }
        let tau = phase / self.duration;
        let a = self.efficiency_parameter;
        let np1 = self.form_factor + 1.0;
        a * np1 / self.duration * tau.powf(self.form_factor) * (-a * tau.powf(np1)).exp()
    }
}

// ---------------------------------------------------------------------------
// Knock: Livengood-Wu autoignition integral
// ---------------------------------------------------------------------------

/// Livengood-Wu autoignition accumulator.
///
/// ```text
/// I_knock = integral( dt / tau(P, T_end) )
/// ```
///
/// The charge is taken to autoignite when `I_knock` reaches 1. The ignition
/// delay `tau` comes from the Douaud-Eyzat fit for gasoline,
///
/// ```text
/// tau = 17.68 ms * (ON/100)^3.402 * (P/1 atm)^-1.7 * exp(3800 K / T_end)
/// ```
///
/// evaluated on the *unburned* end gas, not the bulk mixture: the end gas is
/// the pocket that actually detonates, and it is several hundred Kelvin cooler
/// than the mass-averaged temperature once the flame is running.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KnockModel {
    /// Research octane number of the fuel [-].
    pub octane_number: f64,
    /// Pre-exponential time constant [s].
    pub time_constant: f64,
    /// Octane exponent [-].
    pub octane_exponent: f64,
    /// Pressure exponent (negative: pressure shortens the delay) [-].
    pub pressure_exponent: f64,
    /// Reduced activation temperature `E_a / R_u` [K].
    pub activation_temperature: f64,
}

impl Default for KnockModel {
    /// 95 RON pump fuel with the published Douaud-Eyzat coefficients.
    fn default() -> Self {
        Self {
            octane_number: 95.0,
            time_constant: 17.68e-3,
            octane_exponent: 3.402,
            pressure_exponent: -1.7,
            activation_temperature: 3800.0,
        }
    }
}

impl KnockModel {
    /// Ignition delay of the end gas at a given pressure and temperature [s].
    pub fn ignition_delay(&self, pressure: f64, end_gas_temperature: f64) -> f64 {
        let p_atm = (pressure / ATMOSPHERE).max(1e-3);
        let t = end_gas_temperature.clamp(200.0, 4000.0);
        let tau = self.time_constant
            * (self.octane_number / 100.0).powf(self.octane_exponent)
            * p_atm.powf(self.pressure_exponent)
            * (self.activation_temperature / t).exp();
        // Below ~600 K the exponential overflows into absurd delays; cap both
        // ends so `1/tau` stays a finite, well-scaled derivative.
        tau.clamp(1e-6, 1e9)
    }

    /// Rate of the Livengood-Wu integral, `dI/dt = 1 / tau` [1/s].
    pub fn dknock_dt(&self, pressure: f64, end_gas_temperature: f64) -> f64 {
        1.0 / self.ignition_delay(pressure, end_gas_temperature)
    }
}

// ---------------------------------------------------------------------------
// Wall heat transfer: Woschni
// ---------------------------------------------------------------------------

/// Woschni convective wall heat transfer correlation.
///
/// ```text
/// h_c = 3.26 * B^-0.2 * P_kPa^0.8 * T^-0.55 * w^0.8      [W/(m^2 K)]
/// w   = C1 * S_p_mean + C2 * (V_d T_r)/(P_r V_r) * (P - P_motored)
/// ```
///
/// `C2` is only switched on once combustion starts: it is the term that models
/// the extra charge motion the expanding flame produces, and leaving it live
/// during gas exchange badly over-predicts the loss.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WoschniModel {
    /// Mean combustion-chamber wall temperature [K].
    ///
    /// State, not a setting: [`crate::physics::thermal`] integrates the block it
    /// belongs to and writes this every frame, so a cold engine has a cold
    /// chamber and takes less heat out of its charge. The value a model is built
    /// with is only what it runs on until the first frame has been solved.
    pub wall_temperature: f64,
    /// Velocity coefficient during gas exchange [-].
    pub c1_gas_exchange: f64,
    /// Velocity coefficient during the closed period [-].
    pub c1_closed: f64,
    /// Combustion-driven velocity coefficient [m/(s K)].
    pub c2_combustion: f64,
    /// Global multiplier for tuning against measured heat rejection [-].
    pub scaling: f64,
}

impl Default for WoschniModel {
    /// A liquid-cooled aluminium head running a ~450 K mean wall.
    fn default() -> Self {
        Self {
            wall_temperature: 450.0,
            c1_gas_exchange: 6.18,
            c1_closed: 2.28,
            c2_combustion: 3.24e-3,
            scaling: 1.0,
        }
    }
}

impl WoschniModel {
    /// Heat transfer coefficient [W/(m^2 K)].
    pub fn coefficient(
        &self,
        geometry: &CylinderGeometry,
        pressure: f64,
        temperature: f64,
        gas_velocity: f64,
    ) -> f64 {
        let p_kpa = (pressure / 1000.0).max(1e-6);
        let t = temperature.max(1.0);
        let w = gas_velocity.max(0.1);
        self.scaling
            * 3.26
            * geometry.bore.powf(-0.2)
            * p_kpa.powf(0.8)
            * t.powf(-0.55)
            * w.powf(0.8)
    }

    /// Instantaneous wetted area: crown + head + exposed liner [m^2].
    pub fn surface_area(&self, geometry: &CylinderGeometry, theta: f64) -> f64 {
        2.0 * geometry.piston_area() + PI * geometry.bore * geometry.piston_position(theta)
    }

    /// Cycle-mean wetted area [m^2].
    ///
    /// The instantaneous area swings between the crown and head alone at TDC and
    /// the whole exposed liner at BDC. What the path from the chamber surface
    /// into the block metal sees over a cycle is the mean of that, and the mean
    /// of the slider-crank displacement is the crank radius plus a small
    /// second-order term from the rod obliquity: expanding the radical gives
    /// `<x> = r + r^2 / (4 l)`, since the cosine averages away over a full
    /// revolution and `<sin^2> = 1/2`.
    pub fn mean_surface_area(&self, geometry: &CylinderGeometry) -> f64 {
        let r = geometry.crank_radius();
        let mean_position = r + r * r / (4.0 * geometry.rod_length.max(1e-6));
        2.0 * geometry.piston_area() + PI * geometry.bore * mean_position
    }
}

// ---------------------------------------------------------------------------
// Valves and compressible port flow
// ---------------------------------------------------------------------------

/// One poppet valve: when it opens, how far it lifts, and how well it flows.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ValveEvent {
    /// Opening angle in cycle coordinates [rad].
    pub open_angle: f64,
    /// Angular duration from open to close [rad].
    pub duration: f64,
    /// Peak lift [m].
    pub max_lift: f64,
    /// Valve head diameter [m].
    pub diameter: f64,
    /// Discharge coefficient at the reference area [-].
    pub discharge_coefficient: f64,
}

impl ValveEvent {
    /// Builds an event, clamping to a physical envelope.
    pub fn new(
        open_angle: f64,
        duration: f64,
        max_lift: f64,
        diameter: f64,
        discharge_coefficient: f64,
    ) -> Self {
        Self {
            open_angle: wrap_cycle(open_angle),
            duration: duration.clamp(0.0, CYCLE_ANGLE),
            max_lift: max_lift.max(0.0),
            diameter: diameter.max(1e-4),
            discharge_coefficient: discharge_coefficient.clamp(0.0, 1.0),
        }
    }

    /// Fraction of the event elapsed at a crank angle; `None` while shut.
    fn progress(&self, theta: f64) -> Option<f64> {
        if self.duration <= 0.0 {
            return None;
        }
        let phase = wrap_cycle(theta - self.open_angle);
        (phase < self.duration).then(|| phase / self.duration)
    }

    /// Valve lift [m].
    ///
    /// A raised cosine: lift *and* lift velocity both vanish at the seat, so the
    /// flow area the integrator sees is C1-continuous. A trapezoidal or purely
    /// linear ramp puts a kink in `dm/dt` that costs RK4 its fourth order right
    /// where the flow is fastest.
    pub fn lift(&self, theta: f64) -> f64 {
        match self.progress(theta) {
            Some(u) => 0.5 * self.max_lift * (1.0 - (2.0 * PI * u).cos()),
            None => 0.0,
        }
    }

    /// Effective flow area `C_d * A_ref` [m^2].
    ///
    /// The reference area is the curtain `pi * D * L`, capped at the seat area
    /// `pi/4 * D^2`: past `L = D/4` the curtain stops being the restriction.
    pub fn effective_area(&self, theta: f64) -> f64 {
        let lift = self.lift(theta);
        if lift <= 0.0 {
            return 0.0;
        }
        let curtain = PI * self.diameter * lift;
        let seat = PI * self.diameter * self.diameter / 4.0;
        self.discharge_coefficient * curtain.min(seat)
    }

    /// Closing angle in cycle coordinates [rad].
    pub fn close_angle(&self) -> f64 {
        wrap_cycle(self.open_angle + self.duration)
    }

    /// Whether the valve is off its seat at this angle.
    pub fn is_open(&self, theta: f64) -> bool {
        self.progress(theta).is_some()
    }
}

/// The intake and exhaust events of one cylinder.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ValveTrain {
    /// Intake valve event.
    pub intake: ValveEvent,
    /// Exhaust valve event.
    pub exhaust: ValveEvent,
}

impl Default for ValveTrain {
    /// A mild performance cam: 240 degrees duration a side, 40 degrees overlap.
    ///
    /// IVO 20 BTDC (700), IVC 40 ABDC (220); EVO 40 BBDC (500), EVC 20 ATDC (20).
    fn default() -> Self {
        Self {
            intake: ValveEvent::new(deg(700.0), deg(240.0), 0.010, 0.037, 0.65),
            exhaust: ValveEvent::new(deg(500.0), deg(240.0), 0.009, 0.031, 0.60),
        }
    }
}

impl ValveTrain {
    /// True if either valve is off its seat: the cylinder is an open system.
    pub fn in_gas_exchange(&self, theta: f64) -> bool {
        self.intake.is_open(theta) || self.exhaust.is_open(theta)
    }
}

/// Subsonic/choked isentropic orifice flow function.
///
/// ```text
/// Phi(P_r) = P_r^(1/g) sqrt( 2g/(g-1) * (1 - P_r^((g-1)/g)) )     P_r >  P_crit
///          = sqrt(g) * (2/(g+1))^((g+1)/(2(g-1)))                 P_r <= P_crit
/// ```
///
/// so that `m_dot = C_d A P_up / sqrt(R T_up) * Phi(P_down / P_up)`.
pub fn flow_function(pressure_ratio: f64, gamma: f64) -> f64 {
    let g = gamma.clamp(1.01, 1.99);
    let critical = (2.0 / (g + 1.0)).powf(g / (g - 1.0));
    let pr = pressure_ratio.clamp(0.0, 1.0);
    if pr <= critical {
        g.sqrt() * (2.0 / (g + 1.0)).powf((g + 1.0) / (2.0 * (g - 1.0)))
    } else {
        let term = (1.0 - pr.powf((g - 1.0) / g)).max(0.0);
        pr.powf(1.0 / g) * (2.0 * g / (g - 1.0) * term).sqrt()
    }
}

/// Static conditions in the port on the far side of a valve.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PortState {
    /// Static pressure [Pa].
    pub pressure: f64,
    /// Static temperature [K].
    pub temperature: f64,
    /// Specific gas constant of the port gas [J/(kg K)].
    pub gas_constant: f64,
    /// Ratio of specific heats of the port gas [-].
    pub gamma: f64,
    /// Burned mass fraction the port would push back in [-].
    pub burned_fraction: f64,
}

impl PortState {
    /// Specific heat at constant pressure of the port gas [J/(kg K)].
    pub fn cp(&self) -> f64 {
        self.gamma * self.gas_constant / (self.gamma - 1.0)
    }
}

/// Both port boundary conditions seen by a cylinder.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PortConditions {
    /// Intake runner, just upstream of the intake valve.
    pub intake: PortState,
    /// Exhaust runner, just downstream of the exhaust valve.
    pub exhaust: PortState,
}

impl PortConditions {
    /// Naturally aspirated ports at ambient, with a hot exhaust runner.
    pub fn from_environment(env: &Environment, gas: &GasProperties) -> Self {
        Self {
            intake: PortState {
                pressure: env.pressure,
                temperature: env.temperature,
                gas_constant: gas.r_unburned,
                gamma: gas.gamma_unburned,
                burned_fraction: 0.0,
            },
            exhaust: PortState {
                // A stock exhaust sits slightly above ambient from its own
                // back pressure even before any tuning wave arrives.
                pressure: env.pressure * 1.03,
                temperature: 900.0,
                gas_constant: gas.r_burned,
                gamma: gas.gamma_burned,
                burned_fraction: 1.0,
            },
        }
    }
}

/// One stream crossing the control surface, signed positive *into* the cylinder.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PortFlux {
    /// Mass flow rate [kg/s].
    pub mass_flow: f64,
    /// Specific enthalpy carried by the stream [J/kg].
    pub enthalpy: f64,
    /// Burned mass fraction carried by the stream [-].
    pub burned_fraction: f64,
}

/// The cylinder side of a port solve, gathered so it is evaluated once per
/// derivative rather than once per valve.
#[derive(Debug, Clone, Copy, PartialEq)]
struct CylinderGas {
    pressure: f64,
    temperature: f64,
    gas_constant: f64,
    gamma: f64,
    cp: f64,
    burned_fraction: f64,
}

impl CylinderGas {
    /// Reads the blended gas state off a cylinder at its current burned fraction.
    fn of(state: &CylinderState, geometry: &CylinderGeometry, gas: &GasProperties) -> Self {
        let x = state.burned_fraction;
        Self {
            pressure: state.pressure(geometry, gas),
            temperature: state.temperature.max(1.0),
            gas_constant: gas.r_specific(x),
            gamma: gas.gamma(x),
            cp: gas.cp(x),
            burned_fraction: x,
        }
    }
}

/// Solves one valve as a compressible orifice.
///
/// Direction falls out of the pressure difference, and the upstream state is
/// whichever side is at the higher pressure, so backflow and reverse blowdown
/// come out of the same expression as forward flow.
fn port_flux(effective_area: f64, port: &PortState, cylinder: &CylinderGas) -> PortFlux {
    if effective_area <= 0.0 {
        return PortFlux::default();
    }
    let p_cyl = cylinder.pressure.max(1.0);
    let p_port = port.pressure.max(1.0);

    if p_port >= p_cyl {
        let mass_flow = effective_area * p_port
            / (port.gas_constant * port.temperature.max(1.0)).sqrt()
            * flow_function(p_cyl / p_port, port.gamma);
        PortFlux {
            mass_flow,
            enthalpy: port.cp() * port.temperature,
            burned_fraction: port.burned_fraction,
        }
    } else {
        let mass_flow = effective_area * p_cyl
            / (cylinder.gas_constant * cylinder.temperature).sqrt()
            * flow_function(p_port / p_cyl, cylinder.gamma);
        PortFlux {
            mass_flow: -mass_flow,
            enthalpy: cylinder.cp * cylinder.temperature,
            burned_fraction: cylinder.burned_fraction,
        }
    }
}

// ---------------------------------------------------------------------------
// State vector, per-cycle latch, derivatives
// ---------------------------------------------------------------------------

/// Quantities latched once per cycle at intake valve close.
///
/// Woschni needs a motored-pressure reference and the knock model needs an
/// end-gas isentrope; both are anchored to the trapped condition at IVC. They
/// are latched rather than integrated because they define the *reference* the
/// integrated state is measured against.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CycleLatch {
    /// Trapped fuel mass for this cycle [kg].
    pub fuel_mass: f64,
    /// Cylinder pressure at IVC [Pa].
    pub pressure: f64,
    /// Cylinder temperature at IVC [K].
    pub temperature: f64,
    /// Cylinder volume at IVC [m^3].
    pub volume: f64,
    /// Ratio of specific heats of the trapped charge at IVC [-].
    pub gamma: f64,
}

impl CycleLatch {
    /// A latch for an unfuelled cylinder full of ambient air.
    pub fn ambient(geometry: &CylinderGeometry, gas: &GasProperties, env: &Environment) -> Self {
        Self {
            fuel_mass: 0.0,
            pressure: env.pressure,
            temperature: env.temperature,
            volume: geometry.max_volume(),
            gamma: gas.gamma_unburned,
        }
    }

    /// Motored (no-combustion) pressure at the current volume [Pa].
    ///
    /// `P_mot = P_ivc (V_ivc / V)^gamma` — the isentrope the charge would have
    /// followed had the spark never fired.
    pub fn motored_pressure(&self, volume: f64) -> f64 {
        self.pressure * (self.volume / volume.max(1e-12)).powf(self.gamma)
    }

    /// Unburned end-gas temperature at the current pressure [K].
    ///
    /// The end gas is compressed isentropically by the flame ahead of it, so it
    /// rides `T_u = T_ivc (P / P_ivc)^((gamma-1)/gamma)` while the bulk average
    /// runs away with the burned products.
    pub fn end_gas_temperature(&self, pressure: f64) -> f64 {
        let exponent = (self.gamma - 1.0) / self.gamma;
        self.temperature * (pressure.max(1.0) / self.pressure.max(1.0)).powf(exponent)
    }
}

/// Full solver state: the cylinder vector plus the knock integral and the latch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThermoState {
    /// `[theta, T, m, x_b]`.
    pub cylinder: CylinderState,
    /// Livengood-Wu autoignition integral; knock at `>= 1` [-].
    pub knock_integral: f64,
    /// Per-cycle reference conditions.
    pub latch: CycleLatch,
}

impl ThermoState {
    /// Seeds a cylinder at intake TDC with ambient charge and a cleared integral.
    pub fn at_ambient(geometry: &CylinderGeometry, gas: &GasProperties, env: &Environment) -> Self {
        Self {
            cylinder: CylinderState::at_ambient(geometry, gas, env.pressure, env.temperature),
            knock_integral: 0.0,
            latch: CycleLatch::ambient(geometry, gas, env),
        }
    }

    /// Whether the charge has autoignited ahead of the flame.
    pub fn is_knocking(&self) -> bool {
        self.knock_integral >= 1.0
    }

    /// True if any component of the state vector has gone non-finite.
    pub fn is_corrupt(&self) -> bool {
        !(self.cylinder.theta.is_finite()
            && self.cylinder.temperature.is_finite()
            && self.cylinder.mass.is_finite()
            && self.cylinder.burned_fraction.is_finite()
            && self.knock_integral.is_finite())
    }
}

/// Time derivatives of the integrated vector.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Derivatives {
    /// `dT/dt` [K/s].
    pub dtemperature: f64,
    /// `dm/dt` [kg/s].
    pub dmass: f64,
    /// `dx_b/dt` [1/s].
    pub dburned: f64,
    /// `dI/dt` [1/s].
    pub dknock: f64,
}

/// Instantaneous energy and mass audit at one crank angle, for instrumentation.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ThermoFlows {
    /// Combustion heat release rate [W].
    pub heat_release: f64,
    /// Wall heat loss rate, positive out of the gas [W].
    pub wall_loss: f64,
    /// Piston work rate `P dV/dt`, positive done by the gas [W].
    pub piston_power: f64,
    /// Intake port flux, positive into the cylinder [kg/s].
    pub intake_flow: f64,
    /// Exhaust port flux, positive into the cylinder [kg/s].
    pub exhaust_flow: f64,
}

// ---------------------------------------------------------------------------
// The cylinder model
// ---------------------------------------------------------------------------

/// Everything needed to evaluate the derivatives of one cylinder.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CylinderModel {
    /// Slider-crank geometry.
    pub geometry: CylinderGeometry,
    /// Working-gas properties.
    pub gas: GasProperties,
    /// Wiebe heat release.
    pub wiebe: WiebeProfile,
    /// Woschni wall heat transfer.
    pub heat: WoschniModel,
    /// Valve events.
    pub valves: ValveTrain,
    /// Autoignition model.
    pub knock: KnockModel,
    /// Fuel lower heating value [J/kg].
    pub fuel_lhv: f64,
    /// Air/fuel mass ratio the charge is metered to [-].
    pub air_fuel_ratio: f64,
}

impl Default for CylinderModel {
    fn default() -> Self {
        Self {
            geometry: CylinderGeometry::default(),
            gas: GasProperties::default(),
            wiebe: WiebeProfile::default(),
            heat: WoschniModel::default(),
            valves: ValveTrain::default(),
            knock: KnockModel::default(),
            fuel_lhv: GASOLINE_LHV,
            air_fuel_ratio: STOICH_AFR,
        }
    }
}

impl CylinderModel {
    /// Evaluates every flux at the current state, without integrating.
    pub fn flows(&self, st: &ThermoState, omega: f64, ports: &PortConditions) -> ThermoFlows {
        let cyl = &st.cylinder;
        let theta = cyl.theta;
        let volume = self.geometry.safe_volume(theta);
        let gas = CylinderGas::of(cyl, &self.geometry, &self.gas);
        let pressure = gas.pressure;

        let intake = port_flux(
            self.valves.intake.effective_area(theta),
            &ports.intake,
            &gas,
        );
        let exhaust = port_flux(
            self.valves.exhaust.effective_area(theta),
            &ports.exhaust,
            &gas,
        );

        let dxb_dt = if st.latch.fuel_mass > 0.0 {
            self.wiebe.dburned_dtheta(theta) * omega
        } else {
            0.0
        };
        let heat_release =
            st.latch.fuel_mass * self.fuel_lhv * self.wiebe.combustion_efficiency * dxb_dt;

        let motored = st.latch.motored_pressure(volume);
        let gas_velocity = self.woschni_velocity(theta, pressure, motored, omega, st);
        let h_c = self
            .heat
            .coefficient(&self.geometry, pressure, cyl.temperature, gas_velocity);
        let wall_loss = h_c
            * self.heat.surface_area(&self.geometry, theta)
            * (cyl.temperature - self.heat.wall_temperature);

        ThermoFlows {
            heat_release,
            wall_loss,
            piston_power: pressure * self.geometry.dvolume_dtheta(theta) * omega,
            intake_flow: intake.mass_flow,
            exhaust_flow: exhaust.mass_flow,
        }
    }

    /// Woschni characteristic gas velocity `w` [m/s].
    fn woschni_velocity(
        &self,
        theta: f64,
        pressure: f64,
        motored_pressure: f64,
        omega: f64,
        st: &ThermoState,
    ) -> f64 {
        let rpm = omega * 60.0 / (2.0 * PI);
        let mean_piston_speed = self.geometry.mean_piston_speed(rpm.abs());

        if self.valves.in_gas_exchange(theta) {
            // Open period: scavenging swirl dominates, no combustion term.
            return self.heat.c1_gas_exchange * mean_piston_speed;
        }

        let mut w = self.heat.c1_closed * mean_piston_speed;
        // The combustion term only exists once there are products to expand.
        if st.cylinder.burned_fraction > 1e-6 || self.wiebe.is_burning(theta) {
            let reference = st.latch.pressure * st.latch.volume;
            if reference > 1e-12 {
                w += self.heat.c2_combustion * self.geometry.displacement() * st.latch.temperature
                    / reference
                    * (pressure - motored_pressure).max(0.0);
            }
        }
        w
    }

    /// The right-hand side of the ODE system at a given state.
    pub fn derivatives(&self, st: &ThermoState, omega: f64, ports: &PortConditions) -> Derivatives {
        let cyl = &st.cylinder;
        let theta = cyl.theta;
        let mass = cyl.mass.max(MASS_FLOOR);
        let temperature = cyl.temperature.max(1.0);

        let volume = self.geometry.safe_volume(theta);
        let cv = self.gas.cv(cyl.burned_fraction);
        // Built from the floored mass and temperature rather than read straight
        // off the state, so an RK4 stage that has driven the cylinder to
        // near-vacuum still yields a finite pressure to flow against.
        let gas = CylinderGas {
            pressure: mass * self.gas.r_specific(cyl.burned_fraction) * temperature / volume,
            temperature,
            gas_constant: self.gas.r_specific(cyl.burned_fraction),
            gamma: self.gas.gamma(cyl.burned_fraction),
            cp: self.gas.cp(cyl.burned_fraction),
            burned_fraction: cyl.burned_fraction,
        };
        let pressure = gas.pressure;

        // --- mass fluxes ---------------------------------------------------
        let intake = port_flux(
            self.valves.intake.effective_area(theta),
            &ports.intake,
            &gas,
        );
        let exhaust = port_flux(
            self.valves.exhaust.effective_area(theta),
            &ports.exhaust,
            &gas,
        );
        let dmass = intake.mass_flow + exhaust.mass_flow;

        // --- heat release ---------------------------------------------------
        // No fuel, no flame: a motored or fuel-cut cylinder must keep unburned
        // gas properties, not just skip the heat release.
        let dxb_chem = if st.latch.fuel_mass > 0.0 {
            self.wiebe.dburned_dtheta(theta) * omega
        } else {
            0.0
        };
        let heat_release =
            st.latch.fuel_mass * self.fuel_lhv * self.wiebe.combustion_efficiency * dxb_chem;

        // --- wall loss -------------------------------------------------------
        let motored = st.latch.motored_pressure(volume);
        let w = self.woschni_velocity(theta, pressure, motored, omega, st);
        let h_c = self
            .heat
            .coefficient(&self.geometry, pressure, temperature, w);
        let wall_loss = h_c
            * self.heat.surface_area(&self.geometry, theta)
            * (temperature - self.heat.wall_temperature);

        // --- piston work -----------------------------------------------------
        let piston_power = pressure * self.geometry.dvolume_dtheta(theta) * omega;

        // --- first law -------------------------------------------------------
        // sum_j m_dot_j (h_j - u), with u = c_v T.
        let internal_energy = cv * temperature;
        let flux_energy = intake.mass_flow * (intake.enthalpy - internal_energy)
            + exhaust.mass_flow * (exhaust.enthalpy - internal_energy);

        let dtemperature = (heat_release - wall_loss - piston_power + flux_energy) / (mass * cv);

        // --- burned fraction transport ---------------------------------------
        // d(m x)/dt = m x_chem + sum_j m_dot_j x_j  =>  dx/dt = x_chem + sum_j m_dot_j (x_j - x)/m.
        // Outflow carries x_j = x and drops out; fresh charge dilutes, exhaust
        // backflow enriches.
        let dburned = dxb_chem
            + (intake.mass_flow * (intake.burned_fraction - cyl.burned_fraction)
                + exhaust.mass_flow * (exhaust.burned_fraction - cyl.burned_fraction))
                / mass;

        // --- autoignition -----------------------------------------------------
        // Only the unburned pocket can autoignite, and only while the cylinder
        // is sealed: once the exhaust cracks open the end gas is gone.
        let dknock = if cyl.burned_fraction < 0.999
            && st.latch.fuel_mass > 0.0
            && !self.valves.in_gas_exchange(theta)
        {
            self.knock
                .dknock_dt(pressure, st.latch.end_gas_temperature(pressure))
        } else {
            0.0
        };

        Derivatives {
            dtemperature,
            dmass,
            dburned,
            dknock,
        }
    }

    /// Trapped fuel mass implied by a charge mass and its residual fraction [kg].
    ///
    /// Only the *fresh* part of the trapped mass carries fuel, and it arrives as
    /// a metered mixture, so `m_fuel = m (1 - x_b) / (1 + AFR)`.
    pub fn trapped_fuel_mass(&self, mass: f64, burned_fraction: f64) -> f64 {
        let fresh = mass.max(0.0) * (1.0 - burned_fraction.clamp(0.0, 1.0));
        fresh / (1.0 + self.air_fuel_ratio.max(1e-3))
    }
}

// ---------------------------------------------------------------------------
// Guardrails: watchdog and adaptive angular budget
// ---------------------------------------------------------------------------

/// NaN/Inf watchdog.
///
/// A single non-finite value in a state vector is contagious: it propagates
/// through the pressure closure into every downstream consumer and, in the audio
/// path, straight into the speaker as a full-scale click. The watchdog checks
/// after every macro step and resets a corrupted cylinder to ambient rather than
/// letting it poison the rest of the block.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Watchdog {
    /// How many times a corrupt state has been reset since construction.
    pub resets: u64,
    /// Whether the most recent step tripped the watchdog.
    pub tripped: bool,
}

impl Watchdog {
    /// Inspects a state, resetting it to ambient if it has gone non-finite.
    ///
    /// Returns `true` when a reset happened.
    pub fn guard(
        &mut self,
        st: &mut ThermoState,
        geometry: &CylinderGeometry,
        gas: &GasProperties,
        env: &Environment,
    ) -> bool {
        self.tripped = false;
        if !st.is_corrupt() {
            // Still enforce the physical envelope: clamping a merely extreme
            // state is much cheaper than letting it reach the NaN that follows.
            st.cylinder.sanitize();
            st.knock_integral = st.knock_integral.clamp(0.0, 1e6);
            return false;
        }

        // Keep the crank phase if it is the one thing still intact: the firing
        // order stays coherent and the reset is inaudible rather than a jump.
        let theta = if st.cylinder.theta.is_finite() {
            wrap_cycle(st.cylinder.theta)
        } else {
            0.0
        };
        let mut fresh = ThermoState::at_ambient(geometry, gas, env);
        fresh.cylinder.theta = theta;
        *st = fresh;

        self.resets = self.resets.saturating_add(1);
        self.tripped = true;
        true
    }
}

/// How a wall-clock frame is turned into crank-angle substeps.
///
/// Two separate jobs:
///
/// 1. Clamp the frame delta so a stalled frame (an alt-tab, a GC pause) cannot
///    ask the solver to advance a second of engine time in one call.
/// 2. Cap the substep count. When the clamped frame still needs more substeps
///    than the budget allows, `delta_theta` is stretched — up to `max_dtheta` —
///    so CPU load stays flat during a lag spike instead of spiralling.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdaptiveBudget {
    /// Longest frame the solver will honour [s]. Anything past this is dropped.
    pub max_frame_dt: f64,
    /// Preferred angular substep [rad].
    pub base_dtheta: f64,
    /// Coarsest angular substep the accuracy floor allows [rad].
    pub max_dtheta: f64,
    /// Substep count above which `delta_theta` starts stretching.
    pub target_substeps: usize,
    /// Hard ceiling on substeps per frame, whatever the budget says.
    pub max_substeps: usize,
}

impl Default for AdaptiveBudget {
    /// 1 degree nominal, 4 degrees under load, frames clamped to 1/30 s.
    fn default() -> Self {
        Self {
            max_frame_dt: 1.0 / 30.0,
            base_dtheta: deg(1.0),
            max_dtheta: deg(4.0),
            target_substeps: 768,
            max_substeps: 4096,
        }
    }
}

/// The substep schedule chosen for one frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StepPlan {
    /// Frame delta after clamping [s].
    pub dt: f64,
    /// Angular substep actually used [rad].
    pub dtheta: f64,
    /// Number of RK4 substeps.
    pub substeps: usize,
    /// True if the frame delta was clamped (a lag spike was absorbed).
    pub clamped: bool,
    /// True if `dtheta` had to be stretched past `base_dtheta`.
    pub stretched: bool,
}

impl AdaptiveBudget {
    /// Plans the substeps for a frame at a given shaft speed.
    pub fn plan(&self, frame_dt: f64, omega: f64) -> StepPlan {
        // A non-finite or negative dt is treated as a dropped frame.
        let raw = if frame_dt.is_finite() {
            frame_dt.max(0.0)
        } else {
            0.0
        };
        let clamped = raw > self.max_frame_dt;
        let dt = raw.min(self.max_frame_dt);

        let travel = omega.abs() * dt;
        if travel <= 0.0 || dt <= 0.0 {
            return StepPlan {
                dt,
                dtheta: self.base_dtheta,
                substeps: 0,
                clamped,
                stretched: false,
            };
        }

        let mut dtheta = self.base_dtheta;
        let mut substeps = (travel / dtheta).ceil() as usize;
        let mut stretched = false;

        if substeps > self.target_substeps {
            // Spread the same travel over the budget, but never coarser than the
            // accuracy floor: past ~4 degrees the Wiebe peak is under-resolved.
            dtheta = (travel / self.target_substeps as f64).min(self.max_dtheta);
            substeps = (travel / dtheta).ceil() as usize;
            stretched = true;
        }

        substeps = substeps.clamp(1, self.max_substeps);
        // Land exactly on the requested travel rather than overshooting on the
        // final substep.
        dtheta = travel / substeps as f64;

        StepPlan {
            dt,
            dtheta,
            substeps,
            clamped,
            stretched,
        }
    }
}

// ---------------------------------------------------------------------------
// The RK4 solver
// ---------------------------------------------------------------------------

/// What one frame of integration did.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StepReport {
    /// The schedule that was used.
    pub plan: StepPlan,
    /// Crank angle actually covered [rad].
    pub advanced: f64,
    /// True if the watchdog fired during the frame.
    pub watchdog_tripped: bool,
    /// True if the charge autoignited during the frame.
    pub knocked: bool,
    /// Peak cylinder pressure seen during the frame [Pa].
    pub peak_pressure: f64,
}

/// Explicit fourth-order Runge-Kutta integrator over crank-angle substeps.
///
/// Classic RK4 on `Y = [T, m, x_b, I]`:
///
/// ```text
/// k1 = f(t,        Y)
/// k2 = f(t + h/2,  Y + h/2 k1)
/// k3 = f(t + h/2,  Y + h/2 k2)
/// k4 = f(t + h,    Y + h k3)
/// Y' = Y + h/6 (k1 + 2 k2 + 2 k3 + k4)
/// ```
///
/// with `h = delta_theta / omega` and the stage angles advanced analytically.
/// Explicit RK4 is chosen over an implicit scheme deliberately: the stiffest
/// term here is the Wiebe peak, which a 1-degree step resolves comfortably, and
/// a fixed four-evaluation cost per substep is what makes the per-frame budget
/// in [`AdaptiveBudget`] predictable enough to be worth having.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Rk4Solver {
    /// Angular budget policy.
    pub budget: AdaptiveBudget,
    /// NaN/Inf watchdog.
    pub watchdog: Watchdog,
}

impl Rk4Solver {
    /// Builds a solver with a specific angular budget.
    pub fn new(budget: AdaptiveBudget) -> Self {
        Self {
            budget,
            watchdog: Watchdog::default(),
        }
    }

    /// Advances one RK4 substep of `dtheta` at shaft speed `omega`.
    ///
    /// The crank angle is carried analytically, so this only integrates the four
    /// genuinely dynamic components.
    pub fn substep(
        &self,
        model: &CylinderModel,
        st: &ThermoState,
        omega: f64,
        dtheta: f64,
        ports: &PortConditions,
    ) -> ThermoState {
        if omega.abs() < 1e-9 || dtheta == 0.0 {
            return *st;
        }
        let h = dtheta / omega;

        let stage =
            |base: &ThermoState, k: &Derivatives, frac: f64, angle_frac: f64| -> ThermoState {
                let mut next = *base;
                next.cylinder.theta = wrap_cycle(st.cylinder.theta + dtheta * angle_frac);
                next.cylinder.temperature =
                    (base.cylinder.temperature + frac * h * k.dtemperature).max(1.0);
                next.cylinder.mass = (base.cylinder.mass + frac * h * k.dmass).max(MASS_FLOOR);
                next.cylinder.burned_fraction =
                    (base.cylinder.burned_fraction + frac * h * k.dburned).clamp(0.0, 1.0);
                next.knock_integral = (base.knock_integral + frac * h * k.dknock).max(0.0);
                next
            };

        let k1 = model.derivatives(st, omega, ports);
        let s2 = stage(st, &k1, 0.5, 0.5);
        let k2 = model.derivatives(&s2, omega, ports);
        let s3 = stage(st, &k2, 0.5, 0.5);
        let k3 = model.derivatives(&s3, omega, ports);
        let s4 = stage(st, &k3, 1.0, 1.0);
        let k4 = model.derivatives(&s4, omega, ports);

        let weight = |a: f64, b: f64, c: f64, d: f64| h / 6.0 * (a + 2.0 * b + 2.0 * c + d);

        let mut out = *st;
        out.cylinder.theta = wrap_cycle(st.cylinder.theta + dtheta);
        out.cylinder.temperature += weight(
            k1.dtemperature,
            k2.dtemperature,
            k3.dtemperature,
            k4.dtemperature,
        );
        out.cylinder.mass += weight(k1.dmass, k2.dmass, k3.dmass, k4.dmass);
        out.cylinder.burned_fraction += weight(k1.dburned, k2.dburned, k3.dburned, k4.dburned);
        out.knock_integral += weight(k1.dknock, k2.dknock, k3.dknock, k4.dknock);

        out.cylinder.sanitize();
        out.knock_integral = out.knock_integral.max(0.0);
        out
    }

    /// Advances one wall-clock frame, latching the cycle references and running
    /// the watchdog.
    ///
    /// `observer` is called after every substep with the freshly advanced state,
    /// which is how the phase ring in
    /// [`engine_block`](super::engine_block) gets a crank-angle-resolved trace
    /// without the solver knowing anything about the ring.
    #[allow(clippy::too_many_arguments)]
    pub fn step_frame<F>(
        &mut self,
        model: &CylinderModel,
        st: &mut ThermoState,
        omega: f64,
        frame_dt: f64,
        ports: &PortConditions,
        env: &Environment,
        mut observer: F,
    ) -> StepReport
    where
        F: FnMut(&ThermoState, f64),
    {
        let plan = self.budget.plan(frame_dt, omega);
        let mut knocked = false;
        let mut peak_pressure = st.cylinder.pressure(&model.geometry, &model.gas);
        let mut tripped = false;

        for _ in 0..plan.substeps {
            let previous_theta = st.cylinder.theta;
            let next = self.substep(model, st, omega, plan.dtheta, ports);
            *st = next;

            // Latching happens on the substep that crosses IVC, so the reference
            // is the genuinely trapped charge and not a coarse-step average.
            if crossed(
                previous_theta,
                plan.dtheta,
                model.valves.intake.close_angle(),
            ) {
                self.latch_cycle(model, st);
            }
            // The exhaust valve cracking open ends the cycle's knock window.
            if crossed(previous_theta, plan.dtheta, model.valves.exhaust.open_angle) {
                st.knock_integral = 0.0;
            }

            if self.watchdog.guard(st, &model.geometry, &model.gas, env) {
                tripped = true;
            }

            let pressure = st.cylinder.pressure(&model.geometry, &model.gas);
            if pressure > peak_pressure {
                peak_pressure = pressure;
            }
            knocked |= st.is_knocking();

            observer(st, previous_theta);
        }

        StepReport {
            plan,
            advanced: plan.dtheta * plan.substeps as f64,
            watchdog_tripped: tripped,
            knocked,
            peak_pressure,
        }
    }

    /// Latches the per-cycle references from the state trapped at IVC.
    fn latch_cycle(&self, model: &CylinderModel, st: &mut ThermoState) {
        let cyl = &st.cylinder;
        st.latch = CycleLatch {
            fuel_mass: model.trapped_fuel_mass(cyl.mass, cyl.burned_fraction),
            pressure: cyl.pressure(&model.geometry, &model.gas),
            temperature: cyl.temperature,
            volume: model.geometry.safe_volume(cyl.theta),
            gamma: model.gas.gamma(cyl.burned_fraction),
        };
        st.knock_integral = 0.0;
    }
}

/// Whether a step of `dtheta` starting at `from` passed through `target`.
///
/// Wrap-aware: the whole cycle is a circle, so "did we cross 20 degrees" has to
/// hold for a step running from 710 to 730 (= 10) degrees just as well.
pub fn crossed(from: f64, dtheta: f64, target: f64) -> bool {
    if dtheta <= 0.0 || dtheta >= CYCLE_ANGLE {
        return dtheta >= CYCLE_ANGLE;
    }
    // Half-open in `[from, from + dtheta)`, so consecutive steps tile the cycle
    // and a target trips exactly once per revolution.
    //
    // The tolerance is what makes that hold in floating point. A target sitting
    // a hair *behind* `from` wraps round to nearly a full cycle rather than to
    // the zero it really is, so it has to be folded back; and a target a hair
    // *under* `from + dtheta` must not fire on both this step and the next. One
    // band handles both: `eps` is scaled to the step so it stays meaningful
    // whether the solver is running 0.25-degree or 4-degree substeps.
    let phase = wrap_cycle(target - from);
    let eps = dtheta * 1e-9;
    phase < dtheta - eps || phase > CYCLE_ANGLE - eps
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() <= tol, "expected {b}, got {a} (tol {tol})");
    }

    fn rpm_to_omega(rpm: f64) -> f64 {
        rpm * 2.0 * PI / 60.0
    }

    /// A sealed cylinder: no valves, no fuel, no wall loss. Pure compression.
    fn adiabatic_model() -> CylinderModel {
        let mut m = CylinderModel::default();
        m.heat.scaling = 0.0;
        m.valves.intake.duration = 0.0;
        m.valves.exhaust.duration = 0.0;
        m.wiebe.combustion_efficiency = 0.0;
        m
    }

    #[test]
    fn wiebe_runs_from_zero_to_almost_one_over_its_duration() {
        let w = WiebeProfile::default();
        approx(w.burned_fraction(w.spark_angle), 0.0, 1e-12);
        // a = 5 leaves exp(-5) ~ 0.67 % unburned at the end of the duration.
        let end = w.burned_fraction(w.spark_angle + w.duration * 0.999_999);
        approx(end, 1.0 - (-5.0f64).exp(), 1e-4);
        assert!(w.dburned_dtheta(w.spark_angle - deg(1.0)) == 0.0);
        assert!(w.dburned_dtheta(w.spark_angle + w.duration + deg(1.0)) == 0.0);
    }

    #[test]
    fn wiebe_rate_integrates_back_to_the_wiebe_profile() {
        let w = WiebeProfile::default();
        let steps = 20_000;
        let h = w.duration / steps as f64;
        let mut x = 0.0;
        for i in 0..steps {
            // Midpoint rule: second order, plenty for a 1e-6 check.
            x += w.dburned_dtheta(w.spark_angle + (i as f64 + 0.5) * h) * h;
        }
        approx(
            x,
            w.burned_fraction(w.spark_angle + w.duration * 0.999_999),
            1e-6,
        );
    }

    #[test]
    fn wiebe_rate_peaks_inside_the_burn_window() {
        let w = WiebeProfile::default();
        let mut peak_u = 0.0;
        let mut peak = 0.0;
        for i in 0..=1000 {
            let u = i as f64 / 1000.0;
            let rate = w.dburned_dtheta(w.spark_angle + u * w.duration);
            if rate > peak {
                peak = rate;
                peak_u = u;
            }
        }
        // n = 2 puts the peak burn rate a bit past mid-burn.
        assert!(
            (0.4..0.8).contains(&peak_u),
            "peak burn rate at u = {peak_u}, expected mid-burn"
        );
    }

    #[test]
    fn flow_function_chokes_at_the_critical_pressure_ratio() {
        let g = 1.4;
        let critical = (2.0f64 / (g + 1.0)).powf(g / (g - 1.0));
        approx(critical, 0.5283, 1e-4);

        // Continuous across the choke point.
        let just_above = flow_function(critical + 1e-9, g);
        let just_below = flow_function(critical - 1e-9, g);
        approx(just_above, just_below, 1e-6);

        // Flat once choked, and zero with no pressure difference.
        approx(flow_function(0.1, g), flow_function(0.4, g), 1e-12);
        approx(flow_function(1.0, g), 0.0, 1e-12);
        // Monotone rising as the ratio falls to the choke point.
        assert!(flow_function(0.9, g) < flow_function(0.7, g));
        assert!(flow_function(0.7, g) < flow_function(critical, g));
    }

    #[test]
    fn valve_lift_is_smooth_and_closed_outside_its_event() {
        let v = ValveEvent::new(deg(700.0), deg(240.0), 0.010, 0.037, 0.65);
        approx(v.lift(deg(700.0)), 0.0, 1e-15);
        approx(v.lift(v.close_angle() - 1e-9), 0.0, 1e-9);
        approx(v.lift(deg(700.0 + 120.0)), 0.010, 1e-12);
        assert_eq!(v.lift(deg(400.0)), 0.0, "shut mid-power-stroke");
        assert_eq!(v.effective_area(deg(400.0)), 0.0);
        assert!(v.is_open(deg(10.0)), "event wraps through 720 degrees");
        approx(v.close_angle().to_degrees(), 220.0, 1e-9);
    }

    /// The load-bearing accuracy claim: on a closed, adiabatic, non-reacting
    /// cylinder the solver must reproduce `T V^(gamma-1) = const` to well under
    /// a Kelvin over a full compression stroke.
    #[test]
    fn rk4_reproduces_the_isentrope_on_a_sealed_cylinder() {
        let model = adiabatic_model();
        let env = Environment::default();
        let ports = PortConditions::from_environment(&env, &model.gas);
        let solver = Rk4Solver::default();
        let omega = rpm_to_omega(3000.0);

        let mut st = ThermoState::at_ambient(&model.geometry, &model.gas, &env);
        st.cylinder.theta = PI; // BDC, start of compression
        st.cylinder.mass =
            env.pressure * model.geometry.max_volume() / (model.gas.r_unburned * env.temperature);
        st.cylinder.burned_fraction = 0.0;

        let t0 = st.cylinder.temperature;
        let v0 = model.geometry.safe_volume(st.cylinder.theta);
        let gamma = model.gas.gamma(0.0);

        // 180 degrees of compression at 1 degree a step.
        for _ in 0..180 {
            st = solver.substep(&model, &st, omega, deg(1.0), &ports);
        }

        let v1 = model.geometry.safe_volume(st.cylinder.theta);
        let expected = t0 * (v0 / v1).powf(gamma - 1.0);
        approx(st.cylinder.temperature, expected, 0.05);
        // Mass is exactly conserved with both valves shut.
        approx(
            st.cylinder.mass,
            env.pressure * v0 / (model.gas.r_unburned * env.temperature),
            1e-15,
        );
    }

    /// RK4 is fourth order, so quartering the step should cut the error by ~256.
    /// Anything better than 100x confirms we have not silently dropped to a
    /// lower-order scheme in the stage construction.
    #[test]
    fn rk4_converges_at_fourth_order() {
        let model = adiabatic_model();
        let env = Environment::default();
        let ports = PortConditions::from_environment(&env, &model.gas);
        let solver = Rk4Solver::default();
        let omega = rpm_to_omega(3000.0);
        let gamma = model.gas.gamma(0.0);

        let error_at = |dtheta_deg: f64| -> f64 {
            let mut st = ThermoState::at_ambient(&model.geometry, &model.gas, &env);
            st.cylinder.theta = PI;
            let t0 = st.cylinder.temperature;
            let v0 = model.geometry.safe_volume(st.cylinder.theta);
            let steps = (180.0 / dtheta_deg).round() as usize;
            for _ in 0..steps {
                st = solver.substep(&model, &st, omega, deg(dtheta_deg), &ports);
            }
            let v1 = model.geometry.safe_volume(st.cylinder.theta);
            (st.cylinder.temperature - t0 * (v0 / v1).powf(gamma - 1.0)).abs()
        };

        let coarse = error_at(4.0);
        let fine = error_at(1.0);
        assert!(
            fine * 100.0 < coarse,
            "expected ~4th order convergence, got {coarse} -> {fine}"
        );
    }

    #[test]
    fn combustion_raises_pressure_and_temperature() {
        let model = CylinderModel::default();
        let env = Environment::default();
        let ports = PortConditions::from_environment(&env, &model.gas);
        let solver = Rk4Solver::default();
        let omega = rpm_to_omega(3000.0);

        // Trap a fresh charge at BDC and run the real compression-and-burn path
        // rather than dropping a burn onto a cold cylinder mid-stroke.
        let mut st = ThermoState::at_ambient(&model.geometry, &model.gas, &env);
        st.cylinder.theta = PI;
        st.cylinder.mass =
            env.pressure * model.geometry.max_volume() / (model.gas.r_unburned * env.temperature);
        st.latch = CycleLatch {
            fuel_mass: model.trapped_fuel_mass(st.cylinder.mass, 0.0),
            pressure: env.pressure,
            temperature: env.temperature,
            volume: model.geometry.max_volume(),
            gamma: model.gas.gamma_unburned,
        };

        let before = st.cylinder.pressure(&model.geometry, &model.gas);
        // 180 degrees BDC to TDC, then 80 more to finish the burn.
        for _ in 0..260 {
            st = solver.substep(&model, &st, omega, deg(1.0), &ports);
        }
        let after = st.cylinder.pressure(&model.geometry, &model.gas);

        assert!(
            after > before * 10.0,
            "burn produced no pressure rise: {before} -> {after}"
        );
        assert!(
            st.cylinder.burned_fraction > 0.99,
            "burn did not complete: x_b = {}",
            st.cylinder.burned_fraction
        );
        assert!(
            (1500.0..4000.0).contains(&st.cylinder.temperature),
            "unphysical peak temperature {}",
            st.cylinder.temperature
        );
    }

    #[test]
    fn wall_loss_cools_the_charge_relative_to_adiabatic() {
        let env = Environment::default();
        let omega = rpm_to_omega(3000.0);
        let solver = Rk4Solver::default();

        let run = |scaling: f64| {
            let mut model = adiabatic_model();
            model.heat.scaling = scaling;
            let ports = PortConditions::from_environment(&env, &model.gas);
            let mut st = ThermoState::at_ambient(&model.geometry, &model.gas, &env);
            st.cylinder.theta = PI;
            st.cylinder.temperature = 900.0; // well above the 450 K wall
            for _ in 0..180 {
                st = solver.substep(&model, &st, omega, deg(1.0), &ports);
            }
            st.cylinder.temperature
        };

        assert!(run(1.0) < run(0.0), "Woschni loss must cool the charge");
    }

    #[test]
    fn knock_integral_accumulates_faster_on_low_octane_fuel() {
        let hot = KnockModel::default();
        let low = KnockModel {
            octane_number: 80.0,
            ..KnockModel::default()
        };

        let p = 40e5;
        let t = 850.0;
        assert!(
            low.ignition_delay(p, t) < hot.ignition_delay(p, t),
            "lower octane must autoignite sooner"
        );
        // Pressure shortens the delay, temperature shortens it much faster.
        assert!(hot.ignition_delay(80e5, t) < hot.ignition_delay(40e5, t));
        assert!(hot.ignition_delay(p, 1000.0) < hot.ignition_delay(p, 700.0));
        assert!(hot.ignition_delay(p, t).is_finite());
        // Cold end gas is effectively inert, not infinite.
        assert!(hot.ignition_delay(1e5, 300.0).is_finite());
    }

    #[test]
    fn advancing_the_spark_drives_the_knock_integral_up() {
        let env = Environment::default();
        let omega = rpm_to_omega(3000.0);

        let run = |spark_deg: f64| {
            let model = CylinderModel {
                wiebe: WiebeProfile::new(deg(spark_deg), deg(60.0), 5.0, 2.0, 0.97),
                // A 14:1 squeeze to put the end gas firmly into knock territory.
                geometry: CylinderGeometry::new(0.086, 0.086, 0.1345, 14.0),
                ..CylinderModel::default()
            };
            let ports = PortConditions::from_environment(&env, &model.gas);
            let solver = Rk4Solver::default();

            let mut st = ThermoState::at_ambient(&model.geometry, &model.gas, &env);
            st.cylinder.theta = PI;
            st.cylinder.mass = 2.2 * env.pressure * model.geometry.max_volume()
                / (model.gas.r_unburned * env.temperature);
            st.cylinder.temperature = 380.0;
            st.latch = CycleLatch {
                fuel_mass: model.trapped_fuel_mass(st.cylinder.mass, 0.0),
                pressure: st.cylinder.pressure(&model.geometry, &model.gas),
                temperature: st.cylinder.temperature,
                volume: model.geometry.max_volume(),
                gamma: model.gas.gamma_unburned,
            };
            for _ in 0..300 {
                st = solver.substep(&model, &st, omega, deg(1.0), &ports);
            }
            st.knock_integral
        };

        let retarded = run(355.0);
        let advanced = run(325.0);
        assert!(
            advanced > retarded,
            "advancing the spark must raise I_knock: {advanced} vs {retarded}"
        );
        assert!(advanced.is_finite() && retarded >= 0.0);
    }

    #[test]
    fn watchdog_resets_a_corrupted_state_and_keeps_the_phase() {
        let model = CylinderModel::default();
        let env = Environment::default();
        let mut dog = Watchdog::default();

        let mut st = ThermoState::at_ambient(&model.geometry, &model.gas, &env);
        st.cylinder.theta = deg(123.0);
        st.cylinder.temperature = f64::NAN;
        st.cylinder.mass = f64::INFINITY;

        assert!(dog.guard(&mut st, &model.geometry, &model.gas, &env));
        assert_eq!(dog.resets, 1);
        assert!(dog.tripped);
        assert!(!st.is_corrupt());
        approx(st.cylinder.theta.to_degrees(), 123.0, 1e-9);
        approx(st.cylinder.temperature, env.temperature, 1e-9);

        // A healthy state passes through untouched and does not count a reset.
        let before = st;
        assert!(!dog.guard(&mut st, &model.geometry, &model.gas, &env));
        assert_eq!(dog.resets, 1);
        assert_eq!(st, before);
    }

    #[test]
    fn watchdog_recovers_the_solver_from_an_injected_nan() {
        let model = CylinderModel::default();
        let env = Environment::default();
        let ports = PortConditions::from_environment(&env, &model.gas);
        let mut solver = Rk4Solver::default();
        let omega = rpm_to_omega(3000.0);

        let mut st = ThermoState::at_ambient(&model.geometry, &model.gas, &env);
        st.cylinder.temperature = f64::NAN;

        let report = solver.step_frame(&model, &mut st, omega, 1.0 / 60.0, &ports, &env, |_, _| {});
        assert!(report.watchdog_tripped);
        assert!(!st.is_corrupt());
        assert!(st
            .cylinder
            .pressure(&model.geometry, &model.gas)
            .is_finite());
        assert!(solver.watchdog.resets >= 1);
    }

    #[test]
    fn adaptive_budget_clamps_lag_spikes_and_stretches_dtheta() {
        let budget = AdaptiveBudget::default();
        let omega = rpm_to_omega(7000.0);

        // A 60 Hz frame at redline still fits the budget at the nominal step:
        // stretching is meant to be a lag response, not the normal case.
        let normal = budget.plan(1.0 / 60.0, omega);
        assert!(!normal.clamped && !normal.stretched);
        assert!(normal.dtheta <= budget.base_dtheta + 1e-12);
        approx(normal.dtheta * normal.substeps as f64, omega / 60.0, 1e-9);

        // A two-second stall is clamped to 1/30 s: the engine cannot skip ahead.
        let spike = budget.plan(2.0, omega);
        assert!(spike.clamped);
        approx(spike.dt, 1.0 / 30.0, 1e-15);
        assert!(spike.substeps <= budget.max_substeps);
        // The clamp is what caps the work, so the same travel is covered either
        // way: 1/30 s at 7000 rpm is 1400 degrees, which does need stretching.
        assert!(spike.stretched);
        assert!(spike.dtheta <= budget.max_dtheta + 1e-12);
        approx(spike.dtheta * spike.substeps as f64, omega / 30.0, 1e-9);

        // Once dtheta hits the accuracy floor the substep count is allowed to
        // exceed the target: correctness wins over the CPU budget, and the
        // frame clamp is what stops that from being unbounded.
        let fast = budget.plan(1.0 / 30.0, rpm_to_omega(20_000.0));
        approx(fast.dtheta, budget.max_dtheta, 1e-3);
        assert!(fast.substeps > budget.target_substeps);
        assert!(fast.substeps <= budget.max_substeps);

        // Garbage input degrades to a dropped frame rather than a NaN plan.
        for bad in [f64::NAN, f64::INFINITY, -1.0] {
            let p = budget.plan(bad, omega);
            assert!(p.dt.is_finite() && p.dt >= 0.0 && p.dtheta.is_finite());
        }
        // A stopped engine asks for no substeps at all.
        assert_eq!(budget.plan(1.0 / 60.0, 0.0).substeps, 0);
    }

    #[test]
    fn crossed_is_wrap_aware() {
        assert!(crossed(deg(719.0), deg(4.0), deg(2.0)));
        assert!(crossed(deg(10.0), deg(4.0), deg(12.0)));
        assert!(!crossed(deg(10.0), deg(1.0), deg(20.0)));
        // The interval is half-open `[from, from + dtheta)`: the start angle
        // counts, the end angle belongs to the next step. That is what makes
        // consecutive steps catch a target exactly once.
        assert!(crossed(deg(20.0), deg(4.0), deg(20.0)));
        assert!(!crossed(deg(20.0), deg(4.0), deg(24.0)));
        assert!(crossed(deg(24.0), deg(4.0), deg(24.0)));

        // Walking a whole cycle at any stride must trip a given target once.
        for stride_deg in [0.25, 1.0, 3.0, 4.0] {
            let stride = deg(stride_deg);
            let steps = (CYCLE_ANGLE / stride).round() as usize;
            let hits = (0..steps)
                .filter(|i| crossed(stride * *i as f64, stride, deg(220.0)))
                .count();
            assert_eq!(hits, 1, "target hit {hits} times at {stride_deg} deg/step");
        }
    }

    /// Walks a full four-stroke cycle with the valves live and checks the gas
    /// exchange actually happened: the cylinder must breathe in and blow down.
    #[test]
    fn full_cycle_breathes_and_stays_bounded() {
        let model = CylinderModel::default();
        let env = Environment::default();
        let ports = PortConditions::from_environment(&env, &model.gas);
        let solver = Rk4Solver::default();
        let omega = rpm_to_omega(3000.0);

        let mut st = ThermoState::at_ambient(&model.geometry, &model.gas, &env);
        let mut min_mass = f64::MAX;
        let mut max_mass: f64 = 0.0;
        let mut max_pressure: f64 = 0.0;

        // Six cycles: enough for the trapped mass to settle into a limit cycle.
        for _ in 0..(720 * 6) {
            let previous = st.cylinder.theta;
            st = solver.substep(&model, &st, omega, deg(1.0), &ports);
            if crossed(previous, deg(1.0), model.valves.intake.close_angle()) {
                st.latch = CycleLatch {
                    fuel_mass: model
                        .trapped_fuel_mass(st.cylinder.mass, st.cylinder.burned_fraction),
                    pressure: st.cylinder.pressure(&model.geometry, &model.gas),
                    temperature: st.cylinder.temperature,
                    volume: model.geometry.safe_volume(st.cylinder.theta),
                    gamma: model.gas.gamma(st.cylinder.burned_fraction),
                };
            }
            let p = st.cylinder.pressure(&model.geometry, &model.gas);
            assert!(p.is_finite() && st.cylinder.temperature.is_finite());
            min_mass = min_mass.min(st.cylinder.mass);
            max_mass = max_mass.max(st.cylinder.mass);
            max_pressure = max_pressure.max(p);
        }

        assert!(
            max_mass > min_mass * 5.0,
            "cylinder did not breathe: mass ranged {min_mass} to {max_mass}"
        );
        assert!(
            (20e5..250e5).contains(&max_pressure),
            "peak pressure {max_pressure} Pa is outside any plausible SI engine"
        );
    }

    #[test]
    fn a_stopped_engine_does_not_move_the_state() {
        let model = CylinderModel::default();
        let env = Environment::default();
        let ports = PortConditions::from_environment(&env, &model.gas);
        let solver = Rk4Solver::default();
        let st = ThermoState::at_ambient(&model.geometry, &model.gas, &env);
        assert_eq!(solver.substep(&model, &st, 0.0, deg(1.0), &ports), st);
    }
}
