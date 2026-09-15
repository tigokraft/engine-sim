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

/// Lower heating value of automotive diesel [J/kg].
///
/// Slightly below gasoline's per kilogram and well above it per litre, which is
/// most of why a diesel car goes further on a tank.
pub const DIESEL_LHV: f64 = 42.6e6;
/// Standard atmosphere, used to non-dimensionalise the knock correlation [Pa].
pub const ATMOSPHERE: f64 = 101_325.0;

/// Smallest mass we will divide by, so an emptied cylinder cannot produce an
/// infinite `dT/dt` [kg].
const MASS_FLOOR: f64 = 1e-9;

// ---------------------------------------------------------------------------
// Combustion: Wiebe heat release
// ---------------------------------------------------------------------------

/// Residual mass fraction at which a spark flame will not propagate [-].
///
/// Trapped exhaust gas dilutes the fresh charge without contributing to the
/// burn, and a homogeneous-charge spark flame gives up somewhere around a
/// third of the trapped mass being inert. Production engines schedule external
/// EGR to stay under roughly 0.25 for exactly this reason; a big-overlap cam
/// at idle, with no plenum pressure to stop the exhaust coming back through
/// the overlap, walks straight up to it. See [`WiebeProfile::diluted`].
pub const DILUTION_LIMIT: f64 = 0.35;

/// Normalised flame speed floor at and past [`DILUTION_LIMIT`] [-].
///
/// A flame speed of exactly zero is a burn of infinite duration and zero
/// completeness, which is a divide by zero rather than a misfire. This is what
/// a misfire is instead: a burn twenty times too slow that consumes five per
/// cent of the charge, which leaves the cycle with no useful work and a
/// cylinder full of fuel — the correct outcome, and a finite one.
pub const MISFIRE_FLAME_SPEED: f64 = 0.05;

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

    /// The profile a charge diluted by `residual_fraction` of burned gas
    /// actually burns to.
    ///
    /// Residual exhaust gas trapped with the fresh charge is inert: it absorbs
    /// heat without releasing any, so it drops the flame temperature and with
    /// it the laminar flame speed. Taking that fall as linear in residual
    /// fraction, towards zero at [`DILUTION_LIMIT`], gives a single normalised
    /// flame speed `f` that both of the Wiebe's shape parameters hang off:
    ///
    /// ```text
    /// f          = 1 - x_res / DILUTION_LIMIT
    /// dtheta_eff = dtheta / sqrt(f)     a slower flame takes longer
    /// a_eff      = a * f                and quenches before it finishes
    /// ```
    ///
    /// The two exponents are different because the two effects are. What sets
    /// the *duration* is the turbulent burning velocity, and Damkoehler's
    /// scaling makes that go as `sqrt(S_L u')` — the turbulence the piston
    /// stirred up does not care what is in the charge, so halving the laminar
    /// flame speed lengthens the burn by only the square root of two. What
    /// sets the *completeness* is the laminar flame alone, in the crevices and
    /// the cold boundary layer where turbulence has died away and only `S_L`
    /// decides whether the flame gets there before it is quenched.
    ///
    /// The second line is the one this stage exists for. `a` is by definition
    /// how much of the charge is consumed by the end of the burn — `x_b(1) =
    /// 1 - exp(-a)` — so lowering it is *partial burn*, not merely slow burn:
    /// the cycle ends with `exp(-a_eff)` of its fuel never lit, and that fuel
    /// leaves through the exhaust valve. It is the same quantity the backfire
    /// voice keys off, which is why an engine lopey enough to partial-burn
    /// crackles at idle without anything being told to make it crackle.
    ///
    /// Exactly the identity at zero residual, so a cylinder that scavenged
    /// cleanly burns precisely as it did before this existed.
    pub fn diluted(&self, residual_fraction: f64) -> Self {
        let saturation = (residual_fraction.max(0.0) / DILUTION_LIMIT).min(1.0);
        let flame_speed = (1.0 - saturation * saturation).clamp(MISFIRE_FLAME_SPEED, 1.0);
        Self {
            duration: (self.duration / flame_speed.sqrt()).min(CYCLE_ANGLE),
            efficiency_parameter: self.efficiency_parameter * flame_speed,
            ..*self
        }
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
// Compression ignition: the Arrhenius delay of a diesel spray
// ---------------------------------------------------------------------------

/// Arrhenius ignition delay of diesel fuel sprayed into hot compressed air.
///
/// ```text
/// tau = A * ((CN_ref - CN_0) / (CN - CN_0))^b * (P / 1 bar)^-n * exp(T_a / T)
/// ```
///
/// The same law as [`KnockModel`], and deliberately so: autoignition is one
/// chemical process, and whether it is a catastrophe or the entire operating
/// principle depends only on which engine it happens in. A petrol engine is
/// built so the Livengood-Wu integral never reaches one before the flame gets
/// there; a diesel is built so it always does, a few crank degrees after the
/// injector opens. Both are `tau = A P^-n exp(E_a / R T)` with a fuel-quality
/// prefactor in front, and the only real difference is the direction the rating
/// pulls: octane resists ignition, cetane promotes it.
///
/// The coefficients are Wolfer's, which is the fit every diesel delay
/// correlation since is measured against, with the cetane term from
/// Hardenberg and Hase normalised to [`REFERENCE_CETANE`] so that the
/// pre-exponential stays Wolfer's own number.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IgnitionDelay {
    /// Cetane number of the fuel [-].
    pub cetane_number: f64,
    /// Pre-exponential time constant [s].
    pub time_constant: f64,
    /// Cetane exponent (positive: more cetane shortens the delay) [-].
    pub cetane_exponent: f64,
    /// Pressure exponent (negative: pressure shortens the delay) [-].
    pub pressure_exponent: f64,
    /// Reduced activation temperature `E_a / R_u` [K].
    pub activation_temperature: f64,
}

/// Cetane number the pre-exponential is quoted at [-].
///
/// Pump diesel in Europe is 51 minimum and in North America 40 to 45; fifty is
/// the middle of that and the number the reference delay belongs to.
pub const REFERENCE_CETANE: f64 = 50.0;

/// Cetane number at which Hardenberg and Hase's delay term goes singular [-].
///
/// Their fit divides by `CN - 17.2`, which says that a fuel below roughly
/// cetane 17 will not compression-ignite at all. It is a real edge of the
/// correlation rather than a numerical one, and it is why the cetane number is
/// clamped above it rather than floored at zero.
pub const CETANE_FLOOR: f64 = 17.2;

/// One bar, the unit Wolfer's pressure term is quoted in [Pa].
pub const BAR: f64 = 1.0e5;

impl Default for IgnitionDelay {
    /// Cetane 50 pump diesel on Wolfer's coefficients.
    fn default() -> Self {
        Self {
            cetane_number: REFERENCE_CETANE,
            time_constant: 0.44e-3,
            cetane_exponent: 0.63,
            pressure_exponent: -1.19,
            activation_temperature: 4_650.0,
        }
    }
}

impl IgnitionDelay {
    /// Ignition delay of the spray at a given charge pressure and temperature [s].
    pub fn delay(&self, pressure: f64, temperature: f64) -> f64 {
        let p_bar = (pressure / BAR).max(1e-3);
        let t = temperature.clamp(200.0, 4000.0);
        let cn = self.cetane_number.max(CETANE_FLOOR + 1.0);
        let cetane =
            ((REFERENCE_CETANE - CETANE_FLOOR) / (cn - CETANE_FLOOR)).powf(self.cetane_exponent);
        let tau = self.time_constant
            * cetane
            * p_bar.powf(self.pressure_exponent)
            * (self.activation_temperature / t).exp();
        // The same guard `KnockModel` needs and for the same reason: below
        // ~500 K the exponential runs away, and `1/tau` has to stay a finite,
        // well-scaled rate at both ends.
        tau.clamp(1e-6, 1e9)
    }

    /// Rate of the Livengood-Wu integral, `dI/dt = 1 / tau` [1/s].
    pub fn rate(&self, pressure: f64, temperature: f64) -> f64 {
        1.0 / self.delay(pressure, temperature)
    }
}

/// Where and how hard this cycle's charge lit itself.
///
/// Solved once per cycle, at the latch, because none of it is a function of the
/// integrated state: the charge is pure air on a known isentrope from intake
/// valve close until the moment it ignites, so where that moment falls is
/// decided entirely by what was trapped. See
/// [`DieselCombustion::autoignition`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Autoignition {
    /// Crank angle at which the Livengood-Wu integral reached one [rad, cycle coords].
    pub angle: f64,
    /// Ignition delay measured from the start of injection [s].
    pub delay: f64,
    /// Fraction of the charge injected before ignition, and so burned premixed [-].
    pub premixed_fraction: f64,
}

/// Crank-angle step the autoignition march takes [rad].
///
/// A quarter of a degree. The integrand is `1/tau` on a charge whose
/// temperature is climbing through the steepest part of the compression, so it
/// varies by a factor of ten over the last ten degrees before top dead centre;
/// a quarter degree resolves that to well under a tenth of a degree of ignition
/// angle, which is finer than the burn it decides.
const AUTOIGNITION_STEP: f64 = PI / 720.0;

/// How far past the start of injection the march looks before giving up [rad].
///
/// Sixty degrees. A charge that has not lit by then is on the expansion stroke
/// with a falling temperature and is not going to: the cycle misfires, which is
/// what a diesel cranking on a winter morning actually does.
const AUTOIGNITION_WINDOW: f64 = PI / 3.0;

/// Two-stage Wiebe heat release for a compression-ignition engine.
///
/// ```text
/// x_b(theta) = f * wiebe(theta - theta_ign; delta_p, n_p)
///            + (1 - f) * wiebe(theta - theta_ign; delta_d, n_d)
/// ```
///
/// Two burns, not one, because a diesel really does burn twice. Fuel sprayed
/// into air that is not yet hot enough to light it just sits there evaporating
/// and mixing; when the air finally does reach the temperature, everything that
/// arrived in the meantime goes off at once, in a *premixed* spike a few crank
/// degrees wide. Only after that does the engine settle into the burn it is
/// named for — *diffusion*, where the rate is set by how fast the spray can
/// find oxygen, and which runs on for most of the expansion stroke.
///
/// The split `f` between them is not a parameter. It is the fraction of the
/// charge the injector managed to deliver before ignition, so it falls straight
/// out of the delay:
///
/// ```text
/// f = (theta_ign - theta_inj) / delta_inj
/// ```
///
/// which is why a cold diesel clatters and a hot one does not. A long delay
/// piles up fuel; the pile goes off in one piece; `dP/dtheta` goes through the
/// roof and the block is hit with it. Nothing about that is voiced. It is the
/// arithmetic above and [`crate::audio::structure`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DieselCombustion {
    /// Crank angle at which the injector opens [rad, cycle coords].
    pub injection_angle: f64,
    /// Crank angle over which the injector delivers the charge [rad].
    pub injection_duration: f64,
    /// Nominal duration of the premixed spike [rad].
    pub premixed_duration: f64,
    /// Nominal duration of the diffusion burn [rad].
    pub diffusion_duration: f64,
    /// Wiebe form factor of the premixed spike [-].
    pub premixed_form_factor: f64,
    /// Wiebe form factor of the diffusion burn [-].
    pub diffusion_form_factor: f64,
    /// Wiebe efficiency parameter `a`, shared by both stages [-].
    pub efficiency_parameter: f64,
    /// Fraction of the fuel's chemical energy that shows up as heat [-].
    pub combustion_efficiency: f64,
    /// The Arrhenius delay correlation the ignition angle is solved from.
    pub delay: IgnitionDelay,
}

impl Default for DieselCombustion {
    /// A direct-injection passenger diesel: injection 8 degrees before top dead
    /// centre over 25 degrees, a 9-degree premixed spike and a 65-degree
    /// diffusion tail.
    fn default() -> Self {
        Self {
            injection_angle: deg(352.0),
            injection_duration: deg(25.0),
            premixed_duration: deg(9.0),
            diffusion_duration: deg(65.0),
            // Below one the Wiebe rate peaks almost at its own start, which is
            // what a premixed spike is: everything already mixed, lighting at
            // once. Above one it has to build, which is what diffusion is.
            premixed_form_factor: 0.6,
            diffusion_form_factor: 1.2,
            efficiency_parameter: 5.0,
            // A diesel runs lean and sooty: less of the fuel finds oxygen in
            // time than in a homogeneous petrol charge.
            combustion_efficiency: 0.95,
            delay: IgnitionDelay::default(),
        }
    }
}

impl DieselCombustion {
    /// Solves the ignition point for a charge trapped as `latch` describes.
    ///
    /// Marches the Livengood-Wu integral forward from the start of injection
    /// along the motored compression isentrope — which is what the charge
    /// genuinely rides, since nothing has burned yet — and stops where it
    /// reaches one:
    ///
    /// ```text
    /// I = integral_{theta_inj}^{theta_ign} dtheta / (omega tau(P_mot, T_mot)) = 1
    /// ```
    ///
    /// Returns `None` when the integral never gets there inside
    /// [`AUTOIGNITION_WINDOW`]: the charge was too cold to light and the cycle
    /// misfires.
    ///
    /// `omega` is in the integrand, not decoration — the delay is a *time*, and
    /// the faster the crank turns the more degrees of it go by. That is the
    /// whole reason a diesel's ignition retards with speed without anyone
    /// scheduling it.
    pub fn autoignition(
        &self,
        latch: &CycleLatch,
        geometry: &CylinderGeometry,
        omega: f64,
    ) -> Option<Autoignition> {
        let omega = omega.abs().max(1e-3);
        let mut integral = 0.0;
        let mut travelled = 0.0;
        while travelled < AUTOIGNITION_WINDOW {
            // Midpoint of the step: second order, and it keeps the first
            // sample off the injection angle itself where nothing has mixed.
            let theta = self.injection_angle + travelled + 0.5 * AUTOIGNITION_STEP;
            let pressure = latch.motored_pressure(geometry.safe_volume(theta));
            let temperature = latch.end_gas_temperature(pressure);
            integral += AUTOIGNITION_STEP / omega * self.delay.rate(pressure, temperature);
            travelled += AUTOIGNITION_STEP;
            if integral >= 1.0 {
                return Some(Autoignition {
                    angle: wrap_cycle(self.injection_angle + travelled),
                    delay: travelled / omega,
                    premixed_fraction: self.premixed_fraction(travelled),
                });
            }
        }
        None
    }

    /// Fraction of the charge delivered during a delay of `travelled` radians [-].
    ///
    /// Floored rather than allowed to reach zero because an injector that has
    /// only just cracked open still has a spray cone in the chamber, and capped
    /// short of one because the tail of the delivery is still arriving into a
    /// fire however long the delay was.
    pub fn premixed_fraction(&self, travelled: f64) -> f64 {
        (travelled / self.injection_duration.max(1e-6)).clamp(0.02, 0.9)
    }

    /// True while either stage is still releasing heat.
    pub fn is_burning(&self, theta: f64, ignition: &Autoignition) -> bool {
        wrap_cycle(theta - ignition.angle) < self.premixed_duration.max(self.diffusion_duration)
    }

    /// Burned mass fraction of the two stages summed [-].
    pub fn burned_fraction(&self, theta: f64, ignition: &Autoignition) -> f64 {
        let phase = wrap_cycle(theta - ignition.angle);
        let f = ignition.premixed_fraction;
        f * self.stage_fraction(phase, self.premixed_duration, self.premixed_form_factor)
            + (1.0 - f)
                * self.stage_fraction(phase, self.diffusion_duration, self.diffusion_form_factor)
    }

    /// Burn rate of the two stages summed [1/rad].
    pub fn dburned_dtheta(&self, theta: f64, ignition: &Autoignition) -> f64 {
        let phase = wrap_cycle(theta - ignition.angle);
        let f = ignition.premixed_fraction;
        f * self.stage_rate(phase, self.premixed_duration, self.premixed_form_factor)
            + (1.0 - f)
                * self.stage_rate(phase, self.diffusion_duration, self.diffusion_form_factor)
    }

    /// One stage's Wiebe fraction at a phase past ignition [-].
    fn stage_fraction(&self, phase: f64, duration: f64, form_factor: f64) -> f64 {
        if phase >= duration {
            return 1.0;
        }
        let tau = (phase / duration.max(1e-9)).max(0.0);
        1.0 - (-self.efficiency_parameter * tau.powf(form_factor + 1.0)).exp()
    }

    /// One stage's Wiebe rate at a phase past ignition [1/rad].
    fn stage_rate(&self, phase: f64, duration: f64, form_factor: f64) -> f64 {
        if phase >= duration || phase < 0.0 {
            return 0.0;
        }
        let duration = duration.max(1e-9);
        let tau = phase / duration;
        let a = self.efficiency_parameter;
        let np1 = form_factor + 1.0;
        a * np1 / duration * tau.powf(form_factor) * (-a * tau.powf(np1)).exp()
    }
}

/// What lights the charge, and therefore what shape the heat release has.
///
/// The two topologies are not a parameter of one model. A spark engine's burn
/// starts when the coil fires, at an angle the ECU chooses, and proceeds as one
/// flame front through a charge that was mixed long before; a compression
/// engine's starts when the air gets hot enough, at an angle nobody chooses,
/// and proceeds in two stages because the fuel arrives in the middle of it.
/// Every consumer of a heat release has to know which it is holding, which is
/// what an enum is for.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HeatRelease {
    /// Spark ignition: one Wiebe from the spark.
    Spark(WiebeProfile),
    /// Compression ignition: two Wiebes from a solved autoignition point.
    Compression(DieselCombustion),
}

impl Default for HeatRelease {
    fn default() -> Self {
        Self::Spark(WiebeProfile::default())
    }
}

impl HeatRelease {
    /// Burn rate with respect to crank angle [1/rad].
    ///
    /// A compression-ignition cycle whose charge never autoignited releases no
    /// heat at all: that is a misfire, and it is the honest answer rather than
    /// a fallback.
    pub fn dburned_dtheta(&self, theta: f64, latch: &CycleLatch) -> f64 {
        match self {
            // A spark flame has to cross a chamber, so what is in the way of
            // it matters; a diesel's heat release is rate-limited by
            // injection and mixing instead, and it is unthrottled, so it
            // never sees the residual fraction a throttled idle does. Only
            // the spark engine is diluted here, and that is why.
            Self::Spark(wiebe) => wiebe.diluted(latch.dilution).dburned_dtheta(theta),
            Self::Compression(diesel) => match &latch.autoignition {
                Some(ignition) => diesel.dburned_dtheta(theta, ignition),
                None => 0.0,
            },
        }
    }

    /// True while heat release is actively happening.
    pub fn is_burning(&self, theta: f64, latch: &CycleLatch) -> bool {
        match self {
            Self::Spark(wiebe) => wiebe.diluted(latch.dilution).is_burning(theta),
            Self::Compression(diesel) => match &latch.autoignition {
                Some(ignition) => diesel.is_burning(theta, ignition),
                None => false,
            },
        }
    }

    /// Fraction of the fuel's chemical energy that shows up as heat [-].
    pub fn combustion_efficiency(&self) -> f64 {
        match self {
            Self::Spark(wiebe) => wiebe.combustion_efficiency,
            Self::Compression(diesel) => diesel.combustion_efficiency,
        }
    }

    /// Where heat release is commanded from: the spark, or the injector opening
    /// [rad, cycle coords].
    ///
    /// Not where it starts on a diesel — that is
    /// [`Autoignition::angle`], and it is solved rather than commanded.
    pub fn commanded_angle(&self) -> f64 {
        match self {
            Self::Spark(wiebe) => wiebe.spark_angle,
            Self::Compression(diesel) => diesel.injection_angle,
        }
    }

    /// Nominal angular extent of the burn [rad].
    pub fn duration(&self) -> f64 {
        match self {
            Self::Spark(wiebe) => wiebe.duration,
            Self::Compression(diesel) => diesel.diffusion_duration,
        }
    }

    /// The spark profile, on an engine that has a spark plug.
    pub fn spark(&self) -> Option<&WiebeProfile> {
        match self {
            Self::Spark(wiebe) => Some(wiebe),
            Self::Compression(_) => None,
        }
    }

    /// The spark profile for the ECU to retime, on an engine that has one.
    pub fn spark_mut(&mut self) -> Option<&mut WiebeProfile> {
        match self {
            Self::Spark(wiebe) => Some(wiebe),
            Self::Compression(_) => None,
        }
    }

    /// The compression-ignition profile, on an engine that has no spark plug.
    pub fn compression(&self) -> Option<&DieselCombustion> {
        match self {
            Self::Spark(_) => None,
            Self::Compression(diesel) => Some(diesel),
        }
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

    /// Intake lobe centreline, as crank angle after gas-exchange TDC [rad].
    ///
    /// Half a duration past the opening point, which is where a cam card
    /// measures it from: the lobe is symmetric about its own peak, so the
    /// centreline is the one timing figure that does not move when the
    /// duration is re-specified at a different checking height.
    pub fn intake_centreline(&self) -> f64 {
        wrap_cycle(self.intake.open_angle + self.intake.duration / 2.0)
    }

    /// Exhaust lobe centreline, as crank angle *before* gas-exchange TDC [rad].
    ///
    /// Measured backwards for the same reason a cam card does: the exhaust
    /// lobe sits before TDC and the intake lobe after it, so quoting both as
    /// distances from TDC makes them two positive numbers that straddle it.
    pub fn exhaust_centreline(&self) -> f64 {
        wrap_cycle(-(self.exhaust.open_angle + self.exhaust.duration / 2.0))
    }

    /// Lobe separation angle: the mean of the two centrelines [rad].
    ///
    /// ```text
    /// LSA = (ICL + ECL) / 2
    /// ```
    ///
    /// This is how a camshaft is actually specified, and it is a property of
    /// the billet: the angle between the two lobes is ground in and cannot be
    /// changed by turning the cam in its drive. It is quoted in camshaft
    /// degrees, which is why it comes out as the *mean* of two crank angles
    /// rather than their sum — the cam turns at half crank speed. A tight
    /// separation puts the two lobes close together and is what buys overlap;
    /// see [`ValveTrain::overlap`].
    pub fn lobe_separation(&self) -> f64 {
        (self.intake_centreline() + self.exhaust_centreline()) / 2.0
    }

    /// Cam advance: half the difference of the two centrelines [rad, crank].
    ///
    /// ```text
    /// advance = (ECL - ICL) / 2
    /// ```
    ///
    /// The one timing figure that *is* adjustable after the cam is ground —
    /// turning the whole cam ahead of the crank moves both lobes earlier by
    /// the same crank angle, which raises `ECL` and lowers `ICL` without
    /// touching their mean. Positive is advanced: cylinder pressure peaks
    /// earlier, low-speed torque rises and the top end gives up.
    pub fn advance(&self) -> f64 {
        (self.exhaust_centreline() - self.intake_centreline()) / 2.0
    }

    /// Crank angle with both valves off their seats [rad].
    ///
    /// ```text
    /// overlap = (duration_i + duration_e) / 2 - 2 * LSA
    /// ```
    ///
    /// Never specified directly on a cam card, because it is not a free
    /// parameter: it falls out of the two durations and the separation. That
    /// is the whole reason this stage re-specified the cam — overlap is the
    /// term the reversion, the residual and therefore the idle all key off,
    /// and it was previously only reachable by moving two absolute angles in
    /// opposite directions and hoping.
    pub fn overlap(&self) -> f64 {
        (self.intake.duration + self.exhaust.duration) / 2.0 - 2.0 * self.lobe_separation()
    }

    /// Re-times both lobes onto a lobe separation and an advance [rad].
    ///
    /// The inverse of [`ValveTrain::lobe_separation`] and
    /// [`ValveTrain::advance`]: durations, lifts, diameters and discharge
    /// coefficients are the lobe's own and are carried through untouched, and
    /// only the two opening angles move. Handing it a train's own separation
    /// and advance therefore returns that train.
    pub fn with_cam_timing(mut self, lobe_separation: f64, advance: f64) -> Self {
        let intake_centreline = lobe_separation - advance;
        let exhaust_centreline = lobe_separation + advance;
        self.intake.open_angle = wrap_cycle(intake_centreline - self.intake.duration / 2.0);
        self.exhaust.open_angle = wrap_cycle(-exhaust_centreline - self.exhaust.duration / 2.0);
        self
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
    /// Residual mass fraction of the trapped charge [-].
    ///
    /// What fraction of what the cylinder sealed at IVC is burned gas rather
    /// than fresh mixture — the exhaust it failed to scavenge, plus whatever
    /// came back up the port during overlap. Latched with the rest of the
    /// cycle references because it is a property of the trapped charge and
    /// must not move between RK4 stages, and read by
    /// [`WiebeProfile::diluted`], which is where it costs the burn.
    pub dilution: f64,
    /// Cylinder pressure at IVC [Pa].
    pub pressure: f64,
    /// Cylinder temperature at IVC [K].
    pub temperature: f64,
    /// Cylinder volume at IVC [m^3].
    pub volume: f64,
    /// Ratio of specific heats of the trapped charge at IVC [-].
    pub gamma: f64,
    /// Where this cycle's charge lit itself, on a compression-ignition engine.
    ///
    /// `None` on a spark engine — there is nothing to solve, the coil decides —
    /// and also on a compression engine whose charge was too cold to light,
    /// which is a misfire. See [`DieselCombustion::autoignition`].
    pub autoignition: Option<Autoignition>,
}

impl CycleLatch {
    /// A latch for an unfuelled cylinder full of ambient air.
    pub fn ambient(geometry: &CylinderGeometry, gas: &GasProperties, env: &Environment) -> Self {
        Self {
            fuel_mass: 0.0,
            dilution: 0.0,
            pressure: env.pressure,
            temperature: env.temperature,
            volume: geometry.max_volume(),
            gamma: gas.gamma_unburned,
            autoignition: None,
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
    /// Heat release: a spark engine's single Wiebe or a diesel's two.
    pub combustion: HeatRelease,
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
    /// Whether fuel delivery is cut this cycle (DFCO or fuel-cut limiter).
    pub fuel_cut: bool,
}

impl Default for CylinderModel {
    fn default() -> Self {
        Self {
            geometry: CylinderGeometry::default(),
            gas: GasProperties::default(),
            combustion: HeatRelease::default(),
            heat: WoschniModel::default(),
            valves: ValveTrain::default(),
            knock: KnockModel::default(),
            fuel_lhv: GASOLINE_LHV,
            air_fuel_ratio: STOICH_AFR,
            fuel_cut: false,
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
            self.combustion.dburned_dtheta(theta, &st.latch) * omega
        } else {
            0.0
        };
        let phi = STOICH_AFR / self.air_fuel_ratio.max(1e-3);
        let richness_efficiency = if phi > 1.0 {
            (1.0 / phi) * (1.0 - 0.15 * (phi - 1.0)).max(0.7)
        } else {
            1.0
        };
        let heat_release = st.latch.fuel_mass
            * self.fuel_lhv
            * (self.combustion.combustion_efficiency() * richness_efficiency)
            * dxb_dt;

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
        if st.cylinder.burned_fraction > 1e-6 || self.combustion.is_burning(theta, &st.latch) {
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
            self.combustion.dburned_dtheta(theta, &st.latch) * omega
        } else {
            0.0
        };
        let phi = STOICH_AFR / self.air_fuel_ratio.max(1e-3);
        let richness_efficiency = if phi > 1.0 {
            (1.0 / phi) * (1.0 - 0.15 * (phi - 1.0)).max(0.7)
        } else {
            1.0
        };
        let heat_release = st.latch.fuel_mass
            * self.fuel_lhv
            * (self.combustion.combustion_efficiency() * richness_efficiency)
            * dxb_chem;

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

    /// The per-cycle references implied by the charge trapped in `cylinder`.
    ///
    /// On a compression-ignition engine this is also where the cycle's ignition
    /// point is solved, because this is the moment everything it depends on is
    /// known: the charge is sealed, nothing has burned, and the isentrope it
    /// will ride to the injector opening is fixed. Solving it once here rather
    /// than inside the derivative is not an optimisation — the derivative is
    /// evaluated four times per substep at angles the crank has not reached,
    /// and an ignition angle that moved between RK4 stages would not be one.
    pub fn latch(&self, cylinder: &CylinderState, omega: f64) -> CycleLatch {
        let mut latch = CycleLatch {
            fuel_mass: self.trapped_fuel_mass(cylinder.mass, cylinder.burned_fraction),
            dilution: cylinder.burned_fraction.clamp(0.0, 1.0),
            pressure: cylinder.pressure(&self.geometry, &self.gas),
            temperature: cylinder.temperature,
            volume: self.geometry.safe_volume(cylinder.theta),
            gamma: self.gas.gamma(cylinder.burned_fraction),
            autoignition: None,
        };
        if latch.fuel_mass > 0.0 {
            if let HeatRelease::Compression(diesel) = &self.combustion {
                latch.autoignition = diesel.autoignition(&latch, &self.geometry, omega);
            }
        }
        latch
    }

    /// Trapped fuel mass implied by a charge mass and its residual fraction [kg].
    ///
    /// Only the *fresh* part of the trapped mass carries fuel, and it arrives as
    /// a metered mixture, so `m_fuel = m (1 - x_b) / (1 + AFR)`.
    pub fn trapped_fuel_mass(&self, mass: f64, burned_fraction: f64) -> f64 {
        if self.fuel_cut {
            return 0.0;
        }
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
                self.latch_cycle(model, st, omega);
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
    fn latch_cycle(&self, model: &CylinderModel, st: &mut ThermoState, omega: f64) {
        st.latch = model.latch(&st.cylinder, omega);
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
        if let Some(wiebe) = m.combustion.spark_mut() {
            wiebe.combustion_efficiency = 0.0;
        }
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
    fn an_undiluted_charge_burns_exactly_as_it_did_before_dilution_existed() {
        // The neutrality half of the dilution model. A cylinder that scavenged
        // perfectly has nothing inert in it, so nothing about its burn may
        // move — not by a tolerance, exactly.
        let wiebe = WiebeProfile::default();
        assert_eq!(wiebe.diluted(0.0), wiebe);
        assert_eq!(
            wiebe.diluted(-1.0),
            wiebe,
            "a negative residual is no residual"
        );
    }

    #[test]
    fn a_diluted_charge_burns_slower_and_less_completely() {
        // Both halves of the claim, against the analytic Wiebe rather than
        // against a rendered anything. `x_b` at the end of the nominal burn is
        // `1 - exp(-a)` by construction, so the completeness is readable
        // straight off the efficiency parameter.
        let wiebe = WiebeProfile::default();
        let clean = 1.0 - (-wiebe.efficiency_parameter).exp();

        let mut previous_completeness = clean;
        let mut previous_duration = wiebe.duration;
        for residual in [0.05, 0.10, 0.20, 0.30] {
            let diluted = wiebe.diluted(residual);
            let completeness = 1.0 - (-diluted.efficiency_parameter).exp();
            assert!(
                completeness < previous_completeness,
                "burn completeness must fall with residual: {completeness:.4} at \
                 {residual} is not under {previous_completeness:.4}"
            );
            assert!(
                diluted.duration > previous_duration,
                "burn duration must rise with residual: {:.1} deg at {residual} is \
                 not over {:.1} deg",
                diluted.duration.to_degrees(),
                previous_duration.to_degrees()
            );
            previous_completeness = completeness;
            previous_duration = diluted.duration;
        }

        // At the dilution limit the flame is a misfire rather than a divide by
        // zero: a finite, very long burn that consumes almost nothing.
        let dead = wiebe.diluted(DILUTION_LIMIT);
        assert!(dead.duration.is_finite() && dead.duration > 4.0 * wiebe.duration);
        let dead_completeness = 1.0 - (-dead.efficiency_parameter).exp();
        assert!(
            dead_completeness < 0.3,
            "a charge past the dilution limit must barely light: {dead_completeness:.3}"
        );

        // And the fuel that does not burn is what leaves through the port.
        // A tenth residual is a percent or two of the charge's fuel going out
        // unburnt, which is the quantity the backfire voice reads.
        let lopey = wiebe.diluted(0.10);
        let unburnt = (-lopey.efficiency_parameter).exp();
        assert!(
            (0.005..0.10).contains(&unburnt),
            "a tenth residual should leave a few per cent of the fuel unburnt, got {unburnt:.4}"
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
            dilution: 0.0,
            pressure: env.pressure,
            temperature: env.temperature,
            volume: model.geometry.max_volume(),
            gamma: model.gas.gamma_unburned,
            autoignition: None,
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

    /// A charge lit exactly at top dead centre, so the two-stage shape can be
    /// looked at on its own without solving a cycle for it.
    fn lit_at_tdc(premixed_fraction: f64) -> Autoignition {
        Autoignition {
            angle: deg(360.0),
            delay: 1.0e-3,
            premixed_fraction,
        }
    }

    #[test]
    fn two_stage_burn_spikes_before_it_humps() {
        // The shape the whole stage exists for: a narrow premixed spike a
        // degree or two after ignition, then a broad diffusion hump twenty-odd
        // degrees later. One Wiebe cannot do that, which is why there are two.
        let d = DieselCombustion::default();
        let ign = lit_at_tdc(0.3);
        let step = deg(0.1);
        let samples: Vec<(f64, f64)> = (0..900)
            .map(|i| {
                let phase = i as f64 * step;
                (phase, d.dburned_dtheta(ign.angle + phase, &ign))
            })
            .collect();

        let peaks: Vec<(f64, f64)> = samples
            .windows(3)
            .filter(|w| w[1].1 > w[0].1 && w[1].1 >= w[2].1)
            .map(|w| w[1])
            .collect();
        assert_eq!(
            peaks.len(),
            2,
            "a two-stage release must have two peaks, found {}: {:?}",
            peaks.len(),
            peaks
                .iter()
                .map(|(p, _)| p.to_degrees())
                .collect::<Vec<_>>()
        );

        let (premixed_at, premixed_rate) = peaks[0];
        let (diffusion_at, diffusion_rate) = peaks[1];
        assert!(
            premixed_at < deg(5.0),
            "the premixed spike arrived {:.1} degrees after ignition, not promptly",
            premixed_at.to_degrees()
        );
        assert!(
            diffusion_at > deg(10.0) && diffusion_at < deg(45.0),
            "the diffusion hump landed at {:.1} degrees",
            diffusion_at.to_degrees()
        );
        assert!(
            premixed_rate > 2.0 * diffusion_rate,
            "the spike ({premixed_rate:.2}/rad) is not sharper than the hump \
             ({diffusion_rate:.2}/rad), so it is not a spike"
        );
    }

    #[test]
    fn two_stage_rate_integrates_back_to_its_own_fraction() {
        let d = DieselCombustion::default();
        for f in [0.05, 0.3, 0.6] {
            let ign = lit_at_tdc(f);
            let steps = 40_000;
            let span = d.diffusion_duration.max(d.premixed_duration);
            let h = span / steps as f64;
            let mut x = 0.0;
            for i in 0..steps {
                x += d.dburned_dtheta(ign.angle + (i as f64 + 0.5) * h, &ign) * h;
            }
            // Both stages leave the usual exp(-a) unburned, so the pair does
            // too, whatever the split between them. The tolerance is the
            // quadrature's, not the model's: the spike's form factor is below
            // one, so its rate starts with an infinite slope and the midpoint
            // rule pays for that in the first degree.
            approx(x, 1.0 - (-d.efficiency_parameter).exp(), 1e-3);
            // The fraction the two stages report together is monotone and
            // arrives at the whole charge.
            let mut previous = 0.0;
            for i in 0..=720 {
                let at = d.burned_fraction(ign.angle + i as f64 * deg(0.1), &ign);
                assert!(at >= previous - 1e-12, "burn ran backwards at step {i}");
                previous = at;
            }
            assert!(
                previous > 0.99,
                "the charge never finished burning: {previous}"
            );
        }
    }

    /// A charge trapped at BDC at a stated pressure and temperature.
    fn trapped(geometry: &CylinderGeometry, pressure: f64, temperature: f64) -> CycleLatch {
        CycleLatch {
            fuel_mass: 1.0e-5,
            dilution: 0.0,
            pressure,
            temperature,
            volume: geometry.max_volume(),
            gamma: 1.38,
            autoignition: None,
        }
    }

    #[test]
    fn ignition_delay_lengthens_on_a_colder_compression() {
        // The whole reason a diesel is hard to start and clatters when it is:
        // the charge has to reach the temperature on its own, and a cylinder
        // that starts colder takes longer to get there. Nothing schedules this.
        let diesel = DieselCombustion::default();
        let geometry = CylinderGeometry::new(0.083, 0.092, 0.147, 18.0);
        let omega = rpm_to_omega(1_500.0);

        let hot = diesel
            .autoignition(&trapped(&geometry, 1.6e5, 330.0), &geometry, omega)
            .expect("a warm charge must light");
        let cold = diesel
            .autoignition(&trapped(&geometry, 1.6e5, 290.0), &geometry, omega)
            .expect("a cool charge must still light on 18:1");

        assert!(
            cold.delay > hot.delay,
            "a colder charge must be slower to light: {:.2} ms against {:.2} ms",
            cold.delay * 1e3,
            hot.delay * 1e3
        );
        assert!(
            cold.angle > hot.angle,
            "a longer delay must also put ignition later in the cycle"
        );
        // And the late one piles up more fuel while it waits, which is the
        // clatter.
        assert!(
            cold.premixed_fraction > hot.premixed_fraction,
            "the colder cycle did not premix more: {:.3} against {:.3}",
            cold.premixed_fraction,
            hot.premixed_fraction
        );
        // A real DI diesel lights within a millisecond or two of the injector.
        assert!(
            (0.2e-3..3.0e-3).contains(&hot.delay),
            "implausible delay: {:.2} ms",
            hot.delay * 1e3
        );
    }

    #[test]
    fn compression_ratio_is_what_makes_a_diesel_light_at_all() {
        let diesel = DieselCombustion::default();
        let omega = rpm_to_omega(1_500.0);
        let squeeze = |ratio: f64| {
            let geometry = CylinderGeometry::new(0.083, 0.092, 0.147, ratio);
            diesel.autoignition(&trapped(&geometry, 1.6e5, 330.0), &geometry, omega)
        };

        let high = squeeze(20.0).expect("20:1 must light");
        let low = squeeze(14.0).expect("14:1 must still light on a warm charge");
        assert!(
            low.delay > high.delay,
            "less squeeze must mean a longer wait: {:.2} ms against {:.2} ms",
            low.delay * 1e3,
            high.delay * 1e3
        );
        // A petrol engine's squeeze cannot light diesel at all, which is why
        // one has no injectors in it.
        assert!(
            squeeze(9.0).is_none(),
            "9:1 compression lit a diesel charge, which no engine has ever done"
        );
    }

    #[test]
    fn the_latch_solves_ignition_only_on_a_compression_engine() {
        let env = Environment::default();
        let omega = rpm_to_omega(1_500.0);
        let mut model = CylinderModel {
            geometry: CylinderGeometry::new(0.083, 0.092, 0.147, 18.0),
            ..CylinderModel::default()
        };
        let mut cylinder =
            CylinderState::at_ambient(&model.geometry, &model.gas, env.pressure, env.temperature);
        cylinder.theta = PI;
        cylinder.mass =
            env.pressure * model.geometry.max_volume() / (model.gas.r_unburned * env.temperature);

        assert!(
            model.latch(&cylinder, omega).autoignition.is_none(),
            "a spark engine has nothing to autoignite"
        );

        model.combustion = HeatRelease::Compression(DieselCombustion::default());
        let latch = model.latch(&cylinder, omega);
        let ignition = latch.autoignition.expect("the diesel charge must light");
        assert!(ignition.angle > model.combustion.commanded_angle());
        // And with the fuel cut there is no spray to light, so there is no
        // ignition point either.
        model.fuel_cut = true;
        assert!(model.latch(&cylinder, omega).autoignition.is_none());
    }

    #[test]
    fn a_longer_delay_puts_more_of_the_charge_in_the_spike() {
        // The one link that makes the clatter physical rather than voiced: the
        // split between the stages is the fraction the injector delivered while
        // the air was still too cold to light it.
        let d = DieselCombustion::default();
        let short = d.premixed_fraction(deg(3.0));
        let long = d.premixed_fraction(deg(12.0));
        assert!(
            long > short,
            "a longer delay must pile up more premixed fuel: {long:.3} against {short:.3}"
        );
        assert!((0.0..=1.0).contains(&d.premixed_fraction(deg(180.0))));
    }

    #[test]
    fn diesel_delay_shortens_with_cetane_pressure_and_temperature() {
        let pump = IgnitionDelay::default();
        let poor = IgnitionDelay {
            cetane_number: 35.0,
            ..IgnitionDelay::default()
        };

        let (p, t) = (50e5, 850.0);
        assert!(
            poor.delay(p, t) > pump.delay(p, t),
            "a low-cetane fuel must be slower to light, not faster"
        );
        // The reference fuel is quoted at the reference cetane, so its prefactor
        // is Wolfer's own number with nothing else on it.
        approx(
            IgnitionDelay {
                cetane_number: REFERENCE_CETANE,
                ..IgnitionDelay::default()
            }
            .delay(BAR, 0.5 * pump.activation_temperature),
            pump.time_constant * std::f64::consts::E.powi(2),
            1e-9,
        );
        assert!(pump.delay(80e5, t) < pump.delay(40e5, t));
        assert!(pump.delay(p, 950.0) < pump.delay(p, 750.0));
        // A real diesel lights within a millisecond or so of the injector
        // opening at top dead centre; anything else is a typo in a coefficient.
        let tdc = pump.delay(55e5, 900.0);
        assert!(
            (0.2e-3..2.0e-3).contains(&tdc),
            "implausible delay at TDC conditions: {:.3} ms",
            tdc * 1e3
        );
        assert!(pump.delay(1e5, 300.0).is_finite());
        assert!(pump.rate(p, t).is_finite() && pump.rate(p, t) > 0.0);
    }

    #[test]
    fn advancing_the_spark_drives_the_knock_integral_up() {
        let env = Environment::default();
        let omega = rpm_to_omega(3000.0);

        let run = |spark_deg: f64| {
            let model = CylinderModel {
                combustion: HeatRelease::Spark(WiebeProfile::new(
                    deg(spark_deg),
                    deg(60.0),
                    5.0,
                    2.0,
                    0.97,
                )),
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
                dilution: 0.0,
                pressure: st.cylinder.pressure(&model.geometry, &model.gas),
                temperature: st.cylinder.temperature,
                volume: model.geometry.max_volume(),
                gamma: model.gas.gamma_unburned,
                autoignition: None,
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
                    dilution: st.cylinder.burned_fraction,
                    pressure: st.cylinder.pressure(&model.geometry, &model.gas),
                    temperature: st.cylinder.temperature,
                    volume: model.geometry.safe_volume(st.cylinder.theta),
                    gamma: model.gas.gamma(st.cylinder.burned_fraction),
                    autoignition: None,
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
