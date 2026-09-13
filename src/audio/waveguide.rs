//! 1D digital waveguide network for engine exhaust and induction acoustics.
//!
//! # Waveguide acoustic principles
//!
//! Sound propagation in acoustic ducts is represented using digital waveguides:
//! bidirectional delay lines carrying forward-travelling pressure waves $p^+$
//! and backward-travelling pressure waves $p^-$.
//!
//! At any point along a pipe of cross-sectional area $A$, acoustic pressure $p$
//! and volume velocity $U$ satisfy:
//!
//! $$p = p^+ + p^-$$
//! $$U = Y (p^+ - p^-)$$
//!
//! where $Y = A / (\rho c)$ is the acoustic admittance of the duct, $\rho$ is the
//! fluid density, and $c = \sqrt{\gamma R T}$ is the local speed of sound.
//!
//! Scattering at discontinuous boundaries and multi-pipe junctions follows the
//! acoustic pressure junction condition:
//!
//! $$p_J = \frac{2 \sum_i Y_i p_i^+}{\sum_i Y_i}, \quad p_i^- = p_J - p_i^+$$
//!
//! Audio processing runs in `f32` at the stream sample rate without heap allocation
//! in the audio callback.

use crate::audio::filters::{runner_delay_seconds, speed_of_sound, DelayLine, OnePole, Smoothed};
use crate::audio::radiation::Mouth;

/// Coldest gas temperature used to size delay line buffers [K].
///
/// Sound travels slowest when cold ($c \propto \sqrt{T}$), which yields the
/// largest transit delay in samples. Sizing buffers for 250 K ensures the
/// delay lines never need reallocation during runtime temperature glides.
pub const COLDEST_EXHAUST_TEMPERATURE_K: f32 = 250.0;

/// Reference atmospheric pressure for gas density estimation [Pa].
pub const REFERENCE_PRESSURE_PA: f32 = 101_325.0;

/// Sutherland's reference dynamic viscosity for air, at [`SUTHERLAND_T0`] [Pa s].
pub const SUTHERLAND_MU0: f32 = 1.716e-5;

/// Sutherland's reference temperature [K].
pub const SUTHERLAND_T0: f32 = 273.15;

/// Sutherland's constant for air [K].
pub const SUTHERLAND_S: f32 = 110.4;

/// Kinematic viscosity of the gas in a duct at `temperature` [m^2 / s].
///
/// $$\nu = \frac{\mu(T)}{\rho(T)}, \qquad
///   \mu(T) = \mu_0 \left(\frac{T}{T_0}\right)^{3/2} \frac{T_0 + S}{T + S},
///   \qquad \rho = \frac{p}{R T}$$
///
/// Sutherland's law over the ideal gas. It is a strong function of temperature
/// and the reason this cannot be one number for the whole synth: hot exhaust
/// runs near `1.0e-4` and the cold air in an intake runner near `1.6e-5`, a
/// factor of six, and the boundary layer that sets the wall loss goes as its
/// square root. A single exhaust figure applied to an intake damps a cold
/// runner two and a half times too hard and drags its resonances flat with it —
/// which showed up, once the loss filter started attenuating the right amount,
/// as a plenum whose Helmholtz mode sat 8 % under the frequency its own
/// geometry predicts.
///
/// Sutherland is quoted for air and the burnt gas in an exhaust is not air, but
/// it is mostly nitrogen either way and the difference is small against the six
/// the temperature is worth.
#[inline]
pub fn kinematic_viscosity(temperature: f32, gas_constant: f32) -> f32 {
    let t = temperature.clamp(150.0, 2_500.0);
    let mu = SUTHERLAND_MU0 * (t / SUTHERLAND_T0).powf(1.5) * (SUTHERLAND_T0 + SUTHERLAND_S)
        / (t + SUTHERLAND_S);
    let rho = REFERENCE_PRESSURE_PA / (gas_constant.max(1.0) * t);
    mu / rho
}

/// Prandtl number of air and lean combustion gas [-].
pub const EXHAUST_PRANDTL_NUMBER: f32 = 0.71;

/// Effective turbulent boundary layer enhancement factor in corrugated/hot exhaust pipe.
///
/// A multiplier on the kinematic viscosity, so it is worth its square root in
/// attenuation: this is $\sqrt{2} \approx 1.4$ times the dissipation of a
/// smooth-walled tube carrying quiescent gas. What it stands for is everything a
/// header is that a laboratory tube is not — weld beads, mandrel bends, flex
/// joints, a perforated silencer core, and a strongly pulsating turbulent flow
/// that never lets a laminar boundary layer form.
///
/// # Where the number comes from
///
/// Not from taste: from what a pipe measures. Sound attenuation down a hot
/// automotive exhaust pipe of 55–65 mm bore runs about **0.2 to 0.6 dB per
/// metre** through 250 Hz–1 kHz, which is the band an exhaust note lives in and
/// the band a tailpipe is long enough to matter over. Against a 60 mm pipe at
/// 800 K [`ViscothermalLoss`] puts this constant at
///
/// ```text
/// factor    250 Hz   500 Hz  1000 Hz   [dB/m]
///    1.0     0.185    0.262    0.370
///    2.0     0.262    0.370    0.524
///    4.0     0.370    0.524    0.741
///   16.0     0.741    1.047    1.481
/// ```
///
/// so 2 sits in the upper half of the measured band — a real header, rougher
/// than plain Kirchhoff, and inside what anybody has ever measured off one.
/// Sixteen is three to four times the top of it, and a tailpipe losing 1.5 dB a
/// metre at 1 kHz strips an exhaust of everything above its firing orders. What
/// is left standing is two or three low peaks in an otherwise empty spectrum,
/// which is the sound of an engine heard through a wall, or from inside a can.
///
/// The value was not always wrong. It was chosen when [`ViscothermalLoss`]
/// delivered a fiftieth of the `alpha` it computed, and against that filter
/// sixteen came out near a third of Kirchhoff — quiet, but the right order.
/// Fixing the filter multiplied what this constant buys by about seven and left
/// the constant alone, and a calibration that outlives the bug it was
/// calibrated against is worse than no calibration at all.
pub const BOUNDARY_LAYER_TURBULENCE_FACTOR: f32 = 2.0;

/// Wall enhancement of a duct that is smooth, cold and not full of exhaust [-].
///
/// The classical viscothermal result, with nothing added to it. An intake tract
/// is cast or moulded, runs at ambient, and draws a steady column rather than
/// venting a jet, so it gets the textbook boundary layer and not the exhaust's.
pub const SMOOTH_WALL: f32 = 1.0;

// ---------------------------------------------------------------------------
// Viscothermal wall loss
// ---------------------------------------------------------------------------

/// Analytic viscothermal acoustic attenuation per metre [Np/m]:
///
/// $$\alpha(f) = \frac{1}{a c} \sqrt{\pi f \nu} \left(1 + \frac{\gamma - 1}{\sqrt{Pr}}\right)$$
#[inline]
pub fn viscothermal_alpha(
    frequency: f64,
    radius: f64,
    speed_of_sound: f64,
    kinematic_viscosity: f64,
    gamma: f64,
    prandtl: f64,
) -> f64 {
    let num = (std::f64::consts::PI * frequency.max(0.0) * kinematic_viscosity.max(1e-9)).sqrt();
    let thermal = 1.0 + (gamma - 1.0) / prandtl.max(0.1).sqrt();
    (1.0 / (radius.max(1e-4) * speed_of_sound.max(1.0))) * num * thermal
}

/// Attenuation of a pipe segment per pass, in decibels per root hertz [dB/Hz^0.5].
///
/// The whole of $\alpha(f) L$ with the frequency taken out of it, so that the
/// loss across one pass is simply
///
/// $$A(f) = \texttt{loss\_slope} \cdot \sqrt{f} \quad [\text{dB}]$$
///
/// Quoting it this way is what makes the curve fittable: the shape never
/// changes, only the height, so one filter design serves every pipe in the
/// network and a pipe's own geometry enters as a single number.
#[inline]
pub fn loss_slope_db(
    radius: f32,
    length: f32,
    speed_of_sound: f32,
    gamma: f32,
    gas_constant: f32,
    temperature: f32,
    wall_enhancement: f32,
) -> f32 {
    let nu_eff = kinematic_viscosity(temperature, gas_constant) * wall_enhancement.max(1.0);
    let thermal = 1.0 + (gamma - 1.0) / EXHAUST_PRANDTL_NUMBER.sqrt();
    let alpha_per_root_hz = (1.0 / (radius.max(1e-4) * speed_of_sound.max(1.0)))
        * (std::f32::consts::PI * nu_eff).sqrt()
        * thermal;
    // Nepers to decibels: 20 / ln(10).
    alpha_per_root_hz * length.max(0.001) * 8.685_889
}

/// Frequencies the loss curve is matched at [Hz].
///
/// Not corners chosen for how they sound: they are the grid the analytic
/// $\sqrt{f}$ law is sampled on, spaced about a decade apart so that three
/// first-order sections between them cover 20 Hz to 20 kHz evenly. The gain of
/// each section is read off [`loss_slope_db`] at the geometric centre of the
/// band it owns, so every number the filter ends up holding came from the
/// pipe's radius, length and gas.
pub const LOSS_MATCH_HZ: [f32; 3] = [125.0, 1_000.0, 8_000.0];

/// Top of the band the highest section is matched over [Hz].
///
/// Above it the plane-wave model has already run out — a 60 mm duct cuts its
/// first cross mode on near 5 kHz — so there is nothing to be gained by fitting
/// further up, and a great deal to be lost by letting the last section's
/// asymptote set the level of everything between 16 kHz and Nyquist.
pub const LOSS_MATCH_TOP_HZ: f32 = 64_000.0;

/// First-order shelf: unity at DC, `gain` above its corner.
///
/// `y = g x + (1 - g) \operatorname{lp}(x)`. One multiply more than the lowpass
/// it is built from, and unlike a lowpass it can sit at a value other than zero
/// or one in the middle of the band — which is the entire reason three of them
/// can trace a curve a single lowpass cannot.
#[derive(Debug, Clone, Copy, Default)]
struct LossShelf {
    lowpass: OnePole,
    gain: f32,
}

impl LossShelf {
    fn new(sample_rate: f32, corner_hz: f32, gain: f32) -> Self {
        Self {
            lowpass: OnePole::new(sample_rate, corner_hz),
            gain: gain.clamp(0.0, 1.0),
        }
    }

    fn tune(&mut self, sample_rate: f32, corner_hz: f32, gain: f32) {
        self.lowpass.set_cutoff(sample_rate, corner_hz);
        self.gain = gain.clamp(0.0, 1.0);
    }

    #[inline(always)]
    fn process(&mut self, x: f32) -> f32 {
        self.gain * x + (1.0 - self.gain) * self.lowpass.process(x)
    }

    /// Delay the shelf adds to a wave at normalised frequency `omega` [samples].
    ///
    /// *Phase* delay, `-phi(w) / w`, and not the group delay taken at DC that a
    /// one-pole is usually asked for. The two agree only well below a filter's
    /// corner, and these corners sit inside the band on purpose — the whole
    /// point of the cascade is that the curve bends where the engine is. What
    /// sets where a pipe resonates is the phase its loop comes back with at the
    /// frequency in question, so that is the number to hand the delay line; a
    /// shelf at 125 Hz lags a 170 Hz wave by 1.6 samples and its DC group delay
    /// says 4.3, and a pipe retuned on the second figure plays 5 % sharp.
    ///
    /// The shelf is `(b0 + b1 z^-1) / (1 - p z^-1)` with `p = 1 - a`,
    /// `b0 = g + (1 - g) a` and `b1 = -g p`.
    fn phase_delay_samples(&self, omega: f32) -> f32 {
        let a = self.lowpass.coefficient();
        let pole = 1.0 - a;
        let g = self.gain;
        let (b0, b1) = (g + (1.0 - g) * a, -g * pole);
        let (sin_w, cos_w) = omega.sin_cos();
        let num = (-b1 * sin_w).atan2(b0 + b1 * cos_w);
        let den = (pole * sin_w).atan2(1.0 - pole * cos_w);
        (den - num) / omega.max(1e-6)
    }

    fn reset(&mut self) {
        self.lowpass.reset();
    }
}

/// Viscothermal boundary layer loss down one pass of a pipe.
///
/// # Why this is not one lowpass
///
/// The loss a wall puts on a wave grows as the square root of frequency, which
/// in decibels is `A(f) = k sqrt(f)`: gentle, unbounded, and never flat. A
/// first-order lowpass is the opposite shape — flat to its corner and then 6 dB
/// per octave forever — and fitting one to the curve at its own $-3$ dB point,
/// which is what this used to do, puts the corner somewhere near 8 kHz and
/// leaves the whole audible band below it attenuated by essentially nothing. A
/// 1.5 m tailpipe should take 1.8 dB out of a 500 Hz wave on each pass; the
/// lowpass took 0.6. Twenty round trips later that is a 25 dB error, and it is
/// audible as exactly what it is — a pipe that will not stop ringing, over a
/// top end that the same filter's 6 dB/octave has meanwhile buried.
///
/// Three first-order shelves matched on the grid at [`LOSS_MATCH_HZ`] hold the
/// curve to a fifth of a decibel on a primary and to about one on a tailpipe,
/// across 30 Hz to 16 kHz, for three multiplies and three adds.
#[derive(Debug, Clone, Copy)]
pub struct ViscothermalLoss {
    shelves: [LossShelf; LOSS_MATCH_HZ.len()],
    slope_db: f32,
    /// Multiplier on the gas viscosity for what the wall is actually like [-].
    wall_enhancement: f32,
}

impl ViscothermalLoss {
    /// Creates a loss filter for a pipe of radius $a$ and length $L$.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        radius: f32,
        length: f32,
        speed_of_sound: f32,
        gamma: f32,
        gas_constant: f32,
        temperature: f32,
        sample_rate: f32,
    ) -> Self {
        let mut loss = Self {
            shelves: [LossShelf::default(); LOSS_MATCH_HZ.len()],
            slope_db: 0.0,
            wall_enhancement: BOUNDARY_LAYER_TURBULENCE_FACTOR,
        };
        for (k, &corner) in LOSS_MATCH_HZ.iter().enumerate() {
            loss.shelves[k] = LossShelf::new(sample_rate, corner, 1.0);
        }
        loss.tune(
            radius,
            length,
            speed_of_sound,
            gamma,
            gas_constant,
            temperature,
            sample_rate,
        );
        loss
    }

    /// Retunes the whole cascade for current sound speed and specific heat ratio.
    ///
    /// Each shelf is handed whatever is left of the target once the shelves
    /// below it have taken their share, evaluated at the geometric centre of
    /// the band it owns. That makes the cascade exact at three points by
    /// construction and close everywhere between them.
    #[allow(clippy::too_many_arguments)]
    pub fn tune(
        &mut self,
        radius: f32,
        length: f32,
        speed_of_sound: f32,
        gamma: f32,
        gas_constant: f32,
        temperature: f32,
        sample_rate: f32,
    ) {
        self.slope_db = loss_slope_db(
            radius,
            length,
            speed_of_sound,
            gamma,
            gas_constant,
            temperature,
            self.wall_enhancement,
        );
        let mut placed_db = 0.0f32;
        for (k, (&corner, shelf)) in LOSS_MATCH_HZ
            .iter()
            .zip(self.shelves.iter_mut())
            .enumerate()
        {
            let upper = LOSS_MATCH_HZ
                .get(k + 1)
                .copied()
                .unwrap_or(LOSS_MATCH_TOP_HZ);
            let centre = (corner * upper).sqrt();
            let want_db = -self.slope_db * centre.sqrt();
            let gain = 10f32.powf(((want_db - placed_db) / 20.0).max(-6.0));
            placed_db += 20.0 * gain.log10();
            shelf.tune(sample_rate, corner, gain);
        }
    }

    /// Attenuation this pass puts on a wave of frequency `f` [dB].
    ///
    /// The analytic target the cascade is fitted to, not a readback of the
    /// filter: the thing a test compares the filter against.
    pub fn attenuation_db(&self, frequency: f32) -> f32 {
        self.slope_db * frequency.max(0.0).sqrt()
    }

    /// Attenuation per root hertz [dB/Hz^0.5].
    pub fn slope_db(&self) -> f32 {
        self.slope_db
    }

    /// Declares what the wall is like, as a multiplier on the gas viscosity.
    ///
    /// Defaults to [`BOUNDARY_LAYER_TURBULENCE_FACTOR`], the exhaust's. Retune
    /// afterwards for it to take effect.
    pub fn set_wall_enhancement(&mut self, factor: f32) {
        self.wall_enhancement = factor.max(1.0);
    }

    /// Filters one travelling sample.
    #[inline(always)]
    pub fn process(&mut self, x: f32) -> f32 {
        let mut y = x;
        for shelf in &mut self.shelves {
            y = shelf.process(y);
        }
        y
    }

    /// Delay the loss cascade adds to a wave of frequency `f` [samples].
    ///
    /// Ask for it at the frequency that matters — a pipe's own fundamental —
    /// and not at DC; see [`LossShelf::phase_delay_samples`].
    pub fn phase_delay_samples(&self, frequency_hz: f32, sample_rate: f32) -> f32 {
        let omega =
            (std::f32::consts::TAU * frequency_hz.max(0.1) / sample_rate.max(1.0)).clamp(1e-5, 3.0);
        self.shelves
            .iter()
            .map(|shelf| shelf.phase_delay_samples(omega))
            .sum()
    }

    /// Clears internal state.
    pub fn reset(&mut self) {
        for shelf in &mut self.shelves {
            shelf.reset();
        }
    }
}

// ---------------------------------------------------------------------------
// Valve-end termination
// ---------------------------------------------------------------------------

/// Acoustic boundary condition at the valve end of a runner: the cylinder.
///
/// A runner is terminated at one end by a junction and at the other by a valve,
/// and what that valve is a boundary *to* is the cylinder. Off its seat it is
/// not a hole into free space, it is a short constriction opening into a closed
/// box with a piston for one wall, and the three things that box and that gap
/// do to an arriving wave are the whole of this type.
///
/// **The gap has mass.** Gas has to be accelerated through the curtain between
/// the valve and its seat, and an aperture of effective area $A_v$ carries an
/// inertance
///
/// ```text
/// M = rho * L_eff / A_v,     L_eff = 2 * FLANGED_END_CORRECTION * sqrt(A_v / pi)
/// ```
///
/// the end correction counted twice because an aperture in a wall loads air on
/// both of its faces. It has no length of its own worth the name; all of it is
/// end correction.
///
/// **The cylinder has stiffness.** A box of volume $V$ that is short against
/// the wavelength does not propagate, it compresses, with acoustic compliance
///
/// ```text
/// C = V / (rho c^2)
/// ```
///
/// **The gap resists.** Gas crossing the curtain at mean velocity $v$ costs
/// $\tfrac{1}{2} \rho v^2$ of head, and differentiating that against volume
/// flow leaves an acoustic resistance
///
/// ```text
/// R = rho v / A_v = mdot / A_v^2
/// ```
///
/// which the density drops straight out of: the port's own mass flow and the
/// area it is crossing are enough.
///
/// In series those are a Helmholtz resonator hung on the end of the pipe,
///
/// ```text
///                   Z_L(s) - Z_p                          1
/// r(s) = ---------------------------- ,   Z_L(s) = R + sM + ---- ,   Z_p = rho c / A_p
///                   Z_L(s) + Z_p                           sC
/// ```
///
/// and multiplying through by $sC$ puts it in the form a biquad runs:
///
/// ```text
///         s^2 MC + sC (R - Z_p) + 1
/// r(s) = ---------------------------
///         s^2 MC + sC (R + Z_p) + 1
/// ```
///
/// # What that buys, and what the area ratio cost
///
/// Read the limits off it. At DC the compliance is infinitely stiff, the
/// numerator and denominator are both 1, and $r = +1$: **a long wave sees a
/// rigid end however far the valve is lifted**, because a box it cannot
/// compress is a wall. At Nyquist the inertance blocks, and $r \to +1$ again.
/// In between, at
///
/// ```text
/// f_H = 1 / (2 pi sqrt(MC))
/// ```
///
/// the reactances cancel and $r = (R - Z_p) / (R + Z_p)$, which for a gap
/// resisting anything like the pipe's own impedance is somewhere near zero —
/// the load swallows what lands on it. So the cylinder is an absorber with a
/// *notch*, not a broadband sink, and the notch moves: $V$ shrinks by an order
/// of magnitude as the piston comes up the bore under an open exhaust valve, so
/// $f_H$ sweeps upward through the midrange once every cycle and no standing
/// pattern survives it.
///
/// Shutting the valve needs no special case. $A_v \to 0$ sends $M \to \infty$,
/// which sends both quadratics to $s^2 MC$ and $r$ to $+1$ at every frequency:
/// the boundary goes rigid on its own, the way the metal does.
///
/// The thing this replaces was the two-pipe step for the two areas,
/// $r = (A_p - A_v)/(A_p + A_v)$, a real number applied flat across the band.
/// That says an open valve leads to a pipe of area $A_v$ running away forever,
/// and it is wrong in the one place it matters most: it throws away 40% of
/// every wave *at 20 Hz*, where a real cylinder returns essentially all of it.
/// A third of every cycle spent bleeding the bottom of the band out of the
/// runners is an engine with no rumble left in it — and, because the loss was
/// flat, it took no more out of the midrange than out of the bottom, so what
/// survived was the hollow two-tone of a tube with nothing damping its middle.
///
/// Blowdown is injected into the forward path here, ahead of the reflection:
///
/// ```text
/// p+ = p_excitation + r{p-}
/// ```
#[derive(Debug, Clone, Copy)]
pub struct ValveTermination {
    pipe_area: f32,
    sample_rate: f32,
    /// $\rho c / A_p$, the pipe's own characteristic impedance [Pa s/m^3].
    pipe_impedance: f32,
    density: f32,
    speed_of_sound: f32,
    /// The load as last set, kept so [`Self::reflection_at`] can report the
    /// curve the biquad is actually running rather than an idealisation of it.
    resistance: f32,
    inertance: f32,
    compliance: f32,
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

/// Effective area below which the valve counts as seated [m^2].
///
/// A square micrometre. Under it the inertance of the gap is large enough that
/// the boundary is rigid to well past Nyquist anyway, and the divide by $A_v^2$
/// in the resistance stops being worth doing in single precision.
const SEATED_AREA: f32 = 1e-9;

impl ValveTermination {
    /// A valve at the head of a pipe of area $A_p$ [m^2], seated, in air.
    ///
    /// Call [`Self::tune`] before use: the gas the runner is carrying decides
    /// the impedance every term here is measured against.
    pub fn new(sample_rate: f32, pipe_area: f64) -> Self {
        let mut valve = Self {
            pipe_area: pipe_area.max(1e-7) as f32,
            sample_rate: sample_rate.max(1.0),
            pipe_impedance: 1.0,
            density: 1.2,
            speed_of_sound: 343.0,
            resistance: 0.0,
            inertance: 0.0,
            compliance: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
            a1: 0.0,
            a2: 0.0,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        };
        valve.tune(1.4, 287.0, 293.0);
        valve
    }

    /// Retunes to the gas in the pipe: `gamma` [-], `gas_constant` [J/(kg K)]
    /// and `temperature` [K].
    ///
    /// Density is taken at [`REFERENCE_PRESSURE_PA`] like every other mean-flow
    /// quantity in the network, so a hot runner is a light one.
    pub fn tune(&mut self, gamma: f32, gas_constant: f32, temperature: f32) {
        let t = temperature.max(1.0);
        self.speed_of_sound = speed_of_sound(gamma, gas_constant, t);
        self.density = REFERENCE_PRESSURE_PA / (gas_constant.max(1.0) * t);
        self.pipe_impedance = self.density * self.speed_of_sound / self.pipe_area;
    }

    /// Sets the cylinder behind the valve this sample:
    /// - `effective_area`: `C_d` times the curtain area at the seat [m^2].
    /// - `cylinder_volume`: the space enclosed above the piston [m^3].
    /// - `port_mass_flow`: gas crossing the valve, either direction [kg/s].
    pub fn set_load(&mut self, effective_area: f64, cylinder_volume: f64, port_mass_flow: f64) {
        let av = (effective_area.max(0.0) as f32).min(1.0);
        if av <= SEATED_AREA {
            self.resistance = 0.0;
            self.inertance = f32::INFINITY;
            self.compliance = 0.0;
            self.set_rigid();
            return;
        }
        let radius = (av / std::f32::consts::PI).sqrt();
        let length = 2.0 * crate::audio::radiation::FLANGED_END_CORRECTION as f32 * radius;
        self.inertance = self.density * length / av;
        let volume = (cylinder_volume as f32).max(1e-9);
        self.compliance =
            volume / (self.density * self.speed_of_sound * self.speed_of_sound).max(1e-9);
        self.resistance = (port_mass_flow.abs() as f32) / (av * av);

        // Bilinear, unwarped. The transform is exact at DC, which is the limit
        // this boundary has to hold — a rigid end at the bottom of the band is
        // the whole point — and `f_H` sits in the midrange at any sample rate
        // worth running, far enough below Nyquist that the warping there is
        // under a percent.
        let k = 2.0 * self.sample_rate;
        let k2 = k * k;
        let mc = self.inertance * self.compliance;
        let quad = mc * k2;
        let lead = self.compliance * k;
        let minus = lead * (self.resistance - self.pipe_impedance);
        let plus = lead * (self.resistance + self.pipe_impedance);
        let d0 = quad + plus + 1.0;
        if !d0.is_finite() || d0.abs() < 1e-20 {
            self.set_rigid();
            return;
        }
        let inv = 1.0 / d0;
        self.b0 = (quad + minus + 1.0) * inv;
        self.b1 = (2.0 - 2.0 * quad) * inv;
        self.b2 = (quad - minus + 1.0) * inv;
        self.a1 = (2.0 - 2.0 * quad) * inv;
        self.a2 = (quad - plus + 1.0) * inv;
        if !(self.b0.is_finite()
            && self.b1.is_finite()
            && self.b2.is_finite()
            && self.a1.is_finite()
            && self.a2.is_finite())
        {
            self.set_rigid();
        }
    }

    /// Puts the boundary back to the seated case: unity at every frequency.
    #[inline]
    fn set_rigid(&mut self) {
        self.b0 = 1.0;
        self.b1 = 0.0;
        self.b2 = 0.0;
        self.a1 = 0.0;
        self.a2 = 0.0;
    }

    /// Magnitude of the reflection this load presents at `hz` [-].
    ///
    /// Evaluated on the analogue prototype rather than the discrete filter, so
    /// it reads the same at any sample rate. Unity at DC by construction.
    pub fn reflection_at(&self, hz: f32) -> f32 {
        if !self.inertance.is_finite() {
            return 1.0;
        }
        let w = std::f32::consts::TAU * hz.max(0.0);
        let real = 1.0 - w * w * self.inertance * self.compliance;
        let num_imag = w * self.compliance * (self.resistance - self.pipe_impedance);
        let den_imag = w * self.compliance * (self.resistance + self.pipe_impedance);
        let num = (real * real + num_imag * num_imag).sqrt();
        let den = (real * real + den_imag * den_imag).sqrt();
        if den < 1e-30 {
            1.0
        } else {
            (num / den).min(1.0)
        }
    }

    /// Frequency at which the gap's mass and the cylinder's stiffness cancel [Hz].
    ///
    /// Infinite while the valve is seated, which is the honest answer: there is
    /// no resonance because there is no aperture.
    pub fn helmholtz_hz(&self) -> f32 {
        let mc = self.inertance * self.compliance;
        if !mc.is_finite() || mc <= 0.0 {
            return f32::INFINITY;
        }
        1.0 / (std::f32::consts::TAU * mc.sqrt())
    }

    /// Clears the filter's memory and reseats the valve.
    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.y1 = 0.0;
        self.y2 = 0.0;
        self.set_load(0.0, 1e-4, 0.0);
    }

    /// Computes the forward-travelling wave entering the runner:
    /// - `excitation`: blowdown pressure pulse injected at the port [Pa].
    /// - `returning_wave`: backward wave arriving at the valve boundary ($p^-(0)$) [Pa].
    #[inline(always)]
    pub fn step(&mut self, excitation: f32, returning_wave: f32) -> f32 {
        // Direct form I, because these coefficients move every sample.
        //
        // A transposed form stores partial sums whose meaning is a function of
        // the coefficients that made them, so changing a coefficient
        // reinterprets the filter's memory as well as its response — and a
        // reflection sits inside a feedback loop, where the energy that
        // reinterpretation invents comes back round and is reinterpreted again.
        // Measured on the intake tract at a shut throttle, where nothing else
        // dissipates: the transposed form grew about a decibel a second with no
        // excitation at all. Direct form I stores past inputs and past outputs,
        // which are pressures either way and mean the same thing whatever the
        // cam is doing.
        let x = returning_wave;
        let y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        excitation + y
    }
}

/// Acoustic efficiency of the dipole a jet makes at an edge [-].
///
/// Curle's sixth-power law says a flow past a solid boundary radiates
/// $W \propto \rho A u^6 / c^3$, and leaves the constant to measurement; in
/// pressure that constant runs from about 0.03 to 0.1 for flow through an
/// orifice. This is the one number in the exhaust path with a range rather than
/// a derivation, and it is recorded as such — see the open questions in
/// `docs/measurements/calibration.md`. What it is *not* is the near-field rule
/// of thumb: the hydrodynamic pressure in a mixing layer is a tenth of the
/// dynamic head, but that pressure does not propagate, and using it as though
/// it did buries the firings under a hiss.
pub const JET_DIPOLE_EFFICIENCY: f32 = 0.005;

/// Strouhal number of a free jet's preferred mode [-]./// Strouhal number of a free jet's preferred mode [-].
///
/// $St = f d / u$. A jet sheds its large structures at about a fifth, so the
/// noise a port makes is pitched by how fast the gas is going and how wide the
/// gap it is going through — which is why a valve cracking open hisses high and
/// a valve at full lift roars low, with no table anywhere saying so.
pub const JET_STROUHAL_NUMBER: f32 = 0.20;

/// Gas velocity through a valve's effective flow area [m/s].
///
/// $u = \dot m / \rho A_e$, limited to the local speed of sound because a
/// throat chokes: past a pressure ratio of about 1.9 the gap passes no more
/// velocity however much harder it is pushed, and the extra mass flow arrives
/// as density instead.
#[inline]
pub fn throat_velocity(
    mass_flow: f32,
    effective_area: f32,
    density: f32,
    speed_of_sound: f32,
) -> f32 {
    if effective_area <= SEATED_AREA || density <= 0.0 {
        return 0.0;
    }
    (mass_flow.abs() / (density * effective_area)).min(speed_of_sound)
}

/// Broadband pressure a port's own jet launches into the runner [Pa].
///
/// Curle's dipole, written for a wave in a duct. Radiated power
/// $W = K \rho A u^6 / c^3$ into a duct carrying $p^2 A / \rho c$ gives
///
/// $$p = \sqrt{K} \, \frac{\rho u^3}{c} = \sqrt{K} \, \rho c u M^2,$$
///
/// a *cube* of velocity and not a square, which is why this is a chuff at each
/// blowdown rather than a hiss across the cycle: at half the velocity it is
/// eighteen decibels down, not twelve. It is the exhaust's half of what the
/// intake already gets from its throttle plate and its valves — gas tearing
/// itself apart on the way past an edge. Without it a pipe radiates a comb with
/// nothing at all between the teeth, which is the one thing no recording of a
/// real engine has ever looked like.
#[inline]
pub fn jet_pressure_fluctuation_pa(
    velocity: f32,
    density: f32,
    speed_of_sound: f32,
    effective_area: f32,
    pipe_area: f32,
) -> f32 {
    let c = speed_of_sound.max(1.0);
    // The source is the gap, not the pipe: the $A$ in the power law is the area
    // the jet actually occupies, and a duct that carries the result away is
    // wider than that, so what reaches the runner as a plane wave is down by
    // the square root of the ratio. A valve barely off its seat couples badly
    // as well as flowing little, which is most of why this is a crack at the
    // valve event and not a wash under the whole cycle.
    let coupling = (effective_area.max(0.0) / pipe_area.max(1e-9))
        .min(1.0)
        .sqrt();
    JET_DIPOLE_EFFICIENCY * coupling * density * velocity * velocity * velocity / c
}

/// Frequency a jet through an aperture of `effective_area` is loudest at [Hz].
///
/// [`JET_STROUHAL_NUMBER`] times $u / d$, with $d$ the diameter of a circle of
/// the same area. A valve just off its seat is a slit and screams; the same
/// valve at full lift is a hole and roars.
#[inline]
pub fn jet_peak_hz(velocity: f32, effective_area: f32) -> f32 {
    if effective_area <= SEATED_AREA {
        return 0.0;
    }
    let diameter = 2.0 * (effective_area / std::f32::consts::PI).sqrt();
    JET_STROUHAL_NUMBER * velocity / diameter
}

/// Coefficient of nonlinearity for a simple wave in an ideal gas [-].
///
/// $$\frac{\gamma + 1}{2\gamma}$$
///
/// A point of the waveform at overpressure $p$ travels at $c_0$ times
/// $1 + \text{this} \cdot p / P_0$: the sum of the flow it induces
/// ($u = p / \rho_0 c_0$) and the rise in sound speed through gas it has
/// compressed ($c = c_0 + \frac{\gamma - 1}{2} u$). About 0.87 in exhaust gas.
#[inline]
pub fn nonlinearity(gamma: f32) -> f32 {
    (gamma.max(1.01) + 1.0) / (2.0 * gamma.max(1.01))
}

// ---------------------------------------------------------------------------
// Waveguide pipe
// ---------------------------------------------------------------------------

/// A bidirectional cylindrical acoustic pipe section.
///
/// Models a duct of physical length $L$ and cross-sectional area $A$.
/// Carries two travelling wave components:
/// - `forward_line`: $p^+$, waves entering at port 0 and propagating toward port 1.
/// - `backward_line`: $p^-$, waves entering at port 1 and propagating toward port 0.
#[derive(Debug, Clone)]
pub struct WaveguidePipe {
    length: f32,
    area: f32,
    /// Delay contributed by filters at the pipe's boundaries, per round trip
    /// [samples]; see [`WaveguidePipe::set_boundary_phase_delay`].
    boundary_phase_delay: f32,
    forward_line: DelayLine,
    backward_line: DelayLine,
    forward_loss: ViscothermalLoss,
    backward_loss: ViscothermalLoss,
    forward_delay_samples: Smoothed,
    backward_delay_samples: Smoothed,
    admittance: f32,
    sample_rate: f32,
    mach: f32,
    gamma: f32,
    gas_constant: f32,
    temperature: f32,
    /// Whether this pipe propagates on the gas dynamics rather than on linear
    /// acoustics; see [`WaveguidePipe::set_steepening`].
    steepening: bool,
    /// $\frac{\gamma+1}{2\gamma}$, the coefficient of nonlinearity for a
    /// simple wave in this gas [-]. Cached because it is wanted every sample
    /// and `gamma` moves at control rate.
    nonlinearity: f32,
    /// Last pressure this pipe's downstream end emitted [Pa].
    ///
    /// The state a front is running into, which is half of what decides how
    /// fast that front travels; see [`WaveguidePipe::shocked_read`].
    emitted: f32,
}

impl WaveguidePipe {
    /// Constructs a new bidirectional pipe section.
    ///
    /// Allocates delay lines sized for the coldest possible exhaust gas ([`COLDEST_EXHAUST_TEMPERATURE_K`]),
    /// so the pipe never allocates or reallocates during subsequent [`WaveguidePipe::tune`] calls.
    pub fn new(
        length: f64,
        area: f64,
        sample_rate: f32,
        gamma: f32,
        gas_constant: f32,
        temperature: f32,
    ) -> Self {
        let length_f32 = length.max(0.001) as f32;
        let area_f32 = area.max(1e-7) as f32;
        let radius_f32 = (area_f32 / std::f32::consts::PI).sqrt();

        let max_delay_sec = runner_delay_seconds(
            length_f32,
            gamma,
            gas_constant,
            COLDEST_EXHAUST_TEMPERATURE_K,
        );
        // Sized with 2x headroom for mean-flow convective bias up to M = 0.5.
        let max_delay_samples = ((max_delay_sec / 0.5) * sample_rate).ceil() as usize + 32;

        let initial_delay_sec = runner_delay_seconds(length_f32, gamma, gas_constant, temperature);

        let c = speed_of_sound(gamma, gas_constant, temperature);
        let rho = REFERENCE_PRESSURE_PA / (gas_constant * temperature.max(1.0));
        let admittance = area_f32 / (rho * c).max(1e-4);

        let forward_loss = ViscothermalLoss::new(
            radius_f32,
            length_f32,
            c,
            gamma,
            gas_constant,
            temperature,
            sample_rate,
        );
        let backward_loss = ViscothermalLoss::new(
            radius_f32,
            length_f32,
            c,
            gamma,
            gas_constant,
            temperature,
            sample_rate,
        );
        let quarter_wave_hz = c / (4.0 * length_f32);
        let initial_delay_samples = (initial_delay_sec * sample_rate
            - forward_loss.phase_delay_samples(quarter_wave_hz, sample_rate))
        .max(1.0);

        Self {
            length: length_f32,
            area: area_f32,
            boundary_phase_delay: 0.0,
            forward_line: DelayLine::with_max_delay(max_delay_samples),
            backward_line: DelayLine::with_max_delay(max_delay_samples),
            forward_loss,
            backward_loss,
            // 40 ms time constant matches the engine temperature glide: fast enough
            // to follow throttle snaps, slow enough that fractional interpolation
            // never produces audible doppler pitch clicks.
            forward_delay_samples: Smoothed::new(initial_delay_samples, sample_rate, 0.040),
            backward_delay_samples: Smoothed::new(initial_delay_samples, sample_rate, 0.040),
            admittance,
            sample_rate,
            mach: 0.0,
            gamma,
            gas_constant,
            temperature,
            steepening: false,
            nonlinearity: nonlinearity(gamma),
            emitted: 0.0,
        }
    }

    /// Declares what this pipe's wall is like, as a multiplier on the gas
    /// viscosity — [`SMOOTH_WALL`] for a cast intake tract,
    /// [`BOUNDARY_LAYER_TURBULENCE_FACTOR`] (the default) for a header.
    pub fn set_wall_enhancement(&mut self, factor: f32) {
        self.forward_loss.set_wall_enhancement(factor);
        self.backward_loss.set_wall_enhancement(factor);
        self.tune(self.gamma, self.gas_constant, self.temperature);
    }

    /// Declares that this pipe carries pulses large enough to steepen.
    ///
    /// A delay line moves every part of a wave at the same speed. A gas does
    /// not: a point of the waveform at overpressure $p$ rides on the flow it
    /// has itself induced and through gas it has itself heated, so it travels
    /// at $c_0(1 + \beta)$ with
    ///
    /// $$\beta = \frac{\gamma + 1}{2\gamma} \frac{p}{P_0}$$
    ///
    /// which is the simple-wave result and carries no constant of its own. The
    /// crest gains on the trough ahead of it, the front stands up, and once the
    /// crest has caught that trough the front is a shock. That is the whole
    /// difference between a pulse that is loud and one that is *hard*.
    ///
    /// It is a switch and not a depth because there is nothing to set: the gas
    /// decides how much a given pulse steepens. What the switch buys is the
    /// search in [`WaveguidePipe::read_outputs`], and it is worth its cost only
    /// where the pressure ratio actually approaches the shock condition — the
    /// primaries, at blowdown. Downstream of the collector the same pulse has
    /// spread over the junction's volume and travels as linear acoustics.
    pub fn set_steepening(&mut self, on: bool) {
        self.steepening = on;
    }

    /// Whether this pipe propagates on the gas dynamics.
    pub fn steepening(&self) -> bool {
        self.steepening
    }

    /// Retunes propagation delay, acoustic admittance, and wall losses for current gas state.
    pub fn tune(&mut self, gamma: f32, gas_constant: f32, temperature: f32) {
        self.gamma = gamma;
        self.gas_constant = gas_constant;
        self.temperature = temperature;
        self.nonlinearity = nonlinearity(gamma);

        let c = speed_of_sound(gamma, gas_constant, temperature);
        let rho = REFERENCE_PRESSURE_PA / (gas_constant * temperature.max(1.0));
        self.admittance = self.area / (rho * c).max(1e-4);

        let r = self.radius();
        self.forward_loss.tune(
            r,
            self.length,
            c,
            gamma,
            gas_constant,
            temperature,
            self.sample_rate,
        );
        self.backward_loss.tune(
            r,
            self.length,
            c,
            gamma,
            gas_constant,
            temperature,
            self.sample_rate,
        );

        self.update_delay_targets();
    }

    /// Updates forward and backward delay targets accounting for mean flow Mach number.
    ///
    /// Acoustic waves run downstream at `c + u = c(1 + M)` and upstream at
    /// `c - u = c(1 - M)`. Effective tuning lengths become asymmetric and
    /// load-dependent, shifting fundamental resonance by `1 - M^2`.
    fn update_delay_targets(&mut self) {
        let delay_sec =
            runner_delay_seconds(self.length, self.gamma, self.gas_constant, self.temperature);
        let comp = self.compensation();
        let fwd_sec = delay_sec / (1.0 + self.mach).max(0.05);
        let bwd_sec = delay_sec / (1.0 - self.mach).max(0.05);
        let fwd_samples = fwd_sec * self.sample_rate - comp;
        let bwd_samples = bwd_sec * self.sample_rate - comp;
        self.forward_delay_samples
            .set_target(fwd_samples.clamp(1.0, self.forward_line.max_delay()));
        self.backward_delay_samples
            .set_target(bwd_samples.clamp(1.0, self.backward_line.max_delay()));
    }

    /// Sets the mean-flow Mach number along the pipe: positive downstream (0 -> 1).
    pub fn set_mach(&mut self, mach: f32) {
        self.mach = mach.clamp(-MAX_MEAN_FLOW_MACH, MAX_MEAN_FLOW_MACH);
        self.update_delay_targets();
    }

    /// Current mean-flow Mach number [-].
    pub fn mach(&self) -> f32 {
        self.mach
    }

    /// Immediately snaps forward and backward delays to their current targets.
    pub fn snap_delays(&mut self) {
        self.forward_delay_samples
            .snap(self.forward_delay_samples.target());
        self.backward_delay_samples
            .snap(self.backward_delay_samples.target());
    }

    /// Filter delay to take out of each direction of travel [samples].
    ///
    /// One loss filter sits in each direction, so each pays for its own; a
    /// boundary filter is met once per round trip, so each direction pays half.
    #[inline]
    fn compensation(&self) -> f32 {
        self.forward_loss
            .phase_delay_samples(self.quarter_wave_hz(), self.sample_rate)
            + 0.5 * self.boundary_phase_delay
    }

    /// The pipe's own quarter-wave fundamental, `c / 4L` [Hz].
    ///
    /// Where the loss cascade's phase delay is read off, because it is the
    /// resonance a length is heard as and the one every analytic test in this
    /// module is written against. A section buried mid-chain has no such mode
    /// of its own, but it is short, its cascade is nearly transparent, and the
    /// few tenths of a sample the choice moves it by are below what a
    /// fractional delay resolves.
    fn quarter_wave_hz(&self) -> f32 {
        speed_of_sound(self.gamma, self.gas_constant, self.temperature) / (4.0 * self.length)
    }

    /// Declares the delay that filters at this pipe's boundaries add, per round
    /// trip [samples] — a [`Mouth`]'s reflection filter, typically.
    ///
    /// Without this the pipe is longer than its geometry says by however much
    /// phase the terminations happen to carry, and every resonance built on it
    /// sits flat. Retune afterwards, or on the next [`WaveguidePipe::tune`].
    pub fn set_boundary_phase_delay(&mut self, samples: f32) {
        self.boundary_phase_delay = samples.max(0.0);
    }

    /// Length of the pipe section [m].
    pub fn length(&self) -> f32 {
        self.length
    }

    /// Cross-sectional area of the pipe section [m^2].
    pub fn area(&self) -> f32 {
        self.area
    }

    /// Internal bore radius of the pipe section [m].
    pub fn radius(&self) -> f32 {
        (self.area / std::f32::consts::PI).sqrt()
    }

    /// Acoustic admittance $Y = A / (\rho c)$ [m^3 / (Pa s)].
    pub fn admittance(&self) -> f32 {
        self.admittance
    }

    /// Current one-way acoustic transit delay $L / c$ [samples].
    ///
    /// This is the physical transit, not the length of the delay line: the line
    /// is deliberately shorter by whatever the loss and boundary filters
    /// contribute, and that bookkeeping is nobody else's business.
    pub fn delay_samples(&self) -> f32 {
        0.5 * (self.forward_delay_samples.value() + self.backward_delay_samples.value())
            + self.compensation()
    }

    /// Current forward acoustic transit delay [samples].
    pub fn forward_delay_samples(&self) -> f32 {
        self.forward_delay_samples.value() + self.compensation()
    }

    /// Current backward acoustic transit delay [samples].
    pub fn backward_delay_samples(&self) -> f32 {
        self.backward_delay_samples.value() + self.compensation()
    }

    /// One-way transit time through the pipe [s].
    pub fn transit_time_seconds(&self) -> f32 {
        self.delay_samples() / self.sample_rate
    }

    /// Wall loss one pass down the pipe puts on a wave of frequency `f` [dB].
    pub fn wall_loss_db(&self, frequency: f32) -> f32 {
        self.forward_loss.attenuation_db(frequency)
    }

    /// Reads waves arriving at the boundaries from inside the pipe with viscothermal wall loss applied:
    /// - `out_port0`: wave emerging at port 0 from the backward delay line ($p^-(0)$).
    /// - `out_port1`: wave emerging at port 1 from the forward delay line ($p^+(L)$).
    #[inline(always)]
    pub fn read_outputs(&mut self) -> (f32, f32) {
        let d_fwd = self.forward_delay_samples.next_value();
        let d_bwd = self.backward_delay_samples.next_value();
        let raw0 = self.backward_line.read(d_bwd);
        let raw1 = if self.steepening {
            self.shocked_read(d_fwd)
        } else {
            self.forward_line.read(d_fwd)
        };
        let out0 = self.backward_loss.process(raw0);
        let out1 = self.forward_loss.process(raw1);
        (out0, out1)
    }

    /// Reads the downstream end of the pipe at the time the gas dynamics say the
    /// wave arrives, rather than the one time linear acoustics gives all of it.
    ///
    /// Every point of the stored waveform is a characteristic. The one launched
    /// $k$ samples after the point that would arrive now travels $\beta$ faster
    /// and so arrives now as well if it can make up exactly those $k$ samples:
    ///
    /// $$k = D \frac{\beta}{1 + \beta}, \qquad
    ///   \beta = \frac{\gamma + 1}{2\gamma} \frac{p}{P_0}$$
    ///
    /// which rearranges to a residual with no divide in it,
    ///
    /// $$h(k) = \frac{\gamma+1}{2\gamma} (D - k) \, p(k) - k P_0,$$
    ///
    /// zero exactly at a characteristic that arrives now. Where the front is
    /// steep enough that crests overtake the troughs ahead of them, $h$ has
    /// more than one zero: the waveform has become multivalued, which is the
    /// analytic statement that it has shocked. The entropy condition picks the
    /// characteristic that arrives *first*, so this takes the largest root, and
    /// the output jumps from the flank to the crest in one sample — a step,
    /// which is what a shock is. Behind it the roots march back down the decay,
    /// so nothing is held and nothing is repeated.
    ///
    /// It cannot invent energy: every sample it returns is an interpolation
    /// between samples already in the line, so the output can never exceed what
    /// the pipe was given. That matters because this sits inside a feedback
    /// loop, where anything that adds a little comes back round to add it again.
    ///
    /// The search runs out to the advance a crest one atmosphere over ambient
    /// earns, $k_{max} = D\beta/(1+\beta)$ at $p = P_0$ — a pressure ratio of
    /// two, which is the shock condition itself. A crest above that arrives at
    /// the window edge instead of a root, and it costs nothing: past the shock
    /// the front is already a step, and arriving earlier still does not make a
    /// step steeper.
    #[inline(always)]
    fn shocked_read(&mut self, transit: f32) -> f32 {
        let b = self.nonlinearity;
        let span = (transit * b / (1.0 + b)).min(transit - 3.0);
        let line = &mut self.forward_line;
        if span < 1.0 {
            let out = line.read(transit);
            self.emitted = out;
            return out;
        }
        // Rankine-Hugoniot: a front does not travel at the speed of the crest
        // behind it but at the mean of the speeds of the two states it
        // separates, so the pressure in the residual is the mean of the
        // characteristic's own and the one the pipe last emitted — the state it
        // is running into. On a smooth wave the two are the same number and
        // this is the simple-wave result unchanged. On a shocked front they are
        // not: the front arrives later than the crest alone would put it, and
        // the sample that comes out is one further down the flank. That is the
        // shock's dissipation, and it is the reason a blowdown does not stay a
        // step all the way to the mouth.
        let ahead = self.emitted;
        let residual = |line: &DelayLine, k: f32| -> f32 {
            let p = 0.5 * (line.read_linear(transit - k) + ahead);
            b * (transit - k) * p - k * REFERENCE_PRESSURE_PA
        };
        let steps = span as i32;
        let mut k = steps;
        let mut h_above = residual(line, k as f32);
        // Nothing in the window is slow enough to have a root: the whole front
        // is past the shock condition and arrives at the window edge.
        let mut advance = span;
        if h_above < 0.0 {
            // The other edge is the fallback: a rarefaction deeper than the
            // window resolves lags by at most this much.
            advance = -(steps as f32);
            while k > -steps {
                let below = k - 1;
                let h_below = residual(line, below as f32);
                if h_below >= 0.0 {
                    let denom = h_below - h_above;
                    let frac = if denom > 1e-12 { h_below / denom } else { 0.0 };
                    advance = below as f32 + frac;
                    break;
                }
                h_above = h_below;
                k = below;
            }
        }
        let out = line.read_lagrange3(transit - advance);

        // And what the front loses by being a front. A shock is not a lossless
        // feature of a wave: it carries an entropy jump, and the crest is eaten
        // by it as it runs. The same group that decides how far a crest
        // advances decides how fast a front can be, because they are the same
        // statement read two ways —
        //
        // $$\sigma = \frac{\gamma+1}{2\gamma} \frac{D \, |\Delta p|}{P_0}$$
        //
        // is the number of sample-steps' worth of steepening this section asks
        // of a jump of `dp` per sample. At `sigma = 1` the section has used up
        // exactly the slope it had, which is the shock formation condition;
        // past it the front is asking to become steeper than a step, and what a
        // real gas does instead is dissipate the difference. So the jump is
        // divided by how far past the condition it is, which is the sawtooth
        // decay law with the length and the pressure it is actually carrying
        // and no constant of its own. Below the condition nothing is touched:
        // a wave that has not shocked does not pay for a shock.
        let jump = out - ahead;
        let sigma = b * transit * jump.abs() * (1.0 / REFERENCE_PRESSURE_PA);
        let out = if sigma > 1.0 { out / sigma } else { out };
        self.emitted = out;
        out
    }

    /// Injects waves incident on the pipe boundaries into the travelling delay lines:
    /// - `in_port0`: forward-travelling wave entering at port 0 ($p^+(0)$).
    /// - `in_port1`: backward-travelling wave entering at port 1 ($p^-(L)$).
    #[inline(always)]
    pub fn push_inputs(&mut self, in_port0: f32, in_port1: f32) {
        self.forward_line.push(in_port0);
        self.backward_line.push(in_port1);
    }

    /// Resets all delay line state to silence.
    pub fn reset(&mut self) {
        self.emitted = 0.0;
        self.forward_line.reset();
        self.backward_line.reset();
        self.forward_loss.reset();
        self.backward_loss.reset();
        self.forward_delay_samples
            .snap(self.forward_delay_samples.target());
        self.backward_delay_samples
            .snap(self.backward_delay_samples.target());
    }
}

// ---------------------------------------------------------------------------
// Scattering junction
// ---------------------------------------------------------------------------

/// N-port acoustic scattering junction.
///
/// Joins $N$ waveguide ducts meeting at a single acoustic node.
/// Given port admittances $Y_i = A_i / (\rho c)$ and incoming pressure waves $p_i^+$,
/// the acoustic pressure at the junction is:
///
/// $$p_J = \frac{2 \sum_i Y_i p_i^+}{\sum_i Y_i}$$
///
/// and outgoing scattered pressure waves returning into each duct are:
///
/// $$p_i^- = p_J - p_i^+$$
///
/// For two pipes of area $A_1$ and $A_2$, the reflection coefficient for a wave
/// entering from pipe 1 collapses to:
///
/// $$r = \frac{A_1 - A_2}{A_1 + A_2}$$
#[derive(Debug, Clone)]
pub struct ScatteringJunction {
    admittances: Vec<f32>,
    inv_total_admittance: f32,
}

impl ScatteringJunction {
    /// Constructs a new N-port junction from port admittances $Y_i$.
    pub fn new(admittances: &[f32]) -> Self {
        let total: f32 = admittances.iter().sum();
        let inv_total = if total > 1e-12 { 1.0 / total } else { 0.0 };
        Self {
            admittances: admittances.to_vec(),
            inv_total_admittance: inv_total,
        }
    }

    /// Constructs a junction from port cross-sectional areas [m^2] under uniform gas conditions.
    pub fn from_areas(areas: &[f64]) -> Self {
        let admittances: Vec<f32> = areas.iter().map(|&a| a.max(1e-7) as f32).collect();
        Self::new(&admittances)
    }

    /// Number of connected ports.
    pub fn port_count(&self) -> usize {
        self.admittances.len()
    }

    /// Returns a slice of the current port admittances.
    pub fn admittances(&self) -> &[f32] {
        &self.admittances
    }

    /// Updates port admittances when gas state or geometry changes.
    pub fn set_admittances(&mut self, admittances: &[f32]) {
        assert_eq!(admittances.len(), self.admittances.len());
        self.admittances.copy_from_slice(admittances);
        let total: f32 = self.admittances.iter().sum();
        self.inv_total_admittance = if total > 1e-12 { 1.0 / total } else { 0.0 };
    }

    /// Computes scattering across all ports:
    /// - `p_plus`: incident pressure waves arriving at the junction from each connected duct.
    /// - `p_minus`: buffer filled with outgoing pressure waves returning into each duct.
    ///
    /// Returns the junction pressure $p_J$.
    #[inline(always)]
    pub fn scatter(&self, p_plus: &[f32], p_minus: &mut [f32]) -> f32 {
        debug_assert_eq!(p_plus.len(), self.admittances.len());
        debug_assert_eq!(p_minus.len(), self.admittances.len());

        let mut sum_yp = 0.0f32;
        for (&y, &p) in self.admittances.iter().zip(p_plus.iter()) {
            sum_yp += y * p;
        }
        let p_j = 2.0 * sum_yp * self.inv_total_admittance;
        for (out, &p) in p_minus.iter_mut().zip(p_plus.iter()) {
            *out = p_j - p;
        }
        p_j
    }
}

// ---------------------------------------------------------------------------
// Composed elements: Expansion chamber
// ---------------------------------------------------------------------------

/// Acoustic expansion chamber silencer composed from area steps.
///
/// Models a sudden expansion to area $A_2 = m A_1$ over length $L$, followed by
/// a contraction back to $A_1$. Built from two 2-port scattering junctions
/// separated by a bidirectional waveguide pipe.
///
/// Classical transmission loss for an expansion chamber of length $L$ and area ratio $m$:
///
/// $$TL = 10 \log_{10} \left[ 1 + \frac{1}{4} \left(m - \frac{1}{m}\right)^2 \sin^2(k L) \right]$$
#[derive(Debug, Clone)]
pub struct ExpansionChamber {
    junction_in: ScatteringJunction,
    cavity: WaveguidePipe,
    junction_out: ScatteringJunction,
    scatter_buf_in: [f32; 2],
    scatter_buf_out: [f32; 2],
}

impl ExpansionChamber {
    /// Constructs a single-stage expansion chamber:
    /// - `pipe_area`: cross-sectional area of inlet and outlet pipes ($A_1$) [m^2].
    /// - `area_ratio`: expansion ratio $m = A_2 / A_1$ [-].
    /// - `length`: length of the expansion cavity $L$ [m].
    pub fn new(
        pipe_area: f64,
        area_ratio: f64,
        length: f64,
        sample_rate: f32,
        gamma: f32,
        gas_constant: f32,
        temperature: f32,
    ) -> Self {
        let a1 = pipe_area.max(1e-7);
        let a2 = a1 * area_ratio.max(1.0);
        let junction_in = ScatteringJunction::from_areas(&[a1, a2]);
        let cavity = WaveguidePipe::new(length, a2, sample_rate, gamma, gas_constant, temperature);
        let junction_out = ScatteringJunction::from_areas(&[a2, a1]);

        Self {
            junction_in,
            cavity,
            junction_out,
            scatter_buf_in: [0.0; 2],
            scatter_buf_out: [0.0; 2],
        }
    }

    /// Retunes propagation delay and acoustic admittance for current gas state.
    pub fn tune(&mut self, gamma: f32, gas_constant: f32, temperature: f32) {
        self.cavity.tune(gamma, gas_constant, temperature);
    }

    /// Steps the expansion chamber by one sample:
    /// - `p_in_plus`: forward wave entering the chamber from upstream ($p^+$).
    /// - `p_out_minus`: backward wave incident on the chamber exit from downstream ($p^-$).
    ///
    /// Returns `(p_in_minus, p_out_plus)`:
    /// - `p_in_minus`: reflected wave returning upstream.
    /// - `p_out_plus`: transmitted wave continuing downstream.
    #[inline(always)]
    pub fn step(&mut self, p_in_plus: f32, p_out_minus: f32) -> (f32, f32) {
        let (p_cav_0, p_cav_1) = self.cavity.read_outputs();

        // Upstream junction: inlet duct (0) and cavity inlet (1)
        self.junction_in
            .scatter(&[p_in_plus, p_cav_0], &mut self.scatter_buf_in);
        let p_in_minus = self.scatter_buf_in[0];
        let p_into_cav_0 = self.scatter_buf_in[1];

        // Downstream junction: cavity exit (0) and outlet duct (1)
        self.junction_out
            .scatter(&[p_cav_1, p_out_minus], &mut self.scatter_buf_out);
        let p_into_cav_1 = self.scatter_buf_out[0];
        let p_out_plus = self.scatter_buf_out[1];

        self.cavity.push_inputs(p_into_cav_0, p_into_cav_1);

        (p_in_minus, p_out_plus)
    }

    /// Clears internal state.
    pub fn reset(&mut self) {
        self.cavity.reset();
        self.scatter_buf_in = [0.0; 2];
        self.scatter_buf_out = [0.0; 2];
    }
}

// ---------------------------------------------------------------------------
// Composed elements: Quarter-wave stub (drone killer)
// ---------------------------------------------------------------------------

/// Acoustic quarter-wave side-branch resonator (destructive interference notch).
///
/// Composed of a 3-port scattering junction joining the main duct with a closed-end
/// side branch. At the quarter-wave frequency:
///
/// $$f_{\text{notch}} = \frac{c}{4 L_{\text{stub}}}$$
///
/// the round-trip through the stub covers $\lambda / 2$ ($\pi$ phase delay).
/// Combined with the in-phase rigid reflection ($r = +1.0$) at the closed end,
/// the returning wave arrives in anti-phase at the junction, producing a deep
/// transmission notch.
#[derive(Debug, Clone)]
pub struct QuarterWaveStub {
    junction: ScatteringJunction,
    stub_pipe: WaveguidePipe,
    length: f32,
    scatter_buf: [f32; 3],
}

impl QuarterWaveStub {
    /// Constructs a quarter-wave stub:
    /// - `pipe_area`: through-duct cross-sectional area [m^2].
    /// - `stub_area`: side branch cross-sectional area [m^2].
    /// - `stub_length`: side branch centerline length $L_{\text{stub}}$ [m].
    pub fn new(
        pipe_area: f64,
        stub_area: f64,
        stub_length: f64,
        sample_rate: f32,
        gamma: f32,
        gas_constant: f32,
        temperature: f32,
    ) -> Self {
        let ap = pipe_area.max(1e-7);
        let as_ = stub_area.max(1e-7);
        let junction = ScatteringJunction::from_areas(&[ap, ap, as_]);
        let stub_pipe = WaveguidePipe::new(
            stub_length,
            as_,
            sample_rate,
            gamma,
            gas_constant,
            temperature,
        );

        Self {
            junction,
            stub_pipe,
            length: stub_length.max(0.001) as f32,
            scatter_buf: [0.0; 3],
        }
    }

    /// Theoretical quarter-wave notch frequency [Hz]:
    ///
    /// $$f_0 = \frac{c}{4 L_{\text{stub}}}$$
    pub fn notch_frequency_hz(&self, speed_of_sound: f32) -> f32 {
        speed_of_sound / (4.0 * self.length)
    }

    /// Retunes propagation delay and acoustic admittance for current gas state.
    pub fn tune(&mut self, gamma: f32, gas_constant: f32, temperature: f32) {
        self.stub_pipe.tune(gamma, gas_constant, temperature);
    }

    /// Steps the stub by one sample:
    /// - `p_in_plus`: forward wave arriving from upstream ($p^+$).
    /// - `p_out_minus`: backward wave arriving from downstream ($p^-$).
    ///
    /// Returns `(p_in_minus, p_out_plus)`:
    /// - `p_in_minus`: reflected wave returning upstream.
    /// - `p_out_plus`: transmitted wave continuing downstream.
    #[inline(always)]
    pub fn step(&mut self, p_in_plus: f32, p_out_minus: f32) -> (f32, f32) {
        let (p_stub_0, p_stub_1) = self.stub_pipe.read_outputs();

        // Closed rigid end at port 1: reflection is +1.0
        let p_rigid_reflected = p_stub_1;

        // 3-port junction: upstream (0), downstream (1), stub inlet (2)
        self.junction
            .scatter(&[p_in_plus, p_out_minus, p_stub_0], &mut self.scatter_buf);
        let p_in_minus = self.scatter_buf[0];
        let p_out_plus = self.scatter_buf[1];
        let p_into_stub = self.scatter_buf[2];

        self.stub_pipe.push_inputs(p_into_stub, p_rigid_reflected);

        (p_in_minus, p_out_plus)
    }

    /// Clears internal state.
    pub fn reset(&mut self) {
        self.stub_pipe.reset();
        self.scatter_buf = [0.0; 3];
    }
}

// ---------------------------------------------------------------------------
// Composed elements: Tapered collector
// ---------------------------------------------------------------------------

/// Exhaust collector junction uniting multiple primary runners into a collector pipe.
///
/// Composed of an $(N_{\text{in}} + 1)$-port scattering junction and a transition pipe
/// section of length $L_{\text{taper}}$.
///
/// Accurately models the arithmetic reflection at the collector and the acoustic
/// wave cross-talk between primary runners: when one cylinder exhausts into the
/// collector, the area expansion inverts the returning reflection ($r < 0$) while
/// transmitting positive pressure waves into the other primary runners and downstream.
#[derive(Debug, Clone)]
pub struct TaperedCollector {
    junction: ScatteringJunction,
    taper_pipe: WaveguidePipe,
    inlet_count: usize,
    scatter_in: Vec<f32>,
    scatter_out: Vec<f32>,
}

impl TaperedCollector {
    /// Constructs a collector junction:
    /// - `inlet_count`: number of primary inlet runners $N_{\text{in}}$.
    /// - `primary_area`: cross-sectional area of each primary runner [m^2].
    /// - `outlet_area`: cross-sectional area of collector outlet [m^2].
    /// - `taper_length`: length of the converging transition section [m].
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        inlet_count: usize,
        primary_area: f64,
        outlet_area: f64,
        taper_length: f64,
        sample_rate: f32,
        gamma: f32,
        gas_constant: f32,
        temperature: f32,
    ) -> Self {
        let n = inlet_count.max(1);
        let a_prim = primary_area.max(1e-7);
        let a_out = outlet_area.max(1e-7);

        let mut areas = vec![a_prim; n];
        areas.push(a_out);
        let junction = ScatteringJunction::from_areas(&areas);

        let l_taper = taper_length.max(0.02);
        let taper_pipe = WaveguidePipe::new(
            l_taper,
            a_out,
            sample_rate,
            gamma,
            gas_constant,
            temperature,
        );

        Self {
            junction,
            taper_pipe,
            inlet_count: n,
            scatter_in: vec![0.0; n + 1],
            scatter_out: vec![0.0; n + 1],
        }
    }

    /// Number of inlet runners.
    pub fn inlet_count(&self) -> usize {
        self.inlet_count
    }

    /// One-way transit delay of the converging taper [samples].
    pub fn taper_delay_samples(&self) -> f32 {
        self.taper_pipe.delay_samples()
    }

    /// Retunes propagation delay and acoustic admittance for current gas state.
    pub fn tune(&mut self, gamma: f32, gas_constant: f32, temperature: f32) {
        self.taper_pipe.tune(gamma, gas_constant, temperature);
    }

    /// Steps the collector by one sample:
    /// - `primaries_plus`: forward waves arriving from each primary runner ($p_i^+$).
    /// - `excitation`: pressure released *inside* the collector this sample [Pa].
    /// - `downstream_minus`: backward wave returning from downstream ($p^-$).
    /// - `primaries_minus`: output buffer filled with waves returning up each primary ($p_i^-$).
    ///
    /// The excitation enters as an incident wave at the junction rather than as
    /// something added to the output, so it scatters into every port the way a
    /// pressure rise at that node physically does: down the collector, and back
    /// up every primary standing open to it.
    ///
    /// Returns the transmitted wave continuing downstream.
    #[inline(always)]
    pub fn step(
        &mut self,
        primaries_plus: &[f32],
        excitation: f32,
        downstream_minus: f32,
        primaries_minus: &mut [f32],
    ) -> f32 {
        debug_assert_eq!(primaries_plus.len(), self.inlet_count);
        debug_assert_eq!(primaries_minus.len(), self.inlet_count);

        let (p_taper_upstream, p_taper_downstream) = self.taper_pipe.read_outputs();

        // Assemble incident waves at (N + 1) junction:
        // Ports 0..N: primaries; Port N: taper inlet
        for (dst, &src) in self.scatter_in[..self.inlet_count]
            .iter_mut()
            .zip(primaries_plus.iter())
        {
            *dst = src;
        }
        self.scatter_in[self.inlet_count] = p_taper_upstream + excitation;

        self.junction
            .scatter(&self.scatter_in, &mut self.scatter_out);

        for (dst, &src) in primaries_minus
            .iter_mut()
            .zip(self.scatter_out[..self.inlet_count].iter())
        {
            *dst = src;
        }
        let p_into_taper = self.scatter_out[self.inlet_count];

        self.taper_pipe.push_inputs(p_into_taper, downstream_minus);

        p_taper_downstream
    }

    /// Clears internal state.
    pub fn reset(&mut self) {
        self.taper_pipe.reset();
        self.scatter_in.fill(0.0);
        self.scatter_out.fill(0.0);
    }
}

/// Phase speed inside acoustic packing, as a fraction of the free-gas value [-].
///
/// A wave in a fibre bed does not travel at $c$. It threads a tortuous path
/// through the fibres and the bed's own compliance adds to the gas's, and both
/// slow it down; for the mineral and glass wools used in silencers the phase
/// speed comes out around half of free-field. It matters here because it is
/// what sets the depth a given wavelength needs — packing that would be a
/// quarter wave deep at 4 kHz in open gas is a quarter wave deep at 2 kHz once
/// the wave is inside it, so the absorber reaches an octave further down than
/// its dry dimensions suggest.
pub const PACKING_SOUND_SPEED_RATIO: f32 = 0.5;

/// Attenuation of a lined duct, from Sabine's empirical formula [dB/m]:
///
/// ```text
/// alpha_dB = 1.05 (P / A) a_bar^1.4
/// ```
///
/// `P / A` is the lined perimeter over the open flow area — for a circular
/// core of radius $a$ that is $2 / a$, so a narrow core is silenced far harder
/// than a wide one of the same packing, because every part of the gas is
/// closer to something absorbing. `a_bar` is the absorption coefficient of the
/// packing itself, near 0.8 for the mineral and glass wools used in silencers.
///
/// This is where the decibels come from, rather than from a number somebody
/// liked: a 64 mm core in good packing works out near 48 dB/m, so a 0.6 m body
/// is worth some 29 dB across the band it covers — which is what a real
/// straight-through absorptive muffler measures, and an order of magnitude
/// more than a figure picked to sound about right.
#[inline]
pub fn sabine_attenuation_db_per_m(core_radius: f64, packing_absorption: f64) -> f64 {
    let perimeter_over_area = 2.0 / core_radius.max(1e-4);
    1.05 * perimeter_over_area * packing_absorption.clamp(0.0, 1.0).powf(1.4)
}

/// Frequency at which packing of depth `thickness` starts absorbing properly [Hz]:
///
/// ```text
/// f_q = c_packing / (4 t)
/// ```
///
/// Porous packing dissipates by dragging gas through fibres, so it can only
/// work where the gas is moving. Against the rigid shell the particle velocity
/// is zero, and it is greatest a quarter wavelength out from it — so a layer
/// of depth $t$ is fully effective once $t$ reaches $\lambda / 4$, and below
/// that its attenuation falls away as $f^2$ with the square of the velocity it
/// has to work with.
///
/// This is the single number that decides which half of the spectrum a
/// silencer takes, and it is why packing is the wrong tool for a drone: a
/// 35 mm wrap in hot gas turns over around 2 kHz, so it takes the hiss and the
/// rasp and leaves a 100 Hz boom completely untouched. Reaching that low
/// wants a reactive element, not a thicker blanket.
#[inline]
pub fn packing_corner_hz(thickness: f64, speed_of_sound: f32) -> f32 {
    PACKING_SOUND_SPEED_RATIO * speed_of_sound / (4.0 * thickness.max(1e-4) as f32)
}

// ---------------------------------------------------------------------------
// Composed elements: Absorptive silencer
// ---------------------------------------------------------------------------

/// Straight-through packed silencer: a perforated core inside acoustic packing.
///
/// The core is a duct like any other, so it is a [`WaveguidePipe`] with area
/// steps at each end. What the packing adds is resistive attenuation, quoted by
/// the geometry as a loss in decibels per metre. Over a body of length $L$ the
/// amplitude surviving one pass, *once the packing is working*, is
///
/// $$g = 10^{-\frac{\alpha_{dB} L}{20}}$$
///
/// applied to each direction of travel, so a wave that goes in and comes back
/// pays for the length twice — which is exactly why a packed silencer kills the
/// returning reflection harder than it kills the through path.
///
/// # Why it is not one number
///
/// The packing does not take that much out at every frequency, and modelling
/// it as a flat gain gets the whole point of the element backwards. Fibre
/// absorbs by dragging gas through itself, so it works where the gas moves,
/// and the gas barely moves within a quarter wavelength of the shell — see
/// [`packing_corner_hz`]. Below that corner the attenuation falls away as
/// $f^2$; above it the packing is as effective as it is ever going to be.
///
/// So the element is a shelf, not a scalar:
///
/// ```text
/// H(s) = (1 + g s / w_q) / (1 + s / w_q)
/// ```
///
/// unity at DC, `g` above the corner, with the $f^2$ approach to it that the
/// velocity argument demands. A flat gain of `g` would take the same decibels
/// off the firing fundamental as off the rasp, leaving the balance between them
/// exactly where it found it — which is not what a muffler is for, and not what
/// one sounds like.
#[derive(Debug, Clone)]
pub struct AbsorptiveSilencer {
    junction_in: ScatteringJunction,
    core: WaveguidePipe,
    junction_out: ScatteringJunction,
    /// Amplitude surviving one pass through the packing, above its corner [-].
    pass: f32,
    /// Radial depth of packing around the core [m].
    thickness: f64,
    /// Corner of the absorption shelf as currently tuned [Hz].
    corner_hz: f32,
    /// Shelf state for each direction of travel through the body.
    shelf_forward: OnePole,
    shelf_backward: OnePole,
    sample_rate: f32,
    scatter_buf_in: [f32; 2],
    scatter_buf_out: [f32; 2],
}

impl AbsorptiveSilencer {
    /// Constructs a packed silencer:
    /// - `pipe_area`: area of the ducts either side of the body [m^2].
    /// - `core_area`: flow area through the perforated core [m^2].
    /// - `length`: length of the packed body [m].
    /// - `packing_thickness`: radial depth of packing around the core [m].
    /// - `loss_db_per_m`: attenuation of the packing above its corner [dB/m].
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        pipe_area: f64,
        core_area: f64,
        length: f64,
        packing_thickness: f64,
        loss_db_per_m: f64,
        sample_rate: f32,
        gamma: f32,
        gas_constant: f32,
        temperature: f32,
    ) -> Self {
        let a1 = pipe_area.max(1e-7);
        let a2 = core_area.max(1e-7);
        let l = length.max(0.001);
        let core = WaveguidePipe::new(l, a2, sample_rate, gamma, gas_constant, temperature);
        let pass = 10f64.powf(-loss_db_per_m.max(0.0) * l / 20.0) as f32;
        let corner_hz = packing_corner_hz(
            packing_thickness,
            speed_of_sound(gamma, gas_constant, temperature),
        );

        Self {
            junction_in: ScatteringJunction::from_areas(&[a1, a2]),
            core,
            junction_out: ScatteringJunction::from_areas(&[a2, a1]),
            pass,
            thickness: packing_thickness,
            corner_hz,
            shelf_forward: OnePole::new(sample_rate, corner_hz),
            shelf_backward: OnePole::new(sample_rate, corner_hz),
            sample_rate,
            scatter_buf_in: [0.0; 2],
            scatter_buf_out: [0.0; 2],
        }
    }

    /// Amplitude surviving one pass through the packing, above its corner [-].
    pub fn pass_gain(&self) -> f32 {
        self.pass
    }

    /// Frequency above which the packing is fully effective [Hz].
    pub fn corner_hz(&self) -> f32 {
        self.corner_hz
    }

    /// Retunes propagation delay, admittance and the absorption corner for the
    /// current gas state.
    ///
    /// The corner moves with the gas like every other acoustic length in the
    /// model: hot packing is acoustically shallower than cold packing, so a
    /// silencer absorbs a little less of the midrange as the system warms.
    pub fn tune(&mut self, gamma: f32, gas_constant: f32, temperature: f32) {
        self.core.tune(gamma, gas_constant, temperature);
        self.corner_hz = packing_corner_hz(
            self.thickness,
            speed_of_sound(gamma, gas_constant, temperature),
        );
        self.shelf_forward
            .set_cutoff(self.sample_rate, self.corner_hz);
        self.shelf_backward
            .set_cutoff(self.sample_rate, self.corner_hz);
    }

    /// Applies the packing's absorption shelf to one direction of travel.
    ///
    /// Unity at DC, [`pass_gain`](Self::pass_gain) above the corner: the
    /// low-frequency part of the wave is handed back untouched and only what
    /// the fibre can actually work on is taken out.
    #[inline(always)]
    fn absorb(pass: f32, shelf: &mut OnePole, x: f32) -> f32 {
        pass * x + (1.0 - pass) * shelf.process(x)
    }

    /// Steps the silencer by one sample; see [`ExpansionChamber::step`] for the
    /// port convention.
    #[inline(always)]
    pub fn step(&mut self, p_in_plus: f32, p_out_minus: f32) -> (f32, f32) {
        let (p_core_0, p_core_1) = self.core.read_outputs();
        let p_core_0 = Self::absorb(self.pass, &mut self.shelf_backward, p_core_0);
        let p_core_1 = Self::absorb(self.pass, &mut self.shelf_forward, p_core_1);

        self.junction_in
            .scatter(&[p_in_plus, p_core_0], &mut self.scatter_buf_in);
        let p_in_minus = self.scatter_buf_in[0];
        let p_into_core_0 = self.scatter_buf_in[1];

        self.junction_out
            .scatter(&[p_core_1, p_out_minus], &mut self.scatter_buf_out);
        let p_into_core_1 = self.scatter_buf_out[0];
        let p_out_plus = self.scatter_buf_out[1];

        self.core.push_inputs(p_into_core_0, p_into_core_1);

        (p_in_minus, p_out_plus)
    }

    /// Clears internal state.
    pub fn reset(&mut self) {
        self.core.reset();
        self.shelf_forward.reset();
        self.shelf_backward.reset();
        self.scatter_buf_in = [0.0; 2];
        self.scatter_buf_out = [0.0; 2];
    }
}

// ---------------------------------------------------------------------------
// Composed elements: the silencer chain
// ---------------------------------------------------------------------------

/// Side branch length that puts a quarter-wave notch on a Helmholtz resonance [m].
///
/// A Helmholtz chamber of volume $V$ behind a neck of area $A_n$ and length
/// $L_n$ resonates at $f_H = \frac{c}{2\pi}\sqrt{A_n / (V L_n)}$, and a closed
/// side branch of length $L$ notches at $c / 4L$. Equating the two,
///
/// $$L = \frac{\pi}{2} \sqrt{\frac{V L_n}{A_n}}$$
///
/// and the speed of sound cancels: the equivalent length is pure geometry, so
/// the branch tracks temperature exactly the way the chamber it stands in for
/// does. That is what lets a Helmholtz muffler be a side branch on the same
/// 3-port junction as every other side branch rather than a special case.
#[inline]
pub fn helmholtz_equivalent_stub_length(
    neck_area: f64,
    chamber_volume: f64,
    neck_length: f64,
) -> f64 {
    let ratio = chamber_volume.max(1e-9) * neck_length.max(1e-6) / neck_area.max(1e-9);
    std::f64::consts::FRAC_PI_2 * ratio.sqrt()
}

/// One silencing element in a bank's chain, composed from the primitives.
///
/// Every variant of [`crate::physics::plumbing::Silencer`] lands here, so the
/// network never has to ask what kind of silencer it is holding — a straight
/// pipe is simply a chain with nothing in it.
#[derive(Debug, Clone)]
pub enum SilencerElement {
    /// A sudden expansion and the contraction back, one stage.
    Chamber(ExpansionChamber),
    /// A packed straight-through body.
    Absorptive(AbsorptiveSilencer),
    /// A closed side branch, either a drone-killer stub or a Helmholtz chamber
    /// standing in as one; see [`helmholtz_equivalent_stub_length`].
    SideBranch(QuarterWaveStub),
}

/// Temperature of silencer element `index` of `count`, down the gradient from
/// the collector to the tailpipe [K].
///
/// A silencer chain is strung out along the coolest half of the exhaust, and
/// where each element sits in it decides what it is tuned to: a chamber
/// breathing 1260 K gas passes a band a tenth above the same chamber breathing
/// 1050 K. The first element does not start at the collector's own temperature
/// and the last does not reach the tailpipe's — there is pipe either side of
/// the chain — so the run is taken over the middle 70 % of the gradient,
/// starting a fifth of the way down it.
///
/// Named here rather than written out at each of its uses because the
/// calibration in `examples/calibrate.rs` predicts a chamber's pass band from
/// it: a prediction taken off a different gradient than the one the network is
/// tuned to would be measuring the difference between two guesses at the
/// temperature.
pub fn chain_element_temperature(collector: f32, tailpipe: f32, index: usize, count: usize) -> f32 {
    let frac = (index as f32 + 0.5) / count.max(1) as f32;
    collector + (tailpipe - collector) * (0.2 + 0.7 * frac)
}

impl SilencerElement {
    /// Appends the elements a geometric silencer is made of to `chain`.
    ///
    /// A [`crate::physics::plumbing::Silencer::Straight`] appends nothing, and a
    /// multi-stage expansion chamber appends one [`ExpansionChamber`] per stage,
    /// which is what a stage *is*.
    pub fn extend_chain(
        chain: &mut Vec<Self>,
        silencer: &crate::physics::plumbing::Silencer,
        pipe_area: f64,
        sample_rate: f32,
        gamma: f32,
        gas_constant: f32,
        temperature: f32,
    ) {
        use crate::physics::plumbing::Silencer;
        match silencer {
            Silencer::Straight => {}
            Silencer::ExpansionChamber {
                length,
                area_ratio,
                stages,
            } => {
                for _ in 0..(*stages).max(1) {
                    chain.push(Self::Chamber(ExpansionChamber::new(
                        pipe_area,
                        *area_ratio,
                        *length,
                        sample_rate,
                        gamma,
                        gas_constant,
                        temperature,
                    )));
                }
            }
            Silencer::Absorptive {
                length,
                area,
                packing_thickness,
                packing_absorption,
            } => {
                let core_radius = (area.max(1e-7) / std::f64::consts::PI).sqrt();
                chain.push(Self::Absorptive(AbsorptiveSilencer::new(
                    pipe_area,
                    *area,
                    *length,
                    *packing_thickness,
                    sabine_attenuation_db_per_m(core_radius, *packing_absorption),
                    sample_rate,
                    gamma,
                    gas_constant,
                    temperature,
                )))
            }
            Silencer::QuarterWaveStub { length, area } => {
                chain.push(Self::SideBranch(QuarterWaveStub::new(
                    pipe_area,
                    *area,
                    *length,
                    sample_rate,
                    gamma,
                    gas_constant,
                    temperature,
                )))
            }
            Silencer::Helmholtz(geometry) => {
                let length = helmholtz_equivalent_stub_length(
                    geometry.neck_area,
                    geometry.chamber_volume,
                    geometry.neck_length,
                );
                chain.push(Self::SideBranch(QuarterWaveStub::new(
                    pipe_area,
                    geometry.neck_area,
                    length,
                    sample_rate,
                    gamma,
                    gas_constant,
                    temperature,
                )))
            }
        }
    }

    /// Retunes propagation delay and acoustic admittance for current gas state.
    pub fn tune(&mut self, gamma: f32, gas_constant: f32, temperature: f32) {
        match self {
            Self::Chamber(c) => c.tune(gamma, gas_constant, temperature),
            Self::Absorptive(a) => a.tune(gamma, gas_constant, temperature),
            Self::SideBranch(s) => s.tune(gamma, gas_constant, temperature),
        }
    }

    /// Steps the element by one sample; see [`ExpansionChamber::step`] for the
    /// port convention.
    #[inline(always)]
    pub fn step(&mut self, p_in_plus: f32, p_out_minus: f32) -> (f32, f32) {
        match self {
            Self::Chamber(c) => c.step(p_in_plus, p_out_minus),
            Self::Absorptive(a) => a.step(p_in_plus, p_out_minus),
            Self::SideBranch(s) => s.step(p_in_plus, p_out_minus),
        }
    }

    /// Clears internal state.
    pub fn reset(&mut self) {
        match self {
            Self::Chamber(c) => c.reset(),
            Self::Absorptive(a) => a.reset(),
            Self::SideBranch(s) => s.reset(),
        }
    }
}

// ---------------------------------------------------------------------------
// Composed elements: Bank crossover
// ---------------------------------------------------------------------------

/// Acoustic crossover linking dual exhaust banks.
///
/// Implements:
/// - [`BankCrossover::None`]: isolated banks with no cross-talk.
/// - [`BankCrossover::XPipe`]: a 4-port scattering junction where pulses from both banks
///   cross and mix directly.
/// - [`BankCrossover::HPipe`]: a balance tube linking the banks through two 3-port junctions.
#[derive(Debug, Clone)]
#[allow(clippy::large_enum_variant)]
pub enum BankCrossover {
    /// Independent dual exhaust with no link between banks.
    None,
    /// 4-port merged junction linking bank 0 and bank 1.
    XPipe {
        junction: ScatteringJunction,
        scatter_buf: [f32; 4],
    },
    /// Balance tube between banks with two 3-port scattering junctions.
    HPipe {
        junction_bank0: ScatteringJunction,
        junction_bank1: ScatteringJunction,
        balance_pipe: WaveguidePipe,
        buf_bank0: [f32; 3],
        buf_bank1: [f32; 3],
    },
}

impl BankCrossover {
    /// Constructs a crossover from the geometric specification in [`crate::physics::plumbing::Crossover`].
    pub fn from_crossover(
        crossover: &crate::physics::plumbing::Crossover,
        pipe_area: f64,
        sample_rate: f32,
        gamma: f32,
        gas_constant: f32,
        temperature: f32,
    ) -> Self {
        let ap = pipe_area.max(1e-7);
        match crossover {
            crate::physics::plumbing::Crossover::None => Self::None,
            crate::physics::plumbing::Crossover::XPipe { .. }
            | crate::physics::plumbing::Crossover::Balance180 => {
                let junction = ScatteringJunction::from_areas(&[ap, ap, ap, ap]);
                Self::XPipe {
                    junction,
                    scatter_buf: [0.0; 4],
                }
            }
            crate::physics::plumbing::Crossover::HPipe { area, .. } => {
                let ab = (*area).max(1e-7);
                let junction_bank0 = ScatteringJunction::from_areas(&[ap, ap, ab]);
                let junction_bank1 = ScatteringJunction::from_areas(&[ap, ap, ab]);
                // Typical balance tube span between cylinder banks is ~0.25 m
                let balance_pipe =
                    WaveguidePipe::new(0.25, ab, sample_rate, gamma, gas_constant, temperature);
                Self::HPipe {
                    junction_bank0,
                    junction_bank1,
                    balance_pipe,
                    buf_bank0: [0.0; 3],
                    buf_bank1: [0.0; 3],
                }
            }
        }
    }

    /// Retunes propagation delay and acoustic admittance for current gas state.
    pub fn tune(&mut self, gamma: f32, gas_constant: f32, temperature: f32) {
        if let Self::HPipe { balance_pipe, .. } = self {
            balance_pipe.tune(gamma, gas_constant, temperature);
        }
    }

    /// Steps the crossover by one sample:
    /// - `bank0_in_plus`: forward wave arriving from bank 0 upstream.
    /// - `bank1_in_plus`: forward wave arriving from bank 1 upstream.
    /// - `bank0_out_minus`: backward wave returning from bank 0 downstream.
    /// - `bank1_out_minus`: backward wave returning from bank 1 downstream.
    ///
    /// Returns `(bank0_in_minus, bank1_in_minus, bank0_out_plus, bank1_out_plus)`.
    #[inline(always)]
    pub fn step(
        &mut self,
        bank0_in_plus: f32,
        bank1_in_plus: f32,
        bank0_out_minus: f32,
        bank1_out_minus: f32,
    ) -> (f32, f32, f32, f32) {
        match self {
            Self::None => (
                bank0_out_minus,
                bank1_out_minus,
                bank0_in_plus,
                bank1_in_plus,
            ),
            Self::XPipe {
                junction,
                scatter_buf,
            } => {
                let p_plus = [
                    bank0_in_plus,
                    bank1_in_plus,
                    bank0_out_minus,
                    bank1_out_minus,
                ];
                junction.scatter(&p_plus, scatter_buf);
                (
                    scatter_buf[0],
                    scatter_buf[1],
                    scatter_buf[2],
                    scatter_buf[3],
                )
            }
            Self::HPipe {
                junction_bank0,
                junction_bank1,
                balance_pipe,
                buf_bank0,
                buf_bank1,
            } => {
                let (p_bal_0, p_bal_1) = balance_pipe.read_outputs();

                junction_bank0.scatter(&[bank0_in_plus, bank0_out_minus, p_bal_0], buf_bank0);
                let b0_in_minus = buf_bank0[0];
                let b0_out_plus = buf_bank0[1];
                let into_bal_0 = buf_bank0[2];

                junction_bank1.scatter(&[bank1_in_plus, bank1_out_minus, p_bal_1], buf_bank1);
                let b1_in_minus = buf_bank1[0];
                let b1_out_plus = buf_bank1[1];
                let into_bal_1 = buf_bank1[2];

                balance_pipe.push_inputs(into_bal_0, into_bal_1);

                (b0_in_minus, b1_in_minus, b0_out_plus, b1_out_plus)
            }
        }
    }

    /// Clears internal state.
    pub fn reset(&mut self) {
        match self {
            Self::None => {}
            Self::XPipe { scatter_buf, .. } => *scatter_buf = [0.0; 4],
            Self::HPipe {
                balance_pipe,
                buf_bank0,
                buf_bank1,
                ..
            } => {
                balance_pipe.reset();
                *buf_bank0 = [0.0; 3];
                *buf_bank1 = [0.0; 3];
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Exhaust waveguide network
// ---------------------------------------------------------------------------

/// Gas temperature at each station of the exhaust, in flow order [K].
///
/// One temperature for the whole system would say a header and a tailpipe are
/// the same thing acoustically, and they are not: gas leaves the port near
/// 1200 K and reaches the tailpipe two or three hundred Kelvin down, which is a
/// sixth off the speed of sound and therefore a sixth off every resonance the
/// back half of the system has. The gradient is [`crate::physics::thermal`]'s,
/// section by section.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExhaustTemperatures {
    /// Gas in each cylinder's primary runner [K], in cylinder order.
    pub primaries: [f32; crate::audio::dsp::MAX_CYLINDERS],
    /// Gas at the collector, the crossover and the silencers [K].
    pub collector: f32,
    /// Gas in the tailpipe, and therefore at the mouth [K].
    pub tailpipe: f32,
}

impl ExhaustTemperatures {
    /// Every station at one temperature, for a system with no gradient yet.
    pub fn uniform(temperature: f32) -> Self {
        Self {
            primaries: [temperature; crate::audio::dsp::MAX_CYLINDERS],
            collector: temperature,
            tailpipe: temperature,
        }
    }

    /// The temperature of cylinder `i`'s primary [K].
    pub fn primary(&self, cylinder: usize) -> f32 {
        self.primaries[cylinder.min(self.primaries.len() - 1)]
    }
}

/// Highest mean-flow Mach number a pipe is allowed to carry [-].
///
/// The time a wave takes to travel upstream goes as `1 / (1 - M)`, so at
/// `M = 1` the pipe never returns anything at all and above it the model is no
/// longer acoustic. Real exhaust flow does not reach it; the clamp is there so
/// that a transient in the solver cannot ask a delay line for a negative delay.
pub const MAX_MEAN_FLOW_MACH: f32 = 0.85;

/// Mean-flow Mach number in a pipe carrying `mass_flow` [-].
///
/// `u = mdot / (rho A)` with `rho = p / (R T)` at the reference pressure, and
/// `M = u / c`. It is what tilts a pipe's two delays apart — downstream at
/// `c + u` and upstream at `c - u` — and therefore what drops its resonance to
/// `c (1 - M^2) / 4L`, the mean-flow line of Appendix A.
///
/// Named here rather than written out inside the network because the
/// calibration in `examples/calibrate.rs` predicts that resonance and has to be
/// working from the same flow the network is: a prediction on a different Mach
/// number reports the difference between two guesses at the gas velocity as an
/// error in the geometry.
pub fn mean_flow_mach(
    mass_flow: f32,
    area: f32,
    gamma: f32,
    gas_constant: f32,
    temperature: f32,
) -> f32 {
    let c = speed_of_sound(gamma, gas_constant, temperature);
    let rho = REFERENCE_PRESSURE_PA / (gas_constant * temperature.max(1.0));
    let u = mass_flow.max(0.0) / (rho * area).max(1e-5);
    (u / c).clamp(-MAX_MEAN_FLOW_MACH, MAX_MEAN_FLOW_MACH)
}

/// Complete physical 1D exhaust waveguide network.
///
/// Instantiated from an [`ExhaustSystem`] geometry description:
/// - One primary runner per cylinder, terminated at the valve with [`ValveTermination`].
/// - An arithmetic [`TaperedCollector`] per bank.
/// - Bank [`BankCrossover`] linking dual-bank systems.
/// - Expansion chamber silencers.
/// - Tailpipes terminated with a [`Mouth`], which reflects, lengthens and radiates.
///
/// Fully preallocated at construction — zero allocations in the audio callback.
#[derive(Debug, Clone)]
pub struct ExhaustNetwork {
    primaries: Vec<WaveguidePipe>,
    valves: Vec<ValveTermination>,
    collectors: Vec<TaperedCollector>,
    crossover: BankCrossover,
    pre_cross_pipes: Vec<WaveguidePipe>,
    silencers: Vec<Vec<SilencerElement>>,
    tailpipes: Vec<WaveguidePipe>,
    mouths: Vec<Mouth>,
    bank_cylinders: Vec<Vec<usize>>,
    bank_count: usize,

    // Preallocated buffers for real-time processing
    prim_in_0: Vec<f32>,
    prim_to_collector: Vec<f32>,
    bank_prim_in: Vec<Vec<f32>>,
    bank_prim_refl: Vec<Vec<f32>>,
    bank_trans: Vec<f32>,
    bank_down: Vec<f32>,
    pre_cross_up: Vec<f32>,
    pre_cross_down: Vec<f32>,

    /// Backward wave waiting at each collector outlet [Pa].
    ///
    /// Only used where no pipe separates the collector from what follows it; a
    /// crossover at a stated position supplies its own delay and this stays zero.
    collector_returns: Vec<f32>,

    /// Backward waves waiting at each interface of a bank's downstream chain [Pa].
    ///
    /// Interface `0` is where the crossover hands over to the chain, interface
    /// `j` is between silencer `j-1` and silencer `j`, and the last one is the
    /// tailpipe entrance. Holding them for a sample is what closes the loop: a
    /// silencer and a mouth both reflect, and until those reflections can travel
    /// back up to the collector the network downstream of the header is a
    /// one-way chain rather than an acoustic system. One sample at 48 kHz is
    /// about 7 mm of pipe, an order below the shortest length any geometry here
    /// describes.
    chain_returns: Vec<Vec<f32>>,
    /// Whether the exhaust cutout / bypass junction is open.
    cutout_open: bool,
}

impl ExhaustNetwork {
    /// Constructs the network from physical geometry and engine configuration.
    pub fn new(
        exhaust: &crate::physics::plumbing::ExhaustSystem,
        cylinders: &[crate::audio::dsp::CylinderTap],
        bank_count: usize,
        sample_rate: f32,
        snapshot: &crate::audio::dsp::EngineSnapshot,
    ) -> Self {
        let n_cyl = cylinders.len().max(1);
        let n_banks = bank_count.max(1);

        let gamma = snapshot.exhaust_gamma;
        let r = snapshot.exhaust_gas_constant;
        let stations = ExhaustTemperatures {
            primaries: snapshot.primary_temperature,
            collector: snapshot.collector_temperature,
            tailpipe: snapshot.tailpipe_temperature,
        };
        let temp = stations.collector;
        let c = speed_of_sound(gamma, r, stations.tailpipe);

        // Map cylinders to banks
        let mut bank_cylinders = vec![Vec::new(); n_banks];
        for (i, tap) in cylinders.iter().enumerate() {
            let b = tap.bank % n_banks;
            bank_cylinders[b].push(i);
        }

        // 1. Primary runners and valve terminations
        let mut primaries = Vec::with_capacity(n_cyl);
        let mut valves = Vec::with_capacity(n_cyl);
        for i in 0..n_cyl {
            let spec = if !exhaust.primaries.is_empty() {
                exhaust.primaries[i % exhaust.primaries.len()]
            } else {
                crate::physics::plumbing::PipeSection::from_diameter(0.45, 0.040, 850.0)
            };
            let mut prim = WaveguidePipe::new(
                spec.length,
                spec.area,
                sample_rate,
                gamma,
                r,
                stations.primary(i),
            );
            // Off, and open question 7 in `docs/measurements/calibration.md`
            // says what it costs to turn on: the primary's loop is nearly
            // lossless, so a pulse goes round it thirty times and steepens on
            // each pass, and the accumulation drowns the geometry the pipe is
            // there to express.
            prim.set_steepening(false);
            let valve = ValveTermination::new(sample_rate, spec.area);
            primaries.push(prim);
            valves.push(valve);
        }

        // 2. Collectors per bank
        let mut collectors = Vec::with_capacity(n_banks);
        for cyls in &bank_cylinders {
            let inlets = cyls.len().max(1);
            let prim_area = if !exhaust.primaries.is_empty() {
                exhaust.primary_area()
            } else {
                std::f64::consts::PI * 0.020 * 0.020
            };
            let collector = TaperedCollector::new(
                inlets,
                prim_area,
                exhaust.collector.outlet_area,
                exhaust.collector.taper_length,
                sample_rate,
                gamma,
                r,
                temp,
            );
            collectors.push(collector);
        }

        // 3. Crossover and intermediate pipes
        let temp_cross = stations.collector + (stations.tailpipe - stations.collector) * 0.25;
        let crossover = BankCrossover::from_crossover(
            &exhaust.crossover,
            exhaust.collector.outlet_area,
            sample_rate,
            gamma,
            r,
            temp_cross,
        );

        // Where the banks meet is geometry, not a constant: the crossover sits a
        // stated distance downstream of the collector, and that run of pipe is
        // what decides which harmonics arrive at the junction in phase. An X-pipe
        // 0.4 m back and one 1.2 m back are different exhausts.
        let cross_position = match exhaust.crossover {
            crate::physics::plumbing::Crossover::None => None,
            crate::physics::plumbing::Crossover::XPipe { position }
            | crate::physics::plumbing::Crossover::HPipe { position, .. } => Some(position),
            // A 180-degree bundle crosses inside the header itself, so the banks
            // meet as soon as the collectors do.
            crate::physics::plumbing::Crossover::Balance180 => Some(0.0),
        };

        let temp_precross = stations.collector + (stations.tailpipe - stations.collector) * 0.15;
        let mut pre_cross_pipes = Vec::with_capacity(n_banks);
        if let Some(position) = cross_position {
            for _ in 0..n_banks {
                pre_cross_pipes.push(WaveguidePipe::new(
                    position,
                    exhaust.collector.outlet_area,
                    sample_rate,
                    gamma,
                    r,
                    temp_precross,
                ));
            }
        }

        // 4. Silencer chain per bank: interpolate temperatures down the gradient
        let mut silencers = vec![Vec::new(); n_banks];
        for chain in &mut silencers {
            for silencer in &exhaust.silencers {
                SilencerElement::extend_chain(
                    chain,
                    silencer,
                    exhaust.collector.outlet_area,
                    sample_rate,
                    gamma,
                    r,
                    temp,
                );
            }
            let n = chain.len();
            for (k, element) in chain.iter_mut().enumerate() {
                let t_elem = chain_element_temperature(stations.collector, stations.tailpipe, k, n);
                element.tune(gamma, r, t_elem);
            }
        }

        // 5. Tailpipes and Mouth terminations per bank
        let mut tailpipes = Vec::with_capacity(n_banks);
        let mut mouths = Vec::with_capacity(n_banks);
        for _ in 0..n_banks {
            let mouth = Mouth::new(
                sample_rate,
                (exhaust.tailpipe.diameter() * 0.5) as f32,
                exhaust.tailpipe_flanged,
                c,
            );
            let eff_tail_len = exhaust.tailpipe.length + mouth.end_correction() as f64;
            let mut tailpipe = WaveguidePipe::new(
                eff_tail_len,
                exhaust.tailpipe.area,
                sample_rate,
                gamma,
                r,
                stations.tailpipe,
            );
            // The mouth's reflection filter already holds part of the round
            // trip; leave it in the pipe as well and the tailpipe plays flat.
            tailpipe.set_boundary_phase_delay(mouth.phase_delay_samples());
            tailpipe.set_steepening(exhaust.cutout || exhaust.is_open_headers());
            tailpipe.tune(gamma, r, stations.tailpipe);
            tailpipes.push(tailpipe);
            mouths.push(mouth);
        }

        // Preallocate buffers
        let prim_in_0 = vec![0.0; n_cyl];
        let prim_to_collector = vec![0.0; n_cyl];
        let bank_prim_in = bank_cylinders
            .iter()
            .map(|cyls| vec![0.0; cyls.len().max(1)])
            .collect();
        let bank_prim_refl = bank_cylinders
            .iter()
            .map(|cyls| vec![0.0; cyls.len().max(1)])
            .collect();

        let chain_returns = silencers
            .iter()
            .map(|chain| vec![0.0; chain.len() + 1])
            .collect();

        Self {
            primaries,
            valves,
            collectors,
            crossover,
            pre_cross_pipes,
            silencers,
            tailpipes,
            mouths,
            bank_cylinders,
            bank_count: n_banks,
            prim_in_0,
            prim_to_collector,
            bank_prim_in,
            bank_prim_refl,
            bank_trans: vec![0.0; n_banks],
            bank_down: vec![0.0; n_banks],
            pre_cross_up: vec![0.0; n_banks],
            pre_cross_down: vec![0.0; n_banks],
            collector_returns: vec![0.0; n_banks],
            chain_returns,
            cutout_open: exhaust.cutout,
        }
    }

    /// Retunes propagation delay and acoustic filters across the whole network.
    ///
    /// Each station is retuned against its own gas, so the pitch of a primary
    /// and the pitch of the tailpipe move independently as the exhaust warms.
    pub fn tune(&mut self, gamma: f32, gas_constant: f32, stations: &ExhaustTemperatures) {
        for (i, p) in self.primaries.iter_mut().enumerate() {
            p.tune(gamma, gas_constant, stations.primary(i));
        }
        // The valve is a boundary on the same gas the primary behind it is
        // carrying, and every term of its load is measured against that pipe's
        // impedance, so it retunes with the pipe and not separately.
        for (i, v) in self.valves.iter_mut().enumerate() {
            v.tune(gamma, gas_constant, stations.primary(i));
        }
        for coll in &mut self.collectors {
            coll.tune(gamma, gas_constant, stations.collector);
        }
        let t_cross = stations.collector + (stations.tailpipe - stations.collector) * 0.25;
        self.crossover.tune(gamma, gas_constant, t_cross);
        let t_precross = stations.collector + (stations.tailpipe - stations.collector) * 0.15;
        for p in &mut self.pre_cross_pipes {
            p.tune(gamma, gas_constant, t_precross);
        }
        for chain in &mut self.silencers {
            let n = chain.len();
            for (k, element) in chain.iter_mut().enumerate() {
                let t_elem = chain_element_temperature(stations.collector, stations.tailpipe, k, n);
                element.tune(gamma, gas_constant, t_elem);
            }
        }
        let c_tail = speed_of_sound(gamma, gas_constant, stations.tailpipe);
        for tp in &mut self.tailpipes {
            tp.tune(gamma, gas_constant, stations.tailpipe);
        }
        for m in &mut self.mouths {
            m.tune(c_tail);
        }
    }

    /// Sets mean-flow Mach number for the primaries and tailpipes from mass flow [kg/s].
    ///
    /// At higher load and flow rates, gas velocity biases propagation delays
    /// downstream at `c + u` and upstream at `c - u`.
    pub fn set_mean_flow(&mut self, mass_flow: f32, gamma: f32, gas_constant: f32) {
        let n_cyl = self.primaries.len().max(1);
        let cyl_flow = mass_flow.max(0.0) / n_cyl as f32;
        for p in &mut self.primaries {
            let mach = mean_flow_mach(cyl_flow, p.area(), gamma, gas_constant, p.temperature);
            p.set_mach(mach);
        }
        let bank_flow = mass_flow.max(0.0) / self.bank_count.max(1) as f32;
        for (tp, mouth) in self.tailpipes.iter_mut().zip(self.mouths.iter_mut()) {
            let mach = mean_flow_mach(bank_flow, tp.area(), gamma, gas_constant, tp.temperature);
            tp.set_mach(mach);
            // The same gas leaves through the mouth, which is what decides how
            // much of the reflection the jet takes: see
            // [`vortex_reflection_factor`](crate::audio::radiation::vortex_reflection_factor).
            // Without this the network is very nearly lossless below the
            // radiation corner and a single blowdown rings for seconds.
            mouth.set_mach(mach * tp.area() / mouth.area());
        }
    }

    /// Number of banks the network radiates from.
    pub fn bank_count(&self) -> usize {
        self.bank_count
    }

    /// Sets whether the exhaust cutout bypass junction is open.
    pub fn set_cutout(&mut self, open: bool) {
        self.cutout_open = open;
        for tp in &mut self.tailpipes {
            tp.set_steepening(open);
        }
    }

    /// Returns whether the exhaust cutout is currently open.
    pub fn is_cutout_open(&self) -> bool {
        self.cutout_open
    }

    /// One-way transit delay of a cylinder's primary [samples].
    pub fn primary_delay_samples(&self, cylinder: usize) -> f32 {
        self.primaries[cylinder.min(self.primaries.len() - 1)].delay_samples()
    }

    /// Acoustic round-trip time of a cylinder's primary [s].
    ///
    /// This is the `tau_pulse` of the Transit-Time Decision Rule, read back from
    /// the delay the pipe is actually using rather than recomputed from
    /// temperature, so it follows the glide instead of leading it.
    pub fn primary_round_trip_seconds(&self, cylinder: usize) -> f32 {
        let i = cylinder.min(self.primaries.len() - 1);
        2.0 * self.primaries[i].transit_time_seconds()
    }

    /// One-way transit delay of a bank's collector taper [samples].
    pub fn collector_delay_samples(&self, bank: usize) -> f32 {
        self.collectors[bank.min(self.collectors.len() - 1)].taper_delay_samples()
    }

    /// One-way transit delay of a bank's tailpipe [samples].
    pub fn tailpipe_delay_samples(&self, bank: usize) -> f32 {
        self.tailpipes[bank.min(self.tailpipes.len() - 1)].delay_samples()
    }

    /// Mean acoustic round-trip time of a bank's primaries [s].
    ///
    /// A bank no longer has *a* runner — it has one primary per cylinder, and on
    /// an unequal-length header no two of them agree. This is the average, which
    /// is the quantity a per-bank lumped model was ever standing in for.
    pub fn bank_mean_round_trip_seconds(&self, bank: usize) -> f32 {
        let cyls = match self.bank_cylinders.get(bank) {
            Some(cyls) if !cyls.is_empty() => cyls,
            _ => return 0.0,
        };
        let total: f32 = cyls
            .iter()
            .map(|&i| 2.0 * self.primaries[i].transit_time_seconds())
            .sum();
        total / cyls.len() as f32
    }

    /// Magnitude of the reflection at a cylinder's valve end at `hz` [-].
    ///
    /// Takes a frequency because the answer is a function of one: the cylinder
    /// behind an open valve reflects the bottom of the band and absorbs its own
    /// Helmholtz band, and a single number for "the reflection" would have to
    /// pick one of those and call it the other.
    pub fn valve_reflection_at(&self, cylinder: usize, hz: f32) -> f32 {
        self.valves[cylinder.min(self.valves.len() - 1)].reflection_at(hz)
    }

    /// Frequency the cylinder behind a valve resonates at [Hz].
    pub fn valve_helmholtz_hz(&self, cylinder: usize) -> f32 {
        self.valves[cylinder.min(self.valves.len() - 1)].helmholtz_hz()
    }

    /// Sets the cylinder each valve opens into, for all cylinders.
    ///
    /// Areas [m^2], volumes [m^3] and port mass flows [kg/s], one per cylinder
    /// and in the same order the taps were declared.
    pub fn set_valve_loads(&mut self, areas: &[f64], volumes: &[f64], flows: &[f64]) {
        for (i, v) in self.valves.iter_mut().enumerate() {
            v.set_load(
                areas.get(i).copied().unwrap_or(0.0),
                volumes.get(i).copied().unwrap_or(1e-4),
                flows.get(i).copied().unwrap_or(0.0),
            );
        }
    }

    /// Hands a bank's upstream-travelling wave back toward its collector: down
    /// the pre-crossover pipe where the geometry gave one, and through the
    /// one-sample connector where it did not.
    #[inline(always)]
    fn push_upstream(&mut self, bank: usize, upstream: f32, crossed: bool) {
        if crossed {
            let forward = self.bank_trans[bank];
            self.pre_cross_pipes[bank].push_inputs(forward, upstream);
        } else {
            self.collector_returns[bank] = upstream;
        }
    }

    /// Steps the entire waveguide network by one audio sample:
    /// - `excitations`: blowdown pressure pulse injected at each cylinder's port.
    /// - `bank_excitations`: pressure released inside each bank's collector — an
    ///   exhaust backfire is unburnt fuel lighting off in the pipework, not a
    ///   cylinder event, so it belongs at the junction and not at a valve.
    /// - `radiated`: filled with the pressure radiated from each bank's mouth.
    ///
    /// A bank is a tailpipe, so the caller gets one radiated signal per bank and
    /// decides where each one sits in the image. Imaging is not the network's
    /// business, and folding it to stereo here would silently drop the outer
    /// banks of anything with more than two.
    #[inline(always)]
    pub fn step(&mut self, excitations: &[f32], bank_excitations: &[f32], radiated: &mut [f32]) {
        let n_cyl = self.primaries.len();
        let crossed = !self.pre_cross_pipes.is_empty();

        // 1. Read the pipes running from the collectors to the crossover first,
        //    so a wave coming back up arrives at the collector with that pipe's
        //    own transit time rather than an invented one.
        if crossed {
            for b in 0..self.bank_count {
                let (up, down) = self.pre_cross_pipes[b].read_outputs();
                self.pre_cross_up[b] = up;
                self.pre_cross_down[b] = down;
            }
        }

        // 2. Read outputs from primaries and apply valve boundaries
        for i in 0..n_cyl {
            let (p_at_valve, p_at_coll) = self.primaries[i].read_outputs();
            let excit = if i < excitations.len() {
                excitations[i]
            } else {
                0.0
            };
            self.prim_in_0[i] = self.valves[i].step(excit, p_at_valve);
            self.prim_to_collector[i] = p_at_coll;
        }

        // 3. Step collectors for each bank
        for b in 0..self.bank_count {
            let cyls = &self.bank_cylinders[b];
            for (k, &cyl_idx) in cyls.iter().enumerate() {
                self.bank_prim_in[b][k] = self.prim_to_collector[cyl_idx];
            }

            let downstream_refl = if crossed {
                self.pre_cross_up[b]
            } else {
                self.collector_returns[b]
            };
            let excitation = bank_excitations.get(b).copied().unwrap_or(0.0);
            self.bank_trans[b] = self.collectors[b].step(
                &self.bank_prim_in[b],
                excitation,
                downstream_refl,
                &mut self.bank_prim_refl[b],
            );

            for (k, &cyl_idx) in cyls.iter().enumerate() {
                let p_in_1 = self.bank_prim_refl[b][k];
                self.primaries[cyl_idx].push_inputs(self.prim_in_0[cyl_idx], p_in_1);
            }
        }

        // 4. Crossover. It links the first two banks; anything beyond them — the
        //    outer banks of a W engine — has no partner to cross with and runs
        //    straight through. Either way the wave arrives through the
        //    pre-crossover pipe when the geometry described one.
        let linked = if self.bank_count > 1 { 2 } else { 0 };
        if linked == 2 {
            let (in0, in1) = if crossed {
                (self.pre_cross_down[0], self.pre_cross_down[1])
            } else {
                (self.bank_trans[0], self.bank_trans[1])
            };
            let (b0_up, b1_up, b0_down, b1_down) =
                self.crossover
                    .step(in0, in1, self.chain_returns[0][0], self.chain_returns[1][0]);
            self.bank_down[0] = b0_down;
            self.bank_down[1] = b1_down;
            self.push_upstream(0, b0_up, crossed);
            self.push_upstream(1, b1_up, crossed);
        }
        for b in linked..self.bank_count {
            self.bank_down[b] = if crossed {
                self.pre_cross_down[b]
            } else {
                self.bank_trans[b]
            };
            let upstream = self.chain_returns[b][0];
            self.push_upstream(b, upstream, crossed);
        }

        // 5. Silencer chain, tailpipe and mouth. Each element hands its upstream
        //    reflection to the interface above it, so what a silencer or the open
        //    mouth sends back reaches the collector and the primaries beyond it.
        //    When the cutout is open, pulses bypass the silencer chain via the
        //    bypass junction straight into the tailpipe.
        for b in 0..self.bank_count {
            let (p_tail_up, p_tail_exit) = self.tailpipes[b].read_outputs();
            let (p_mouth_refl, p_mouth_rad) = self.mouths[b].step(p_tail_exit);

            if self.cutout_open {
                let sig = self.bank_down[b];
                self.tailpipes[b].push_inputs(sig, p_mouth_refl);
                self.chain_returns[b][0] = p_tail_up;
            } else {
                let mut sig = self.bank_down[b];
                let n_sil = self.silencers[b].len();
                for k in 0..n_sil {
                    let downstream = self.chain_returns[b][k + 1];
                    let (upstream, trans) = self.silencers[b][k].step(sig, downstream);
                    self.chain_returns[b][k] = upstream;
                    sig = trans;
                }
                self.tailpipes[b].push_inputs(sig, p_mouth_refl);
                self.chain_returns[b][n_sil] = p_tail_up;
            }

            if let Some(slot) = radiated.get_mut(b) {
                *slot = p_mouth_rad;
            }
        }
    }

    /// Clears internal state across the entire network.
    pub fn reset(&mut self) {
        for p in &mut self.primaries {
            p.reset();
        }
        for v in &mut self.valves {
            v.reset();
        }
        for coll in &mut self.collectors {
            coll.reset();
        }
        self.crossover.reset();
        for p in &mut self.pre_cross_pipes {
            p.reset();
        }
        for chain in &mut self.silencers {
            for element in chain {
                element.reset();
            }
        }
        for tp in &mut self.tailpipes {
            tp.reset();
        }
        for m in &mut self.mouths {
            m.reset();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Amplitude of `signal` at `frequency`, by Goertzel-style projection.
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
    fn wall_loss_follows_alpha_across_the_band() {
        // The claim the cascade exists to keep: one pass down a pipe costs
        // `alpha(f) L` nepers, and `alpha` goes as the square root of
        // frequency. A first-order lowpass cannot hold that shape over three
        // decades — fitted at its own -3 dB point it leaves the whole audible
        // band below the corner attenuated by almost nothing and buries
        // everything above it at 6 dB an octave. Three shelves hold it to a
        // fraction of a decibel.
        const FS: f32 = 48_000.0;
        const GAMMA: f32 = 1.33;
        const R: f32 = 287.0;

        for (radius, length, temperature) in [
            (0.022f32, 0.55f32, 900.0f32), // a V8 primary
            (0.030, 1.50, 600.0),          // its tailpipe
            (0.019, 0.40, 1_100.0),        // a four-cylinder's, hotter and narrower
        ] {
            let c = speed_of_sound(GAMMA, R, temperature);
            let mut loss = ViscothermalLoss::new(radius, length, c, GAMMA, R, temperature, FS);

            // The analytic target, straight from the formula in Appendix A.
            let nu = kinematic_viscosity(temperature, R) * BOUNDARY_LAYER_TURBULENCE_FACTOR;
            let analytic_db = |f: f64| {
                let alpha = viscothermal_alpha(
                    f,
                    radius as f64,
                    c as f64,
                    nu as f64,
                    GAMMA as f64,
                    EXHAUST_PRANDTL_NUMBER as f64,
                );
                alpha * length as f64 * 8.685_889_638_065_035
            };

            // Checked to 8 kHz and no further: a 60 mm duct cuts its first
            // cross mode on near 5 kHz, above which a one-dimensional
            // waveguide has nothing to claim and the mouth's own
            // [`DUCT_CUT_ON`] rolls the band away regardless. The cascade is
            // 2.5 dB shy of `alpha` at 16 kHz and that is not worth a fourth
            // shelf to fix.
            for f in [31.0f32, 125.0, 500.0, 2_000.0, 8_000.0] {
                // What the filter claims, against the formula.
                let claimed = loss.attenuation_db(f) as f64;
                let wanted = analytic_db(f as f64);
                assert!(
                    (claimed - wanted).abs() < 1e-3,
                    "{radius} m x {length} m at {f} Hz: slope says {claimed:.4} dB, \
                     alpha says {wanted:.4} dB"
                );

                // And what it actually does, against what it claims.
                let (mut re, mut im) = (0.0f64, 0.0f64);
                let (settle, measure) = (24_000usize, 24_000usize);
                for i in 0..settle + measure {
                    let phase = std::f32::consts::TAU * f * i as f32 / FS;
                    let y = loss.process(phase.sin());
                    if i >= settle {
                        re += y as f64 * phase.sin() as f64;
                        im += y as f64 * phase.cos() as f64;
                    }
                }
                let gain = 2.0 * (re * re + im * im).sqrt() / measure as f64;
                let measured_db = -20.0 * gain.max(1e-12).log10();
                assert!(
                    (measured_db - wanted).abs() < 1.5,
                    "{radius} m x {length} m at {f} Hz: {measured_db:.2} dB measured, \
                     {wanted:.2} dB from alpha"
                );
            }
        }
    }

    #[test]
    fn cold_air_is_less_viscous_than_hot_exhaust() {
        // Sutherland over the ideal gas. The number matters because the wall
        // loss goes as its square root, so one figure for the whole synth damps
        // a cold intake runner two and a half times too hard.
        let cold = kinematic_viscosity(300.0, 287.0);
        let hot = kinematic_viscosity(900.0, 287.0);
        assert!(
            (cold - 1.57e-5).abs() < 1.0e-6,
            "air at 300 K should be near 1.57e-5 m^2/s, got {cold:e}"
        );
        assert!(
            (hot - 9.9e-5).abs() < 5.0e-6,
            "exhaust at 900 K should be near 9.9e-5 m^2/s, got {hot:e}"
        );
        // And the attenuation follows the square root of it.
        let ratio = (hot / cold).sqrt();
        let c = 400.0;
        let slope_cold = loss_slope_db(0.02, 0.5, c, 1.4, 287.0, 300.0, SMOOTH_WALL);
        let slope_hot = loss_slope_db(0.02, 0.5, c, 1.4, 287.0, 900.0, SMOOTH_WALL);
        assert!(
            ((slope_hot / slope_cold) - ratio).abs() < 0.02,
            "attenuation did not follow sqrt(nu): {:.3} against {ratio:.3}",
            slope_hot / slope_cold
        );
    }

    #[test]
    fn a_hot_tailpipe_attenuates_the_way_a_measured_one_does() {
        // The calibration [`BOUNDARY_LAYER_TURBULENCE_FACTOR`] answers to.
        // A 60 mm automotive tailpipe running hot loses something like 0.2 to
        // 0.6 dB of a 250 Hz to 1 kHz wave per metre. That band and that pipe
        // are where an exhaust note is decided, and a constant that puts the
        // loss outside it takes the note's harmonics with it — which is what
        // sixteen did, at three to four times the top of the range.
        let (radius, length, gamma, gas_constant, temperature) = (0.030, 1.5, 1.33, 287.0, 800.0);
        let c = speed_of_sound(gamma, gas_constant, temperature);
        let loss = ViscothermalLoss::new(
            radius,
            length,
            c,
            gamma,
            gas_constant,
            temperature,
            48_000.0,
        );
        for hz in [250.0, 500.0, 1_000.0] {
            let db_per_m = loss.attenuation_db(hz) / length;
            assert!(
                (0.2..=0.6).contains(&db_per_m),
                "a hot tailpipe took {db_per_m:.3} dB/m out of {hz:.0} Hz; measurement says 0.2 to 0.6"
            );
        }
    }

    #[test]
    fn a_smooth_wall_loses_half_of_what_a_rough_one_does() {
        // The enhancement multiplies the viscosity, so it is worth its square
        // root in attenuation: an intake tract that declares itself smooth is
        // less lossy than a header at the same size and gas, by exactly the
        // root of what it declares.
        let c = 343.0;
        let mut rough = ViscothermalLoss::new(0.021, 0.25, c, 1.4, 287.0, 300.0, 48_000.0);
        let mut smooth = ViscothermalLoss::new(0.021, 0.25, c, 1.4, 287.0, 300.0, 48_000.0);
        smooth.set_wall_enhancement(SMOOTH_WALL);
        smooth.tune(0.021, 0.25, c, 1.4, 287.0, 300.0, 48_000.0);
        rough.tune(0.021, 0.25, c, 1.4, 287.0, 300.0, 48_000.0);
        let ratio = rough.slope_db() / smooth.slope_db();
        assert!(
            (ratio - BOUNDARY_LAYER_TURBULENCE_FACTOR.sqrt()).abs() < 0.01,
            "enhancement bought {ratio:.3} in attenuation, not its own square root"
        );
    }

    #[test]
    fn closed_open_pipe_resonates_at_c_over_four_l() {
        // A pipe shut at one end and open at the other is a quarter-wave
        // resonator: the closed end forces a pressure antinode, the open end a
        // node, and the lowest mode that fits is f = c / 4L. This is the single
        // claim the whole network rests on — every tuned length in an exhaust is
        // some version of it.
        const FS: f32 = 48_000.0;
        const GAMMA: f32 = 1.4;
        const R: f32 = 287.0;
        const TEMPERATURE: f32 = 300.0;

        for (length, radius) in [(0.5f64, 0.025f64), (0.9, 0.030), (0.35, 0.020)] {
            let area = std::f64::consts::PI * radius * radius;
            let mouth = Mouth::new(
                FS,
                radius as f32,
                false,
                speed_of_sound(GAMMA, R, TEMPERATURE),
            );
            // The open end acts as if the pipe ran on past its edge; the network
            // adds the same correction, so the resonance is set by the effective
            // length rather than the machined one.
            let effective = length + mouth.end_correction() as f64;
            let mut pipe = WaveguidePipe::new(effective, area, FS, GAMMA, R, TEMPERATURE);
            pipe.set_boundary_phase_delay(mouth.phase_delay_samples());
            pipe.tune(GAMMA, R, TEMPERATURE);
            // The delay smoother glides over 40 ms and this pipe rings for
            // about 50, so an impulse fired the instant after a retune measures
            // the glide as much as the pipe. A running network is always
            // settled; put the test in the same state.
            pipe.snap_delays();
            let mut mouth = mouth;

            // Impulse in at the closed end, radiated pressure out at the mouth.
            let mut radiated = vec![0.0f32; 1 << 16];
            for (i, out) in radiated.iter_mut().enumerate() {
                let (p_at_closed, p_at_mouth) = pipe.read_outputs();
                let (p_reflected, p_rad) = mouth.step(p_at_mouth);
                // A rigid closed end reflects in phase: r = +1.
                let excitation = if i == 0 { 1.0 } else { 0.0 };
                pipe.push_inputs(excitation + p_at_closed, p_reflected);
                *out = p_rad;
            }

            let c = speed_of_sound(GAMMA, R, TEMPERATURE);
            let expected = c / (4.0 * effective as f32);

            // Sweep a band around the prediction and take the peak.
            let mut best = (0.0f32, 0.0f32);
            let mut f = expected * 0.7;
            while f <= expected * 1.3 {
                let m = magnitude_at(&radiated[1..], f, FS);
                if m > best.1 {
                    best = (f, m);
                }
                f += 0.05;
            }

            let error = (best.0 - expected).abs() / expected;
            assert!(
                error < 0.01,
                "L = {length} m: resonance at {:.2} Hz, expected {:.2} Hz ({:.2} % off)",
                best.0,
                expected,
                error * 100.0
            );
        }
    }

    #[test]
    fn fundamental_shifts_with_mach_number() {
        // Convective mean flow biases forward and backward acoustic wave travel:
        // waves move downstream at c + u = c(1 + M) and upstream at c - u = c(1 - M).
        // For a closed-open pipe (quarter-wave resonator), the round trip period is:
        //   T = 2 * (L / (c + u) + L / (c - u)) = 4L / (c * (1 - M^2))
        // So the fundamental mode frequency shifts as:
        //   f_1 = c * (1 - M^2) / (4L)
        // in both forward (M > 0) and backward (M < 0) directions.
        const FS: f32 = 48_000.0;
        const GAMMA: f32 = 1.4;
        const R: f32 = 287.0;
        const TEMPERATURE: f32 = 300.0;
        let c = speed_of_sound(GAMMA, R, TEMPERATURE);

        let length = 0.5f64;
        let radius = 0.025f64;
        let area = std::f64::consts::PI * radius * radius;
        let mouth = Mouth::new(FS, radius as f32, false, c);
        let effective = length + mouth.end_correction() as f64;

        // Test with M = 0.20 and M = -0.20 (in both directions)
        for mach in [0.20f32, -0.20f32] {
            let mut pipe = WaveguidePipe::new(effective, area, FS, GAMMA, R, TEMPERATURE);
            pipe.set_boundary_phase_delay(mouth.phase_delay_samples());
            pipe.tune(GAMMA, R, TEMPERATURE);
            pipe.set_mach(mach);
            pipe.snap_delays();
            let mut mouth = mouth;

            // Check that forward and backward delays are asymmetric:
            let tau0 = (effective as f32) / c;
            let expected_fwd = tau0 / (1.0 + mach);
            let expected_bwd = tau0 / (1.0 - mach);
            let actual_fwd = pipe.forward_delay_samples() / FS;
            let actual_bwd = pipe.backward_delay_samples() / FS;
            assert!(
                (actual_fwd - expected_fwd).abs() / expected_fwd < 0.02,
                "Forward transit delay mismatch: actual {actual_fwd:.5}, expected {expected_fwd:.5}"
            );
            assert!(
                (actual_bwd - expected_bwd).abs() / expected_bwd < 0.02,
                "Backward transit delay mismatch: actual {actual_bwd:.5}, expected {expected_bwd:.5}"
            );
            assert!(
                (pipe.forward_delay_samples() - pipe.backward_delay_samples()).abs() > 2.0,
                "Forward and backward delays must be asymmetric under mean flow"
            );

            // Impulse response measurement
            let mut radiated = vec![0.0f32; 1 << 16];
            for (i, out) in radiated.iter_mut().enumerate() {
                let (p_at_closed, p_at_mouth) = pipe.read_outputs();
                let (p_reflected, p_rad) = mouth.step(p_at_mouth);
                let excitation = if i == 0 { 1.0 } else { 0.0 };
                pipe.push_inputs(excitation + p_at_closed, p_reflected);
                *out = p_rad;
            }

            let expected_f1 = c * (1.0 - mach * mach) / (4.0 * effective as f32);
            let mut best = (0.0f32, 0.0f32);
            let mut f = expected_f1 * 0.7;
            while f <= expected_f1 * 1.3 {
                let m = magnitude_at(&radiated[1..], f, FS);
                if m > best.1 {
                    best = (f, m);
                }
                f += 0.05;
            }

            let error = (best.0 - expected_f1).abs() / expected_f1;
            assert!(
                error < 0.01,
                "Mach = {mach}: resonance at {:.2} Hz, expected {:.2} Hz ({:.2} % off)",
                best.0,
                expected_f1,
                error * 100.0
            );
        }
    }

    #[test]
    fn pipe_with_hot_end_and_cold_end_resonates_between_bulk_predictions() {
        // A pipe that runs hot at the port (~1000 K) and cool at the tailpipe (~400 K)
        // has a sound speed that varies along its length. Its acoustic travel time
        // is the sum of the travel times through the hot and cool zones, so its
        // fundamental resonance must sit strictly between the two bulk predictions:
        //   f_cold < f_resonance < f_hot
        // and cannot land at either.
        const FS: f32 = 48_000.0;
        const GAMMA: f32 = 1.4;
        const R: f32 = 287.0;
        const T_HOT: f32 = 1000.0;
        const T_COLD: f32 = 400.0;

        let c_hot = speed_of_sound(GAMMA, R, T_HOT);
        let c_cold = speed_of_sound(GAMMA, R, T_COLD);

        let l1 = 0.40f64; // hot section
        let l2 = 0.40f64; // cold section
        let radius = 0.025f64;
        let area = std::f64::consts::PI * radius * radius;

        let mouth = Mouth::new(FS, radius as f32, false, c_cold);
        let eff_l2 = l2 + mouth.end_correction() as f64;
        let total_effective = l1 + eff_l2;

        let f_bulk_hot = c_hot / (4.0 * total_effective as f32);
        let f_bulk_cold = c_cold / (4.0 * total_effective as f32);

        let mut pipe_hot = WaveguidePipe::new(l1, area, FS, GAMMA, R, T_HOT);
        let mut pipe_cold = WaveguidePipe::new(eff_l2, area, FS, GAMMA, R, T_COLD);
        pipe_cold.set_boundary_phase_delay(mouth.phase_delay_samples());
        pipe_cold.tune(GAMMA, R, T_COLD);
        pipe_hot.snap_delays();
        pipe_cold.snap_delays();
        let junction = ScatteringJunction::new(&[pipe_hot.admittance(), pipe_cold.admittance()]);
        let mut mouth = mouth;

        let mut j_in = [0.0f32; 2];
        let mut j_out = [0.0f32; 2];

        let mut radiated = vec![0.0f32; 1 << 16];
        for (i, out) in radiated.iter_mut().enumerate() {
            let (p_at_closed, p1_to_j) = pipe_hot.read_outputs();
            let (p2_to_j, p2_to_mouth) = pipe_cold.read_outputs();

            j_in[0] = p1_to_j;
            j_in[1] = p2_to_j;
            junction.scatter(&j_in, &mut j_out);

            let (p_reflected, p_rad) = mouth.step(p2_to_mouth);

            let excitation = if i == 0 { 1.0 } else { 0.0 };
            pipe_hot.push_inputs(excitation + p_at_closed, j_out[0]);
            pipe_cold.push_inputs(j_out[1], p_reflected);
            *out = p_rad;
        }

        let mut best = (0.0f32, 0.0f32);
        let mut f = f_bulk_cold * 0.9;
        while f <= f_bulk_hot * 1.1 {
            let m = magnitude_at(&radiated[1..], f, FS);
            if m > best.1 {
                best = (f, m);
            }
            f += 0.05;
        }

        let measured_f = best.0;
        assert!(
            measured_f > f_bulk_cold + 2.0,
            "Resonance ({measured_f:.1} Hz) must be strictly above cold prediction ({f_bulk_cold:.1} Hz)"
        );
        assert!(
            measured_f < f_bulk_hot - 10.0,
            "Resonance ({measured_f:.1} Hz) must be strictly below hot prediction ({f_bulk_hot:.1} Hz)"
        );
    }

    #[test]
    fn chamber_transmission_loss_matches_theory() {
        // A single-expansion chamber terminated anechoically has a closed-form
        // transmission loss that depends only on the area ratio and how many
        // wavelengths fit the cavity:
        //
        //   TL = 10 log10[ 1 + (1/4) (m - 1/m)^2 sin^2(kL) ]
        //
        // It is zero whenever kL is a multiple of pi — the chamber is
        // transparent at those frequencies, which is exactly why a single
        // chamber cannot silence an engine on its own — and peaks at the
        // quarter-wave points. Nothing in it is tunable, so it is a real check
        // that the two area steps and the pipe between them scatter correctly.
        const FS: f32 = 48_000.0;
        const GAMMA: f32 = 1.4;
        const R: f32 = 287.0;
        const TEMPERATURE: f32 = 300.0;

        let c = speed_of_sound(GAMMA, R, TEMPERATURE);
        let pipe_area = std::f64::consts::PI * 0.030 * 0.030;
        let cavity_length = 0.30f64;
        let area_ratio = 4.0f64;

        let analytic = |f: f32| {
            let k = std::f32::consts::TAU * f / c;
            let m = area_ratio as f32;
            let s = (k * cavity_length as f32).sin();
            10.0 * (1.0 + 0.25 * (m - 1.0 / m).powi(2) * s * s).log10()
        };

        // Quarter-wave point (peak loss), half-wave point (transparent), and a
        // frequency between the two.
        let quarter = c / (4.0 * cavity_length as f32);
        for f in [quarter, 2.0 * quarter, 0.5 * quarter, 1.5 * quarter] {
            let mut chamber = ExpansionChamber::new(
                pipe_area,
                area_ratio,
                cavity_length,
                FS,
                GAMMA,
                R,
                TEMPERATURE,
            );

            // Drive with a sine and read the transmitted wave. Handing the
            // chamber a zero backward wave from downstream *is* the anechoic
            // termination the analytic result assumes.
            let settle = 24_000;
            let measure = 24_000;
            let mut transmitted = vec![0.0f32; measure];
            for i in 0..(settle + measure) {
                let phase = std::f32::consts::TAU * f * i as f32 / FS;
                let (_, out) = chamber.step(phase.sin(), 0.0);
                if i >= settle {
                    transmitted[i - settle] = out;
                }
            }

            let amplitude = magnitude_at(&transmitted, f, FS);
            let measured = -20.0 * amplitude.max(1e-9).log10();
            let expected = analytic(f);

            assert!(
                (measured - expected).abs() < 1.0,
                "at {f:.0} Hz (kL = {:.2} rad): {measured:.2} dB measured, {expected:.2} dB from theory",
                std::f32::consts::TAU * f / c * cavity_length as f32
            );
        }
    }

    #[test]
    fn packed_absorption_rises_with_frequency() {
        // The claim that separates a muffler from a volume knob: packing takes
        // the top out and leaves the bottom alone. Fibre dissipates by dragging
        // gas through itself, and the gas is still within a quarter wavelength
        // of the shell, so a layer of depth t is transparent below
        // c_packing / 4t and fully effective above it.
        //
        // Modelled as the shelf that argument implies,
        //
        //   |H(x)|^2 = (1 + p^2 x^2) / (1 + x^2),   x = f / f_q
        //
        // which is unity at DC and the packing's rated gain p far above the
        // corner. The core is given the same area as the pipe either side of
        // it, so the two area steps are transparent and what is left to measure
        // is the packing alone.
        const FS: f32 = 48_000.0;
        const GAMMA: f32 = 1.4;
        const R: f32 = 287.0;
        const TEMPERATURE: f32 = 300.0;

        let area = std::f64::consts::PI * 0.030 * 0.030;
        let length = 0.50f64;
        let thickness = 0.035f64;
        let loss_db_per_m = 20.0f64;

        let c = speed_of_sound(GAMMA, R, TEMPERATURE);
        let corner = packing_corner_hz(thickness, c);
        let pass = 10f64.powf(-loss_db_per_m * length / 20.0) as f32;

        let shelf_loss = |f: f32| {
            let x = f / corner;
            -10.0 * ((1.0 + pass * pass * x * x) / (1.0 + x * x)).log10()
        };
        // The core is a duct like any other and its wall takes its own
        // `sqrt(f)` out of whatever passes through. That is not the packing, so
        // it is predicted separately and charged separately rather than left to
        // contaminate the shelf it would otherwise be mistaken for.
        let wall_loss = |f: f32| {
            loss_slope_db(
                0.030,
                length as f32,
                c,
                GAMMA,
                R,
                TEMPERATURE,
                BOUNDARY_LAYER_TURBULENCE_FACTOR,
            ) * f.max(0.0).sqrt()
        };
        let analytic_loss = |f: f32| shelf_loss(f) + wall_loss(f);

        let measured_loss = |f: f32| {
            let mut silencer = AbsorptiveSilencer::new(
                area,
                area,
                length,
                thickness,
                loss_db_per_m,
                FS,
                GAMMA,
                R,
                TEMPERATURE,
            );
            let (settle, measure) = (24_000, 24_000);
            let mut transmitted = vec![0.0f32; measure];
            for i in 0..(settle + measure) {
                let phase = std::f32::consts::TAU * f * i as f32 / FS;
                // A zero backward wave from downstream is the anechoic
                // termination, as in the chamber test above.
                let (_, out) = silencer.step(phase.sin(), 0.0);
                if i >= settle {
                    transmitted[i - settle] = out;
                }
            }
            -20.0 * magnitude_at(&transmitted, f, FS).max(1e-9).log10()
        };

        // Transparent an octave and a half below the corner, working hard two
        // octaves above it, and never going backwards in between. The upper
        // bound on the low point is the whole difference from a flat gain,
        // which would have taken all 10 dB out down here as well.
        let low = measured_loss(corner / 8.0);
        let mid = measured_loss(corner);
        let high = measured_loss(4.0 * corner);
        let packing_only = low - wall_loss(corner / 8.0);
        assert!(
            packing_only < 0.5,
            "packing is not transparent below its corner: {packing_only:.2} dB at {:.0} Hz,              beyond the {:.2} dB the core's own wall takes",
            corner / 8.0,
            wall_loss(corner / 8.0)
        );
        assert!(
            high > 7.0,
            "packing is not working above its corner: {high:.2} dB at {:.0} Hz",
            4.0 * corner
        );
        assert!(
            low < mid && mid < high,
            "absorption did not rise with frequency: {low:.2}, {mid:.2}, {high:.2} dB"
        );

        // And it follows the shelf, not merely the right direction. The core's
        // wall loss is in the prediction now rather than excused from it, so
        // what is left over is the shelf's own discretisation.
        for f in [corner / 8.0, corner / 2.0, corner, 2.0 * corner] {
            let (measured, expected) = (measured_loss(f), analytic_loss(f));
            assert!(
                (measured - expected).abs() < 0.5,
                "at {f:.0} Hz (f/f_q = {:.2}): {measured:.2} dB measured, {expected:.2} dB from the shelf",
                f / corner
            );
        }
    }

    #[test]
    fn crossover_transfers_energy_between_banks() {
        // What separates a flat-plane V8 from a cross-plane one at equal firing
        // order is whether the banks can hear each other. Fire only bank 0 and
        // listen at bank 1's tailpipe: with an X-pipe, energy arrives; with
        // `Crossover::None` the banks are two separate exhausts and nothing
        // does. Nothing here is a mixing coefficient — the transfer is whatever
        // the 4-port junction scatters.
        use crate::physics::plumbing::{
            Collector, Crossover, ExhaustSystem, PipeSection, Silencer,
        };

        const FS: f32 = 48_000.0;

        let system = |crossover: Crossover| ExhaustSystem {
            primaries: vec![PipeSection::from_diameter(0.45, 0.040, 850.0); 8],
            collector: Collector::from_diameter(4, 0.060, 0.15),
            secondary: vec![],
            crossover,
            silencers: vec![Silencer::Straight],
            tailpipe: PipeSection::from_diameter(1.0, 0.060, 600.0),
            tailpipe_flanged: false,
            cutout: false,
        };

        // A cross-plane V8's banks: cylinders 0, 2, 3, 7 on one, the rest on the other.
        let cylinders: Vec<crate::audio::dsp::CylinderTap> = [0, 1, 0, 0, 1, 1, 1, 0]
            .iter()
            .enumerate()
            .map(|(i, &bank)| crate::audio::dsp::CylinderTap {
                evo_phase: i as f32 / 8.0,
                bank,
            })
            .collect();

        let snapshot = crate::audio::dsp::EngineSnapshot::default();
        let far_bank_energy = |crossover: Crossover| {
            let exhaust = system(crossover);
            let mut network = ExhaustNetwork::new(&exhaust, &cylinders, 2, FS, &snapshot);
            let mut excitations = vec![0.0f32; cylinders.len()];
            let bank_excitations = vec![0.0f32; 2];
            let mut radiated = vec![0.0f32; 2];

            let mut near = 0.0f64;
            let mut far = 0.0f64;
            for i in 0..48_000 {
                // Impulse into one cylinder of bank 0 only. Every other port
                // stays silent, so anything at bank 1's mouth crossed over.
                excitations.fill(0.0);
                if i == 0 {
                    excitations[0] = 1.0;
                }
                network.step(&excitations, &bank_excitations, &mut radiated);
                near += (radiated[0] as f64).powi(2);
                far += (radiated[1] as f64).powi(2);
            }
            (near, far)
        };

        let (isolated_near, isolated_far) = far_bank_energy(Crossover::None);
        let (crossed_near, crossed_far) = far_bank_energy(Crossover::XPipe { position: 0.80 });

        assert!(
            isolated_near > 0.0 && crossed_near > 0.0,
            "the fired bank radiated nothing at all"
        );
        assert_eq!(
            isolated_far, 0.0,
            "energy reached the far bank with no crossover fitted: {isolated_far:e}"
        );
        assert!(
            crossed_far > 0.05 * crossed_near,
            "the X-pipe passed almost nothing across: {crossed_far:e} against {crossed_near:e} on the fired bank"
        );
    }

    #[test]
    fn stub_notches_at_c_over_four_l_stub() {
        // A closed side branch is a drone killer: the round trip up and back
        // covers half a wavelength at f = c / 4L, and the rigid end reflects in
        // phase, so what returns to the junction arrives inverted and cancels.
        // The notch frequency is set by the branch length and nothing else.
        const FS: f32 = 48_000.0;
        const GAMMA: f32 = 1.4;
        const R: f32 = 287.0;
        const TEMPERATURE: f32 = 300.0;

        let c = speed_of_sound(GAMMA, R, TEMPERATURE);
        let pipe_area = std::f64::consts::PI * 0.030 * 0.030;
        let stub_area = std::f64::consts::PI * 0.020 * 0.020;

        for stub_length in [0.25f64, 0.40] {
            let expected = c / (4.0 * stub_length as f32);

            // Sweep the band around the prediction and find the deepest point.
            let mut deepest = (0.0f32, f32::MAX);
            let mut f = expected * 0.75;
            while f <= expected * 1.25 {
                let mut stub = QuarterWaveStub::new(
                    pipe_area,
                    stub_area,
                    stub_length,
                    FS,
                    GAMMA,
                    R,
                    TEMPERATURE,
                );
                let settle = 24_000;
                let measure = 24_000;
                let mut transmitted = vec![0.0f32; measure];
                for i in 0..(settle + measure) {
                    let phase = std::f32::consts::TAU * f * i as f32 / FS;
                    let (_, out) = stub.step(phase.sin(), 0.0);
                    if i >= settle {
                        transmitted[i - settle] = out;
                    }
                }
                let m = magnitude_at(&transmitted, f, FS);
                if m < deepest.1 {
                    deepest = (f, m);
                }
                f += expected * 0.002;
            }

            let error = (deepest.0 - expected).abs() / expected;
            assert!(
                error < 0.02,
                "stub of {stub_length} m notched at {:.1} Hz, expected {:.1} Hz ({:.1} % off)",
                deepest.0,
                expected,
                error * 100.0
            );
            assert!(
                deepest.1 < 0.5,
                "the notch is not a notch: {:.3} of the drive still gets through",
                deepest.1
            );
        }
    }

    #[test]
    fn a_narrow_pipe_is_duller_than_a_wide_one() {
        // Wall loss goes as sqrt(f) / a, so a narrow pipe swallows its top end
        // and a wide one does not. This is free differentiation between a bike's
        // 38 mm primary and a truck's 76 mm pipe — no tone control anywhere.
        const FS: f32 = 48_000.0;
        const GAMMA: f32 = 1.4;
        const R: f32 = 287.0;
        const TEMPERATURE: f32 = 300.0;

        let brightness = |radius: f64| {
            let area = std::f64::consts::PI * radius * radius;
            let mut pipe = WaveguidePipe::new(0.6, area, FS, GAMMA, R, TEMPERATURE);
            // Straight through: impulse in one end, read what leaves the other,
            // with nothing reflecting at either boundary.
            let mut out = vec![0.0f32; 4_096];
            for (i, sample) in out.iter_mut().enumerate() {
                let (_, p_far) = pipe.read_outputs();
                pipe.push_inputs(if i == 0 { 1.0 } else { 0.0 }, 0.0);
                *sample = p_far;
            }
            let high = magnitude_at(&out, 6_000.0, FS);
            let low = magnitude_at(&out, 300.0, FS);
            high / low.max(1e-9)
        };

        // What the law predicts for the same two pipes: the tilt between the
        // two probe frequencies is the difference of the attenuations there,
        // and it doubles when the radius halves.
        let predicted = |radius: f64| {
            let alpha = |f: f64| {
                viscothermal_alpha(
                    f,
                    radius,
                    speed_of_sound(GAMMA, R, TEMPERATURE) as f64,
                    (kinematic_viscosity(TEMPERATURE, R) * BOUNDARY_LAYER_TURBULENCE_FACTOR) as f64,
                    GAMMA as f64,
                    EXHAUST_PRANDTL_NUMBER as f64,
                )
            };
            (-(alpha(6_000.0) - alpha(300.0)) * 0.6).exp() as f32
        };

        let narrow = brightness(0.019);
        let wide = brightness(0.038);
        assert!(
            wide > narrow,
            "the narrow pipe was not duller: {narrow:.3} against {wide:.3} wide"
        );
        // The ordering alone is cheap — a filter that rolls off at 6 dB/octave
        // instead of as sqrt(f) also gets it right, and gets the size of it
        // wrong by an octave's worth. So the tilt is held to the analytic
        // figure, within the tolerance of the three-shelf fit and the delay
        // line's own interpolation.
        for (radius, measured) in [(0.019f64, narrow), (0.038, wide)] {
            let want = predicted(radius);
            let error_db = 20.0 * (measured / want).log10();
            assert!(
                error_db.abs() < 1.5,
                "a {radius} m pipe tilted {measured:.3} between 300 Hz and 6 kHz,                  against {want:.3} from alpha ({error_db:+.2} dB out)"
            );
        }
    }

    #[test]
    fn two_pipe_junction_reflects_exact_area_ratio() {
        let test_cases = [
            (0.0010, 0.0020), // expansion: r = (1-2)/(1+2) = -1/3
            (0.0030, 0.0010), // contraction: r = (3-1)/(3+1) = +0.5
            (0.0025, 0.0025), // matched: r = 0
            (0.0012, 0.0060), // large expansion: r = (1.2-6)/(1.2+6) = -4.8/7.2 = -2/3
        ];

        for (a1, a2) in test_cases {
            let junction = ScatteringJunction::from_areas(&[a1, a2]);
            let expected_r = ((a1 - a2) / (a1 + a2)) as f32;
            let expected_t = (2.0 * a1 / (a1 + a2)) as f32;

            let p_plus = [1.0f32, 0.0f32];
            let mut p_minus = [0.0f32, 0.0f32];
            let p_j = junction.scatter(&p_plus, &mut p_minus);

            assert!(
                (p_minus[0] - expected_r).abs() < 1e-6,
                "expected reflection {expected_r}, got {}",
                p_minus[0]
            );
            assert!(
                (p_minus[1] - expected_t).abs() < 1e-6,
                "expected transmission {expected_t}, got {}",
                p_minus[1]
            );
            assert!(
                (p_j - expected_t).abs() < 1e-6,
                "junction pressure should match transmitted pressure"
            );
        }
    }

    #[test]
    fn n_port_junction_conserves_volume_flow_and_power() {
        // 4-1 collector junction: 4 primaries of 40 mm bore meeting 60 mm outlet
        let a_primary = std::f64::consts::PI * 0.020 * 0.020;
        let a_outlet = std::f64::consts::PI * 0.030 * 0.030;
        let junction =
            ScatteringJunction::from_areas(&[a_primary, a_primary, a_primary, a_primary, a_outlet]);

        let p_plus = [1.0f32, 0.3f32, -0.2f32, 0.0f32, -0.5f32];
        let mut p_minus = [0.0f32; 5];
        junction.scatter(&p_plus, &mut p_minus);

        let y = junction.admittances();

        // 1. Volume flow conservation: sum_i Y_i (p_i^+ - p_i^-) == 0
        let mut net_flow = 0.0f32;
        for i in 0..5 {
            net_flow += y[i] * (p_plus[i] - p_minus[i]);
        }
        assert!(
            net_flow.abs() < 1e-6,
            "volume flow not conserved: net_flow = {net_flow}"
        );

        // 2. Power conservation: sum_i Y_i (p_i^+)^2 == sum_i Y_i (p_i^-)^2
        let mut power_in = 0.0f32;
        let mut power_out = 0.0f32;
        for i in 0..5 {
            power_in += y[i] * p_plus[i] * p_plus[i];
            power_out += y[i] * p_minus[i] * p_minus[i];
        }
        assert!(
            (power_out - power_in).abs() < 1e-6 * power_in,
            "power not conserved: in={power_in}, out={power_out}"
        );
        assert!(
            power_out <= power_in + 1e-7,
            "outgoing energy exceeds incoming"
        );
    }

    #[test]
    fn port_jet_noise_is_the_cube_of_throat_velocity() {
        // Curle's dipole is a sixth power in radiated *power*, which is a cube
        // in pressure. Doubling the velocity has to be eighteen decibels, not
        // twelve — that is the whole difference between a chuff at each valve
        // event and a hiss across the cycle.
        let rho = 0.39;
        let c = 600.0;
        let area = 5.0e-4;
        let pipe = std::f32::consts::PI * 0.020 * 0.020;
        let at = |u: f32| jet_pressure_fluctuation_pa(u, rho, c, area, pipe);
        let ratio = at(300.0) / at(150.0);
        assert!(
            (ratio - 8.0).abs() < 1e-3,
            "doubling velocity must be eight times the pressure: {ratio}"
        );
        assert_eq!(
            at(0.0),
            0.0,
            "a port with no flow through it makes no noise"
        );
    }

    #[test]
    fn a_port_that_chokes_stops_getting_louder() {
        // The gap passes no more velocity past its sonic limit however much
        // harder the cylinder pushes, so the noise has a ceiling and it is the
        // gas that sets it.
        let rho = 0.39;
        let c = 600.0;
        let area = 5.0e-4;
        let sonic = throat_velocity(rho * area * c, area, rho, c);
        let beyond = throat_velocity(rho * area * c * 4.0, area, rho, c);
        assert!(
            (sonic - c).abs() < 1e-3,
            "the throat should be at Mach 1: {sonic}"
        );
        assert_eq!(sonic, beyond, "a choked throat cannot go faster");
    }

    #[test]
    fn a_narrower_gap_hisses_higher() {
        // St = f d / u. At one velocity the pitch is set by the gap alone, and
        // it goes as one over the diameter, so a quarter of the area is twice
        // the frequency.
        let wide = jet_peak_hz(300.0, 4.0e-4);
        let narrow = jet_peak_hz(300.0, 1.0e-4);
        assert!(
            ((narrow / wide) - 2.0).abs() < 1e-3,
            "quartering the area must double the pitch: {wide:.0} Hz to {narrow:.0} Hz"
        );
        // And the absolute number is the Strouhal law, not a tuning.
        let d = 2.0 * (4.0e-4f32 / std::f32::consts::PI).sqrt();
        assert!((wide - JET_STROUHAL_NUMBER * 300.0 / d).abs() < 1e-3);
    }

    #[test]
    fn steepening_raises_high_orders_with_amplitude() {
        // A crest rides on its own induced flow through gas it has itself
        // heated, so it gains on the trough ahead of it and the front stands
        // up. At a fixed fundamental that shows as high-order content that
        // grows with amplitude — and vanishes when the pulse is small, because
        // beta is p / P_0 and a quiet wave has nothing to gain on.
        const FS: f32 = 48_000.0;
        const FREQ: f32 = 200.0; // 240 samples per cycle at 48 kHz
        const GAMMA: f32 = 1.35;
        const R: f32 = 287.0;
        const TEMPERATURE: f32 = 800.0;
        let area = std::f64::consts::PI * 0.020 * 0.020;

        let measure_harmonic_ratio = |amp: f32| -> f32 {
            let mut pipe = WaveguidePipe::new(0.5, area, FS, GAMMA, R, TEMPERATURE);
            pipe.set_steepening(true);
            pipe.snap_delays();

            let n_total = 4800; // 20 complete cycles
            let n_eval = 2400; // 10 cycles for steady-state evaluation
            let mut re_fund = 0.0f64;
            let mut im_fund = 0.0f64;
            let mut re_h2 = 0.0f64;
            let mut im_h2 = 0.0f64;

            for i in 0..n_total {
                let t = i as f32 / FS;
                let input = amp * (std::f32::consts::TAU * FREQ * t).sin();
                pipe.push_inputs(input, 0.0);
                let (_out0, out1) = pipe.read_outputs();

                if i >= n_total - n_eval {
                    let phase1 = std::f32::consts::TAU * FREQ * t;
                    let phase2 = std::f32::consts::TAU * 2.0 * FREQ * t;
                    re_fund += out1 as f64 * phase1.sin() as f64;
                    im_fund += out1 as f64 * phase1.cos() as f64;
                    re_h2 += out1 as f64 * phase2.sin() as f64;
                    im_h2 += out1 as f64 * phase2.cos() as f64;
                }
            }

            let m_fund = (re_fund * re_fund + im_fund * im_fund).sqrt() / n_eval as f64;
            let m_h2 = (re_h2 * re_h2 + im_h2 * im_h2).sqrt() / n_eval as f64;
            (m_h2 / m_fund.max(1e-12)) as f32
        };

        // A hundred pascals is a loud noise and a thousandth of an atmosphere.
        let ratio_quiet = measure_harmonic_ratio(100.0);
        // A blowdown is a fair fraction of an atmosphere.
        let ratio_loud = measure_harmonic_ratio(0.5 * REFERENCE_PRESSURE_PA);

        assert!(
            ratio_quiet < 0.001,
            "high orders should be absent at low amplitude: got {ratio_quiet}"
        );
        assert!(
            ratio_loud > 0.05,
            "high orders should rise with amplitude: got {ratio_loud}"
        );
        assert!(
            ratio_loud > 50.0 * ratio_quiet,
            "high-order content must rise strongly with amplitude: loud={ratio_loud}, quiet={ratio_quiet}"
        );
    }

    #[test]
    fn opening_the_cutout_raises_high_order_content_and_lowers_back_pressure() {
        use crate::physics::plumbing::{
            Collector, Crossover, ExhaustSystem, PipeSection, Silencer,
        };

        const FS: f32 = 48_000.0;

        let exhaust = ExhaustSystem {
            primaries: vec![PipeSection::from_diameter(0.45, 0.040, 850.0); 4],
            collector: Collector::from_diameter(4, 0.060, 0.15),
            secondary: vec![],
            crossover: Crossover::None,
            silencers: vec![
                Silencer::ExpansionChamber {
                    length: 0.40,
                    area_ratio: 4.0,
                    stages: 2,
                },
                Silencer::Absorptive {
                    length: 0.35,
                    area: std::f64::consts::PI * 0.030 * 0.030,
                    packing_thickness: 0.035,
                    packing_absorption: 0.85,
                },
            ],
            tailpipe: PipeSection::from_diameter(1.0, 0.060, 600.0),
            tailpipe_flanged: false,
            cutout: false,
        };

        // 1. Back pressure must be strictly lower with cutout open than closed
        let mass_flow = 0.20; // 200 g/s exhaust flow
        let bp_closed = exhaust.back_pressure(mass_flow, false);
        let bp_open = exhaust.back_pressure(mass_flow, true);
        assert!(
            bp_open < bp_closed * 0.70,
            "opening cutout must significantly lower back pressure: open={bp_open}, closed={bp_closed}"
        );

        // 2. High-order harmonic content in radiated sound must be higher with cutout open
        let cylinders: Vec<crate::audio::dsp::CylinderTap> = (0..4)
            .map(|i| crate::audio::dsp::CylinderTap {
                evo_phase: i as f32 / 4.0,
                bank: 0,
            })
            .collect();

        let snapshot = crate::audio::dsp::EngineSnapshot::default();
        let run_and_measure_high_order_energy = |cutout: bool| -> f64 {
            let mut network = ExhaustNetwork::new(&exhaust, &cylinders, 1, FS, &snapshot);
            network.set_cutout(cutout);

            let mut high_energy = 0.0f64;
            let mut radiated = [0.0f32; 1];
            let bank_excitations = [0.0f32; 1];

            for i in 0..9600 {
                let pulse = if i % 240 < 6 { 1.0 } else { 0.0 };
                let excitations = [pulse, 0.0, 0.0, 0.0];
                network.step(&excitations, &bank_excitations, &mut radiated);

                if i >= 2400 {
                    for freq in [1500.0, 2000.0, 3000.0, 4000.0] {
                        let phase = std::f32::consts::TAU * freq * (i as f32) / FS;
                        high_energy += (radiated[0] as f64 * phase.sin() as f64).powi(2);
                    }
                }
            }
            high_energy
        };

        let high_closed = run_and_measure_high_order_energy(false);
        let high_open = run_and_measure_high_order_energy(true);

        assert!(
            high_open > high_closed * 2.0,
            "opening cutout bypass must raise high-order spectral content: open={high_open}, closed={high_closed}"
        );
    }

    #[test]
    fn open_headers_and_cutout_enable_tailpipe_steepening() {
        use crate::physics::plumbing::ExhaustSystem;

        let primaries =
            vec![crate::physics::plumbing::PipeSection::from_diameter(0.60, 0.040, 850.0); 4];
        let collector = crate::physics::plumbing::Collector::from_diameter(4, 0.060, 0.15);
        let exhaust = ExhaustSystem::open_headers(primaries, collector);
        let cylinders = vec![
            crate::audio::dsp::CylinderTap {
                evo_phase: 0.0,
                bank: 0,
            },
            crate::audio::dsp::CylinderTap {
                evo_phase: 0.5,
                bank: 0,
            },
        ];
        let snapshot = crate::audio::dsp::EngineSnapshot::default();
        let mut network = ExhaustNetwork::new(&exhaust, &cylinders, 1, 48_000.0, &snapshot);
        assert!(network.tailpipes[0].steepening());

        network.set_cutout(false);
        assert!(!network.tailpipes[0].steepening());

        network.set_cutout(true);
        assert!(network.tailpipes[0].steepening());
    }

    /// Builds a single-cylinder exhaust that is, acoustically, one
    /// uninterrupted pipe: a collector with one inlet whose outlet area
    /// matches the primary's leaves nothing for the junction to reflect, so
    /// the primary, the collector's taper and the tailpipe scatter as though
    /// they were cut from the same tube. Closed at the valve (unset, so
    /// [`ValveTermination`] reflects at unity — see its `new`), open at the
    /// mouth. That is Stage T4's "single pipe with no silencers and no
    /// collector": the only boundaries left are the two ends.
    fn single_pipe(
        l_primary: f64,
        taper: f64,
        tailpipe: f64,
        diameter: f64,
    ) -> crate::physics::plumbing::ExhaustSystem {
        single_pipe_flanged(l_primary, taper, tailpipe, diameter, false)
    }

    /// As [`single_pipe`], but with the mouth's flange choice exposed.
    fn single_pipe_flanged(
        l_primary: f64,
        taper: f64,
        tailpipe: f64,
        diameter: f64,
        flanged: bool,
    ) -> crate::physics::plumbing::ExhaustSystem {
        use crate::physics::plumbing::{
            Collector, Crossover, ExhaustSystem, PipeSection, Silencer,
        };
        let area = std::f64::consts::PI * (diameter * 0.5).powi(2);
        ExhaustSystem {
            primaries: vec![PipeSection::from_diameter(l_primary, diameter, 300.0)],
            collector: Collector::new(1, area, taper),
            secondary: vec![],
            crossover: Crossover::None,
            silencers: vec![Silencer::Straight],
            tailpipe: PipeSection::from_diameter(tailpipe, diameter, 300.0),
            tailpipe_flanged: flanged,
            cutout: false,
        }
    }

    /// The strongest resonance a single impulse rings up in a network's
    /// radiated output, scanned by Goertzel projection near `guess_hz`.
    ///
    /// One impulse excites every mode of the pipe at once; scanning the
    /// projection of the resulting ring-down onto each candidate frequency and
    /// keeping the largest finds where the energy actually piled up, which for
    /// an undamped-ish closed-open pipe is its quarter-wave fundamental.
    fn impulse_resonance_hz(
        exhaust: &crate::physics::plumbing::ExhaustSystem,
        fs: f32,
        guess_hz: f32,
        search_fraction: f32,
    ) -> f32 {
        let cylinders = vec![crate::audio::dsp::CylinderTap {
            evo_phase: 0.0,
            bank: 0,
        }];
        let snapshot = crate::audio::dsp::EngineSnapshot::default();
        let mut network = ExhaustNetwork::new(exhaust, &cylinders, 1, fs, &snapshot);

        let capture = 4 * fs as usize; // 4 s: hundreds of round trips, 0.25 Hz Goertzel resolution
        let mut response = Vec::with_capacity(capture);
        let mut radiated = [0.0f32; 1];
        let bank_excitations = [0.0f32; 1];
        for i in 0..capture {
            let excitations = [if i == 0 { 1.0 } else { 0.0 }];
            network.step(&excitations, &bank_excitations, &mut radiated);
            response.push(radiated[0]);
        }

        let mut best = (0.0f32, -1.0f32);
        let lo = guess_hz * (1.0 - search_fraction);
        let hi = guess_hz * (1.0 + search_fraction);
        let mut f = lo;
        while f <= hi {
            let m = magnitude_at(&response, f, fs);
            if m > best.1 {
                best = (f, m);
            }
            f += guess_hz * 0.0005;
        }
        best.0
    }

    #[test]
    fn single_pipe_resonates_at_the_quarter_wave_prediction() {
        // Appendix A's `f = c(1 - M^2) / 4L`, with M = 0 here (no mean flow is
        // set on this network) and L the pipe's *acoustic* length — physical
        // length plus the mouth's own Karal-Flugge end correction, per
        // `examples/calibrate.rs`'s treatment of the same mode.
        use crate::audio::radiation::end_correction;

        const FS: f32 = 48_000.0;
        const DIAMETER: f64 = 0.045;
        const TAPER: f64 = 0.02; // TaperedCollector's own floor, see its `new`
        const TAILPIPE: f64 = 0.05;
        let radius = DIAMETER * 0.5;
        let delta = end_correction(radius, false);
        let c = speed_of_sound(1.33, 287.0, 300.0);

        for &l_primary in &[0.50f64, 0.80, 1.20] {
            let exhaust = single_pipe(l_primary, TAPER, TAILPIPE, DIAMETER);
            let total_length = l_primary + TAPER + TAILPIPE + delta;
            let expected = c / (4.0 * total_length as f32);

            let measured = impulse_resonance_hz(&exhaust, FS, expected, 0.3);
            let error = (measured - expected).abs() / expected;
            assert!(
                error < 0.02,
                "primary {l_primary} m: measured {measured:.1} Hz against {expected:.1} Hz \
                 predicted ({:.1} % off)",
                error * 100.0
            );
        }
    }

    #[test]
    fn doubling_primary_length_halves_the_tuning_peak() {
        // Long enough that the fixed taper, tailpipe and end correction are a
        // small fraction of the total, so doubling the primary comes close to
        // doubling the whole acoustic length and the peak comes close to
        // halving. The stage's own claim is about the primary, not about a
        // pipe with nothing else attached to it.
        const FS: f32 = 48_000.0;
        const DIAMETER: f64 = 0.045;
        const TAPER: f64 = 0.02;
        const TAILPIPE: f64 = 0.05;
        let c = speed_of_sound(1.33, 287.0, 300.0);

        let guess = |l: f64| c / (4.0 * (l + TAPER + TAILPIPE) as f32);

        let short = 1.0f64;
        let long = 2.0 * short;
        let exhaust_short = single_pipe(short, TAPER, TAILPIPE, DIAMETER);
        let exhaust_long = single_pipe(long, TAPER, TAILPIPE, DIAMETER);

        let f_short = impulse_resonance_hz(&exhaust_short, FS, guess(short), 0.3);
        let f_long = impulse_resonance_hz(&exhaust_long, FS, guess(long), 0.3);

        let ratio = f_long / f_short;
        assert!(
            (ratio - 0.5).abs() < 0.05,
            "doubling the primary from {short} m to {long} m moved the peak from \
             {f_short:.1} Hz to {f_long:.1} Hz, a ratio of {ratio:.3} rather than one half"
        );
    }

    #[test]
    fn unflanged_tailpipe_effective_length_matches_karal_flugge() {
        // A tailpipe read through the full `ExhaustNetwork` — valve, junction,
        // taper and the collector's one-sample return register, see that
        // field's own doc comment — carries a little extra latency beside the
        // mouth's, on the order of the "about 7 mm of pipe" the network
        // documents for that register alone. That is shared by any tailpipe on
        // this rig regardless of its flange, so building the *same* geometry
        // flanged and unflanged and differencing the two measured peaks
        // cancels it and leaves exactly the term this test is about: the
        // 0.8216a and 0.6133a end corrections read back off where the peak
        // actually landed, rather than asserted.
        use crate::audio::radiation::{FLANGED_END_CORRECTION, UNFLANGED_END_CORRECTION};

        const FS: f32 = 48_000.0;
        const DIAMETER: f64 = 0.050;
        const TAPER: f64 = 0.02;
        const TAILPIPE: f64 = 0.35;
        const L_PRIMARY: f64 = 0.60;
        let radius = DIAMETER * 0.5;
        let c = speed_of_sound(1.33, 287.0, 300.0);
        let guess = c / (4.0 * (L_PRIMARY + TAPER + TAILPIPE) as f32);

        let unflanged = single_pipe_flanged(L_PRIMARY, TAPER, TAILPIPE, DIAMETER, false);
        let flanged = single_pipe_flanged(L_PRIMARY, TAPER, TAILPIPE, DIAMETER, true);

        let f_unflanged = impulse_resonance_hz(&unflanged, FS, guess, 0.3);
        let f_flanged = impulse_resonance_hz(&flanged, FS, guess, 0.3);

        // A wider end correction is a longer effective pipe, and a longer pipe
        // resonates lower: the flanged case must land under the unflanged one.
        assert!(
            f_flanged < f_unflanged,
            "the flanged mouth ({f_flanged:.1} Hz) should resonate lower than the \
             unflanged one ({f_unflanged:.1} Hz), not higher or the same"
        );

        let implied_gap = c / (4.0 * f_flanged) - c / (4.0 * f_unflanged);
        let expected_gap = ((FLANGED_END_CORRECTION - UNFLANGED_END_CORRECTION) * radius) as f32;

        let error = (implied_gap - expected_gap).abs() / expected_gap;
        assert!(
            error < 0.20,
            "flanged vs unflanged moved the effective length by {implied_gap:.4} m; the \
             Karal-Flugge constants ({FLANGED_END_CORRECTION} - {UNFLANGED_END_CORRECTION}) x \
             {radius:.4} m radius predict {expected_gap:.4} m ({:.1} % off)",
            error * 100.0
        );
    }
}
