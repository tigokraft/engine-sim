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

/// Coldest gas temperature used to size delay line buffers [K].
///
/// Sound travels slowest when cold ($c \propto \sqrt{T}$), which yields the
/// largest transit delay in samples. Sizing buffers for 250 K ensures the
/// delay lines never need reallocation during runtime temperature glides.
pub const COLDEST_EXHAUST_TEMPERATURE_K: f32 = 250.0;

/// Reference atmospheric pressure for gas density estimation [Pa].
pub const REFERENCE_PRESSURE_PA: f32 = 101_325.0;

/// Typical kinematic viscosity of exhaust gas at reference conditions [m^2 / s].
pub const EXHAUST_KINEMATIC_VISCOSITY: f32 = 8.4e-5;

/// Prandtl number of air and lean combustion gas [-].
pub const EXHAUST_PRANDTL_NUMBER: f32 = 0.71;

/// Effective turbulent boundary layer enhancement factor in corrugated/hot exhaust pipe.
const BOUNDARY_LAYER_TURBULENCE_FACTOR: f32 = 16.0;

/// Fraction of the valve-end reflection that survives full damping [-].
///
/// Not zero: even wide open, a port is a real area change and does send
/// something back. It is small enough that the primary's ringdown falls below
/// one firing interval, which is what stops a comb from forming.
pub const VALVE_DAMPED_REFLECTION: f32 = 0.35;

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

/// Derives the one-pole lowpass loss cutoff [Hz] for a pipe segment of length $L$ and radius $a$.
///
/// Equating the total boundary layer attenuation $\alpha(f_c) L$ to the filter's
/// $-3\text{ dB}$ point ($\ln\sqrt{2} \approx 0.3466$) yields a cutoff scaling as:
///
/// $$f_c \propto \frac{a^2 c^2}{L^2}$$
///
/// Under this scaling, a 38 mm primary runner has its loss cutoff two octaves
/// below an equivalent 76 mm pipe, making the narrow pipe substantially duller.
#[inline]
pub fn viscothermal_loss_cutoff_hz(
    radius: f32,
    length: f32,
    speed_of_sound: f32,
    gamma: f32,
) -> f32 {
    let nu_eff = EXHAUST_KINEMATIC_VISCOSITY * BOUNDARY_LAYER_TURBULENCE_FACTOR;
    let thermal = 1.0 + (gamma - 1.0) / EXHAUST_PRANDTL_NUMBER.sqrt();
    let denom = length.max(0.02) * (std::f32::consts::PI * nu_eff).sqrt() * thermal;
    let sqrt_fc = (0.3466 * radius.max(1e-4) * speed_of_sound) / denom.max(1e-6);
    let fc = sqrt_fc * sqrt_fc;
    fc.clamp(150.0, 18_000.0)
}

/// Viscothermal boundary layer loss filter.
///
/// Implemented as a one-pole lowpass filter inside each waveguide directional path.
#[derive(Debug, Clone, Copy)]
pub struct ViscothermalLoss {
    filter: OnePole,
    cutoff_hz: f32,
}

impl ViscothermalLoss {
    /// Creates a new loss filter for a pipe of given radius $a$ and length $L$.
    pub fn new(
        radius: f32,
        length: f32,
        speed_of_sound: f32,
        gamma: f32,
        sample_rate: f32,
    ) -> Self {
        let cutoff_hz = viscothermal_loss_cutoff_hz(radius, length, speed_of_sound, gamma);
        Self {
            filter: OnePole::new(sample_rate, cutoff_hz),
            cutoff_hz,
        }
    }

    /// Retunes the loss cutoff for current sound speed and specific heat ratio.
    pub fn tune(
        &mut self,
        radius: f32,
        length: f32,
        speed_of_sound: f32,
        gamma: f32,
        sample_rate: f32,
    ) {
        let cutoff_hz = viscothermal_loss_cutoff_hz(radius, length, speed_of_sound, gamma);
        self.cutoff_hz = cutoff_hz;
        self.filter.set_cutoff(sample_rate, cutoff_hz);
    }

    /// Filters one travelling sample.
    #[inline(always)]
    pub fn process(&mut self, x: f32) -> f32 {
        self.filter.process(x)
    }

    /// Current filter cutoff frequency [Hz].
    pub fn cutoff_hz(&self) -> f32 {
        self.cutoff_hz
    }

    /// Clears internal state.
    pub fn reset(&mut self) {
        self.filter.reset();
    }
}

// ---------------------------------------------------------------------------
// Valve-end termination
// ---------------------------------------------------------------------------

/// Acoustic boundary condition at the exhaust valve end of a primary runner.
///
/// Models the interface between the cylinder combustion chamber and the exhaust runner.
/// When the exhaust valve is shut ($A_v = 0$), the boundary is acoustically rigid
/// with pressure reflection coefficient $r = +1.0$. As the valve opens with effective
/// area $A_v$, the reflection coefficient glides according to:
///
/// $$r = \frac{A_p - A_v}{A_p + A_v}$$
///
/// allowing wave energy to transmit into the cylinder cavity. Blowdown excitation
/// pulses are injected into the forward path at this boundary:
///
/// $$p^+ = p_{\text{excitation}} + r \cdot p^-$$
#[derive(Debug, Clone, Copy)]
pub struct ValveTermination {
    pipe_area: f32,
    reflection: f32,
    damping: f32,
}

impl ValveTermination {
    /// Constructs a valve termination for a pipe of area $A_p$ [m^2].
    pub fn new(pipe_area: f64) -> Self {
        Self {
            pipe_area: pipe_area.max(1e-7) as f32,
            reflection: 1.0,
            damping: 0.0,
        }
    }

    /// Updates the reflection coefficient from the valve effective flow area $A_v$ [m^2].
    pub fn set_effective_area(&mut self, effective_area: f64) {
        let av = effective_area.max(0.0) as f32;
        let ap = self.pipe_area;
        self.reflection = if av <= 1e-9 {
            1.0
        } else {
            ((ap - av) / (ap + av)).clamp(-1.0, 1.0)
        };
    }

    /// Reflection coefficient currently in effect [-].
    pub fn reflection(&self) -> f32 {
        self.reflection * (1.0 - self.damping * (1.0 - VALVE_DAMPED_REFLECTION))
    }

    /// Softens the reflection, `0` for the bare boundary and `1` fully damped.
    pub fn set_damping(&mut self, damping: f32) {
        self.damping = damping.clamp(0.0, 1.0);
    }

    /// Damping currently applied, `0..=1` [-].
    pub fn damping(&self) -> f32 {
        self.damping
    }

    /// Computes the forward-travelling wave entering the runner:
    /// - `excitation`: blowdown pressure pulse injected at the port [Pa].
    /// - `returning_wave`: backward wave arriving at the valve boundary ($p^-(0)$) [Pa].
    #[inline(always)]
    pub fn step(&self, excitation: f32, returning_wave: f32) -> f32 {
        excitation + self.reflection() * returning_wave
    }
}

// ---------------------------------------------------------------------------
// Mouth termination (Stage 4 open end)
// ---------------------------------------------------------------------------

/// End correction for an open circular pipe [m].
///
/// Acoustically, an open pipe behaves as if it extends beyond its physical edge
/// by $\delta$: $0.6133 a$ for an unflanged pipe, $0.8216 a$ for a flanged pipe.
#[inline]
pub fn mouth_end_correction(radius: f64, flanged: bool) -> f64 {
    let factor = if flanged { 0.8216 } else { 0.6133 };
    factor * radius.max(0.0)
}

/// Radiation cutoff corner frequency for an open mouth [Hz]:
///
/// $$f_c = \frac{c}{2 \pi a}$$
///
/// Below $f_c$ ($k a < 1$), sound reflects strongly off the open boundary ($|R| \to 1$).
/// Above $f_c$ ($k a > 1$), sound beams out and reflection drops toward zero.
#[inline]
pub fn mouth_corner_hz(radius: f32, speed_of_sound: f32) -> f32 {
    speed_of_sound / (std::f32::consts::TAU * radius.max(1e-4))
}

/// Acoustic open-end mouth termination with Levine–Schwinger reflection fit and monopole radiation.
///
/// Implements three acoustic properties derived purely from mouth geometry:
/// 1. **Reflection**: First-order Levine-Schwinger fit with DC gain $-1$ (pressure release)
///    and corner frequency $f_c = c / (2 \pi a)$:
///    $$p^- = - H_{LP}(z) p^+$$
/// 2. **End correction**: Pipe length is augmented by $\delta = 0.6133 a$ (or $0.8216 a$ flanged).
/// 3. **Radiated field**: Transmitted component $(1 + R) p^+ = p^+ + p^-$, which naturally
///    differentiates the low frequencies (+6 dB/octave monopole tilt below $f_c$) and flattens
///    above $f_c$, scaled by mouth cross-sectional area.
#[derive(Debug, Clone, Copy)]
pub struct MouthTermination {
    radius: f32,
    area: f32,
    flanged: bool,
    filter: OnePole,
    corner_hz: f32,
}

impl MouthTermination {
    /// Constructs a mouth termination for an open pipe of radius $a$ [m].
    pub fn new(radius: f64, flanged: bool, speed_of_sound: f32, sample_rate: f32) -> Self {
        let r = radius.max(1e-4) as f32;
        let area = std::f32::consts::PI * r * r;
        let corner_hz = mouth_corner_hz(r, speed_of_sound);
        Self {
            radius: r,
            area,
            flanged,
            filter: OnePole::new(sample_rate, corner_hz),
            corner_hz,
        }
    }

    /// Acoustic end correction length $\delta$ to add to the attached pipe section [m].
    pub fn end_correction(&self) -> f64 {
        mouth_end_correction(self.radius as f64, self.flanged)
    }

    /// Retunes the reflection filter for current speed of sound.
    pub fn tune(&mut self, speed_of_sound: f32, sample_rate: f32) {
        self.corner_hz = mouth_corner_hz(self.radius, speed_of_sound);
        self.filter.set_cutoff(sample_rate, self.corner_hz);
    }

    /// Corner frequency $f_c = c / (2 \pi a)$ [Hz].
    pub fn corner_hz(&self) -> f32 {
        self.corner_hz
    }

    /// Mouth cross-sectional area [m^2].
    pub fn area(&self) -> f32 {
        self.area
    }

    /// Steps the termination with incident forward wave $p^+$ from the pipe:
    /// Returns `(p_minus, p_radiated)`:
    /// - `p_minus`: reflected backward wave returning down the pipe ($p^-$) [Pa].
    /// - `p_radiated`: sound pressure radiated into free space [Pa].
    #[inline(always)]
    pub fn step(&mut self, p_plus: f32) -> (f32, f32) {
        // Lowpass reflection magnitude: H_LP -> 1 at DC, -> 0 at high frequencies
        let lp = self.filter.process(p_plus);
        // Pressure-release reflection: R(0) = -1
        let p_minus = -lp;
        // Transmitted pressure: (1 + R) p^+ = p^+ + p^-
        // Since p^- = -lp, this is (p^+ - lp), a differentiator (+6 dB/octave below fc)
        // Scaled by mouth area
        let p_trans = p_plus + p_minus;
        let p_rad = p_trans * (self.area * 500.0);
        (p_minus, p_rad)
    }

    /// Clears internal filter state.
    pub fn reset(&mut self) {
        self.filter.reset();
    }
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
    forward_line: DelayLine,
    backward_line: DelayLine,
    forward_loss: ViscothermalLoss,
    backward_loss: ViscothermalLoss,
    delay_samples: Smoothed,
    admittance: f32,
    sample_rate: f32,
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
        let max_delay_samples = (max_delay_sec * sample_rate).ceil() as usize + 8;

        let initial_delay_sec = runner_delay_seconds(length_f32, gamma, gas_constant, temperature);
        let initial_delay_samples = (initial_delay_sec * sample_rate).max(1.0);

        let c = speed_of_sound(gamma, gas_constant, temperature);
        let rho = REFERENCE_PRESSURE_PA / (gas_constant * temperature.max(1.0));
        let admittance = area_f32 / (rho * c).max(1e-4);

        let forward_loss = ViscothermalLoss::new(radius_f32, length_f32, c, gamma, sample_rate);
        let backward_loss = ViscothermalLoss::new(radius_f32, length_f32, c, gamma, sample_rate);

        Self {
            length: length_f32,
            area: area_f32,
            forward_line: DelayLine::with_max_delay(max_delay_samples),
            backward_line: DelayLine::with_max_delay(max_delay_samples),
            forward_loss,
            backward_loss,
            // 40 ms time constant matches the engine temperature glide: fast enough
            // to follow throttle snaps, slow enough that fractional interpolation
            // never produces audible doppler pitch clicks.
            delay_samples: Smoothed::new(initial_delay_samples, sample_rate, 0.040),
            admittance,
            sample_rate,
        }
    }

    /// Retunes propagation delay, acoustic admittance, and wall losses for current gas state.
    pub fn tune(&mut self, gamma: f32, gas_constant: f32, temperature: f32) {
        let delay_sec = runner_delay_seconds(self.length, gamma, gas_constant, temperature);
        let samples = delay_sec * self.sample_rate;
        self.delay_samples
            .set_target(samples.clamp(1.0, self.forward_line.max_delay()));

        let c = speed_of_sound(gamma, gas_constant, temperature);
        let rho = REFERENCE_PRESSURE_PA / (gas_constant * temperature.max(1.0));
        self.admittance = self.area / (rho * c).max(1e-4);

        let r = self.radius();
        self.forward_loss
            .tune(r, self.length, c, gamma, self.sample_rate);
        self.backward_loss
            .tune(r, self.length, c, gamma, self.sample_rate);
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

    /// Current one-way transit delay [samples].
    pub fn delay_samples(&self) -> f32 {
        self.delay_samples.value()
    }

    /// One-way transit time through the pipe [s].
    pub fn transit_time_seconds(&self) -> f32 {
        self.delay_samples.value() / self.sample_rate
    }

    /// Wall loss filter cutoff [Hz].
    pub fn loss_cutoff_hz(&self) -> f32 {
        self.forward_loss.cutoff_hz()
    }

    /// Reads waves arriving at the boundaries from inside the pipe with viscothermal wall loss applied:
    /// - `out_port0`: wave emerging at port 0 from the backward delay line ($p^-(0)$).
    /// - `out_port1`: wave emerging at port 1 from the forward delay line ($p^+(L)$).
    #[inline(always)]
    pub fn read_outputs(&mut self) -> (f32, f32) {
        let d = self.delay_samples.next_value();
        let raw0 = self.backward_line.read(d);
        let raw1 = self.forward_line.read(d);
        let out0 = self.backward_loss.process(raw0);
        let out1 = self.forward_loss.process(raw1);
        (out0, out1)
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
        self.forward_line.reset();
        self.backward_line.reset();
        self.forward_loss.reset();
        self.backward_loss.reset();
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

// ---------------------------------------------------------------------------
// Composed elements: Absorptive silencer
// ---------------------------------------------------------------------------

/// Straight-through packed silencer: a perforated core inside acoustic packing.
///
/// The core is a duct like any other, so it is a [`WaveguidePipe`] with area
/// steps at each end. What the packing adds is resistive attenuation, quoted by
/// the geometry as a loss in decibels per metre. Over a body of length $L$ the
/// surviving amplitude is
///
/// $$g = 10^{-\frac{\alpha_{dB} L}{20}}$$
///
/// applied to each direction of travel, so a wave that goes in and comes back
/// pays for the length twice — which is exactly why a packed silencer kills the
/// returning reflection harder than it kills the through path.
#[derive(Debug, Clone)]
pub struct AbsorptiveSilencer {
    junction_in: ScatteringJunction,
    core: WaveguidePipe,
    junction_out: ScatteringJunction,
    /// Amplitude surviving one pass through the packing [-].
    pass: f32,
    scatter_buf_in: [f32; 2],
    scatter_buf_out: [f32; 2],
}

impl AbsorptiveSilencer {
    /// Constructs a packed silencer:
    /// - `pipe_area`: area of the ducts either side of the body [m^2].
    /// - `core_area`: flow area through the perforated core [m^2].
    /// - `length`: length of the packed body [m].
    /// - `loss_db_per_m`: attenuation of the packing [dB/m].
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        pipe_area: f64,
        core_area: f64,
        length: f64,
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

        Self {
            junction_in: ScatteringJunction::from_areas(&[a1, a2]),
            core,
            junction_out: ScatteringJunction::from_areas(&[a2, a1]),
            pass,
            scatter_buf_in: [0.0; 2],
            scatter_buf_out: [0.0; 2],
        }
    }

    /// Amplitude surviving one pass through the packing [-].
    pub fn pass_gain(&self) -> f32 {
        self.pass
    }

    /// Retunes propagation delay and acoustic admittance for current gas state.
    pub fn tune(&mut self, gamma: f32, gas_constant: f32, temperature: f32) {
        self.core.tune(gamma, gas_constant, temperature);
    }

    /// Steps the silencer by one sample; see [`ExpansionChamber::step`] for the
    /// port convention.
    #[inline(always)]
    pub fn step(&mut self, p_in_plus: f32, p_out_minus: f32) -> (f32, f32) {
        let (p_core_0, p_core_1) = self.core.read_outputs();
        let p_core_0 = p_core_0 * self.pass;
        let p_core_1 = p_core_1 * self.pass;

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
                loss_db_per_m,
            } => chain.push(Self::Absorptive(AbsorptiveSilencer::new(
                pipe_area,
                *area,
                *length,
                *loss_db_per_m,
                sample_rate,
                gamma,
                gas_constant,
                temperature,
            ))),
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

/// Complete physical 1D exhaust waveguide network.
///
/// Instantiated from an [`ExhaustSystem`] geometry description:
/// - One primary runner per cylinder, terminated at the valve with [`ValveTermination`].
/// - An arithmetic [`TaperedCollector`] per bank.
/// - Bank [`BankCrossover`] linking dual-bank systems.
/// - Expansion chamber silencers.
/// - Tailpipes terminated with Stage 4 [`MouthTermination`].
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
    mouths: Vec<MouthTermination>,
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
        let temp = snapshot.exhaust_temperature;
        let c = speed_of_sound(gamma, r, temp);

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
            let prim = WaveguidePipe::new(spec.length, spec.area, sample_rate, gamma, r, temp);
            let valve = ValveTermination::new(spec.area);
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
        let crossover = BankCrossover::from_crossover(
            &exhaust.crossover,
            exhaust.collector.outlet_area,
            sample_rate,
            gamma,
            r,
            temp,
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

        let mut pre_cross_pipes = Vec::with_capacity(n_banks);
        if let Some(position) = cross_position {
            for _ in 0..n_banks {
                pre_cross_pipes.push(WaveguidePipe::new(
                    position,
                    exhaust.collector.outlet_area,
                    sample_rate,
                    gamma,
                    r,
                    temp,
                ));
            }
        }

        // 4. Silencer chain per bank
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
        }

        // 5. Tailpipes and Mouth terminations per bank
        let mut tailpipes = Vec::with_capacity(n_banks);
        let mut mouths = Vec::with_capacity(n_banks);
        for _ in 0..n_banks {
            let mouth = MouthTermination::new(
                exhaust.tailpipe.diameter() * 0.5,
                exhaust.tailpipe_flanged,
                c,
                sample_rate,
            );
            let eff_tail_len = exhaust.tailpipe.length + mouth.end_correction();
            let tailpipe = WaveguidePipe::new(
                eff_tail_len,
                exhaust.tailpipe.area,
                sample_rate,
                gamma,
                r,
                temp,
            );
            tailpipes.push(tailpipe);
            mouths.push(mouth);
        }

        // Preallocate buffers
        let prim_in_0 = vec![0.0; n_cyl];
        let prim_to_collector = vec![0.0; n_cyl];
        let mut bank_prim_in = Vec::with_capacity(n_banks);
        let mut bank_prim_refl = Vec::with_capacity(n_banks);
        for cyls in &bank_cylinders {
            let cnt = cyls.len();
            bank_prim_in.push(vec![0.0; cnt]);
            bank_prim_refl.push(vec![0.0; cnt]);
        }
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
        }
    }

    /// Retunes propagation delay and acoustic filters across the whole network.
    pub fn tune(&mut self, gamma: f32, gas_constant: f32, temperature: f32, sample_rate: f32) {
        let c = speed_of_sound(gamma, gas_constant, temperature);
        for p in &mut self.primaries {
            p.tune(gamma, gas_constant, temperature);
        }
        for coll in &mut self.collectors {
            coll.tune(gamma, gas_constant, temperature);
        }
        self.crossover.tune(gamma, gas_constant, temperature);
        for p in &mut self.pre_cross_pipes {
            p.tune(gamma, gas_constant, temperature);
        }
        for chain in &mut self.silencers {
            for element in chain {
                element.tune(gamma, gas_constant, temperature);
            }
        }
        for tp in &mut self.tailpipes {
            tp.tune(gamma, gas_constant, temperature);
        }
        for m in &mut self.mouths {
            m.tune(c, sample_rate);
        }
    }

    /// Number of banks the network radiates from.
    pub fn bank_count(&self) -> usize {
        self.bank_count
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

    /// Damping currently applied to a cylinder's valve end, `0..=1` [-].
    pub fn valve_damping(&self, cylinder: usize) -> f32 {
        self.valves[cylinder.min(self.valves.len() - 1)].damping()
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

    /// Reflection coefficient currently at a cylinder's valve end [-].
    pub fn valve_reflection(&self, cylinder: usize) -> f32 {
        self.valves[cylinder.min(self.valves.len() - 1)].reflection()
    }

    /// Damps a primary by softening its valve-end reflection, `0` for the bare
    /// closed valve and `1` for fully damped.
    ///
    /// Until the solver's own valve lift reaches the snapshot, a primary's head
    /// is a rigid closed end at every instant, which is what a real one is for
    /// most of the cycle but never all of it: for the part of the cycle the
    /// valve is open, the pipe is looking into the cylinder and the reflection
    /// is far weaker. Scaling the reflection here stands in for that, and it is
    /// the same physical quantity a lift curve would set — see
    /// [`ValveTermination::set_effective_area`].
    pub fn set_valve_damping(&mut self, cylinder: usize, damping: f32) {
        if let Some(valve) = self.valves.get_mut(cylinder) {
            valve.set_damping(damping);
        }
    }

    /// Updates valve effective flow areas for all cylinders.
    pub fn set_valve_areas(&mut self, valve_areas: &[f64]) {
        for (v, &area) in self.valves.iter_mut().zip(valve_areas.iter()) {
            v.set_effective_area(area);
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
        for b in 0..self.bank_count {
            let mut sig = self.bank_down[b];
            let n_sil = self.silencers[b].len();
            for k in 0..n_sil {
                let downstream = self.chain_returns[b][k + 1];
                let (upstream, trans) = self.silencers[b][k].step(sig, downstream);
                self.chain_returns[b][k] = upstream;
                sig = trans;
            }

            let (p_tail_up, p_tail_exit) = self.tailpipes[b].read_outputs();
            let (p_mouth_refl, p_mouth_rad) = self.mouths[b].step(p_tail_exit);
            self.tailpipes[b].push_inputs(sig, p_mouth_refl);
            self.chain_returns[b][n_sil] = p_tail_up;
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
}
