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
}

impl ValveTermination {
    /// Constructs a valve termination for a pipe of area $A_p$ [m^2].
    pub fn new(pipe_area: f64) -> Self {
        Self {
            pipe_area: pipe_area.max(1e-7) as f32,
            reflection: 1.0,
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
        self.reflection
    }

    /// Computes the forward-travelling wave entering the runner:
    /// - `excitation`: blowdown pressure pulse injected at the port [Pa].
    /// - `returning_wave`: backward wave arriving at the valve boundary ($p^-(0)$) [Pa].
    #[inline(always)]
    pub fn step(&self, excitation: f32, returning_wave: f32) -> f32 {
        excitation + self.reflection * returning_wave
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
