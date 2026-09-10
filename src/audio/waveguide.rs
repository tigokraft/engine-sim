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

use crate::audio::filters::{runner_delay_seconds, speed_of_sound, DelayLine, Smoothed};

/// Coldest gas temperature used to size delay line buffers [K].
///
/// Sound travels slowest when cold ($c \propto \sqrt{T}$), which yields the
/// largest transit delay in samples. Sizing buffers for 250 K ensures the
/// delay lines never need reallocation during runtime temperature glides.
pub const COLDEST_EXHAUST_TEMPERATURE_K: f32 = 250.0;

/// Reference atmospheric pressure for gas density estimation [Pa].
pub const REFERENCE_PRESSURE_PA: f32 = 101_325.0;

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

        Self {
            length: length_f32,
            area: area_f32,
            forward_line: DelayLine::with_max_delay(max_delay_samples),
            backward_line: DelayLine::with_max_delay(max_delay_samples),
            // 40 ms time constant matches the engine temperature glide: fast enough
            // to follow throttle snaps, slow enough that fractional interpolation
            // never produces audible doppler pitch clicks.
            delay_samples: Smoothed::new(initial_delay_samples, sample_rate, 0.040),
            admittance,
            sample_rate,
        }
    }

    /// Retunes propagation delay and acoustic admittance for current gas state.
    pub fn tune(&mut self, gamma: f32, gas_constant: f32, temperature: f32) {
        let delay_sec = runner_delay_seconds(self.length, gamma, gas_constant, temperature);
        let samples = delay_sec * self.sample_rate;
        self.delay_samples
            .set_target(samples.clamp(1.0, self.forward_line.max_delay()));

        let c = speed_of_sound(gamma, gas_constant, temperature);
        let rho = REFERENCE_PRESSURE_PA / (gas_constant * temperature.max(1.0));
        self.admittance = self.area / (rho * c).max(1e-4);
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

    /// Reads waves arriving at the boundaries from inside the pipe:
    /// - `out_port0`: wave emerging at port 0 from the backward delay line ($p^-(0)$).
    /// - `out_port1`: wave emerging at port 1 from the forward delay line ($p^+(L)$).
    #[inline(always)]
    pub fn read_outputs(&mut self) -> (f32, f32) {
        let d = self.delay_samples.next_value();
        let out0 = self.backward_line.read(d);
        let out1 = self.forward_line.read(d);
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
