//! The structural path: what the block itself radiates.
//!
//! # Why an engine needs a second radiating path
//!
//! Everything else in [`crate::audio`] is air: gas leaves a port, travels a
//! pipe, and a mouth radiates it. That is not how most of a diesel reaches the
//! listener, and it is not how any engine's idle does. Cylinder pressure acts on
//! the piston crown and on the head face, and the difference between those two
//! forces is reacted by the block — so every combustion event is also a hammer
//! blow delivered to a large lump of metal, which answers at its own modes and
//! radiates from its own surfaces. In NVH that path has a name, *combustion
//! noise*, and its governing quantity is not peak pressure but `dP/dtheta`: a
//! charge that arrives all at once puts energy at the frequencies the structure
//! actually responds at, and one that arrives gently does not. Two engines can
//! reach the same 60 bar peak and sound nothing alike for that reason alone.
//! It is the whole difference between a diesel's clatter and a petrol engine's
//! thump.
//!
//! # What is modelled here
//!
//! A filterbank of six resonances standing for three physical members, driven
//! in parallel and summed:
//!
//! - **Global bending of the dressed block**, three modes. The block with heads,
//!   pan and accessories hung on it is a beam-like lump on soft mounts, and its
//!   lowest mode is the `f = (1/2 pi) sqrt(K/m)` law
//!   [`crate::audio::filters::block_resonance_hz`] already ships — which this
//!   bank keeps exactly, as its anchor. The two above it follow the free-free
//!   Euler-Bernoulli eigenvalues.
//! - **Oil pan drumming**, one mode. A thin stamped panel over the crankcase
//!   footprint, which is the largest single radiating surface on the engine and
//!   the reason a bare block on a stand sounds nothing like one in a car.
//! - **Bore wall ovalling**, two modes. The cylinder walls are thin rings driven
//!   from inside by the same pressure, and they ring in the 2-8 kHz band. This
//!   is the bright end of clatter.
//!
//! Above the highest of those the mode count of a real block runs into the
//! hundreds and the individual peaks merge into a smooth plateau. That is
//! carried here as a residual term rather than as more filters: the bank passes
//! its input through at unity and adds the resolved modes on top, so the
//! plateau is the reference every source level in the synth is already measured
//! against and the modes are a departure from it.
//!
//! # Where the mass law lives
//!
//! An alloy block and an iron one of the same displacement differ far more in
//! density than in section, which is why the light one rings *higher* rather
//! than merely quieter. Mass therefore enters twice and in the same direction:
//! directly in the bending anchor, and through the smeared density of the block
//! envelope in the bore-wall speed of sound. Nothing in this module is typed in
//! per engine.

use crate::audio::filters::{Biquad, BiquadCoeffs};

// ---------------------------------------------------------------------------
// Bank constants
// ---------------------------------------------------------------------------

/// How far the strongest resolved mode stands above the unresolved plateau [-].
///
/// The bank's broadband transmission is unity by construction — that is the
/// plateau the hundreds of unmodelled high modes merge into — and the resolved
/// peaks rise above it by this factor. 2.5 is 8 dB, which is the order a block's
/// measured mobility shows between its low modes and its high-frequency floor
/// once the joints and mounts have done their damping.
pub const MODAL_PEAK_RATIO: f32 = 2.5;

/// Number of resolved modes in the bank.
pub const STRUCTURAL_MODES: usize = 6;

// ---------------------------------------------------------------------------
// Modes
// ---------------------------------------------------------------------------

/// One resolved structural mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StructuralMode {
    /// Resonant frequency [Hz].
    pub frequency: f32,
    /// Sharpness, `1 / loss_factor` [-].
    pub q: f32,
    /// Share of the radiated output this mode carries [-].
    pub weight: f32,
}

// ---------------------------------------------------------------------------
// The filterbank
// ---------------------------------------------------------------------------

/// The block as a radiating structure: a modal filterbank with a residual.
///
/// Input is a normalised structural force — combustion, mechanical impacts and
/// knock all arrive here as one drive signal, because the block cannot tell them
/// apart either. Output is mono: the block is one object radiating a four-metre
/// wavelength at its lowest mode, so there is nothing to image.
///
/// Retuned by nobody. Every frequency in it is geometry and mass, and neither
/// changes while the stream is open, so the bank is designed once at
/// construction and the audio thread only ever runs the difference equations.
#[derive(Debug, Clone)]
pub struct StructuralPath {
    modes: [Biquad; STRUCTURAL_MODES],
    gains: [f32; STRUCTURAL_MODES],
    frequencies: [f32; STRUCTURAL_MODES],
    qs: [f32; STRUCTURAL_MODES],
}

impl StructuralPath {
    /// Builds the bank from a mode list.
    ///
    /// The weights are taken as given and scaled to [`MODAL_PEAK_RATIO`]
    /// relative to the residual; nothing here knows what a block is.
    pub fn new(sample_rate: f32, modes: &[StructuralMode; STRUCTURAL_MODES]) -> Self {
        let mut path = Self {
            modes: [Biquad::default(); STRUCTURAL_MODES],
            gains: [0.0; STRUCTURAL_MODES],
            frequencies: [0.0; STRUCTURAL_MODES],
            qs: [0.0; STRUCTURAL_MODES],
        };
        for (i, mode) in modes.iter().enumerate() {
            let frequency = mode.frequency.clamp(5.0, 0.45 * sample_rate);
            path.modes[i] = Biquad::new(BiquadCoeffs::bandpass(sample_rate, frequency, mode.q));
            path.gains[i] = mode.weight * (MODAL_PEAK_RATIO - 1.0);
            path.frequencies[i] = frequency;
            path.qs[i] = mode.q;
        }
        path
    }

    /// Mode frequencies as built [Hz].
    pub fn mode_frequencies(&self) -> [f32; STRUCTURAL_MODES] {
        self.frequencies
    }

    /// Mode sharpnesses as built [-].
    pub fn mode_qs(&self) -> [f32; STRUCTURAL_MODES] {
        self.qs
    }

    /// Radiates one sample of structural drive.
    ///
    /// The bare input term is the residual: the mode continuum above the
    /// resolved band, which is dense enough to be flat and is what makes this a
    /// transfer function with peaks rather than a set of six tuned tones.
    #[inline(always)]
    pub fn process(&mut self, drive: f32) -> f32 {
        let mut out = drive;
        for (mode, gain) in self.modes.iter_mut().zip(self.gains.iter()) {
            out += mode.process(drive) * gain;
        }
        out
    }

    /// Clears every mode's state.
    pub fn reset(&mut self) {
        self.modes.iter_mut().for_each(Biquad::reset);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::audio::filters::Noise;
    use std::f32::consts::TAU;

    const FS: f32 = 48_000.0;

    /// Six modes spread far enough apart to be measured one at a time.
    fn spread() -> [StructuralMode; STRUCTURAL_MODES] {
        let mut modes = [StructuralMode {
            frequency: 0.0,
            q: 10.0,
            weight: 0.0,
        }; STRUCTURAL_MODES];
        for (i, mode) in modes.iter_mut().enumerate() {
            mode.frequency = 80.0 * 3.0f32.powi(i as i32);
            mode.weight = 1.0 / (i as f32 + 1.0);
        }
        modes
    }

    /// Steady-state magnitude response at one frequency, by quadrature
    /// correlation after the transient has passed.
    fn magnitude_at(path: &mut StructuralPath, frequency: f32) -> f32 {
        let n = 32_768;
        let (mut re, mut im) = (0.0f32, 0.0f32);
        for i in 0..n {
            let t = i as f32 / FS;
            let phase = TAU * frequency * t;
            let y = path.process(phase.sin());
            if i >= n / 2 {
                re += y * phase.sin();
                im += y * phase.cos();
            }
        }
        let half = (n / 2) as f32;
        2.0 * (re * re + im * im).sqrt() / half
    }

    #[test]
    fn the_residual_passes_everything_at_unity() {
        // Far from every mode the bank is a wire. That is what makes a source
        // level mean the same thing before and after the structure.
        let mut path = StructuralPath::new(FS, &spread());
        let far = magnitude_at(&mut path, 12_000.0);
        assert!(
            (far - 1.0).abs() < 0.15,
            "residual is not unity off-mode: {far}"
        );
    }

    #[test]
    fn the_strongest_mode_peaks_at_the_modal_ratio() {
        let modes = spread();
        let mut path = StructuralPath::new(FS, &modes);
        let peak = magnitude_at(&mut path, modes[0].frequency);
        assert!(
            (peak - MODAL_PEAK_RATIO).abs() < 0.2,
            "strongest mode peaked at {peak}, expected {MODAL_PEAK_RATIO}"
        );
        // And a half-weight mode stands half as far above the plateau.
        let mut path = StructuralPath::new(FS, &modes);
        let second = magnitude_at(&mut path, modes[1].frequency);
        let expected = 1.0 + 0.5 * (MODAL_PEAK_RATIO - 1.0);
        assert!(
            (second - expected).abs() < 0.2,
            "second mode peaked at {second}, expected {expected}"
        );
    }

    #[test]
    fn modes_sit_where_they_were_asked_to() {
        let modes = spread();
        let path = StructuralPath::new(FS, &modes);
        for (built, asked) in path.mode_frequencies().iter().zip(modes.iter()) {
            assert!(
                (built - asked.frequency).abs() < 1e-3,
                "built {built} Hz for a requested {} Hz",
                asked.frequency
            );
        }
        assert_eq!(path.mode_qs()[0], modes[0].q);
    }

    #[test]
    fn a_mode_above_nyquist_does_not_go_unstable() {
        let mut modes = spread();
        modes[5].frequency = 4.0 * FS;
        let mut path = StructuralPath::new(FS, &modes);
        let mut noise = Noise::new(7);
        for _ in 0..FS as usize {
            let y = path.process(noise.next_bipolar());
            assert!(y.is_finite() && y.abs() < 100.0, "unstable: {y}");
        }
    }

    #[test]
    fn the_bank_is_bounded_and_dc_free_on_its_modes() {
        // A constant drive must not leave a constant offset on the modal part:
        // every mode is a bandpass, so only the residual survives DC.
        let mut path = StructuralPath::new(FS, &spread());
        let mut last = 0.0;
        for _ in 0..FS as usize {
            last = path.process(1.0);
        }
        assert!(
            (last - 1.0).abs() < 1e-3,
            "the modes left {last} of gain at DC, expected the residual's 1.0"
        );

        // And a bounded input stays bounded.
        let mut noise = Noise::new(11);
        let mut peak = 0.0f32;
        for _ in 0..(4 * FS as usize) {
            peak = peak.max(path.process(noise.next_bipolar()).abs());
        }
        assert!(
            peak.is_finite() && peak < 8.0,
            "bank ran away on unit noise: peak {peak}"
        );
    }

    #[test]
    fn reset_clears_the_ring() {
        let mut path = StructuralPath::new(FS, &spread());
        for _ in 0..64 {
            path.process(1.0);
        }
        path.reset();
        assert_eq!(path.process(0.0), 0.0);
    }
}
