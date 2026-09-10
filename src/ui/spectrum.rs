//! The spectrum analyser behind the audio display.
//!
//! The dashboard shows the spectrum of what the sound card is *playing*, read
//! back through [`AudioScope`]. That is a deliberate choice over the cheaper
//! alternative of drawing bars from the firing frequency and its harmonics: the
//! synth integrates crank phase itself, adds runner resonance, muffler
//! filtering, induction noise and backfires, and none of that is recoverable
//! from the snapshot that went in. A display driven from the snapshot would be
//! an illustration of the engine rather than a measurement of it, and would keep
//! showing a plausible spectrum after the audio path went silent.
//!
//! The transform is a textbook iterative radix-2 Cooley-Tukey FFT. At 1024
//! points, sixty times a second, it costs about five microseconds a frame — far
//! too little to justify a dependency, and small enough to run on the UI thread
//! between draws.

use std::f32::consts::PI;

use crate::audio::AudioScope;

/// FFT size. A power of two, as radix-2 requires.
///
/// At 48 kHz this is 21 ms of audio and 46.9 Hz of resolution — enough to
/// separate the firing orders of an idling engine (a V8 at 750 rpm fires at
/// 50 Hz) while staying short enough that the display tracks a throttle blip.
pub const FFT_SIZE: usize = 1_024;

/// Number of display bands the bins are folded into.
pub const BANDS: usize = 56;

/// Lowest frequency the display resolves [Hz].
pub const MIN_HZ: f32 = 25.0;
/// Highest frequency the display resolves [Hz].
pub const MAX_HZ: f32 = 12_000.0;

/// Bottom of the decibel scale. Anything quieter reads as an empty band.
///
/// Deep enough to keep the quiet upper harmonics visible, shallow enough that
/// the loud low orders do not all sit against the ceiling — at -78 dB the
/// bottom two octaves of a running engine were saturated and the display had no
/// shape left in it.
pub const FLOOR_DB: f32 = -66.0;

// ---------------------------------------------------------------------------
// FFT
// ---------------------------------------------------------------------------

/// In-place iterative radix-2 Cooley-Tukey FFT on split real/imaginary buffers.
///
/// `re` and `im` must be the same power-of-two length. Split buffers rather than
/// a complex type because the input is real and the output is only ever read as
/// a magnitude, so a complex number would be a struct that is destructured at
/// both ends.
fn fft_in_place(re: &mut [f32], im: &mut [f32]) {
    let n = re.len();
    debug_assert_eq!(n, im.len());
    debug_assert!(n.is_power_of_two());
    if n < 2 {
        return;
    }

    // Bit-reversal permutation, by incrementing a reversed counter rather than
    // reversing each index: the carry propagates from the high bit down.
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }

    // Danielson-Lanczos: combine pairs of length-`len/2` transforms.
    let mut len = 2;
    while len <= n {
        let angle = -2.0 * PI / len as f32;
        let (wr, wi) = (angle.cos(), angle.sin());
        for start in (0..n).step_by(len) {
            // The twiddle factor is advanced by complex multiplication rather
            // than recomputed with a trig call per butterfly. Error accumulates
            // over the block, but at this size it stays far below the display's
            // resolution.
            let (mut ur, mut ui) = (1.0f32, 0.0f32);
            for k in 0..len / 2 {
                let (a, b) = (start + k, start + k + len / 2);
                let tr = re[b] * ur - im[b] * ui;
                let ti = re[b] * ui + im[b] * ur;
                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
                let next_ur = ur * wr - ui * wi;
                ui = ur * wi + ui * wr;
                ur = next_ur;
            }
        }
        len <<= 1;
    }
}

// ---------------------------------------------------------------------------
// Analyser
// ---------------------------------------------------------------------------

/// A rolling FFT over the audio the device is playing.
///
/// Owns its scratch buffers so a frame of analysis allocates nothing.
pub struct SpectrumAnalyzer {
    /// Rolling window of tapped mono samples.
    window: Vec<f32>,
    /// Hann window coefficients, computed once.
    taper: Vec<f32>,
    /// FFT scratch.
    re: Vec<f32>,
    im: Vec<f32>,
    /// Band magnitudes in dB, smoothed across frames, `0..=1` after mapping.
    bands: Vec<f32>,
    /// Per-band peak hold, `0..=1`.
    peaks: Vec<f32>,
    /// Instantaneous RMS of the last window, `0..=1`.
    rms: f32,
    /// Peak magnitude of the last window, `0..=1`.
    peak: f32,
    /// Decaying peak hold for the level meter, `0..=1`.
    peak_hold: f32,
    /// Sample rate of the tapped stream [Hz].
    sample_rate: f32,
    /// Whether any audio has ever arrived.
    live: bool,
}

impl Default for SpectrumAnalyzer {
    fn default() -> Self {
        Self::new(48_000.0)
    }
}

impl SpectrumAnalyzer {
    /// Builds an analyser for a stream at `sample_rate`.
    pub fn new(sample_rate: f32) -> Self {
        Self {
            window: Vec::with_capacity(FFT_SIZE * 2),
            // Hann: the engine note is a dense harmonic series, and a
            // rectangular window would smear each order's leakage across its
            // neighbours until the harmonic structure stopped being visible.
            taper: (0..FFT_SIZE)
                .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f32 / (FFT_SIZE - 1) as f32).cos()))
                .collect(),
            re: vec![0.0; FFT_SIZE],
            im: vec![0.0; FFT_SIZE],
            bands: vec![0.0; BANDS],
            peaks: vec![0.0; BANDS],
            rms: 0.0,
            peak: 0.0,
            peak_hold: 0.0,
            sample_rate: sample_rate.max(1.0),
            live: false,
        }
    }

    /// Retunes the analyser after the device renegotiated its rate.
    pub fn set_sample_rate(&mut self, sample_rate: f32) {
        self.sample_rate = sample_rate.max(1.0);
    }

    /// Whether the tap has ever delivered audio.
    pub fn is_live(&self) -> bool {
        self.live
    }

    /// Smoothed band magnitudes, `0..=1`, low frequency first.
    pub fn bands(&self) -> &[f32] {
        &self.bands
    }

    /// Per-band peak hold, `0..=1`.
    pub fn peaks(&self) -> &[f32] {
        &self.peaks
    }

    /// RMS level of the last analysed window, `0..=1`.
    pub fn rms(&self) -> f32 {
        self.rms
    }

    /// Peak level of the last analysed window, `0..=1`.
    pub fn peak(&self) -> f32 {
        self.peak
    }

    /// Decaying peak hold for the level meter, `0..=1`.
    pub fn peak_hold(&self) -> f32 {
        self.peak_hold
    }

    /// Centre frequency of a display band [Hz].
    pub fn band_frequency(index: usize) -> f32 {
        let t = index as f32 / (BANDS - 1).max(1) as f32;
        MIN_HZ * (MAX_HZ / MIN_HZ).powf(t)
    }

    /// Drains the scope and re-analyses, decaying the display if it is silent.
    ///
    /// Call once per rendered frame. `dt` is the frame time [s], used for the
    /// decay rates so the display falls at the same speed whatever the frame
    /// rate is.
    pub fn update(&mut self, scope: Option<&mut AudioScope>, dt: f32) {
        if let Some(scope) = scope {
            if scope.drain_into(&mut self.window, FFT_SIZE) > 0 {
                self.live = true;
            }
        }

        // Decay towards silence rather than snapping there, so a frame that
        // arrives late does not read as a dropout on the display.
        let decay = (-dt / 0.09).exp();
        let peak_decay = (-dt / 0.55).exp();

        if self.window.len() < FFT_SIZE {
            self.fade(decay, peak_decay);
            return;
        }
        self.analyse(decay, peak_decay);
    }

    /// Runs one transform over the current window.
    fn analyse(&mut self, decay: f32, peak_decay: f32) {
        let start = self.window.len() - FFT_SIZE;
        let block = &self.window[start..];

        let mut sum_squares = 0.0f32;
        let mut peak = 0.0f32;
        for (i, &sample) in block.iter().enumerate() {
            sum_squares += sample * sample;
            peak = peak.max(sample.abs());
            self.re[i] = sample * self.taper[i];
            self.im[i] = 0.0;
        }
        self.rms = (sum_squares / FFT_SIZE as f32).sqrt();
        self.peak = peak;
        self.peak_hold = (self.peak_hold * peak_decay).max(peak);

        fft_in_place(&mut self.re, &mut self.im);

        // Only the first half of the spectrum is independent for a real signal;
        // the rest is its mirror image.
        let bins = FFT_SIZE / 2;
        let hz_per_bin = self.sample_rate / FFT_SIZE as f32;
        // A Hann window halves the coherent gain and the transform is unscaled,
        // so undo both to get back to something proportional to amplitude.
        let scale = 4.0 / FFT_SIZE as f32;

        for band in 0..BANDS {
            // Each band spans from halfway to its lower neighbour to halfway to
            // its upper one, in log space — geometric rather than arithmetic
            // midpoints, because the bands themselves are geometrically spaced.
            let centre = Self::band_frequency(band);
            let lower = if band == 0 {
                MIN_HZ
            } else {
                (centre * Self::band_frequency(band - 1)).sqrt()
            };
            let upper = if band + 1 == BANDS {
                MAX_HZ
            } else {
                (centre * Self::band_frequency(band + 1)).sqrt()
            };

            let first = ((lower / hz_per_bin).floor() as usize).clamp(1, bins - 1);
            let last = ((upper / hz_per_bin).ceil() as usize).clamp(first + 1, bins);

            // The maximum, not the mean, across the bins in a band. At the low
            // end a band is narrower than one bin and the two agree; at the high
            // end a band covers dozens, and averaging would bury a single strong
            // harmonic under its silent neighbours.
            let mut magnitude = 0.0f32;
            for bin in first..last {
                let power = self.re[bin] * self.re[bin] + self.im[bin] * self.im[bin];
                magnitude = magnitude.max(power.sqrt() * scale);
            }

            let db = 20.0 * magnitude.max(1e-9).log10();
            let level = ((db - FLOOR_DB) / -FLOOR_DB).clamp(0.0, 1.0);

            // Fast attack, slow release: a backfire should appear on the frame
            // it happens, and stay visible long enough to be seen.
            self.bands[band] = if level > self.bands[band] {
                level
            } else {
                self.bands[band] * decay + level * (1.0 - decay)
            };
            self.peaks[band] = (self.peaks[band] * peak_decay).max(self.bands[band]);
        }
    }

    /// Lets the display fall when there is nothing to analyse.
    fn fade(&mut self, decay: f32, peak_decay: f32) {
        for band in 0..BANDS {
            self.bands[band] *= decay;
            self.peaks[band] = (self.peaks[band] * peak_decay).max(self.bands[band]);
        }
        self.rms *= decay;
        self.peak *= decay;
        self.peak_hold *= peak_decay;
    }

    /// Clears everything, for a stream that has been replaced.
    pub fn reset(&mut self) {
        self.window.clear();
        self.bands.iter_mut().for_each(|b| *b = 0.0);
        self.peaks.iter_mut().for_each(|p| *p = 0.0);
        self.rms = 0.0;
        self.peak = 0.0;
        self.peak_hold = 0.0;
        self.live = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Magnitude spectrum of a real signal, as bin -> amplitude.
    fn spectrum_of(signal: &[f32]) -> Vec<f32> {
        let mut re = signal.to_vec();
        let mut im = vec![0.0; signal.len()];
        fft_in_place(&mut re, &mut im);
        (0..signal.len() / 2)
            .map(|i| (re[i] * re[i] + im[i] * im[i]).sqrt() * 2.0 / signal.len() as f32)
            .collect()
    }

    #[test]
    fn a_pure_tone_lands_in_its_own_bin() {
        // Exactly 8 cycles across the block, so the tone falls on bin 8 with no
        // leakage and the amplitude can be checked directly.
        let n = 256;
        let signal: Vec<f32> = (0..n)
            .map(|i| (2.0 * PI * 8.0 * i as f32 / n as f32).sin())
            .collect();
        let spectrum = spectrum_of(&signal);

        let peak = spectrum
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap();
        assert_eq!(peak.0, 8, "the tone landed in bin {}", peak.0);
        assert!(
            (peak.1 - 1.0).abs() < 1e-3,
            "amplitude came back as {:.4}, not 1.0",
            peak.1
        );
        // Everything else should be numerical dust.
        for (bin, &magnitude) in spectrum.iter().enumerate() {
            if bin != 8 {
                assert!(magnitude < 1e-4, "bin {bin} leaked {magnitude:.6}");
            }
        }
    }

    #[test]
    fn two_tones_are_resolved_separately() {
        let n = 512;
        let signal: Vec<f32> = (0..n)
            .map(|i| {
                let t = i as f32 / n as f32;
                (2.0 * PI * 10.0 * t).sin() + 0.5 * (2.0 * PI * 40.0 * t).sin()
            })
            .collect();
        let spectrum = spectrum_of(&signal);
        assert!((spectrum[10] - 1.0).abs() < 1e-3);
        assert!((spectrum[40] - 0.5).abs() < 1e-3);
        assert!(spectrum[25] < 1e-4, "energy appeared where there was none");
    }

    #[test]
    fn dc_and_silence_behave() {
        let flat = vec![1.0f32; 64];
        let spectrum = spectrum_of(&flat);
        assert!((spectrum[0] - 2.0).abs() < 1e-4, "DC is not in bin 0");
        assert!(spectrum[1..].iter().all(|&m| m < 1e-5));

        let silence = vec![0.0f32; 64];
        assert!(spectrum_of(&silence).iter().all(|&m| m == 0.0));
    }

    #[test]
    fn bands_span_the_display_range_geometrically() {
        assert!((SpectrumAnalyzer::band_frequency(0) - MIN_HZ).abs() < 1e-3);
        assert!((SpectrumAnalyzer::band_frequency(BANDS - 1) - MAX_HZ).abs() < 1.0);
        // Geometric spacing means a constant ratio between neighbours.
        let ratio = SpectrumAnalyzer::band_frequency(1) / SpectrumAnalyzer::band_frequency(0);
        let late = SpectrumAnalyzer::band_frequency(BANDS - 1)
            / SpectrumAnalyzer::band_frequency(BANDS - 2);
        assert!((ratio - late).abs() < 1e-3);
    }

    #[test]
    fn a_tone_pushed_through_the_analyser_shows_up_in_the_right_band() {
        let mut analyzer = SpectrumAnalyzer::new(48_000.0);
        // 1 kHz at half scale.
        for i in 0..FFT_SIZE {
            let t = i as f32 / 48_000.0;
            analyzer.window.push(0.5 * (2.0 * PI * 1_000.0 * t).sin());
        }
        analyzer.analyse(0.0, 0.0);

        let loudest = analyzer
            .bands()
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        let hz = SpectrumAnalyzer::band_frequency(loudest);
        assert!(
            (hz - 1_000.0).abs() / 1_000.0 < 0.15,
            "1 kHz was reported as {hz:.0} Hz"
        );
        assert!(
            (analyzer.rms() - 0.5 / 2.0f32.sqrt()).abs() < 0.02,
            "RMS of a half-scale sine came back as {:.3}",
            analyzer.rms()
        );
        assert!((analyzer.peak() - 0.5).abs() < 0.01);
    }

    #[test]
    fn silence_fades_the_display_instead_of_dropping_it() {
        let mut analyzer = SpectrumAnalyzer::new(48_000.0);
        for i in 0..FFT_SIZE {
            let t = i as f32 / 48_000.0;
            analyzer.window.push((2.0 * PI * 400.0 * t).sin());
        }
        analyzer.analyse(0.0, 1.0);
        let lit = analyzer.bands().iter().copied().fold(0.0f32, f32::max);
        assert!(lit > 0.5, "a full-scale tone barely registered: {lit:.3}");

        // No new audio: the display should sag towards zero over a few frames,
        // not blank on the first one.
        analyzer.window.clear();
        analyzer.update(None, 1.0 / 60.0);
        let after_one = analyzer.bands().iter().copied().fold(0.0f32, f32::max);
        assert!(after_one < lit && after_one > 0.5 * lit, "decay too abrupt");

        for _ in 0..120 {
            analyzer.update(None, 1.0 / 60.0);
        }
        assert!(analyzer.bands().iter().copied().fold(0.0f32, f32::max) < 0.01);
    }
}
