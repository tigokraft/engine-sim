//! The timbre regression: what every engine in the catalogue sounds like, as a
//! handful of numbers, checked on every test run.
//!
//! Stage 0 made timbre measurable and Stage 16 made it comparable. This is the
//! part that runs in CI: one fixed sweep per preset,
//! [`script::calibration_sweep`](crate::analysis::script::calibration_sweep),
//! reduced to a fingerprint — the balance between its order lines, the
//! frequencies of its strongest resonances, and the slope of its noise floor —
//! and compared against the fingerprint recorded in [`RECORDED`].
//!
//! # Why a recorded fingerprint and not an analytic one
//!
//! Everything else in this crate is tested against a result that can be derived:
//! a closed-open pipe resonates at `c/4L`, a junction reflects
//! `(A1-A2)/(A1+A2)`, a hotter pipe raises every mode by `sqrt(T2/T1)`. Those
//! tests are the reason the model is physical, and they stay the primary ones.
//!
//! They cannot catch a change in *timbre*, though, because timbre is what the
//! whole network does at once. A stage that reorders two filters, loses a
//! reflection, or lets a smoothing constant swallow a transient will leave every
//! analytic test green, produce no discontinuity, no NaN and no dropout, and
//! sound different. That is the failure this module exists to catch, and the
//! only reference for it is what the build sounded like yesterday.
//!
//! So this is a regression, not a claim of correctness. A number here being
//! matched says the timbre has not drifted; it says nothing about whether the
//! timbre is right. What it is *right* against lives in
//! `docs/measurements/calibration.md`, and where the two disagree the
//! calibration is the one that is about reality.
//!
//! # When it fails
//!
//! A failure is not a bug on its own — a stage that deliberately changes the
//! exhaust network is *supposed* to move these numbers. The discipline is:
//!
//! 1. Look at what moved and satisfy yourself that the change accounts for it.
//!    A stage that rebuilt the silencer chain moving a chamber pass band is
//!    expected; the same stage moving the block's bending mode is not.
//! 2. Re-record with `cargo run --release --example calibrate -- --fingerprints`
//!    and paste the table it prints over [`RECORDED`].
//! 3. Re-record `docs/measurements/calibration.md` in the same commit, so the
//!    numbers a person reads and the numbers the tests check are the same
//!    numbers.
//!
//! Never widen a tolerance to make a failure go away. The tolerances here are
//! the reproducibility of the harness, not a margin for taste: a render is
//! bit-identical between two runs of one build, so a level that moves by more
//! than a decibel moved because the audio did.

use crate::analysis::orders::{
    self, crest_db, half_orders, octave_bands, AverageSpectrum, Peak, SILENCE_DB,
};
use crate::analysis::render::RenderPlan;
use crate::analysis::script;
use crate::bench::EnginePreset;

/// How far an order's level may drift before it is a regression [dB].
///
/// A render is bit-identical between two runs of one build, so this is not run
/// to run scatter — it is the margin for a different compiler, a different
/// architecture's floating point, and a difference in the last bit of a
/// transcendental. A decibel and a half is far under the smallest difference a
/// person hears as a change in timbre, and far over anything arithmetic does.
pub const LEVEL_TOLERANCE_DB: f64 = 1.5;

/// How far a resonance may drift before it is a regression [%].
///
/// Two per cent is a third of an analysis bin at 300 Hz and a fifth of a
/// semitone: under what a listener would call the same note, over what the peak
/// interpolator's own precision is.
pub const RESONANCE_TOLERANCE_PCT: f64 = 2.0;

/// How far the noise floor's slope may drift before it is a regression
/// [dB/octave].
pub const TILT_TOLERANCE_DB: f64 = 1.0;

/// Relative level under which an order is only held to being quiet [dB].
///
/// An order forty decibels under the firing order is inaudible under it, and
/// its exact level is a property of the noise floor rather than of the engine.
/// Below this the regression asks only that it still be that quiet, which is a
/// real test — an order that climbs out of the floor is exactly the kind of
/// drift worth catching — without pinning a number that means nothing.
pub const QUIET_FLOOR_DB: f64 = -40.0;

/// Resonances a fingerprint records.
///
/// The three most prominent: the ones a person would name if asked what the
/// engine sounds like. Further down the list the peaks are real but crowded,
/// and which of two neighbours is the taller is not a stable thing to assert.
pub const RESONANCES: usize = 3;

/// Band a fingerprint's resonances are taken from [Hz].
///
/// The plumbing's own range. Below fifty hertz an eight-second sweep has too
/// few frames to tell a mode from the first order it dragged past; above four
/// kilohertz there is no pipe in an exhaust system whose fundamental reaches,
/// and what stands proud up there is the radiation and loss shaping — genuine
/// peaks, high prominence, because the floor beneath them is so low, and no
/// part of what anyone would call the engine's note. Without the bound the
/// fingerprint of a V8 anchors on a mode at twenty-one kilohertz.
pub const RESONANCE_BAND_HZ: (f64, f64) = (50.0, 4_000.0);

/// Band the noise floor's tilt is taken over [Hz].
///
/// The same band `examples/calibrate.rs` reports, so the two figures are the
/// same figure.
pub const TILT_BAND_HZ: (f64, f64) = (200.0, 12_000.0);

/// How far an octave band's energy share may drift before it is a regression
/// [dB].
///
/// A starting figure, not a derived one — unlike [`LEVEL_TOLERANCE_DB`] this
/// quantity has no history of run-to-run scatter to measure yet, since this is
/// the first stage that reads it. Two decibels is comfortably over the sub-dB
/// noise a compiler or an architecture change can introduce and comfortably
/// under the seven-to-eight decibel moves this plan exists to catch.
pub const BAND_TOLERANCE_DB: f64 = 2.0;

/// How far crest factor may drift before it is a regression [dB].
///
/// Also a starting figure. A decibel and a half is on the same order as
/// [`LEVEL_TOLERANCE_DB`], and far under the twenty decibels the transients
/// have been measured to have lost.
pub const CREST_TOLERANCE_DB: f64 = 1.5;

/// What one preset sounds like, as numbers.
#[derive(Debug, Clone)]
pub struct Fingerprint {
    /// The preset's name, as [`EnginePreset::name`] gives it.
    pub preset: &'static str,
    /// Its firing order: one firing per cylinder every two revolutions.
    pub firing_order: f64,
    /// Each half-order up to twice the firing order, and its line level
    /// relative to the firing order's [dB].
    ///
    /// [`SILENCE_DB`] for an order whose line could not be measured at all.
    pub balance: &'static [(f64, f64)],
    /// The most prominent resonances in the sweep average [Hz].
    pub resonances: &'static [f64],
    /// Slope of the noise floor over [`TILT_BAND_HZ`] [dB/octave].
    pub tilt_db_per_octave: f64,
    /// Energy share per octave band, 31.5 Hz through 16 kHz, relative to the
    /// render's total energy [dB]. See [`orders::octave_bands`].
    pub octave_share: &'static [f64; 10],
    /// Crest factor of the whole render, `20 log10(peak / rms)` [dB]. See
    /// [`orders::crest_db`].
    pub crest_db: f64,
}

/// The same, freshly measured, with the strings owned by the measurement.
#[derive(Debug, Clone)]
pub struct Measured {
    /// The preset measured.
    pub preset: &'static str,
    /// Its firing order.
    pub firing_order: f64,
    /// Line levels relative to the firing order [dB].
    pub balance: Vec<(f64, f64)>,
    /// The most prominent peaks, in prominence order.
    pub resonances: Vec<Peak>,
    /// Slope of the noise floor [dB/octave].
    pub tilt_db_per_octave: f64,
    /// Energy share per octave band, 31.5 Hz through 16 kHz [dB].
    pub octave_share: [f64; 10],
    /// Crest factor of the whole render [dB].
    pub crest_db: f64,
}

/// Renders one preset through the calibration sweep and reduces it to numbers.
///
/// The same render, the same sweep and the same analysis
/// `examples/calibrate.rs` uses, so a number here can be read straight off the
/// calibration table and back.
pub fn measure(preset: &EnginePreset) -> Measured {
    let firing_order = preset.firing.len() as f64 / 2.0;
    let script = script::calibration_sweep(preset);
    let render = RenderPlan::new(preset, &script).render();
    let mono = render.mono();

    let table = orders::track(&mono, render.sample_rate, &render.rpm, &half_orders());
    let balance = table.line_balance(firing_order);
    let spectrum = AverageSpectrum::of(&mono, render.sample_rate);

    // Prominence order, not frequency order: the fingerprint is the loudest few
    // and it has to stay the loudest few for a comparison to line up.
    let mut resonances: Vec<Peak> = spectrum
        .peaks(usize::MAX >> 1, orders::MIN_PROMINENCE_DB)
        .into_iter()
        .filter(|peak| (RESONANCE_BAND_HZ.0..=RESONANCE_BAND_HZ.1).contains(&peak.hz))
        .collect();
    resonances.sort_by(|a, b| b.prominence_db.total_cmp(&a.prominence_db));
    resonances.truncate(RESONANCES);

    Measured {
        preset: preset.name,
        firing_order,
        balance: balance
            .levels
            .iter()
            .filter(|level| level.order <= 2.0 * firing_order)
            .map(|level| (level.order, level.relative_db.unwrap_or(SILENCE_DB)))
            .collect(),
        resonances,
        tilt_db_per_octave: spectrum
            .tilt_db_per_octave(TILT_BAND_HZ.0, TILT_BAND_HZ.1)
            .unwrap_or(0.0),
        octave_share: octave_bands(&mono, render.sample_rate),
        crest_db: crest_db(&mono),
    }
}

impl Measured {
    /// The recorded fingerprint for this preset, if there is one.
    pub fn recorded(&self) -> Option<&'static Fingerprint> {
        RECORDED.iter().find(|f| f.preset == self.preset)
    }

    /// Everything that has drifted past tolerance since the fingerprint was
    /// recorded, as lines a failure can print.
    ///
    /// Empty is a pass. Each line names the number, what it was, what it is,
    /// and by how much it moved, because a regression report that says only
    /// "timbre changed" sends the next person back to the spectrum analyser.
    pub fn drift(&self, recorded: &Fingerprint) -> Vec<String> {
        let mut drifted = Vec::new();

        if (self.firing_order - recorded.firing_order).abs() > 1e-9 {
            drifted.push(format!(
                "firing order is now {} where it was {}",
                self.firing_order, recorded.firing_order
            ));
        }

        for &(order, was) in recorded.balance {
            let Some(&(_, now)) = self.balance.iter().find(|(o, _)| (o - order).abs() < 1e-9)
            else {
                drifted.push(format!("order {order} is no longer measured at all"));
                continue;
            };
            // A quiet order is held to being quiet rather than to a number: its
            // exact level is the noise floor's, not the engine's. `was` is a
            // recorded literal carrying one decimal digit, so anything within
            // half that last digit of the floor prints as exactly
            // `QUIET_FLOOR_DB` whether it sits a hair above or below it — the
            // margin below absorbs that rounding rather than reading it as a
            // climb.
            if was <= QUIET_FLOOR_DB {
                if now > QUIET_FLOOR_DB + 0.05 {
                    drifted.push(format!(
                        "order {order} has climbed out of the floor: {was:.1} dB to {now:.1} dB"
                    ));
                }
                continue;
            }
            if (now - was).abs() > LEVEL_TOLERANCE_DB {
                drifted.push(format!(
                    "order {order} moved {:+.1} dB: {was:.1} dB to {now:.1} dB",
                    now - was
                ));
            }
        }

        for (i, &was) in recorded.resonances.iter().enumerate() {
            let Some(peak) = self.resonances.get(i) else {
                drifted.push(format!(
                    "resonance {} of {} is gone; it was at {was:.1} Hz",
                    i + 1,
                    recorded.resonances.len()
                ));
                continue;
            };
            let error = 100.0 * (peak.hz - was) / was.max(f64::MIN_POSITIVE);
            if error.abs() > RESONANCE_TOLERANCE_PCT {
                drifted.push(format!(
                    "resonance {} moved {error:+.1} %: {was:.1} Hz to {:.1} Hz",
                    i + 1,
                    peak.hz
                ));
            }
        }

        if (self.tilt_db_per_octave - recorded.tilt_db_per_octave).abs() > TILT_TOLERANCE_DB {
            drifted.push(format!(
                "the noise floor tilted {:+.1} dB/octave: {:.1} to {:.1}",
                self.tilt_db_per_octave - recorded.tilt_db_per_octave,
                recorded.tilt_db_per_octave,
                self.tilt_db_per_octave,
            ));
        }

        for (i, (&was, &now)) in recorded
            .octave_share
            .iter()
            .zip(self.octave_share.iter())
            .enumerate()
        {
            if (now - was).abs() > BAND_TOLERANCE_DB {
                drifted.push(format!(
                    "the {} Hz band moved {:+.1} dB: {was:.1} dB to {now:.1} dB",
                    orders::OCTAVE_CENTERS_HZ[i],
                    now - was,
                ));
            }
        }

        if (self.crest_db - recorded.crest_db).abs() > CREST_TOLERANCE_DB {
            drifted.push(format!(
                "crest factor moved {:+.1} dB: {:.1} dB to {:.1} dB",
                self.crest_db - recorded.crest_db,
                recorded.crest_db,
                self.crest_db,
            ));
        }

        drifted
    }

    /// The measurement as a [`Fingerprint`] literal, ready to paste into
    /// [`RECORDED`].
    ///
    /// What `cargo run --release --example calibrate -- --fingerprints` prints.
    /// Re-recording is a deliberate act with a diff a reviewer can read, which
    /// is the point of keeping the numbers in source rather than in a file the
    /// tests rewrite.
    pub fn literal(&self) -> String {
        use std::fmt::Write as _;
        let mut out = String::new();
        let _ = writeln!(out, "    Fingerprint {{");
        let _ = writeln!(out, "        preset: {:?},", self.preset);
        let _ = writeln!(out, "        firing_order: {:?},", self.firing_order);
        let _ = write!(out, "        balance: &[");
        for (i, (order, db)) in self.balance.iter().enumerate() {
            if i % 4 == 0 {
                let _ = write!(out, "\n            ");
            }
            let _ = write!(out, "({order:?}, {db:.1}), ");
        }
        let _ = writeln!(out, "\n        ],");
        let _ = write!(out, "        resonances: &[");
        for peak in &self.resonances {
            let _ = write!(out, "{:.1}, ", peak.hz);
        }
        let _ = writeln!(out, "],");
        let _ = writeln!(
            out,
            "        tilt_db_per_octave: {:.1},",
            self.tilt_db_per_octave
        );
        let _ = write!(out, "        octave_share: &[");
        for (i, share) in self.octave_share.iter().enumerate() {
            if i % 5 == 0 {
                let _ = write!(out, "\n            ");
            }
            let _ = write!(out, "{share:.1}, ");
        }
        let _ = writeln!(out, "\n        ],");
        let _ = writeln!(out, "        crest_db: {:.1},", self.crest_db);
        let _ = writeln!(out, "    }},");
        out
    }
}

/// What the catalogue sounded like when this table was last recorded.
///
/// Recorded by `cargo run --release --example calibrate -- --fingerprints` on
/// the commit that added it, at 48 kHz with the solver at 240 Hz, through
/// [`script::calibration_sweep`](crate::analysis::script::calibration_sweep).
/// Every figure is relative — an order against the firing order, a frequency
/// against itself, a slope against an octave — so none of it moves when a
/// master gain does, and a change here is a change in timbre.
///
/// # Stage T1's bisect
///
/// Every figure here, including `octave_share` and `crest_db`, is now what
/// `--fingerprints` actually printed — the placeholder reconstruction this
/// comment used to describe is gone. T1's bisect ran the four suspects in
/// `docs/TIMBRE_PLAN.md` in order: viscothermal wall loss was already correct
/// (it was recalibrated before Stage T0 ever measured a "today" figure, and
/// the regression persisted anyway); radiation corner placement is already
/// where the geometry puts it, and moving it in either direction cost crest
/// factor rather than recovering it; the output clipper is not reached hard
/// enough by the calibration sweep to matter, bypassing it entirely moved
/// nothing. The fourth suspect was it: `structure_level` in
/// [`SynthConfig`](crate::audio::dsp::SynthConfig) was `0.07`, calibrated
/// once, before the exhaust's wall loss was fitted to a measured pipe, before
/// the network was rebuilt to lose what a real one loses, and before Stage T5
/// gave the silenced presets a real loss term — three stages that each made
/// the exhaust louder without `structure_level` moving to match. Turning it
/// down to `0.05` is the whole fix; see its doc comment for the measurement.
pub const RECORDED: &[Fingerprint] = &[
    Fingerprint {
        preset: "Inline-4",
        firing_order: 2.0,
        balance: &[
            (0.5, -21.0), (1.0, -16.3), (1.5, -29.9), (2.0, 0.0), 
            (2.5, -38.6), (3.0, -28.4), (3.5, -200.0), (4.0, -6.0), 
        ],
        resonances: &[1557.5, 3752.6, 3022.3, ],
        tilt_db_per_octave: -10.6,
        octave_share: &[
            -18.8, -13.9, -6.8, -2.7, -8.8, 
            -13.1, -20.6, -29.9, -39.0, -51.3, 
        ],
        crest_db: 12.6,
    },
    Fingerprint {
        preset: "Cross-plane V8",
        firing_order: 4.0,
        balance: &[
            (0.5, -11.7), (1.0, -13.1), (1.5, -0.2), (2.0, -14.9), 
            (2.5, 2.0), (3.0, -1.2), (3.5, 1.7), (4.0, 0.0), 
            (4.5, -21.2), (5.0, -29.7), (5.5, -3.0), (6.0, -25.3), 
            (6.5, 0.1), (7.0, -29.6), (7.5, -45.4), (8.0, -1.9), 
        ],
        resonances: &[139.3, 1535.2, 3901.0, ],
        tilt_db_per_octave: -10.9,
        octave_share: &[
            -18.5, -10.7, -1.9, -8.1, -12.3, 
            -14.8, -28.8, -34.1, -43.1, -53.7, 
        ],
        crest_db: 15.6,
    },
    Fingerprint {
        preset: "Big Cam Chopping V8",
        firing_order: 4.0,
        balance: &[
            (0.5, -20.1), (1.0, -22.2), (1.5, 4.2), (2.0, -9.9), 
            (2.5, 12.1), (3.0, -24.3), (3.5, -18.2), (4.0, 0.0), 
            (4.5, -22.8), (5.0, -46.4), (5.5, -1.2), (6.0, -200.0), 
            (6.5, 5.7), (7.0, -200.0), (7.5, -200.0), (8.0, -3.7), 
        ],
        resonances: &[192.6, 659.2, 2003.8, ],
        tilt_db_per_octave: -10.4,
        octave_share: &[
            -34.7, -23.9, -7.2, -1.5, -11.1, 
            -16.4, -28.4, -37.4, -45.0, -59.2, 
        ],
        crest_db: 14.7,
    },
    Fingerprint {
        preset: "Flat-plane V8",
        firing_order: 4.0,
        balance: &[
            (0.5, -12.6), (1.0, -15.3), (1.5, -21.9), (2.0, 0.6), 
            (2.5, -40.4), (3.0, -17.7), (3.5, -26.9), (4.0, 0.0), 
            (4.5, -27.9), (5.0, -25.9), (5.5, -28.6), (6.0, -20.4), 
            (6.5, -29.8), (7.0, -35.0), (7.5, -200.0), (8.0, -9.7), 
        ],
        resonances: &[2081.2, 2819.0, 1071.9, ],
        tilt_db_per_octave: -9.7,
        octave_share: &[
            -16.8, -9.4, -4.8, -6.0, -6.5, 
            -14.7, -21.1, -29.7, -39.6, -49.6, 
        ],
        crest_db: 15.7,
    },
    Fingerprint {
        preset: "V10",
        firing_order: 5.0,
        balance: &[
            (0.5, -6.6), (1.0, 14.7), (1.5, -19.8), (2.0, -8.7), 
            (2.5, 1.8), (3.0, 3.0), (3.5, -20.0), (4.0, -17.9), 
            (4.5, -37.7), (5.0, 0.0), (5.5, -28.6), (6.0, -22.9), 
            (6.5, -29.2), (7.0, -22.5), (7.5, -23.1), (8.0, -8.7), 
            (8.5, -32.8), (9.0, -23.9), (9.5, -40.4), (10.0, -11.1), 
        ],
        resonances: &[132.0, 321.7, 1022.3, ],
        tilt_db_per_octave: -7.9,
        octave_share: &[
            -20.0, -6.6, -2.4, -12.4, -10.3, 
            -18.3, -15.3, -27.2, -44.3, -55.1, 
        ],
        crest_db: 14.9,
    },
    Fingerprint {
        preset: "V12",
        firing_order: 6.0,
        balance: &[
            (0.5, -16.9), (1.0, -21.2), (1.5, -22.7), (2.0, -21.1), 
            (2.5, -17.7), (3.0, 7.8), (3.5, 4.7), (4.0, -1.0), 
            (4.5, -22.5), (5.0, -20.6), (5.5, -200.0), (6.0, 0.0), 
            (6.5, -200.0), (7.0, -32.2), (7.5, -32.5), (8.0, -21.5), 
            (8.5, -31.5), (9.0, -10.9), (9.5, -27.0), (10.0, -24.4), 
            (10.5, -29.8), (11.0, -26.8), (11.5, -31.2), (12.0, -8.1), 
        ],
        resonances: &[1656.4, 480.8, 823.1, ],
        tilt_db_per_octave: -11.8,
        octave_share: &[
            -26.6, -20.5, -18.2, -3.8, -3.4, 
            -11.3, -17.2, -29.3, -38.1, -50.1, 
        ],
        crest_db: 17.0,
    },
    Fingerprint {
        preset: "2-Rotor Wankel",
        firing_order: 2.0,
        balance: &[
            (0.5, -19.1), (1.0, -22.4), (1.5, -33.2), (2.0, 0.0), 
            (2.5, -26.1), (3.0, -24.7), (3.5, -43.0), (4.0, -0.5), 
        ],
        resonances: &[213.3, 670.2, 1856.8, ],
        tilt_db_per_octave: -9.5,
        octave_share: &[
            -26.1, -13.8, -15.3, -5.3, -4.1, 
            -6.3, -22.0, -35.2, -44.5, -55.7, 
        ],
        crest_db: 15.3,
    },
    Fingerprint {
        preset: "Turbo Inline-4",
        firing_order: 2.0,
        balance: &[
            (0.5, -31.4), (1.0, -26.0), (1.5, -200.0), (2.0, 0.0), 
            (2.5, -46.4), (3.0, -34.7), (3.5, -35.7), (4.0, -10.1), 
        ],
        resonances: &[1645.2, 200.2, 394.5, ],
        tilt_db_per_octave: -8.2,
        octave_share: &[
            -20.5, -11.9, -6.9, -2.1, -12.0, 
            -15.4, -24.5, -28.2, -27.0, -42.3, 
        ],
        crest_db: 12.7,
    },
    Fingerprint {
        preset: "Twin-turbo V8",
        firing_order: 4.0,
        balance: &[
            (0.5, -20.7), (1.0, -19.4), (1.5, -20.8), (2.0, -31.1), 
            (2.5, -16.5), (3.0, -49.4), (3.5, -19.2), (4.0, 0.0), 
            (4.5, -38.3), (5.0, -26.1), (5.5, -24.8), (6.0, -28.6), 
            (6.5, -19.4), (7.0, -33.6), (7.5, -48.8), (8.0, -15.7), 
        ],
        resonances: &[1112.8, 3021.8, 1551.4, ],
        tilt_db_per_octave: -7.7,
        octave_share: &[
            -19.2, -9.3, -9.7, -7.6, -3.3, 
            -10.2, -22.9, -19.6, -32.7, -50.0, 
        ],
        crest_db: 13.7,
    },
    Fingerprint {
        preset: "Turbo Inline-6",
        firing_order: 3.0,
        balance: &[
            (0.5, -21.1), (1.0, -26.5), (1.5, -28.5), (2.0, -33.9), 
            (2.5, -200.0), (3.0, 0.0), (3.5, -42.3), (4.0, -37.4), 
            (4.5, -39.1), (5.0, -34.6), (5.5, -200.0), (6.0, -7.4), 
        ],
        resonances: &[137.6, 1727.7, 698.3, ],
        tilt_db_per_octave: -10.1,
        octave_share: &[
            -16.9, -13.7, -2.4, -5.2, -14.1, 
            -18.4, -27.4, -36.5, -49.8, -64.3, 
        ],
        crest_db: 13.2,
    },
    Fingerprint {
        preset: "Turbodiesel I4",
        firing_order: 2.0,
        balance: &[
            (0.5, -4.2), (1.0, 3.2), (1.5, -11.1), (2.0, 0.0), 
            (2.5, -21.3), (3.0, -7.2), (3.5, -200.0), (4.0, -2.9), 
        ],
        resonances: &[399.9, 3592.6, 1680.3, ],
        tilt_db_per_octave: -4.3,
        octave_share: &[
            -14.0, -7.8, -4.5, -11.8, -6.5, 
            -16.3, -23.7, -10.5, -19.9, -43.3, 
        ],
        crest_db: 13.7,
    },
    Fingerprint {
        preset: "Big Single",
        firing_order: 0.5,
        balance: &[
            (0.5, 0.0), (1.0, 12.4), 
        ],
        resonances: &[115.5, 1287.7, 551.7, ],
        tilt_db_per_octave: -8.6,
        octave_share: &[
            -22.5, -12.5, -2.0, -13.4, -8.8, 
            -9.1, -21.8, -24.9, -42.0, -57.1, 
        ],
        crest_db: 14.9,
    },
    Fingerprint {
        preset: "Porsche 911 GT3 Cup (992)",
        firing_order: 3.0,
        balance: &[
            (0.5, -30.4), (1.0, -22.8), (1.5, -13.9), (2.0, -33.9), 
            (2.5, -38.0), (3.0, 0.0), (3.5, -39.8), (4.0, -39.4), 
            (4.5, -26.5), (5.0, -37.5), (5.5, -55.5), (6.0, -19.1), 
        ],
        resonances: &[111.1, 2303.9, 928.2, ],
        tilt_db_per_octave: -8.7,
        octave_share: &[
            -31.9, -14.0, -1.7, -10.4, -7.4, 
            -19.1, -27.7, -32.2, -43.0, -51.0, 
        ],
        crest_db: 14.1,
    },
    Fingerprint {
        preset: "Porsche 911 GT3 Cup (997.2)",
        firing_order: 3.0,
        balance: &[
            (0.5, -36.0), (1.0, -28.1), (1.5, -28.2), (2.0, -39.4), 
            (2.5, -200.0), (3.0, 0.0), (3.5, -53.6), (4.0, -42.9), 
            (4.5, -21.4), (5.0, -35.7), (5.5, -200.0), (6.0, -18.1), 
        ],
        resonances: &[399.9, 2014.7, 621.5, ],
        tilt_db_per_octave: -8.2,
        octave_share: &[
            -33.9, -16.7, -9.9, -9.3, -1.5, 
            -13.1, -21.8, -29.1, -40.3, -49.3, 
        ],
        crest_db: 14.9,
    },
    Fingerprint {
        preset: "Mercedes-AMG GT3",
        firing_order: 4.0,
        balance: &[
            (0.5, -11.1), (1.0, -27.5), (1.5, 2.8), (2.0, -32.8), 
            (2.5, -8.8), (3.0, -200.0), (3.5, -14.8), (4.0, 0.0), 
            (4.5, -11.4), (5.0, -200.0), (5.5, -1.2), (6.0, -16.7), 
            (6.5, -5.8), (7.0, -200.0), (7.5, -32.2), (8.0, -12.2), 
        ],
        resonances: &[105.4, 1773.4, 672.7, ],
        tilt_db_per_octave: -8.2,
        octave_share: &[
            -28.7, -11.2, -2.2, -14.9, -6.1, 
            -15.4, -21.3, -29.0, -40.8, -56.2, 
        ],
        crest_db: 12.5,
    },
    Fingerprint {
        preset: "Ferrari 458 Italia GT3",
        firing_order: 4.0,
        balance: &[
            (0.5, -14.2), (1.0, -11.7), (1.5, -25.1), (2.0, -5.9), 
            (2.5, -29.9), (3.0, -25.0), (3.5, -18.6), (4.0, 0.0), 
            (4.5, -30.6), (5.0, -28.6), (5.5, -21.2), (6.0, -17.4), 
            (6.5, -23.9), (7.0, -30.8), (7.5, -200.0), (8.0, -6.2), 
        ],
        resonances: &[2344.0, 246.9, 3469.1, ],
        tilt_db_per_octave: -8.1,
        octave_share: &[
            -22.7, -10.5, -11.1, -4.0, -6.2, 
            -8.0, -17.4, -24.4, -36.4, -47.3, 
        ],
        crest_db: 16.0,
    },
    Fingerprint {
        preset: "Audi R8 LMS GT3",
        firing_order: 5.0,
        balance: &[
            (0.5, -10.3), (1.0, 8.8), (1.5, -24.6), (2.0, -14.7), 
            (2.5, -14.2), (3.0, -0.4), (3.5, -25.9), (4.0, -25.3), 
            (4.5, -55.5), (5.0, 0.0), (5.5, -30.7), (6.0, -30.4), 
            (6.5, -33.0), (7.0, -14.0), (7.5, -25.4), (8.0, -29.0), 
            (8.5, -33.6), (9.0, -33.0), (9.5, -26.8), (10.0, -8.5), 
        ],
        resonances: &[409.8, 2026.4, 136.0, ],
        tilt_db_per_octave: -8.5,
        octave_share: &[
            -22.3, -8.2, -4.4, -9.5, -4.9, 
            -14.8, -21.6, -28.3, -41.8, -52.5, 
        ],
        crest_db: 15.1,
    },
];

#[cfg(test)]
mod tests;
