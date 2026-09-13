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
            // exact level is the noise floor's, not the engine's.
            if was <= QUIET_FLOOR_DB {
                if now > QUIET_FLOOR_DB {
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
/// # `octave_share` and `crest_db` are the one exception
///
/// Every other figure here is what `--fingerprints` actually printed. These
/// two are not: they are the pre-Stage-0 target from the top of
/// `docs/TIMBRE_PLAN.md`, reconstructed rather than measured, because no build
/// from before the regression is at hand to measure directly — recovering one
/// is Stage T1's bisect, not this stage's.
///
/// The reconstruction is the plan's own aggregate finding applied to each
/// preset's own render: `--fingerprints` gave the true current octave share
/// and crest factor, and 63 Hz, 2 kHz, 4 kHz and crest were then shifted by
/// the deltas the plan measured on the 12 s drive cycle — `-7.3`, `+8.2`,
/// `+8.4` and `+20.0` dB respectively — undoing the one known regression. The
/// other six bands carry the current measurement unchanged, because nothing
/// in the plan's evidence says they moved.
///
/// That makes this table wrong in a specific, bounded way: it assumes the
/// regression was exactly these four numbers and nothing else, on every
/// preset alike, which is certainly false in the details. It is the best
/// reconstruction the evidence in the plan supports, and it is why
/// [`Measured::drift`] is expected to report all four on every preset until
/// Stage T1 lands — that failure is this stage's deliverable, not a bug in
/// it. Replace it with a real pre-regression measurement the moment one
/// exists.
pub const RECORDED: &[Fingerprint] = &[
    Fingerprint {
        preset: "Inline-4",
        firing_order: 2.0,
        balance: &[
            (0.5, -25.7),
            (1.0, -20.6),
            (1.5, -41.5),
            (2.0, 0.0),
            (2.5, -40.3),
            (3.0, -35.2),
            (3.5, -43.5),
            (4.0, -18.6),
        ],
        resonances: &[1483.3, 216.3, 3597.9],
        tilt_db_per_octave: -9.3,
        octave_share: &[
            -8.3, -22.1, -13.6, -1.3, -17.6, -17.4, -18.6, -27.4, -41.6, -52.7,
        ],
        crest_db: 32.0,
    },
    Fingerprint {
        preset: "Cross-plane V8",
        firing_order: 4.0,
        balance: &[
            (0.5, -11.5),
            (1.0, -8.5),
            (1.5, -10.0),
            (2.0, -18.6),
            (2.5, 1.4),
            (3.0, -17.5),
            (3.5, -10.7),
            (4.0, 0.0),
            (4.5, -24.1),
            (5.0, -28.9),
            (5.5, -8.9),
            (6.0, -19.4),
            (6.5, -7.7),
            (7.0, -27.2),
            (7.5, -27.1),
            (8.0, -10.4),
        ],
        resonances: &[1536.0, 3900.3, 83.6],
        tilt_db_per_octave: -10.9,
        octave_share: &[
            -18.3, -10.7, -8.2, -6.9, -9.2, -14.0, -19.4, -24.6, -44.1, -56.5,
        ],
        crest_db: 35.2,
    },
    Fingerprint {
        preset: "Flat-plane V8",
        firing_order: 4.0,
        balance: &[
            (0.5, -14.8),
            (1.0, -15.8),
            (1.5, -23.4),
            (2.0, -24.3),
            (2.5, -30.7),
            (3.0, -25.6),
            (3.5, -29.5),
            (4.0, 0.0),
            (4.5, -33.2),
            (5.0, -29.0),
            (5.5, -30.2),
            (6.0, -26.8),
            (6.5, -34.1),
            (7.0, -35.5),
            (7.5, -50.4),
            (8.0, -12.8),
        ],
        resonances: &[943.8, 1416.0, 209.5],
        tilt_db_per_octave: -9.7,
        octave_share: &[
            -17.7, -14.2, -8.8, -4.0, -6.8, -16.9, -18.9, -28.1, -45.2, -53.6,
        ],
        crest_db: 33.9,
    },
    Fingerprint {
        preset: "V10",
        firing_order: 5.0,
        balance: &[
            (0.5, -3.7),
            (1.0, -11.1),
            (1.5, -14.2),
            (2.0, -20.5),
            (2.5, -19.4),
            (3.0, -15.7),
            (3.5, -13.0),
            (4.0, -17.2),
            (4.5, -33.9),
            (5.0, 0.0),
            (5.5, -32.0),
            (6.0, -25.8),
            (6.5, -28.6),
            (7.0, -25.3),
            (7.5, -28.7),
            (8.0, -18.3),
            (8.5, -33.5),
            (9.0, -34.7),
            (9.5, -41.1),
            (10.0, -13.6),
        ],
        resonances: &[1321.3, 2079.9, 519.3],
        tilt_db_per_octave: -7.9,
        octave_share: &[
            -12.8, -16.4, -3.3, -13.9, -6.6, -16.0, -5.2, -17.4, -41.3, -52.1,
        ],
        crest_db: 35.9,
    },
    Fingerprint {
        preset: "V12",
        firing_order: 6.0,
        balance: &[
            (0.5, -11.5),
            (1.0, -15.7),
            (1.5, -16.8),
            (2.0, -21.9),
            (2.5, -43.3),
            (3.0, 11.4),
            (3.5, -21.2),
            (4.0, -13.9),
            (4.5, -27.1),
            (5.0, -25.5),
            (5.5, -200.0),
            (6.0, 0.0),
            (6.5, -43.8),
            (7.0, -31.4),
            (7.5, -34.4),
            (8.0, -31.8),
            (8.5, -38.6),
            (9.0, -11.8),
            (9.5, -44.3),
            (10.0, -38.1),
            (10.5, -40.5),
            (11.0, -39.7),
            (11.5, -33.0),
            (12.0, -18.6),
        ],
        resonances: &[1552.9, 1192.7, 274.9],
        tilt_db_per_octave: -14.2,
        octave_share: &[
            -21.6, -19.2, -13.3, -2.6, -5.3, -15.4, -22.8, -31.6, -45.5, -55.5,
        ],
        crest_db: 32.0,
    },
    Fingerprint {
        preset: "2-Rotor Wankel",
        firing_order: 2.0,
        balance: &[
            (0.5, -24.2),
            (1.0, -25.1),
            (1.5, -32.2),
            (2.0, 0.0),
            (2.5, -38.0),
            (3.0, -34.6),
            (3.5, -60.2),
            (4.0, -14.8),
        ],
        resonances: &[53.6, 1144.6, 1826.1],
        tilt_db_per_octave: -10.1,
        octave_share: &[
            -16.5, -11.7, -10.5, -3.3, -15.6, -15.8, -19.7, -29.4, -47.8, -60.4,
        ],
        crest_db: 34.4,
    },
    Fingerprint {
        preset: "Turbo Inline-4",
        firing_order: 2.0,
        balance: &[
            (0.5, -15.5),
            (1.0, -13.7),
            (1.5, -29.8),
            (2.0, 0.0),
            (2.5, -25.6),
            (3.0, -20.7),
            (3.5, -41.3),
            (4.0, -4.0),
        ],
        resonances: &[1636.3, 50.4, 1149.5],
        tilt_db_per_octave: -8.1,
        octave_share: &[
            -11.6, -9.3, -12.3, -7.3, -16.3, -16.8, -18.2, -21.5, -27.7, -42.9,
        ],
        crest_db: 35.7,
    },
    Fingerprint {
        preset: "Twin-turbo V8",
        firing_order: 4.0,
        balance: &[
            (0.5, -17.2),
            (1.0, -15.6),
            (1.5, 0.3),
            (2.0, -21.2),
            (2.5, -1.6),
            (3.0, -32.8),
            (3.5, -15.2),
            (4.0, 0.0),
            (4.5, -33.0),
            (5.0, -28.1),
            (5.5, -14.1),
            (6.0, -36.6),
            (6.5, -7.0),
            (7.0, -32.5),
            (7.5, -32.8),
            (8.0, -7.1),
        ],
        resonances: &[104.5, 1786.8, 1112.0],
        tilt_db_per_octave: -8.7,
        octave_share: &[
            -19.6, -15.4, -2.8, -6.8, -12.2, -15.1, -21.4, -18.4, -39.8, -59.3,
        ],
        crest_db: 35.4,
    },
    Fingerprint {
        preset: "Turbo Inline-6",
        firing_order: 3.0,
        balance: &[
            (0.5, -23.3),
            (1.0, -21.7),
            (1.5, -25.7),
            (2.0, -26.7),
            (2.5, -200.0),
            (3.0, 0.0),
            (3.5, -38.0),
            (4.0, -32.4),
            (4.5, -33.4),
            (5.0, -30.0),
            (5.5, -200.0),
            (6.0, -16.9),
        ],
        resonances: &[148.7, 1407.7, 1025.0],
        tilt_db_per_octave: -9.7,
        octave_share: &[
            -15.4, -18.1, -1.7, -7.1, -20.6, -20.0, -19.4, -27.6, -50.6, -64.6,
        ],
        crest_db: 34.2,
    },
    Fingerprint {
        preset: "Turbodiesel I4",
        firing_order: 2.0,
        balance: &[
            (0.5, -11.8),
            (1.0, -7.1),
            (1.5, -200.0),
            (2.0, 0.0),
            (2.5, -200.0),
            (3.0, -20.5),
            (3.5, -200.0),
            (4.0, -16.8),
        ],
        resonances: &[3592.5, 408.7, 2036.4],
        tilt_db_per_octave: -5.0,
        octave_share: &[
            -15.4, -16.5, -3.1, -11.7, -7.6, -12.1, -17.0, -7.0, -24.8, -47.9,
        ],
        crest_db: 34.2,
    },
    Fingerprint {
        preset: "Big Single",
        firing_order: 0.5,
        balance: &[(0.5, 0.0), (1.0, 10.9)],
        resonances: &[116.0, 2096.0, 1197.4],
        tilt_db_per_octave: -7.5,
        octave_share: &[
            -23.7, -19.9, -3.7, -10.0, -6.2, -8.7, -8.1, -12.2, -35.8, -46.6,
        ],
        crest_db: 38.4,
    },
];

#[cfg(test)]
mod tests {
    use super::*;

    /// The regression itself: every engine in the catalogue still sounds like
    /// the numbers recorded for it.
    #[test]
    fn no_preset_has_drifted_from_its_recorded_timbre() {
        let mut failures = Vec::new();
        for preset in EnginePreset::catalogue() {
            let measured = measure(&preset);
            let Some(recorded) = measured.recorded() else {
                continue;
            };
            for line in measured.drift(recorded) {
                failures.push(format!("{}: {line}", preset.name));
            }
        }
        assert!(
            failures.is_empty(),
            "timbre drifted:\n  {}\n\nIf the change was intended, re-record with \
             `cargo run --release --example calibrate -- --fingerprints` and \
             regenerate docs/measurements/calibration.md in the same commit.",
            failures.join("\n  ")
        );
    }

    /// The whole calibration loop, end to end: a render written to a file,
    /// read back at a quarter of the level, order-tracked against a supplied
    /// speed curve, and compared with the render it came from.
    ///
    /// The reference here is the synth's own output, so this is a test of the
    /// *loop* and not of the model — it cannot say the engine sounds right, and
    /// no reference recording is committed for it to ask. What it does say is
    /// that the parts between a file on disk and a per-order comparison are
    /// sound: the WAV survives the round trip, a supplied curve puts the orders
    /// where they belong, and twelve decibels of gain between the two sides
    /// cancels out of every figure in the table. If that last part ever breaks,
    /// a calibration against a real recording becomes a measurement of the
    /// recording's mastering.
    #[test]
    fn the_reference_loop_matches_a_render_to_itself_at_another_level() {
        use crate::analysis::orders::{Reference, RpmCurve};
        use crate::analysis::render::{read_wav, write_wav};

        let preset = EnginePreset::inline_four();
        let script = script::calibration_sweep(&preset);
        let render = RenderPlan::new(&preset, &script).render();

        let dir = std::env::temp_dir().join("engine-sim-reference-loop");
        let _ = std::fs::create_dir_all(&dir);
        let path = dir.join("inline-4-sweep.wav");
        // A quarter of the amplitude: a recording arrives at whatever level the
        // chain that made it left it at.
        let quiet: Vec<f32> = render.samples.iter().map(|s| s * 0.25).collect();
        write_wav(
            &path,
            &quiet,
            render.channels as u16,
            render.sample_rate as u32,
        )
        .expect("writing the reference");

        let recording = read_wav(&path).expect("reading the reference");
        assert_eq!(recording.channels, render.channels);
        assert!((recording.seconds() - render.seconds()).abs() < 1e-6);

        // The curve is supplied from outside, as a recording's has to be: two
        // endpoints of a pull, exactly as `--rpm 850:7400` gives it.
        let supplied = RpmCurve::sweep(preset.idle, preset.redline, recording.seconds());
        let firing_order = preset.firing.len() as f64 / 2.0;
        let reference = Reference::extract(
            &recording.mono(),
            recording.sample_rate,
            &supplied,
            &half_orders(),
        );

        let measured = measure(&preset);
        let mine = orders::Balance::from_levels(
            firing_order,
            &measured
                .balance
                .iter()
                .map(|&(order, db)| (order, (db > SILENCE_DB).then_some(db)))
                .collect::<Vec<_>>(),
        );
        // The same render at full scale, for a level to compare against.
        let loud = orders::track(
            &render.mono(),
            render.sample_rate,
            &render.rpm,
            &half_orders(),
        )
        .line_balance(firing_order);

        let theirs = reference.orders.line_balance(firing_order);

        // Twelve decibels apart in absolute level, and the same engine.
        assert!(
            (loud.reference_db - theirs.reference_db - 12.04).abs() < 0.1,
            "a quarter of the amplitude read {:.2} dB down, not 12.04",
            loud.reference_db - theirs.reference_db
        );
        let comparison = mine.against(&theirs);
        assert!(
            comparison.deltas.len() >= 4,
            "only {} orders compared",
            comparison.deltas.len()
        );
        assert!(
            comparison.within(LEVEL_TOLERANCE_DB),
            "the loop disagreed with itself by {:.2} dB on order {:?}",
            comparison.max_abs_db(),
            comparison.worst().map(|d| d.order),
        );

        let _ = std::fs::remove_file(&path);
    }

    /// The declared primary length still reaches the sound: stretch the
    /// primaries and the rendered exhaust band moves down with them.
    ///
    /// # Why this no longer looks for a peak
    ///
    /// It used to. It located the peak nearest `c/4L`, stretched the primaries
    /// by half, and required the peak to move by about half as much again. That
    /// worked because the network it measured was very nearly lossless: with
    /// the wall taking a fiftieth of the decibels `alpha` asks for, the whole
    /// run from the valve to the mouth was one resonator of enormous Q, and
    /// stretching the primaries moved its modes bodily — 358 Hz to 172 Hz on
    /// the network's own response, at a resonant gain of 51 dB.
    ///
    /// The same measurement now reads 21 dB, and the primary's own quarter wave
    /// is a broad hump rather than a peak: its loop pays 1.4 dB a round trip to
    /// the wall, and the four-into-one collector behind it sends most of what
    /// arrives onward rather than back. That is what a header does, and it is
    /// why the peak picker reports the mode as *not found* — a finding recorded
    /// per preset in `docs/measurements/calibration.md`, not a failure of the
    /// synth to read the length.
    ///
    /// So the claim is made where it survives: at the network, where the
    /// primary's transit is exactly proportional to the length it was given,
    /// and in the render, where the spectrum's centre of gravity over the band
    /// the primaries occupy moves down monotonically as they are stretched.
    /// A synth that had stopped reading the length would fail both.
    #[test]
    fn the_declared_primary_length_still_reaches_the_sound() {
        use crate::analysis::orders::Stft;
        use crate::audio::dsp::EngineSnapshot;
        use crate::audio::waveguide::ExhaustNetwork;
        use crate::environment::Environment;
        use crate::physics::plumbing::PipeSection;

        /// The band the inline-four's primaries and the modes around them
        /// occupy at the temperatures the sweep reaches [Hz].
        const BAND: (f64, f64) = (150.0, 900.0);

        let stretched = |factor: f64| -> EnginePreset {
            let mut preset = EnginePreset::inline_four();
            let primary = preset.exhaust.primaries[0];
            preset.exhaust.primaries = vec![
                PipeSection::from_diameter(
                    primary.length * factor,
                    primary.diameter(),
                    primary.wall_temperature,
                );
                preset.firing.len()
            ];
            preset
        };

        // 1. The network's own transit, which is where the length enters.
        let transit = |factor: f64| -> f32 {
            let preset = stretched(factor);
            let block = preset.block(Environment::default());
            let config = preset.synth_config(&block, 48_000.0);
            let network = ExhaustNetwork::new(
                &config.exhaust,
                &config.cylinders,
                config.bank_count,
                48_000.0,
                &EngineSnapshot::default(),
            );
            network.primary_round_trip_seconds(0)
        };
        let ratio = transit(1.5) / transit(1.0);
        assert!(
            (ratio - 1.5).abs() < 0.02,
            "a primary half again as long round-trips in {ratio:.3} of the time"
        );

        // 2. And it reaches the render: the centre of gravity of the band the
        //    primaries work in falls as they lengthen, every step of the way.
        let centroid = |factor: f64| -> f64 {
            let preset = stretched(factor);
            let script = script::calibration_sweep(&preset);
            let render = RenderPlan::new(&preset, &script).render();
            let mono = render.mono();
            let mut stft = Stft::new(8_192, render.sample_rate);
            let mut power = vec![0.0f64; 8_192 / 2 + 1];
            let (mut frames, mut at) = (0usize, 0usize);
            let hop = stft.hop();
            while at + 8_192 <= mono.len() {
                stft.analyse(&mono[at..at + 8_192]);
                for (sum, p) in power.iter_mut().zip(stft.power()) {
                    *sum += *p;
                }
                frames += 1;
                at += hop;
            }
            let bin = render.sample_rate / 8_192.0;
            let (mut moment, mut total) = (0.0, 0.0);
            for (k, &p) in power.iter().enumerate() {
                let hz = k as f64 * bin;
                if (BAND.0..=BAND.1).contains(&hz) {
                    moment += hz * p / frames.max(1) as f64;
                    total += p / frames.max(1) as f64;
                }
            }
            moment / total.max(1e-30)
        };

        let short = centroid(1.0);
        let middle = centroid(1.25);
        let long = centroid(1.5);
        assert!(
            short > middle && middle > long,
            "the band did not follow the length: {short:.1}, {middle:.1}, {long:.1} Hz"
        );
        // Five per cent over a half-again stretch. Far short of proportional,
        // because most of what is in this band is the collector, the chamber
        // and the tailpipe, and none of those moved.
        assert!(
            long < 0.95 * short,
            "stretching the primaries by half moved the band only from \
             {short:.1} Hz to {long:.1} Hz"
        );
    }

    /// The comparator catches what it is there to catch, without rendering
    /// anything: a level that moved, a resonance that moved, a quiet order that
    /// climbed out of the floor, and a tilt that flattened.
    #[test]
    fn drift_is_reported_number_by_number() {
        let recorded = Fingerprint {
            preset: "Test",
            firing_order: 2.0,
            balance: &[(0.5, -45.0), (1.0, -12.0), (2.0, 0.0), (4.0, -8.0)],
            resonances: &[200.0, 800.0],
            tilt_db_per_octave: -8.0,
            octave_share: &[
                -20.0, -18.0, -16.0, -14.0, -12.0, -10.0, -12.0, -14.0, -18.0, -22.0,
            ],
            crest_db: 14.0,
        };
        let unchanged = Measured {
            preset: "Test",
            firing_order: 2.0,
            balance: vec![(0.5, -47.0), (1.0, -12.4), (2.0, 0.0), (4.0, -8.3)],
            resonances: vec![
                Peak {
                    hz: 201.0,
                    db: -20.0,
                    prominence_db: 12.0,
                },
                Peak {
                    hz: 806.0,
                    db: -30.0,
                    prominence_db: 9.0,
                },
            ],
            tilt_db_per_octave: -8.4,
            octave_share: [
                -20.3, -18.2, -16.1, -14.2, -12.3, -10.2, -12.4, -14.3, -18.4, -22.4,
            ],
            crest_db: 14.3,
        };
        assert!(
            unchanged.drift(&recorded).is_empty(),
            "arithmetic-sized differences read as drift: {:?}",
            unchanged.drift(&recorded)
        );

        let drifted = Measured {
            // Order 4 down three decibels, the quiet half-order up out of the
            // floor, the second resonance a fifth of an octave sharp, and the
            // floor two decibels an octave flatter.
            balance: vec![(0.5, -20.0), (1.0, -12.0), (2.0, 0.0), (4.0, -11.0)],
            resonances: vec![
                Peak {
                    hz: 200.0,
                    db: -20.0,
                    prominence_db: 12.0,
                },
                Peak {
                    hz: 900.0,
                    db: -30.0,
                    prominence_db: 9.0,
                },
            ],
            tilt_db_per_octave: -6.0,
            ..unchanged
        };
        let lines = drifted.drift(&recorded);
        assert_eq!(lines.len(), 4, "reported {lines:?}");
        assert!(lines[0].contains("order 0.5") && lines[0].contains("floor"));
        assert!(lines[1].contains("order 4") && lines[1].contains("-3.0 dB"));
        assert!(lines[2].contains("resonance 2") && lines[2].contains("+12.5 %"));
        assert!(lines[3].contains("tilted"));
    }

    /// Stage T0's own additions to the comparator: a band or a crest factor
    /// that moved by an arithmetic-sized amount stays silent, and one that
    /// moved by the kind of amount this plan exists to catch is named.
    #[test]
    fn drift_names_the_band_and_crest_that_moved() {
        let recorded = Fingerprint {
            preset: "Test",
            firing_order: 2.0,
            balance: &[(2.0, 0.0)],
            resonances: &[],
            tilt_db_per_octave: -8.0,
            octave_share: &[
                -20.0, -18.0, -16.0, -14.0, -12.0, -10.0, -12.0, -14.0, -18.0, -22.0,
            ],
            crest_db: 14.0,
        };
        let mut quiet = Measured {
            preset: "Test",
            firing_order: 2.0,
            balance: vec![(2.0, 0.0)],
            resonances: vec![],
            tilt_db_per_octave: -8.0,
            octave_share: *recorded.octave_share,
            crest_db: recorded.crest_db,
        };

        // A one-decibel nudge at 2 kHz (index 6) is ordinary noise.
        quiet.octave_share[6] -= 1.0;
        assert!(
            quiet.drift(&recorded).is_empty(),
            "a 1 dB move at 2 kHz read as drift: {:?}",
            quiet.drift(&recorded)
        );

        // Three decibels there is not.
        let mut loud = quiet.clone();
        loud.octave_share[6] = recorded.octave_share[6] - 3.0;
        let lines = loud.drift(&recorded);
        assert_eq!(lines.len(), 1, "reported {lines:?}");
        assert!(lines[0].contains("2000") && lines[0].contains("-3.0 dB"));

        // The same shape of test for crest factor: silent under tolerance,
        // named over it.
        let mut hollow = quiet;
        hollow.octave_share[6] = recorded.octave_share[6];
        hollow.crest_db = recorded.crest_db - 1.0;
        assert!(
            hollow.drift(&recorded).is_empty(),
            "a 1 dB crest move drifted"
        );
        hollow.crest_db = recorded.crest_db - 3.0;
        let lines = hollow.drift(&recorded);
        assert_eq!(lines.len(), 1, "reported {lines:?}");
        assert!(lines[0].contains("crest factor") && lines[0].contains("-3.0 dB"));
    }

    /// A preset with no fingerprint is not regressed, so adding one to the
    /// catalogue has to come with recording it.
    #[test]
    fn every_preset_in_the_catalogue_has_a_fingerprint() {
        let missing: Vec<&str> = EnginePreset::catalogue()
            .iter()
            .map(|preset| preset.name)
            .filter(|name| !RECORDED.iter().any(|f| f.preset == *name))
            .collect();
        assert!(
            missing.is_empty(),
            "no recorded timbre for {} — record one with \
             `cargo run --release --example calibrate -- --fingerprints`",
            missing.join(", ")
        );
    }
}
