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

use crate::analysis::orders::{self, half_orders, AverageSpectrum, Peak, SILENCE_DB};
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
pub const RECORDED: &[Fingerprint] = &[
    Fingerprint {
        preset: "Inline-4",
        firing_order: 2.0,
        balance: &[
            (0.5, -34.5),
            (1.0, -32.3),
            (1.5, -51.6),
            (2.0, 0.0),
            (2.5, -35.0),
            (3.0, -39.2),
            (3.5, -32.7),
            (4.0, -26.4),
        ],
        resonances: &[1481.7, 216.3, 1046.0],
        tilt_db_per_octave: -8.6,
    },
    Fingerprint {
        preset: "Cross-plane V8",
        firing_order: 4.0,
        balance: &[
            (0.5, -18.3),
            (1.0, 11.2),
            (1.5, -0.4),
            (2.0, -3.5),
            (2.5, 9.3),
            (3.0, 3.3),
            (3.5, 3.9),
            (4.0, 0.0),
            (4.5, -13.4),
            (5.0, -22.5),
            (5.5, 0.6),
            (6.0, 1.8),
            (6.5, -3.5),
            (7.0, -11.5),
            (7.5, -16.9),
            (8.0, -10.6),
        ],
        resonances: &[1231.6, 963.7, 346.0],
        tilt_db_per_octave: -9.5,
    },
    Fingerprint {
        preset: "Flat-plane V8",
        firing_order: 4.0,
        balance: &[
            (0.5, -8.4),
            (1.0, -20.2),
            (1.5, -24.5),
            (2.0, -24.3),
            (2.5, -27.4),
            (3.0, -28.7),
            (3.5, -28.5),
            (4.0, 0.0),
            (4.5, -38.4),
            (5.0, -29.2),
            (5.5, -29.1),
            (6.0, -15.9),
            (6.5, -23.8),
            (7.0, -11.3),
            (7.5, -17.5),
            (8.0, -9.8),
        ],
        resonances: &[3336.5, 806.3, 1401.0],
        tilt_db_per_octave: -6.6,
    },
    Fingerprint {
        preset: "V10",
        firing_order: 5.0,
        balance: &[
            (0.5, 3.6),
            (1.0, -0.9),
            (1.5, -11.6),
            (2.0, -13.0),
            (2.5, -21.8),
            (3.0, -16.7),
            (3.5, 0.0),
            (4.0, -6.3),
            (4.5, -2.5),
            (5.0, 0.0),
            (5.5, -25.8),
            (6.0, -24.7),
            (6.5, -17.1),
            (7.0, -19.4),
            (7.5, -15.6),
            (8.0, -19.0),
            (8.5, -17.7),
            (9.0, -16.1),
            (9.5, -11.5),
            (10.0, -8.4),
        ],
        resonances: &[1431.8, 465.2, 263.8],
        tilt_db_per_octave: -5.6,
    },
    Fingerprint {
        preset: "V12",
        firing_order: 6.0,
        balance: &[
            (0.5, -8.8),
            (1.0, -18.7),
            (1.5, -17.9),
            (2.0, 5.1),
            (2.5, 20.2),
            (3.0, 25.8),
            (3.5, -19.3),
            (4.0, -23.5),
            (4.5, -24.2),
            (5.0, -27.1),
            (5.5, -200.0),
            (6.0, 0.0),
            (6.5, -48.7),
            (7.0, -31.9),
            (7.5, -26.9),
            (8.0, -26.6),
            (8.5, -5.9),
            (9.0, 4.0),
            (9.5, -24.9),
            (10.0, -29.1),
            (10.5, -17.9),
            (11.0, -13.0),
            (11.5, -17.5),
            (12.0, -11.8),
        ],
        resonances: &[1188.9, 269.9, 3566.6],
        tilt_db_per_octave: -11.5,
    },
    Fingerprint {
        preset: "2-Rotor Wankel",
        firing_order: 2.0,
        balance: &[
            (0.5, -27.8),
            (1.0, -39.4),
            (1.5, -9.8),
            (2.0, 0.0),
            (2.5, -26.0),
            (3.0, -24.5),
            (3.5, -32.4),
            (4.0, -15.8),
        ],
        resonances: &[205.2, 1144.3, 1718.9],
        tilt_db_per_octave: -6.7,
    },
    Fingerprint {
        preset: "Turbo Inline-4",
        firing_order: 2.0,
        balance: &[
            (0.5, -3.7),
            (1.0, -14.6),
            (1.5, -26.3),
            (2.0, 0.0),
            (2.5, -18.1),
            (3.0, -22.0),
            (3.5, -45.8),
            (4.0, 3.6),
        ],
        resonances: &[1631.9, 1150.4, 246.9],
        tilt_db_per_octave: -7.9,
    },
    Fingerprint {
        preset: "Twin-turbo V8",
        firing_order: 4.0,
        balance: &[
            (0.5, -18.5),
            (1.0, 4.0),
            (1.5, 6.9),
            (2.0, -2.5),
            (2.5, 4.2),
            (3.0, -4.7),
            (3.5, -1.1),
            (4.0, 0.0),
            (4.5, -14.8),
            (5.0, -12.3),
            (5.5, -13.9),
            (6.0, -24.5),
            (6.5, -7.4),
            (7.0, -11.2),
            (7.5, -24.3),
            (8.0, -7.9),
        ],
        resonances: &[1535.5, 114.3, 3072.6],
        tilt_db_per_octave: -8.1,
    },
    Fingerprint {
        preset: "Turbo Inline-6",
        firing_order: 3.0,
        balance: &[
            (0.5, -7.8),
            (1.0, -47.1),
            (1.5, -12.8),
            (2.0, -6.3),
            (2.5, -19.0),
            (3.0, 0.0),
            (3.5, -46.1),
            (4.0, -42.2),
            (4.5, -41.7),
            (5.0, -44.4),
            (5.5, -200.0),
            (6.0, -21.7),
        ],
        resonances: &[148.5, 1477.2, 731.0],
        tilt_db_per_octave: -8.6,
    },
    Fingerprint {
        preset: "Turbodiesel I4",
        firing_order: 2.0,
        balance: &[
            (0.5, -11.0),
            (1.0, -7.4),
            (1.5, -27.4),
            (2.0, 0.0),
            (2.5, -28.2),
            (3.0, -28.5),
            (3.5, -15.6),
            (4.0, -11.1),
        ],
        resonances: &[2031.8, 409.0, 1678.9],
        tilt_db_per_octave: -4.0,
    },
    Fingerprint {
        preset: "Big Single",
        firing_order: 0.5,
        balance: &[(0.5, 0.0), (1.0, 17.8)],
        resonances: &[2874.8, 111.1, 1197.9],
        tilt_db_per_octave: -4.5,
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

    /// The audible exhaust mode is set by the declared primary length: stretch
    /// the primaries by half and the peak near their quarter wave drops by
    /// about half as much again.
    ///
    /// End to end, through the physics, the whole network, the radiation model
    /// and the analysis — which is what makes it worth having and also what
    /// limits what it can claim. The relation is *not* asserted to be `c/4L`
    /// exactly, because as measured it is not: a primary's peak moves with its
    /// own length and with the collector taper and chain behind it, so
    /// stretching the inline-four's primaries by half moves the peak from
    /// 403 Hz to 244 Hz where strict proportionality says 269. Twenty per cent
    /// either side of proportional is the stated tolerance, and it is wide
    /// enough to hold the taper's share and narrow enough to fail a synth that
    /// had stopped reading the length at all — which is the regression this is
    /// here to catch. The exact placement against `c/4(L+d)` per preset is
    /// recorded in `docs/measurements/calibration.md`.
    #[test]
    fn the_exhaust_peak_follows_the_declared_primary_length() {
        use crate::analysis::orders::{place, resonances, PLACEMENT_WINDOW_PCT};
        use crate::audio::filters::speed_of_sound;
        use crate::physics::plumbing::PipeSection;

        /// How much longer the stretched primaries are.
        const STRETCH: f64 = 1.5;
        /// How far from strict proportionality the peak may land.
        const TOLERANCE: f64 = 0.20;

        // A primary at the temperature the solver settles them to through a
        // pull, which is all the prediction is for: it only has to be close
        // enough to find the right peak.
        let nominal_c = speed_of_sound(1.33, 287.0, 1_200.0) as f64;
        let measure_peak = |factor: f64| -> f64 {
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

            let script = script::calibration_sweep(&preset);
            let render = RenderPlan::new(&preset, &script).render();
            let peaks = resonances(&render.mono(), render.sample_rate, 24);
            let predicted = nominal_c / (4.0 * preset.exhaust.primary_length());
            place(predicted, &peaks, PLACEMENT_WINDOW_PCT)
                .measured_hz
                .unwrap_or_else(|| {
                    panic!("no peak within reach of {predicted:.0} Hz at {factor}x length")
                })
        };

        let short = measure_peak(1.0);
        let long = measure_peak(STRETCH);
        let ratio = long / short;
        assert!(
            (ratio - 1.0 / STRETCH).abs() <= TOLERANCE / STRETCH,
            "primaries {STRETCH}x longer moved the peak from {short:.1} Hz to \
             {long:.1} Hz, a ratio of {ratio:.3} where proportional is {:.3}",
            1.0 / STRETCH
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
