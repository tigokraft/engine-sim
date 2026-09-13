//! Calibration: what the synth does against what something else says it should.
//!
//! ```text
//! cargo run --release --example calibrate
//! cargo run --release --example calibrate -- --preset V12
//! cargo run --release --example calibrate -- --markdown docs/measurements/calibration.md
//! cargo run --release --example calibrate -- --reference pull.wav --preset Inline-4 --rpm 1200:6800
//! cargo run --release --example calibrate -- --fingerprints
//! ```
//!
//! The last of those re-records the timbre regression's baseline in
//! [`timbre::RECORDED`](rust_engine_sim::analysis::timbre::RECORDED); see that
//! module for when doing so is legitimate.
//!
//! Stage 0 answered *what does it sound like*, as numbers. This answers *is
//! that right*, against a reference the synth did not produce. Three metrics,
//! which are the three the plan names:
//!
//! - **Order balance.** How loud each engine order is relative to the firing
//!   order. Quoted relative, never absolute, because a recording arrives at
//!   whatever level a microphone and a mastering chain left it at — and because
//!   a relative table is one no gain constant can move, which is what keeps a
//!   calibration honest.
//! - **Resonance placement.** Every length, volume and radius in the geometry
//!   predicts a frequency. This is whether the audio resonates there.
//! - **Noise-floor tilt.** The slope of what is left between the orders. A
//!   model can put every order in the right place and still read as synthetic
//!   because its floor is flat where a real one falls away.
//!
//! # Where the reference comes from
//!
//! With `--reference` it is a recording, order-tracked against an rpm curve
//! supplied on the command line — nothing in a recording says how fast the
//! engine was turning, so it has to be told.
//!
//! With no `--reference` it is the engine's own construction, which is a
//! reference this repository can carry and a recording is not:
//!
//! - the **crank comb**, the order spectrum of the firing pattern itself. An
//!   inline-four's even 180-degree spacing can drive the even orders and
//!   mathematically nothing else; a cross-plane V8's 90-180-270-180 bank leaves
//!   order 1.5 a few decibels under its firing order. Neither figure is tunable
//!   and neither comes from a recording.
//! - the **analytic modes** of the plumbing: `c / 4L` down a primary, `n c / 2L`
//!   across an expansion chamber, a Helmholtz plenum, a quarter-wave stub, the
//!   block's own mass law. Appendix A of the plan, applied to the geometry the
//!   preset declares and the gas the solver actually delivers.
//!
//! # The one rule
//!
//! When the synth disagrees, the fix is a geometric parameter — a primary
//! length, a chamber volume, a mouth radius — and never a gain or an
//! equaliser. So every placement row reports the *length that would put the
//! mode where it was measured*, which is the correction in the currency the fix
//! has to be paid in. A disagreement that cannot be explained geometrically
//! goes into `docs/measurements/calibration.md` as an open question instead.

use std::env;
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};

use rust_engine_sim::analysis::orders::{
    self, half_orders, place, place_all, Balance, Comparison, Peak, Placement, Reference, RpmCurve,
    BACKGROUND_OFFSET, CRANK_COMB_ORDERS, PLACEMENT_WINDOW_PCT,
};
use rust_engine_sim::analysis::render::{
    read_wav, RenderPlan, OFFLINE_RATE, PHYSICS_HZ, PRIME_STEPS,
};
use rust_engine_sim::analysis::script::{self, RenderScript, CALIBRATION_SECONDS};
use rust_engine_sim::analysis::timbre;
use rust_engine_sim::audio::dsp::{EngineSnapshot, SourceRate};
use rust_engine_sim::audio::filters::{block_resonance_hz, speed_of_sound};
use rust_engine_sim::audio::intake_voice::{
    INTAKE_AIR_GAMMA, INTAKE_AMBIENT_TEMPERATURE_K, INTAKE_GAS_CONSTANT,
};
use rust_engine_sim::audio::radiation::end_correction;
use rust_engine_sim::audio::waveguide::{chain_element_temperature, mean_flow_mach};
use rust_engine_sim::audio::SnapshotSource;
use rust_engine_sim::bench::EnginePreset;
use rust_engine_sim::environment::Environment;
use rust_engine_sim::physics::plumbing::{Silencer, ThrottleLayout};

/// Straight legs a reference recording's speed curve is rendered as.
///
/// Sixty-four over a pull of a few seconds is a leg every tenth of a second,
/// which is finer than the analysis window the result is read through and
/// coarse enough that the script stays a script rather than a sample stream.
const REFERENCE_LEGS: usize = 64;

/// Resonance peaks read out of the sweep average.
///
/// Enough to cover the primaries, the chamber modes, the intake and the block
/// without the tail of the list being scatter.
const PEAKS: usize = 24;

/// Agreement required of the orders the crank drives [dB].
///
/// Applied against a *recording*, and reported but not enforced against the
/// crank. The comb is the excitation, and what a sweep measures is the
/// excitation after the pipes have had it: averaging a whole pull smears the
/// transfer function across every order, which is what leaves the comb as the
/// leading term, but it does not flatten it. A recording carries the same
/// smearing and the same pipes, so against a recording the tolerance means
/// something; against the bare crank it would be a tolerance on the difference
/// between an excitation and a radiated sound.
const ORDER_TOLERANCE_DB: f64 = 8.0;

/// How far under the firing order an order the crank cannot drive has to sit
/// [dB].
///
/// The crank's nulls are exact: an even-firing bank drives its odd half-orders
/// with mathematically nothing, so nothing should stand on them as a *line*.
/// What is left there in the audio is the parts of the engine that are not the
/// firing — mechanical noise, turbulence, the half-cycle asymmetry of a real
/// four-stroke — so the test is that they stay well under the note rather than
/// that they are absent.
///
/// Enforced only where the comb has no other unknown in it; see
/// [`Calibrated::comb_is_unambiguous`].
const NULL_FLOOR_DB: f64 = -12.0;

/// Placement tolerance for an analytic mode [%].
///
/// A quarter-wave mode is set by a length and a speed of sound, and both are
/// known here to better than a per cent. Ten is what is left for the things the
/// formula leaves out: the junction at the far end of a primary is not the free
/// open end the formula assumes, the gas in a pipe is not at one temperature,
/// and a network's modes pull each other about.
const PLACEMENT_TOLERANCE_PCT: f64 = 10.0;

/// Band the noise floor's tilt is measured over [Hz].
///
/// Above the firing orders of every engine in the catalogue at idle, and below
/// the rate at which the render's own resampling would start to show.
const TILT_BAND_HZ: (f64, f64) = (200.0, 12_000.0);

// ---------------------------------------------------------------------------
// What geometry predicts
// ---------------------------------------------------------------------------

/// One mode the geometry says exists, and where it says it is.
struct Predicted {
    /// What it is.
    label: &'static str,
    /// The formula, in the shape Appendix A writes it.
    formula: String,
    /// Where it should be [Hz].
    hz: f64,
    /// The geometric parameter that moves it, and its present value.
    parameter: String,
    /// The value of that parameter, so a correction can be quoted in it.
    ///
    /// A length in metres for a quarter-wave mode; `None` where the correction
    /// is not a single length — a Helmholtz volume moves as the square, and a
    /// block mode as the square root of a mass.
    length: Option<f64>,
}

/// Every analytic mode of one preset, in the gas the solver actually delivers.
///
/// Two things go into each figure and they come from different places, which is
/// what keeps this from being a restatement of the model:
///
/// - the **acoustics** are Appendix A of the plan and nothing else — `c/4L`
///   down a closed-open pipe, `nc/2L` across a chamber, `(c/2pi)sqrt(A/VL)` for
///   a Helmholtz volume, `c(1-M^2)/4L` once there is flow in the pipe, and the
///   end corrections `0.6133a` unflanged and `0.8216a` at a baffle;
/// - the **gas** is whatever the solver delivered — the temperature at each
///   station, the gradient down the silencer chain, the mass flow — because a
///   prediction on a nominal temperature is a prediction about a different
///   engine.
///
/// Where a pipe ends in a merge into a wider duct it is given the *flanged* end
/// correction, not the unflanged one. An abrupt expansion loads the small pipe
/// the way a baffle does: the wall of the wider duct is the baffle. It is the
/// same 0.8216a the formula list has for a pipe cut flush into a panel, and it
/// is the difference between predicting a primary at 452 Hz and at 427.
fn predictions(preset: &EnginePreset, gas: &EngineSnapshot) -> Vec<Predicted> {
    let gamma = gas.exhaust_gamma;
    let r = gas.exhaust_gas_constant;
    let c_primary = speed_of_sound(gamma, r, gas.primary_temperature[0]) as f64;
    let c_tail = speed_of_sound(gamma, r, gas.tailpipe_temperature) as f64;
    let c_intake = speed_of_sound(
        INTAKE_AIR_GAMMA,
        INTAKE_GAS_CONSTANT,
        INTAKE_AMBIENT_TEMPERATURE_K,
    ) as f64;

    let exhaust = &preset.exhaust;
    let intake = &preset.intake;
    let cylinders = preset.firing.len().max(1);
    let banks = preset.firing.bank_count().max(1);
    let mut out = Vec::new();

    // The primary runner: closed at the valve for most of the cycle, opening
    // into the collector, and carrying the whole of one cylinder's flow.
    let l_primary = exhaust.primary_length();
    let a_primary = exhaust.primary_area();
    if l_primary > 0.0 {
        let radius = (a_primary / std::f64::consts::PI).sqrt();
        let delta = end_correction(radius, true);
        let mach = mean_flow_mach(
            gas.intake_mass_flow / cylinders as f32,
            a_primary as f32,
            gamma,
            r,
            gas.primary_temperature[0],
        ) as f64;
        let flow = 1.0 - mach * mach;
        let acoustic = l_primary + delta;
        for n in [1.0, 3.0] {
            out.push(Predicted {
                label: if n == 1.0 {
                    "exhaust primary, quarter wave"
                } else {
                    "exhaust primary, third mode"
                },
                formula: format!(
                    "{}c(1-M^2)/4(L+d) = {n:.0} x {c_primary:.0} x {flow:.3}/(4 x {acoustic:.3})",
                    if n == 1.0 { "" } else { "3" }
                ),
                hz: n * c_primary * flow / (4.0 * acoustic),
                parameter: format!("primary length {l_primary:.3} m"),
                length: Some(l_primary),
            });
        }
    }

    // The tailpipe: fed from the silencer chain, open to the air, lengthened by
    // its own mouth, and carrying one bank's flow.
    let tail_radius = (exhaust.tailpipe.area / std::f64::consts::PI).sqrt();
    let tail_delta = end_correction(tail_radius, exhaust.tailpipe_flanged);
    let tail_acoustic = exhaust.tailpipe.length + tail_delta;
    let tail_mach = mean_flow_mach(
        gas.intake_mass_flow / banks as f32,
        exhaust.tailpipe.area as f32,
        gamma,
        r,
        gas.tailpipe_temperature,
    ) as f64;
    if exhaust.tailpipe.length > 0.0 {
        out.push(Predicted {
            label: "tailpipe, quarter wave",
            formula: format!(
                "c(1-M^2)/4(L+d) = {c_tail:.0} x {:.3}/(4 x ({:.3} + {tail_delta:.3}))",
                1.0 - tail_mach * tail_mach,
                exhaust.tailpipe.length,
            ),
            hz: c_tail * (1.0 - tail_mach * tail_mach) / (4.0 * tail_acoustic),
            parameter: format!(
                "tailpipe length {:.3} m, mouth radius {tail_radius:.3} m",
                exhaust.tailpipe.length
            ),
            length: Some(tail_acoustic),
        });
    }

    // The silencers, each in the gas of its own place down the chain: the whole
    // run is strung out along the coolest half of the exhaust, and a chamber
    // breathing gas a tenth cooler passes a band a twentieth lower.
    let elements = exhaust
        .silencers
        .iter()
        .map(|s| match s {
            Silencer::ExpansionChamber { stages, .. } => (*stages).max(1),
            Silencer::Straight => 0,
            _ => 1,
        })
        .sum::<usize>();
    let mut element = 0usize;
    for silencer in &exhaust.silencers {
        let c_element = |index: usize| {
            speed_of_sound(
                gamma,
                r,
                chain_element_temperature(
                    gas.collector_temperature,
                    gas.tailpipe_temperature,
                    index,
                    elements,
                ),
            ) as f64
        };
        match silencer {
            Silencer::ExpansionChamber {
                length,
                area_ratio,
                stages,
            } => {
                let volume =
                    length * exhaust.collector.outlet_area * area_ratio * (*stages).max(1) as f64;
                let c = c_element(element);
                for n in 1..=2 {
                    out.push(Predicted {
                        label: if n == 1 {
                            "expansion chamber, first pass band"
                        } else {
                            "expansion chamber, second pass band"
                        },
                        formula: format!("nc/2L = {n} x {c:.0}/(2 x {length:.3})"),
                        hz: n as f64 * c / (2.0 * length),
                        parameter: format!(
                            "chamber length {length:.3} m, volume {:.1} L",
                            volume * 1e3
                        ),
                        length: Some(*length),
                    });
                }
                element += (*stages).max(1);
            }
            Silencer::QuarterWaveStub { length, .. } => {
                let c = c_element(element);
                out.push(Predicted {
                    label: "quarter-wave stub, notch",
                    formula: format!("c/4L = {c:.0}/(4 x {length:.3})"),
                    hz: c / (4.0 * length),
                    parameter: format!("stub length {length:.3} m"),
                    length: Some(*length),
                });
                element += 1;
            }
            Silencer::Helmholtz(geometry) => {
                let c = c_element(element);
                out.push(Predicted {
                    label: "helmholtz silencer",
                    formula: format!(
                        "(c/2pi)sqrt(A/VL) = ({c:.0}/2pi)sqrt({:.5}/({:.5} x {:.3}))",
                        geometry.neck_area, geometry.chamber_volume, geometry.neck_length
                    ),
                    hz: (c / std::f64::consts::TAU)
                        * (geometry.neck_area / (geometry.chamber_volume * geometry.neck_length))
                            .sqrt(),
                    parameter: format!("chamber volume {:.1} L", geometry.chamber_volume * 1e3),
                    length: None,
                });
                element += 1;
            }
            Silencer::Absorptive { length, .. } => {
                let c = c_element(element);
                out.push(Predicted {
                    label: "absorptive silencer, first pass band",
                    formula: format!("c/2L = {c:.0}/(2 x {length:.3})"),
                    hz: c / (2.0 * length),
                    parameter: format!("silencer length {length:.3} m"),
                    length: Some(*length),
                });
                element += 1;
            }
            Silencer::Straight => {}
        }
    }

    // The whole run from the collector to the mouth, as one pipe. Every element
    // in it is a length of gas at a temperature, so its transit time is the sum
    // of theirs, and the run is closed at the collector — where four primaries
    // meet, the area steps down — and open at the mouth. This is the only
    // prediction here that is not about one component, and it exists because the
    // tallest peak in most of these spectra belongs to the system rather than to
    // any part of it.
    let mut transit = tail_acoustic / c_tail;
    let mut run_length = tail_acoustic;
    let mut index = 0usize;
    for silencer in &exhaust.silencers {
        let (length, count) = match silencer {
            Silencer::ExpansionChamber { length, stages, .. } => (*length, (*stages).max(1)),
            Silencer::Absorptive { length, .. } => (*length, 1),
            Silencer::QuarterWaveStub { .. } | Silencer::Helmholtz(_) | Silencer::Straight => {
                (0.0, 0)
            }
        };
        for _ in 0..count {
            let c = speed_of_sound(
                gamma,
                r,
                chain_element_temperature(
                    gas.collector_temperature,
                    gas.tailpipe_temperature,
                    index,
                    elements,
                ),
            ) as f64;
            transit += length / c;
            run_length += length;
            index += 1;
        }
    }
    for section in &exhaust.secondary {
        let c = speed_of_sound(gamma, r, gas.collector_temperature) as f64;
        transit += section.length / c;
        run_length += section.length;
    }
    if transit > 0.0 {
        for n in [1.0, 3.0, 5.0] {
            out.push(Predicted {
                label: match n as u32 {
                    1 => "collector to mouth, quarter wave",
                    3 => "collector to mouth, third mode",
                    _ => "collector to mouth, fifth mode",
                },
                formula: format!(
                    "{n:.0}/4T with T = sum(L/c) = {:.5} s over {run_length:.2} m",
                    transit
                ),
                hz: n / (4.0 * transit),
                parameter: format!("downstream run {run_length:.2} m"),
                length: Some(run_length),
            });
        }
    }

    // The intake. A runner behind a plenum ends in a merge into a much wider
    // cavity, so it takes the same flanged end correction a primary does;
    // individual bodies open to the air and take the trumpet's own.
    let l_runner = intake.runner_length();
    let a_runner = intake.runner_area();
    if l_runner > 0.0 {
        let itb = matches!(intake.throttle, ThrottleLayout::IndividualBodies { .. });
        let radius = (a_runner / std::f64::consts::PI).sqrt();
        let delta = end_correction(radius, if itb { intake.trumpet_flanged } else { true });
        let acoustic = l_runner + delta;
        out.push(Predicted {
            label: "intake runner, ram quarter wave",
            formula: format!("c/4(L+d) = {c_intake:.0}/(4 x ({l_runner:.3} + {delta:.3}))"),
            hz: c_intake / (4.0 * acoustic),
            parameter: format!("runner length {l_runner:.3} m"),
            length: Some(acoustic),
        });
    }
    if let Some(hz) = intake.helmholtz_resonance_hz(c_intake) {
        out.push(Predicted {
            label: "intake plenum, helmholtz",
            formula: format!(
                "(c/2pi)sqrt(A/VL) with V = {:.1} L",
                intake.plenum_volume * 1e3
            ),
            hz,
            parameter: format!("plenum volume {:.1} L", intake.plenum_volume * 1e3),
            length: None,
        });
    }

    // And the block itself, which is geometry too: a mode set by how much metal
    // is bolted together and nothing else.
    out.push(Predicted {
        label: "block, first bending mode",
        formula: format!("mass law on {:.0} kg", preset.block_mass),
        hz: block_resonance_hz(preset.block_mass as f32) as f64,
        parameter: format!("block mass {:.0} kg", preset.block_mass),
        length: None,
    });

    out.sort_by(|a, b| a.hz.total_cmp(&b.hz));
    out
}

// ---------------------------------------------------------------------------
// Measuring one engine
// ---------------------------------------------------------------------------

/// The gas state the render itself had, halfway up its sweep.
///
/// The analytic modes need a speed of sound and a speed of sound needs the
/// temperature of the gas actually in the pipe. It has to be *this* render's
/// gas, not a nominal figure and not a plateau: the exhaust warms on a time
/// constant of tens of seconds and a sweep lasts eight, so a block run to its
/// thermal plateau carries gas a few hundred kelvin hotter than anything the
/// sweep ever sees, and every prediction taken off it would sit a good tenth
/// high for no better reason than that. So the block is primed exactly as
/// [`RenderPlan::render`] primes it and stepped along the same script to its
/// midpoint, which is where a resonance smeared across a warming sweep lands.
fn gas_state(preset: &EnginePreset, script: &RenderScript) -> EngineSnapshot {
    let dt = 1.0 / PHYSICS_HZ;
    let mut block = preset.block(Environment::default());
    let mut source = SnapshotSource::with_induction(&block, preset.induction);

    for _ in 0..PRIME_STEPS {
        block.update(dt, script.start_rpm());
    }

    let mut snapshot = EngineSnapshot::default();
    let steps = (0.5 * script.seconds() * PHYSICS_HZ).round() as usize;
    for step in 0..steps {
        let (rpm, controls) = script.at(step as f64 * dt);
        block.update(dt, rpm);
        snapshot = source.sample(&block, rpm, dt, controls);
    }
    snapshot
}

/// One engine, measured and compared.
struct Calibrated {
    preset: EnginePreset,
    /// The firing order: one firing per cylinder every two revolutions.
    firing_order: f64,
    /// What the reference says, order by order.
    reference: Balance,
    /// What the synth did.
    measured: Balance,
    /// Where the two part company, on the orders the crank drives.
    driven: Comparison,
    /// The loudest order the crank cannot drive, relative to the firing order.
    worst_null: Option<(f64, f64)>,
    /// Every analytic mode, and where the audio put it.
    placements: Vec<(Predicted, Placement)>,
    /// Peaks found in the sweep average.
    peaks: Vec<Peak>,
    /// Slope of the floor over [`TILT_BAND_HZ`] [dB/octave].
    tilt: Option<f64>,
    /// The reference's own tilt, where there is a recording to take one from.
    reference_tilt: Option<f64>,
    /// Resonance placement against a recording, where there is one.
    reference_placements: Vec<(Peak, Placement)>,
    /// Where the reference came from, for the record.
    provenance: String,
    /// Whether that reference was a recording rather than the crank.
    against_recording: bool,
    /// The sweep [s] and the speed range it covered [rev/min].
    span: (f64, (f64, f64)),
}

/// Orders the crank drives hard enough to be worth comparing a level on.
///
/// The comb's own nulls are a different question with a different test, and
/// above two octaves over the firing order the comb is no longer the right
/// reference at all — see [`CRANK_COMB_ORDERS`].
fn driven_orders(reference: &Balance, firing_order: f64) -> Vec<f64> {
    reference
        .levels
        .iter()
        .filter(|l| l.order <= CRANK_COMB_ORDERS * firing_order)
        .filter(|l| l.relative_db.is_some_and(|db| db > NULL_FLOOR_DB))
        .map(|l| l.order)
        .collect()
}

/// Orders the crank leaves empty, which are the ones a null test is about.
///
/// An order carrying a line the *mechanical* rig declares is not one of them,
/// however empty the crank leaves it. Every preset in the catalogue drives an
/// accessory belt at order 1.37 and most sing a timing drive in the twenties;
/// those are lines at real orders that no crank puts there, and at this window
/// length a line at 1.37 cannot be told from one at 1.5 — a tenth of an order
/// is two bins at the top of a pull. So the rig's own orders are read off
/// [`MechanicalSpec`] and the bands around them left out of the test, which is
/// the difference between a metric about the firing pattern and one about
/// everything that happens to be near it.
fn null_orders(preset: &EnginePreset, reference: &Balance, firing_order: f64) -> Vec<f64> {
    let mechanical = mechanical_orders(preset);
    reference
        .levels
        .iter()
        .filter(|l| l.order <= CRANK_COMB_ORDERS * firing_order)
        .filter(|l| l.relative_db.is_none_or(|db| db <= NULL_FLOOR_DB))
        .filter(|l| {
            !mechanical
                .iter()
                .any(|m| (m - l.order).abs() < BACKGROUND_OFFSET)
        })
        .map(|l| l.order)
        .collect()
}

/// Every order the mechanical rig puts a line on.
///
/// A per-cylinder source lands on the firing order by definition; the rest
/// declare an order outright, and a belt drive's is not even a whole one.
fn mechanical_orders(preset: &EnginePreset) -> Vec<f64> {
    let spec = &preset.mechanical;
    let firing_order = preset.firing.len() as f64 / 2.0;
    [
        spec.intake_valve,
        spec.exhaust_valve,
        spec.piston_slap,
        spec.injector,
        spec.timing_chain,
        spec.gear_whine,
        spec.accessory,
    ]
    .into_iter()
    .flatten()
    .map(|source| match source.rate {
        SourceRate::PerCylinder => firing_order,
        SourceRate::Order(order) => order as f64,
    })
    .collect()
}

/// Renders the sweep and compares it against whatever reference was supplied.
fn calibrate(preset: EnginePreset, recording: Option<&Recorded>) -> Result<Calibrated> {
    let wanted = half_orders();
    let firing_order = preset.firing.len() as f64 / 2.0;

    // The synth, over the reference's own speed range where there is one, and
    // over its own rev range otherwise.
    let script = match recording {
        // The recording's own curve, leg by leg, rather than a ramp between its
        // endpoints: a real pull idles, climbs, holds and falls away, and a
        // recording that ends where it started would otherwise be rendered as
        // an engine that never moved.
        Some(recorded) => script::following(
            "reference_match",
            "the reference recording's own pull, rendered",
            &recorded.rpm,
            REFERENCE_LEGS,
        ),
        None => script::calibration_sweep(&preset),
    };
    let render = RenderPlan::new(&preset, &script).render();
    let mono = render.mono();
    let synth = Reference::extract(&mono, render.sample_rate, &render.rpm, &wanted);

    let measured = synth.orders.line_balance(firing_order);
    let peaks = synth.peaks(PEAKS);

    // The reference: a recording if one was supplied, the crank otherwise.
    let (reference, provenance, reference_tilt, reference_placements) = match recording {
        Some(recorded) => {
            let extracted = &recorded.reference;
            let reference_peaks = extracted.peaks(PEAKS);
            // Every peak the recording has, and where the synth put it.
            let paired = reference_peaks
                .iter()
                .map(|peak| (*peak, place(peak.hz, &peaks, PLACEMENT_WINDOW_PCT)))
                .collect();
            (
                extracted.orders.line_balance(firing_order),
                recorded.provenance.clone(),
                extracted.tilt_db_per_octave(TILT_BAND_HZ.0, TILT_BAND_HZ.1),
                paired,
            )
        }
        None => {
            let banks: Vec<Vec<f64>> = (0..preset.firing.bank_count())
                .map(|bank| preset.firing.bank_offsets(bank as u8))
                .collect();
            (
                orders::crank_balance(&banks, &wanted, firing_order),
                format!(
                    "the crank itself: {} firings a cycle on {} bank{}",
                    preset.firing.len(),
                    banks.len(),
                    if banks.len() == 1 { "" } else { "s" }
                ),
                None,
                Vec::new(),
            )
        }
    };

    let driven = measured
        .only(&driven_orders(&reference, firing_order))
        .against(&reference);
    let worst_null = null_orders(&preset, &reference, firing_order)
        .into_iter()
        .filter_map(|order| measured.at(order).map(|db| (order, db)))
        .max_by(|a, b| a.1.total_cmp(&b.1));

    // Placed as a set rather than one at a time: a sparse spectrum will
    // otherwise hand the same peak to an intake runner, an exhaust primary and
    // a silencer, and report three modes where there is one.
    let gas = gas_state(&preset, &script);
    let predicted = predictions(&preset, &gas);
    let frequencies: Vec<f64> = predicted.iter().map(|p| p.hz).collect();
    let placements = predicted
        .into_iter()
        .zip(place_all(&frequencies, &peaks, PLACEMENT_WINDOW_PCT))
        .collect();

    Ok(Calibrated {
        preset,
        firing_order,
        reference,
        measured,
        driven,
        worst_null,
        placements,
        peaks,
        tilt: synth.tilt_db_per_octave(TILT_BAND_HZ.0, TILT_BAND_HZ.1),
        reference_tilt,
        reference_placements,
        provenance,
        against_recording: recording.is_some(),
        span: (synth.seconds, synth.speed),
    })
}

impl Calibrated {
    /// Whether the crank's comb is the whole story for this engine.
    ///
    /// It is, for one bank breathing atmospherically: the firing pattern is
    /// then the only thing in the engine that repeats once a cycle, and the
    /// orders it cannot drive should carry no line at all.
    ///
    /// It is not, for two other kinds of engine, and both exclusions are
    /// findings rather than conveniences:
    ///
    /// - **A vee.** Two banks fire into two collectors and radiate from two
    ///   tailpipes at two places on the car, and how much their pulse trains
    ///   cancel in the mix is a property of those paths. Summed coherently the
    ///   banks of a cross-plane V8 would leave *nothing* on order 1.5 and
    ///   double order 4; summed in power they leave both. Neither extreme is
    ///   what a pair of real pipes does, so the comb brackets a vee's
    ///   half-orders rather than predicting them.
    /// - **Forced induction.** A compressor puts a line at its own blade-pass
    ///   order, which is the *shaft's* order and not the crank's. The shaft
    ///   turns at roughly a fixed ratio to the engine, so the whistle lands at
    ///   a steady engine order that the crank has no null at and no line on.
    ///
    /// Both are still reported; they are just not failures of the crank.
    fn comb_is_unambiguous(&self) -> bool {
        self.preset.firing.bank_count() <= 1 && !self.preset.is_forced()
    }

    /// Whether the order balance holds to [`ORDER_TOLERANCE_DB`].
    ///
    /// Only asked of a recording; see [`ORDER_TOLERANCE_DB`].
    fn orders_agree(&self) -> bool {
        !self.against_recording || self.driven.within(ORDER_TOLERANCE_DB)
    }

    /// Whether the orders the crank cannot drive stayed under the note.
    fn nulls_agree(&self) -> bool {
        !self.comb_is_unambiguous() || self.worst_null.is_none_or(|(_, db)| db <= NULL_FLOOR_DB)
    }

    /// Whether every analytic mode the audio does have lands within tolerance.
    fn placements_agree(&self) -> bool {
        let (within, found, _) = self.placement_score();
        within == found
    }

    /// Analytic modes found within tolerance, out of those found at all.
    fn placement_score(&self) -> (usize, usize, usize) {
        let found = self
            .placements
            .iter()
            .filter(|(_, p)| p.measured_hz.is_some())
            .count();
        let within = self
            .placements
            .iter()
            .filter(|(_, p)| p.within(PLACEMENT_TOLERANCE_PCT))
            .count();
        (within, found, self.placements.len())
    }
}

// ---------------------------------------------------------------------------
// The reference recording
// ---------------------------------------------------------------------------

/// A reference recording, read off disk and order-tracked.
struct Recorded {
    reference: Reference,
    rpm: RpmCurve,
    provenance: String,
}

/// Reads a recording and order-tracks it against a supplied speed curve.
fn record(path: &Path, rpm: RpmCurve, orders: &[f64]) -> Result<Recorded> {
    let audio = read_wav(path)?;
    let mono = audio.mono();
    let seconds = audio.seconds();
    // The curve was described in its own terms — two endpoints, or points off a
    // tachometer — and has to be stretched onto the recording it belongs to, or
    // every order in the table is read at the wrong frequency.
    let rpm = stretch(&rpm, seconds);
    Ok(Recorded {
        provenance: format!(
            "{} · {:.1} s · {:.0} channel{} at {:.0} Hz · a supplied curve over {:.0}-{:.0} rpm",
            path.display(),
            seconds,
            audio.channels,
            if audio.channels == 1 { "" } else { "s" },
            audio.sample_rate,
            rpm.range(0.0, seconds).0,
            rpm.range(0.0, seconds).1,
        ),
        reference: Reference::extract(&mono, audio.sample_rate, &rpm, orders),
        rpm,
    })
}

/// The same speed curve, stretched to cover `seconds`.
fn stretch(rpm: &RpmCurve, seconds: f64) -> RpmCurve {
    let steps = 512;
    let points: Vec<(f64, f64)> = (0..=steps)
        .map(|i| {
            let u = i as f64 / steps as f64;
            (u * seconds, rpm.at(u * rpm.seconds()))
        })
        .collect();
    RpmCurve::from_points(&points, seconds / steps as f64)
}

/// Parses `--rpm`: either `from:to`, or `t=rpm,t=rpm,...` off a tachometer.
fn parse_rpm(spec: &str) -> Result<RpmCurve> {
    if let Some((from, to)) = spec.split_once(':') {
        let from: f64 = from.trim().parse().context("--rpm from")?;
        let to: f64 = to.trim().parse().context("--rpm to")?;
        // The length is the recording's; this curve only has to carry a shape.
        return Ok(RpmCurve::sweep(from, to, 1.0));
    }

    let mut points = Vec::new();
    for pair in spec.split(',') {
        let (t, rpm) = pair
            .split_once('=')
            .with_context(|| format!("--rpm expects from:to or t=rpm pairs, got {pair:?}"))?;
        points.push((
            t.trim().parse::<f64>().context("--rpm time")?,
            rpm.trim().parse::<f64>().context("--rpm speed")?,
        ));
    }
    if points.len() < 2 {
        anyhow::bail!("--rpm needs at least two points to be a curve");
    }
    let span = points
        .iter()
        .map(|p| p.0)
        .fold(0.0f64, f64::max)
        .max(f64::MIN_POSITIVE);
    Ok(RpmCurve::from_points(&points, span / 512.0))
}

// ---------------------------------------------------------------------------
// Formatting
// ---------------------------------------------------------------------------

/// `4` rather than `4.0`, but `4.5` still `4.5`.
fn trim(order: f64) -> String {
    if order.fract().abs() < 1e-9 {
        format!("{order:.0}")
    } else {
        format!("{order:.1}")
    }
}

/// A level relative to the firing order, or a dash where it was unresolvable.
fn level(balance: &Balance, order: f64) -> String {
    match balance.at(order) {
        Some(db) if db > orders::SILENCE_DB + 1.0 => format!("{db:+.1}"),
        Some(_) => "null".into(),
        None => "—".into(),
    }
}

fn print_engine(calibrated: &Calibrated) {
    let (seconds, (slow, fast)) = calibrated.span;
    println!("\n== {} ==", calibrated.preset.name);
    println!(
        "  {}  ·  firing order {}",
        calibrated.preset.spec(),
        trim(calibrated.firing_order)
    );
    println!("  reference: {}", calibrated.provenance);
    println!("  sweep: {slow:.0}-{fast:.0} rpm over {seconds:.1} s");

    println!(
        "\n  order balance [dB relative to order {}]",
        trim(calibrated.firing_order)
    );
    println!(
        "  {:>7}{:>12}{:>10}{:>10}",
        "order", "reference", "synth", "delta"
    );
    for level_row in &calibrated.reference.levels {
        let order = level_row.order;
        if order > CRANK_COMB_ORDERS * calibrated.firing_order {
            continue;
        }
        let delta = calibrated
            .driven
            .at(order)
            .map_or_else(|| "—".into(), |d| format!("{d:+.1}"));
        println!(
            "  {:>7}{:>12}{:>10}{:>10}",
            trim(order),
            level(&calibrated.reference, order),
            level(&calibrated.measured, order),
            delta,
        );
    }

    let worst = calibrated.driven.worst().map_or_else(
        || "nothing compared".into(),
        |d| format!("{:+.1} dB on order {}", d.delta_db, trim(d.order)),
    );
    println!(
        "  driven orders: rms {:.1} dB, mean {:+.1} dB, worst {worst}  [{}]",
        calibrated.driven.rms_db(),
        calibrated.driven.mean_db(),
        if calibrated.against_recording {
            format!(
                "tolerance {ORDER_TOLERANCE_DB:.0} dB: {}",
                if calibrated.orders_agree() {
                    "pass"
                } else {
                    "FAIL"
                }
            )
        } else {
            "reported: the crank is an excitation, not a radiated sound".into()
        },
    );
    match (calibrated.worst_null, calibrated.comb_is_unambiguous()) {
        (Some((order, db)), true) => println!(
            "  crank nulls:   loudest {db:+.1} dB on order {}  [limit {NULL_FLOOR_DB:.0} dB: {}]",
            trim(order),
            if calibrated.nulls_agree() {
                "pass"
            } else {
                "FAIL"
            },
        ),
        (Some((order, db)), false) if calibrated.against_recording => println!(
            "  quiet orders:  loudest {db:+.1} dB on order {}  [quiet in the reference too]",
            trim(order),
        ),
        (Some((order, db)), false) => println!(
            "  crank nulls:   loudest {db:+.1} dB on order {}  [not enforced: {}]",
            trim(order),
            if calibrated.preset.firing.bank_count() > 1 {
                "two banks, so the comb only brackets the half-orders"
            } else {
                "a compressor puts a line on the shaft's order"
            },
        ),
        (None, _) => println!("  crank nulls:   none resolvable in this sweep"),
    }

    println!("\n  resonance placement [Hz]");
    println!(
        "  {:<38}{:>10}{:>10}{:>9}  implies",
        "mode", "predicted", "measured", "error"
    );
    for (predicted, placement) in &calibrated.placements {
        let implied = match (placement.implied_length(0.0), predicted.length) {
            (_, Some(length)) => placement
                .implied_length(length)
                .map_or_else(String::new, |l| {
                    format!("{} → {l:.3} m", predicted.parameter)
                }),
            _ => String::new(),
        };
        println!(
            "  {:<38}{:>10.1}{:>10}{:>9}  {implied}",
            predicted.label,
            predicted.hz,
            placement
                .measured_hz
                .map_or_else(|| "not found".into(), |hz| format!("{hz:.1}")),
            placement
                .error_pct()
                .map_or_else(|| "—".into(), |e| format!("{e:+.1}%")),
        );
    }
    let (within, found, all) = calibrated.placement_score();
    println!(
        "  {within} of {found} modes found are within {PLACEMENT_TOLERANCE_PCT:.0} %, \
         out of {all} predicted  [{}]",
        if calibrated.placements_agree() {
            "pass"
        } else {
            "FAIL"
        }
    );

    if !calibrated.reference_placements.is_empty() {
        println!("\n  the recording's own peaks, and where the synth put them [Hz]");
        println!(
            "  {:>10}{:>12}{:>10}{:>9}",
            "reference", "prominence", "synth", "error"
        );
        for (peak, placement) in &calibrated.reference_placements {
            println!(
                "  {:>10.1}{:>12.1}{:>10}{:>9}",
                peak.hz,
                peak.prominence_db,
                placement
                    .measured_hz
                    .map_or_else(|| "not found".into(), |hz| format!("{hz:.1}")),
                placement
                    .error_pct()
                    .map_or_else(|| "—".into(), |e| format!("{e:+.1}%")),
            );
        }
    }

    print!(
        "\n  noise floor tilt over {:.0} Hz-{:.0} kHz: ",
        TILT_BAND_HZ.0,
        TILT_BAND_HZ.1 / 1e3
    );
    match calibrated.tilt {
        Some(tilt) => print!("{tilt:+.1} dB/octave"),
        None => print!("not measurable"),
    }
    match calibrated.reference_tilt {
        Some(tilt) => println!("  (reference {tilt:+.1})"),
        None => println!(),
    }
}

// ---------------------------------------------------------------------------
// The recorded calibration
// ---------------------------------------------------------------------------

fn markdown(all: &[Calibrated]) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "# Calibration\n");
    let _ = writeln!(
        out,
        "Stage 16 of [the implementation plan](../IMPLEMENTATION_PLAN.md): the \
         synth measured against a reference it did not produce. Regenerate with:\n"
    );
    let _ = writeln!(
        out,
        "```bash\ncargo run --release --example calibrate -- --markdown \
         docs/measurements/calibration.md\n```\n"
    );
    let _ = writeln!(
        out,
        "Every figure below is *relative*: an order against the firing order, a \
         measured frequency against a predicted one, a slope against an octave. \
         That is deliberate and it is the point of the stage. A gain constant \
         anywhere in the chain moves every order by the same number of decibels \
         and cancels out of all of it, so nothing here can be closed by turning \
         something up — only by changing a length, a volume or a radius.\n"
    );
    let _ = writeln!(
        out,
        "| Engine | Firing order | Order balance rms | Worst order | Crank nulls | \
         Modes placed | Floor tilt |"
    );
    let _ = writeln!(out, "|---|---:|---:|---:|---:|---:|---:|");
    for c in all {
        let (within, found, all_modes) = c.placement_score();
        let _ = writeln!(
            out,
            "| [{}](#{}) | {} | {:.1} dB | {} | {} | {within}/{found} of {all_modes} | {} |",
            c.preset.name,
            anchor(c.preset.name),
            trim(c.firing_order),
            c.driven.rms_db(),
            c.driven.worst().map_or_else(
                || "—".into(),
                |d| format!("{:+.1} dB on {}", d.delta_db, trim(d.order))
            ),
            c.worst_null.map_or_else(
                || "—".into(),
                |(order, db)| format!(
                    "{db:+.1} dB on {}{}",
                    trim(order),
                    if c.comb_is_unambiguous() {
                        ""
                    } else {
                        ", not enforced"
                    }
                )
            ),
            c.tilt
                .map_or_else(|| "—".into(), |t| format!("{t:+.1} dB/oct")),
        );
    }

    let _ = writeln!(out, "\n## Method\n");
    let _ = writeln!(
        out,
        "One unbroken pull from idle to the redline over {CALIBRATION_SECONDS:.0} s at \
         {:.0} kHz, no hold at either end, analysed with the Stage 0 harness: an \
         8192-point Hann STFT, orders read against the speed curve the render was \
         driven by, resonances picked off the Welch average of the same audio. A \
         hold at one speed would leave its own order lines standing in that \
         average exactly where a resonance would, which is why the calibration \
         sweep is not one of the six Stage 0 profiles.\n",
        OFFLINE_RATE / 1e3
    );
    let _ = writeln!(
        out,
        "**Order balance** is compared against the *crank comb*: the order \
         spectrum of the firing pattern itself, `|sum exp(-i n theta_k)|` over \
         the firing angles of each bank, summed in power across banks. It is a \
         reference this repository can carry — derived from the crank, not \
         recorded, and not tunable. Orders are compared up to \
         {CRANK_COMB_ORDERS:.0} x the firing order, above which a firing is no \
         longer usefully a Dirac impulse: blowdown lasts a valve event, a valve \
         event is a fixed number of crank degrees, and its envelope rolls the \
         comb off from somewhere above there.\n"
    );
    let _ = writeln!(
        out,
        "**Resonance placement** is compared against the analytic modes of the \
         declared geometry, in the gas this render actually had: the block is \
         primed as the render primes it and stepped along the same sweep to its \
         midpoint, because the exhaust warms on a time constant of tens of \
         seconds and a sweep lasts eight. A prediction taken off a block at its \
         thermal plateau would sit a tenth high on every mode for no reason but \
         that. Tolerance {PLACEMENT_TOLERANCE_PCT:.0} %, and each measured peak \
         is given to at most one prediction — the nearest — so a sparse \
         spectrum cannot report one peak as three modes. A peak further than \
         {PLACEMENT_WINDOW_PCT:.0} % from a prediction is reported as *not \
         found* rather than as that mode in the wrong place.\n"
    );
    let _ = writeln!(
        out,
        "**The `implies` column** is the stage's one rule as arithmetic: the \
         length that would put the mode where it was actually measured. A mode \
         8 % low is not an equaliser's problem, it is a pipe 9 % longer than the \
         one it was credited with.\n"
    );

    let _ = writeln!(
        out,
        "**No reference recording is committed to this repository.** `*.wav` is \
         gitignored and no recording of a real engine is licensed for \
         redistribution here, so the reference above is the crank and the \
         declared geometry. The recording path is real and is the one this \
         stage was built for — point it at a pull of your own with:\n"
    );
    let _ = writeln!(
        out,
        "```bash\ncargo run --release --example calibrate -- --preset Inline-4 \
         \\\n    --reference pull.wav --rpm 1200:6800\n```\n"
    );
    let _ = writeln!(
        out,
        "`--rpm` also takes a list of `t=rpm` points read off a tachometer, for \
         a pull that is not linear. Whatever is supplied, record the recording \
         and the curve next to the table it produced: a calibration whose \
         reference cannot be identified is not attributable, and a later \
         regression against it cannot be either.\n"
    );

    let _ = writeln!(out, "## Open questions\n");
    let _ = writeln!(
        out,
        "Recorded rather than closed, as Stage 16 requires: each of these is a \
         disagreement that no length, volume or radius in the declared geometry \
         accounts for, and none of them is closed with a gain.\n"
    );
    let _ = writeln!(
        out,
        "1. **The blown vees' chamber pass bands land 16 to 20 % high, by the \
         same factor at both harmonics.** One factor on two harmonics is a \
         speed of sound or a length, not a misidentified peak — a misread peak \
         would be wrong by different amounts at different frequencies. Neither \
         the declared chamber length nor the thermal gradient the chain is tuned \
         down accounts for it, and the engines whose chains are the same shape \
         but atmospheric (the V10, the V12) place theirs inside 6 %. Open.\n"
    );
    let _ = writeln!(
        out,
        "2. **A vee's half-order content depends on how much its two banks \
         cancel, and the comb can only bracket that.** Summed coherently the \
         banks of a cross-plane V8 leave nothing at all on order 1.5; summed in \
         power they leave it 3.7 dB under the firing order. The V12 shows the \
         same question from the other side: its per-bank order 3 comes out well \
         above its engine order 6, so as measured it is two straight sixes \
         rather than one V12, where a real V12's sixth order is its voice. If \
         that is wrong the fix is in the bank paths — lengths, the crossover, \
         where the two tailpipes sit — and not in a level.\n"
    );
    let _ = writeln!(
        out,
        "3. **The crank comb is a comb of Dirac impulses and a firing is not \
         one.** Every engine here reads its first harmonic above the firing \
         order 10 to 26 dB under what the comb says, and that is the blowdown \
         envelope: a pulse lasting a valve event is a fixed width in crank \
         degrees, so it rolls the comb off at a fixed order. Putting it in the \
         reference needs the fraction of the event that blowdown occupies, and \
         choosing that fraction to fit the measurement is exactly the kind of \
         constant this stage exists to refuse. Left out, and the comparison \
         stops at {CRANK_COMB_ORDERS:.0} x the firing order.\n"
    );
    let _ = writeln!(
        out,
        "4. **A primary's measured mode implies an acoustic length a few per \
         cent off its declared centre line, in both directions.** The model's \
         collector junction is memoryless, so the extra length a primary shows \
         is its collector taper's own delay line; the residual runs from -6 % to \
         +14 % across the catalogue with no consistent sign. No declared primary \
         length is changed on the strength of it: a correction that scatters \
         both ways is scatter, and fitting each preset to its own scatter would \
         be the tone knob in different clothes.\n"
    );
    let _ = writeln!(
        out,
        "5. **A primary's quarter wave is now a hump rather than a peak, and \
         mostly reports as not found.** It used to be the loudest thing near its \
         own frequency, and stretching the inline-four's primaries by half moved \
         it bodily from 403 Hz to 244 Hz. That was a nearly lossless network \
         talking: with the wall taking a fiftieth of the decibels `alpha` asks \
         for, the whole run from the valve to the mouth was one resonator of \
         enormous Q. With the loss filter tracking `alpha` and the valve opening \
         once a cycle, a primary's loop pays about 1.4 dB a round trip and the \
         four-into-one behind it sends most of what arrives onward. The length \
         still reaches the sound — the band's centre of gravity moves down \
         monotonically as the primaries are stretched, which is what the test \
         suite now asserts — but it no longer stands up as a peak the picker can \
         place, and the count of modes found in the table above fell across the \
         catalogue when it stopped doing so. Whether a real header's primary is \
         more prominent than this one is the open question, and it is a question \
         about the enhancement factor on the wall loss, which has no \
         derivation.\n"
    );
    let _ = writeln!(
        out,
        "6. **Several predicted modes leave no peak at all.** The turbodiesel is \
         the extreme — it radiates through its block rather than its pipe, so \
         the exhaust chain barely reaches the listener and five of its twelve \
         predicted modes are absent rather than misplaced. Absence is the \
         honest report: a peak found more than {PLACEMENT_WINDOW_PCT:.0} % from \
         a prediction is a different mode, not that one in the wrong place.\n"
    );
    let _ = writeln!(
        out,
        "7. **The primaries steepen correctly and are not allowed to.** \
         `WaveguidePipe` no longer advances a crest by a factor and clamps it. \
         Each point of the stored waveform is a characteristic travelling at \
         `c(1 + (gamma+1)/(2 gamma) p/P_0)`, the pipe reads out the one that \
         arrives first, and where a crest has overtaken the trough ahead of it \
         the output steps rather than rises; the front then pays the shock's \
         own dissipation, `sigma = (gamma+1)/(2 gamma) D |dp| / P_0` past one \
         being the sawtooth decay of a jump the section can no longer steepen. \
         Measured on a half-metre primary at 800 K, second harmonic against \
         fundamental: 0.0005 at 100 Pa, 0.024 at 5 kPa, 0.095 at 20 kPa, 0.32 \
         at one atmosphere. Excited once and left alone in a 90 per cent \
         reflecting loop it decays faster than the linear pipe does, so it is \
         not the transposed-form mistake in another costume.\n\n\
         It is switched off. Driven at the pressure a port actually launches — \
         `c mdot / A`, 0.31 atmospheres at idle and 0.65 at the limiter across \
         this catalogue, a third of the difference across the valve because a \
         port is a restriction and not an open end — three of this \
         repository's own guards fail. The band the primaries work in stops \
         falling as they are stretched: 234.6, 259.8, 237.4 Hz over a \
         half-again stretch, where it has to fall every step. The cam's grip \
         on mid-band tilt falls from over ten decibels to five and a half. The \
         limiter bounce stops standing out of a clean pull. The cause is open \
         question 5: a primary's loop pays about 1.4 dB a round trip, so a \
         pulse goes round it some thirty times and steepens on every one of \
         them, and a nonlinearity accumulated thirty times is louder than the \
         geometry it is supposed to be colouring. Four times the wall \
         enhancement brings two of the three guards back, which is precisely \
         the constant question 5 says has no derivation, so it is not taken. \
         What this wants is either that derivation, or the nonlinearity \
         applied to the launched pulse on its one-way run down the primary and \
         not to the resonant field behind it — `beta` is proportional to `p` \
         and the field is twenty decibels under the pulse, so steepening the \
         one and not the other is a statement about where the gas is \
         nonlinear rather than a knob. Open.\n"
    );
    let _ = writeln!(
        out,
        "8. **The port jet's dipole efficiency is the one number in the exhaust \
         path with a range instead of a derivation.** Curle's law says a flow \
         past a solid boundary radiates as the sixth power of velocity and \
         leaves the constant to measurement, so `JET_DIPOLE_EFFICIENCY` is \
         measured and not derived. It is set at 0.005, which is *below* the \
         published range for an orifice in a duct, and it is there because that \
         is the loudest this catalogue's own guards allow: at 0.006 the \
         mechanical floor stops measurably filling the gaps between firings, \
         and at 0.02 the pulse stops reading as linear in the pressure \
         difference that made it. Either the jet is genuinely this quiet in a \
         runner — plausible, since a jet a fifth of the pipe's area couples \
         into a plane wave badly — or the mechanical layer under it is too \
         quiet and is masking how much room there is. The measurement that \
         would settle it is the one this repository does not have: the \
         tone-to-floor ratio of a recording of a real engine, which runs 5 to \
         12 dB a bin where this catalogue runs 11 to 19. Open.\n"
    );
    let _ = writeln!(
        out,
        "9. **The blowers' loudness is derived, their level is not, and it \
         cannot be until the solver makes boost.** A displacement blower's \
         noise is its rotors handing pockets of gas to a discharge port, and a \
         volume velocity across an aperture launches `c mdot / A` like every \
         other aperture here, so both supercharger voices now scale with the \
         induction mass flow the solver actually reports. That replaced a law \
         that went as the square of crank speed times a throttle term, which \
         had a blower spinning fast on a shut throttle at a third of full voice \
         when a bypassed blower is pumping almost nothing. What did not change \
         is the constant in front, and it cannot: nothing in the physics \
         produces boost, so there is no pressure ratio across the machine for \
         its loudness to be a fraction of, and there is no bypass to open when \
         the throttle shuts. The audible consequence is open question 2's other \
         half — on the blown V8 at 4500 rpm the whine at order 8.4 is still \
         among the three loudest things in the render and the firing order is \
         not in the top seven, which is the sound of an inverter and not of an \
         engine. It closes with the compressor model in Stage 13, not with a \
         number. Open.\n"
    );

    for c in all {
        let (seconds, (slow, fast)) = c.span;
        let _ = writeln!(out, "\n## {}\n", c.preset.name);
        let _ = writeln!(out, "> {}\n", c.preset.note);
        let _ = writeln!(
            out,
            "{}  ·  firing order {}  ·  {slow:.0}-{fast:.0} rpm over {seconds:.1} s\n",
            c.preset.spec(),
            trim(c.firing_order)
        );
        let _ = writeln!(out, "Reference: {}\n", c.provenance);

        let _ = writeln!(
            out,
            "### Order balance [dB relative to order {}]\n",
            trim(c.firing_order)
        );
        let _ = writeln!(out, "| Order | Reference | Synth | Delta |");
        let _ = writeln!(out, "|---:|---:|---:|---:|");
        for row in &c.reference.levels {
            if row.order > CRANK_COMB_ORDERS * c.firing_order {
                continue;
            }
            let firing = (row.order - c.firing_order).abs() < 1e-9;
            let label = if firing {
                format!("**{}**", trim(row.order))
            } else {
                trim(row.order)
            };
            let _ = writeln!(
                out,
                "| {label} | {} | {} | {} |",
                level(&c.reference, row.order),
                level(&c.measured, row.order),
                c.driven
                    .at(row.order)
                    .map_or_else(|| "—".into(), |d| format!("{d:+.1}")),
            );
        }
        let _ = writeln!(
            out,
            "\nDriven orders: rms **{:.1} dB**, mean {:+.1} dB, worst {} \
             (tolerance {ORDER_TOLERANCE_DB:.0} dB, {}). Orders the crank cannot \
             drive are marked `null` and are held under {NULL_FLOOR_DB:.0} dB \
             instead of compared: {}.\n",
            c.driven.rms_db(),
            c.driven.mean_db(),
            c.driven.worst().map_or_else(
                || "nothing".into(),
                |d| format!("{:+.1} dB on order {}", d.delta_db, trim(d.order))
            ),
            if c.orders_agree() { "pass" } else { "**fail**" },
            c.worst_null.map_or_else(
                || "none resolvable".into(),
                |(order, db)| format!(
                    "loudest {db:+.1} dB on order {} ({})",
                    trim(order),
                    if c.nulls_agree() { "pass" } else { "**fail**" }
                )
            ),
        );

        let _ = writeln!(out, "### Resonance placement\n");
        let _ = writeln!(
            out,
            "| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | \
             Level [dBFS] | Prominence [dB] | Implies |"
        );
        let _ = writeln!(out, "|---|---|---:|---:|---:|---:|---:|---|");
        for (predicted, placement) in &c.placements {
            let implied = predicted
                .length
                .and_then(|l| placement.implied_length(l))
                .map_or_else(String::new, |l| {
                    format!("{} → {l:.3} m", predicted.parameter)
                });
            // The level and the prominence are what say how much a placement
            // is worth: a mode standing 3 dB out of its surroundings is a
            // shoulder that happened to be near a prediction, and one standing
            // 25 dB out is the mode.
            let _ = writeln!(
                out,
                "| {} | `{}` | {:.1} | {} | {} | {} | {} | {implied} |",
                predicted.label,
                predicted.formula,
                predicted.hz,
                placement
                    .measured_hz
                    .map_or_else(|| "not found".into(), |hz| format!("{hz:.1}")),
                placement
                    .error_pct()
                    .map_or_else(|| "—".into(), |e| format!("{e:+.1} %")),
                placement
                    .db
                    .map_or_else(|| "—".into(), |db| format!("{db:.1}")),
                placement
                    .prominence_db
                    .map_or_else(|| "—".into(), |db| format!("{db:.1}")),
            );
        }

        let _ = writeln!(out, "\n### Peaks in the sweep average\n");
        let _ = writeln!(out, "| Hz | dBFS | Prominence [dB] |");
        let _ = writeln!(out, "|---:|---:|---:|");
        for peak in &c.peaks {
            let _ = writeln!(
                out,
                "| {:.1} | {:.1} | {:.1} |",
                peak.hz, peak.db, peak.prominence_db
            );
        }

        let _ = writeln!(
            out,
            "\nNoise floor tilt over {:.0} Hz-{:.0} kHz: **{}**.\n",
            TILT_BAND_HZ.0,
            TILT_BAND_HZ.1 / 1e3,
            c.tilt
                .map_or_else(|| "not measurable".into(), |t| format!("{t:+.1} dB/octave")),
        );
    }
    out
}

/// A GitHub-style heading anchor for an engine's name.
fn anchor(name: &str) -> String {
    name.chars()
        .filter(|c| c.is_ascii_alphanumeric() || *c == '-' || *c == ' ')
        .map(|c| {
            if c == ' ' {
                '-'
            } else {
                c.to_ascii_lowercase()
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------

fn main() -> Result<()> {
    let mut only: Option<String> = None;
    let mut reference: Option<PathBuf> = None;
    let mut rpm: Option<String> = None;
    let mut markdown_path: Option<PathBuf> = None;
    let mut fingerprints = false;

    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--preset" => only = args.next(),
            "--reference" => reference = args.next().map(PathBuf::from),
            "--rpm" => rpm = args.next(),
            "--markdown" => markdown_path = args.next().map(PathBuf::from),
            "--fingerprints" => fingerprints = true,
            "--help" | "-h" => {
                println!(
                    "calibrate [--preset NAME] [--reference FILE.wav --rpm FROM:TO] \
                     [--markdown FILE.md] [--fingerprints]\n\n\
                     --rpm takes either FROM:TO for a linear pull or a list of \
                     t=rpm points read off a tachometer."
                );
                return Ok(());
            }
            other => eprintln!("ignoring unknown argument: {other}"),
        }
    }

    let presets: Vec<EnginePreset> = EnginePreset::catalogue()
        .into_iter()
        .filter(|p| match &only {
            Some(name) => p.name.to_lowercase().contains(&name.to_lowercase()),
            None => true,
        })
        .collect();
    if presets.is_empty() {
        anyhow::bail!("no engine in the catalogue matches {only:?}");
    }

    // Re-recording the timbre regression's baseline: the same sweep and the
    // same analysis, printed as the Rust table the test compares against. Kept
    // in the example rather than written by the test itself, so that moving a
    // number is a diff a reviewer reads rather than a file a test rewrites.
    if fingerprints {
        println!("pub const RECORDED: &[Fingerprint] = &[");
        for preset in &presets {
            print!("{}", timbre::measure(preset).literal());
        }
        println!("];");
        return Ok(());
    }

    let recording = match (&reference, &rpm) {
        (Some(path), Some(spec)) => {
            if presets.len() > 1 {
                anyhow::bail!(
                    "a recording is of one engine: name it with --preset, not {} of them",
                    presets.len()
                );
            }
            Some(record(path, parse_rpm(spec)?, &half_orders())?)
        }
        (Some(_), None) => anyhow::bail!(
            "--reference needs --rpm: nothing in a recording says how fast the \
             engine was turning, and a curve guessed from the audio would make \
             the order table a restatement of the guess"
        ),
        (None, Some(_)) => anyhow::bail!("--rpm describes a --reference, and there is none"),
        (None, None) => None,
    };

    println!(
        "calibrating {} engine{} against {}",
        presets.len(),
        if presets.len() == 1 { "" } else { "s" },
        match &recording {
            Some(recorded) => recorded.provenance.clone(),
            None => "the crank and the declared geometry".into(),
        }
    );

    let mut all = Vec::new();
    for preset in presets {
        let calibrated = calibrate(preset, recording.as_ref())?;
        print_engine(&calibrated);
        all.push(calibrated);
    }

    if let Some(path) = &markdown_path {
        if let Some(dir) = path.parent() {
            fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
        }
        fs::write(path, markdown(&all)).with_context(|| format!("writing {}", path.display()))?;
        println!("\nwrote {}", path.display());
    }

    let failed: Vec<&str> = all
        .iter()
        .filter(|c| !c.orders_agree() || !c.nulls_agree() || !c.placements_agree())
        .map(|c| c.preset.name)
        .collect();
    if !failed.is_empty() {
        eprintln!(
            "\noutside tolerance on {} — and every figure above is relative, so \
             the fix is a length, a volume or a radius",
            failed.join(", ")
        );
    }
    Ok(())
}
