# Stage T6 — bandlimit the pops

Stage T6 of [the timbre plan](../TIMBRE_PLAN.md): the backfire's attack was a
one-sample step into a soft clipper, unbandlimited, and triggered on a
16-sample control grid; its trigger-site gain then doubled it on top of an
already-tuned `backfire_level`. Per
[Definition of done, rule 7](../TIMBRE_PLAN.md#definition-of-done-every-stage),
this records the stage's effect on octave-band share and crest factor against
the pre-Stage-0 reference, whether or not it moved as intended.

## No effect on the calibration fingerprint, and that is expected

`RECORDED` in [`src/analysis/timbre.rs`](../../src/analysis/timbre.rs) comes
from `examples/calibrate.rs`'s steady-state sweep, which never sets
`spark_cut`. A backfire only fires during a cut, so a stage that only touches
`BackfireVoice` and its trigger cannot move a fingerprint that never triggers
one — confirmed by `no_preset_has_drifted_from_its_recorded_timbre`, which
passes unchanged before and after every commit in this stage. This is not a
gap in the stage; the calibration sweep measures the wrong scenario for it by
design, which is why T6's own Tests section asks for a limiter-bounce
measurement instead.

## The re-levelling, measured on the plan's own reference case

`examples/diagnose_sound --preset "cross-plane v8"`, the same tool and preset
[the Measured section](../TIMBRE_PLAN.md#measured-2026-09-13) used for the
original 13 dB finding. Section 2 renders 0.5 s fired at 6000 rpm WOT against
0.5 s of spark-cut at the same speed:

| | fired RMS | cut RMS | contrast |
|---|---:|---:|---:|
| before (trigger gain `2.0`) | -39.4 dBFS | -21.1 dBFS | **+18.3 dB** |
| after (trigger gain `0.5`) | -39.4 dBFS | -32.6 dBFS | **+6.8 dB** |

The "before" figure is worse than the plan's original 13 dB, not the same
number: [Stage T1](timbre-t1.md) cut `structure_level` from `0.07` to `0.05` in
between, which quietens the fired engine without touching the backfire, so the
same undocumented `* 2.0` reads as a bigger contrast today than it did when it
was first measured. The trigger-site gain is now a named constant,
`BACKFIRE_TRIGGER_GAIN` in `src/audio/dsp.rs`, at `0.5` — a quarter of the old
doubled gain — which lands the reference case at 6.8 dB above the fired
engine: a clear crack over the top, not a firecracker that drowns it out.

## The catalogue does not move uniformly, and this stage does not fix that

The same measurement on four more presets, at `BACKFIRE_TRIGGER_GAIN = 0.5`:

| preset | fired RMS | cut RMS | contrast |
|---|---:|---:|---:|
| Flat-plane V8 | -23.7 dBFS | -23.5 dBFS | -0.2 dB |
| Inline-4 | -35.0 dBFS | -33.8 dBFS | +1.2 dB |
| Turbo Inline-4 | -28.7 dBFS | -29.1 dBFS | -0.4 dB |
| V12 | -32.0 dBFS | -33.8 dBFS | -1.8 dB |

`BACKFIRE_TRIGGER_GAIN` is a single flat multiplier, and the "fired" reference
itself spans -39.4 to -23.7 dBFS across these five presets — a 15+ dB spread at
matched rpm and throttle that has nothing to do with backfires. Levelling one
constant against one preset cannot correct for that spread; it was not this
stage's brief to, and per-preset `backfire_level` already exists as the knob
that should absorb it. Naming it here so it is not mistaken for something this
stage closed: **the catalogue-wide "fired" loudness spread is a separate,
larger finding, and it is not fixed.**

## Tests

New this stage: `backfire_contrast_against_the_fired_engine_lands_a_few_db_up`
in `src/audio/dsp/tests.rs`, which renders the reference case above and
asserts the contrast lands in `2.0..=10.0` dB (not the old 13+) and that the
cut section's crest factor stays above 12 dB, so bandlimiting the pulse in this
stage's earlier commits did not also turn the crack into a thud. Pre-existing
`backfire_pulse_has_no_energy_above_nyquist_bandlimit` and
`backfire_ages_one_sample_apart_are_a_pure_delay`, from this stage's first
three commits, are unchanged by this one.
