# Stage T4 — exhaust tuning surface

Stage T4 of [the timbre plan](../TIMBRE_PLAN.md): a `--sweep` and `--mode` report on
`examples/acoustic_bench.rs`, printing the analytic prediction beside the measured
resonance for one exhaust geometry parameter at a time. Per
[Definition of done, rule 7](../TIMBRE_PLAN.md#definition-of-done-every-stage), this
records the stage's effect on octave-band share and crest factor against the
pre-Stage-0 reference, whether or not it moved.

## It did not move

T4 adds no physics — `ExhaustNetwork` is unchanged; the work is a diagnostic tool and
four unit tests. `cargo run --release --example calibrate -- --fingerprints`, run at
the commit immediately before this stage (`3ca14dd`) and again at the end of it,
produces byte-identical output for every one of the eleven catalogue presets: same
order balance, same resonances, same octave share, same crest factor. There is
nothing to tabulate that [Stage T3](timbre-t3.md) does not already show, because
nothing rendered differently.

## A pre-existing inconsistency, found in passing

Checking that "no change" claim turned up something unrelated to T4 worth recording
rather than quietly stepping around: [Stage T3](timbre-t3.md)'s own "after" row does
not reproduce from the codebase at the commit that recorded it. Checking out `3ca14dd`
in a worktree and running `--fingerprints` there gives, for **V12**, an `octave_share`
of `-21.6, -11.9, -13.3, -2.6, -5.3, -15.4, -31.0, -40.0, -45.5, -55.5` — not the
`-15.4, -11.9, -10.9, -4.4, -8.2, -13.6, -31.0, -40.0, -44.2, -54.6` the T3 document
records as both "before" and "after". The same is true of **2-Rotor Wankel**, **Turbo
Inline-4**, **Twin-turbo V8**, **Turbo Inline-6**, **Turbodiesel I4** and **Big
Single** — seven of the eleven presets, all of them the ones with forced induction, a
rotary firing pattern, or (Big Single) the shortest render in the catalogue.
**Inline-4**, **Cross-plane V8**, **Flat-plane V8** and **V10** reproduce exactly.

This is not a T4 regression — the mismatch is present at `3ca14dd` itself, before any
commit in this stage — and it is not something T4's brief covers fixing. It is left
here because it means `no_preset_has_drifted_from_its_recorded_timbre`'s baseline (and
by extension every "before" row in every stage's measurement doc back through T3) is
suspect for exactly those seven presets, and the next stage that touches any of them
should re-verify from a live render rather than trust the recorded table.
