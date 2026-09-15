# Stage T5 — the cutout, the silencers, and the preset that lies

Stage T5 of [the timbre plan](../TIMBRE_PLAN.md): the exhaust cutout defaulted
open on two presets, `into_straight_pipe`/`into_open_headers` opened it again
on conversion, and a lossless `ExpansionChamber` measured louder with a
silencer fitted than without one. Per
[Definition of done, rule 7](../TIMBRE_PLAN.md#definition-of-done-every-stage),
this records the stage's effect on octave-band share and crest factor against
the pre-Stage-0 reference, whether or not it moved as intended.

## It moved, on almost every preset

Unlike [Stage T4](timbre-t4.md), which touched no physics, T5 changes what the
exhaust network actually does: eleven of sixteen catalogue presets carry a
`Silencer::ExpansionChamber` somewhere in their chain, and that element went
from perfectly lossless to genuinely dissipative. Two presets on top of that —
`Cross-plane V8` and `Twin-turbo V8` — had their entire silencer chain in
bypass from frame zero regardless of `--exhaust` mode, and now do not.

### The reproduction, before and after

```
cargo run --release --example acoustic_bench -- --engine cross_plane_v8 --exhaust muffled --out /tmp/ab_muffled.wav
cargo run --release --example acoustic_bench -- --engine cross_plane_v8 --exhaust straight-pipe --out /tmp/ab_straight-pipe.wav
cmp /tmp/ab_muffled.wav /tmp/ab_straight-pipe.wav
```

Before this stage: `IDENTICAL`, byte for byte. After: the files differ from
byte 4267 (well inside the header, i.e. before the audio even starts
diverging on the data itself).

### Octave share and crest factor, before vs. after T5

`cargo run --release --example calibrate -- --fingerprints`, run at the commit
immediately before this stage and again at the end of it. Bands moved less
than 2 dB are omitted; every preset with a chamber moved on at least one.

**Cross-plane V8** — cutout was open from construction; now closed until the
snapshot opens it, and its `ExpansionChamber` (6:1 area ratio, 0.65 m) now
dissipates instead of only reflecting:

| band | before | after | delta |
|---|---:|---:|---:|
| 31.5 Hz | -18.3 dB | -13.5 dB | **+4.8** |
| 63 Hz | -3.4 dB | -6.5 dB | **-3.1** |
| 250 Hz | -6.8 dB | -11.8 dB | **-5.0** |
| 500 Hz | -9.3 dB | -4.0 dB | **+5.3** |
| 1 kHz | -14.0 dB | -11.0 dB | **+3.0** |
| 2 kHz | -27.6 dB | -23.6 dB | **+4.0** |
| 4 kHz | -33.0 dB | -29.0 dB | **+4.0** |
| 16 kHz | -56.5 dB | -53.5 dB | **+3.0** |
| crest | 15.2 dB | 14.2 dB | **-1.0** |

**Twin-turbo V8** — the other permanently-open preset, and its chamber is the
smallest area ratio in the catalogue, so the flow loss at each area step
(Borda-Carnot, scaled by area ratio) bites hardest here:

| band | before | after | delta |
|---|---:|---:|---:|
| 31.5 Hz | -19.6 dB | -15.9 dB | **+3.7** |
| 500 Hz | -12.3 dB | -5.5 dB | **+6.8** |
| 2 kHz | -29.6 dB | -26.8 dB | **+2.8** |
| 4 kHz | -26.8 dB | -19.2 dB | **+7.6** |
| 8 kHz | -39.8 dB | -32.3 dB | **+7.5** |
| 16 kHz | -59.3 dB | -54.2 dB | **+5.1** |
| crest | 15.4 dB | 14.7 dB | **-0.7** |

**Inline-4** — cutout was already closed by default here (`cutout_fitted:
false`); every decibel below is the chamber's new loss term alone, on a
preset nobody's cutout ever touched:

| band | before | after | delta |
|---|---:|---:|---:|
| 31.5 Hz | -8.2 dB | -13.3 dB | **-5.1** |
| 63 Hz | -13.7 dB | -9.3 dB | **+4.4** |
| 125 Hz | -10.1 dB | -4.4 dB | **+5.7** |
| 250 Hz | -1.8 dB | -5.2 dB | **-3.4** |
| 500 Hz | -17.5 dB | -10.3 dB | **+7.2** |
| 1 kHz | -17.3 dB | -12.7 dB | **+4.6** |
| 2 kHz | -26.7 dB | -22.3 dB | **+4.4** |
| 4 kHz | -35.7 dB | -31.5 dB | **+4.2** |
| crest | 12.2 dB | 13.7 dB | **+1.5** |

**Flat-plane V8, V12** move by 1-3 dB on several bands from the chamber loss
alone (V12's cutout was already closed); **V10** barely moves (max 2.1 dB, on
8 kHz) because its chamber's 3.5:1 area ratio is close to its stage count's
natural loss floor. The full sixteen-preset table is in
[`RECORDED`](../../src/analysis/timbre.rs) and
[`calibration.md`](calibration.md), both re-recorded in this stage's commits.

Direction is not uniform and should not be read as one: a band can go up
because the chamber's own resonance is damped and stops absorbing as much of
that band into itself, or down because the same damping removes energy the
network used to recirculate into it. What is consistent is that every preset
with a real silencer moved, and every preset without one (the `Silencer::
Straight`-only GT3 cars, whose bug is a documented, separate fact) did not.

## flat_plane_v8 gains a chamber it never had

`Silencer::Straight` appends nothing to `SilencerElement::extend_chain`, so
this preset was acoustically a straight pipe in "muffled" mode by
construction — no muffled mode existed to compare against, cutout aside. It
now carries the same `ExpansionChamber` geometry as V10 (0.40 m, 3.5:1, two
stages), on a similar-diameter primary. Its octave share moves 1-3 dB on
several bands as a result, tabulated above.

## What did not move, and the pre-existing reproducibility gap it rests on

`no_preset_has_drifted_from_its_recorded_timbre`'s `octave_share` and
`crest_db` columns are not measurements — [`RECORDED`](../../src/analysis/timbre.rs)'s
own doc comment explains they are the pre-Stage-0 *target*, reconstructed by
shifting each preset's current measurement back by the plan's one known
regression (`63 Hz -7.3`, `2 kHz +8.2`, `4 kHz +8.4`, `crest +20.0` dB). Those
four numbers are deliberately still failing on every preset after this stage,
exactly as they were before it: that is Stage T1's deliverable, not this
one's, and re-recording is not the same as fixing it. What this stage's
commits do is re-anchor that same reconstruction to the new, post-T5
measurement underneath it, so the *drift the test reports going forward* is
against a T5-correct baseline rather than one still carrying the bypassed
cutout and the lossless chamber.

Doing that re-anchoring surfaced [Stage T4](timbre-t4.md)'s already-documented
non-reproducibility again: **Inline-4** (125 Hz), **V10** (six bands) and
**Turbo Inline-4** (125 Hz) — three of the same seven presets T4 named as not
reproducing `--fingerprints` output from one build to the next — show a
handful of extra drifted bands beyond the four deliberate ones, up to 7.1 dB
on V10. This is not a T5 regression: reverting this stage's changes and
re-running `--fingerprints` twice on the identical commit reproduces the same
spread T4 measured. It is a harness bug in the build or the render, not in
the exhaust network, and it remains out of this stage's brief to fix.

## Tests

`opening_the_cutout_raises_high_order_content_and_lowers_back_pressure` and
`exhaust_modes_raise_high_order_content_monotonically` (both pre-existing)
still pass unchanged. New this stage, in `src/audio/waveguide.rs` and
`src/bench.rs`:

- `cutout_fitment_alone_does_not_open_the_cutout` — a cutout fitted but never
  opened renders bit-identically to no cutout at all.
- `muffled_and_straight_pipe_renders_differ_for_every_silenced_preset` — the
  reproduction above, generalised to the whole catalogue.
- `expansion_chamber_transmission_loss_is_positive_at_the_transparent_frequency`
  — TL at the half-wave point, where the lossless closed-form theory predicts
  exactly zero, now measures strictly positive.
- `expansion_chamber_radiates_less_energy_than_no_silencer` — "a muffler that
  makes the engine 9.7 dB louder is not a muffler," as a passing assertion.
- `find_by_name_fails_loudly_on_an_unknown_preset` — `diagnose_sound
  --preset <unknown>` now resolves through `EnginePreset::find_by_name`,
  which returns `None` rather than a different engine.

Two pre-existing tests needed re-baselining because the physics they measure
genuinely changed, not because they were wrong: `the_declared_primary_length_
still_reaches_the_sound`'s spectral centroid claim reversed sign (still
monotonic, still real, now driven by the primary's hump rather than by an
undamped chamber resonance that no longer dominates as strongly), and
`the_diesel_radiates_through_its_block_and_the_petrol_four_through_its_pipe`'s
margin narrowed from 4.5x to as little as 2.8x because the petrol four's own
chamber now quietens its pipe path. Both are explained in place, in the
commit that touches them.
