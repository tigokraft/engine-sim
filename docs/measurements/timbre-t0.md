# Stage T0 — broadband metrics

Stage T0 of [the timbre plan](../TIMBRE_PLAN.md): the octave-band energy share
and crest factor added to the timbre regression in
[`src/analysis/timbre.rs`](../../src/analysis/timbre.rs). This stage adds no
audio code — nothing in `src/audio` or `src/physics` changed — so it makes no
sound different. What it changes is that the drift these two symptoms were
always in the catalogue's `calibration_sweep` render, unmeasured, and
`no_preset_has_drifted_from_its_recorded_timbre` could not see either of them.

## The two tables

**"today"** is what `cargo run --release --example calibrate -- --fingerprints`
measured on the commit this stage landed on — the real octave-band share and
crest factor of the current synth.

**"target"** is [`timbre::RECORDED`](../../src/analysis/timbre.rs)'s
`octave_share` and `crest_db`: not a second measurement, but "today" shifted by
the one regression the plan's own evidence describes — 63 Hz down 7.3 dB, 2 kHz
and 4 kHz up 8.2 and 8.4 dB, crest factor up 20 dB, the other six bands left
where they measured. See the doc comment on `RECORDED` for the derivation and
its limits. A real pre-regression measurement, recovered by Stage T1's bisect,
replaces this table; this one is a bound, not a fact.

Bands are energy share relative to the render's total, in dB — a gain change
does not move them, only a stage that moves energy between bands does.

### Inline-4

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| today  | -8.3 | -14.8 | -13.6 | -1.3 | -17.6 | -17.4 | -26.8 | -35.8 | -41.6 | -52.7 | 12.0 |
| target | -8.3 | -22.1 | -13.6 | -1.3 | -17.6 | -17.4 | -18.6 | -27.4 | -41.6 | -52.7 | 32.0 |

### Cross-plane V8

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| today  | -18.3 | -3.4 | -8.2 | -6.9 | -9.2 | -14.0 | -27.6 | -33.0 | -44.1 | -56.5 | 15.2 |
| target | -18.3 | -10.7 | -8.2 | -6.9 | -9.2 | -14.0 | -19.4 | -24.6 | -44.1 | -56.5 | 35.2 |

### Flat-plane V8

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| today  | -17.7 | -6.9 | -8.8 | -4.0 | -6.8 | -16.9 | -27.1 | -36.5 | -45.2 | -53.6 | 13.9 |
| target | -17.7 | -14.2 | -8.8 | -4.0 | -6.8 | -16.9 | -18.9 | -28.1 | -45.2 | -53.6 | 33.9 |

### V10

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| today  | -12.8 | -9.1 | -3.3 | -13.9 | -6.6 | -16.0 | -13.4 | -25.8 | -41.3 | -52.1 | 15.9 |
| target | -12.8 | -16.4 | -3.3 | -13.9 | -6.6 | -16.0 | -5.2 | -17.4 | -41.3 | -52.1 | 35.9 |

### V12

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| today  | -21.6 | -11.9 | -13.3 | -2.6 | -5.3 | -15.4 | -31.0 | -40.0 | -45.5 | -55.5 | 12.0 |
| target | -21.6 | -19.2 | -13.3 | -2.6 | -5.3 | -15.4 | -22.8 | -31.6 | -45.5 | -55.5 | 32.0 |

### 2-Rotor Wankel

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| today  | -16.5 | -4.4 | -10.5 | -3.3 | -15.6 | -15.8 | -27.9 | -37.8 | -47.8 | -60.4 | 14.4 |
| target | -16.5 | -11.7 | -10.5 | -3.3 | -15.6 | -15.8 | -19.7 | -29.4 | -47.8 | -60.4 | 34.4 |

### Turbo Inline-4

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| today  | -11.6 | -2.0 | -12.3 | -7.3 | -16.3 | -16.8 | -26.4 | -29.9 | -27.7 | -42.9 | 15.7 |
| target | -11.6 | -9.3 | -12.3 | -7.3 | -16.3 | -16.8 | -18.2 | -21.5 | -27.7 | -42.9 | 35.7 |

### Twin-turbo V8

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| today  | -19.6 | -8.1 | -2.8 | -6.8 | -12.2 | -15.1 | -29.6 | -26.8 | -39.8 | -59.3 | 15.4 |
| target | -19.6 | -15.4 | -2.8 | -6.8 | -12.2 | -15.1 | -21.4 | -18.4 | -39.8 | -59.3 | 35.4 |

### Turbo Inline-6

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| today  | -15.4 | -10.8 | -1.7 | -7.1 | -20.6 | -20.0 | -27.6 | -36.0 | -50.6 | -64.6 | 14.2 |
| target | -15.4 | -18.1 | -1.7 | -7.1 | -20.6 | -20.0 | -19.4 | -27.6 | -50.6 | -64.6 | 34.2 |

### Turbodiesel I4

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| today  | -15.4 | -9.2 | -3.1 | -11.7 | -7.6 | -12.1 | -25.2 | -15.4 | -24.8 | -47.9 | 14.2 |
| target | -15.4 | -16.5 | -3.1 | -11.7 | -7.6 | -12.1 | -17.0 | -7.0 | -24.8 | -47.9 | 34.2 |

### Big Single

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| today  | -23.7 | -12.6 | -3.7 | -10.0 | -6.2 | -8.7 | -16.3 | -20.6 | -35.8 | -46.6 | 18.4 |
| target | -23.7 | -19.9 | -3.7 | -10.0 | -6.2 | -8.7 | -8.1 | -12.2 | -35.8 | -46.6 | 38.4 |

## Reading this

Every preset shows the same four-line gap, because the correction is the same
four numbers applied everywhere — that is a property of the reconstruction,
not a finding that all eleven engines regressed identically. `cargo test`
reports it as:

```
<preset>: the 63 Hz band moved +7.3 dB: <target> dB to <today> dB
<preset>: the 2000 Hz band moved -8.2 dB: <target> dB to <today> dB
<preset>: the 4000 Hz band moved -8.4 dB: <target> dB to <today> dB
<preset>: crest factor moved -20.0 dB: <target> dB to <today> dB
```

on every preset in the catalogue, which is the deliverable this stage exists
to produce: a `cargo test` that fails on the regression `docs/TIMBRE_PLAN.md`
describes, instead of a spectrum a person has to render and read by hand.

Stage T1 fixes the cause and re-records both this table and `RECORDED` against
what the corrected synth actually measures.
