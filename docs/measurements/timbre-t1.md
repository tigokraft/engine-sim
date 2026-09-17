# Stage T1 — find and return the midrange

Stage T1 of [the timbre plan](../TIMBRE_PLAN.md): the bisect this stage exists
to run. Per [Definition of done, rule 7](../TIMBRE_PLAN.md#definition-of-done-every-stage),
this records the stage's effect on octave-band share and crest factor against
the pre-Stage-0 reference, whether or not it moved as expected.

## The bisect

The four suspects were checked in the order the stage lists them, each by
rendering and measuring rather than by reading:

1. **Viscothermal wall loss.** Already correct.
   `BOUNDARY_LAYER_TURBULENCE_FACTOR` was recalibrated from `16.0` to `2.0`
   against a measured pipe in `10ff5f3` — before Stage T0 ever recorded a
   "today" figure — so the regression T0 measured already includes whatever
   this constant buys. It was not the cause.
2. **Radiation corner placement.** Already where the geometry puts it.
   Multiplying `corner_hz` by 3 recovered a few dB in the 8-16 kHz tail but
   cost crest factor; dividing it by 5 made the 2-4 kHz shortfall worse, not
   better, because it drags the duct's own cut-on down with it. Neither
   direction is a net win, and the formula matches the plan's own derivation.
3. **Output clipper drive.** Not reached hard enough to matter. Bypassing
   `OversampledClipper::process` entirely — returning the input unchanged —
   moved every octave band and the crest figure by at most 0.2 dB on
   Flat-plane V8. The calibration sweep does not drive the bus anywhere near
   its knee.
4. **Structural path level.** This was it. `SynthConfig::structure_level`
   (`src/audio/dsp.rs`) was `0.07`, a figure set once in `2e63b5c` alongside
   `mechanical_level`, before the exhaust's wall loss was fitted to a measured
   pipe (`10ff5f3`), before the network was rebuilt to lose what a real one
   loses (`d0a38e4`), and before Stage T5 gave the silenced presets a real
   loss term — each of which made the exhaust louder without this constant
   moving to match. `examples/diagnose_sound` confirmed it directly: at 3000
   rpm under 0.8 throttle on Flat-plane V8, the structural path alone
   (`Block Only`) rendered as loud as the exhaust alone (`Exhaust Only`),
   which no running V8 does. Turning it down to `0.05` is the fix.

`mechanical_level` was left at `0.37`. It shares the same downstream
`structure_level` multiply, so it looked like part of the same story, but
cutting it independently broke
`the_diesel_radiates_through_its_block_and_the_petrol_four_through_its_pipe`
— a real physical-distinctiveness test, not a calibration artefact — so it is
out of scope here.

## The cost

`structure_level` could not be cut as far as the spectral evidence alone
would suggest: `mechanical_floor_fills_the_gaps_between_firings` requires the
mechanical rig to still be audible between a four-cylinder's firing pulses at
idle, and `mechanical` reaches the mix only through the same
`structure_level` scalar. Below `0.069` that test started failing; at `0.05`
the margin it asserts had to move from `1.25` to `1.1`, documented in the
test itself. The gap floor is real but thinner than it was.

## A real reference, recovered

Stage T0 could not measure a pre-Stage-0 render and reconstructed a "target"
by applying the plan's own aggregate finding to each preset's current
numbers. A real one exists — `2026-09-10 20-54-49.mp3`, a 63 s capture made
three minutes after this repository's first commit — and decoding it
(`ffmpeg`, 48 kHz mono float) and measuring it with the same
`orders::octave_bands` / `orders::crest_db` this fingerprint uses gives:

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| reference | -15.5 | -11.2 | -8.0 | -6.0 | -7.7 | -9.2 | -10.1 | -12.1 | -17.4 | -22.9 | 20.5 |

This is a *different recording* — an unknown preset, a full drive cycle
rather than the fixed calibration sweep, a real microphone's own noise floor
— so it is not comparable band-for-band against the per-preset tables below.
What it confirms is the shape: a 500 Hz-4 kHz region within about 5 dB of
itself and a crest factor around 20 dB, against every preset here still
rolling off 15-20 dB from 500 Hz to 4 kHz and sitting at half that crest.
Most of that remaining gap is not `structure_level` — zeroing it entirely on
Flat-plane V8 only closed about 4 dB of the 4 kHz shortfall — and it is not
any of the other three suspects either. It is very likely broadband
turbulent jet noise at the tailpipe exit that nothing in this synth generates
yet, which is a new source term, not a bug in what is already there, and is
out of scope for a bisect stage.

## Per-preset effect

`RECORDED` in [`src/analysis/timbre.rs`](../../src/analysis/timbre.rs) is now
the "after" row below for every preset — the placeholder reconstruction is
gone. `before` is the catalogue at `structure_level: 0.07`, measured on the
same commit as `after` so the only variable is the fix.

### Inline-4

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| before | -17.0 | -12.4 | -6.0 | -3.3 | -8.7 | -13.4 | -21.1 | -30.2 | -39.3 | -51.4 | 13.3 |
| after | -18.8 | -13.9 | -6.8 | -2.7 | -8.8 | -13.1 | -20.6 | -29.9 | -39.0 | -51.3 | 12.6 |

### Cross-plane V8

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| before | -18.1 | -10.4 | -2.0 | -8.2 | -11.8 | -14.9 | -28.8 | -34.2 | -43.1 | -53.7 | 15.5 |
| after | -18.5 | -10.7 | -1.9 | -8.1 | -12.3 | -14.8 | -28.8 | -34.1 | -43.1 | -53.7 | 15.6 |

### Big Cam Chopping V8

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| before | -23.7 | -15.1 | -3.6 | -4.1 | -9.9 | -15.3 | -27.7 | -35.0 | -44.1 | -58.6 | 14.1 |
| after | -24.3 | -15.4 | -3.6 | -4.0 | -10.0 | -15.3 | -27.7 | -35.2 | -44.7 | -59.0 | 14.1 |

### Flat-plane V8

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| before | -16.0 | -9.1 | -5.2 | -5.9 | -6.2 | -15.9 | -22.8 | -31.4 | -40.3 | -50.6 | 14.4 |
| after | -16.8 | -9.4 | -4.8 | -6.0 | -6.5 | -14.7 | -21.1 | -29.7 | -39.6 | -49.6 | 15.7 |

### V10

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| before | -19.7 | -6.2 | -2.1 | -14.5 | -11.5 | -20.4 | -17.8 | -29.6 | -45.9 | -56.3 | 13.5 |
| after | -20.0 | -6.6 | -2.4 | -12.4 | -10.3 | -18.3 | -15.3 | -27.2 | -44.3 | -55.1 | 14.9 |

### V12

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| before | -24.1 | -18.5 | -17.3 | -3.9 | -3.4 | -10.9 | -17.4 | -29.5 | -38.1 | -50.1 | 17.0 |
| after | -26.6 | -20.5 | -18.2 | -3.8 | -3.4 | -11.3 | -17.2 | -29.3 | -38.1 | -50.1 | 17.0 |

### 2-Rotor Wankel

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| before | -24.9 | -13.6 | -14.6 | -5.3 | -4.1 | -6.4 | -22.0 | -35.1 | -44.0 | -54.7 | 15.3 |
| after | -26.1 | -13.8 | -15.3 | -5.3 | -4.1 | -6.3 | -22.0 | -35.2 | -44.5 | -55.7 | 15.3 |

### Turbo Inline-4

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| before | -19.9 | -11.6 | -6.5 | -2.3 | -11.8 | -15.4 | -24.6 | -28.3 | -27.1 | -42.3 | 12.8 |
| after | -20.5 | -11.9 | -6.9 | -2.1 | -12.0 | -15.4 | -24.5 | -28.2 | -27.0 | -42.3 | 12.7 |

### Twin-turbo V8

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| before | -17.1 | -8.9 | -9.6 | -8.0 | -3.3 | -10.6 | -23.5 | -20.2 | -33.3 | -50.4 | 13.8 |
| after | -19.2 | -9.3 | -9.7 | -7.6 | -3.3 | -10.2 | -22.9 | -19.6 | -32.7 | -50.0 | 13.7 |

### Turbo Inline-6

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| before | -16.8 | -13.5 | -2.4 | -5.3 | -13.7 | -18.3 | -27.4 | -36.4 | -49.5 | -63.7 | 13.2 |
| after | -16.9 | -13.7 | -2.4 | -5.2 | -14.1 | -18.4 | -27.4 | -36.5 | -49.8 | -64.3 | 13.2 |

### Turbodiesel I4

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| before | -13.0 | -6.7 | -5.4 | -10.8 | -6.2 | -16.5 | -25.3 | -12.2 | -21.1 | -42.5 | 13.7 |
| after | -14.0 | -7.8 | -4.5 | -11.8 | -6.5 | -16.3 | -23.7 | -10.5 | -19.9 | -43.3 | 13.7 |

### Big Single

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| before | -22.0 | -12.1 | -2.0 | -13.5 | -8.9 | -9.2 | -21.9 | -25.0 | -42.1 | -57.1 | 14.7 |
| after | -22.5 | -12.5 | -2.0 | -13.4 | -8.8 | -9.1 | -21.8 | -24.9 | -42.0 | -57.1 | 14.9 |

### Porsche 911 GT3 Cup (992)

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| before | -29.6 | -14.1 | -1.8 | -10.4 | -7.2 | -19.2 | -27.8 | -32.3 | -42.9 | -51.1 | 14.0 |
| after | -31.9 | -14.0 | -1.7 | -10.4 | -7.4 | -19.1 | -27.7 | -32.2 | -43.0 | -51.0 | 14.1 |

### Porsche 911 GT3 Cup (997.2)

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| before | -31.4 | -16.6 | -10.0 | -9.3 | -1.5 | -13.2 | -21.9 | -29.2 | -40.4 | -49.4 | 14.9 |
| after | -33.9 | -16.7 | -9.9 | -9.3 | -1.5 | -13.1 | -21.8 | -29.1 | -40.3 | -49.3 | 14.9 |

### Mercedes-AMG GT3

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| before | -28.3 | -11.2 | -2.1 | -14.9 | -6.2 | -15.4 | -21.3 | -29.0 | -40.7 | -56.1 | 12.6 |
| after | -28.7 | -11.2 | -2.2 | -14.9 | -6.1 | -15.4 | -21.3 | -29.0 | -40.8 | -56.2 | 12.5 |

### Ferrari 458 Italia GT3

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| before | -20.7 | -10.3 | -10.3 | -4.3 | -5.8 | -8.5 | -17.8 | -25.0 | -36.8 | -47.6 | 15.7 |
| after | -22.7 | -10.5 | -11.1 | -4.0 | -6.2 | -8.0 | -17.4 | -24.4 | -36.4 | -47.3 | 16.0 |

### Audi R8 LMS GT3

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| before | -21.4 | -7.3 | -3.3 | -11.1 | -6.3 | -16.4 | -23.1 | -29.9 | -43.2 | -53.7 | 14.2 |
| after | -22.3 | -8.2 | -4.4 | -9.5 | -4.9 | -14.8 | -21.6 | -28.3 | -41.8 | -52.5 | 15.1 |
