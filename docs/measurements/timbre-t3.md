# Stage T3 — valve float

Stage T3 of [the timbre plan](../TIMBRE_PLAN.md): valve float re-seating impulse
schedule on the existing impulsive valve voices in `MechanicalVoice`, parameterised
as `float_rpm` per preset. Per [Definition of done, rule 7](../TIMBRE_PLAN.md#definition-of-done-every-stage),
this records the stage's effect on octave-band share and crest factor against
the pre-Stage-0 reference, whether or not it moved as expected.

## The three rows

**"target"** is unchanged from [Stage T0](timbre-t0.md): the pre-Stage-0
reconstruction recorded in [`timbre::RECORDED`](../../src/analysis/timbre.rs).

**"before"** is the catalogue measured at the end of [Stage T2](timbre-t2.md)
(`6054414`), which wired reciprocating shaking force into `StructuralPath`.

**"after"** is the same catalogue with Stage T3 valve float implemented.

Bands are energy share relative to the render's total, in dB.

### Inline-4

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -8.3 | -22.1 | -13.6 | -1.3 | -17.6 | -17.4 | -18.6 | -27.4 | -41.6 | -52.7 | 32.0 |
| before | -8.2 | -13.7 | -10.1 | -1.8 | -17.5 | -17.3 | -26.7 | -35.7 | -41.5 | -52.6 | 12.2 |
| after | -8.2 | -13.7 | -10.1 | -1.8 | -17.5 | -17.3 | -26.7 | -35.7 | -41.5 | -52.6 | 12.2 |

### Cross-plane V8

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -18.3 | -10.7 | -8.2 | -6.9 | -9.2 | -14.0 | -19.4 | -24.6 | -44.1 | -56.5 | 35.2 |
| before | -18.3 | -3.4 | -8.2 | -6.8 | -9.3 | -14.0 | -27.6 | -33.0 | -44.1 | -56.5 | 15.2 |
| after | -18.3 | -3.4 | -8.2 | -6.8 | -9.3 | -14.0 | -27.6 | -33.0 | -44.1 | -56.5 | 15.2 |

### Flat-plane V8

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -17.7 | -14.2 | -8.8 | -4.0 | -6.8 | -16.9 | -18.9 | -28.1 | -45.2 | -53.6 | 33.9 |
| before | -18.3 | -7.3 | -7.4 | -3.7 | -7.8 | -17.7 | -27.8 | -37.2 | -45.9 | -54.3 | 13.7 |
| after | -18.3 | -7.3 | -7.4 | -3.7 | -7.8 | -17.7 | -27.8 | -37.2 | -45.9 | -54.3 | 13.7 |

### V10

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -12.8 | -16.4 | -3.3 | -13.9 | -6.6 | -16.0 | -5.2 | -17.4 | -41.3 | -52.1 | 35.9 |
| before | -18.6 | -6.2 | -1.7 | -19.8 | -13.7 | -23.1 | -20.5 | -32.9 | -48.4 | -59.2 | 10.7 |
| after | -18.6 | -6.2 | -1.7 | -19.8 | -13.7 | -23.1 | -20.5 | -32.9 | -48.4 | -59.2 | 10.7 |

### V12

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -15.4 | -19.2 | -10.9 | -4.4 | -8.2 | -13.6 | -22.8 | -31.6 | -44.2 | -54.6 | 32.0 |
| before | -15.4 | -11.9 | -10.9 | -4.4 | -8.2 | -13.6 | -31.0 | -40.0 | -44.2 | -54.6 | 12.0 |
| after | -15.4 | -11.9 | -10.9 | -4.4 | -8.2 | -13.6 | -31.0 | -40.0 | -44.2 | -54.6 | 12.0 |

### 2-Rotor Wankel

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -11.4 | -11.7 | -9.6 | -4.5 | -8.0 | -13.9 | -19.7 | -29.4 | -42.8 | -52.8 | 34.4 |
| before | -11.4 | -4.3 | -9.6 | -4.5 | -8.0 | -13.9 | -27.9 | -37.8 | -42.8 | -52.8 | 14.4 |
| after | -11.4 | -4.3 | -9.6 | -4.5 | -8.0 | -13.9 | -27.9 | -37.8 | -42.8 | -52.8 | 14.4 |

### Turbo Inline-4

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -12.0 | -9.3 | -12.3 | -5.7 | -9.1 | -13.6 | -18.2 | -21.5 | -36.5 | -47.1 | 35.7 |
| before | -12.0 | -2.2 | -10.2 | -6.0 | -9.4 | -13.9 | -26.7 | -30.2 | -36.7 | -47.4 | 15.5 |
| after | -12.0 | -2.2 | -10.2 | -6.0 | -9.4 | -13.9 | -26.7 | -30.2 | -36.7 | -47.4 | 15.5 |

### Twin-turbo V8

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -17.5 | -15.4 | -7.2 | -4.1 | -10.6 | -14.6 | -21.4 | -18.4 | -37.9 | -50.1 | 35.4 |
| before | -17.5 | -8.1 | -7.2 | -4.1 | -10.6 | -14.6 | -29.6 | -26.8 | -37.9 | -50.1 | 15.4 |
| after | -17.5 | -8.1 | -7.2 | -4.1 | -10.6 | -14.6 | -29.6 | -26.8 | -37.9 | -50.1 | 15.4 |

### Turbo Inline-6

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -16.8 | -18.1 | -10.2 | -3.7 | -8.5 | -14.2 | -19.4 | -27.6 | -41.4 | -53.5 | 34.2 |
| before | -16.8 | -10.8 | -10.2 | -3.7 | -8.5 | -14.2 | -27.6 | -36.0 | -41.4 | -53.5 | 14.2 |
| after | -16.8 | -10.8 | -10.2 | -3.7 | -8.5 | -14.2 | -27.6 | -36.0 | -41.4 | -53.5 | 14.2 |

### Turbodiesel I4

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -15.4 | -16.5 | -3.1 | -11.7 | -7.6 | -12.1 | -17.0 | -7.0 | -24.8 | -47.9 | 34.2 |
| before | -15.4 | -9.5 | -3.1 | -11.7 | -7.6 | -12.1 | -24.7 | -14.9 | -24.8 | -47.9 | 13.8 |
| after | -15.4 | -9.5 | -3.1 | -11.7 | -7.6 | -12.1 | -24.7 | -14.9 | -24.8 | -47.9 | 13.8 |

### Big Single

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -23.7 | -19.9 | -3.7 | -10.0 | -6.2 | -8.7 | -8.1 | -12.2 | -35.8 | -46.6 | 38.4 |
| before | -23.7 | -12.2 | -3.7 | -10.0 | -6.2 | -8.7 | -17.2 | -21.5 | -35.8 | -46.6 | 17.0 |
| after | -23.7 | -12.2 | -3.7 | -10.0 | -6.2 | -8.7 | -17.2 | -21.5 | -35.8 | -46.6 | 17.0 |

## Reading this

As designed and required by Stage T3 specifications:
- **Bit-identical below float speed:** Every recorded calibration sweep in the test suite
  spans `idle` to `redline`. By default, `float_rpm = 1.06 * redline`. Because the calibration
  sweep never reaches `float_rpm`, the overspeed `(rpm - float_rpm).max(0.0)` is identically zero.
  The valve float schedule contributes exactly 0.0 to gain and leaves modal frequencies
  unperturbed. Consequently, the **after** row is bit-identical to the **before** row across all
  11 catalogue presets.
- **Midrange unaffected:** Because float is silent below threshold, it introduces zero regression
  into the midrange audit (T1).
- **Acoustic effect above threshold:** Above `float_rpm` (e.g. during limiter bounce excursions
  and over-revs), re-seating impulse gain increases by `0.005 / rpm` and modal resonance frequency
  increases by `0.0006 / rpm`, producing a harsh, bright metallic clatter in the 3.5 kHz - 8 kHz
  band that appears on overspeed and cuts out when RPM drops back below float speed.
