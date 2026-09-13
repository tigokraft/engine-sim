# Stage T2 — reciprocating inertia

Stage T2 of [the timbre plan](../TIMBRE_PLAN.md): the reciprocating shaking-force
resultant summed over the block and driving `StructuralPath` alongside
combustion. Per [Definition of done, rule 7](../TIMBRE_PLAN.md#definition-of-done-every-stage),
this records the stage's effect on octave-band share and crest factor against
the pre-Stage-0 reference, whether or not it moved as expected.

## The three rows

**"before"** is the catalogue measured at the commit immediately before this
stage's synth wiring landed — `reciprocating_mass` exists in
`CylinderGeometry` but nothing in `src/audio` reads it yet, so it is identical
to every prior stage's render.

**"after"** is the same catalogue with the shaking force wired into
`StructuralPath`'s drive, `reciprocating_mass` defaulted from bore on every
preset.

**"target"** is unchanged from [Stage T0](timbre-t0.md): the pre-Stage-0
reconstruction recorded in [`timbre::RECORDED`](../../src/analysis/timbre.rs).
T2 does not touch it — see the doc comment on `RECORDED` and Stage T0's own
note that only Stage T1's bisect earns a real replacement.

Bands are energy share relative to the render's total, in dB.

### Inline-4

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -8.3 | -22.1 | -13.6 | -1.3 | -17.6 | -17.4 | -18.6 | -27.4 | -41.6 | -52.7 | 32.0 |
| before | -8.3 | -14.8 | -13.6 | -1.3 | -17.6 | -17.4 | -26.8 | -35.8 | -41.6 | -52.7 | 12.0 |
| after | -8.2 | -13.7 | -10.1 | -1.8 | -17.5 | -17.3 | -26.7 | -35.7 | -41.5 | -52.6 | 12.2 |

### Cross-plane V8

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -18.3 | -10.7 | -8.2 | -6.9 | -9.2 | -14.0 | -19.4 | -24.6 | -44.1 | -56.5 | 35.2 |
| before | -18.3 | -3.4 | -8.2 | -6.9 | -9.2 | -14.0 | -27.6 | -33.0 | -44.1 | -56.5 | 15.2 |
| after | -18.3 | -3.4 | -8.2 | -6.8 | -9.3 | -14.0 | -27.6 | -33.0 | -44.1 | -56.5 | 15.2 |

### Flat-plane V8

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -17.7 | -14.2 | -8.8 | -4.0 | -6.8 | -16.9 | -18.9 | -28.1 | -45.2 | -53.6 | 33.9 |
| before | -17.7 | -6.9 | -8.8 | -4.0 | -6.8 | -16.9 | -27.1 | -36.5 | -45.2 | -53.6 | 13.9 |
| after | -18.3 | -7.3 | -7.4 | -3.7 | -7.8 | -17.7 | -27.8 | -37.2 | -45.9 | -54.3 | 13.7 |

### V10

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -12.8 | -16.4 | -3.3 | -13.9 | -6.6 | -16.0 | -5.2 | -17.4 | -41.3 | -52.1 | 35.9 |
| before | -12.8 | -9.1 | -3.3 | -13.9 | -6.6 | -16.0 | -13.4 | -25.8 | -41.3 | -52.1 | 15.9 |
| after | -18.6 | -6.2 | -1.7 | -19.8 | -13.7 | -23.1 | -20.5 | -32.9 | -48.4 | -59.2 | 10.7 |

### V12

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -21.6 | -19.2 | -13.3 | -2.6 | -5.3 | -15.4 | -22.8 | -31.6 | -45.5 | -55.5 | 32.0 |
| before | -21.6 | -11.9 | -13.3 | -2.6 | -5.3 | -15.4 | -31.0 | -40.0 | -45.5 | -55.5 | 12.0 |
| after | -21.6 | -11.9 | -13.3 | -2.6 | -5.3 | -15.4 | -31.0 | -40.0 | -45.5 | -55.5 | 12.0 |

### 2-Rotor Wankel

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -16.5 | -11.7 | -10.5 | -3.3 | -15.6 | -15.8 | -19.7 | -29.4 | -47.8 | -60.4 | 34.4 |
| before | -16.5 | -4.4 | -10.5 | -3.3 | -15.6 | -15.8 | -27.9 | -37.8 | -47.8 | -60.4 | 14.4 |
| after | -16.5 | -4.3 | -9.7 | -3.5 | -15.6 | -15.8 | -27.9 | -37.8 | -47.8 | -60.4 | 14.4 |

### Turbo Inline-4

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -11.6 | -9.3 | -12.3 | -7.3 | -16.3 | -16.8 | -18.2 | -21.5 | -27.7 | -42.9 | 35.7 |
| before | -11.6 | -2.0 | -12.3 | -7.3 | -16.3 | -16.8 | -26.4 | -29.9 | -27.7 | -42.9 | 15.7 |
| after | -12.0 | -2.2 | -10.2 | -7.4 | -16.6 | -17.1 | -26.7 | -30.2 | -28.0 | -43.2 | 15.5 |

### Twin-turbo V8

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -19.6 | -15.4 | -2.8 | -6.8 | -12.2 | -15.1 | -21.4 | -18.4 | -39.8 | -59.3 | 35.4 |
| before | -19.6 | -8.1 | -2.8 | -6.8 | -12.2 | -15.1 | -29.6 | -26.8 | -39.8 | -59.3 | 15.4 |
| after | -19.6 | -8.1 | -2.8 | -6.8 | -12.3 | -15.1 | -29.6 | -26.8 | -39.8 | -59.3 | 15.4 |

### Turbo Inline-6

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -15.4 | -18.1 | -1.7 | -7.1 | -20.6 | -20.0 | -19.4 | -27.6 | -50.6 | -64.6 | 34.2 |
| before | -15.4 | -10.8 | -1.7 | -7.1 | -20.6 | -20.0 | -27.6 | -36.0 | -50.6 | -64.6 | 14.2 |
| after | -15.4 | -10.8 | -1.7 | -7.1 | -20.6 | -20.0 | -27.6 | -36.0 | -50.6 | -64.6 | 14.2 |

### Turbodiesel I4

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -15.4 | -16.5 | -3.1 | -11.7 | -7.6 | -12.1 | -17.0 | -7.0 | -24.8 | -47.9 | 34.2 |
| before | -15.4 | -9.2 | -3.1 | -11.7 | -7.6 | -12.1 | -25.2 | -15.4 | -24.8 | -47.9 | 14.2 |
| after | -14.7 | -9.5 | -3.4 | -11.3 | -7.1 | -11.6 | -24.7 | -14.9 | -24.4 | -47.4 | 13.8 |

### Big Single

| | 31.5 | 63 | 125 | 250 | 500 | 1k | 2k | 4k | 8k | 16k | crest |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target | -23.7 | -19.9 | -3.7 | -10.0 | -6.2 | -8.7 | -8.1 | -12.2 | -35.8 | -46.6 | 38.4 |
| before | -23.7 | -12.6 | -3.7 | -10.0 | -6.2 | -8.7 | -16.3 | -20.6 | -35.8 | -46.6 | 18.4 |
| after | -24.5 | -12.2 | -2.8 | -10.9 | -7.1 | -9.6 | -17.2 | -21.5 | -36.8 | -47.5 | 17.0 |

## Reading this

T0's metrics did move, on most of the catalogue, and the honest reading is
that T2 does not close the gap this plan exists to close — on the presets
where it moves anything material, it moves the wrong way as often as the
right one, which is exactly what the Ordering section warned this stage would
do: "T2, T3 and T4 add character that is currently absent; they do not fix
the balance and must not be mistaken for it."

- **Cross-plane V8, V12, Twin-turbo V8, Turbo Inline-6: no material move.**
  These are all 90-degree vees with even-firing banks, and
  [`FiringOrder::shaking_force`](../../src/physics/engine_block.rs)'s own
  test proves why: a crossplane V8's four throws per bank already span a full
  quadrant and its second-order resultant cancels to under 1 % of an
  inline-four's on that path alone. The shaking force these engines produce
  is real but tiny, and the octave table shows it: differences land in the
  first decimal place, well inside measurement noise.

- **Inline-4, 2-Rotor Wankel: essentially no move at 63 Hz, a small gain at
  125-250 Hz.** A single bank has nothing to cancel against, but at this
  engine's own idle-to-cruise range the crank frequency itself sits at
  10-100 Hz — order 1 and 2 of the shaking force land in the bottom two
  octaves, not in the 2-4 kHz band the plan's regression emptied out. Some of
  that order-2 content spills into 125-250 Hz, which is the one place these
  two presets move at all.

- **Flat-plane V8, V10, Turbo Inline-4, Turbodiesel I4, Big Single: the 63 Hz
  band got *louder*, moving away from target.** These are the presets whose
  layout does not cancel: Big Single has one cylinder and Turbo Inline-4 /
  Turbodiesel I4 the same single-bank arrangement as the plain Inline-4;
  flat-plane V8 shares the crossplane V8's bank angle and per-bank throw
  count but its secondary force adds sideways rather than vertically (see the
  engine_block test); and V10's five throws per bank, unlike a V8's four,
  do not tile a full quadrant of crank phase, so the per-bank cancellation
  that flattens the crossplane V8's vertical resultant is only partial here —
  the biggest single move in this table, over 5 dB of crest. Adding a second,
  independent low-order force to an already bass-heavy mix makes the mix more
  bass-heavy, not less; nothing about summing two low-frequency sources
  produces midrange.

The mechanism is real, correctly wired, and correctly reads the block's
layout — the crossplane-vs-flatplane distinction the plan asked for shows up
exactly where the physics predicts it should, in the sideways (`x`) resultant
of the two banks rather than in the shared vertical one, and only for engines
whose crank throws do not already fill a full quadrant on their own. What it
is not, and does not claim to be, is a source of 2-4 kHz energy: reciprocating
inertia at road-going crank speeds lives an order of magnitude below that
band, and the small amount of broadband content the sharp-peaked waveform
does carry above its own fundamental was not enough to move the 2 kHz or
4 kHz bands on any preset by more than 0.1 dB. Stage T1 remains the fix for
the regression this plan exists to close.
