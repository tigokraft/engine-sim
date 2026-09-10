# Measurements

The Stage 0 baseline from [the implementation plan](../IMPLEMENTATION_PLAN.md): what every engine in the catalogue sounds like, as numbers, before any of the later stages touch the physics or the synth. Every stage after this one is measured against these tables.

Regenerate the whole directory with:

```bash
cargo run --release --example measure -- --markdown docs/measurements
```

## What the numbers are

**Order levels.** An engine order is a frequency in cycles per crank revolution, so order `n` sits at `f = n * rpm / 60` and follows the engine up and down the rev range. A four-stroke fires `N_cyl / 2` times a revolution, so an inline-four's firing order is 2, a V8's is 4 and a V12's is 6. Levels are dBFS on an amplitude reference: a full-scale sine is 0 dB. Each figure is the level averaged in power over every analysis frame of that render.

**Resonances.** Peaks in the long-term average spectrum, picked by prominence and interpolated between bins. A resonance is defined by standing still while the orders move past it, so only the two sweeping profiles get a table and only the part of them the engine spent moving is averaged: a held speed leaves its own order lines in the average, at one frequency, looking exactly like plumbing.

That is a filter, not a proof. A sweep this length still cannot smear the lowest orders across more than an analysis bin, so treat a peak as a resonance only if `sweep_up` and `sweep_down` both put one at the same frequency — they dwell at opposite ends of the rev range, so their leftover order lines do not land in the same place.

**Continuity.** Peak, RMS and the largest step between consecutive samples — a pop is a step discontinuity — plus a check that no block of 256 frames came out silent.

The analysis is an 8192-point Hann STFT at 48 kHz: a 5.86 Hz bin, and a 17.6 Hz floor under which an order cannot be told apart from DC. Orders below it are recorded as a dash rather than as a number that would be mostly leakage.

## Engines

| Engine | Spec | Induction | Firing order | Top resonance | Clean |
|---|---|---|---:|---:|---|
| [Inline-4](inline-4.md) | 2.0 L  4 cyl  11.5:1 | naturally aspirated | 2 | 195 Hz | yes |
| [Cross-plane V8](cross-plane-v8.md) | 5.0 L  8 cyl  11.0:1 | naturally aspirated | 4 | 170 Hz | yes |
| [Flat-plane V8](flat-plane-v8.md) | 4.5 L  8 cyl  12.5:1 | naturally aspirated | 4 | 147 Hz | yes |
| [V10](v10.md) | 5.2 L  10 cyl  12.7:1 | naturally aspirated | 5 | 148 Hz | yes |
| [V12](v12.md) | 6.5 L  12 cyl  11.8:1 | naturally aspirated | 6 | 160 Hz | yes |
| [2-Rotor Wankel](2-rotor-wankel.md) | 2.6 L  4 cyl  10.0:1 | naturally aspirated | 2 | 200 Hz | yes |
| [Turbo Inline-4](turbo-inline-4.md) | 2.0 L  4 cyl  9.6:1 | turbocharged | 2 | 191 Hz | yes |
| [Twin-turbo V8](twin-turbo-v8.md) | 4.0 L  8 cyl  10.0:1 | turbocharged | 4 | 170 Hz | yes |
| [Turbo Inline-6](turbo-inline-6.md) | 3.0 L  6 cyl  9.2:1 | turbocharged | 3 | 180 Hz | yes |

Every engine's strongest resonance lands in the same narrow band, because every engine is currently breathing through the same muffler: `SynthConfig::uniform` gives all nine eight litres of chamber behind a 50 mm neck, and `f = (c / 2 pi) sqrt(A_neck / (V L_neck))` puts that at `0.249 c`. The only thing separating one engine from another here is the temperature of its own exhaust, through `c = sqrt(gamma R T)`. Stage 3 is where they stop sharing plumbing, and this column is the first number it has to pull apart.

## CPU

Seconds of audio produced per second of wall clock, physics and synth together, and the share of one core the synth alone would need to keep up in real time. Appendix B of the plan holds that share under 5 % for the largest preset.

Unlike everything else here these figures are a property of the machine that ran them, not of the build: compare them within one run, and re-record them on the same machine when comparing across stages. This set was taken on aarch64 macos, release profile, at 48 kHz with the solver at 240 Hz.

| Engine | `idle_hold` | `sweep_up` | `sweep_down` | `tip_in` | `overrun_cut` | `limiter_bounce` | Synth core, worst |
|---|---:|---:|---:|---:|---:|---:|---:|
| Inline-4 | 237x | 79x | 70x | 111x | 70x | 49x | 0.13 % |
| Cross-plane V8 | 235x | 81x | 71x | 114x | 72x | 51x | 0.17 % |
| Flat-plane V8 | 215x | 69x | 60x | 98x | 60x | 43x | 0.16 % |
| V10 | 208x | 69x | 61x | 97x | 61x | 43x | 0.18 % |
| V12 | 217x | 71x | 61x | 97x | 60x | 43x | 0.19 % |
| 2-Rotor Wankel | 220x | 67x | 59x | 96x | 58x | 41x | 0.12 % |
| Turbo Inline-4 | 197x | 77x | 68x | 105x | 68x | 49x | 0.24 % |
| Twin-turbo V8 | 188x | 72x | 63x | 98x | 64x | 46x | 0.31 % |
| Turbo Inline-6 | 195x | 74x | 66x | 101x | 66x | 47x | 0.24 % |
