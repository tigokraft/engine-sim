# Measurements

The Stage 0 baseline from [the implementation plan](../IMPLEMENTATION_PLAN.md): what every engine in the catalogue sounds like, as numbers, before any of the later stages touch the physics or the synth. Every stage after this one is measured against these tables.

Regenerate the whole directory with:

```bash
cargo run --release --example measure -- --markdown docs/measurements
```

[Calibration](calibration.md) is the other half of this directory: these tables say what the catalogue sounds like, and that one says how far it is from what its own crank and its own plumbing predict.

## What the numbers are

**Order levels.** An engine order is a frequency in cycles per crank revolution, so order `n` sits at `f = n * rpm / 60` and follows the engine up and down the rev range. A four-stroke fires `N_cyl / 2` times a revolution, so an inline-four's firing order is 2, a V8's is 4 and a V12's is 6. Levels are dBFS on an amplitude reference: a full-scale sine is 0 dB. Each figure is the level averaged in power over every analysis frame of that render.

**Resonances.** Peaks in the long-term average spectrum, picked by prominence and interpolated between bins. A resonance is defined by standing still while the orders move past it, so only the two sweeping profiles get a table and only the part of them the engine spent moving is averaged: a held speed leaves its own order lines in the average, at one frequency, looking exactly like plumbing.

That is a filter, not a proof. A sweep this length still cannot smear the lowest orders across more than an analysis bin, so treat a peak as a resonance only if `sweep_up` and `sweep_down` both put one at the same frequency — they dwell at opposite ends of the rev range, so their leftover order lines do not land in the same place.

**Continuity.** Peak, RMS and the largest step between consecutive samples — a pop is a step discontinuity — plus a check that no block of 256 frames came out silent.

The analysis is an 8192-point Hann STFT at 48 kHz: a 5.86 Hz bin, and a 17.6 Hz floor under which an order cannot be told apart from DC. Orders below it are recorded as a dash rather than as a number that would be mostly leakage.

## Engines

| Engine | Spec | Induction | Firing order | Top resonance | Clean |
|---|---|---|---:|---:|---|
| [Inline-4](inline-4.md) | 2.0 L  4 cyl  11.5:1 | naturally aspirated | 2 | 1484 Hz | yes |
| [Cross-plane V8](cross-plane-v8.md) | 5.0 L  8 cyl  11.0:1 | roots supercharged | 4 | 13200 Hz | yes |
| [Flat-plane V8](flat-plane-v8.md) | 4.5 L  8 cyl  12.5:1 | naturally aspirated | 4 | 6480 Hz | yes |
| [V10](v10.md) | 5.2 L  10 cyl  12.7:1 | centrifugal supercharged | 5 | 23280 Hz | yes |
| [V12](v12.md) | 6.5 L  12 cyl  11.8:1 | naturally aspirated | 6 | 6000 Hz | yes |
| [2-Rotor Wankel](2-rotor-wankel.md) | 2.6 L  4 cyl  10.0:1 | naturally aspirated | 2 | 53 Hz | yes |
| [Turbo Inline-4](turbo-inline-4.md) | 2.0 L  4 cyl  9.6:1 | turbocharged | 2 | 13076 Hz | yes |
| [Twin-turbo V8](twin-turbo-v8.md) | 4.0 L  8 cyl  10.0:1 | turbocharged | 4 | 9463 Hz | yes |
| [Turbo Inline-6](turbo-inline-6.md) | 3.0 L  6 cyl  9.2:1 | turbocharged | 3 | 10800 Hz | yes |
| [Turbodiesel I4](turbodiesel-i4.md) | 2.0 L  4 cyl  21.5:1 | turbocharged | 2 | 10046 Hz | yes |
| [Big Single](big-single.md) | 0.7 L  1 cyl  12.0:1 | naturally aspirated | 0.5 | 2035 Hz | yes |

Every engine now breathes through its own plumbing, so this column no longer reads the one muffler they all used to share and no longer lands in a narrow band. It is the most prominent peak in the sweep average and nothing more: a network of primaries, a collector and a silencer chain has modes all the way up, and which of them stands tallest is a property of that engine's pipes. Read the per-engine tables for the peaks in order rather than this one number.

## CPU

Seconds of audio produced per second of wall clock, physics and synth together, and the share of one core the synth alone would need to keep up in real time. Appendix B of the plan holds that share under 5 % for the largest preset.

Unlike everything else here these figures are a property of the machine that ran them, not of the build: compare them within one run, and re-record them on the same machine when comparing across stages. This set was taken on aarch64 macos, release profile, at 48 kHz with the solver at 240 Hz.

| Engine | `idle_hold` | `sweep_up` | `sweep_down` | `tip_in` | `overrun_cut` | `limiter_bounce` | Synth core, worst |
|---|---:|---:|---:|---:|---:|---:|---:|
| Inline-4 | 31x | 24x | 23x | 26x | 23x | 19x | 2.77 % |
| Cross-plane V8 | 19x | 16x | 15x | 17x | 14x | 14x | 5.30 % |
| Flat-plane V8 | 21x | 16x | 16x | 18x | 16x | 14x | 4.38 % |
| V10 | 17x | 14x | 13x | 15x | 14x | 12x | 5.44 % |
| V12 | 18x | 14x | 13x | 15x | 13x | 12x | 5.63 % |
| 2-Rotor Wankel | 31x | 22x | 21x | 25x | 21x | 18x | 2.68 % |
| Turbo Inline-4 | 30x | 23x | 22x | 26x | 23x | 19x | 2.95 % |
| Twin-turbo V8 | 19x | 15x | 15x | 15x | 15x | 13x | 5.34 % |
| Turbo Inline-6 | 26x | 20x | 19x | 22x | 19x | 17x | 3.61 % |
| Turbodiesel I4 | 28x | 23x | 23x | 25x | 22x | 20x | 3.15 % |
| Big Single | 41x | 29x | 27x | 34x | 28x | 24x | 1.79 % |
