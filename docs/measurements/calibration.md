# Calibration

Stage 16 of [the implementation plan](../IMPLEMENTATION_PLAN.md): the synth measured against a reference it did not produce. Regenerate with:

```bash
cargo run --release --example calibrate -- --markdown docs/measurements/calibration.md
```

Every figure below is *relative*: an order against the firing order, a measured frequency against a predicted one, a slope against an octave. That is deliberate and it is the point of the stage. A gain constant anywhere in the chain moves every order by the same number of decibels and cancels out of all of it, so nothing here can be closed by turning something up — only by changing a length, a volume or a radius.

| Engine | Firing order | Order balance rms | Worst order | Crank nulls | Modes placed | Floor tilt |
|---|---:|---:|---:|---:|---:|---:|
| [Inline-4](#inline-4) | 2 | 18.7 dB | -26.4 dB on 4 | -32.3 dB on 1 | 5/6 of 11 | -8.6 dB/oct |
| [Cross-plane V8](#cross-plane-v8) | 4 | 7.9 dB | +15.3 dB on 3.5 | +11.2 dB on 1, not enforced | 5/8 of 11 | -9.5 dB/oct |
| [Flat-plane V8](#flat-plane-v8) | 4 | 15.3 dB | -24.3 dB on 2 | -8.4 dB on 0.5, not enforced | 2/4 of 8 | -6.6 dB/oct |
| [V10](#v10) | 5 | 8.9 dB | -16.3 dB on 6 | +3.6 dB on 0.5, not enforced | 5/5 of 10 | -5.6 dB/oct |
| [V12](#v12) | 6 | 14.3 dB | +25.8 dB on 3 | +20.2 dB on 2.5, not enforced | 4/4 of 10 | -11.5 dB/oct |
| [2-Rotor Wankel](#2-rotor-wankel) | 2 | 11.2 dB | -15.8 dB on 4 | -26.0 dB on 2.5 | 4/4 of 9 | -6.7 dB/oct |
| [Turbo Inline-4](#turbo-inline-4) | 2 | 2.5 dB | +3.6 dB on 4 | -3.7 dB on 0.5, not enforced | 3/6 of 11 | -7.9 dB/oct |
| [Twin-turbo V8](#twin-turbo-v8) | 4 | 8.3 dB | -12.9 dB on 7.5 | +4.0 dB on 1, not enforced | 4/8 of 11 | -8.1 dB/oct |
| [Turbo Inline-6](#turbo-inline-6) | 3 | 15.3 dB | -21.7 dB on 6 | -6.3 dB on 2, not enforced | 3/3 of 10 | -8.6 dB/oct |
| [Turbodiesel I4](#turbodiesel-i4) | 2 | 7.8 dB | -11.1 dB on 4 | -7.4 dB on 1, not enforced | 1/5 of 12 | -4.0 dB/oct |
| [Big Single](#big-single) | 0.5 | 12.6 dB | +17.8 dB on 1 | — | 3/4 of 9 | -4.5 dB/oct |

## Method

One unbroken pull from idle to the redline over 8 s at 48 kHz, no hold at either end, analysed with the Stage 0 harness: an 8192-point Hann STFT, orders read against the speed curve the render was driven by, resonances picked off the Welch average of the same audio. A hold at one speed would leave its own order lines standing in that average exactly where a resonance would, which is why the calibration sweep is not one of the six Stage 0 profiles.

**Order balance** is compared against the *crank comb*: the order spectrum of the firing pattern itself, `|sum exp(-i n theta_k)|` over the firing angles of each bank, summed in power across banks. It is a reference this repository can carry — derived from the crank, not recorded, and not tunable. Orders are compared up to 2 x the firing order, above which a firing is no longer usefully a Dirac impulse: blowdown lasts a valve event, a valve event is a fixed number of crank degrees, and its envelope rolls the comb off from somewhere above there.

**Resonance placement** is compared against the analytic modes of the declared geometry, in the gas this render actually had: the block is primed as the render primes it and stepped along the same sweep to its midpoint, because the exhaust warms on a time constant of tens of seconds and a sweep lasts eight. A prediction taken off a block at its thermal plateau would sit a tenth high on every mode for no reason but that. Tolerance 10 %, and each measured peak is given to at most one prediction — the nearest — so a sparse spectrum cannot report one peak as three modes. A peak further than 25 % from a prediction is reported as *not found* rather than as that mode in the wrong place.

**The `implies` column** is the stage's one rule as arithmetic: the length that would put the mode where it was actually measured. A mode 8 % low is not an equaliser's problem, it is a pipe 9 % longer than the one it was credited with.

**No reference recording is committed to this repository.** `*.wav` is gitignored and no recording of a real engine is licensed for redistribution here, so the reference above is the crank and the declared geometry. The recording path is real and is the one this stage was built for — point it at a pull of your own with:

```bash
cargo run --release --example calibrate -- --preset Inline-4 \
    --reference pull.wav --rpm 1200:6800
```

`--rpm` also takes a list of `t=rpm` points read off a tachometer, for a pull that is not linear. Whatever is supplied, record the recording and the curve next to the table it produced: a calibration whose reference cannot be identified is not attributable, and a later regression against it cannot be either.

## Open questions

Recorded rather than closed, as Stage 16 requires: each of these is a disagreement that no length, volume or radius in the declared geometry accounts for, and none of them is closed with a gain.

1. **The blown vees' chamber pass bands land 16 to 20 % high, by the same factor at both harmonics.** One factor on two harmonics is a speed of sound or a length, not a misidentified peak — a misread peak would be wrong by different amounts at different frequencies. Neither the declared chamber length nor the thermal gradient the chain is tuned down accounts for it, and the engines whose chains are the same shape but atmospheric (the V10, the V12) place theirs inside 6 %. Open.

2. **A vee's half-order content depends on how much its two banks cancel, and the comb can only bracket that.** Summed coherently the banks of a cross-plane V8 leave nothing at all on order 1.5; summed in power they leave it 3.7 dB under the firing order. The V12 shows the same question from the other side: its per-bank order 3 comes out well above its engine order 6, so as measured it is two straight sixes rather than one V12, where a real V12's sixth order is its voice. If that is wrong the fix is in the bank paths — lengths, the crossover, where the two tailpipes sit — and not in a level.

3. **The crank comb is a comb of Dirac impulses and a firing is not one.** Every engine here reads its first harmonic above the firing order 10 to 26 dB under what the comb says, and that is the blowdown envelope: a pulse lasting a valve event is a fixed width in crank degrees, so it rolls the comb off at a fixed order. Putting it in the reference needs the fraction of the event that blowdown occupies, and choosing that fraction to fit the measurement is exactly the kind of constant this stage exists to refuse. Left out, and the comparison stops at 2 x the firing order.

4. **A primary's measured mode implies an acoustic length a few per cent off its declared centre line, in both directions.** The model's collector junction is memoryless, so the extra length a primary shows is its collector taper's own delay line; the residual runs from -6 % to +14 % across the catalogue with no consistent sign. No declared primary length is changed on the strength of it: a correction that scatters both ways is scatter, and fitting each preset to its own scatter would be the tone knob in different clothes.

5. **Several predicted modes leave no peak at all.** The turbodiesel is the extreme — it radiates through its block rather than its pipe, so the exhaust chain barely reaches the listener and five of its twelve predicted modes are absent rather than misplaced. Absence is the honest report: a peak found more than 25 % from a prediction is a different mode, not that one in the wrong place.


## Inline-4

> Even 180 deg firing on one bank: a hard, plain four-cylinder bark.

2.0 L  4 cyl  11.5:1  ·  firing order 2  ·  850-7397 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -34.5 | — |
| 1 | null | -32.3 | — |
| 1.5 | null | -51.6 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -35.0 | — |
| 3 | null | -39.2 | — |
| 3.5 | null | -32.7 | — |
| 4 | +0.0 | -26.4 | -26.4 |

Driven orders: rms **18.7 dB**, mean -13.2 dB, worst -26.4 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -32.3 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Implies |
|---|---|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00330 s over 2.12 m` | 75.7 | not found | — |  |
| block, first bending mode | `mass law on 110 kg` | 102.3 | not found | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 622 x 0.951/(4 x (1.200 + 0.017))` | 121.6 | not found | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.2 L` | 162.1 | not found | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00330 s over 2.12 m` | 227.2 | 216.3 | -4.8 % | downstream run 2.12 m → 2.223 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.280 + 0.017))` | 292.0 | 267.7 | -8.3 % | runner length 0.280 m → 0.324 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00330 s over 2.12 m` | 378.7 | not found | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 724 x 0.983/(4 x 0.416)` | 428.2 | 403.0 | -5.9 % | primary length 0.400 m → 0.425 m |
| expansion chamber, first pass band | `nc/2L = 1 x 688/(2 x 0.450)` | 764.0 | 700.5 | -8.3 % | chamber length 0.450 m, volume 9.3 L → 0.491 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 724 x 0.983/(4 x 0.416)` | 1284.7 | 1046.0 | -18.6 % | primary length 0.400 m → 0.491 m |
| expansion chamber, second pass band | `nc/2L = 2 x 688/(2 x 0.450)` | 1528.0 | 1481.7 | -3.0 % | chamber length 0.450 m, volume 9.3 L → 0.464 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 38.8 | -45.5 | 11.8 |
| 216.3 | -22.7 | 38.1 |
| 267.7 | -36.6 | 13.3 |
| 403.0 | -49.0 | 15.8 |
| 700.5 | -53.7 | 12.4 |
| 927.4 | -47.8 | 10.5 |
| 1046.0 | -46.1 | 27.4 |
| 1481.7 | -48.5 | 39.7 |
| 2156.6 | -62.7 | 23.3 |
| 2539.0 | -72.8 | 17.2 |
| 2868.5 | -69.7 | 23.6 |
| 3305.4 | -69.8 | 17.3 |
| 3595.1 | -69.0 | 26.1 |
| 3998.9 | -73.9 | 17.6 |
| 4688.4 | -74.9 | 17.9 |
| 5110.6 | -78.2 | 22.0 |
| 5822.5 | -83.4 | 14.1 |
| 6146.7 | -86.2 | 17.8 |
| 7658.8 | -93.8 | 11.1 |
| 8693.9 | -93.2 | 15.2 |
| 9074.2 | -96.5 | 11.9 |
| 13868.8 | -107.4 | 16.9 |
| 19641.9 | -116.5 | 16.8 |
| 22763.1 | -127.0 | 10.8 |

Noise floor tilt over 200 Hz-12 kHz: **-8.6 dB/octave**.


## Cross-plane V8

> 90-180-270-180 gaps on each bank: the offbeat American burble.

5.0 L  8 cyl  11.0:1  ·  firing order 4  ·  750-6997 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -18.3 | -7.0 |
| 1 | null | +11.2 | — |
| 1.5 | -3.7 | -0.4 | +3.2 |
| 2 | null | -3.5 | — |
| 2.5 | -3.7 | +9.3 | +13.0 |
| 3 | null | +3.3 | — |
| 3.5 | -11.4 | +3.9 | +15.3 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -13.4 | -2.0 |
| 5 | null | -22.5 | — |
| 5.5 | -3.7 | +0.6 | +4.3 |
| 6 | null | +1.8 | — |
| 6.5 | -3.7 | -3.5 | +0.2 |
| 7 | null | -11.5 | — |
| 7.5 | -11.4 | -16.9 | -5.5 |
| 8 | +0.0 | -10.6 | -10.6 |

Driven orders: rms **7.9 dB**, mean +1.1 dB, worst +15.3 dB on order 3.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest +11.2 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Implies |
|---|---|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00445 s over 2.82 m` | 56.1 | not found | — |  |
| block, first bending mode | `mass law on 210 kg` | 74.1 | not found | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 611 x 0.986/(4 x (1.500 + 0.018))` | 99.1 | 94.0 | -5.1 % | tailpipe length 1.500 m, mouth radius 0.030 m → 1.600 m |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 4.8 L` | 140.3 | not found | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00445 s over 2.82 m` | 168.4 | 183.0 | +8.7 % | downstream run 2.82 m → 2.593 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.380 + 0.018))` | 218.0 | 245.5 | +12.6 % | runner length 0.380 m → 0.354 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00445 s over 2.82 m` | 280.6 | 268.3 | -4.4 % | downstream run 2.82 m → 2.948 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 722 x 0.996/(4 x 0.568)` | 316.4 | 346.0 | +9.4 % | primary length 0.550 m → 0.503 m |
| expansion chamber, first pass band | `nc/2L = 1 x 680/(2 x 0.650)` | 523.1 | 608.8 | +16.4 % | chamber length 0.650 m, volume 22.1 L → 0.559 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 722 x 0.996/(4 x 0.568)` | 949.1 | 963.7 | +1.5 % | primary length 0.550 m → 0.542 m |
| expansion chamber, second pass band | `nc/2L = 2 x 680/(2 x 0.650)` | 1046.3 | 1231.6 | +17.7 % | chamber length 0.650 m, volume 22.1 L → 0.552 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 94.0 | -31.0 | 29.5 |
| 183.0 | -32.9 | 25.8 |
| 245.5 | -35.5 | 15.9 |
| 268.3 | -34.5 | 26.0 |
| 346.0 | -38.2 | 30.1 |
| 608.8 | -38.5 | 29.6 |
| 722.1 | -48.4 | 16.3 |
| 963.7 | -41.8 | 34.7 |
| 1231.6 | -40.4 | 39.2 |
| 1519.9 | -64.4 | 15.2 |
| 1602.2 | -59.5 | 21.4 |
| 1847.1 | -61.9 | 19.8 |
| 1978.1 | -58.6 | 30.0 |
| 2256.1 | -66.2 | 16.3 |
| 2636.8 | -65.2 | 25.5 |
| 3038.3 | -68.8 | 16.3 |
| 3227.0 | -65.6 | 25.4 |
| 3545.4 | -72.3 | 14.1 |
| 4254.3 | -78.7 | 17.2 |
| 6537.4 | -85.2 | 15.6 |
| 7958.6 | -92.5 | 16.6 |
| 9444.0 | -97.2 | 15.7 |
| 11964.2 | -111.4 | 14.4 |
| 15126.6 | -116.3 | 15.1 |

Noise floor tilt over 200 Hz-12 kHz: **-9.5 dB/octave**.


## Flat-plane V8

> Even 180 deg on both banks: two inline-fours sharing a crank.

4.5 L  8 cyl  12.5:1  ·  firing order 4  ·  900-8596 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -8.4 | — |
| 1 | null | -20.2 | — |
| 1.5 | null | -24.5 | — |
| 2 | +0.0 | -24.3 | -24.3 |
| 2.5 | null | -27.4 | — |
| 3 | null | -28.7 | — |
| 3.5 | null | -28.5 | — |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | null | -38.4 | — |
| 5 | null | -29.2 | — |
| 5.5 | null | -29.1 | — |
| 6 | +0.0 | -15.9 | -15.9 |
| 6.5 | null | -23.8 | — |
| 7 | null | -11.3 | — |
| 7.5 | null | -17.5 | — |
| 8 | +0.0 | -9.8 | -9.8 |

Driven orders: rms **15.3 dB**, mean -12.5 dB, worst -24.3 dB on order 2 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -8.4 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Implies |
|---|---|---:|---:|---:|---|
| block, first bending mode | `mass law on 180 kg` | 80.0 | not found | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 649 x 0.985/(4 x (0.900 + 0.020))` | 173.7 | not found | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00142 s over 0.92 m` | 176.4 | 207.7 | +17.7 % | downstream run 0.92 m → 0.781 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 710 x 0.993/(4 x 0.437)` | 403.2 | not found | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.015))` | 445.8 | not found | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00142 s over 0.92 m` | 529.3 | 521.3 | -1.5 % | downstream run 0.92 m → 0.934 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00142 s over 0.92 m` | 882.1 | 940.1 | +6.6 % | downstream run 0.92 m → 0.863 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 710 x 0.993/(4 x 0.437)` | 1209.5 | 1401.0 | +15.8 % | primary length 0.420 m → 0.363 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 207.7 | -33.2 | 22.4 |
| 521.3 | -39.6 | 25.4 |
| 806.3 | -46.8 | 27.2 |
| 940.1 | -47.8 | 18.7 |
| 1401.0 | -57.1 | 26.2 |
| 2513.3 | -62.5 | 23.4 |
| 3336.5 | -50.3 | 34.8 |
| 3770.1 | -67.8 | 26.1 |
| 4999.0 | -67.0 | 28.9 |
| 6695.4 | -59.0 | 36.6 |
| 7519.9 | -70.3 | 19.0 |
| 8332.1 | -72.4 | 22.1 |
| 10874.2 | -77.1 | 23.0 |
| 11717.5 | -86.0 | 21.8 |
| 12131.0 | -82.1 | 31.7 |
| 14646.6 | -90.5 | 19.4 |
| 15063.3 | -83.4 | 30.3 |
| 15908.2 | -87.2 | 29.6 |
| 17582.5 | -92.4 | 20.8 |
| 18422.5 | -92.0 | 21.9 |
| 19970.1 | -93.7 | 17.9 |
| 21314.6 | -95.6 | 37.8 |
| 21810.8 | -98.5 | 18.4 |
| 22736.1 | -99.8 | 21.8 |

Noise floor tilt over 200 Hz-12 kHz: **-6.6 dB/octave**.


## V10

> 72 deg firing, unevenly split across the banks: metallic and hard.

5.2 L  10 cyl  12.7:1  ·  firing order 5  ·  900-8496 rpm over 8.0 s

Reference: the crank itself: 10 firings a cycle on 2 banks

### Order balance [dB relative to order 5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -22.3 | +3.6 | — |
| 1 | -8.4 | -0.9 | +7.5 |
| 1.5 | -5.6 | -11.6 | -6.0 |
| 2 | -12.6 | -13.0 | — |
| 2.5 | -14.0 | -21.8 | — |
| 3 | -12.6 | -16.7 | — |
| 3.5 | -5.6 | +0.0 | +5.7 |
| 4 | -8.4 | -6.3 | +2.1 |
| 4.5 | -22.3 | -2.5 | — |
| **5** | +0.0 | +0.0 | +0.0 |
| 5.5 | -22.3 | -25.8 | — |
| 6 | -8.4 | -24.7 | -16.3 |
| 6.5 | -5.6 | -17.1 | -11.5 |
| 7 | -12.6 | -19.4 | — |
| 7.5 | -14.0 | -15.6 | — |
| 8 | -12.6 | -19.0 | — |
| 8.5 | -5.6 | -17.7 | -12.1 |
| 9 | -8.4 | -16.1 | -7.7 |
| 9.5 | -22.3 | -11.5 | — |
| 10 | +0.0 | -8.4 | -8.4 |

Driven orders: rms **8.9 dB**, mean -4.7 dB, worst -16.3 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest +3.6 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Implies |
|---|---|---:|---:|---:|---|
| block, first bending mode | `mass law on 220 kg` | 72.4 | not found | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 82.6 | not found | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 623 x 0.982/(4 x (1.100 + 0.019))` | 136.6 | not found | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 247.9 | 263.8 | +6.4 % | downstream run 1.92 m → 1.803 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.019))` | 363.3 | not found | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 413.1 | not found | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 687 x 0.995/(4 x 0.376)` | 454.1 | 465.2 | +2.5 % | primary length 0.360 m → 0.351 m |
| expansion chamber, first pass band | `nc/2L = 1 x 662/(2 x 0.400)` | 828.1 | 866.8 | +4.7 % | chamber length 0.400 m, volume 8.5 L → 0.382 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 687 x 0.995/(4 x 0.376)` | 1362.2 | 1309.1 | -3.9 % | primary length 0.360 m → 0.375 m |
| expansion chamber, second pass band | `nc/2L = 2 x 662/(2 x 0.400)` | 1656.2 | 1564.2 | -5.6 % | chamber length 0.400 m, volume 8.5 L → 0.424 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 42.7 | -38.6 | 17.5 |
| 263.8 | -41.4 | 24.2 |
| 465.2 | -40.7 | 24.9 |
| 866.8 | -56.2 | 14.2 |
| 1005.6 | -53.8 | 21.9 |
| 1130.5 | -53.8 | 19.7 |
| 1218.4 | -58.2 | 18.1 |
| 1309.1 | -55.7 | 13.9 |
| 1431.8 | -52.7 | 27.0 |
| 1564.2 | -63.5 | 13.6 |
| 2076.0 | -60.4 | 13.3 |
| 2537.1 | -64.0 | 15.1 |
| 3144.9 | -71.1 | 12.4 |
| 3794.1 | -69.6 | 16.5 |
| 4294.8 | -71.9 | 13.6 |
| 5466.5 | -75.1 | 15.6 |
| 6106.1 | -77.8 | 16.9 |
| 6801.5 | -82.9 | 21.3 |
| 9802.1 | -92.5 | 12.9 |
| 18084.2 | -109.9 | 12.6 |
| 18921.4 | -110.1 | 13.1 |
| 19717.4 | -111.0 | 14.8 |
| 21274.9 | -116.0 | 12.5 |
| 22220.3 | -115.5 | 13.9 |

Noise floor tilt over 200 Hz-12 kHz: **-5.6 dB/octave**.


## V12

> 60 deg firing, even on both banks: no beat left to hear, only pitch.

6.5 L  12 cyl  11.8:1  ·  firing order 6  ·  800-8496 rpm over 8.0 s

Reference: the crank itself: 12 firings a cycle on 2 banks

### Order balance [dB relative to order 6]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -8.8 | — |
| 1 | null | -18.7 | — |
| 1.5 | null | -17.9 | — |
| 2 | null | +5.1 | — |
| 2.5 | null | +20.2 | — |
| 3 | +0.0 | +25.8 | +25.8 |
| 3.5 | null | -19.3 | — |
| 4 | null | -23.5 | — |
| 4.5 | null | -24.2 | — |
| 5 | null | -27.1 | — |
| 5.5 | null | — | — |
| **6** | +0.0 | +0.0 | +0.0 |
| 6.5 | null | -48.7 | — |
| 7 | null | -31.9 | — |
| 7.5 | null | -26.9 | — |
| 8 | null | -26.6 | — |
| 8.5 | null | -5.9 | — |
| 9 | +0.0 | +4.0 | +4.0 |
| 9.5 | null | -24.9 | — |
| 10 | null | -29.1 | — |
| 10.5 | null | -17.9 | — |
| 11 | null | -13.0 | — |
| 11.5 | null | -17.5 | — |
| 12 | +0.0 | -11.8 | -11.8 |

Driven orders: rms **14.3 dB**, mean +4.5 dB, worst +25.8 dB on order 3 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest +20.2 dB on order 2.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Implies |
|---|---|---:|---:|---:|---|
| block, first bending mode | `mass law on 260 kg` | 66.6 | not found | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 119.9 | not found | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 648 x 0.946/(4 x (1.000 + 0.017))` | 150.7 | not found | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 359.6 | 374.9 | +4.3 % | downstream run 1.37 m → 1.311 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.140 + 0.017))` | 552.0 | not found | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 714 x 0.989/(4 x 0.314)` | 561.6 | not found | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 599.3 | not found | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 678/(2 x 0.350)` | 969.1 | 980.7 | +1.2 % | chamber length 0.350 m, volume 2.5 L → 0.346 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 714 x 0.989/(4 x 0.314)` | 1684.7 | 1769.1 | +5.0 % | primary length 0.300 m → 0.286 m |
| expansion chamber, second pass band | `nc/2L = 2 x 678/(2 x 0.350)` | 1938.2 | 1939.6 | +0.1 % | chamber length 0.350 m, volume 2.5 L → 0.350 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 269.9 | -19.0 | 40.9 |
| 374.9 | -22.1 | 29.3 |
| 980.7 | -57.7 | 15.3 |
| 1188.9 | -39.7 | 43.0 |
| 1498.7 | -74.5 | 12.5 |
| 1769.1 | -70.2 | 24.2 |
| 1866.0 | -69.8 | 14.7 |
| 1939.6 | -74.1 | 12.6 |
| 2042.0 | -68.7 | 19.3 |
| 2378.2 | -58.9 | 35.7 |
| 3127.0 | -78.6 | 16.0 |
| 3566.6 | -59.5 | 36.3 |
| 4316.4 | -74.7 | 18.9 |
| 4756.0 | -70.6 | 25.4 |
| 5945.3 | -78.2 | 24.3 |
| 7065.7 | -80.0 | 19.0 |
| 8173.9 | -84.4 | 15.1 |
| 8698.5 | -88.6 | 14.5 |
| 9667.6 | -91.2 | 12.9 |
| 13011.7 | -97.4 | 14.0 |
| 16845.6 | -99.9 | 13.4 |
| 17908.2 | -100.6 | 13.1 |
| 18933.2 | -104.1 | 14.7 |
| 21239.7 | -107.0 | 12.4 |

Noise floor tilt over 200 Hz-12 kHz: **-11.5 dB/octave**.


## 2-Rotor Wankel

> Four firings per cycle from two rotors: no valvetrain, no beat, just buzz.

2.6 L  4 cyl  10.0:1  ·  firing order 2  ·  950-8796 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -27.8 | — |
| 1 | null | -39.4 | — |
| 1.5 | null | -9.8 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -26.0 | — |
| 3 | null | -24.5 | — |
| 3.5 | null | -32.4 | — |
| 4 | +0.0 | -15.8 | -15.8 |

Driven orders: rms **11.2 dB**, mean -7.9 dB, worst -15.8 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -26.0 dB on order 2.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Implies |
|---|---|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00232 s over 1.52 m` | 107.8 | not found | — |  |
| block, first bending mode | `mass law on 95 kg` | 110.1 | not found | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 646 x 0.949/(4 x (1.000 + 0.018))` | 150.5 | not found | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 706 x 0.991/(4 x 0.620)` | 282.3 | not found | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00232 s over 1.52 m` | 323.5 | not found | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.200 + 0.016))` | 401.9 | 375.9 | -6.5 % | runner length 0.200 m → 0.231 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00232 s over 1.52 m` | 539.2 | 523.1 | -3.0 % | downstream run 1.52 m → 1.565 m |
| absorptive silencer, first pass band | `c/2L = 674/(2 x 0.500)` | 674.0 | 639.1 | -5.2 % | silencer length 0.500 m → 0.527 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 706 x 0.991/(4 x 0.620)` | 847.0 | 783.6 | -7.5 % | primary length 0.600 m → 0.649 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 56.6 | -34.2 | 13.0 |
| 205.2 | -15.3 | 42.6 |
| 375.9 | -35.2 | 27.1 |
| 523.1 | -35.2 | 18.6 |
| 582.9 | -33.4 | 27.7 |
| 639.1 | -40.2 | 12.2 |
| 783.6 | -43.0 | 18.8 |
| 959.7 | -50.9 | 10.6 |
| 1144.3 | -29.9 | 35.6 |
| 1718.9 | -43.1 | 35.2 |
| 2323.1 | -60.9 | 14.4 |
| 2854.1 | -61.7 | 10.2 |
| 3470.2 | -60.0 | 17.1 |
| 3997.8 | -66.0 | 12.6 |
| 4591.8 | -60.3 | 20.5 |
| 5290.8 | -65.7 | 15.0 |
| 6314.4 | -70.2 | 12.0 |
| 7039.0 | -75.9 | 11.1 |
| 8122.8 | -80.6 | 12.9 |
| 9487.6 | -85.9 | 15.7 |
| 12816.7 | -90.7 | 10.8 |
| 13364.6 | -94.1 | 12.7 |
| 15871.4 | -97.3 | 17.9 |
| 20414.4 | -100.9 | 14.1 |

Noise floor tilt over 200 Hz-12 kHz: **-6.7 dB/octave**.


## Turbo Inline-4

> Even 180 deg firing under a small fast single: bark, whistle, flutter.

2.0 L  4 cyl  9.6:1  ·  firing order 2  ·  820-6897 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -3.7 | — |
| 1 | null | -14.6 | — |
| 1.5 | null | -26.3 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -18.1 | — |
| 3 | null | -22.0 | — |
| 3.5 | null | -45.8 | — |
| 4 | +0.0 | +3.6 | +3.6 |

Driven orders: rms **2.5 dB**, mean +1.8 dB, worst +3.6 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -3.7 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Implies |
|---|---|---:|---:|---:|---|
| block, first bending mode | `mass law on 125 kg` | 96.0 | not found | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 101.1 | not found | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 643 x 0.996/(4 x (1.200 + 0.018))` | 131.4 | not found | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.5 L` | 163.7 | not found | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 303.4 | 246.9 | -18.6 % | downstream run 1.62 m → 1.989 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.240 + 0.017))` | 337.4 | 348.8 | +3.4 % | runner length 0.240 m → 0.249 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 505.7 | 382.2 | -24.4 % | downstream run 1.62 m → 2.141 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 748 x 0.998/(4 x 0.366)` | 510.5 | not found | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 692/(2 x 0.400)` | 865.6 | 874.3 | +1.0 % | chamber length 0.400 m, volume 3.1 L → 0.396 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 748 x 0.998/(4 x 0.366)` | 1531.6 | 1150.4 | -24.9 % | primary length 0.350 m → 0.466 m |
| expansion chamber, second pass band | `nc/2L = 2 x 692/(2 x 0.400)` | 1731.2 | 1631.9 | -5.7 % | chamber length 0.400 m, volume 3.1 L → 0.424 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 55.1 | -39.8 | 11.4 |
| 246.9 | -35.6 | 23.7 |
| 348.8 | -40.0 | 18.7 |
| 382.2 | -44.5 | 11.5 |
| 874.3 | -54.0 | 17.7 |
| 958.4 | -58.8 | 13.9 |
| 1150.4 | -43.8 | 28.5 |
| 1631.9 | -50.7 | 38.4 |
| 1917.6 | -64.6 | 18.6 |
| 2417.4 | -69.1 | 13.3 |
| 3182.4 | -68.7 | 16.3 |
| 3503.0 | -71.5 | 12.5 |
| 3960.9 | -70.4 | 14.5 |
| 4743.8 | -70.5 | 14.7 |
| 6257.9 | -70.3 | 12.0 |
| 7044.2 | -83.2 | 12.5 |
| 7503.4 | -88.0 | 12.1 |
| 9285.0 | -85.6 | 12.7 |
| 9853.5 | -83.5 | 19.7 |
| 10439.3 | -84.1 | 15.3 |
| 12759.3 | -84.4 | 27.9 |
| 13208.0 | -85.9 | 15.5 |
| 15410.7 | -109.5 | 12.9 |
| 19629.8 | -114.9 | 11.1 |

Noise floor tilt over 200 Hz-12 kHz: **-7.9 dB/octave**.


## Twin-turbo V8

> Hot-vee twins over the 90-180-270-180 burble: offbeat, but muffled.

4.0 L  8 cyl  10.0:1  ·  firing order 4  ·  760-7097 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -18.5 | -7.2 |
| 1 | null | +4.0 | — |
| 1.5 | -3.7 | +6.9 | +10.6 |
| 2 | null | -2.5 | — |
| 2.5 | -3.7 | +4.2 | +7.9 |
| 3 | null | -4.7 | — |
| 3.5 | -11.4 | -1.1 | +10.3 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -14.8 | -3.4 |
| 5 | null | -12.3 | — |
| 5.5 | -3.7 | -13.9 | -10.2 |
| 6 | null | -24.5 | — |
| 6.5 | -3.7 | -7.4 | -3.7 |
| 7 | null | -11.2 | — |
| 7.5 | -11.4 | -24.3 | -12.9 |
| 8 | +0.0 | -7.9 | -7.9 |

Driven orders: rms **8.3 dB**, mean -1.7 dB, worst -12.9 dB on order 7.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest +4.0 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Implies |
|---|---|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00408 s over 2.52 m` | 61.2 | not found | — |  |
| block, first bending mode | `mass law on 235 kg` | 70.0 | not found | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 600 x 0.990/(4 x (1.400 + 0.020))` | 104.5 | 114.3 | +9.3 % | tailpipe length 1.400 m, mouth radius 0.033 m → 1.299 m |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 4.5 L` | 173.8 | not found | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00408 s over 2.52 m` | 183.7 | 205.8 | +12.0 % | downstream run 2.52 m → 2.249 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00408 s over 2.52 m` | 306.1 | 292.2 | -4.5 % | downstream run 2.52 m → 2.640 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.260 + 0.018))` | 312.1 | 316.5 | +1.4 % | runner length 0.260 m → 0.274 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 692 x 0.996/(4 x 0.468)` | 367.9 | 416.5 | +13.2 % | primary length 0.450 m → 0.398 m |
| expansion chamber, first pass band | `nc/2L = 1 x 658/(2 x 0.550)` | 597.7 | 718.6 | +20.2 % | chamber length 0.550 m, volume 13.1 L → 0.458 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 692 x 0.996/(4 x 0.468)` | 1103.8 | 1277.4 | +15.7 % | primary length 0.450 m → 0.389 m |
| expansion chamber, second pass band | `nc/2L = 2 x 658/(2 x 0.550)` | 1195.5 | 1156.1 | -3.3 % | chamber length 0.550 m, volume 13.1 L → 0.569 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 30.7 | -41.2 | 17.1 |
| 114.3 | -28.1 | 30.9 |
| 205.8 | -27.5 | 26.3 |
| 292.2 | -35.1 | 25.2 |
| 316.5 | -38.2 | 16.2 |
| 416.5 | -39.2 | 27.5 |
| 718.6 | -42.6 | 26.5 |
| 833.6 | -44.4 | 23.1 |
| 1156.1 | -51.0 | 24.2 |
| 1277.4 | -57.5 | 18.9 |
| 1535.5 | -48.7 | 41.0 |
| 1920.1 | -62.7 | 22.0 |
| 2438.2 | -69.9 | 17.3 |
| 2687.9 | -65.1 | 22.9 |
| 3072.6 | -60.5 | 29.1 |
| 4236.0 | -69.6 | 18.9 |
| 5771.3 | -80.5 | 17.8 |
| 6539.9 | -80.3 | 19.1 |
| 7762.9 | -83.8 | 15.2 |
| 9667.4 | -85.1 | 17.2 |
| 12004.9 | -101.7 | 23.4 |
| 14395.5 | -104.4 | 16.7 |
| 15133.1 | -107.8 | 19.8 |
| 16173.1 | -111.6 | 15.1 |

Noise floor tilt over 200 Hz-12 kHz: **-8.1 dB/octave**.


## Turbo Inline-6

> Even 120 deg firing, no gaps at all, and a big lazy single over it.

3.0 L  6 cyl  9.2:1  ·  firing order 3  ·  780-7197 rpm over 8.0 s

Reference: the crank itself: 6 firings a cycle on 1 bank

### Order balance [dB relative to order 3]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -7.8 | — |
| 1 | null | -47.1 | — |
| 1.5 | null | -12.8 | — |
| 2 | null | -6.3 | — |
| 2.5 | null | -19.0 | — |
| **3** | +0.0 | +0.0 | +0.0 |
| 3.5 | null | -46.1 | — |
| 4 | null | -42.2 | — |
| 4.5 | null | -41.7 | — |
| 5 | null | -44.4 | — |
| 5.5 | null | — | — |
| 6 | +0.0 | -21.7 | -21.7 |

Driven orders: rms **15.3 dB**, mean -10.8 dB, worst -21.7 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -6.3 dB on order 2 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Implies |
|---|---|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00361 s over 2.22 m` | 69.2 | not found | — |  |
| block, first bending mode | `mass law on 195 kg` | 76.9 | not found | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 604 x 0.991/(4 x (1.600 + 0.021))` | 92.2 | not found | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 3.5 L` | 159.4 | 148.5 | -6.9 % |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00361 s over 2.22 m` | 207.5 | not found | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.300 + 0.018))` | 272.9 | 266.4 | -2.4 % | runner length 0.300 m → 0.326 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00361 s over 2.22 m` | 345.9 | not found | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 693 x 0.997/(4 x 0.497)` | 347.3 | not found | — |  |
| absorptive silencer, first pass band | `c/2L = 645/(2 x 0.600)` | 537.8 | not found | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 693 x 0.997/(4 x 0.497)` | 1041.9 | 1007.3 | -3.3 % | primary length 0.480 m → 0.496 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 45.7 | -35.8 | 14.7 |
| 148.5 | -18.6 | 40.9 |
| 266.4 | -27.1 | 26.2 |
| 731.0 | -40.5 | 28.5 |
| 809.4 | -57.9 | 12.4 |
| 1007.3 | -53.6 | 17.1 |
| 1477.2 | -54.1 | 34.1 |
| 1726.3 | -65.6 | 13.0 |
| 1839.7 | -64.8 | 15.9 |
| 2056.3 | -66.0 | 16.7 |
| 2214.2 | -66.8 | 11.7 |
| 2824.3 | -66.3 | 11.6 |
| 2945.6 | -69.7 | 14.3 |
| 3685.8 | -74.8 | 19.3 |
| 4423.5 | -75.6 | 15.4 |
| 6586.0 | -80.8 | 23.8 |
| 7379.9 | -87.6 | 16.3 |
| 8092.9 | -95.1 | 15.9 |
| 9432.1 | -103.7 | 13.1 |
| 12942.8 | -107.9 | 13.0 |
| 16290.1 | -113.7 | 13.5 |
| 16980.2 | -115.6 | 11.7 |
| 19803.7 | -116.7 | 10.9 |
| 20406.7 | -119.2 | 13.1 |

Noise floor tilt over 200 Hz-12 kHz: **-8.6 dB/octave**.


## Turbodiesel I4

> No spark at all: a premixed spike, an iron block, and clatter.

2.0 L  4 cyl  21.5:1  ·  firing order 2  ·  800-4998 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -11.0 | — |
| 1 | null | -7.4 | — |
| 1.5 | null | -27.4 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -28.2 | — |
| 3 | null | -28.5 | — |
| 3.5 | null | -15.6 | — |
| 4 | +0.0 | -11.1 | -11.1 |

Driven orders: rms **7.8 dB**, mean -5.5 dB, worst -11.1 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -7.4 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Implies |
|---|---|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 46.4 | not found | — |  |
| block, first bending mode | `mass law on 190 kg` | 77.9 | not found | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 511 x 0.995/(4 x (1.300 + 0.017))` | 96.5 | not found | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 139.1 | not found | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.8 L` | 153.7 | 183.5 | +19.4 % |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 231.8 | 269.2 | +16.1 % | downstream run 2.87 m → 2.469 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.016))` | 367.1 | 409.0 | +11.4 % | runner length 0.220 m → 0.212 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 595 x 0.998/(4 x 0.315)` | 471.7 | not found | — |  |
| absorptive silencer, first pass band | `c/2L = 570/(2 x 0.550)` | 518.2 | not found | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 551/(2 x 0.500)` | 550.6 | not found | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 551/(2 x 0.500)` | 1101.1 | 1199.9 | +9.0 % | chamber length 0.500 m, volume 22.4 L → 0.459 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 595 x 0.998/(4 x 0.315)` | 1415.1 | 1678.9 | +18.6 % | primary length 0.300 m → 0.253 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 183.5 | -45.2 | 15.8 |
| 269.2 | -57.0 | 13.6 |
| 409.0 | -44.8 | 29.6 |
| 1199.9 | -54.5 | 19.0 |
| 1678.9 | -60.6 | 24.6 |
| 2031.8 | -59.0 | 31.7 |
| 2529.9 | -72.8 | 15.1 |
| 2840.6 | -68.4 | 17.7 |
| 3663.2 | -67.2 | 21.9 |
| 4187.0 | -66.2 | 24.6 |
| 4746.7 | -68.4 | 16.2 |
| 7186.9 | -81.8 | 20.5 |
| 9170.3 | -80.5 | 26.0 |
| 9824.6 | -79.3 | 27.0 |
| 10347.4 | -79.9 | 13.9 |
| 12794.9 | -105.1 | 15.8 |
| 13382.7 | -103.2 | 19.6 |
| 13903.8 | -103.8 | 18.2 |
| 14517.6 | -107.3 | 18.5 |
| 17482.7 | -110.7 | 14.7 |
| 18059.0 | -111.7 | 19.4 |
| 20176.8 | -113.8 | 17.2 |
| 22559.6 | -127.1 | 20.9 |
| 23661.8 | -132.8 | 15.4 |

Noise floor tilt over 200 Hz-12 kHz: **-4.0 dB/octave**.


## Big Single

> One firing every two turns: the crank itself is the rhythm.

0.7 L  1 cyl  12.0:1  ·  firing order 0.5  ·  1250-7597 rpm over 8.0 s

Reference: the crank itself: 1 firings a cycle on 1 bank

### Order balance [dB relative to order 0.5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| **0.5** | +0.0 | +0.0 | +0.0 |
| 1 | +0.0 | +17.8 | +17.8 |

Driven orders: rms **12.6 dB**, mean +8.9 dB, worst +17.8 dB on order 1 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: none resolvable.

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Implies |
|---|---|---:|---:|---:|---|
| block, first bending mode | `mass law on 45 kg` | 120.0 | 111.1 | -7.4 % |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 259.1 | not found | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 808 x 1.000/(4 x 0.637)` | 317.1 | 360.0 | +13.5 % | primary length 0.620 m → 0.546 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.020))` | 434.6 | 419.8 | -3.4 % | runner length 0.180 m → 0.207 m |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 735 x 1.000/(4 x (0.350 + 0.015))` | 503.6 | not found | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 777.4 | not found | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 808 x 1.000/(4 x 0.637)` | 951.2 | not found | — |  |
| absorptive silencer, first pass band | `c/2L = 769/(2 x 0.360)` | 1067.6 | not found | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 1295.6 | 1197.9 | -7.5 % | downstream run 0.72 m → 0.784 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 111.1 | -31.4 | 20.3 |
| 360.0 | -36.6 | 15.0 |
| 419.8 | -35.9 | 17.7 |
| 1197.9 | -42.7 | 19.2 |
| 1916.6 | -45.3 | 14.8 |
| 2097.5 | -44.3 | 17.7 |
| 2874.8 | -43.6 | 23.8 |
| 3714.4 | -50.6 | 15.4 |
| 4698.8 | -57.7 | 14.8 |
| 5214.4 | -58.8 | 15.3 |
| 7250.7 | -71.8 | 14.8 |
| 7969.0 | -66.2 | 23.0 |
| 8447.8 | -68.9 | 16.8 |
| 8796.7 | -70.4 | 14.8 |
| 10528.3 | -72.7 | 15.0 |
| 11315.1 | -74.8 | 15.0 |
| 12870.8 | -81.4 | 16.1 |
| 14846.4 | -84.2 | 14.4 |
| 15327.2 | -84.7 | 14.8 |
| 15674.5 | -81.4 | 21.8 |
| 16482.3 | -87.3 | 16.2 |
| 18187.0 | -84.5 | 22.4 |
| 22528.5 | -95.5 | 15.7 |
| 23006.5 | -97.7 | 14.5 |

Noise floor tilt over 200 Hz-12 kHz: **-4.5 dB/octave**.

