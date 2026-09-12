# Calibration

Stage 16 of [the implementation plan](../IMPLEMENTATION_PLAN.md): the synth measured against a reference it did not produce. Regenerate with:

```bash
cargo run --release --example calibrate -- --markdown docs/measurements/calibration.md
```

Every figure below is *relative*: an order against the firing order, a measured frequency against a predicted one, a slope against an octave. That is deliberate and it is the point of the stage. A gain constant anywhere in the chain moves every order by the same number of decibels and cancels out of all of it, so nothing here can be closed by turning something up — only by changing a length, a volume or a radius.

| Engine | Firing order | Order balance rms | Worst order | Crank nulls | Modes placed | Floor tilt |
|---|---:|---:|---:|---:|---:|---:|
| [Inline-4](#inline-4) | 2 | 12.2 dB | -17.3 dB on 4 | -15.7 dB on 1 | 2/3 of 11 | -9.9 dB/oct |
| [Cross-plane V8](#cross-plane-v8) | 4 | 13.1 dB | -25.6 dB on 7.5 | -12.5 dB on 1, not enforced | 1/4 of 11 | -11.7 dB/oct |
| [Flat-plane V8](#flat-plane-v8) | 4 | 20.5 dB | -27.6 dB on 2 | -13.8 dB on 0.5, not enforced | 1/2 of 8 | -10.2 dB/oct |
| [V10](#v10) | 5 | 17.1 dB | -29.8 dB on 8.5 | -5.1 dB on 0.5, not enforced | 2/3 of 10 | -8.2 dB/oct |
| [V12](#v12) | 6 | 14.4 dB | -20.7 dB on 12 | -11.5 dB on 0.5, not enforced | 1/2 of 10 | -13.9 dB/oct |
| [2-Rotor Wankel](#2-rotor-wankel) | 2 | 8.1 dB | -11.4 dB on 4 | -19.8 dB on 0.5 | 1/1 of 9 | -9.7 dB/oct |
| [Turbo Inline-4](#turbo-inline-4) | 2 | 3.5 dB | -5.0 dB on 4 | -12.7 dB on 1, not enforced | 1/4 of 11 | -8.2 dB/oct |
| [Twin-turbo V8](#twin-turbo-v8) | 4 | 14.9 dB | -35.8 dB on 7.5 | -15.4 dB on 1, not enforced | 3/6 of 11 | -8.6 dB/oct |
| [Turbo Inline-6](#turbo-inline-6) | 3 | 11.5 dB | -16.3 dB on 6 | -20.0 dB on 0.5, not enforced | 2/2 of 10 | -9.8 dB/oct |
| [Turbodiesel I4](#turbodiesel-i4) | 2 | 12.1 dB | -17.2 dB on 4 | -8.6 dB on 1, not enforced | 1/3 of 12 | -5.0 dB/oct |
| [Big Single](#big-single) | 0.5 | 5.5 dB | +7.8 dB on 1 | — | 6/6 of 9 | -7.7 dB/oct |

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

5. **A primary's quarter wave is now a hump rather than a peak, and mostly reports as not found.** It used to be the loudest thing near its own frequency, and stretching the inline-four's primaries by half moved it bodily from 403 Hz to 244 Hz. That was a nearly lossless network talking: with the wall taking a fiftieth of the decibels `alpha` asks for, the whole run from the valve to the mouth was one resonator of enormous Q. With the loss filter tracking `alpha` and the valve opening once a cycle, a primary's loop pays about 1.4 dB a round trip and the four-into-one behind it sends most of what arrives onward. The length still reaches the sound — the band's centre of gravity moves down monotonically as the primaries are stretched, which is what the test suite now asserts — but it no longer stands up as a peak the picker can place, and the count of modes found in the table above fell across the catalogue when it stopped doing so. Whether a real header's primary is more prominent than this one is the open question, and it is a question about the enhancement factor on the wall loss, which has no derivation.

6. **Several predicted modes leave no peak at all.** The turbodiesel is the extreme — it radiates through its block rather than its pipe, so the exhaust chain barely reaches the listener and five of its twelve predicted modes are absent rather than misplaced. Absence is the honest report: a peak found more than 25 % from a prediction is a different mode, not that one in the wrong place.


## Inline-4

> Even 180 deg firing on one bank: a hard, plain four-cylinder bark.

2.0 L  4 cyl  11.5:1  ·  firing order 2  ·  850-7397 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -21.9 | — |
| 1 | null | -15.7 | — |
| 1.5 | null | -35.5 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -48.8 | — |
| 3 | null | -34.6 | — |
| 3.5 | null | -39.9 | — |
| 4 | +0.0 | -17.3 | -17.3 |

Driven orders: rms **12.2 dB**, mean -8.7 dB, worst -17.3 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -15.7 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00330 s over 2.12 m` | 75.7 | not found | — | — | — |  |
| block, first bending mode | `mass law on 110 kg` | 102.3 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 622 x 0.951/(4 x (1.200 + 0.017))` | 121.6 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.2 L` | 162.1 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00330 s over 2.12 m` | 227.2 | 214.8 | -5.5 % | -40.6 | 15.7 | downstream run 2.12 m → 2.239 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.280 + 0.017))` | 292.0 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00330 s over 2.12 m` | 378.7 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 724 x 0.983/(4 x 0.416)` | 428.2 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 688/(2 x 0.450)` | 764.0 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 724 x 0.983/(4 x 0.416)` | 1284.7 | 1047.5 | -18.5 % | -60.4 | 11.0 | primary length 0.400 m → 0.491 m |
| expansion chamber, second pass band | `nc/2L = 2 x 688/(2 x 0.450)` | 1528.0 | 1483.0 | -3.0 % | -68.2 | 21.9 | chamber length 0.450 m, volume 9.3 L → 0.464 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 34.5 | -43.0 | 15.6 |
| 214.8 | -40.6 | 15.7 |
| 959.2 | -61.7 | 12.4 |
| 1047.5 | -60.4 | 11.0 |
| 1483.0 | -68.2 | 21.9 |
| 2158.6 | -79.5 | 10.3 |
| 2400.3 | -87.8 | 11.8 |
| 2878.7 | -86.0 | 12.7 |
| 3302.3 | -88.4 | 9.6 |
| 3598.5 | -82.0 | 18.8 |
| 4320.3 | -88.2 | 11.3 |
| 4703.0 | -87.9 | 9.7 |
| 5520.2 | -91.2 | 14.7 |
| 5760.4 | -93.3 | 10.2 |
| 6240.2 | -97.4 | 9.3 |
| 6480.4 | -96.1 | 13.1 |
| 7200.4 | -100.9 | 9.0 |
| 8398.6 | -107.4 | 9.8 |
| 10174.0 | -107.8 | 11.7 |
| 12000.2 | -111.0 | 11.0 |
| 12240.3 | -111.2 | 9.9 |
| 15165.7 | -121.9 | 10.9 |
| 18000.0 | -117.5 | 17.4 |
| 19440.2 | -119.5 | 9.6 |

Noise floor tilt over 200 Hz-12 kHz: **-9.9 dB/octave**.


## Cross-plane V8

> 90-180-270-180 gaps on each bank: the offbeat American burble.

5.0 L  8 cyl  11.0:1  ·  firing order 4  ·  750-6997 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -11.8 | -0.5 |
| 1 | null | -12.5 | — |
| 1.5 | -3.7 | -12.1 | -8.4 |
| 2 | null | -26.3 | — |
| 2.5 | -3.7 | -1.8 | +1.9 |
| 3 | null | -36.6 | — |
| 3.5 | -11.4 | -16.8 | -5.5 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -34.3 | -23.0 |
| 5 | null | -23.8 | — |
| 5.5 | -3.7 | -15.1 | -11.4 |
| 6 | null | -27.3 | — |
| 6.5 | -3.7 | -12.7 | -9.0 |
| 7 | null | -28.2 | — |
| 7.5 | -11.4 | -36.9 | -25.6 |
| 8 | +0.0 | -14.5 | -14.5 |

Driven orders: rms **13.1 dB**, mean -9.6 dB, worst -25.6 dB on order 7.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -12.5 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00445 s over 2.82 m` | 56.1 | not found | — | — | — |  |
| block, first bending mode | `mass law on 210 kg` | 74.1 | 81.6 | +10.2 % | -43.7 | 12.7 |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 611 x 0.986/(4 x (1.500 + 0.018))` | 99.1 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 4.8 L` | 140.3 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00445 s over 2.82 m` | 168.4 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.380 + 0.018))` | 218.0 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00445 s over 2.82 m` | 280.6 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 722 x 0.996/(4 x 0.568)` | 316.4 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 680/(2 x 0.650)` | 523.1 | 443.6 | -15.2 % | -54.0 | 12.7 | chamber length 0.650 m, volume 22.1 L → 0.767 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 722 x 0.996/(4 x 0.568)` | 949.1 | 960.4 | +1.2 % | -65.5 | 13.8 | primary length 0.550 m → 0.544 m |
| expansion chamber, second pass band | `nc/2L = 2 x 680/(2 x 0.650)` | 1046.3 | 1199.7 | +14.7 % | -65.6 | 15.5 | chamber length 0.650 m, volume 22.1 L → 0.567 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 81.6 | -43.7 | 12.7 |
| 443.6 | -54.0 | 12.7 |
| 960.4 | -65.5 | 13.8 |
| 1199.7 | -65.6 | 15.5 |
| 1535.3 | -74.7 | 17.7 |
| 3360.2 | -84.8 | 13.5 |
| 3840.0 | -92.3 | 11.2 |
| 4319.9 | -94.0 | 11.3 |
| 4800.1 | -93.6 | 16.8 |
| 5040.0 | -98.5 | 14.6 |
| 6960.7 | -100.7 | 14.8 |
| 8400.1 | -108.3 | 14.1 |
| 9599.9 | -103.4 | 21.4 |
| 11759.9 | -117.2 | 15.2 |
| 12000.0 | -118.6 | 16.7 |
| 12240.1 | -117.0 | 12.6 |
| 13200.0 | -111.3 | 25.0 |
| 15360.0 | -125.3 | 13.0 |
| 15600.1 | -119.3 | 15.0 |
| 16799.8 | -117.2 | 21.4 |
| 21360.0 | -121.5 | 22.3 |
| 21599.9 | -122.5 | 14.1 |
| 22319.9 | -134.3 | 11.4 |
| 22799.9 | -132.4 | 13.3 |

Noise floor tilt over 200 Hz-12 kHz: **-11.7 dB/octave**.


## Flat-plane V8

> Even 180 deg on both banks: two inline-fours sharing a crank.

4.5 L  8 cyl  12.5:1  ·  firing order 4  ·  900-8596 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -13.8 | — |
| 1 | null | -16.6 | — |
| 1.5 | null | -21.5 | — |
| 2 | +0.0 | -27.6 | -27.6 |
| 2.5 | null | -26.7 | — |
| 3 | null | -26.5 | — |
| 3.5 | null | -27.6 | — |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | null | -33.5 | — |
| 5 | null | -26.2 | — |
| 5.5 | null | -30.2 | — |
| 6 | +0.0 | -27.4 | -27.4 |
| 6.5 | null | -33.3 | — |
| 7 | null | -32.4 | — |
| 7.5 | null | -56.0 | — |
| 8 | +0.0 | -13.3 | -13.3 |

Driven orders: rms **20.5 dB**, mean -17.1 dB, worst -27.6 dB on order 2 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -13.8 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 180 kg` | 80.0 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 649 x 0.985/(4 x (0.900 + 0.020))` | 173.7 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00142 s over 0.92 m` | 176.4 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 710 x 0.993/(4 x 0.437)` | 403.2 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.015))` | 445.8 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00142 s over 0.92 m` | 529.3 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00142 s over 0.92 m` | 882.1 | 959.8 | +8.8 % | -58.7 | 16.2 | downstream run 0.92 m → 0.845 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 710 x 0.993/(4 x 0.437)` | 1209.5 | 1440.3 | +19.1 % | -68.6 | 18.7 | primary length 0.420 m → 0.353 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 959.8 | -58.7 | 16.2 |
| 1440.3 | -68.6 | 18.7 |
| 3120.2 | -77.1 | 16.3 |
| 3360.2 | -84.4 | 14.8 |
| 3840.3 | -85.2 | 14.1 |
| 4080.3 | -82.1 | 24.7 |
| 4800.2 | -85.7 | 17.1 |
| 5280.2 | -87.0 | 15.9 |
| 5520.3 | -85.3 | 19.2 |
| 5760.4 | -89.2 | 17.2 |
| 6240.4 | -92.5 | 17.1 |
| 6480.4 | -90.8 | 27.4 |
| 8640.4 | -100.8 | 15.1 |
| 9600.7 | -97.6 | 22.3 |
| 10560.6 | -101.5 | 15.9 |
| 10800.3 | -104.4 | 16.5 |
| 11040.6 | -105.5 | 14.5 |
| 12000.8 | -104.6 | 18.0 |
| 13200.7 | -110.8 | 14.4 |
| 13440.1 | -107.8 | 17.9 |
| 16080.5 | -108.4 | 21.1 |
| 17760.1 | -113.4 | 14.4 |
| 18720.1 | -112.3 | 18.8 |
| 21360.1 | -113.1 | 18.7 |

Noise floor tilt over 200 Hz-12 kHz: **-10.2 dB/octave**.


## V10

> 72 deg firing, unevenly split across the banks: metallic and hard.

5.2 L  10 cyl  12.7:1  ·  firing order 5  ·  900-8496 rpm over 8.0 s

Reference: the crank itself: 10 firings a cycle on 2 banks

### Order balance [dB relative to order 5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -22.3 | -5.1 | — |
| 1 | -8.4 | -10.8 | -2.4 |
| 1.5 | -5.6 | -14.9 | -9.3 |
| 2 | -12.6 | -21.5 | — |
| 2.5 | -14.0 | -25.1 | — |
| 3 | -12.6 | -17.6 | — |
| 3.5 | -5.6 | -18.2 | -12.6 |
| 4 | -8.4 | -18.2 | -9.8 |
| 4.5 | -22.3 | -31.8 | — |
| **5** | +0.0 | +0.0 | +0.0 |
| 5.5 | -22.3 | -49.8 | — |
| 6 | -8.4 | -27.0 | -18.6 |
| 6.5 | -5.6 | -26.6 | -21.0 |
| 7 | -12.6 | -29.8 | — |
| 7.5 | -14.0 | -32.5 | — |
| 8 | -12.6 | -28.0 | — |
| 8.5 | -5.6 | -35.4 | -29.8 |
| 9 | -8.4 | -34.3 | -25.9 |
| 9.5 | -22.3 | -39.4 | — |
| 10 | +0.0 | -14.7 | -14.7 |

Driven orders: rms **17.1 dB**, mean -14.4 dB, worst -29.8 dB on order 8.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -5.1 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 220 kg` | 72.4 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 82.6 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 623 x 0.982/(4 x (1.100 + 0.019))` | 136.6 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 247.9 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.019))` | 363.3 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 413.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 687 x 0.995/(4 x 0.376)` | 454.1 | 499.3 | +10.0 % | -52.7 | 10.9 | primary length 0.360 m → 0.327 m |
| expansion chamber, first pass band | `nc/2L = 1 x 662/(2 x 0.400)` | 828.1 | 719.9 | -13.1 % | -66.5 | 11.6 | chamber length 0.400 m, volume 8.5 L → 0.460 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 687 x 0.995/(4 x 0.376)` | 1362.2 | 1344.4 | -1.3 % | -66.0 | 11.6 | primary length 0.360 m → 0.365 m |
| expansion chamber, second pass band | `nc/2L = 2 x 662/(2 x 0.400)` | 1656.2 | not found | — | — | — |  |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 499.3 | -52.7 | 10.9 |
| 719.9 | -66.5 | 11.6 |
| 960.2 | -65.4 | 15.5 |
| 1344.4 | -66.0 | 11.6 |
| 2158.9 | -61.7 | 17.5 |
| 2880.1 | -79.6 | 13.6 |
| 4410.8 | -75.6 | 15.5 |
| 5760.1 | -97.0 | 14.2 |
| 6000.5 | -99.6 | 10.9 |
| 6240.3 | -94.8 | 19.2 |
| 8400.2 | -107.4 | 11.7 |
| 10319.9 | -104.6 | 15.8 |
| 10800.0 | -110.2 | 12.5 |
| 11040.0 | -110.3 | 11.5 |
| 12480.2 | -109.4 | 10.6 |
| 13200.0 | -110.7 | 15.9 |
| 13920.6 | -110.4 | 12.6 |
| 15360.0 | -114.7 | 11.4 |
| 16080.0 | -112.7 | 17.2 |
| 18240.0 | -115.7 | 20.5 |
| 18960.1 | -114.7 | 12.8 |
| 20400.0 | -119.6 | 15.7 |
| 21119.9 | -115.5 | 20.5 |
| 23280.0 | -126.6 | 30.4 |

Noise floor tilt over 200 Hz-12 kHz: **-8.2 dB/octave**.


## V12

> 60 deg firing, even on both banks: no beat left to hear, only pitch.

6.5 L  12 cyl  11.8:1  ·  firing order 6  ·  800-8496 rpm over 8.0 s

Reference: the crank itself: 12 firings a cycle on 2 banks

### Order balance [dB relative to order 6]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -11.5 | — |
| 1 | null | -17.0 | — |
| 1.5 | null | -17.3 | — |
| 2 | null | -19.8 | — |
| 2.5 | null | -31.6 | — |
| 3 | +0.0 | +8.9 | +8.9 |
| 3.5 | null | -15.1 | — |
| 4 | null | -17.4 | — |
| 4.5 | null | -24.6 | — |
| 5 | null | -26.5 | — |
| 5.5 | null | — | — |
| **6** | +0.0 | +0.0 | +0.0 |
| 6.5 | null | — | — |
| 7 | null | -32.0 | — |
| 7.5 | null | -32.1 | — |
| 8 | null | -30.5 | — |
| 8.5 | null | -41.8 | — |
| 9 | +0.0 | -17.8 | -17.8 |
| 9.5 | null | -31.0 | — |
| 10 | null | -29.6 | — |
| 10.5 | null | -30.1 | — |
| 11 | null | -30.8 | — |
| 11.5 | null | -33.9 | — |
| 12 | +0.0 | -20.7 | -20.7 |

Driven orders: rms **14.4 dB**, mean -7.4 dB, worst -20.7 dB on order 12 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -11.5 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 260 kg` | 66.6 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 119.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 648 x 0.946/(4 x (1.000 + 0.017))` | 150.7 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 359.6 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.140 + 0.017))` | 552.0 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 714 x 0.989/(4 x 0.314)` | 561.6 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 599.3 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 678/(2 x 0.350)` | 969.1 | 1200.3 | +23.9 % | -62.6 | 19.1 | chamber length 0.350 m, volume 2.5 L → 0.283 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 714 x 0.989/(4 x 0.314)` | 1684.7 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 678/(2 x 0.350)` | 1938.2 | 1920.2 | -0.9 % | -73.7 | 22.1 | chamber length 0.350 m, volume 2.5 L → 0.353 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 1200.3 | -62.6 | 19.1 |
| 1920.2 | -73.7 | 22.1 |
| 2160.1 | -75.1 | 17.1 |
| 2400.4 | -82.0 | 15.9 |
| 3600.1 | -79.9 | 20.3 |
| 4560.2 | -88.9 | 17.0 |
| 4800.3 | -89.1 | 16.4 |
| 5280.2 | -96.9 | 17.0 |
| 6000.4 | -85.7 | 30.2 |
| 6960.4 | -89.3 | 17.2 |
| 7920.4 | -98.8 | 20.5 |
| 8160.5 | -98.2 | 21.6 |
| 9120.4 | -97.6 | 16.0 |
| 11040.6 | -105.3 | 18.7 |
| 11280.6 | -111.6 | 17.5 |
| 12960.6 | -106.3 | 26.5 |
| 14640.7 | -107.3 | 19.6 |
| 14880.7 | -110.7 | 15.9 |
| 17040.7 | -109.7 | 19.2 |
| 17280.7 | -111.3 | 17.8 |
| 18000.6 | -114.3 | 16.9 |
| 18240.7 | -116.3 | 15.5 |
| 20880.4 | -114.9 | 21.4 |
| 21840.2 | -117.5 | 16.2 |

Noise floor tilt over 200 Hz-12 kHz: **-13.9 dB/octave**.


## 2-Rotor Wankel

> Four firings per cycle from two rotors: no valvetrain, no beat, just buzz.

2.6 L  4 cyl  10.0:1  ·  firing order 2  ·  950-8796 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -19.8 | — |
| 1 | null | -21.8 | — |
| 1.5 | null | -32.5 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -37.6 | — |
| 3 | null | -32.5 | — |
| 3.5 | null | -48.4 | — |
| 4 | +0.0 | -11.4 | -11.4 |

Driven orders: rms **8.1 dB**, mean -5.7 dB, worst -11.4 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -19.8 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00232 s over 1.52 m` | 107.8 | not found | — | — | — |  |
| block, first bending mode | `mass law on 95 kg` | 110.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 646 x 0.949/(4 x (1.000 + 0.018))` | 150.5 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 706 x 0.991/(4 x 0.620)` | 282.3 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00232 s over 1.52 m` | 323.5 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.200 + 0.016))` | 401.9 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00232 s over 1.52 m` | 539.2 | 485.5 | -10.0 % | -51.5 | 8.8 | downstream run 1.52 m → 1.687 m |
| absorptive silencer, first pass band | `c/2L = 674/(2 x 0.500)` | 674.0 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 706 x 0.991/(4 x 0.620)` | 847.0 | not found | — | — | — |  |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 52.4 | -33.9 | 21.2 |
| 485.5 | -51.5 | 8.8 |
| 1118.9 | -52.7 | 13.8 |
| 1732.3 | -67.8 | 14.0 |
| 2400.2 | -76.3 | 13.2 |
| 3359.9 | -77.4 | 13.0 |
| 4080.0 | -80.9 | 13.5 |
| 4560.2 | -86.4 | 9.2 |
| 4800.3 | -84.3 | 12.1 |
| 5520.4 | -86.3 | 16.2 |
| 5760.6 | -89.3 | 9.2 |
| 6000.6 | -91.2 | 8.7 |
| 6240.5 | -91.9 | 9.0 |
| 6480.5 | -88.8 | 12.9 |
| 7200.4 | -95.6 | 10.7 |
| 7440.3 | -95.3 | 9.1 |
| 8160.4 | -103.9 | 10.7 |
| 9600.0 | -99.7 | 17.8 |
| 11280.3 | -106.2 | 8.9 |
| 13680.1 | -104.7 | 15.2 |
| 14400.1 | -110.0 | 9.9 |
| 16080.1 | -109.4 | 9.7 |
| 16800.0 | -108.4 | 15.7 |
| 20880.1 | -111.7 | 15.4 |

Noise floor tilt over 200 Hz-12 kHz: **-9.7 dB/octave**.


## Turbo Inline-4

> Even 180 deg firing under a small fast single: bark, whistle, flutter.

2.0 L  4 cyl  9.6:1  ·  firing order 2  ·  820-6897 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -18.4 | — |
| 1 | null | -12.7 | — |
| 1.5 | null | -26.7 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -27.9 | — |
| 3 | null | -23.3 | — |
| 3.5 | null | -46.9 | — |
| 4 | +0.0 | -5.0 | -5.0 |

Driven orders: rms **3.5 dB**, mean -2.5 dB, worst -5.0 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -12.7 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 125 kg` | 96.0 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 101.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 643 x 0.996/(4 x (1.200 + 0.018))` | 131.4 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.5 L` | 163.7 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 303.4 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.240 + 0.017))` | 337.4 | 383.7 | +13.7 % | -51.3 | 11.0 | runner length 0.240 m → 0.226 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 505.7 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 748 x 0.998/(4 x 0.366)` | 510.5 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 692/(2 x 0.400)` | 865.6 | 959.4 | +10.8 % | -65.2 | 7.9 | chamber length 0.400 m, volume 3.1 L → 0.361 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 748 x 0.998/(4 x 0.366)` | 1531.6 | 1150.4 | -24.9 % | -57.8 | 16.5 | primary length 0.350 m → 0.466 m |
| expansion chamber, second pass band | `nc/2L = 2 x 692/(2 x 0.400)` | 1731.2 | 1636.8 | -5.4 % | -63.7 | 24.6 | chamber length 0.400 m, volume 3.1 L → 0.423 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 49.3 | -34.7 | 22.9 |
| 383.7 | -51.3 | 11.0 |
| 719.8 | -62.7 | 9.3 |
| 959.4 | -65.2 | 7.9 |
| 1150.4 | -57.8 | 16.5 |
| 1636.8 | -63.7 | 24.6 |
| 1920.1 | -73.8 | 9.6 |
| 3100.7 | -80.0 | 7.7 |
| 3585.7 | -75.4 | 12.2 |
| 4187.5 | -74.6 | 18.1 |
| 4735.9 | -77.4 | 15.6 |
| 5220.6 | -80.7 | 7.7 |
| 6480.1 | -71.4 | 20.5 |
| 7337.7 | -90.7 | 7.8 |
| 7679.8 | -93.5 | 7.4 |
| 7848.2 | -92.4 | 8.1 |
| 8374.6 | -98.7 | 10.8 |
| 8915.6 | -93.9 | 9.6 |
| 9473.2 | -88.0 | 8.3 |
| 10046.1 | -86.3 | 22.8 |
| 13130.8 | -85.9 | 26.5 |
| 14639.9 | -116.3 | 9.9 |
| 18001.4 | -114.2 | 21.4 |
| 22079.6 | -125.1 | 7.7 |

Noise floor tilt over 200 Hz-12 kHz: **-8.2 dB/octave**.


## Twin-turbo V8

> Hot-vee twins over the 90-180-270-180 burble: offbeat, but muffled.

4.0 L  8 cyl  10.0:1  ·  firing order 4  ·  760-7097 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -14.0 | -2.6 |
| 1 | null | -15.4 | — |
| 1.5 | -3.7 | -2.0 | +1.7 |
| 2 | null | -24.2 | — |
| 2.5 | -3.7 | -3.2 | +0.5 |
| 3 | null | -42.5 | — |
| 3.5 | -11.4 | -19.5 | -8.1 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -32.6 | -21.2 |
| 5 | null | -28.9 | — |
| 5.5 | -3.7 | -18.4 | -14.7 |
| 6 | null | -32.7 | — |
| 6.5 | -3.7 | -10.3 | -6.6 |
| 7 | null | -29.3 | — |
| 7.5 | -11.4 | -47.1 | -35.8 |
| 8 | +0.0 | -12.2 | -12.2 |

Driven orders: rms **14.9 dB**, mean -9.9 dB, worst -35.8 dB on order 7.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -15.4 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00408 s over 2.52 m` | 61.2 | not found | — | — | — |  |
| block, first bending mode | `mass law on 235 kg` | 70.0 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 600 x 0.990/(4 x (1.400 + 0.020))` | 104.5 | 102.5 | -2.0 % | -39.8 | 16.1 | tailpipe length 1.400 m, mouth radius 0.033 m → 1.448 m |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 4.5 L` | 173.8 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00408 s over 2.52 m` | 183.7 | 209.7 | +14.2 % | -45.5 | 7.4 | downstream run 2.52 m → 2.207 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00408 s over 2.52 m` | 306.1 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.260 + 0.018))` | 312.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 692 x 0.996/(4 x 0.468)` | 367.9 | 371.0 | +0.8 % | -50.6 | 14.4 | primary length 0.450 m → 0.446 m |
| expansion chamber, first pass band | `nc/2L = 1 x 658/(2 x 0.550)` | 597.7 | 693.8 | +16.1 % | -58.3 | 8.4 | chamber length 0.550 m, volume 13.1 L → 0.474 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 692 x 0.996/(4 x 0.468)` | 1103.8 | 1111.7 | +0.7 % | -62.8 | 16.3 | primary length 0.450 m → 0.447 m |
| expansion chamber, second pass band | `nc/2L = 2 x 658/(2 x 0.550)` | 1195.5 | 960.0 | -19.7 % | -64.4 | 7.6 | chamber length 0.550 m, volume 13.1 L → 0.685 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 102.5 | -39.8 | 16.1 |
| 209.7 | -45.5 | 7.4 |
| 371.0 | -50.6 | 14.4 |
| 693.8 | -58.3 | 8.4 |
| 885.0 | -61.6 | 11.8 |
| 960.0 | -64.4 | 7.6 |
| 1111.7 | -62.8 | 16.3 |
| 1534.7 | -76.9 | 12.9 |
| 1815.2 | -77.2 | 8.9 |
| 2400.5 | -82.1 | 8.2 |
| 3062.7 | -76.6 | 8.5 |
| 3615.3 | -73.4 | 21.7 |
| 4215.7 | -73.3 | 10.3 |
| 4764.7 | -76.7 | 13.6 |
| 5280.0 | -98.0 | 12.2 |
| 5999.4 | -94.6 | 7.0 |
| 7233.4 | -89.4 | 20.3 |
| 9477.5 | -88.6 | 22.8 |
| 12001.1 | -121.8 | 8.3 |
| 13198.8 | -113.6 | 18.4 |
| 16729.9 | -117.9 | 15.7 |
| 20374.0 | -122.4 | 16.8 |
| 22178.3 | -132.2 | 8.2 |
| 22475.3 | -133.4 | 9.1 |

Noise floor tilt over 200 Hz-12 kHz: **-8.6 dB/octave**.


## Turbo Inline-6

> Even 120 deg firing, no gaps at all, and a big lazy single over it.

3.0 L  6 cyl  9.2:1  ·  firing order 3  ·  780-7197 rpm over 8.0 s

Reference: the crank itself: 6 firings a cycle on 1 bank

### Order balance [dB relative to order 3]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -20.0 | — |
| 1 | null | -20.3 | — |
| 1.5 | null | -24.2 | — |
| 2 | null | -24.0 | — |
| 2.5 | null | — | — |
| **3** | +0.0 | +0.0 | +0.0 |
| 3.5 | null | -32.5 | — |
| 4 | null | -30.0 | — |
| 4.5 | null | -29.2 | — |
| 5 | null | -24.9 | — |
| 5.5 | null | — | — |
| 6 | +0.0 | -16.3 | -16.3 |

Driven orders: rms **11.5 dB**, mean -8.1 dB, worst -16.3 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -20.0 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00361 s over 2.22 m` | 69.2 | not found | — | — | — |  |
| block, first bending mode | `mass law on 195 kg` | 76.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 604 x 0.991/(4 x (1.600 + 0.021))` | 92.2 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 3.5 L` | 159.4 | 147.6 | -7.4 % | -35.7 | 20.2 |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00361 s over 2.22 m` | 207.5 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.300 + 0.018))` | 272.9 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00361 s over 2.22 m` | 345.9 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 693 x 0.997/(4 x 0.497)` | 347.3 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 645/(2 x 0.600)` | 537.8 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 693 x 0.997/(4 x 0.497)` | 1041.9 | 1025.6 | -1.6 % | -54.6 | 18.8 | primary length 0.480 m → 0.488 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 147.6 | -35.7 | 20.2 |
| 1025.6 | -54.6 | 18.8 |
| 1407.4 | -72.4 | 13.7 |
| 1680.3 | -73.7 | 15.2 |
| 2826.7 | -67.5 | 20.5 |
| 3513.3 | -84.8 | 9.4 |
| 4095.7 | -84.7 | 9.7 |
| 4559.3 | -88.5 | 15.6 |
| 5554.4 | -84.2 | 19.1 |
| 6000.3 | -98.1 | 12.9 |
| 6960.3 | -100.2 | 11.8 |
| 7680.7 | -105.3 | 12.6 |
| 7920.6 | -114.9 | 13.6 |
| 8400.5 | -113.5 | 9.1 |
| 9702.1 | -108.4 | 10.7 |
| 10799.7 | -109.8 | 23.7 |
| 11175.7 | -114.1 | 11.7 |
| 11464.3 | -115.6 | 10.4 |
| 12156.7 | -114.7 | 14.0 |
| 14640.2 | -119.9 | 12.2 |
| 15359.9 | -117.4 | 17.6 |
| 18955.2 | -120.8 | 15.1 |
| 20879.2 | -128.6 | 10.0 |
| 21943.4 | -122.9 | 17.9 |

Noise floor tilt over 200 Hz-12 kHz: **-9.8 dB/octave**.


## Turbodiesel I4

> No spark at all: a premixed spike, an iron block, and clatter.

2.0 L  4 cyl  21.5:1  ·  firing order 2  ·  800-4998 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -12.7 | — |
| 1 | null | -8.6 | — |
| 1.5 | null | — | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | — | — |
| 3 | null | -24.5 | — |
| 3.5 | null | — | — |
| 4 | +0.0 | -17.2 | -17.2 |

Driven orders: rms **12.1 dB**, mean -8.6 dB, worst -17.2 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -8.6 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 46.4 | not found | — | — | — |  |
| block, first bending mode | `mass law on 190 kg` | 77.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 511 x 0.995/(4 x (1.300 + 0.017))` | 96.5 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 139.1 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.8 L` | 153.7 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 231.8 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.016))` | 367.1 | 409.2 | +11.5 % | -51.5 | 22.4 | runner length 0.220 m → 0.212 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 595 x 0.998/(4 x 0.315)` | 471.7 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 570/(2 x 0.550)` | 518.2 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 551/(2 x 0.500)` | 550.6 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 551/(2 x 0.500)` | 1101.1 | 1095.6 | -0.5 % | -68.0 | 12.6 | chamber length 0.500 m, volume 22.4 L → 0.503 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 595 x 0.998/(4 x 0.315)` | 1415.1 | 1641.6 | +16.0 % | -78.3 | 15.7 | primary length 0.300 m → 0.259 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 409.2 | -51.5 | 22.4 |
| 847.8 | -57.1 | 16.3 |
| 959.4 | -71.0 | 8.5 |
| 1095.6 | -68.0 | 12.6 |
| 1641.6 | -78.3 | 15.7 |
| 1678.5 | -77.8 | 9.7 |
| 2037.2 | -76.3 | 16.0 |
| 3043.0 | -73.9 | 11.6 |
| 3592.5 | -70.0 | 14.3 |
| 4187.6 | -69.0 | 26.5 |
| 4747.9 | -72.9 | 16.6 |
| 5695.9 | -91.6 | 8.3 |
| 5824.6 | -90.7 | 11.2 |
| 5954.8 | -92.0 | 8.5 |
| 6086.5 | -91.1 | 8.3 |
| 7187.0 | -82.6 | 20.0 |
| 7768.6 | -86.9 | 9.1 |
| 8375.2 | -94.0 | 10.8 |
| 8846.7 | -88.7 | 8.9 |
| 10003.4 | -82.6 | 27.7 |
| 13652.6 | -107.3 | 24.1 |
| 16899.9 | -112.6 | 26.4 |
| 20543.9 | -117.3 | 17.3 |
| 23191.6 | -131.9 | 10.9 |

Noise floor tilt over 200 Hz-12 kHz: **-5.0 dB/octave**.


## Big Single

> One firing every two turns: the crank itself is the rhythm.

0.7 L  1 cyl  12.0:1  ·  firing order 0.5  ·  1250-7597 rpm over 8.0 s

Reference: the crank itself: 1 firings a cycle on 1 bank

### Order balance [dB relative to order 0.5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| **0.5** | +0.0 | +0.0 | +0.0 |
| 1 | +0.0 | +7.8 | +7.8 |

Driven orders: rms **5.5 dB**, mean +3.9 dB, worst +7.8 dB on order 1 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: none resolvable.

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 45 kg` | 120.0 | 116.2 | -3.1 % | -34.3 | 18.0 |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 259.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 808 x 1.000/(4 x 0.637)` | 317.1 | 299.6 | -5.5 % | -42.3 | 11.8 | primary length 0.620 m → 0.656 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.020))` | 434.6 | 419.4 | -3.5 % | -39.7 | 14.2 | runner length 0.180 m → 0.207 m |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 735 x 1.000/(4 x (0.350 + 0.015))` | 503.6 | 479.1 | -4.9 % | -42.4 | 13.6 | tailpipe length 0.350 m, mouth radius 0.024 m → 0.383 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 777.4 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 808 x 1.000/(4 x 0.637)` | 951.2 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 769/(2 x 0.360)` | 1067.6 | 1138.5 | +6.6 % | -55.0 | 11.5 | silencer length 0.360 m → 0.338 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 1295.6 | 1255.8 | -3.1 % | -54.5 | 10.4 | downstream run 0.72 m → 0.748 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 116.2 | -34.3 | 18.0 |
| 299.6 | -42.3 | 11.8 |
| 359.4 | -39.6 | 15.1 |
| 419.4 | -39.7 | 14.2 |
| 479.1 | -42.4 | 13.6 |
| 1138.5 | -55.0 | 11.5 |
| 1197.6 | -53.1 | 16.8 |
| 1255.8 | -54.5 | 10.4 |
| 1857.2 | -58.7 | 9.2 |
| 1916.1 | -57.9 | 13.2 |
| 1974.7 | -57.0 | 11.3 |
| 2035.4 | -55.8 | 20.1 |
| 2096.0 | -56.5 | 11.5 |
| 2155.4 | -58.9 | 10.8 |
| 2214.2 | -60.6 | 9.7 |
| 2874.7 | -59.3 | 12.9 |
| 5278.9 | -73.2 | 16.5 |
| 8765.1 | -81.4 | 15.6 |
| 11092.6 | -85.5 | 13.4 |
| 13089.3 | -93.6 | 10.0 |
| 15598.8 | -96.6 | 11.4 |
| 18130.1 | -98.0 | 13.4 |
| 20686.9 | -100.8 | 12.0 |
| 22479.0 | -107.1 | 14.1 |

Noise floor tilt over 200 Hz-12 kHz: **-7.7 dB/octave**.

