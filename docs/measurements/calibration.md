# Calibration

Stage 16 of [the implementation plan](../IMPLEMENTATION_PLAN.md): the synth measured against a reference it did not produce. Regenerate with:

```bash
cargo run --release --example calibrate -- --markdown docs/measurements/calibration.md
```

Every figure below is *relative*: an order against the firing order, a measured frequency against a predicted one, a slope against an octave. That is deliberate and it is the point of the stage. A gain constant anywhere in the chain moves every order by the same number of decibels and cancels out of all of it, so nothing here can be closed by turning something up — only by changing a length, a volume or a radius.

| Engine | Firing order | Order balance rms | Worst order | Crank nulls | Modes placed | Floor tilt |
|---|---:|---:|---:|---:|---:|---:|
| [Inline-4](#inline-4) | 2 | 13.6 dB | -19.3 dB on 4 | -18.6 dB on 1 | 3/3 of 11 | -8.7 dB/oct |
| [Cross-plane V8](#cross-plane-v8) | 4 | 8.5 dB | -16.3 dB on 4.5 | -9.8 dB on 1, not enforced | 2/6 of 11 | -9.6 dB/oct |
| [Flat-plane V8](#flat-plane-v8) | 4 | 19.8 dB | -27.8 dB on 2 | -14.0 dB on 0.5, not enforced | 3/4 of 8 | -8.2 dB/oct |
| [V10](#v10) | 5 | 16.2 dB | -30.3 dB on 8.5 | -5.3 dB on 0.5, not enforced | 3/4 of 10 | -7.3 dB/oct |
| [V12](#v12) | 6 | 12.7 dB | -19.0 dB on 12 | -11.6 dB on 0.5, not enforced | 1/3 of 10 | -13.0 dB/oct |
| [2-Rotor Wankel](#2-rotor-wankel) | 2 | 9.3 dB | -13.1 dB on 4 | -21.9 dB on 0.5 | 3/3 of 9 | -8.9 dB/oct |
| [Turbo Inline-4](#turbo-inline-4) | 2 | 2.6 dB | -3.6 dB on 4 | -12.7 dB on 1, not enforced | 2/4 of 11 | -8.2 dB/oct |
| [Twin-turbo V8](#twin-turbo-v8) | 4 | 11.4 dB | -24.5 dB on 7.5 | -14.8 dB on 1, not enforced | 5/7 of 11 | -8.6 dB/oct |
| [Turbo Inline-6](#turbo-inline-6) | 3 | 11.7 dB | -16.6 dB on 6 | -22.9 dB on 0.5, not enforced | 3/3 of 10 | -9.7 dB/oct |
| [Turbodiesel I4](#turbodiesel-i4) | 2 | 12.5 dB | -17.6 dB on 4 | -9.1 dB on 1, not enforced | 2/5 of 12 | -5.1 dB/oct |
| [Big Single](#big-single) | 0.5 | 6.7 dB | +9.5 dB on 1 | — | 7/8 of 9 | -7.5 dB/oct |

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
| 0.5 | null | -24.7 | — |
| 1 | null | -18.6 | — |
| 1.5 | null | -38.9 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -43.0 | — |
| 3 | null | -36.0 | — |
| 3.5 | null | -42.9 | — |
| 4 | +0.0 | -19.3 | -19.3 |

Driven orders: rms **13.6 dB**, mean -9.7 dB, worst -19.3 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -18.6 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00330 s over 2.12 m` | 75.7 | not found | — | — | — |  |
| block, first bending mode | `mass law on 110 kg` | 102.3 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 622 x 0.951/(4 x (1.200 + 0.017))` | 121.6 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.2 L` | 162.1 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00330 s over 2.12 m` | 227.2 | 216.3 | -4.8 % | -37.1 | 20.0 | downstream run 2.12 m → 2.224 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.280 + 0.017))` | 292.0 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00330 s over 2.12 m` | 378.7 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 724 x 0.983/(4 x 0.416)` | 428.2 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 688/(2 x 0.450)` | 764.0 | 720.6 | -5.7 % | -57.9 | 10.3 | chamber length 0.450 m, volume 9.3 L → 0.477 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 724 x 0.983/(4 x 0.416)` | 1284.7 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 688/(2 x 0.450)` | 1528.0 | 1483.1 | -2.9 % | -67.8 | 21.0 | chamber length 0.450 m, volume 9.3 L → 0.464 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 34.8 | -40.5 | 18.5 |
| 216.3 | -37.1 | 20.0 |
| 720.6 | -57.9 | 10.3 |
| 926.9 | -57.6 | 13.4 |
| 1483.1 | -67.8 | 21.0 |
| 1680.6 | -69.6 | 10.8 |
| 2159.8 | -78.4 | 8.3 |
| 2518.3 | -83.7 | 13.3 |
| 2879.3 | -84.6 | 9.1 |
| 3304.2 | -87.3 | 7.6 |
| 3598.7 | -81.2 | 17.1 |
| 3824.7 | -85.1 | 8.7 |
| 4065.8 | -85.4 | 8.1 |
| 4558.8 | -82.7 | 10.9 |
| 5040.1 | -88.0 | 7.6 |
| 5520.2 | -88.1 | 7.5 |
| 6342.8 | -87.0 | 13.4 |
| 6614.0 | -88.8 | 8.3 |
| 7439.4 | -94.0 | 8.5 |
| 8145.3 | -101.5 | 9.3 |
| 11280.0 | -103.0 | 16.4 |
| 14778.1 | -118.7 | 7.9 |
| 17864.8 | -109.4 | 22.3 |
| 22456.5 | -119.4 | 8.8 |

Noise floor tilt over 200 Hz-12 kHz: **-8.7 dB/octave**.


## Cross-plane V8

> 90-180-270-180 gaps on each bank: the offbeat American burble.

5.0 L  8 cyl  11.0:1  ·  firing order 4  ·  750-6997 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -12.3 | -1.0 |
| 1 | null | -9.8 | — |
| 1.5 | -3.7 | -11.6 | -7.9 |
| 2 | null | -29.5 | — |
| 2.5 | -3.7 | +1.3 | +5.0 |
| 3 | null | -17.6 | — |
| 3.5 | -11.4 | -11.7 | -0.3 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -27.7 | -16.3 |
| 5 | null | -26.7 | — |
| 5.5 | -3.7 | -7.9 | -4.2 |
| 6 | null | -27.7 | — |
| 6.5 | -3.7 | -7.7 | -4.0 |
| 7 | null | -26.2 | — |
| 7.5 | -11.4 | -24.3 | -12.9 |
| 8 | +0.0 | -12.9 | -12.9 |

Driven orders: rms **8.5 dB**, mean -5.4 dB, worst -16.3 dB on order 4.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -9.8 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00445 s over 2.82 m` | 56.1 | not found | — | — | — |  |
| block, first bending mode | `mass law on 210 kg` | 74.1 | 84.5 | +14.0 % | -40.8 | 16.5 |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 611 x 0.986/(4 x (1.500 + 0.018))` | 99.1 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 4.8 L` | 140.3 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00445 s over 2.82 m` | 168.4 | 184.7 | +9.7 % | -46.9 | 9.2 | downstream run 2.82 m → 2.569 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.380 + 0.018))` | 218.0 | 244.9 | +12.3 % | -48.6 | 9.4 | runner length 0.380 m → 0.354 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00445 s over 2.82 m` | 280.6 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 722 x 0.996/(4 x 0.568)` | 316.4 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 680/(2 x 0.650)` | 523.1 | 407.2 | -22.2 % | -54.0 | 10.3 | chamber length 0.650 m, volume 22.1 L → 0.835 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 722 x 0.996/(4 x 0.568)` | 949.1 | 961.6 | +1.3 % | -59.5 | 13.6 | primary length 0.550 m → 0.543 m |
| expansion chamber, second pass band | `nc/2L = 2 x 680/(2 x 0.650)` | 1046.3 | 1200.0 | +14.7 % | -63.6 | 10.8 | chamber length 0.650 m, volume 22.1 L → 0.567 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 84.5 | -40.8 | 16.5 |
| 184.7 | -46.9 | 9.2 |
| 244.9 | -48.6 | 9.4 |
| 407.2 | -54.0 | 10.3 |
| 724.1 | -57.1 | 8.3 |
| 961.6 | -59.5 | 13.6 |
| 1200.0 | -63.6 | 10.8 |
| 1859.9 | -71.9 | 18.1 |
| 2319.6 | -76.8 | 8.7 |
| 3210.8 | -78.4 | 10.2 |
| 4383.5 | -89.8 | 8.7 |
| 4799.9 | -91.5 | 8.9 |
| 5919.8 | -90.4 | 12.7 |
| 8987.8 | -102.7 | 8.4 |
| 9599.9 | -102.2 | 11.5 |
| 11759.9 | -116.4 | 12.3 |
| 11999.8 | -115.9 | 8.3 |
| 12240.0 | -114.6 | 8.6 |
| 13200.2 | -110.2 | 18.4 |
| 15600.1 | -116.4 | 8.5 |
| 16799.6 | -115.9 | 10.8 |
| 21360.0 | -119.6 | 14.3 |
| 21599.7 | -121.0 | 8.0 |
| 23039.5 | -127.3 | 10.2 |

Noise floor tilt over 200 Hz-12 kHz: **-9.6 dB/octave**.


## Flat-plane V8

> Even 180 deg on both banks: two inline-fours sharing a crank.

4.5 L  8 cyl  12.5:1  ·  firing order 4  ·  900-8596 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -14.0 | — |
| 1 | null | -16.9 | — |
| 1.5 | null | -22.0 | — |
| 2 | +0.0 | -27.8 | -27.8 |
| 2.5 | null | -27.2 | — |
| 3 | null | -27.0 | — |
| 3.5 | null | -28.0 | — |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | null | -34.1 | — |
| 5 | null | -26.6 | — |
| 5.5 | null | -30.3 | — |
| 6 | +0.0 | -25.0 | -25.0 |
| 6.5 | null | -34.3 | — |
| 7 | null | -33.2 | — |
| 7.5 | null | -47.8 | — |
| 8 | +0.0 | -12.9 | -12.9 |

Driven orders: rms **19.8 dB**, mean -16.4 dB, worst -27.8 dB on order 2 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -14.0 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 180 kg` | 80.0 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 649 x 0.985/(4 x (0.900 + 0.020))` | 173.7 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00142 s over 0.92 m` | 176.4 | 211.9 | +20.1 % | -38.6 | 15.1 | downstream run 0.92 m → 0.766 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 710 x 0.993/(4 x 0.437)` | 403.2 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.015))` | 445.8 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00142 s over 0.92 m` | 529.3 | 527.7 | -0.3 % | -46.0 | 14.7 | downstream run 0.92 m → 0.923 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00142 s over 0.92 m` | 882.1 | 944.1 | +7.0 % | -56.2 | 15.6 | downstream run 0.92 m → 0.860 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 710 x 0.993/(4 x 0.437)` | 1209.5 | 1255.2 | +3.8 % | -64.2 | 15.5 | primary length 0.420 m → 0.405 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 211.9 | -38.6 | 15.1 |
| 527.7 | -46.0 | 14.7 |
| 944.1 | -56.2 | 15.6 |
| 1255.2 | -64.2 | 15.5 |
| 4080.6 | -74.3 | 29.7 |
| 4800.3 | -82.7 | 15.2 |
| 5520.1 | -83.3 | 15.6 |
| 6480.4 | -86.4 | 26.4 |
| 8641.0 | -95.9 | 13.2 |
| 9361.0 | -93.1 | 21.4 |
| 11040.7 | -99.2 | 13.3 |
| 12000.8 | -97.2 | 19.7 |
| 12960.9 | -104.6 | 13.8 |
| 13440.4 | -104.3 | 16.4 |
| 15120.9 | -100.0 | 20.4 |
| 16081.0 | -100.3 | 28.4 |
| 17041.4 | -104.5 | 14.1 |
| 17521.1 | -107.3 | 13.5 |
| 18480.9 | -106.9 | 17.3 |
| 20161.0 | -105.9 | 17.2 |
| 20641.1 | -108.4 | 13.2 |
| 21600.0 | -104.7 | 23.4 |
| 22320.3 | -107.6 | 19.4 |
| 23279.9 | -120.1 | 16.4 |

Noise floor tilt over 200 Hz-12 kHz: **-8.2 dB/octave**.


## V10

> 72 deg firing, unevenly split across the banks: metallic and hard.

5.2 L  10 cyl  12.7:1  ·  firing order 5  ·  900-8496 rpm over 8.0 s

Reference: the crank itself: 10 firings a cycle on 2 banks

### Order balance [dB relative to order 5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -22.3 | -5.3 | — |
| 1 | -8.4 | -10.8 | -2.4 |
| 1.5 | -5.6 | -14.8 | -9.2 |
| 2 | -12.6 | -21.5 | — |
| 2.5 | -14.0 | -23.2 | — |
| 3 | -12.6 | -17.6 | — |
| 3.5 | -5.6 | -12.8 | -7.2 |
| 4 | -8.4 | -16.5 | -8.1 |
| 4.5 | -22.3 | -32.4 | — |
| **5** | +0.0 | +0.0 | +0.0 |
| 5.5 | -22.3 | -35.5 | — |
| 6 | -8.4 | -24.2 | -15.8 |
| 6.5 | -5.6 | -26.5 | -20.8 |
| 7 | -12.6 | -24.8 | — |
| 7.5 | -14.0 | -31.2 | — |
| 8 | -12.6 | -27.7 | — |
| 8.5 | -5.6 | -36.0 | -30.3 |
| 9 | -8.4 | -33.5 | -25.1 |
| 9.5 | -22.3 | -35.0 | — |
| 10 | +0.0 | -13.6 | -13.6 |

Driven orders: rms **16.2 dB**, mean -13.3 dB, worst -30.3 dB on order 8.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -5.3 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 220 kg` | 72.4 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 82.6 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 623 x 0.982/(4 x (1.100 + 0.019))` | 136.6 | 117.9 | -13.7 % | -41.2 | 13.1 | tailpipe length 1.100 m, mouth radius 0.031 m → 1.297 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 247.9 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.019))` | 363.3 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 413.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 687 x 0.995/(4 x 0.376)` | 454.1 | 498.2 | +9.7 % | -52.4 | 10.9 | primary length 0.360 m → 0.328 m |
| expansion chamber, first pass band | `nc/2L = 1 x 662/(2 x 0.400)` | 828.1 | 841.8 | +1.7 % | -58.8 | 18.0 | chamber length 0.400 m, volume 8.5 L → 0.393 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 687 x 0.995/(4 x 0.376)` | 1362.2 | 1320.9 | -3.0 % | -63.0 | 12.4 | primary length 0.360 m → 0.371 m |
| expansion chamber, second pass band | `nc/2L = 2 x 662/(2 x 0.400)` | 1656.2 | not found | — | — | — |  |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 117.9 | -41.2 | 13.1 |
| 498.2 | -52.4 | 10.9 |
| 720.1 | -65.4 | 10.1 |
| 841.8 | -58.8 | 18.0 |
| 1200.5 | -65.8 | 9.3 |
| 1320.9 | -63.0 | 12.4 |
| 2156.9 | -61.6 | 13.2 |
| 2880.2 | -77.9 | 9.5 |
| 4374.6 | -75.4 | 14.4 |
| 4684.0 | -79.1 | 8.7 |
| 6364.2 | -84.3 | 22.4 |
| 6605.4 | -88.3 | 8.4 |
| 8880.7 | -98.4 | 9.7 |
| 9965.5 | -97.6 | 17.0 |
| 11041.4 | -105.5 | 8.4 |
| 13200.7 | -108.2 | 11.5 |
| 16079.9 | -110.9 | 14.8 |
| 18240.0 | -113.5 | 9.2 |
| 18960.6 | -112.0 | 9.6 |
| 19921.1 | -110.1 | 15.0 |
| 21120.0 | -114.7 | 14.7 |
| 21839.9 | -118.8 | 8.8 |
| 22317.8 | -119.0 | 8.5 |
| 23186.5 | -118.1 | 27.1 |

Noise floor tilt over 200 Hz-12 kHz: **-7.3 dB/octave**.


## V12

> 60 deg firing, even on both banks: no beat left to hear, only pitch.

6.5 L  12 cyl  11.8:1  ·  firing order 6  ·  800-8496 rpm over 8.0 s

Reference: the crank itself: 12 firings a cycle on 2 banks

### Order balance [dB relative to order 6]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -11.6 | — |
| 1 | null | -17.2 | — |
| 1.5 | null | -17.4 | — |
| 2 | null | -20.7 | — |
| 2.5 | null | -31.8 | — |
| 3 | +0.0 | +11.2 | +11.2 |
| 3.5 | null | -15.3 | — |
| 4 | null | -17.6 | — |
| 4.5 | null | -24.7 | — |
| 5 | null | -26.6 | — |
| 5.5 | null | — | — |
| **6** | +0.0 | +0.0 | +0.0 |
| 6.5 | null | -43.6 | — |
| 7 | null | -32.2 | — |
| 7.5 | null | -33.2 | — |
| 8 | null | -29.3 | — |
| 8.5 | null | -37.9 | — |
| 9 | +0.0 | -12.4 | -12.4 |
| 9.5 | null | -30.9 | — |
| 10 | null | -29.3 | — |
| 10.5 | null | -30.6 | — |
| 11 | null | -30.6 | — |
| 11.5 | null | -30.0 | — |
| 12 | +0.0 | -19.0 | -19.0 |

Driven orders: rms **12.7 dB**, mean -5.1 dB, worst -19.0 dB on order 12 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -11.6 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 260 kg` | 66.6 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 119.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 648 x 0.946/(4 x (1.000 + 0.017))` | 150.7 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 359.6 | 275.0 | -23.5 % | -40.3 | 14.5 | downstream run 1.37 m → 1.787 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.140 + 0.017))` | 552.0 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 714 x 0.989/(4 x 0.314)` | 561.6 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 599.3 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 678/(2 x 0.350)` | 969.1 | 1200.2 | +23.8 % | -58.6 | 14.9 | chamber length 0.350 m, volume 2.5 L → 0.283 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 714 x 0.989/(4 x 0.314)` | 1684.7 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 678/(2 x 0.350)` | 1938.2 | 1983.4 | +2.3 % | -68.8 | 19.4 | chamber length 0.350 m, volume 2.5 L → 0.342 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 275.0 | -40.3 | 14.5 |
| 1200.2 | -58.6 | 14.9 |
| 1983.4 | -68.8 | 19.4 |
| 2400.5 | -77.5 | 15.8 |
| 3360.3 | -77.2 | 16.5 |
| 3840.2 | -84.4 | 14.0 |
| 4800.1 | -87.1 | 14.6 |
| 6000.3 | -86.5 | 24.0 |
| 6960.4 | -88.8 | 14.7 |
| 7920.7 | -97.2 | 14.9 |
| 8160.6 | -92.9 | 19.1 |
| 9120.6 | -97.2 | 14.2 |
| 10320.8 | -98.1 | 14.4 |
| 11040.6 | -102.6 | 16.1 |
| 13200.7 | -103.6 | 22.9 |
| 14161.1 | -105.2 | 15.0 |
| 15601.2 | -105.7 | 15.7 |
| 17280.7 | -108.5 | 14.0 |
| 18000.7 | -107.3 | 16.4 |
| 19920.5 | -111.5 | 15.2 |
| 20880.4 | -110.2 | 16.9 |
| 21840.1 | -110.9 | 15.7 |
| 23040.2 | -114.9 | 14.7 |
| 23280.0 | -117.9 | 14.4 |

Noise floor tilt over 200 Hz-12 kHz: **-13.0 dB/octave**.


## 2-Rotor Wankel

> Four firings per cycle from two rotors: no valvetrain, no beat, just buzz.

2.6 L  4 cyl  10.0:1  ·  firing order 2  ·  950-8796 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -21.9 | — |
| 1 | null | -25.0 | — |
| 1.5 | null | -34.6 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -35.3 | — |
| 3 | null | -33.1 | — |
| 3.5 | null | -48.6 | — |
| 4 | +0.0 | -13.1 | -13.1 |

Driven orders: rms **9.3 dB**, mean -6.6 dB, worst -13.1 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -21.9 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00232 s over 1.52 m` | 107.8 | not found | — | — | — |  |
| block, first bending mode | `mass law on 95 kg` | 110.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 646 x 0.949/(4 x (1.000 + 0.018))` | 150.5 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 706 x 0.991/(4 x 0.620)` | 282.3 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00232 s over 1.52 m` | 323.5 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.200 + 0.016))` | 401.9 | 384.8 | -4.3 % | -48.6 | 10.1 | runner length 0.200 m → 0.226 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00232 s over 1.52 m` | 539.2 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 674/(2 x 0.500)` | 674.0 | 666.9 | -1.1 % | -46.9 | 8.9 | silencer length 0.500 m → 0.505 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 706 x 0.991/(4 x 0.620)` | 847.0 | 772.2 | -8.8 % | -43.4 | 21.6 | primary length 0.600 m → 0.658 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 53.0 | -30.8 | 25.3 |
| 208.4 | -32.3 | 12.3 |
| 384.8 | -48.6 | 10.1 |
| 666.9 | -46.9 | 8.9 |
| 772.2 | -43.4 | 21.6 |
| 960.0 | -46.7 | 14.1 |
| 1146.9 | -48.7 | 7.8 |
| 1680.5 | -56.8 | 20.4 |
| 2109.0 | -66.6 | 9.9 |
| 2693.1 | -72.9 | 11.9 |
| 4079.4 | -76.5 | 8.8 |
| 4560.2 | -77.1 | 9.9 |
| 5280.0 | -78.5 | 11.1 |
| 6480.6 | -80.8 | 13.3 |
| 7200.8 | -90.5 | 8.5 |
| 8161.9 | -100.9 | 10.6 |
| 8880.5 | -99.8 | 8.9 |
| 9600.0 | -97.2 | 15.9 |
| 10560.9 | -96.4 | 7.7 |
| 11281.2 | -97.5 | 8.5 |
| 11760.4 | -100.2 | 7.8 |
| 13680.0 | -104.7 | 11.2 |
| 16080.8 | -105.3 | 13.0 |
| 20880.4 | -109.9 | 15.1 |

Noise floor tilt over 200 Hz-12 kHz: **-8.9 dB/octave**.


## Turbo Inline-4

> Even 180 deg firing under a small fast single: bark, whistle, flutter.

2.0 L  4 cyl  9.6:1  ·  firing order 2  ·  820-6897 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -18.2 | — |
| 1 | null | -12.7 | — |
| 1.5 | null | -26.7 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -24.6 | — |
| 3 | null | -20.3 | — |
| 3.5 | null | -46.8 | — |
| 4 | +0.0 | -3.6 | -3.6 |

Driven orders: rms **2.6 dB**, mean -1.8 dB, worst -3.6 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -12.7 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 125 kg` | 96.0 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 101.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 643 x 0.996/(4 x (1.200 + 0.018))` | 131.4 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.5 L` | 163.7 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 303.4 | 250.1 | -17.6 % | -41.2 | 9.2 | downstream run 1.62 m → 1.963 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.240 + 0.017))` | 337.4 | 383.6 | +13.7 % | -51.1 | 9.1 | runner length 0.240 m → 0.226 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 505.7 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 748 x 0.998/(4 x 0.366)` | 510.5 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 692/(2 x 0.400)` | 865.6 | 826.6 | -4.5 % | -56.0 | 7.1 | chamber length 0.400 m, volume 3.1 L → 0.419 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 748 x 0.998/(4 x 0.366)` | 1531.6 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 692/(2 x 0.400)` | 1731.2 | 1635.6 | -5.5 % | -62.7 | 25.1 | chamber length 0.400 m, volume 3.1 L → 0.423 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 50.3 | -32.5 | 25.7 |
| 250.1 | -41.2 | 9.2 |
| 383.6 | -51.1 | 9.1 |
| 719.1 | -55.7 | 14.4 |
| 826.6 | -56.0 | 7.1 |
| 1012.2 | -55.3 | 12.1 |
| 1635.6 | -62.7 | 25.1 |
| 1969.0 | -67.1 | 10.2 |
| 2520.2 | -80.3 | 9.3 |
| 2690.2 | -77.2 | 11.6 |
| 3585.8 | -75.3 | 9.8 |
| 4187.4 | -74.4 | 14.7 |
| 4736.4 | -76.0 | 11.1 |
| 6480.8 | -71.4 | 20.2 |
| 6961.2 | -91.6 | 7.5 |
| 8374.7 | -97.1 | 7.7 |
| 8914.9 | -93.8 | 9.3 |
| 9473.7 | -88.0 | 8.1 |
| 10047.8 | -86.1 | 20.2 |
| 13127.0 | -85.9 | 21.8 |
| 14639.9 | -116.3 | 9.8 |
| 18000.7 | -107.1 | 26.6 |
| 19014.7 | -108.8 | 7.2 |
| 22319.5 | -116.7 | 9.2 |

Noise floor tilt over 200 Hz-12 kHz: **-8.2 dB/octave**.


## Twin-turbo V8

> Hot-vee twins over the 90-180-270-180 burble: offbeat, but muffled.

4.0 L  8 cyl  10.0:1  ·  firing order 4  ·  760-7097 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -15.0 | -3.7 |
| 1 | null | -14.8 | — |
| 1.5 | -3.7 | +0.8 | +4.5 |
| 2 | null | -23.2 | — |
| 2.5 | -3.7 | -1.8 | +1.9 |
| 3 | null | -30.7 | — |
| 3.5 | -11.4 | -13.8 | -2.5 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -33.0 | -21.6 |
| 5 | null | -27.7 | — |
| 5.5 | -3.7 | -14.8 | -11.1 |
| 6 | null | -35.5 | — |
| 6.5 | -3.7 | -7.0 | -3.3 |
| 7 | null | -29.9 | — |
| 7.5 | -11.4 | -35.8 | -24.5 |
| 8 | +0.0 | -7.6 | -7.6 |

Driven orders: rms **11.4 dB**, mean -6.8 dB, worst -24.5 dB on order 7.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -14.8 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00408 s over 2.52 m` | 61.2 | not found | — | — | — |  |
| block, first bending mode | `mass law on 235 kg` | 70.0 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 600 x 0.990/(4 x (1.400 + 0.020))` | 104.5 | 104.4 | -0.1 % | -36.0 | 21.3 | tailpipe length 1.400 m, mouth radius 0.033 m → 1.421 m |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 4.5 L` | 173.8 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00408 s over 2.52 m` | 183.7 | 207.0 | +12.7 % | -41.3 | 12.4 | downstream run 2.52 m → 2.237 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00408 s over 2.52 m` | 306.1 | 291.2 | -4.9 % | -48.0 | 9.0 | downstream run 2.52 m → 2.649 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.260 + 0.018))` | 312.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 692 x 0.996/(4 x 0.468)` | 367.9 | 371.1 | +0.9 % | -50.4 | 13.8 | primary length 0.450 m → 0.446 m |
| expansion chamber, first pass band | `nc/2L = 1 x 658/(2 x 0.550)` | 597.7 | 729.8 | +22.1 % | -54.0 | 11.2 | chamber length 0.550 m, volume 13.1 L → 0.450 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 692 x 0.996/(4 x 0.468)` | 1103.8 | 1111.9 | +0.7 % | -60.0 | 14.7 | primary length 0.450 m → 0.447 m |
| expansion chamber, second pass band | `nc/2L = 2 x 658/(2 x 0.550)` | 1195.5 | 1273.4 | +6.5 % | -64.1 | 9.1 | chamber length 0.550 m, volume 13.1 L → 0.516 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 104.4 | -36.0 | 21.3 |
| 207.0 | -41.3 | 12.4 |
| 291.2 | -48.0 | 9.0 |
| 371.1 | -50.4 | 13.8 |
| 729.8 | -54.0 | 11.2 |
| 854.4 | -51.7 | 14.8 |
| 1111.9 | -60.0 | 14.7 |
| 1273.4 | -64.1 | 9.1 |
| 1752.8 | -67.4 | 19.0 |
| 2023.6 | -68.5 | 13.0 |
| 2297.2 | -71.6 | 12.4 |
| 3194.3 | -73.1 | 12.8 |
| 3473.5 | -73.0 | 8.5 |
| 4215.3 | -73.2 | 10.2 |
| 4774.9 | -76.5 | 12.6 |
| 5355.9 | -91.4 | 10.7 |
| 5661.1 | -88.0 | 9.4 |
| 5865.7 | -86.3 | 17.6 |
| 9474.2 | -88.3 | 15.0 |
| 9959.0 | -101.2 | 8.8 |
| 12380.2 | -108.4 | 9.8 |
| 12961.2 | -108.1 | 15.3 |
| 20478.0 | -115.1 | 13.4 |
| 22208.0 | -119.4 | 10.0 |

Noise floor tilt over 200 Hz-12 kHz: **-8.6 dB/octave**.


## Turbo Inline-6

> Even 120 deg firing, no gaps at all, and a big lazy single over it.

3.0 L  6 cyl  9.2:1  ·  firing order 3  ·  780-7197 rpm over 8.0 s

Reference: the crank itself: 6 firings a cycle on 1 bank

### Order balance [dB relative to order 3]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -22.9 | — |
| 1 | null | -23.1 | — |
| 1.5 | null | -26.6 | — |
| 2 | null | -26.8 | — |
| 2.5 | null | — | — |
| **3** | +0.0 | +0.0 | +0.0 |
| 3.5 | null | -35.6 | — |
| 4 | null | -32.4 | — |
| 4.5 | null | -32.0 | — |
| 5 | null | -27.7 | — |
| 5.5 | null | — | — |
| 6 | +0.0 | -16.6 | -16.6 |

Driven orders: rms **11.7 dB**, mean -8.3 dB, worst -16.6 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -22.9 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00361 s over 2.22 m` | 69.2 | not found | — | — | — |  |
| block, first bending mode | `mass law on 195 kg` | 76.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 604 x 0.991/(4 x (1.600 + 0.021))` | 92.2 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 3.5 L` | 159.4 | 148.6 | -6.8 % | -32.6 | 24.0 |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00361 s over 2.22 m` | 207.5 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.300 + 0.018))` | 272.9 | 269.3 | -1.3 % | -39.1 | 10.9 | runner length 0.300 m → 0.322 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00361 s over 2.22 m` | 345.9 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 693 x 0.997/(4 x 0.497)` | 347.3 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 645/(2 x 0.600)` | 537.8 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 693 x 0.997/(4 x 0.497)` | 1041.9 | 1024.9 | -1.6 % | -53.7 | 14.0 | primary length 0.480 m → 0.488 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 41.4 | -39.5 | 8.9 |
| 148.6 | -32.6 | 24.0 |
| 269.3 | -39.1 | 10.9 |
| 721.1 | -55.4 | 12.9 |
| 1024.9 | -53.7 | 14.0 |
| 1407.1 | -71.7 | 10.6 |
| 1728.6 | -69.4 | 13.8 |
| 2827.2 | -67.5 | 15.4 |
| 3513.2 | -84.7 | 9.3 |
| 4095.8 | -84.7 | 9.6 |
| 4559.0 | -88.5 | 10.4 |
| 5554.5 | -84.1 | 15.9 |
| 6000.2 | -96.1 | 9.3 |
| 6720.6 | -98.2 | 11.2 |
| 6960.3 | -100.6 | 9.1 |
| 7440.7 | -104.6 | 10.1 |
| 9701.9 | -108.3 | 10.2 |
| 10799.4 | -109.5 | 19.5 |
| 12720.4 | -113.7 | 10.2 |
| 14640.2 | -119.8 | 11.5 |
| 15359.8 | -117.4 | 16.0 |
| 18481.0 | -121.1 | 10.8 |
| 21258.0 | -123.3 | 9.2 |
| 21943.3 | -122.3 | 18.2 |

Noise floor tilt over 200 Hz-12 kHz: **-9.7 dB/octave**.


## Turbodiesel I4

> No spark at all: a premixed spike, an iron block, and clatter.

2.0 L  4 cyl  21.5:1  ·  firing order 2  ·  800-4998 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -13.2 | — |
| 1 | null | -9.1 | — |
| 1.5 | null | — | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | — | — |
| 3 | null | -24.8 | — |
| 3.5 | null | — | — |
| 4 | +0.0 | -17.6 | -17.6 |

Driven orders: rms **12.5 dB**, mean -8.8 dB, worst -17.6 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -9.1 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 46.4 | not found | — | — | — |  |
| block, first bending mode | `mass law on 190 kg` | 77.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 511 x 0.995/(4 x (1.300 + 0.017))` | 96.5 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 139.1 | 119.8 | -13.9 % | -45.7 | 12.4 | downstream run 2.87 m → 3.328 m |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.8 L` | 153.7 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 231.8 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.016))` | 367.1 | 409.3 | +11.5 % | -51.6 | 22.2 | runner length 0.220 m → 0.212 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 595 x 0.998/(4 x 0.315)` | 471.7 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 570/(2 x 0.550)` | 518.2 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 551/(2 x 0.500)` | 550.6 | 602.5 | +9.4 % | -59.0 | 8.8 | chamber length 0.500 m, volume 22.4 L → 0.457 m |
| expansion chamber, second pass band | `nc/2L = 2 x 551/(2 x 0.500)` | 1101.1 | 1095.6 | -0.5 % | -65.5 | 8.5 | chamber length 0.500 m, volume 22.4 L → 0.503 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 595 x 0.998/(4 x 0.315)` | 1415.1 | 1559.4 | +10.2 % | -70.3 | 21.7 | primary length 0.300 m → 0.272 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 119.8 | -45.7 | 12.4 |
| 409.3 | -51.6 | 22.2 |
| 602.5 | -59.0 | 8.8 |
| 847.8 | -56.9 | 13.3 |
| 961.3 | -64.7 | 11.3 |
| 1095.6 | -65.5 | 8.5 |
| 1559.4 | -70.3 | 21.7 |
| 2037.7 | -74.9 | 15.9 |
| 3043.0 | -73.9 | 11.5 |
| 3592.5 | -69.9 | 14.4 |
| 4187.5 | -68.9 | 21.5 |
| 4747.6 | -72.9 | 16.3 |
| 5824.7 | -90.4 | 10.9 |
| 5954.9 | -91.9 | 8.3 |
| 7187.2 | -82.5 | 18.6 |
| 7768.6 | -86.9 | 9.3 |
| 8375.4 | -93.9 | 10.8 |
| 8846.8 | -88.7 | 8.9 |
| 10003.8 | -82.6 | 26.4 |
| 11873.1 | -112.5 | 8.3 |
| 12109.7 | -111.9 | 9.2 |
| 13071.3 | -106.8 | 15.8 |
| 16899.5 | -112.2 | 24.7 |
| 20543.8 | -117.3 | 10.6 |

Noise floor tilt over 200 Hz-12 kHz: **-5.1 dB/octave**.


## Big Single

> One firing every two turns: the crank itself is the rhythm.

0.7 L  1 cyl  12.0:1  ·  firing order 0.5  ·  1250-7597 rpm over 8.0 s

Reference: the crank itself: 1 firings a cycle on 1 bank

### Order balance [dB relative to order 0.5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| **0.5** | +0.0 | +0.0 | +0.0 |
| 1 | +0.0 | +9.5 | +9.5 |

Driven orders: rms **6.7 dB**, mean +4.7 dB, worst +9.5 dB on order 1 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: none resolvable.

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 45 kg` | 120.0 | 115.9 | -3.4 % | -31.9 | 20.6 |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 259.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 808 x 1.000/(4 x 0.637)` | 317.1 | 299.6 | -5.5 % | -41.4 | 10.4 | primary length 0.620 m → 0.656 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.020))` | 434.6 | 419.4 | -3.5 % | -39.6 | 14.1 | runner length 0.180 m → 0.207 m |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 735 x 1.000/(4 x (0.350 + 0.015))` | 503.6 | 479.1 | -4.9 % | -41.8 | 12.6 | tailpipe length 0.350 m, mouth radius 0.024 m → 0.383 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 777.4 | 778.1 | +0.1 % | -42.3 | 8.2 | downstream run 0.72 m → 0.724 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 808 x 1.000/(4 x 0.637)` | 951.2 | 1138.5 | +19.7 % | -54.8 | 8.2 | primary length 0.620 m → 0.518 m |
| absorptive silencer, first pass band | `c/2L = 769/(2 x 0.360)` | 1067.6 | 1078.2 | +1.0 % | -59.9 | 8.0 | silencer length 0.360 m → 0.356 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 1295.6 | 1255.8 | -3.1 % | -53.4 | 7.0 | downstream run 0.72 m → 0.748 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 115.9 | -31.9 | 20.6 |
| 299.6 | -41.4 | 10.4 |
| 359.4 | -39.5 | 13.3 |
| 419.4 | -39.6 | 14.1 |
| 479.1 | -41.8 | 12.6 |
| 563.5 | -41.1 | 7.4 |
| 778.1 | -42.3 | 8.2 |
| 1078.2 | -59.9 | 8.0 |
| 1138.5 | -54.8 | 8.2 |
| 1197.6 | -52.5 | 15.7 |
| 1255.8 | -53.4 | 7.0 |
| 1915.9 | -57.2 | 9.7 |
| 1974.7 | -56.2 | 8.9 |
| 2035.6 | -55.3 | 9.9 |
| 2096.2 | -55.6 | 15.4 |
| 2155.4 | -57.4 | 9.0 |
| 2875.3 | -57.2 | 9.4 |
| 4401.5 | -72.5 | 8.0 |
| 5334.2 | -73.1 | 13.8 |
| 8764.4 | -80.8 | 11.6 |
| 11092.7 | -85.4 | 10.7 |
| 15612.1 | -95.3 | 10.7 |
| 18130.0 | -97.5 | 11.4 |
| 22419.4 | -105.8 | 7.3 |

Noise floor tilt over 200 Hz-12 kHz: **-7.5 dB/octave**.

