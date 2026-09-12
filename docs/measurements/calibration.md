# Calibration

Stage 16 of [the implementation plan](../IMPLEMENTATION_PLAN.md): the synth measured against a reference it did not produce. Regenerate with:

```bash
cargo run --release --example calibrate -- --markdown docs/measurements/calibration.md
```

Every figure below is *relative*: an order against the firing order, a measured frequency against a predicted one, a slope against an octave. That is deliberate and it is the point of the stage. A gain constant anywhere in the chain moves every order by the same number of decibels and cancels out of all of it, so nothing here can be closed by turning something up — only by changing a length, a volume or a radius.

| Engine | Firing order | Order balance rms | Worst order | Crank nulls | Modes placed | Floor tilt |
|---|---:|---:|---:|---:|---:|---:|
| [Inline-4](#inline-4) | 2 | 12.2 dB | -17.3 dB on 4 | -15.7 dB on 1 | 2/3 of 11 | -9.5 dB/oct |
| [Cross-plane V8](#cross-plane-v8) | 4 | 13.0 dB | -25.4 dB on 7.5 | -12.6 dB on 1, not enforced | 1/3 of 11 | -11.2 dB/oct |
| [Flat-plane V8](#flat-plane-v8) | 4 | 20.5 dB | -27.7 dB on 2 | -13.9 dB on 0.5, not enforced | 0/1 of 8 | -9.8 dB/oct |
| [V10](#v10) | 5 | 16.9 dB | -29.6 dB on 8.5 | -5.2 dB on 0.5, not enforced | 0/1 of 10 | -7.8 dB/oct |
| [V12](#v12) | 6 | 14.1 dB | -20.0 dB on 12 | -11.6 dB on 0.5, not enforced | 1/2 of 10 | -13.4 dB/oct |
| [2-Rotor Wankel](#2-rotor-wankel) | 2 | 8.0 dB | -11.3 dB on 4 | -19.8 dB on 0.5 | 0/0 of 9 | -9.4 dB/oct |
| [Turbo Inline-4](#turbo-inline-4) | 2 | 3.5 dB | -5.0 dB on 4 | -12.7 dB on 1, not enforced | 1/4 of 11 | -7.6 dB/oct |
| [Twin-turbo V8](#twin-turbo-v8) | 4 | 15.2 dB | -37.3 dB on 7.5 | -15.4 dB on 1, not enforced | 3/3 of 11 | -8.6 dB/oct |
| [Turbo Inline-6](#turbo-inline-6) | 3 | 11.5 dB | -16.3 dB on 6 | -20.0 dB on 0.5, not enforced | 2/2 of 10 | -9.1 dB/oct |
| [Turbodiesel I4](#turbodiesel-i4) | 2 | 12.2 dB | -17.3 dB on 4 | -8.6 dB on 1, not enforced | 1/3 of 12 | -4.2 dB/oct |
| [Big Single](#big-single) | 0.5 | 5.5 dB | +7.8 dB on 1 | — | 6/6 of 9 | -7.4 dB/oct |

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
| 2.5 | null | -48.9 | — |
| 3 | null | -34.6 | — |
| 3.5 | null | -39.9 | — |
| 4 | +0.0 | -17.3 | -17.3 |

Driven orders: rms **12.2 dB**, mean -8.6 dB, worst -17.3 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -15.7 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00330 s over 2.12 m` | 75.7 | not found | — | — | — |  |
| block, first bending mode | `mass law on 110 kg` | 102.3 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 622 x 0.951/(4 x (1.200 + 0.017))` | 121.6 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.2 L` | 162.1 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00330 s over 2.12 m` | 227.2 | 214.8 | -5.5 % | -40.6 | 15.8 | downstream run 2.12 m → 2.239 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.280 + 0.017))` | 292.0 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00330 s over 2.12 m` | 378.7 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 724 x 0.983/(4 x 0.416)` | 428.2 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 688/(2 x 0.450)` | 764.0 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 724 x 0.983/(4 x 0.416)` | 1284.7 | 1047.5 | -18.5 % | -60.1 | 11.1 | primary length 0.400 m → 0.491 m |
| expansion chamber, second pass band | `nc/2L = 2 x 688/(2 x 0.450)` | 1528.0 | 1482.9 | -3.0 % | -67.5 | 22.4 | chamber length 0.450 m, volume 9.3 L → 0.464 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 34.5 | -43.0 | 15.6 |
| 214.8 | -40.6 | 15.8 |
| 959.2 | -61.5 | 12.6 |
| 1047.5 | -60.1 | 11.1 |
| 1482.9 | -67.5 | 22.4 |
| 2158.6 | -78.2 | 11.2 |
| 2400.3 | -84.9 | 14.4 |
| 2878.8 | -84.5 | 13.5 |
| 3598.5 | -81.1 | 20.8 |
| 4702.7 | -86.1 | 14.2 |
| 5280.0 | -89.4 | 9.5 |
| 5520.2 | -89.8 | 15.0 |
| 6240.3 | -92.7 | 17.7 |
| 7200.4 | -94.8 | 17.5 |
| 7920.3 | -99.8 | 11.0 |
| 9240.0 | -104.9 | 9.9 |
| 11520.9 | -108.5 | 10.9 |
| 12719.6 | -107.7 | 14.8 |
| 13439.2 | -108.6 | 23.5 |
| 15719.2 | -116.6 | 10.3 |
| 17999.9 | -115.0 | 13.0 |
| 18720.8 | -117.0 | 10.0 |
| 19440.5 | -117.3 | 19.0 |
| 23279.5 | -128.8 | 11.6 |

Noise floor tilt over 200 Hz-12 kHz: **-9.5 dB/octave**.


## Cross-plane V8

> 90-180-270-180 gaps on each bank: the offbeat American burble.

5.0 L  8 cyl  11.0:1  ·  firing order 4  ·  750-6997 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -11.9 | -0.5 |
| 1 | null | -12.6 | — |
| 1.5 | -3.7 | -12.2 | -8.5 |
| 2 | null | -26.4 | — |
| 2.5 | -3.7 | -1.9 | +1.8 |
| 3 | null | -36.6 | — |
| 3.5 | -11.4 | -16.9 | -5.5 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -34.2 | -22.8 |
| 5 | null | -23.7 | — |
| 5.5 | -3.7 | -15.1 | -11.4 |
| 6 | null | -27.3 | — |
| 6.5 | -3.7 | -12.6 | -8.9 |
| 7 | null | -28.1 | — |
| 7.5 | -11.4 | -36.8 | -25.4 |
| 8 | +0.0 | -14.4 | -14.4 |

Driven orders: rms **13.0 dB**, mean -9.6 dB, worst -25.4 dB on order 7.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -12.6 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00445 s over 2.82 m` | 56.1 | not found | — | — | — |  |
| block, first bending mode | `mass law on 210 kg` | 74.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 611 x 0.986/(4 x (1.500 + 0.018))` | 99.1 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 4.8 L` | 140.3 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00445 s over 2.82 m` | 168.4 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.380 + 0.018))` | 218.0 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00445 s over 2.82 m` | 280.6 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 722 x 0.996/(4 x 0.568)` | 316.4 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 680/(2 x 0.650)` | 523.1 | 443.7 | -15.2 % | -54.0 | 12.7 | chamber length 0.650 m, volume 22.1 L → 0.766 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 722 x 0.996/(4 x 0.568)` | 949.1 | 960.4 | +1.2 % | -65.1 | 14.4 | primary length 0.550 m → 0.544 m |
| expansion chamber, second pass band | `nc/2L = 2 x 680/(2 x 0.650)` | 1046.3 | 1199.7 | +14.7 % | -65.2 | 15.4 | chamber length 0.650 m, volume 22.1 L → 0.567 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 443.7 | -54.0 | 12.7 |
| 960.4 | -65.1 | 14.4 |
| 1199.7 | -65.2 | 15.4 |
| 1535.3 | -73.7 | 18.0 |
| 3360.2 | -83.8 | 13.1 |
| 3840.1 | -91.3 | 15.7 |
| 4319.9 | -91.5 | 13.7 |
| 4800.1 | -90.8 | 19.6 |
| 5040.0 | -95.6 | 14.2 |
| 6000.1 | -98.4 | 16.1 |
| 6960.4 | -99.6 | 12.9 |
| 9359.9 | -102.6 | 18.6 |
| 9600.0 | -103.6 | 18.6 |
| 10800.0 | -108.4 | 16.0 |
| 11759.9 | -117.9 | 13.4 |
| 12000.0 | -113.0 | 19.6 |
| 12480.1 | -120.9 | 13.1 |
| 12960.0 | -111.9 | 20.2 |
| 13200.0 | -110.5 | 23.8 |
| 13966.8 | -113.8 | 13.7 |
| 15360.0 | -114.5 | 21.6 |
| 16799.9 | -116.3 | 22.3 |
| 18000.1 | -118.1 | 19.1 |
| 21360.0 | -119.9 | 19.8 |

Noise floor tilt over 200 Hz-12 kHz: **-11.2 dB/octave**.


## Flat-plane V8

> Even 180 deg on both banks: two inline-fours sharing a crank.

4.5 L  8 cyl  12.5:1  ·  firing order 4  ·  900-8596 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -13.9 | — |
| 1 | null | -16.8 | — |
| 1.5 | null | -21.6 | — |
| 2 | +0.0 | -27.7 | -27.7 |
| 2.5 | null | -26.8 | — |
| 3 | null | -26.6 | — |
| 3.5 | null | -27.7 | — |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | null | -33.6 | — |
| 5 | null | -26.2 | — |
| 5.5 | null | -30.2 | — |
| 6 | +0.0 | -27.4 | -27.4 |
| 6.5 | null | -33.2 | — |
| 7 | null | -32.4 | — |
| 7.5 | null | — | — |
| 8 | +0.0 | -12.9 | -12.9 |

Driven orders: rms **20.5 dB**, mean -17.0 dB, worst -27.7 dB on order 2 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -13.9 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 180 kg` | 80.0 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 649 x 0.985/(4 x (0.900 + 0.020))` | 173.7 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00142 s over 0.92 m` | 176.4 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 710 x 0.993/(4 x 0.437)` | 403.2 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.015))` | 445.8 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00142 s over 0.92 m` | 529.3 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00142 s over 0.92 m` | 882.1 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 710 x 0.993/(4 x 0.437)` | 1209.5 | 1440.3 | +19.1 % | -68.0 | 18.4 | primary length 0.420 m → 0.353 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 1440.3 | -68.0 | 18.4 |
| 3120.2 | -75.7 | 16.7 |
| 4080.3 | -81.9 | 18.5 |
| 4320.2 | -83.2 | 16.7 |
| 4800.3 | -83.3 | 22.1 |
| 5280.3 | -84.2 | 19.6 |
| 5520.3 | -85.5 | 16.3 |
| 6000.3 | -94.4 | 20.2 |
| 6240.5 | -94.4 | 17.8 |
| 6720.5 | -88.9 | 25.9 |
| 7200.5 | -90.6 | 18.7 |
| 7920.3 | -93.9 | 19.7 |
| 9840.2 | -97.1 | 16.7 |
| 10800.4 | -99.9 | 18.1 |
| 12000.8 | -102.3 | 18.1 |
| 12480.0 | -101.0 | 32.7 |
| 13200.3 | -107.2 | 19.4 |
| 13920.6 | -108.9 | 17.1 |
| 14641.5 | -106.6 | 18.0 |
| 15120.6 | -104.7 | 23.6 |
| 16080.6 | -107.6 | 17.7 |
| 18960.4 | -111.1 | 22.3 |
| 21360.1 | -111.5 | 30.3 |
| 23279.8 | -126.0 | 16.5 |

Noise floor tilt over 200 Hz-12 kHz: **-9.8 dB/octave**.


## V10

> 72 deg firing, unevenly split across the banks: metallic and hard.

5.2 L  10 cyl  12.7:1  ·  firing order 5  ·  900-8496 rpm over 8.0 s

Reference: the crank itself: 10 firings a cycle on 2 banks

### Order balance [dB relative to order 5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -22.3 | -5.2 | — |
| 1 | -8.4 | -10.8 | -2.4 |
| 1.5 | -5.6 | -14.9 | -9.3 |
| 2 | -12.6 | -21.6 | — |
| 2.5 | -14.0 | -25.1 | — |
| 3 | -12.6 | -17.6 | — |
| 3.5 | -5.6 | -18.2 | -12.6 |
| 4 | -8.4 | -18.2 | -9.8 |
| 4.5 | -22.3 | -31.6 | — |
| **5** | +0.0 | +0.0 | +0.0 |
| 5.5 | -22.3 | -49.3 | — |
| 6 | -8.4 | -26.9 | -18.5 |
| 6.5 | -5.6 | -26.6 | -20.9 |
| 7 | -12.6 | -29.7 | — |
| 7.5 | -14.0 | -32.3 | — |
| 8 | -12.6 | -27.7 | — |
| 8.5 | -5.6 | -35.2 | -29.6 |
| 9 | -8.4 | -34.0 | -25.7 |
| 9.5 | -22.3 | -39.4 | — |
| 10 | +0.0 | -14.1 | -14.1 |

Driven orders: rms **16.9 dB**, mean -14.3 dB, worst -29.6 dB on order 8.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -5.2 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 220 kg` | 72.4 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 82.6 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 623 x 0.982/(4 x (1.100 + 0.019))` | 136.6 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 247.9 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.019))` | 363.3 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 413.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 687 x 0.995/(4 x 0.376)` | 454.1 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 662/(2 x 0.400)` | 828.1 | 960.2 | +16.0 % | -65.1 | 15.1 | chamber length 0.400 m, volume 8.5 L → 0.345 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 687 x 0.995/(4 x 0.376)` | 1362.2 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 662/(2 x 0.400)` | 1656.2 | not found | — | — | — |  |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 960.2 | -65.1 | 15.1 |
| 2086.3 | -61.7 | 18.0 |
| 2880.1 | -78.7 | 15.2 |
| 4319.0 | -73.8 | 19.1 |
| 5760.1 | -93.6 | 17.2 |
| 6240.3 | -93.4 | 16.9 |
| 6833.9 | -95.0 | 13.8 |
| 7200.2 | -96.1 | 13.7 |
| 7920.5 | -93.5 | 20.7 |
| 10320.2 | -99.8 | 13.6 |
| 11660.1 | -102.8 | 18.8 |
| 12960.1 | -110.3 | 13.1 |
| 13680.4 | -108.9 | 14.6 |
| 14160.8 | -107.9 | 15.8 |
| 14640.8 | -107.7 | 16.6 |
| 15360.1 | -112.7 | 13.7 |
| 16079.9 | -111.8 | 17.0 |
| 17520.0 | -115.9 | 14.0 |
| 18240.0 | -114.9 | 14.4 |
| 18480.1 | -113.2 | 13.3 |
| 20640.1 | -121.2 | 12.9 |
| 21599.5 | -116.3 | 17.3 |
| 22559.8 | -121.2 | 16.6 |
| 23279.9 | -125.2 | 17.1 |

Noise floor tilt over 200 Hz-12 kHz: **-7.8 dB/octave**.


## V12

> 60 deg firing, even on both banks: no beat left to hear, only pitch.

6.5 L  12 cyl  11.8:1  ·  firing order 6  ·  800-8496 rpm over 8.0 s

Reference: the crank itself: 12 firings a cycle on 2 banks

### Order balance [dB relative to order 6]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -11.6 | — |
| 1 | null | -17.1 | — |
| 1.5 | null | -17.4 | — |
| 2 | null | -19.9 | — |
| 2.5 | null | -31.7 | — |
| 3 | +0.0 | +8.8 | +8.8 |
| 3.5 | null | -15.2 | — |
| 4 | null | -17.4 | — |
| 4.5 | null | -24.6 | — |
| 5 | null | -26.5 | — |
| 5.5 | null | — | — |
| **6** | +0.0 | +0.0 | +0.0 |
| 6.5 | null | — | — |
| 7 | null | -31.9 | — |
| 7.5 | null | -31.9 | — |
| 8 | null | -30.3 | — |
| 8.5 | null | -41.9 | — |
| 9 | +0.0 | -17.8 | -17.8 |
| 9.5 | null | -30.7 | — |
| 10 | null | -29.4 | — |
| 10.5 | null | -29.9 | — |
| 11 | null | -30.6 | — |
| 11.5 | null | -34.2 | — |
| 12 | +0.0 | -20.0 | -20.0 |

Driven orders: rms **14.1 dB**, mean -7.2 dB, worst -20.0 dB on order 12 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -11.6 dB on order 0.5 (pass).

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
| expansion chamber, first pass band | `nc/2L = 1 x 678/(2 x 0.350)` | 969.1 | 1200.3 | +23.9 % | -62.2 | 19.2 | chamber length 0.350 m, volume 2.5 L → 0.283 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 714 x 0.989/(4 x 0.314)` | 1684.7 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 678/(2 x 0.350)` | 1938.2 | 1920.2 | -0.9 % | -72.6 | 22.0 | chamber length 0.350 m, volume 2.5 L → 0.353 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 1200.3 | -62.2 | 19.2 |
| 1920.2 | -72.6 | 22.0 |
| 2160.1 | -75.0 | 16.4 |
| 3600.1 | -78.9 | 19.8 |
| 4320.2 | -85.5 | 16.6 |
| 5040.3 | -88.4 | 16.8 |
| 6000.4 | -85.0 | 26.5 |
| 6960.4 | -86.7 | 20.2 |
| 7920.4 | -92.6 | 21.1 |
| 8880.5 | -92.3 | 19.8 |
| 9840.4 | -99.7 | 19.6 |
| 11040.3 | -102.8 | 15.9 |
| 11280.5 | -102.3 | 18.5 |
| 11520.6 | -107.5 | 16.3 |
| 12960.7 | -104.7 | 21.1 |
| 13680.6 | -105.6 | 17.5 |
| 13920.7 | -109.6 | 17.0 |
| 14640.7 | -104.6 | 25.1 |
| 14880.6 | -106.9 | 20.5 |
| 15840.8 | -106.7 | 16.7 |
| 17040.6 | -107.7 | 18.8 |
| 18000.6 | -110.4 | 19.4 |
| 18240.7 | -112.5 | 16.6 |
| 23040.2 | -120.9 | 16.6 |

Noise floor tilt over 200 Hz-12 kHz: **-13.4 dB/octave**.


## 2-Rotor Wankel

> Four firings per cycle from two rotors: no valvetrain, no beat, just buzz.

2.6 L  4 cyl  10.0:1  ·  firing order 2  ·  950-8796 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -19.8 | — |
| 1 | null | -21.9 | — |
| 1.5 | null | -32.5 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -37.6 | — |
| 3 | null | -32.5 | — |
| 3.5 | null | -48.6 | — |
| 4 | +0.0 | -11.3 | -11.3 |

Driven orders: rms **8.0 dB**, mean -5.6 dB, worst -11.3 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -19.8 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00232 s over 1.52 m` | 107.8 | not found | — | — | — |  |
| block, first bending mode | `mass law on 95 kg` | 110.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 646 x 0.949/(4 x (1.000 + 0.018))` | 150.5 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 706 x 0.991/(4 x 0.620)` | 282.3 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00232 s over 1.52 m` | 323.5 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.200 + 0.016))` | 401.9 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00232 s over 1.52 m` | 539.2 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 674/(2 x 0.500)` | 674.0 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 706 x 0.991/(4 x 0.620)` | 847.0 | not found | — | — | — |  |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 52.4 | -33.9 | 21.2 |
| 1118.9 | -52.3 | 13.9 |
| 1732.3 | -66.9 | 16.3 |
| 2400.2 | -75.0 | 13.5 |
| 3359.9 | -75.6 | 15.2 |
| 4079.9 | -79.9 | 15.6 |
| 5520.4 | -84.3 | 14.4 |
| 6240.5 | -88.2 | 11.1 |
| 6480.5 | -88.1 | 12.9 |
| 7200.7 | -91.8 | 12.2 |
| 7920.4 | -96.5 | 13.3 |
| 8879.9 | -95.7 | 18.5 |
| 9839.6 | -100.1 | 11.6 |
| 10559.8 | -101.1 | 12.6 |
| 12720.4 | -103.8 | 11.3 |
| 13440.5 | -105.5 | 13.2 |
| 13920.8 | -105.7 | 11.4 |
| 14400.3 | -108.1 | 12.4 |
| 16080.0 | -108.5 | 11.7 |
| 16800.0 | -104.7 | 24.9 |
| 17759.8 | -109.9 | 16.8 |
| 18479.9 | -110.7 | 13.9 |
| 20880.1 | -110.9 | 12.1 |
| 21362.3 | -112.5 | 11.5 |

Noise floor tilt over 200 Hz-12 kHz: **-9.4 dB/octave**.


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
| expansion chamber, first pass band | `nc/2L = 1 x 692/(2 x 0.400)` | 865.6 | 719.8 | -16.8 % | -62.5 | 9.3 | chamber length 0.400 m, volume 3.1 L → 0.481 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 748 x 0.998/(4 x 0.366)` | 1531.6 | 1150.4 | -24.9 % | -57.5 | 16.6 | primary length 0.350 m → 0.466 m |
| expansion chamber, second pass band | `nc/2L = 2 x 692/(2 x 0.400)` | 1731.2 | 1636.8 | -5.4 % | -62.9 | 26.0 | chamber length 0.400 m, volume 3.1 L → 0.423 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 49.3 | -34.7 | 22.9 |
| 383.7 | -51.3 | 11.0 |
| 719.8 | -62.5 | 9.3 |
| 1150.4 | -57.5 | 16.6 |
| 1636.8 | -62.9 | 26.0 |
| 1920.0 | -72.9 | 9.7 |
| 3023.9 | -79.4 | 9.8 |
| 3585.5 | -74.0 | 18.3 |
| 4187.1 | -72.3 | 17.1 |
| 4735.9 | -72.9 | 13.0 |
| 5625.3 | -72.1 | 10.5 |
| 6241.3 | -70.4 | 20.8 |
| 7676.8 | -89.1 | 11.6 |
| 8733.2 | -89.8 | 10.8 |
| 9285.2 | -85.6 | 17.8 |
| 9853.5 | -83.7 | 24.6 |
| 10438.0 | -84.2 | 15.4 |
| 12272.5 | -91.3 | 9.7 |
| 12766.6 | -84.4 | 24.5 |
| 13207.1 | -85.9 | 16.2 |
| 15601.4 | -114.8 | 9.4 |
| 18001.4 | -112.4 | 12.7 |
| 18961.0 | -114.3 | 10.2 |
| 19681.2 | -114.2 | 11.0 |

Noise floor tilt over 200 Hz-12 kHz: **-7.6 dB/octave**.


## Twin-turbo V8

> Hot-vee twins over the 90-180-270-180 burble: offbeat, but muffled.

4.0 L  8 cyl  10.0:1  ·  firing order 4  ·  760-7097 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -14.0 | -2.7 |
| 1 | null | -15.4 | — |
| 1.5 | -3.7 | -2.0 | +1.6 |
| 2 | null | -24.3 | — |
| 2.5 | -3.7 | -3.2 | +0.5 |
| 3 | null | -42.6 | — |
| 3.5 | -11.4 | -19.6 | -8.2 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -32.6 | -21.3 |
| 5 | null | -28.8 | — |
| 5.5 | -3.7 | -18.4 | -14.7 |
| 6 | null | -32.5 | — |
| 6.5 | -3.7 | -10.2 | -6.5 |
| 7 | null | -29.2 | — |
| 7.5 | -11.4 | -48.6 | -37.3 |
| 8 | +0.0 | -12.1 | -12.1 |

Driven orders: rms **15.2 dB**, mean -10.0 dB, worst -37.3 dB on order 7.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -15.4 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00408 s over 2.52 m` | 61.2 | not found | — | — | — |  |
| block, first bending mode | `mass law on 235 kg` | 70.0 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 600 x 0.990/(4 x (1.400 + 0.020))` | 104.5 | 102.5 | -2.0 % | -39.8 | 16.1 | tailpipe length 1.400 m, mouth radius 0.033 m → 1.448 m |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 4.5 L` | 173.8 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00408 s over 2.52 m` | 183.7 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00408 s over 2.52 m` | 306.1 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.260 + 0.018))` | 312.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 692 x 0.996/(4 x 0.468)` | 367.9 | 371.1 | +0.8 % | -50.6 | 14.4 | primary length 0.450 m → 0.446 m |
| expansion chamber, first pass band | `nc/2L = 1 x 658/(2 x 0.550)` | 597.7 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 692 x 0.996/(4 x 0.468)` | 1103.8 | 1111.7 | +0.7 % | -62.4 | 16.3 | primary length 0.450 m → 0.447 m |
| expansion chamber, second pass band | `nc/2L = 2 x 658/(2 x 0.550)` | 1195.5 | not found | — | — | — |  |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 102.5 | -39.8 | 16.1 |
| 371.1 | -50.6 | 14.4 |
| 885.0 | -61.4 | 11.7 |
| 1111.7 | -62.4 | 16.3 |
| 1534.7 | -75.9 | 13.8 |
| 3544.0 | -72.1 | 20.5 |
| 4137.6 | -70.7 | 23.0 |
| 4709.9 | -72.1 | 15.1 |
| 5731.5 | -92.2 | 11.7 |
| 7088.1 | -88.7 | 10.6 |
| 7673.4 | -87.9 | 16.3 |
| 9232.4 | -85.3 | 22.2 |
| 9703.1 | -85.0 | 26.6 |
| 12787.3 | -111.6 | 13.1 |
| 13328.6 | -110.2 | 14.5 |
| 13916.6 | -109.6 | 16.2 |
| 14461.2 | -112.1 | 12.9 |
| 15121.9 | -117.7 | 14.5 |
| 17519.6 | -116.0 | 17.1 |
| 18072.0 | -116.4 | 15.2 |
| 19571.3 | -118.3 | 12.4 |
| 20130.8 | -119.1 | 15.4 |
| 23068.1 | -129.7 | 11.4 |
| 23640.2 | -134.9 | 10.3 |

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
| 4.5 | null | -29.1 | — |
| 5 | null | -24.9 | — |
| 5.5 | null | — | — |
| 6 | +0.0 | -16.3 | -16.3 |

Driven orders: rms **11.5 dB**, mean -8.2 dB, worst -16.3 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -20.0 dB on order 0.5 (pass).

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
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 693 x 0.997/(4 x 0.497)` | 1041.9 | 1025.6 | -1.6 % | -54.3 | 18.8 | primary length 0.480 m → 0.488 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 147.6 | -35.7 | 20.2 |
| 1025.6 | -54.3 | 18.8 |
| 1407.8 | -71.7 | 13.4 |
| 1680.3 | -72.8 | 17.1 |
| 2826.6 | -66.5 | 20.2 |
| 4010.8 | -82.2 | 16.7 |
| 5519.8 | -81.8 | 18.6 |
| 6960.2 | -96.4 | 15.2 |
| 7440.6 | -102.9 | 15.4 |
| 8400.2 | -104.8 | 14.5 |
| 8989.4 | -107.9 | 12.9 |
| 9467.0 | -105.8 | 17.5 |
| 11864.0 | -111.1 | 15.0 |
| 12352.5 | -111.7 | 16.1 |
| 12942.4 | -110.1 | 21.4 |
| 14295.1 | -115.4 | 14.9 |
| 14640.5 | -116.7 | 15.5 |
| 15360.0 | -112.7 | 25.9 |
| 15854.1 | -114.9 | 13.9 |
| 16423.0 | -113.6 | 18.7 |
| 16982.5 | -116.5 | 15.7 |
| 17381.1 | -118.3 | 14.0 |
| 19804.6 | -117.1 | 16.7 |
| 21257.9 | -120.6 | 14.6 |

Noise floor tilt over 200 Hz-12 kHz: **-9.1 dB/octave**.


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
| 4 | +0.0 | -17.3 | -17.3 |

Driven orders: rms **12.2 dB**, mean -8.6 dB, worst -17.3 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -8.6 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 46.4 | not found | — | — | — |  |
| block, first bending mode | `mass law on 190 kg` | 77.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 511 x 0.995/(4 x (1.300 + 0.017))` | 96.5 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 139.1 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.8 L` | 153.7 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 231.8 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.016))` | 367.1 | 409.2 | +11.5 % | -51.4 | 22.5 | runner length 0.220 m → 0.212 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 595 x 0.998/(4 x 0.315)` | 471.7 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 570/(2 x 0.550)` | 518.2 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 551/(2 x 0.500)` | 550.6 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 551/(2 x 0.500)` | 1101.1 | 1095.6 | -0.5 % | -67.7 | 12.7 | chamber length 0.500 m, volume 22.4 L → 0.503 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 595 x 0.998/(4 x 0.315)` | 1415.1 | 1641.5 | +16.0 % | -77.7 | 15.1 | primary length 0.300 m → 0.259 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 409.2 | -51.4 | 22.5 |
| 847.8 | -56.9 | 16.4 |
| 1095.6 | -67.7 | 12.7 |
| 1641.5 | -77.7 | 15.1 |
| 2037.2 | -74.8 | 17.3 |
| 2977.5 | -73.3 | 15.3 |
| 3592.4 | -68.6 | 23.8 |
| 4187.3 | -66.7 | 27.4 |
| 4746.7 | -68.4 | 17.2 |
| 7186.8 | -81.8 | 25.6 |
| 7768.1 | -83.6 | 12.9 |
| 9170.2 | -80.5 | 27.4 |
| 9824.7 | -79.3 | 32.9 |
| 10350.7 | -79.9 | 13.9 |
| 12786.6 | -105.2 | 17.6 |
| 13382.8 | -103.4 | 23.5 |
| 13982.3 | -104.0 | 18.6 |
| 14518.2 | -107.4 | 21.0 |
| 16899.9 | -110.8 | 17.8 |
| 18077.4 | -112.0 | 18.5 |
| 19577.0 | -114.4 | 13.3 |
| 20176.8 | -113.8 | 16.3 |
| 23074.7 | -126.9 | 13.1 |
| 23670.8 | -132.8 | 14.9 |

Noise floor tilt over 200 Hz-12 kHz: **-4.2 dB/octave**.


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
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 735 x 1.000/(4 x (0.350 + 0.015))` | 503.6 | 479.1 | -4.9 % | -42.3 | 13.6 | tailpipe length 0.350 m, mouth radius 0.024 m → 0.383 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 777.4 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 808 x 1.000/(4 x 0.637)` | 951.2 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 769/(2 x 0.360)` | 1067.6 | 1138.5 | +6.6 % | -54.3 | 11.2 | silencer length 0.360 m → 0.338 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 1295.6 | 1197.6 | -7.6 % | -52.4 | 16.9 | downstream run 0.72 m → 0.784 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 116.2 | -34.3 | 18.0 |
| 299.6 | -42.3 | 11.8 |
| 359.5 | -39.5 | 15.2 |
| 419.4 | -39.7 | 14.2 |
| 479.1 | -42.3 | 13.6 |
| 1138.5 | -54.3 | 11.2 |
| 1197.6 | -52.4 | 16.9 |
| 1916.1 | -57.8 | 13.1 |
| 1974.7 | -57.0 | 11.3 |
| 2035.4 | -55.8 | 18.8 |
| 2096.0 | -56.5 | 11.4 |
| 2874.7 | -58.1 | 14.4 |
| 5278.7 | -72.2 | 12.7 |
| 7199.6 | -84.5 | 10.9 |
| 7969.8 | -81.8 | 12.9 |
| 8584.1 | -81.6 | 19.2 |
| 11355.6 | -82.9 | 16.7 |
| 15699.9 | -93.5 | 14.1 |
| 16520.6 | -98.8 | 13.6 |
| 18186.4 | -94.8 | 20.7 |
| 18960.8 | -99.7 | 11.5 |
| 21471.4 | -104.1 | 11.6 |
| 22184.4 | -104.1 | 12.1 |
| 22540.1 | -103.6 | 15.7 |

Noise floor tilt over 200 Hz-12 kHz: **-7.4 dB/octave**.

