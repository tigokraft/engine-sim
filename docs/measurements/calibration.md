# Calibration

Stage 16 of [the implementation plan](../IMPLEMENTATION_PLAN.md): the synth measured against a reference it did not produce. Regenerate with:

```bash
cargo run --release --example calibrate -- --markdown docs/measurements/calibration.md
```

Every figure below is *relative*: an order against the firing order, a measured frequency against a predicted one, a slope against an octave. That is deliberate and it is the point of the stage. A gain constant anywhere in the chain moves every order by the same number of decibels and cancels out of all of it, so nothing here can be closed by turning something up — only by changing a length, a volume or a radius.

| Engine | Firing order | Order balance rms | Worst order | Crank nulls | Modes placed | Floor tilt |
|---|---:|---:|---:|---:|---:|---:|
| [Inline-4](#inline-4) | 2 | 13.3 dB | -18.8 dB on 4 | -18.7 dB on 1 | 2/2 of 11 | -9.2 dB/oct |
| [Cross-plane V8](#cross-plane-v8) | 4 | 8.2 dB | -15.2 dB on 7.5 | -8.7 dB on 1, not enforced | 1/4 of 11 | -10.6 dB/oct |
| [Flat-plane V8](#flat-plane-v8) | 4 | 19.7 dB | -27.7 dB on 6 | -13.8 dB on 0.5, not enforced | 3/4 of 8 | -9.7 dB/oct |
| [V10](#v10) | 5 | 16.5 dB | -29.0 dB on 8.5 | -5.3 dB on 0.5, not enforced | 2/3 of 10 | -8.2 dB/oct |
| [V12](#v12) | 6 | 12.4 dB | -18.7 dB on 12 | -11.5 dB on 0.5, not enforced | 3/6 of 10 | -14.3 dB/oct |
| [2-Rotor Wankel](#2-rotor-wankel) | 2 | 10.4 dB | -14.7 dB on 4 | -24.5 dB on 0.5 | 3/3 of 9 | -10.2 dB/oct |
| [Turbo Inline-4](#turbo-inline-4) | 2 | 2.7 dB | -3.8 dB on 4 | -12.4 dB on 1, not enforced | 2/5 of 11 | -8.1 dB/oct |
| [Twin-turbo V8](#twin-turbo-v8) | 4 | 10.7 dB | -22.9 dB on 7.5 | -14.9 dB on 1, not enforced | 4/7 of 11 | -8.7 dB/oct |
| [Turbo Inline-6](#turbo-inline-6) | 3 | 11.8 dB | -16.6 dB on 6 | -22.8 dB on 1, not enforced | 3/3 of 10 | -9.8 dB/oct |
| [Turbodiesel I4](#turbodiesel-i4) | 2 | 11.9 dB | -16.8 dB on 4 | -9.2 dB on 1, not enforced | 1/4 of 12 | -5.0 dB/oct |
| [Big Single](#big-single) | 0.5 | 7.3 dB | +10.3 dB on 1 | — | 6/6 of 9 | -7.5 dB/oct |

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

7. **A shocked front is steepened but never dissipated.** The primaries now propagate on the gas dynamics rather than on linear acoustics: each point of the stored waveform is a characteristic travelling at `c(1 + (gamma+1)/(2 gamma) * p / P_0)`, and the pipe reads out the one that arrives first, so where a crest has overtaken the trough ahead of it the output steps rather than rises. Measured on a half-metre primary at 800 K, second harmonic against fundamental: 0.0005 at 100 Pa, 0.024 at 5 kPa, 0.095 at 20 kPa, 0.32 at one atmosphere — an advance that grows with the pulse, which is what the old `factor * p` clamped to two samples never did. What is still missing is the other half of a shock. A real one loses energy across the jump, and that loss is the reason a blowdown does not stay a step all the way down the pipe; here the front is bounded only by the rule that the reader cannot return more than the line was given, and then attenuated by the wall loss, which is a viscothermal term and knows nothing about entropy. So a primary carrying a pressure ratio well over two holds its edge further down the pipe than it should. The Rankine-Hugoniot jump gives the missing term without a constant; fitting a decay to the edge instead would be the tone knob this stage exists to refuse. Open.


## Inline-4

> Even 180 deg firing on one bank: a hard, plain four-cylinder bark.

2.0 L  4 cyl  11.5:1  ·  firing order 2  ·  850-7397 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -24.6 | — |
| 1 | null | -18.7 | — |
| 1.5 | null | -39.7 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -38.1 | — |
| 3 | null | -36.1 | — |
| 3.5 | null | -42.3 | — |
| 4 | +0.0 | -18.8 | -18.8 |

Driven orders: rms **13.3 dB**, mean -9.4 dB, worst -18.8 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -18.7 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00330 s over 2.12 m` | 75.7 | not found | — | — | — |  |
| block, first bending mode | `mass law on 110 kg` | 102.3 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 622 x 0.951/(4 x (1.200 + 0.017))` | 121.6 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.2 L` | 162.1 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00330 s over 2.12 m` | 227.2 | 216.4 | -4.8 % | -37.0 | 20.1 | downstream run 2.12 m → 2.223 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.280 + 0.017))` | 292.0 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00330 s over 2.12 m` | 378.7 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 724 x 0.983/(4 x 0.416)` | 428.2 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 688/(2 x 0.450)` | 764.0 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 724 x 0.983/(4 x 0.416)` | 1284.7 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 688/(2 x 0.450)` | 1528.0 | 1483.3 | -2.9 % | -68.5 | 21.3 | chamber length 0.450 m, volume 9.3 L → 0.464 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 34.8 | -40.9 | 18.1 |
| 216.4 | -37.0 | 20.1 |
| 958.6 | -59.7 | 12.2 |
| 1483.3 | -68.5 | 21.3 |
| 2160.9 | -78.0 | 9.5 |
| 2519.8 | -86.2 | 8.1 |
| 2878.2 | -84.3 | 13.3 |
| 3308.4 | -86.9 | 8.3 |
| 3598.1 | -81.2 | 18.0 |
| 3824.9 | -85.7 | 8.9 |
| 4065.7 | -86.0 | 8.9 |
| 4548.6 | -84.6 | 10.8 |
| 5593.9 | -87.0 | 8.8 |
| 6345.2 | -88.0 | 14.7 |
| 6614.3 | -89.6 | 8.9 |
| 7439.0 | -94.6 | 9.0 |
| 8383.9 | -101.8 | 9.3 |
| 9042.5 | -107.3 | 8.3 |
| 11279.9 | -103.3 | 15.9 |
| 12000.3 | -104.4 | 9.9 |
| 14776.8 | -117.2 | 8.9 |
| 15175.7 | -117.3 | 11.7 |
| 17864.6 | -110.2 | 21.2 |
| 22278.2 | -121.2 | 8.3 |

Noise floor tilt over 200 Hz-12 kHz: **-9.2 dB/octave**.


## Cross-plane V8

> 90-180-270-180 gaps on each bank: the offbeat American burble.

5.0 L  8 cyl  11.0:1  ·  firing order 4  ·  750-6997 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -12.0 | -0.7 |
| 1 | null | -8.7 | — |
| 1.5 | -3.7 | -11.6 | -7.9 |
| 2 | null | -20.3 | — |
| 2.5 | -3.7 | +1.7 | +5.4 |
| 3 | null | -17.5 | — |
| 3.5 | -11.4 | -10.6 | +0.7 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -25.6 | -14.3 |
| 5 | null | -27.0 | — |
| 5.5 | -3.7 | -9.3 | -5.6 |
| 6 | null | -19.0 | — |
| 6.5 | -3.7 | -7.4 | -3.7 |
| 7 | null | -28.5 | — |
| 7.5 | -11.4 | -26.5 | -15.2 |
| 8 | +0.0 | -10.2 | -10.2 |

Driven orders: rms **8.2 dB**, mean -5.1 dB, worst -15.2 dB on order 7.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -8.7 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00445 s over 2.82 m` | 56.1 | not found | — | — | — |  |
| block, first bending mode | `mass law on 210 kg` | 74.1 | 84.3 | +13.8 % | -40.8 | 16.6 |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 611 x 0.986/(4 x (1.500 + 0.018))` | 99.1 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 4.8 L` | 140.3 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00445 s over 2.82 m` | 168.4 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.380 + 0.018))` | 218.0 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00445 s over 2.82 m` | 280.6 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 722 x 0.996/(4 x 0.568)` | 316.4 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 680/(2 x 0.650)` | 523.1 | 409.2 | -21.8 % | -53.8 | 11.0 | chamber length 0.650 m, volume 22.1 L → 0.831 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 722 x 0.996/(4 x 0.568)` | 949.1 | 963.5 | +1.5 % | -61.9 | 11.4 | primary length 0.550 m → 0.542 m |
| expansion chamber, second pass band | `nc/2L = 2 x 680/(2 x 0.650)` | 1046.3 | 1199.9 | +14.7 % | -64.3 | 11.6 | chamber length 0.650 m, volume 22.1 L → 0.567 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 84.3 | -40.8 | 16.6 |
| 409.2 | -53.8 | 11.0 |
| 963.5 | -61.9 | 11.4 |
| 1199.9 | -64.3 | 11.6 |
| 1535.9 | -71.4 | 19.8 |
| 2679.6 | -79.5 | 10.5 |
| 2996.2 | -80.0 | 11.3 |
| 3900.6 | -80.2 | 17.9 |
| 4471.4 | -85.7 | 12.5 |
| 4799.6 | -89.8 | 14.3 |
| 5114.8 | -94.6 | 9.7 |
| 5712.1 | -93.2 | 15.3 |
| 8449.0 | -101.0 | 14.3 |
| 8999.5 | -102.8 | 11.0 |
| 9600.0 | -103.2 | 13.3 |
| 11724.7 | -112.6 | 11.5 |
| 11999.9 | -115.3 | 10.0 |
| 12281.9 | -110.9 | 10.0 |
| 13199.6 | -109.0 | 19.4 |
| 15600.0 | -117.5 | 9.6 |
| 16800.1 | -114.4 | 16.2 |
| 18637.4 | -123.8 | 10.5 |
| 21359.6 | -118.8 | 17.6 |
| 21599.6 | -120.5 | 9.7 |

Noise floor tilt over 200 Hz-12 kHz: **-10.6 dB/octave**.


## Flat-plane V8

> Even 180 deg on both banks: two inline-fours sharing a crank.

4.5 L  8 cyl  12.5:1  ·  firing order 4  ·  900-8596 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -13.8 | — |
| 1 | null | -16.7 | — |
| 1.5 | null | -21.9 | — |
| 2 | +0.0 | -24.8 | -24.8 |
| 2.5 | null | -28.6 | — |
| 3 | null | -26.3 | — |
| 3.5 | null | -28.1 | — |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | null | -34.1 | — |
| 5 | null | -26.7 | — |
| 5.5 | null | -30.2 | — |
| 6 | +0.0 | -27.7 | -27.7 |
| 6.5 | null | -34.2 | — |
| 7 | null | -36.3 | — |
| 7.5 | null | -48.3 | — |
| 8 | +0.0 | -12.7 | -12.7 |

Driven orders: rms **19.7 dB**, mean -16.3 dB, worst -27.7 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -13.8 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 180 kg` | 80.0 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 649 x 0.985/(4 x (0.900 + 0.020))` | 173.7 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00142 s over 0.92 m` | 176.4 | 209.4 | +18.7 % | -39.0 | 14.7 | downstream run 0.92 m → 0.775 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 710 x 0.993/(4 x 0.437)` | 403.2 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.015))` | 445.8 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00142 s over 0.92 m` | 529.3 | 527.6 | -0.3 % | -46.2 | 14.4 | downstream run 0.92 m → 0.923 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00142 s over 0.92 m` | 882.1 | 943.8 | +7.0 % | -56.9 | 17.7 | downstream run 0.92 m → 0.860 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 710 x 0.993/(4 x 0.437)` | 1209.5 | 1253.3 | +3.6 % | -67.7 | 8.4 | primary length 0.420 m → 0.405 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 209.4 | -39.0 | 14.7 |
| 527.6 | -46.2 | 14.4 |
| 943.8 | -56.9 | 17.7 |
| 1253.3 | -67.7 | 8.4 |
| 1415.8 | -68.0 | 16.0 |
| 1592.7 | -71.7 | 11.2 |
| 2160.2 | -75.2 | 10.9 |
| 2880.1 | -82.2 | 8.2 |
| 4079.2 | -88.4 | 8.8 |
| 4799.8 | -89.5 | 15.3 |
| 5520.0 | -92.1 | 9.4 |
| 6959.9 | -93.9 | 18.8 |
| 8160.0 | -103.9 | 10.1 |
| 9338.9 | -100.4 | 13.1 |
| 10799.8 | -109.2 | 8.1 |
| 12000.2 | -107.0 | 9.4 |
| 13440.1 | -112.1 | 12.9 |
| 15599.8 | -120.4 | 8.1 |
| 16079.5 | -113.8 | 13.9 |
| 17760.0 | -118.3 | 7.8 |
| 18000.2 | -120.5 | 8.1 |
| 18720.1 | -116.5 | 12.4 |
| 21360.2 | -118.2 | 12.5 |
| 23279.4 | -131.0 | 11.4 |

Noise floor tilt over 200 Hz-12 kHz: **-9.7 dB/octave**.


## V10

> 72 deg firing, unevenly split across the banks: metallic and hard.

5.2 L  10 cyl  12.7:1  ·  firing order 5  ·  900-8496 rpm over 8.0 s

Reference: the crank itself: 10 firings a cycle on 2 banks

### Order balance [dB relative to order 5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -22.3 | -5.3 | — |
| 1 | -8.4 | -10.2 | -1.8 |
| 1.5 | -5.6 | -15.5 | -9.9 |
| 2 | -12.6 | -19.8 | — |
| 2.5 | -14.0 | -22.1 | — |
| 3 | -12.6 | -16.2 | — |
| 3.5 | -5.6 | -13.5 | -7.9 |
| 4 | -8.4 | -17.0 | -8.6 |
| 4.5 | -22.3 | -34.1 | — |
| **5** | +0.0 | +0.0 | +0.0 |
| 5.5 | -22.3 | -32.2 | — |
| 6 | -8.4 | -25.4 | -17.0 |
| 6.5 | -5.6 | -26.7 | -21.1 |
| 7 | -12.6 | -25.6 | — |
| 7.5 | -14.0 | -28.5 | — |
| 8 | -12.6 | -18.5 | — |
| 8.5 | -5.6 | -34.6 | -29.0 |
| 9 | -8.4 | -35.0 | -26.6 |
| 9.5 | -22.3 | -38.1 | — |
| 10 | +0.0 | -13.6 | -13.6 |

Driven orders: rms **16.5 dB**, mean -13.6 dB, worst -29.0 dB on order 8.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -5.3 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 220 kg` | 72.4 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 82.6 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 623 x 0.982/(4 x (1.100 + 0.019))` | 136.6 | 117.9 | -13.7 % | -41.5 | 12.6 | tailpipe length 1.100 m, mouth radius 0.031 m → 1.297 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 247.9 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.019))` | 363.3 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 413.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 687 x 0.995/(4 x 0.376)` | 454.1 | 498.5 | +9.8 % | -52.4 | 11.2 | primary length 0.360 m → 0.328 m |
| expansion chamber, first pass band | `nc/2L = 1 x 662/(2 x 0.400)` | 828.1 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 687 x 0.995/(4 x 0.376)` | 1362.2 | 1321.3 | -3.0 % | -63.2 | 14.8 | primary length 0.360 m → 0.371 m |
| expansion chamber, second pass band | `nc/2L = 2 x 662/(2 x 0.400)` | 1656.2 | not found | — | — | — |  |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 117.9 | -41.5 | 12.6 |
| 498.5 | -52.4 | 11.2 |
| 1036.3 | -64.5 | 9.6 |
| 1321.3 | -63.2 | 14.8 |
| 2159.4 | -61.6 | 16.0 |
| 2400.2 | -73.3 | 13.0 |
| 3119.9 | -76.9 | 13.3 |
| 4376.7 | -75.6 | 15.9 |
| 6333.8 | -93.1 | 9.9 |
| 7440.2 | -95.4 | 9.5 |
| 8400.1 | -104.0 | 9.8 |
| 10320.0 | -102.5 | 11.4 |
| 10800.0 | -109.1 | 8.3 |
| 11039.9 | -107.4 | 11.2 |
| 12960.2 | -112.5 | 8.8 |
| 13200.0 | -109.3 | 12.7 |
| 13921.0 | -110.1 | 8.7 |
| 16079.8 | -113.4 | 13.2 |
| 17760.1 | -120.2 | 8.5 |
| 18240.2 | -116.6 | 13.6 |
| 18960.1 | -115.2 | 8.1 |
| 20400.1 | -121.4 | 10.8 |
| 21120.0 | -116.9 | 14.1 |
| 23279.9 | -128.9 | 23.5 |

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
| 2 | null | -21.6 | — |
| 2.5 | null | -31.4 | — |
| 3 | +0.0 | +11.4 | +11.4 |
| 3.5 | null | -19.2 | — |
| 4 | null | -15.1 | — |
| 4.5 | null | -26.4 | — |
| 5 | null | -26.7 | — |
| 5.5 | null | — | — |
| **6** | +0.0 | +0.0 | +0.0 |
| 6.5 | null | -43.4 | — |
| 7 | null | -32.2 | — |
| 7.5 | null | -34.4 | — |
| 8 | null | -31.6 | — |
| 8.5 | null | -40.5 | — |
| 9 | +0.0 | -11.8 | -11.8 |
| 9.5 | null | -44.6 | — |
| 10 | null | -37.1 | — |
| 10.5 | null | -42.5 | — |
| 11 | null | -39.9 | — |
| 11.5 | null | -33.3 | — |
| 12 | +0.0 | -18.7 | -18.7 |

Driven orders: rms **12.4 dB**, mean -4.8 dB, worst -18.7 dB on order 12 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -11.5 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 260 kg` | 66.6 | 64.8 | -2.7 % | -47.4 | 7.0 |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 119.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 648 x 0.946/(4 x (1.000 + 0.017))` | 150.7 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 359.6 | 274.9 | -23.5 % | -40.2 | 14.6 | downstream run 1.37 m → 1.788 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.140 + 0.017))` | 552.0 | 480.6 | -12.9 % | -54.5 | 8.1 | runner length 0.140 m → 0.181 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 714 x 0.989/(4 x 0.314)` | 561.6 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 599.3 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 678/(2 x 0.350)` | 969.1 | 1187.9 | +22.6 % | -60.3 | 18.1 | chamber length 0.350 m, volume 2.5 L → 0.286 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 714 x 0.989/(4 x 0.314)` | 1684.7 | 1553.1 | -7.8 % | -72.4 | 18.7 | primary length 0.300 m → 0.325 m |
| expansion chamber, second pass band | `nc/2L = 2 x 678/(2 x 0.350)` | 1938.2 | 1889.7 | -2.5 % | -77.2 | 11.8 | chamber length 0.350 m, volume 2.5 L → 0.359 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 64.8 | -47.4 | 7.0 |
| 274.9 | -40.2 | 14.6 |
| 480.6 | -54.5 | 8.1 |
| 1187.9 | -60.3 | 18.1 |
| 1553.1 | -72.4 | 18.7 |
| 1889.7 | -77.2 | 11.8 |
| 2085.9 | -79.7 | 7.6 |
| 2436.5 | -85.0 | 8.3 |
| 2931.5 | -88.7 | 6.0 |
| 3599.9 | -87.6 | 10.2 |
| 4080.3 | -92.1 | 5.4 |
| 4799.7 | -92.9 | 11.3 |
| 5279.9 | -102.5 | 7.4 |
| 7113.1 | -95.9 | 11.9 |
| 7920.8 | -107.4 | 5.6 |
| 8160.1 | -104.7 | 7.4 |
| 9170.8 | -100.7 | 11.1 |
| 13200.5 | -111.2 | 17.0 |
| 16323.4 | -114.6 | 9.3 |
| 17519.8 | -119.2 | 7.2 |
| 17760.2 | -120.7 | 6.1 |
| 18000.0 | -123.9 | 5.7 |
| 18959.7 | -119.2 | 9.1 |
| 21839.8 | -123.3 | 9.1 |

Noise floor tilt over 200 Hz-12 kHz: **-14.3 dB/octave**.


## 2-Rotor Wankel

> Four firings per cycle from two rotors: no valvetrain, no beat, just buzz.

2.6 L  4 cyl  10.0:1  ·  firing order 2  ·  950-8796 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -24.5 | — |
| 1 | null | -24.8 | — |
| 1.5 | null | -34.3 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -35.6 | — |
| 3 | null | -33.5 | — |
| 3.5 | null | -64.9 | — |
| 4 | +0.0 | -14.7 | -14.7 |

Driven orders: rms **10.4 dB**, mean -7.4 dB, worst -14.7 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -24.5 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00232 s over 1.52 m` | 107.8 | not found | — | — | — |  |
| block, first bending mode | `mass law on 95 kg` | 110.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 646 x 0.949/(4 x (1.000 + 0.018))` | 150.5 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 706 x 0.991/(4 x 0.620)` | 282.3 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00232 s over 1.52 m` | 323.5 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.200 + 0.016))` | 401.9 | 385.3 | -4.1 % | -50.6 | 8.1 | runner length 0.200 m → 0.225 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00232 s over 1.52 m` | 539.2 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 674/(2 x 0.500)` | 674.0 | 640.4 | -5.0 % | -49.4 | 12.9 | silencer length 0.500 m → 0.526 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 706 x 0.991/(4 x 0.620)` | 847.0 | 800.1 | -5.5 % | -50.5 | 9.1 | primary length 0.600 m → 0.635 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 53.1 | -31.1 | 25.2 |
| 208.7 | -31.7 | 12.7 |
| 385.3 | -50.6 | 8.1 |
| 640.4 | -49.4 | 12.9 |
| 800.1 | -50.5 | 9.1 |
| 960.5 | -54.0 | 8.0 |
| 1145.2 | -50.1 | 14.5 |
| 1566.8 | -65.9 | 6.2 |
| 1826.4 | -63.7 | 15.5 |
| 2772.4 | -75.0 | 12.7 |
| 3449.7 | -77.1 | 12.5 |
| 4079.9 | -81.8 | 9.0 |
| 4767.3 | -84.7 | 7.0 |
| 5389.9 | -87.4 | 7.7 |
| 6480.5 | -89.2 | 8.5 |
| 8160.1 | -103.7 | 8.8 |
| 9119.5 | -102.4 | 8.8 |
| 9600.0 | -101.2 | 13.0 |
| 10559.6 | -101.9 | 6.1 |
| 13200.5 | -108.3 | 5.9 |
| 13679.9 | -106.4 | 9.5 |
| 14399.9 | -111.2 | 6.7 |
| 16799.8 | -109.0 | 11.6 |
| 20879.9 | -112.0 | 14.0 |

Noise floor tilt over 200 Hz-12 kHz: **-10.2 dB/octave**.


## Turbo Inline-4

> Even 180 deg firing under a small fast single: bark, whistle, flutter.

2.0 L  4 cyl  9.6:1  ·  firing order 2  ·  820-6897 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -17.1 | — |
| 1 | null | -12.4 | — |
| 1.5 | null | -26.9 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -28.2 | — |
| 3 | null | -21.3 | — |
| 3.5 | null | -49.5 | — |
| 4 | +0.0 | -3.8 | -3.8 |

Driven orders: rms **2.7 dB**, mean -1.9 dB, worst -3.8 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -12.4 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 125 kg` | 96.0 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 101.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 643 x 0.996/(4 x (1.200 + 0.018))` | 131.4 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.5 L` | 163.7 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 303.4 | 250.7 | -17.4 % | -41.2 | 9.3 | downstream run 1.62 m → 1.958 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.240 + 0.017))` | 337.4 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 505.7 | 479.4 | -5.2 % | -58.2 | 8.0 | downstream run 1.62 m → 1.707 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 748 x 0.998/(4 x 0.366)` | 510.5 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 692/(2 x 0.400)` | 865.6 | 719.0 | -16.9 % | -57.8 | 11.5 | chamber length 0.400 m, volume 3.1 L → 0.482 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 748 x 0.998/(4 x 0.366)` | 1531.6 | 1149.5 | -24.9 % | -54.4 | 19.4 | primary length 0.350 m → 0.466 m |
| expansion chamber, second pass band | `nc/2L = 2 x 692/(2 x 0.400)` | 1731.2 | 1636.2 | -5.5 % | -63.2 | 25.8 | chamber length 0.400 m, volume 3.1 L → 0.423 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 50.4 | -33.0 | 25.1 |
| 250.7 | -41.2 | 9.3 |
| 479.4 | -58.2 | 8.0 |
| 719.0 | -57.8 | 11.5 |
| 1149.5 | -54.4 | 19.4 |
| 1636.2 | -63.2 | 25.8 |
| 2520.4 | -81.0 | 10.3 |
| 3585.7 | -75.4 | 11.5 |
| 4187.2 | -74.5 | 15.9 |
| 4736.5 | -76.1 | 11.8 |
| 6481.2 | -71.5 | 19.6 |
| 8376.6 | -97.5 | 9.8 |
| 8915.0 | -93.9 | 9.3 |
| 9473.8 | -88.0 | 8.3 |
| 10047.6 | -86.2 | 21.4 |
| 13127.1 | -85.9 | 24.0 |
| 14039.9 | -110.2 | 8.0 |
| 14639.5 | -113.0 | 10.9 |
| 14962.7 | -113.4 | 11.2 |
| 15720.4 | -111.6 | 10.2 |
| 17280.4 | -109.2 | 23.1 |
| 18721.2 | -112.4 | 8.9 |
| 21322.7 | -117.5 | 9.3 |
| 22088.7 | -117.7 | 10.4 |

Noise floor tilt over 200 Hz-12 kHz: **-8.1 dB/octave**.


## Twin-turbo V8

> Hot-vee twins over the 90-180-270-180 burble: offbeat, but muffled.

4.0 L  8 cyl  10.0:1  ·  firing order 4  ·  760-7097 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -14.9 | -3.5 |
| 1 | null | -14.9 | — |
| 1.5 | -3.7 | +0.4 | +4.1 |
| 2 | null | -21.4 | — |
| 2.5 | -3.7 | -1.6 | +2.1 |
| 3 | null | -33.6 | — |
| 3.5 | -11.4 | -14.9 | -3.5 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -31.4 | -20.1 |
| 5 | null | -27.8 | — |
| 5.5 | -3.7 | -14.7 | -11.0 |
| 6 | null | -35.6 | — |
| 6.5 | -3.7 | -7.1 | -3.4 |
| 7 | null | -30.7 | — |
| 7.5 | -11.4 | -34.2 | -22.9 |
| 8 | +0.0 | -7.1 | -7.1 |

Driven orders: rms **10.7 dB**, mean -6.5 dB, worst -22.9 dB on order 7.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -14.9 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00408 s over 2.52 m` | 61.2 | not found | — | — | — |  |
| block, first bending mode | `mass law on 235 kg` | 70.0 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 600 x 0.990/(4 x (1.400 + 0.020))` | 104.5 | 104.4 | -0.1 % | -35.8 | 21.5 | tailpipe length 1.400 m, mouth radius 0.033 m → 1.421 m |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 4.5 L` | 173.8 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00408 s over 2.52 m` | 183.7 | 207.3 | +12.9 % | -41.4 | 11.8 | downstream run 2.52 m → 2.233 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00408 s over 2.52 m` | 306.1 | 291.1 | -4.9 % | -48.1 | 8.9 | downstream run 2.52 m → 2.650 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.260 + 0.018))` | 312.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 692 x 0.996/(4 x 0.468)` | 367.9 | 370.5 | +0.7 % | -49.7 | 15.5 | primary length 0.450 m → 0.447 m |
| expansion chamber, first pass band | `nc/2L = 1 x 658/(2 x 0.550)` | 597.7 | 730.1 | +22.1 % | -54.3 | 12.1 | chamber length 0.550 m, volume 13.1 L → 0.450 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 692 x 0.996/(4 x 0.468)` | 1103.8 | 1111.9 | +0.7 % | -60.5 | 17.8 | primary length 0.450 m → 0.447 m |
| expansion chamber, second pass band | `nc/2L = 2 x 658/(2 x 0.550)` | 1195.5 | 910.5 | -23.8 % | -53.6 | 17.1 | chamber length 0.550 m, volume 13.1 L → 0.722 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 104.4 | -35.8 | 21.5 |
| 207.3 | -41.4 | 11.8 |
| 291.1 | -48.1 | 8.9 |
| 370.5 | -49.7 | 15.5 |
| 730.1 | -54.3 | 12.1 |
| 910.5 | -53.6 | 17.1 |
| 1111.9 | -60.5 | 17.8 |
| 1545.4 | -72.2 | 11.1 |
| 1786.9 | -71.1 | 19.3 |
| 2075.4 | -77.4 | 7.6 |
| 2495.2 | -78.9 | 6.9 |
| 2997.8 | -76.5 | 6.8 |
| 3615.2 | -73.4 | 13.5 |
| 4215.7 | -73.4 | 10.3 |
| 4766.0 | -76.6 | 11.9 |
| 5279.8 | -97.3 | 7.2 |
| 6667.7 | -89.7 | 14.5 |
| 7815.5 | -91.5 | 6.4 |
| 9478.2 | -88.6 | 19.5 |
| 11694.4 | -115.5 | 7.6 |
| 13200.0 | -112.1 | 15.5 |
| 13792.2 | -112.2 | 6.0 |
| 17279.3 | -116.4 | 12.2 |
| 20640.4 | -120.3 | 13.9 |

Noise floor tilt over 200 Hz-12 kHz: **-8.7 dB/octave**.


## Turbo Inline-6

> Even 120 deg firing, no gaps at all, and a big lazy single over it.

3.0 L  6 cyl  9.2:1  ·  firing order 3  ·  780-7197 rpm over 8.0 s

Reference: the crank itself: 6 firings a cycle on 1 bank

### Order balance [dB relative to order 3]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -23.0 | — |
| 1 | null | -22.8 | — |
| 1.5 | null | -24.5 | — |
| 2 | null | -26.7 | — |
| 2.5 | null | — | — |
| **3** | +0.0 | +0.0 | +0.0 |
| 3.5 | null | -35.8 | — |
| 4 | null | -33.1 | — |
| 4.5 | null | -32.0 | — |
| 5 | null | -28.6 | — |
| 5.5 | null | — | — |
| 6 | +0.0 | -16.6 | -16.6 |

Driven orders: rms **11.8 dB**, mean -8.3 dB, worst -16.6 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -22.8 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00361 s over 2.22 m` | 69.2 | not found | — | — | — |  |
| block, first bending mode | `mass law on 195 kg` | 76.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 604 x 0.991/(4 x (1.600 + 0.021))` | 92.2 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 3.5 L` | 159.4 | 148.6 | -6.8 % | -32.4 | 24.2 |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00361 s over 2.22 m` | 207.5 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.300 + 0.018))` | 272.9 | 269.1 | -1.4 % | -39.0 | 11.1 | runner length 0.300 m → 0.323 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00361 s over 2.22 m` | 345.9 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 693 x 0.997/(4 x 0.497)` | 347.3 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 645/(2 x 0.600)` | 537.8 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 693 x 0.997/(4 x 0.497)` | 1041.9 | 1025.0 | -1.6 % | -53.6 | 19.4 | primary length 0.480 m → 0.488 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 148.6 | -32.4 | 24.2 |
| 269.1 | -39.0 | 11.1 |
| 1025.0 | -53.6 | 19.4 |
| 1407.6 | -64.1 | 22.9 |
| 1727.9 | -69.2 | 13.4 |
| 2825.7 | -67.4 | 18.4 |
| 3513.9 | -84.2 | 9.9 |
| 4888.2 | -86.7 | 13.5 |
| 5555.7 | -83.3 | 17.4 |
| 5999.8 | -94.4 | 13.9 |
| 6960.3 | -100.9 | 10.1 |
| 7680.3 | -105.0 | 11.8 |
| 8399.9 | -109.6 | 12.3 |
| 9055.9 | -107.1 | 11.9 |
| 10799.8 | -105.0 | 26.9 |
| 11175.6 | -111.5 | 11.0 |
| 11460.0 | -111.0 | 12.0 |
| 13199.9 | -114.1 | 11.0 |
| 14640.1 | -116.3 | 17.9 |
| 15360.2 | -115.1 | 14.4 |
| 17759.8 | -122.2 | 12.1 |
| 20160.6 | -120.8 | 14.3 |
| 20879.5 | -123.4 | 11.8 |
| 21920.7 | -122.0 | 17.1 |

Noise floor tilt over 200 Hz-12 kHz: **-9.8 dB/octave**.


## Turbodiesel I4

> No spark at all: a premixed spike, an iron block, and clatter.

2.0 L  4 cyl  21.5:1  ·  firing order 2  ·  800-4998 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -13.4 | — |
| 1 | null | -9.2 | — |
| 1.5 | null | — | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | — | — |
| 3 | null | -25.6 | — |
| 3.5 | null | — | — |
| 4 | +0.0 | -16.8 | -16.8 |

Driven orders: rms **11.9 dB**, mean -8.4 dB, worst -16.8 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -9.2 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 46.4 | not found | — | — | — |  |
| block, first bending mode | `mass law on 190 kg` | 77.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 511 x 0.995/(4 x (1.300 + 0.017))` | 96.5 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 139.1 | 119.6 | -14.0 % | -45.9 | 12.0 | downstream run 2.87 m → 3.333 m |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.8 L` | 153.7 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 231.8 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.016))` | 367.1 | 408.9 | +11.4 % | -50.9 | 22.6 | runner length 0.220 m → 0.212 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 595 x 0.998/(4 x 0.315)` | 471.7 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 570/(2 x 0.550)` | 518.2 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 551/(2 x 0.500)` | 550.6 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 551/(2 x 0.500)` | 1101.1 | 1096.7 | -0.4 % | -67.2 | 10.4 | chamber length 0.500 m, volume 22.4 L → 0.502 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 595 x 0.998/(4 x 0.315)` | 1415.1 | 1598.7 | +13.0 % | -77.9 | 14.2 | primary length 0.300 m → 0.266 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 119.6 | -45.9 | 12.0 |
| 408.9 | -50.9 | 22.6 |
| 849.2 | -57.8 | 14.1 |
| 982.2 | -66.8 | 8.1 |
| 1096.7 | -67.2 | 10.4 |
| 1598.7 | -77.9 | 14.2 |
| 2036.6 | -76.2 | 15.7 |
| 3043.0 | -73.9 | 11.5 |
| 3592.5 | -69.9 | 14.4 |
| 4187.6 | -68.9 | 22.5 |
| 4747.6 | -72.9 | 16.4 |
| 5696.1 | -91.4 | 8.1 |
| 5824.8 | -90.4 | 11.1 |
| 5954.9 | -91.8 | 8.3 |
| 6219.1 | -88.3 | 8.0 |
| 7187.1 | -82.5 | 18.3 |
| 7768.6 | -86.9 | 9.3 |
| 8375.5 | -93.9 | 10.8 |
| 8846.8 | -88.7 | 8.9 |
| 10003.8 | -82.6 | 26.4 |
| 12109.0 | -112.2 | 9.3 |
| 13211.8 | -107.1 | 16.2 |
| 16899.6 | -112.2 | 24.3 |
| 20543.9 | -117.3 | 10.3 |

Noise floor tilt over 200 Hz-12 kHz: **-5.0 dB/octave**.


## Big Single

> One firing every two turns: the crank itself is the rhythm.

0.7 L  1 cyl  12.0:1  ·  firing order 0.5  ·  1250-7597 rpm over 8.0 s

Reference: the crank itself: 1 firings a cycle on 1 bank

### Order balance [dB relative to order 0.5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| **0.5** | +0.0 | +0.0 | +0.0 |
| 1 | +0.0 | +10.3 | +10.3 |

Driven orders: rms **7.3 dB**, mean +5.2 dB, worst +10.3 dB on order 1 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: none resolvable.

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 45 kg` | 120.0 | 115.6 | -3.7 % | -31.8 | 20.8 |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 259.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 808 x 1.000/(4 x 0.637)` | 317.1 | 299.5 | -5.5 % | -40.1 | 12.9 | primary length 0.620 m → 0.656 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.020))` | 434.6 | 422.0 | -2.9 % | -41.9 | 9.6 | runner length 0.180 m → 0.206 m |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 735 x 1.000/(4 x (0.350 + 0.015))` | 503.6 | 479.4 | -4.8 % | -40.5 | 13.1 | tailpipe length 0.350 m, mouth radius 0.024 m → 0.383 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 777.4 | 778.2 | +0.1 % | -42.0 | 9.9 | downstream run 0.72 m → 0.724 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 808 x 1.000/(4 x 0.637)` | 951.2 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 769/(2 x 0.360)` | 1067.6 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 1295.6 | 1197.4 | -7.6 % | -52.8 | 14.0 | downstream run 0.72 m → 0.784 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 115.6 | -31.8 | 20.8 |
| 299.5 | -40.1 | 12.9 |
| 358.4 | -43.3 | 8.1 |
| 422.0 | -41.9 | 9.6 |
| 479.4 | -40.5 | 13.1 |
| 538.6 | -40.7 | 8.7 |
| 778.2 | -42.0 | 9.9 |
| 1197.4 | -52.8 | 14.0 |
| 1916.1 | -57.4 | 9.0 |
| 2034.4 | -55.3 | 8.8 |
| 2096.0 | -54.3 | 17.6 |
| 2155.5 | -55.9 | 11.0 |
| 2874.7 | -56.9 | 9.9 |
| 4401.7 | -72.1 | 7.7 |
| 5277.4 | -71.3 | 14.4 |
| 7250.1 | -84.9 | 7.6 |
| 7851.5 | -82.5 | 8.2 |
| 8762.5 | -80.8 | 11.9 |
| 11092.8 | -85.4 | 10.6 |
| 13066.5 | -91.5 | 9.3 |
| 15597.4 | -94.3 | 13.1 |
| 18138.8 | -96.0 | 14.7 |
| 20668.1 | -98.8 | 13.1 |
| 22480.3 | -105.0 | 14.0 |

Noise floor tilt over 200 Hz-12 kHz: **-7.5 dB/octave**.

