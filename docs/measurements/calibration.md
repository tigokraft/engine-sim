# Calibration

Stage 16 of [the implementation plan](../IMPLEMENTATION_PLAN.md): the synth measured against a reference it did not produce. Regenerate with:

```bash
cargo run --release --example calibrate -- --markdown docs/measurements/calibration.md
```

Every figure below is *relative*: an order against the firing order, a measured frequency against a predicted one, a slope against an octave. That is deliberate and it is the point of the stage. A gain constant anywhere in the chain moves every order by the same number of decibels and cancels out of all of it, so nothing here can be closed by turning something up — only by changing a length, a volume or a radius.

| Engine | Firing order | Order balance rms | Worst order | Crank nulls | Modes placed | Floor tilt |
|---|---:|---:|---:|---:|---:|---:|
| [Inline-4](#inline-4) | 2 | 13.2 dB | -18.6 dB on 4 | -20.6 dB on 1 | 2/2 of 11 | -9.3 dB/oct |
| [Cross-plane V8](#cross-plane-v8) | 4 | 7.9 dB | -15.8 dB on 7.5 | -8.5 dB on 1, not enforced | 1/6 of 11 | -10.9 dB/oct |
| [Flat-plane V8](#flat-plane-v8) | 4 | 19.2 dB | -26.8 dB on 6 | -14.8 dB on 0.5, not enforced | 3/4 of 8 | -9.7 dB/oct |
| [V10](#v10) | 5 | 16.4 dB | -27.9 dB on 8.5 | -3.7 dB on 0.5, not enforced | 2/5 of 10 | -7.9 dB/oct |
| [V12](#v12) | 6 | 12.4 dB | -18.6 dB on 12 | -11.5 dB on 0.5, not enforced | 4/6 of 10 | -14.2 dB/oct |
| [2-Rotor Wankel](#2-rotor-wankel) | 2 | 10.5 dB | -14.8 dB on 4 | -24.2 dB on 0.5 | 3/3 of 9 | -10.1 dB/oct |
| [Turbo Inline-4](#turbo-inline-4) | 2 | 2.8 dB | -4.0 dB on 4 | -13.7 dB on 1, not enforced | 2/5 of 11 | -8.1 dB/oct |
| [Twin-turbo V8](#twin-turbo-v8) | 4 | 10.8 dB | -21.6 dB on 4.5 | -15.6 dB on 1, not enforced | 4/7 of 11 | -8.7 dB/oct |
| [Turbo Inline-6](#turbo-inline-6) | 3 | 11.9 dB | -16.9 dB on 6 | -21.7 dB on 1, not enforced | 3/3 of 10 | -9.7 dB/oct |
| [Turbodiesel I4](#turbodiesel-i4) | 2 | 11.9 dB | -16.8 dB on 4 | -7.1 dB on 1, not enforced | 1/4 of 12 | -5.0 dB/oct |
| [Big Single](#big-single) | 0.5 | 7.7 dB | +10.9 dB on 1 | — | 6/6 of 9 | -7.5 dB/oct |

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

7. **The primaries steepen correctly and are not allowed to.** `WaveguidePipe` no longer advances a crest by a factor and clamps it. Each point of the stored waveform is a characteristic travelling at `c(1 + (gamma+1)/(2 gamma) p/P_0)`, the pipe reads out the one that arrives first, and where a crest has overtaken the trough ahead of it the output steps rather than rises; the front then pays the shock's own dissipation, `sigma = (gamma+1)/(2 gamma) D |dp| / P_0` past one being the sawtooth decay of a jump the section can no longer steepen. Measured on a half-metre primary at 800 K, second harmonic against fundamental: 0.0005 at 100 Pa, 0.024 at 5 kPa, 0.095 at 20 kPa, 0.32 at one atmosphere. Excited once and left alone in a 90 per cent reflecting loop it decays faster than the linear pipe does, so it is not the transposed-form mistake in another costume.

It is switched off. Driven at the pressure a port actually launches — `c mdot / A`, 0.31 atmospheres at idle and 0.65 at the limiter across this catalogue, a third of the difference across the valve because a port is a restriction and not an open end — three of this repository's own guards fail. The band the primaries work in stops falling as they are stretched: 234.6, 259.8, 237.4 Hz over a half-again stretch, where it has to fall every step. The cam's grip on mid-band tilt falls from over ten decibels to five and a half. The limiter bounce stops standing out of a clean pull. The cause is open question 5: a primary's loop pays about 1.4 dB a round trip, so a pulse goes round it some thirty times and steepens on every one of them, and a nonlinearity accumulated thirty times is louder than the geometry it is supposed to be colouring. Four times the wall enhancement brings two of the three guards back, which is precisely the constant question 5 says has no derivation, so it is not taken. What this wants is either that derivation, or the nonlinearity applied to the launched pulse on its one-way run down the primary and not to the resonant field behind it — `beta` is proportional to `p` and the field is twenty decibels under the pulse, so steepening the one and not the other is a statement about where the gas is nonlinear rather than a knob. Open.

8. **The port jet's dipole efficiency is the one number in the exhaust path with a range instead of a derivation.** Curle's law says a flow past a solid boundary radiates as the sixth power of velocity and leaves the constant to measurement, so `JET_DIPOLE_EFFICIENCY` is measured and not derived. It is set at 0.005, which is *below* the published range for an orifice in a duct, and it is there because that is the loudest this catalogue's own guards allow: at 0.006 the mechanical floor stops measurably filling the gaps between firings, and at 0.02 the pulse stops reading as linear in the pressure difference that made it. Either the jet is genuinely this quiet in a runner — plausible, since a jet a fifth of the pipe's area couples into a plane wave badly — or the mechanical layer under it is too quiet and is masking how much room there is. The measurement that would settle it is the one this repository does not have: the tone-to-floor ratio of a recording of a real engine, which runs 5 to 12 dB a bin where this catalogue runs 11 to 19. Open.

9. **The blowers' loudness is derived, their level is not, and it cannot be until the solver makes boost.** A displacement blower's noise is its rotors handing pockets of gas to a discharge port, and a volume velocity across an aperture launches `c mdot / A` like every other aperture here, so both supercharger voices now scale with the induction mass flow the solver actually reports. That replaced a law that went as the square of crank speed times a throttle term, which had a blower spinning fast on a shut throttle at a third of full voice when a bypassed blower is pumping almost nothing. What did not change is the constant in front, and it cannot: nothing in the physics produces boost, so there is no pressure ratio across the machine for its loudness to be a fraction of, and there is no bypass to open when the throttle shuts. The audible consequence is open question 2's other half. Held at 4500 rpm wide open, the blown V8's loudest single order is the whine at 8.4, three decibels clear of the next thing, and its firing order is not in the loudest ten at all. A smooth tone locked to a fixed multiple of shaft speed and rising with it is the sound of an inverter, not of an engine. It closes with the compressor model in Stage 13, not with a number. Open.


## Inline-4

> Even 180 deg firing on one bank: a hard, plain four-cylinder bark.

2.0 L  4 cyl  11.5:1  ·  firing order 2  ·  850-7397 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -25.7 | — |
| 1 | null | -20.6 | — |
| 1.5 | null | -41.5 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -40.3 | — |
| 3 | null | -35.2 | — |
| 3.5 | null | -43.5 | — |
| 4 | +0.0 | -18.6 | -18.6 |

Driven orders: rms **13.2 dB**, mean -9.3 dB, worst -18.6 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -20.6 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00330 s over 2.12 m` | 75.7 | not found | — | — | — |  |
| block, first bending mode | `mass law on 110 kg` | 102.3 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 622 x 0.951/(4 x (1.200 + 0.017))` | 121.6 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.2 L` | 162.1 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00330 s over 2.12 m` | 227.2 | 216.3 | -4.8 % | -37.0 | 20.1 | downstream run 2.12 m → 2.223 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.280 + 0.017))` | 292.0 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00330 s over 2.12 m` | 378.7 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 724 x 0.983/(4 x 0.416)` | 428.2 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 688/(2 x 0.450)` | 764.0 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 724 x 0.983/(4 x 0.416)` | 1284.7 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 688/(2 x 0.450)` | 1528.0 | 1483.3 | -2.9 % | -68.5 | 20.6 | chamber length 0.450 m, volume 9.3 L → 0.464 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 35.5 | -42.5 | 16.5 |
| 216.3 | -37.0 | 20.1 |
| 958.9 | -59.8 | 12.2 |
| 1483.3 | -68.5 | 20.6 |
| 2160.6 | -78.0 | 9.9 |
| 2519.6 | -86.3 | 8.2 |
| 2878.4 | -84.4 | 12.9 |
| 3305.3 | -86.9 | 8.1 |
| 3597.9 | -81.3 | 17.2 |
| 3824.8 | -85.8 | 8.5 |
| 4065.3 | -86.1 | 8.3 |
| 4548.7 | -84.7 | 11.2 |
| 5593.7 | -87.1 | 8.8 |
| 6344.8 | -88.1 | 13.6 |
| 6614.4 | -89.6 | 8.5 |
| 7439.1 | -94.8 | 8.7 |
| 8384.1 | -101.6 | 9.1 |
| 9042.7 | -106.9 | 11.1 |
| 11280.1 | -103.2 | 16.1 |
| 12000.4 | -104.1 | 9.5 |
| 14783.6 | -116.9 | 9.3 |
| 15172.1 | -117.3 | 11.2 |
| 17865.4 | -109.7 | 21.4 |
| 22277.5 | -120.5 | 8.1 |

Noise floor tilt over 200 Hz-12 kHz: **-9.3 dB/octave**.


## Cross-plane V8

> 90-180-270-180 gaps on each bank: the offbeat American burble.

5.0 L  8 cyl  11.0:1  ·  firing order 4  ·  750-6997 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -11.5 | -0.1 |
| 1 | null | -8.5 | — |
| 1.5 | -3.7 | -10.0 | -6.3 |
| 2 | null | -18.6 | — |
| 2.5 | -3.7 | +1.4 | +5.1 |
| 3 | null | -17.5 | — |
| 3.5 | -11.4 | -10.7 | +0.7 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -24.1 | -12.8 |
| 5 | null | -28.9 | — |
| 5.5 | -3.7 | -8.9 | -5.2 |
| 6 | null | -19.4 | — |
| 6.5 | -3.7 | -7.7 | -4.0 |
| 7 | null | -27.2 | — |
| 7.5 | -11.4 | -27.1 | -15.8 |
| 8 | +0.0 | -10.4 | -10.4 |

Driven orders: rms **7.9 dB**, mean -4.9 dB, worst -15.8 dB on order 7.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -8.5 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00445 s over 2.82 m` | 56.1 | not found | — | — | — |  |
| block, first bending mode | `mass law on 210 kg` | 74.1 | 83.6 | +12.9 % | -40.5 | 17.0 |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 611 x 0.986/(4 x (1.500 + 0.018))` | 99.1 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 4.8 L` | 140.3 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00445 s over 2.82 m` | 168.4 | 185.2 | +10.0 % | -47.0 | 9.2 | downstream run 2.82 m → 2.561 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.380 + 0.018))` | 218.0 | 245.1 | +12.4 % | -48.6 | 9.2 | runner length 0.380 m → 0.354 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00445 s over 2.82 m` | 280.6 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 722 x 0.996/(4 x 0.568)` | 316.4 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 680/(2 x 0.650)` | 523.1 | 408.2 | -22.0 % | -53.6 | 11.3 | chamber length 0.650 m, volume 22.1 L → 0.833 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 722 x 0.996/(4 x 0.568)` | 949.1 | 905.7 | -4.6 % | -61.8 | 12.0 | primary length 0.550 m → 0.576 m |
| expansion chamber, second pass band | `nc/2L = 2 x 680/(2 x 0.650)` | 1046.3 | 1199.9 | +14.7 % | -64.0 | 10.2 | chamber length 0.650 m, volume 22.1 L → 0.567 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 83.6 | -40.5 | 17.0 |
| 185.2 | -47.0 | 9.2 |
| 245.1 | -48.6 | 9.2 |
| 408.2 | -53.6 | 11.3 |
| 905.7 | -61.8 | 12.0 |
| 1199.9 | -64.0 | 10.2 |
| 1536.0 | -71.3 | 18.3 |
| 2680.4 | -79.5 | 9.5 |
| 2995.6 | -79.9 | 11.7 |
| 3900.3 | -80.3 | 18.0 |
| 4471.7 | -85.6 | 12.9 |
| 4799.6 | -89.7 | 14.3 |
| 5139.0 | -94.1 | 9.3 |
| 5712.2 | -93.2 | 14.7 |
| 8449.2 | -100.7 | 12.8 |
| 8998.6 | -102.2 | 10.5 |
| 9600.0 | -102.9 | 12.1 |
| 11760.1 | -115.4 | 10.9 |
| 11999.9 | -114.4 | 10.0 |
| 12240.2 | -113.7 | 9.0 |
| 13199.7 | -108.9 | 17.9 |
| 16800.1 | -113.9 | 14.3 |
| 21359.8 | -118.7 | 15.0 |
| 22800.0 | -128.6 | 9.0 |

Noise floor tilt over 200 Hz-12 kHz: **-10.9 dB/octave**.


## Flat-plane V8

> Even 180 deg on both banks: two inline-fours sharing a crank.

4.5 L  8 cyl  12.5:1  ·  firing order 4  ·  900-8596 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -14.8 | — |
| 1 | null | -15.8 | — |
| 1.5 | null | -23.4 | — |
| 2 | +0.0 | -24.3 | -24.3 |
| 2.5 | null | -30.7 | — |
| 3 | null | -25.6 | — |
| 3.5 | null | -29.5 | — |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | null | -33.2 | — |
| 5 | null | -29.0 | — |
| 5.5 | null | -30.2 | — |
| 6 | +0.0 | -26.8 | -26.8 |
| 6.5 | null | -34.1 | — |
| 7 | null | -35.5 | — |
| 7.5 | null | -50.4 | — |
| 8 | +0.0 | -12.8 | -12.8 |

Driven orders: rms **19.2 dB**, mean -16.0 dB, worst -26.8 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -14.8 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 180 kg` | 80.0 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 649 x 0.985/(4 x (0.900 + 0.020))` | 173.7 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00142 s over 0.92 m` | 176.4 | 209.5 | +18.7 % | -39.1 | 14.7 | downstream run 0.92 m → 0.775 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 710 x 0.993/(4 x 0.437)` | 403.2 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.015))` | 445.8 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00142 s over 0.92 m` | 529.3 | 528.0 | -0.2 % | -46.3 | 14.0 | downstream run 0.92 m → 0.922 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00142 s over 0.92 m` | 882.1 | 943.8 | +7.0 % | -57.0 | 17.3 | downstream run 0.92 m → 0.860 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 710 x 0.993/(4 x 0.437)` | 1209.5 | 1253.5 | +3.6 % | -67.7 | 8.6 | primary length 0.420 m → 0.405 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 209.5 | -39.1 | 14.7 |
| 528.0 | -46.3 | 14.0 |
| 943.8 | -57.0 | 17.3 |
| 1253.5 | -67.7 | 8.6 |
| 1416.0 | -68.0 | 16.1 |
| 1596.8 | -71.7 | 11.5 |
| 1919.9 | -77.1 | 8.1 |
| 2160.1 | -75.2 | 11.4 |
| 2880.2 | -82.4 | 8.1 |
| 4319.7 | -87.1 | 15.9 |
| 4800.1 | -89.5 | 9.3 |
| 5520.1 | -91.6 | 10.0 |
| 6960.3 | -95.2 | 17.4 |
| 8160.2 | -104.0 | 9.3 |
| 9445.8 | -99.3 | 13.0 |
| 12000.6 | -104.4 | 10.3 |
| 13440.4 | -110.7 | 8.6 |
| 15126.8 | -107.9 | 8.8 |
| 15600.0 | -119.8 | 8.5 |
| 16066.8 | -107.2 | 19.3 |
| 18720.5 | -114.8 | 8.6 |
| 21468.4 | -113.2 | 13.1 |
| 22399.9 | -117.2 | 8.5 |
| 23280.2 | -127.4 | 10.3 |

Noise floor tilt over 200 Hz-12 kHz: **-9.7 dB/octave**.


## V10

> 72 deg firing, unevenly split across the banks: metallic and hard.

5.2 L  10 cyl  12.7:1  ·  firing order 5  ·  900-8496 rpm over 8.0 s

Reference: the crank itself: 10 firings a cycle on 2 banks

### Order balance [dB relative to order 5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -22.3 | -3.7 | — |
| 1 | -8.4 | -11.1 | -2.7 |
| 1.5 | -5.6 | -14.2 | -8.5 |
| 2 | -12.6 | -20.5 | — |
| 2.5 | -14.0 | -19.4 | — |
| 3 | -12.6 | -15.7 | — |
| 3.5 | -5.6 | -13.0 | -7.4 |
| 4 | -8.4 | -17.2 | -8.8 |
| 4.5 | -22.3 | -33.9 | — |
| **5** | +0.0 | +0.0 | +0.0 |
| 5.5 | -22.3 | -32.0 | — |
| 6 | -8.4 | -25.8 | -17.4 |
| 6.5 | -5.6 | -28.6 | -22.9 |
| 7 | -12.6 | -25.3 | — |
| 7.5 | -14.0 | -28.7 | — |
| 8 | -12.6 | -18.3 | — |
| 8.5 | -5.6 | -33.5 | -27.9 |
| 9 | -8.4 | -34.7 | -26.3 |
| 9.5 | -22.3 | -41.1 | — |
| 10 | +0.0 | -13.6 | -13.6 |

Driven orders: rms **16.4 dB**, mean -13.5 dB, worst -27.9 dB on order 8.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -3.7 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 220 kg` | 72.4 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 82.6 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 623 x 0.982/(4 x (1.100 + 0.019))` | 136.6 | 118.4 | -13.3 % | -42.1 | 10.6 | tailpipe length 1.100 m, mouth radius 0.031 m → 1.291 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 247.9 | 264.9 | +6.9 % | -56.2 | 7.1 | downstream run 1.92 m → 1.796 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.019))` | 363.3 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 413.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 687 x 0.995/(4 x 0.376)` | 454.1 | 519.3 | +14.4 % | -52.4 | 11.4 | primary length 0.360 m → 0.315 m |
| expansion chamber, first pass band | `nc/2L = 1 x 662/(2 x 0.400)` | 828.1 | 960.8 | +16.0 % | -63.8 | 8.3 | chamber length 0.400 m, volume 8.5 L → 0.345 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 687 x 0.995/(4 x 0.376)` | 1362.2 | 1321.3 | -3.0 % | -62.5 | 15.1 | primary length 0.360 m → 0.371 m |
| expansion chamber, second pass band | `nc/2L = 2 x 662/(2 x 0.400)` | 1656.2 | not found | — | — | — |  |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 118.4 | -42.1 | 10.6 |
| 264.9 | -56.2 | 7.1 |
| 519.3 | -52.4 | 11.4 |
| 960.8 | -63.8 | 8.3 |
| 1321.3 | -62.5 | 15.1 |
| 2079.9 | -62.2 | 11.9 |
| 2400.3 | -72.5 | 10.8 |
| 3119.9 | -75.5 | 11.0 |
| 3842.0 | -80.2 | 7.8 |
| 4323.3 | -77.4 | 14.6 |
| 6332.4 | -92.5 | 9.7 |
| 7441.0 | -95.6 | 9.0 |
| 8400.2 | -103.7 | 9.0 |
| 10319.8 | -102.7 | 11.6 |
| 10799.9 | -108.8 | 7.8 |
| 11039.9 | -106.4 | 10.7 |
| 12960.2 | -112.0 | 7.6 |
| 13200.0 | -108.3 | 11.9 |
| 15839.9 | -115.3 | 7.2 |
| 16079.5 | -112.1 | 12.2 |
| 18239.5 | -115.5 | 7.4 |
| 18960.4 | -113.7 | 11.3 |
| 21119.8 | -116.2 | 12.5 |
| 23279.7 | -125.6 | 22.0 |

Noise floor tilt over 200 Hz-12 kHz: **-7.9 dB/octave**.


## V12

> 60 deg firing, even on both banks: no beat left to hear, only pitch.

6.5 L  12 cyl  11.8:1  ·  firing order 6  ·  800-8496 rpm over 8.0 s

Reference: the crank itself: 12 firings a cycle on 2 banks

### Order balance [dB relative to order 6]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -11.5 | — |
| 1 | null | -15.7 | — |
| 1.5 | null | -16.8 | — |
| 2 | null | -21.9 | — |
| 2.5 | null | -43.3 | — |
| 3 | +0.0 | +11.4 | +11.4 |
| 3.5 | null | -21.2 | — |
| 4 | null | -13.9 | — |
| 4.5 | null | -27.1 | — |
| 5 | null | -25.5 | — |
| 5.5 | null | — | — |
| **6** | +0.0 | +0.0 | +0.0 |
| 6.5 | null | -43.8 | — |
| 7 | null | -31.4 | — |
| 7.5 | null | -34.4 | — |
| 8 | null | -31.8 | — |
| 8.5 | null | -38.6 | — |
| 9 | +0.0 | -11.8 | -11.8 |
| 9.5 | null | -44.3 | — |
| 10 | null | -38.1 | — |
| 10.5 | null | -40.5 | — |
| 11 | null | -39.7 | — |
| 11.5 | null | -33.0 | — |
| 12 | +0.0 | -18.6 | -18.6 |

Driven orders: rms **12.4 dB**, mean -4.8 dB, worst -18.6 dB on order 12 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -11.5 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 260 kg` | 66.6 | 63.9 | -4.0 % | -46.3 | 9.1 |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 119.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 648 x 0.946/(4 x (1.000 + 0.017))` | 150.7 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 359.6 | 274.9 | -23.6 % | -40.3 | 15.0 | downstream run 1.37 m → 1.788 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.140 + 0.017))` | 552.0 | 480.8 | -12.9 % | -54.8 | 8.0 | runner length 0.140 m → 0.181 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 714 x 0.989/(4 x 0.314)` | 561.6 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 599.3 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 678/(2 x 0.350)` | 969.1 | 1001.3 | +3.3 % | -66.8 | 5.8 | chamber length 0.350 m, volume 2.5 L → 0.339 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 714 x 0.989/(4 x 0.314)` | 1684.7 | 1552.9 | -7.8 % | -72.6 | 19.0 | primary length 0.300 m → 0.325 m |
| expansion chamber, second pass band | `nc/2L = 2 x 678/(2 x 0.350)` | 1938.2 | 1889.5 | -2.5 % | -77.2 | 12.1 | chamber length 0.350 m, volume 2.5 L → 0.359 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 63.9 | -46.3 | 9.1 |
| 274.9 | -40.3 | 15.0 |
| 480.8 | -54.8 | 8.0 |
| 1001.3 | -66.8 | 5.8 |
| 1192.7 | -60.0 | 18.6 |
| 1552.9 | -72.6 | 19.0 |
| 1889.5 | -77.2 | 12.1 |
| 2085.8 | -79.8 | 7.3 |
| 2435.6 | -85.0 | 7.5 |
| 3600.0 | -87.3 | 10.6 |
| 3839.3 | -92.6 | 5.2 |
| 4019.1 | -91.5 | 5.1 |
| 4559.6 | -93.9 | 5.4 |
| 4799.8 | -92.8 | 10.6 |
| 5279.8 | -102.6 | 6.9 |
| 6515.3 | -94.9 | 5.2 |
| 6957.3 | -94.7 | 12.9 |
| 8158.9 | -102.2 | 6.7 |
| 9146.5 | -100.6 | 8.8 |
| 13236.5 | -108.9 | 13.4 |
| 15716.5 | -110.4 | 5.5 |
| 16674.3 | -110.3 | 7.7 |
| 20074.5 | -117.0 | 5.6 |
| 21077.0 | -116.7 | 7.0 |

Noise floor tilt over 200 Hz-12 kHz: **-14.2 dB/octave**.


## 2-Rotor Wankel

> Four firings per cycle from two rotors: no valvetrain, no beat, just buzz.

2.6 L  4 cyl  10.0:1  ·  firing order 2  ·  950-8796 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -24.2 | — |
| 1 | null | -25.1 | — |
| 1.5 | null | -32.2 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -38.0 | — |
| 3 | null | -34.6 | — |
| 3.5 | null | -60.2 | — |
| 4 | +0.0 | -14.8 | -14.8 |

Driven orders: rms **10.5 dB**, mean -7.4 dB, worst -14.8 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -24.2 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00232 s over 1.52 m` | 107.8 | not found | — | — | — |  |
| block, first bending mode | `mass law on 95 kg` | 110.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 646 x 0.949/(4 x (1.000 + 0.018))` | 150.5 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 706 x 0.991/(4 x 0.620)` | 282.3 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00232 s over 1.52 m` | 323.5 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.200 + 0.016))` | 401.9 | 385.5 | -4.1 % | -50.7 | 8.0 | runner length 0.200 m → 0.225 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00232 s over 1.52 m` | 539.2 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 674/(2 x 0.500)` | 674.0 | 640.2 | -5.0 % | -49.4 | 13.4 | silencer length 0.500 m → 0.526 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 706 x 0.991/(4 x 0.620)` | 847.0 | 799.4 | -5.6 % | -50.6 | 8.9 | primary length 0.600 m → 0.636 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 53.6 | -31.6 | 26.2 |
| 209.0 | -31.7 | 12.5 |
| 385.5 | -50.7 | 8.0 |
| 640.2 | -49.4 | 13.4 |
| 799.4 | -50.6 | 8.9 |
| 960.7 | -54.0 | 8.0 |
| 1144.6 | -50.1 | 15.0 |
| 1567.5 | -66.0 | 6.2 |
| 1826.1 | -63.7 | 14.8 |
| 2771.7 | -75.0 | 11.8 |
| 3359.7 | -77.0 | 12.3 |
| 4080.0 | -81.8 | 8.8 |
| 4767.5 | -84.7 | 7.1 |
| 5389.6 | -87.5 | 7.5 |
| 6479.6 | -88.9 | 8.6 |
| 8160.8 | -103.4 | 10.1 |
| 9120.0 | -102.4 | 7.3 |
| 9600.0 | -101.3 | 12.2 |
| 13200.7 | -107.6 | 6.1 |
| 13680.9 | -105.8 | 9.3 |
| 14401.0 | -111.1 | 7.5 |
| 16799.9 | -109.0 | 11.9 |
| 20159.1 | -112.7 | 12.8 |
| 20823.8 | -112.8 | 5.7 |

Noise floor tilt over 200 Hz-12 kHz: **-10.1 dB/octave**.


## Turbo Inline-4

> Even 180 deg firing under a small fast single: bark, whistle, flutter.

2.0 L  4 cyl  9.6:1  ·  firing order 2  ·  820-6897 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -15.5 | — |
| 1 | null | -13.7 | — |
| 1.5 | null | -29.8 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -25.6 | — |
| 3 | null | -20.7 | — |
| 3.5 | null | -41.3 | — |
| 4 | +0.0 | -4.0 | -4.0 |

Driven orders: rms **2.8 dB**, mean -2.0 dB, worst -4.0 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -13.7 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 125 kg` | 96.0 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 101.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 643 x 0.996/(4 x (1.200 + 0.018))` | 131.4 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.5 L` | 163.7 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 303.4 | 250.7 | -17.4 % | -41.2 | 9.0 | downstream run 1.62 m → 1.958 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.240 + 0.017))` | 337.4 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 505.7 | 479.3 | -5.2 % | -58.1 | 8.4 | downstream run 1.62 m → 1.707 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 748 x 0.998/(4 x 0.366)` | 510.5 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 692/(2 x 0.400)` | 865.6 | 719.1 | -16.9 % | -58.0 | 11.3 | chamber length 0.400 m, volume 3.1 L → 0.481 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 748 x 0.998/(4 x 0.366)` | 1531.6 | 1149.5 | -24.9 % | -54.4 | 19.5 | primary length 0.350 m → 0.466 m |
| expansion chamber, second pass band | `nc/2L = 2 x 692/(2 x 0.400)` | 1731.2 | 1636.3 | -5.5 % | -63.3 | 25.7 | chamber length 0.400 m, volume 3.1 L → 0.423 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 50.4 | -33.4 | 25.3 |
| 250.7 | -41.2 | 9.0 |
| 479.3 | -58.1 | 8.4 |
| 719.1 | -58.0 | 11.3 |
| 1149.5 | -54.4 | 19.5 |
| 1636.3 | -63.3 | 25.7 |
| 2520.4 | -81.2 | 9.9 |
| 2632.8 | -80.1 | 7.6 |
| 3585.8 | -75.4 | 11.4 |
| 4187.2 | -74.6 | 15.8 |
| 4736.5 | -76.1 | 11.9 |
| 6481.3 | -71.5 | 19.0 |
| 7337.1 | -89.6 | 7.6 |
| 8376.6 | -97.4 | 9.3 |
| 8914.7 | -93.8 | 9.3 |
| 9474.1 | -88.0 | 8.4 |
| 10048.1 | -86.2 | 21.4 |
| 13125.9 | -85.9 | 22.3 |
| 14639.4 | -113.4 | 9.7 |
| 14961.4 | -113.2 | 11.0 |
| 15720.5 | -110.9 | 8.4 |
| 17280.4 | -108.4 | 22.5 |
| 21322.6 | -117.2 | 8.6 |
| 22089.3 | -116.8 | 9.0 |

Noise floor tilt over 200 Hz-12 kHz: **-8.1 dB/octave**.


## Twin-turbo V8

> Hot-vee twins over the 90-180-270-180 burble: offbeat, but muffled.

4.0 L  8 cyl  10.0:1  ·  firing order 4  ·  760-7097 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -17.2 | -5.9 |
| 1 | null | -15.6 | — |
| 1.5 | -3.7 | +0.3 | +4.0 |
| 2 | null | -21.2 | — |
| 2.5 | -3.7 | -1.6 | +2.1 |
| 3 | null | -32.8 | — |
| 3.5 | -11.4 | -15.2 | -3.8 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -33.0 | -21.6 |
| 5 | null | -28.1 | — |
| 5.5 | -3.7 | -14.1 | -10.4 |
| 6 | null | -36.6 | — |
| 6.5 | -3.7 | -7.0 | -3.3 |
| 7 | null | -32.5 | — |
| 7.5 | -11.4 | -32.8 | -21.4 |
| 8 | +0.0 | -7.1 | -7.1 |

Driven orders: rms **10.8 dB**, mean -6.7 dB, worst -21.6 dB on order 4.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -15.6 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00408 s over 2.52 m` | 61.2 | not found | — | — | — |  |
| block, first bending mode | `mass law on 235 kg` | 70.0 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 600 x 0.990/(4 x (1.400 + 0.020))` | 104.5 | 104.5 | +0.0 % | -35.8 | 20.4 | tailpipe length 1.400 m, mouth radius 0.033 m → 1.420 m |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 4.5 L` | 173.8 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00408 s over 2.52 m` | 183.7 | 207.2 | +12.8 % | -41.4 | 11.5 | downstream run 2.52 m → 2.234 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00408 s over 2.52 m` | 306.1 | 291.2 | -4.9 % | -48.1 | 9.0 | downstream run 2.52 m → 2.650 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.260 + 0.018))` | 312.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 692 x 0.996/(4 x 0.468)` | 367.9 | 370.6 | +0.7 % | -49.7 | 14.8 | primary length 0.450 m → 0.447 m |
| expansion chamber, first pass band | `nc/2L = 1 x 658/(2 x 0.550)` | 597.7 | 730.1 | +22.1 % | -54.3 | 12.1 | chamber length 0.550 m, volume 13.1 L → 0.450 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 692 x 0.996/(4 x 0.468)` | 1103.8 | 1112.0 | +0.7 % | -60.5 | 17.7 | primary length 0.450 m → 0.447 m |
| expansion chamber, second pass band | `nc/2L = 2 x 658/(2 x 0.550)` | 1195.5 | 910.4 | -23.8 % | -53.6 | 16.8 | chamber length 0.550 m, volume 13.1 L → 0.722 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 104.5 | -35.8 | 20.4 |
| 207.2 | -41.4 | 11.5 |
| 291.2 | -48.1 | 9.0 |
| 370.6 | -49.7 | 14.8 |
| 451.4 | -52.1 | 6.3 |
| 730.1 | -54.3 | 12.1 |
| 910.4 | -53.6 | 16.8 |
| 1112.0 | -60.5 | 17.7 |
| 1545.4 | -72.1 | 11.3 |
| 1786.8 | -71.1 | 18.1 |
| 2075.6 | -77.5 | 7.4 |
| 2494.2 | -78.9 | 7.2 |
| 3060.6 | -76.0 | 7.0 |
| 3614.9 | -73.4 | 13.5 |
| 4214.1 | -73.3 | 10.2 |
| 4783.4 | -76.7 | 11.6 |
| 5279.7 | -97.3 | 6.9 |
| 6667.6 | -89.7 | 14.3 |
| 7813.9 | -91.2 | 6.4 |
| 9446.2 | -88.5 | 17.9 |
| 11694.4 | -114.9 | 6.9 |
| 13199.5 | -110.3 | 13.6 |
| 17279.5 | -116.3 | 9.5 |
| 20337.4 | -116.9 | 13.0 |

Noise floor tilt over 200 Hz-12 kHz: **-8.7 dB/octave**.


## Turbo Inline-6

> Even 120 deg firing, no gaps at all, and a big lazy single over it.

3.0 L  6 cyl  9.2:1  ·  firing order 3  ·  780-7197 rpm over 8.0 s

Reference: the crank itself: 6 firings a cycle on 1 bank

### Order balance [dB relative to order 3]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -23.3 | — |
| 1 | null | -21.7 | — |
| 1.5 | null | -25.7 | — |
| 2 | null | -26.7 | — |
| 2.5 | null | — | — |
| **3** | +0.0 | +0.0 | +0.0 |
| 3.5 | null | -38.0 | — |
| 4 | null | -32.4 | — |
| 4.5 | null | -33.4 | — |
| 5 | null | -30.0 | — |
| 5.5 | null | — | — |
| 6 | +0.0 | -16.9 | -16.9 |

Driven orders: rms **11.9 dB**, mean -8.4 dB, worst -16.9 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -21.7 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00361 s over 2.22 m` | 69.2 | not found | — | — | — |  |
| block, first bending mode | `mass law on 195 kg` | 76.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 604 x 0.991/(4 x (1.600 + 0.021))` | 92.2 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 3.5 L` | 159.4 | 148.7 | -6.7 % | -32.4 | 24.8 |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00361 s over 2.22 m` | 207.5 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.300 + 0.018))` | 272.9 | 269.1 | -1.4 % | -39.0 | 11.2 | runner length 0.300 m → 0.323 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00361 s over 2.22 m` | 345.9 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 693 x 0.997/(4 x 0.497)` | 347.3 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 645/(2 x 0.600)` | 537.8 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 693 x 0.997/(4 x 0.497)` | 1041.9 | 1025.0 | -1.6 % | -53.6 | 19.5 | primary length 0.480 m → 0.488 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 148.7 | -32.4 | 24.8 |
| 269.1 | -39.0 | 11.2 |
| 1025.0 | -53.6 | 19.5 |
| 1407.7 | -64.1 | 22.6 |
| 1727.8 | -69.1 | 13.2 |
| 2823.7 | -67.4 | 18.5 |
| 3515.5 | -84.1 | 10.5 |
| 4888.2 | -86.8 | 13.5 |
| 5556.3 | -83.2 | 17.3 |
| 5999.6 | -94.2 | 13.4 |
| 6240.0 | -98.5 | 9.9 |
| 6960.2 | -100.6 | 10.0 |
| 7680.2 | -104.5 | 11.3 |
| 8399.7 | -110.0 | 10.8 |
| 9033.5 | -107.4 | 9.2 |
| 10136.2 | -107.0 | 11.9 |
| 10799.9 | -104.8 | 26.3 |
| 11460.3 | -111.1 | 10.7 |
| 12534.0 | -113.1 | 9.6 |
| 14640.1 | -116.7 | 13.5 |
| 15359.9 | -114.8 | 17.9 |
| 20161.2 | -119.8 | 12.9 |
| 20879.6 | -123.7 | 11.6 |
| 21928.4 | -122.0 | 17.1 |

Noise floor tilt over 200 Hz-12 kHz: **-9.7 dB/octave**.


## Turbodiesel I4

> No spark at all: a premixed spike, an iron block, and clatter.

2.0 L  4 cyl  21.5:1  ·  firing order 2  ·  800-4998 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -11.8 | — |
| 1 | null | -7.1 | — |
| 1.5 | null | — | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | — | — |
| 3 | null | -20.5 | — |
| 3.5 | null | — | — |
| 4 | +0.0 | -16.8 | -16.8 |

Driven orders: rms **11.9 dB**, mean -8.4 dB, worst -16.8 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -7.1 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 46.4 | not found | — | — | — |  |
| block, first bending mode | `mass law on 190 kg` | 77.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 511 x 0.995/(4 x (1.300 + 0.017))` | 96.5 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 139.1 | 119.7 | -13.9 % | -45.6 | 12.8 | downstream run 2.87 m → 3.331 m |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.8 L` | 153.7 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 231.8 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.016))` | 367.1 | 408.7 | +11.3 % | -50.9 | 22.4 | runner length 0.220 m → 0.212 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 595 x 0.998/(4 x 0.315)` | 471.7 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 570/(2 x 0.550)` | 518.2 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 551/(2 x 0.500)` | 550.6 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 551/(2 x 0.500)` | 1101.1 | 1096.7 | -0.4 % | -66.9 | 10.8 | chamber length 0.500 m, volume 22.4 L → 0.502 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 595 x 0.998/(4 x 0.315)` | 1415.1 | 1598.7 | +13.0 % | -77.9 | 14.2 | primary length 0.300 m → 0.266 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 119.7 | -45.6 | 12.8 |
| 408.7 | -50.9 | 22.4 |
| 849.2 | -57.8 | 13.6 |
| 961.2 | -66.8 | 8.3 |
| 1096.7 | -66.9 | 10.8 |
| 1598.7 | -77.9 | 14.2 |
| 2036.4 | -76.3 | 15.8 |
| 3043.1 | -73.9 | 11.7 |
| 3592.5 | -69.9 | 22.7 |
| 4187.7 | -68.8 | 14.4 |
| 4748.4 | -72.9 | 16.2 |
| 5695.9 | -91.5 | 8.2 |
| 5824.8 | -90.4 | 11.0 |
| 5954.8 | -91.8 | 8.1 |
| 6219.1 | -88.3 | 8.3 |
| 7184.8 | -82.9 | 18.2 |
| 7768.1 | -87.0 | 9.0 |
| 8375.3 | -93.9 | 10.7 |
| 8845.9 | -88.7 | 8.9 |
| 10002.3 | -82.5 | 26.6 |
| 12109.7 | -112.0 | 9.0 |
| 13094.9 | -106.8 | 16.0 |
| 16798.2 | -111.4 | 24.3 |
| 20543.9 | -117.2 | 10.4 |

Noise floor tilt over 200 Hz-12 kHz: **-5.0 dB/octave**.


## Big Single

> One firing every two turns: the crank itself is the rhythm.

0.7 L  1 cyl  12.0:1  ·  firing order 0.5  ·  1250-7597 rpm over 8.0 s

Reference: the crank itself: 1 firings a cycle on 1 bank

### Order balance [dB relative to order 0.5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| **0.5** | +0.0 | +0.0 | +0.0 |
| 1 | +0.0 | +10.9 | +10.9 |

Driven orders: rms **7.7 dB**, mean +5.5 dB, worst +10.9 dB on order 1 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: none resolvable.

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 45 kg` | 120.0 | 116.0 | -3.4 % | -31.9 | 20.7 |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 259.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 808 x 1.000/(4 x 0.637)` | 317.1 | 299.5 | -5.5 % | -40.1 | 12.8 | primary length 0.620 m → 0.656 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.020))` | 434.6 | 422.0 | -2.9 % | -41.8 | 9.5 | runner length 0.180 m → 0.206 m |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 735 x 1.000/(4 x (0.350 + 0.015))` | 503.6 | 479.4 | -4.8 % | -40.5 | 13.2 | tailpipe length 0.350 m, mouth radius 0.024 m → 0.383 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 777.4 | 778.2 | +0.1 % | -42.0 | 9.3 | downstream run 0.72 m → 0.724 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 808 x 1.000/(4 x 0.637)` | 951.2 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 769/(2 x 0.360)` | 1067.6 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 1295.6 | 1197.4 | -7.6 % | -52.9 | 13.7 | downstream run 0.72 m → 0.784 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 116.0 | -31.9 | 20.7 |
| 299.5 | -40.1 | 12.8 |
| 358.4 | -43.3 | 8.3 |
| 422.0 | -41.8 | 9.5 |
| 479.4 | -40.5 | 13.2 |
| 538.6 | -40.7 | 9.0 |
| 778.2 | -42.0 | 9.3 |
| 1197.4 | -52.9 | 13.7 |
| 1916.1 | -57.4 | 9.0 |
| 2034.3 | -55.3 | 8.5 |
| 2096.0 | -54.2 | 17.7 |
| 2155.5 | -55.9 | 10.7 |
| 2874.1 | -56.9 | 9.3 |
| 4423.0 | -72.0 | 8.8 |
| 5367.7 | -72.1 | 14.4 |
| 7852.0 | -82.0 | 9.0 |
| 8691.3 | -81.8 | 11.1 |
| 11141.3 | -87.4 | 8.0 |
| 12188.9 | -93.2 | 7.8 |
| 13124.9 | -91.9 | 10.2 |
| 15600.0 | -94.2 | 12.3 |
| 18169.3 | -96.3 | 13.5 |
| 20679.6 | -99.4 | 11.3 |
| 22375.4 | -104.5 | 11.4 |

Noise floor tilt over 200 Hz-12 kHz: **-7.5 dB/octave**.

