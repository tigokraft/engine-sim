# Calibration

Stage 16 of [the implementation plan](../IMPLEMENTATION_PLAN.md): the synth measured against a reference it did not produce. Regenerate with:

```bash
cargo run --release --example calibrate -- --markdown docs/measurements/calibration.md
```

Every figure below is *relative*: an order against the firing order, a measured frequency against a predicted one, a slope against an octave. That is deliberate and it is the point of the stage. A gain constant anywhere in the chain moves every order by the same number of decibels and cancels out of all of it, so nothing here can be closed by turning something up — only by changing a length, a volume or a radius.

| Engine | Firing order | Order balance rms | Worst order | Crank nulls | Modes placed | Floor tilt |
|---|---:|---:|---:|---:|---:|---:|
| [Inline-4](#inline-4) | 2 | 5.2 dB | -7.4 dB on 4 | -9.4 dB on 1 | 2/3 of 11 | -10.2 dB/oct |
| [Cross-plane V8](#cross-plane-v8) | 4 | 15.2 dB | -34.9 dB on 7.5 | -10.4 dB on 1, not enforced | 1/2 of 11 | -10.2 dB/oct |
| [Flat-plane V8](#flat-plane-v8) | 4 | 15.9 dB | -29.2 dB on 6 | -12.9 dB on 0.5, not enforced | 2/4 of 10 | -10.4 dB/oct |
| [V10](#v10) | 5 | 18.0 dB | -27.1 dB on 8.5 | -4.3 dB on 0.5, not enforced | 1/4 of 10 | -8.0 dB/oct |
| [V12](#v12) | 6 | 12.2 dB | -18.8 dB on 12 | -9.7 dB on 0.5, not enforced | 2/5 of 10 | -14.0 dB/oct |
| [2-Rotor Wankel](#2-rotor-wankel) | 2 | 0.3 dB | -0.4 dB on 4 | -18.9 dB on 0.5 | 3/4 of 9 | -9.4 dB/oct |
| [Turbo Inline-4](#turbo-inline-4) | 2 | 3.6 dB | -5.0 dB on 4 | -14.2 dB on 1, not enforced | 2/3 of 11 | -7.8 dB/oct |
| [Twin-turbo V8](#twin-turbo-v8) | 4 | 18.3 dB | -35.6 dB on 7.5 | -15.8 dB on 1, not enforced | 2/4 of 11 | -7.5 dB/oct |
| [Turbo Inline-6](#turbo-inline-6) | 3 | 12.1 dB | -17.0 dB on 6 | -21.3 dB on 1, not enforced | 3/3 of 10 | -10.0 dB/oct |
| [Turbodiesel I4](#turbodiesel-i4) | 2 | 2.8 dB | -3.9 dB on 4 | +7.1 dB on 1, not enforced | 2/3 of 12 | -4.8 dB/oct |
| [Big Single](#big-single) | 0.5 | 9.3 dB | +13.2 dB on 1 | — | 4/6 of 9 | -8.6 dB/oct |
| [Porsche 911 GT3 Cup (992)](#porsche-911-gt3-cup-992) | 3 | 17.8 dB | -26.7 dB on 4.5 | -23.4 dB on 1, not enforced | 2/5 of 9 | -8.8 dB/oct |
| [Porsche 911 GT3 Cup (997.2)](#porsche-911-gt3-cup-9972) | 3 | 22.0 dB | -29.9 dB on 4.5 | -30.7 dB on 1, not enforced | 3/5 of 9 | -9.2 dB/oct |
| [Mercedes-AMG GT3](#mercedes-amg-gt3) | 4 | 8.3 dB | +18.8 dB on 1.5 | -17.7 dB on 1, not enforced | 1/2 of 9 | -7.3 dB/oct |
| [Ferrari 458 Italia GT3](#ferrari-458-italia-gt3) | 4 | 11.9 dB | -20.9 dB on 6 | -14.9 dB on 1, not enforced | 3/4 of 8 | -8.5 dB/oct |
| [Audi R8 LMS GT3](#audi-r8-lms-gt3) | 5 | 19.9 dB | -30.1 dB on 8.5 | -5.8 dB on 3, not enforced | 3/4 of 8 | -9.2 dB/oct |

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
| 0.5 | null | -15.3 | — |
| 1 | null | -9.4 | — |
| 1.5 | null | -22.3 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -33.7 | — |
| 3 | null | -29.5 | — |
| 3.5 | null | -30.3 | — |
| 4 | +0.0 | -7.4 | -7.4 |

Driven orders: rms **5.2 dB**, mean -3.7 dB, worst -7.4 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -9.4 dB on order 1 (**fail**).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00322 s over 2.12 m` | 77.6 | not found | — | — | — |  |
| block, first bending mode | `mass law on 110 kg` | 102.3 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 637 x 0.956/(4 x (1.200 + 0.017))` | 125.2 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.2 L` | 162.1 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00322 s over 2.12 m` | 232.7 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.280 + 0.017))` | 292.0 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00322 s over 2.12 m` | 387.8 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 741 x 0.985/(4 x 0.416)` | 438.8 | 478.5 | +9.1 % | -58.7 | 9.4 | primary length 0.400 m → 0.367 m |
| expansion chamber, first pass band | `nc/2L = 1 x 704/(2 x 0.450)` | 781.8 | 959.8 | +22.8 % | -64.4 | 12.2 | chamber length 0.450 m, volume 9.3 L → 0.367 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 741 x 0.985/(4 x 0.416)` | 1316.3 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 704/(2 x 0.450)` | 1563.5 | 1441.5 | -7.8 % | -75.0 | 16.4 | chamber length 0.450 m, volume 9.3 L → 0.488 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 478.5 | -58.7 | 9.4 |
| 959.8 | -64.4 | 12.2 |
| 1441.5 | -75.0 | 16.4 |
| 2160.5 | -79.3 | 15.1 |
| 2879.8 | -85.3 | 14.3 |
| 3599.4 | -83.4 | 18.8 |
| 4005.0 | -89.1 | 10.2 |
| 4319.8 | -89.3 | 14.3 |
| 4700.4 | -89.8 | 13.6 |
| 5385.8 | -92.2 | 12.0 |
| 5626.3 | -93.4 | 9.9 |
| 5832.4 | -94.0 | 9.8 |
| 6349.6 | -95.7 | 12.9 |
| 8398.0 | -106.4 | 12.4 |
| 8640.2 | -110.2 | 10.3 |
| 9041.4 | -108.3 | 11.4 |
| 11279.8 | -109.7 | 14.6 |
| 12000.0 | -110.3 | 15.3 |
| 12240.0 | -111.4 | 11.7 |
| 15183.6 | -118.2 | 13.1 |
| 17999.9 | -116.4 | 20.1 |
| 18720.0 | -117.7 | 10.6 |
| 19440.5 | -116.7 | 11.6 |
| 22560.0 | -127.3 | 10.6 |

Noise floor tilt over 200 Hz-12 kHz: **-10.2 dB/octave**.


## Cross-plane V8

> 90-180-270-180 gaps on each bank: the offbeat American burble.

5.0 L  8 cyl  11.0:1  ·  firing order 4  ·  750-6997 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -11.7 | -0.3 |
| 1 | null | -10.4 | — |
| 1.5 | -3.7 | -11.4 | -7.7 |
| 2 | null | -15.4 | — |
| 2.5 | -3.7 | -14.3 | -10.6 |
| 3 | null | -23.2 | — |
| 3.5 | -11.4 | -25.8 | -14.4 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -25.4 | -14.1 |
| 5 | null | -23.1 | — |
| 5.5 | -3.7 | -19.1 | -15.4 |
| 6 | null | -13.2 | — |
| 6.5 | -3.7 | -16.3 | -12.6 |
| 7 | null | -26.1 | — |
| 7.5 | -11.4 | -46.3 | -34.9 |
| 8 | +0.0 | -10.6 | -10.6 |

Driven orders: rms **15.2 dB**, mean -12.1 dB, worst -34.9 dB on order 7.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -10.4 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00440 s over 2.82 m` | 56.8 | not found | — | — | — |  |
| block, first bending mode | `mass law on 210 kg` | 74.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 618 x 0.982/(4 x (1.500 + 0.018))` | 99.9 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 4.8 L` | 140.3 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00440 s over 2.82 m` | 170.3 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.380 + 0.018))` | 218.0 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00440 s over 2.82 m` | 283.9 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 730 x 0.995/(4 x 0.568)` | 319.6 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 688/(2 x 0.650)` | 529.1 | 638.8 | +20.7 % | -54.0 | 19.6 | chamber length 0.650 m, volume 22.1 L → 0.538 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 730 x 0.995/(4 x 0.568)` | 958.7 | 904.8 | -5.6 % | -62.4 | 12.1 | primary length 0.550 m → 0.583 m |
| expansion chamber, second pass band | `nc/2L = 2 x 688/(2 x 0.650)` | 1058.1 | not found | — | — | — |  |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 638.8 | -54.0 | 19.6 |
| 904.8 | -62.4 | 12.1 |
| 1535.1 | -71.7 | 19.9 |
| 2683.2 | -80.0 | 13.0 |
| 2992.6 | -80.8 | 13.5 |
| 3359.6 | -83.0 | 16.7 |
| 3901.0 | -80.2 | 20.3 |
| 4471.8 | -84.8 | 15.7 |
| 4799.9 | -90.2 | 17.7 |
| 5117.2 | -93.6 | 13.8 |
| 5712.5 | -94.8 | 19.9 |
| 7540.8 | -99.3 | 12.7 |
| 8453.0 | -101.6 | 17.2 |
| 8998.8 | -104.6 | 14.0 |
| 9599.9 | -103.4 | 18.8 |
| 11759.9 | -113.8 | 15.8 |
| 12276.0 | -111.2 | 12.9 |
| 13199.9 | -109.3 | 22.6 |
| 15599.9 | -118.0 | 14.3 |
| 16800.6 | -115.1 | 21.1 |
| 21360.1 | -119.6 | 21.6 |
| 21599.7 | -121.3 | 12.3 |
| 22236.3 | -126.1 | 12.1 |
| 22800.1 | -129.4 | 12.6 |

Noise floor tilt over 200 Hz-12 kHz: **-10.2 dB/octave**.


## Flat-plane V8

> Even 180 deg on both banks: two inline-fours sharing a crank.

4.5 L  8 cyl  12.5:1  ·  firing order 4  ·  900-8596 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -12.9 | — |
| 1 | null | -14.7 | — |
| 1.5 | null | -25.8 | — |
| 2 | +0.0 | +1.1 | +1.1 |
| 2.5 | null | -36.7 | — |
| 3 | null | -25.1 | — |
| 3.5 | null | -24.0 | — |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | null | -31.7 | — |
| 5 | null | -25.5 | — |
| 5.5 | null | -27.4 | — |
| 6 | +0.0 | -29.2 | -29.2 |
| 6.5 | null | -31.3 | — |
| 7 | null | -32.4 | — |
| 7.5 | null | — | — |
| 8 | +0.0 | -12.7 | -12.7 |

Driven orders: rms **15.9 dB**, mean -10.2 dB, worst -29.2 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -12.9 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 180 kg` | 80.0 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00256 s over 1.72 m` | 97.7 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 660 x 0.978/(4 x (0.900 + 0.020))` | 175.4 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00256 s over 1.72 m` | 293.2 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 721 x 0.990/(4 x 0.437)` | 408.4 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.015))` | 445.8 | 432.1 | -3.1 % | -49.5 | 12.0 | runner length 0.180 m → 0.201 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00256 s over 1.72 m` | 488.6 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 699/(2 x 0.400)` | 873.2 | 1014.3 | +16.2 % | -63.8 | 8.4 | chamber length 0.400 m, volume 9.3 L → 0.344 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 721 x 0.990/(4 x 0.437)` | 1225.1 | 1414.6 | +15.5 % | -71.6 | 11.5 | primary length 0.420 m → 0.364 m |
| expansion chamber, second pass band | `nc/2L = 2 x 699/(2 x 0.400)` | 1746.4 | 1671.1 | -4.3 % | -74.5 | 10.2 | chamber length 0.400 m, volume 9.3 L → 0.418 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 432.1 | -49.5 | 12.0 |
| 1014.3 | -63.8 | 8.4 |
| 1414.6 | -71.6 | 11.5 |
| 1671.1 | -74.5 | 10.2 |
| 2160.2 | -75.3 | 14.8 |
| 2880.1 | -82.0 | 9.2 |
| 3599.7 | -94.3 | 9.4 |
| 3839.8 | -92.6 | 10.7 |
| 4799.7 | -90.6 | 15.1 |
| 5520.0 | -93.1 | 9.9 |
| 5760.0 | -97.7 | 8.8 |
| 7200.4 | -97.9 | 18.4 |
| 7920.1 | -100.5 | 8.7 |
| 8160.0 | -103.3 | 10.2 |
| 9839.6 | -102.2 | 17.6 |
| 10559.7 | -108.5 | 8.6 |
| 10799.9 | -108.8 | 9.7 |
| 12480.3 | -109.3 | 11.5 |
| 13440.0 | -111.0 | 13.0 |
| 16079.6 | -113.1 | 13.8 |
| 17760.1 | -117.4 | 8.7 |
| 18719.8 | -115.5 | 11.1 |
| 21360.0 | -116.8 | 14.1 |
| 23280.0 | -130.2 | 13.3 |

Noise floor tilt over 200 Hz-12 kHz: **-10.4 dB/octave**.


## V10

> 72 deg firing, unevenly split across the banks: metallic and hard.

5.2 L  10 cyl  12.7:1  ·  firing order 5  ·  900-8496 rpm over 8.0 s

Reference: the crank itself: 10 firings a cycle on 2 banks

### Order balance [dB relative to order 5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -22.3 | -4.3 | — |
| 1 | -8.4 | +17.9 | +26.3 |
| 1.5 | -5.6 | -18.0 | -12.3 |
| 2 | -12.6 | -6.0 | — |
| 2.5 | -14.0 | -21.0 | — |
| 3 | -12.6 | -12.8 | — |
| 3.5 | -5.6 | -17.9 | -12.2 |
| 4 | -8.4 | -16.9 | -8.5 |
| 4.5 | -22.3 | -27.5 | — |
| **5** | +0.0 | +0.0 | +0.0 |
| 5.5 | -22.3 | -29.9 | — |
| 6 | -8.4 | -24.4 | -16.0 |
| 6.5 | -5.6 | -26.0 | -20.4 |
| 7 | -12.6 | -26.3 | — |
| 7.5 | -14.0 | -26.6 | — |
| 8 | -12.6 | -16.2 | — |
| 8.5 | -5.6 | -32.7 | -27.1 |
| 9 | -8.4 | -32.5 | -24.1 |
| 9.5 | -22.3 | -42.9 | — |
| 10 | +0.0 | -13.9 | -13.9 |

Driven orders: rms **18.0 dB**, mean -10.8 dB, worst -27.1 dB on order 8.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -4.3 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 220 kg` | 72.4 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00300 s over 1.92 m` | 83.4 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 629 x 0.976/(4 x (1.100 + 0.019))` | 137.1 | 132.1 | -3.7 % | -34.2 | 18.0 | tailpipe length 1.100 m, mouth radius 0.031 m → 1.162 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00300 s over 1.92 m` | 250.2 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.019))` | 363.3 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00300 s over 1.92 m` | 417.0 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 692 x 0.993/(4 x 0.376)` | 456.8 | 509.2 | +11.5 % | -53.5 | 11.4 | primary length 0.360 m → 0.323 m |
| expansion chamber, first pass band | `nc/2L = 1 x 668/(2 x 0.400)` | 835.0 | 1035.7 | +24.0 % | -61.8 | 12.3 | chamber length 0.400 m, volume 8.5 L → 0.322 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 692 x 0.993/(4 x 0.376)` | 1370.4 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 668/(2 x 0.400)` | 1670.0 | 2084.9 | +24.8 % | -61.3 | 16.4 | chamber length 0.400 m, volume 8.5 L → 0.320 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 132.1 | -34.2 | 18.0 |
| 509.2 | -53.5 | 11.4 |
| 1035.7 | -61.8 | 12.3 |
| 2084.9 | -61.3 | 16.4 |
| 2400.2 | -71.8 | 10.7 |
| 3119.9 | -75.2 | 11.2 |
| 3839.2 | -79.5 | 11.5 |
| 4377.6 | -76.3 | 14.8 |
| 5279.8 | -88.5 | 9.9 |
| 5760.0 | -97.3 | 9.7 |
| 5999.8 | -97.3 | 9.2 |
| 7440.5 | -97.0 | 11.0 |
| 8160.0 | -101.8 | 9.8 |
| 8400.0 | -103.7 | 11.2 |
| 10320.0 | -102.8 | 14.9 |
| 10799.8 | -108.3 | 9.8 |
| 11039.7 | -107.6 | 12.0 |
| 13200.1 | -107.6 | 14.1 |
| 13920.1 | -109.6 | 10.1 |
| 16079.8 | -112.3 | 12.4 |
| 18960.1 | -114.0 | 14.5 |
| 20400.2 | -120.1 | 10.0 |
| 21120.0 | -115.5 | 14.6 |
| 23279.8 | -127.7 | 24.7 |

Noise floor tilt over 200 Hz-12 kHz: **-8.0 dB/octave**.


## V12

> 60 deg firing, even on both banks: no beat left to hear, only pitch.

6.5 L  12 cyl  11.8:1  ·  firing order 6  ·  800-8496 rpm over 8.0 s

Reference: the crank itself: 12 firings a cycle on 2 banks

### Order balance [dB relative to order 6]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -9.7 | — |
| 1 | null | -17.1 | — |
| 1.5 | null | -15.9 | — |
| 2 | null | -21.0 | — |
| 2.5 | null | -33.7 | — |
| 3 | +0.0 | +8.9 | +8.9 |
| 3.5 | null | -15.7 | — |
| 4 | null | -13.6 | — |
| 4.5 | null | -25.8 | — |
| 5 | null | -26.6 | — |
| 5.5 | null | — | — |
| **6** | +0.0 | +0.0 | +0.0 |
| 6.5 | null | -55.3 | — |
| 7 | null | -31.0 | — |
| 7.5 | null | -32.3 | — |
| 8 | null | -32.6 | — |
| 8.5 | null | -36.7 | — |
| 9 | +0.0 | -12.8 | -12.8 |
| 9.5 | null | -42.5 | — |
| 10 | null | -37.9 | — |
| 10.5 | null | -40.8 | — |
| 11 | null | -40.0 | — |
| 11.5 | null | -35.1 | — |
| 12 | +0.0 | -18.8 | -18.8 |

Driven orders: rms **12.2 dB**, mean -5.7 dB, worst -18.8 dB on order 12 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -9.7 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 260 kg` | 66.6 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00206 s over 1.37 m` | 121.4 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 656 x 0.930/(4 x (1.000 + 0.017))` | 150.1 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00206 s over 1.37 m` | 364.3 | 275.1 | -24.5 % | -44.0 | 11.8 | downstream run 1.37 m → 1.810 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.140 + 0.017))` | 552.0 | 480.6 | -12.9 % | -54.4 | 9.1 | runner length 0.140 m → 0.181 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 723 x 0.986/(4 x 0.314)` | 566.4 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00206 s over 1.37 m` | 607.2 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 687/(2 x 0.350)` | 981.4 | 1200.8 | +22.4 % | -62.3 | 17.4 | chamber length 0.350 m, volume 2.5 L → 0.286 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 723 x 0.986/(4 x 0.314)` | 1699.1 | 1553.4 | -8.6 % | -71.2 | 25.6 | primary length 0.300 m → 0.328 m |
| expansion chamber, second pass band | `nc/2L = 2 x 687/(2 x 0.350)` | 1962.8 | 1883.4 | -4.0 % | -82.5 | 9.0 | chamber length 0.350 m, volume 2.5 L → 0.365 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 275.1 | -44.0 | 11.8 |
| 480.6 | -54.4 | 9.1 |
| 1200.8 | -62.3 | 17.4 |
| 1553.4 | -71.2 | 25.6 |
| 1883.4 | -82.5 | 9.0 |
| 2080.1 | -83.3 | 5.7 |
| 2439.7 | -87.9 | 6.3 |
| 3599.8 | -86.9 | 13.5 |
| 4080.4 | -93.5 | 5.5 |
| 4319.8 | -96.6 | 5.7 |
| 4559.7 | -95.0 | 6.5 |
| 4799.9 | -92.7 | 13.1 |
| 5279.9 | -102.4 | 8.3 |
| 5519.8 | -98.6 | 11.6 |
| 6000.2 | -97.9 | 5.3 |
| 7920.2 | -106.6 | 5.5 |
| 8160.0 | -104.4 | 7.8 |
| 9118.5 | -103.3 | 9.0 |
| 12239.8 | -113.9 | 5.3 |
| 13200.3 | -111.0 | 15.4 |
| 14401.8 | -112.8 | 6.2 |
| 15709.4 | -112.9 | 7.7 |
| 16800.2 | -113.7 | 5.3 |
| 21843.6 | -120.6 | 6.4 |

Noise floor tilt over 200 Hz-12 kHz: **-14.0 dB/octave**.


## 2-Rotor Wankel

> Four firings per cycle from two rotors: no valvetrain, no beat, just buzz.

2.6 L  4 cyl  10.0:1  ·  firing order 2  ·  950-8196 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -18.9 | — |
| 1 | null | -20.2 | — |
| 1.5 | null | -32.6 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -26.2 | — |
| 3 | null | -24.7 | — |
| 3.5 | null | -43.8 | — |
| 4 | +0.0 | -0.4 | -0.4 |

Driven orders: rms **0.3 dB**, mean -0.2 dB, worst -0.4 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -18.9 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 95 kg` | 110.1 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00222 s over 1.52 m` | 112.5 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 673 x 0.976/(4 x (1.000 + 0.018))` | 161.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 746 x 0.995/(4 x 0.620)` | 299.7 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00222 s over 1.52 m` | 337.6 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.200 + 0.016))` | 401.9 | 404.5 | +0.6 % | -53.3 | 8.0 | runner length 0.200 m → 0.215 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00222 s over 1.52 m` | 562.6 | 529.9 | -5.8 % | -39.7 | 10.7 | downstream run 1.52 m → 1.612 m |
| absorptive silencer, first pass band | `c/2L = 707/(2 x 0.500)` | 706.7 | 669.9 | -5.2 % | -37.9 | 22.2 | silencer length 0.500 m → 0.527 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 746 x 0.995/(4 x 0.620)` | 899.1 | 991.6 | +10.3 % | -48.8 | 9.8 | primary length 0.600 m → 0.544 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 59.0 | -42.4 | 10.3 |
| 213.1 | -34.9 | 20.8 |
| 404.5 | -53.3 | 8.0 |
| 529.9 | -39.7 | 10.7 |
| 669.9 | -37.9 | 22.2 |
| 801.3 | -39.8 | 7.6 |
| 991.6 | -48.8 | 9.8 |
| 1856.8 | -60.1 | 14.2 |
| 2684.7 | -72.2 | 13.9 |
| 3467.9 | -75.6 | 11.4 |
| 5173.8 | -83.4 | 6.4 |
| 6720.0 | -89.2 | 6.0 |
| 8026.9 | -96.4 | 6.0 |
| 8661.5 | -92.8 | 9.9 |
| 9685.6 | -94.6 | 7.0 |
| 12060.6 | -100.4 | 7.6 |
| 13675.0 | -100.7 | 8.8 |
| 14471.1 | -104.2 | 9.7 |
| 16056.7 | -103.9 | 7.1 |
| 16875.3 | -103.7 | 11.4 |
| 19247.6 | -110.9 | 7.3 |
| 20035.2 | -105.9 | 13.0 |
| 20837.6 | -106.5 | 9.3 |
| 23212.6 | -118.2 | 8.3 |

Noise floor tilt over 200 Hz-12 kHz: **-9.4 dB/octave**.


## Turbo Inline-4

> Even 180 deg firing under a small fast single: bark, whistle, flutter.

2.0 L  4 cyl  9.6:1  ·  firing order 2  ·  820-6897 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -19.3 | — |
| 1 | null | -14.2 | — |
| 1.5 | null | -28.9 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -35.7 | — |
| 3 | null | -25.8 | — |
| 3.5 | null | -54.5 | — |
| 4 | +0.0 | -5.0 | -5.0 |

Driven orders: rms **3.6 dB**, mean -2.5 dB, worst -5.0 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -14.2 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 125 kg` | 96.0 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00242 s over 1.62 m` | 103.3 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 657 x 0.995/(4 x (1.200 + 0.018))` | 134.1 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.5 L` | 163.7 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00242 s over 1.62 m` | 309.8 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.240 + 0.017))` | 337.4 | 399.3 | +18.4 % | -51.1 | 11.6 | runner length 0.240 m → 0.217 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00242 s over 1.62 m` | 516.3 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 763 x 0.998/(4 x 0.366)` | 520.7 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 707/(2 x 0.400)` | 883.5 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 763 x 0.998/(4 x 0.366)` | 1562.2 | 1639.7 | +5.0 % | -69.8 | 21.6 | primary length 0.350 m → 0.333 m |
| expansion chamber, second pass band | `nc/2L = 2 x 707/(2 x 0.400)` | 1767.0 | 1858.5 | +5.2 % | -74.7 | 7.7 | chamber length 0.400 m, volume 3.1 L → 0.380 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 58.7 | -43.3 | 12.7 |
| 399.3 | -51.1 | 11.6 |
| 1129.8 | -60.8 | 15.1 |
| 1639.7 | -69.8 | 21.6 |
| 1858.5 | -74.7 | 7.7 |
| 3100.3 | -79.8 | 8.1 |
| 3668.9 | -75.8 | 13.4 |
| 4187.6 | -74.6 | 17.1 |
| 4736.9 | -77.2 | 12.8 |
| 5220.4 | -80.9 | 7.3 |
| 6469.6 | -71.5 | 20.1 |
| 7337.7 | -90.7 | 8.1 |
| 7847.8 | -92.3 | 8.7 |
| 8374.2 | -98.7 | 10.3 |
| 8914.6 | -93.8 | 9.8 |
| 9473.3 | -88.0 | 8.2 |
| 10047.2 | -86.2 | 23.9 |
| 13129.5 | -85.9 | 26.6 |
| 14639.9 | -115.6 | 10.0 |
| 15240.3 | -126.3 | 7.5 |
| 17760.2 | -111.9 | 23.0 |
| 18480.4 | -114.3 | 7.2 |
| 21599.5 | -120.0 | 8.3 |
| 22079.9 | -120.9 | 9.8 |

Noise floor tilt over 200 Hz-12 kHz: **-7.8 dB/octave**.


## Twin-turbo V8

> Hot-vee twins over the 90-180-270-180 burble: offbeat, but muffled.

4.0 L  8 cyl  10.0:1  ·  firing order 4  ·  760-7097 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -14.9 | -3.5 |
| 1 | null | -15.8 | — |
| 1.5 | -3.7 | -15.6 | -11.9 |
| 2 | null | -22.4 | — |
| 2.5 | -3.7 | -16.2 | -12.5 |
| 3 | null | -45.4 | — |
| 3.5 | -11.4 | -25.6 | -14.2 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -34.6 | -23.3 |
| 5 | null | -23.9 | — |
| 5.5 | -3.7 | -25.5 | -21.8 |
| 6 | null | -29.0 | — |
| 6.5 | -3.7 | -18.2 | -14.5 |
| 7 | null | -30.1 | — |
| 7.5 | -11.4 | -47.0 | -35.6 |
| 8 | +0.0 | -18.2 | -18.2 |

Driven orders: rms **18.3 dB**, mean -15.6 dB, worst -35.6 dB on order 7.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -15.8 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00406 s over 2.52 m` | 61.6 | not found | — | — | — |  |
| block, first bending mode | `mass law on 235 kg` | 70.0 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 603 x 0.987/(4 x (1.400 + 0.020))` | 104.9 | 90.1 | -14.0 % | -46.8 | 8.9 | tailpipe length 1.400 m, mouth radius 0.033 m → 1.652 m |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 4.5 L` | 173.8 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00406 s over 2.52 m` | 184.7 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00406 s over 2.52 m` | 307.8 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.260 + 0.018))` | 312.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 694 x 0.995/(4 x 0.468)` | 368.9 | 369.7 | +0.2 % | -49.0 | 22.7 | primary length 0.450 m → 0.449 m |
| expansion chamber, first pass band | `nc/2L = 1 x 660/(2 x 0.550)` | 600.3 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 694 x 0.995/(4 x 0.468)` | 1106.7 | 1112.6 | +0.5 % | -64.1 | 19.3 | primary length 0.450 m → 0.448 m |
| expansion chamber, second pass band | `nc/2L = 2 x 660/(2 x 0.550)` | 1200.7 | 1440.2 | +19.9 % | -90.0 | 6.9 | chamber length 0.550 m, volume 13.1 L → 0.459 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 90.1 | -46.8 | 8.9 |
| 369.7 | -49.0 | 22.7 |
| 1112.6 | -64.1 | 19.3 |
| 1440.2 | -90.0 | 6.9 |
| 1563.7 | -79.3 | 12.4 |
| 1801.0 | -82.4 | 7.2 |
| 3062.2 | -77.1 | 8.6 |
| 3615.5 | -73.5 | 21.2 |
| 4213.6 | -73.2 | 10.5 |
| 4793.0 | -76.8 | 11.4 |
| 5039.8 | -98.3 | 7.3 |
| 5279.8 | -97.2 | 11.5 |
| 5863.1 | -94.3 | 6.5 |
| 7230.6 | -89.3 | 22.6 |
| 9474.7 | -88.6 | 25.8 |
| 11698.8 | -114.6 | 9.3 |
| 12096.9 | -117.5 | 8.5 |
| 12432.3 | -112.0 | 8.2 |
| 13200.1 | -112.0 | 19.2 |
| 17279.9 | -115.7 | 18.5 |
| 18422.1 | -125.3 | 8.1 |
| 20639.2 | -121.3 | 15.8 |
| 22161.6 | -128.1 | 8.7 |
| 22560.2 | -132.0 | 8.5 |

Noise floor tilt over 200 Hz-12 kHz: **-7.5 dB/octave**.


## Turbo Inline-6

> Even 120 deg firing, no gaps at all, and a big lazy single over it.

3.0 L  6 cyl  9.2:1  ·  firing order 3  ·  780-7197 rpm over 8.0 s

Reference: the crank itself: 6 firings a cycle on 1 bank

### Order balance [dB relative to order 3]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -24.6 | — |
| 1 | null | -21.3 | — |
| 1.5 | null | -23.9 | — |
| 2 | null | -27.9 | — |
| 2.5 | null | — | — |
| **3** | +0.0 | +0.0 | +0.0 |
| 3.5 | null | -42.2 | — |
| 4 | null | -34.6 | — |
| 4.5 | null | -33.7 | — |
| 5 | null | -28.3 | — |
| 5.5 | null | — | — |
| 6 | +0.0 | -17.0 | -17.0 |

Driven orders: rms **12.1 dB**, mean -8.5 dB, worst -17.0 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -21.3 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00359 s over 2.22 m` | 69.6 | not found | — | — | — |  |
| block, first bending mode | `mass law on 195 kg` | 76.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 608 x 0.987/(4 x (1.600 + 0.021))` | 92.5 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 3.5 L` | 159.4 | 149.1 | -6.5 % | -32.2 | 24.7 |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00359 s over 2.22 m` | 208.8 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.300 + 0.018))` | 272.9 | 270.5 | -0.9 % | -38.7 | 10.5 | runner length 0.300 m → 0.321 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00359 s over 2.22 m` | 347.9 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 696 x 0.996/(4 x 0.497)` | 348.7 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 649/(2 x 0.600)` | 540.8 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 696 x 0.996/(4 x 0.497)` | 1046.1 | 1023.9 | -2.1 % | -53.0 | 20.6 | primary length 0.480 m → 0.490 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 149.1 | -32.2 | 24.7 |
| 270.5 | -38.7 | 10.5 |
| 1023.9 | -53.0 | 20.6 |
| 1407.7 | -62.6 | 23.6 |
| 1726.6 | -68.3 | 12.5 |
| 2824.6 | -67.4 | 18.9 |
| 3506.1 | -83.7 | 11.1 |
| 4905.1 | -86.6 | 12.5 |
| 5556.6 | -83.1 | 16.0 |
| 5999.6 | -93.1 | 13.9 |
| 6960.3 | -99.3 | 10.7 |
| 7680.3 | -103.4 | 12.3 |
| 8400.1 | -109.0 | 12.9 |
| 10136.6 | -105.6 | 13.2 |
| 10799.9 | -103.6 | 26.3 |
| 11175.4 | -110.1 | 10.3 |
| 11460.3 | -110.0 | 12.1 |
| 11875.6 | -111.5 | 10.3 |
| 14640.3 | -114.9 | 18.4 |
| 15359.7 | -114.2 | 15.6 |
| 17760.4 | -120.0 | 10.1 |
| 20160.9 | -119.3 | 13.4 |
| 20878.8 | -122.1 | 11.6 |
| 21271.1 | -120.3 | 19.0 |

Noise floor tilt over 200 Hz-12 kHz: **-10.0 dB/octave**.


## Turbodiesel I4

> No spark at all: a premixed spike, an iron block, and clatter.

2.0 L  4 cyl  21.5:1  ·  firing order 2  ·  800-4998 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -2.2 | — |
| 1 | null | +7.1 | — |
| 1.5 | null | -9.8 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -2.5 | — |
| 3 | null | -7.0 | — |
| 3.5 | null | — | — |
| 4 | +0.0 | -3.9 | -3.9 |

Driven orders: rms **2.8 dB**, mean -2.0 dB, worst -3.9 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest +7.1 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00536 s over 2.87 m` | 46.7 | not found | — | — | — |  |
| block, first bending mode | `mass law on 190 kg` | 77.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 515 x 0.995/(4 x (1.300 + 0.017))` | 97.2 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00536 s over 2.87 m` | 140.0 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.8 L` | 153.7 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00536 s over 2.87 m` | 233.4 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.016))` | 367.1 | 399.8 | +8.9 % | -56.5 | 16.9 | runner length 0.220 m → 0.217 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 599 x 0.998/(4 x 0.315)` | 474.9 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 574/(2 x 0.550)` | 521.7 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 554/(2 x 0.500)` | 554.4 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 554/(2 x 0.500)` | 1108.8 | 1042.4 | -6.0 % | -73.3 | 14.0 | chamber length 0.500 m, volume 22.4 L → 0.532 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 599 x 0.998/(4 x 0.315)` | 1424.7 | 1680.1 | +17.9 % | -86.6 | 12.9 | primary length 0.300 m → 0.254 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 399.8 | -56.5 | 16.9 |
| 848.4 | -68.9 | 11.7 |
| 1042.4 | -73.3 | 14.0 |
| 1680.1 | -86.6 | 12.9 |
| 3043.2 | -73.9 | 11.5 |
| 3592.5 | -70.0 | 31.9 |
| 4188.2 | -69.0 | 14.2 |
| 4746.8 | -72.9 | 16.8 |
| 5494.5 | -86.5 | 10.4 |
| 5695.5 | -91.6 | 8.2 |
| 5824.4 | -90.6 | 10.6 |
| 5954.5 | -92.0 | 8.6 |
| 6219.5 | -88.3 | 8.0 |
| 6353.0 | -86.7 | 8.4 |
| 6488.6 | -86.4 | 7.7 |
| 7185.0 | -82.2 | 20.4 |
| 7767.2 | -86.1 | 7.7 |
| 8375.5 | -93.9 | 10.7 |
| 8847.0 | -88.7 | 9.0 |
| 9989.0 | -82.6 | 27.3 |
| 13039.1 | -104.2 | 28.3 |
| 16811.1 | -109.4 | 28.4 |
| 20551.0 | -115.7 | 19.4 |
| 23335.4 | -131.6 | 12.0 |

Noise floor tilt over 200 Hz-12 kHz: **-4.8 dB/octave**.


## Big Single

> One firing every two turns: the crank itself is the rhythm.

0.7 L  1 cyl  12.0:1  ·  firing order 0.5  ·  1250-6697 rpm over 8.0 s

Reference: the crank itself: 1 firings a cycle on 1 bank

### Order balance [dB relative to order 0.5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| **0.5** | +0.0 | +0.0 | +0.0 |
| 1 | +0.0 | +13.2 | +13.2 |

Driven orders: rms **9.3 dB**, mean +6.6 dB, worst +13.2 dB on order 1 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: none resolvable.

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 45 kg` | 120.0 | 115.1 | -4.1 % | -29.6 | 22.7 |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 736 x 0.715/(4 x 0.637)` | 206.4 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00103 s over 0.72 m` | 241.6 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 690 x 0.853/(4 x (0.350 + 0.015))` | 403.5 | 316.0 | -21.7 % | -45.8 | 4.3 | tailpipe length 0.350 m, mouth radius 0.024 m → 0.466 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.020))` | 434.6 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 736 x 0.715/(4 x 0.637)` | 619.1 | 551.7 | -10.9 % | -38.6 | 20.7 | primary length 0.620 m → 0.696 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00103 s over 0.72 m` | 724.8 | 720.3 | -0.6 % | -50.1 | 4.3 | downstream run 0.72 m → 0.729 m |
| absorptive silencer, first pass band | `c/2L = 711/(2 x 0.360)` | 987.5 | 1007.0 | +2.0 % | -53.1 | 6.8 | silencer length 0.360 m → 0.353 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00103 s over 0.72 m` | 1208.0 | 1287.7 | +6.6 % | -55.2 | 20.8 | downstream run 0.72 m → 0.680 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 115.1 | -29.6 | 22.7 |
| 316.0 | -45.8 | 4.3 |
| 551.7 | -38.6 | 20.7 |
| 720.3 | -50.1 | 4.3 |
| 792.3 | -39.1 | 15.6 |
| 1007.0 | -53.1 | 6.8 |
| 1287.7 | -55.2 | 20.8 |
| 1439.0 | -59.9 | 4.4 |
| 1916.5 | -63.4 | 5.0 |
| 2156.0 | -62.6 | 12.1 |
| 2386.1 | -63.4 | 4.1 |
| 2872.3 | -63.2 | 5.1 |
| 3611.2 | -67.8 | 6.3 |
| 4353.8 | -78.6 | 14.1 |
| 4837.4 | -83.2 | 4.4 |
| 5555.9 | -82.3 | 4.6 |
| 6000.8 | -83.3 | 7.9 |
| 8707.7 | -92.6 | 11.3 |
| 12002.9 | -102.1 | 11.9 |
| 15601.8 | -105.6 | 14.8 |
| 18135.4 | -109.3 | 10.1 |
| 18920.0 | -109.9 | 4.4 |
| 20625.5 | -115.6 | 5.1 |
| 22133.5 | -113.5 | 11.1 |

Noise floor tilt over 200 Hz-12 kHz: **-8.6 dB/octave**.


## Porsche 911 GT3 Cup (992)

> 4.0L flat-six at 8750 rpm: screaming boxer harmonics and sequential gear whine.

4.0 L  6 cyl  13.3:1  ·  firing order 3  ·  1100-8746 rpm over 8.0 s

Reference: the crank itself: 6 firings a cycle on 2 banks

### Order balance [dB relative to order 3]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -28.8 | — |
| 1 | null | -23.4 | — |
| 1.5 | +0.0 | -14.1 | -14.1 |
| 2 | null | -34.1 | — |
| 2.5 | null | -37.9 | — |
| **3** | +0.0 | +0.0 | +0.0 |
| 3.5 | null | -38.1 | — |
| 4 | null | -37.9 | — |
| 4.5 | +0.0 | -26.7 | -26.7 |
| 5 | null | -37.7 | — |
| 5.5 | null | -52.1 | — |
| 6 | +0.0 | -18.8 | -18.8 |

Driven orders: rms **17.8 dB**, mean -14.9 dB, worst -26.7 dB on order 4.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -23.4 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 145 kg` | 89.1 | 111.2 | +24.7 % | -27.2 | 28.2 |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 658 x 0.978/(4 x (0.750 + 0.025))` | 207.7 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00118 s over 0.78 m` | 212.2 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 3.5 L` | 233.0 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 708 x 0.990/(4 x 0.398)` | 439.6 | 387.4 | -11.9 % | -34.1 | 11.8 | primary length 0.380 m → 0.431 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.160 + 0.020))` | 483.0 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00118 s over 0.78 m` | 636.7 | 675.4 | +6.1 % | -48.9 | 11.5 | downstream run 0.78 m → 0.731 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00118 s over 0.78 m` | 1061.2 | 928.3 | -12.5 % | -48.6 | 13.6 | downstream run 0.78 m → 0.886 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 708 x 0.990/(4 x 0.398)` | 1318.7 | 1288.6 | -2.3 % | -62.1 | 7.3 | primary length 0.380 m → 0.389 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 111.2 | -27.2 | 28.2 |
| 387.4 | -34.1 | 11.8 |
| 675.4 | -48.9 | 11.5 |
| 928.3 | -48.6 | 13.6 |
| 1288.6 | -62.1 | 7.3 |
| 1371.4 | -63.1 | 8.5 |
| 1720.1 | -66.9 | 7.8 |
| 2303.9 | -62.1 | 18.3 |
| 3120.0 | -68.2 | 7.9 |
| 3870.4 | -73.2 | 9.8 |
| 4170.9 | -72.2 | 11.1 |
| 4799.8 | -75.0 | 10.5 |
| 5039.5 | -75.1 | 7.1 |
| 5760.2 | -84.7 | 8.0 |
| 6001.4 | -89.8 | 7.4 |
| 6960.4 | -81.6 | 17.5 |
| 7680.7 | -85.2 | 7.4 |
| 9039.6 | -89.9 | 6.4 |
| 9599.8 | -89.5 | 10.3 |
| 11761.0 | -93.0 | 14.0 |
| 14532.0 | -94.6 | 12.6 |
| 17323.1 | -97.7 | 12.9 |
| 19201.2 | -99.5 | 10.0 |
| 21199.9 | -101.9 | 14.1 |

Noise floor tilt over 200 Hz-12 kHz: **-8.8 dB/octave**.


## Porsche 911 GT3 Cup (997.2)

> 3.8L Mezger flat-six at 8500 rpm: dry-sump mechanical clatter and GT1 lineage.

3.8 L  6 cyl  12.6:1  ·  firing order 3  ·  1150-8496 rpm over 8.0 s

Reference: the crank itself: 6 firings a cycle on 2 banks

### Order balance [dB relative to order 3]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -33.2 | — |
| 1 | null | -30.7 | — |
| 1.5 | +0.0 | -21.8 | -21.8 |
| 2 | null | -36.1 | — |
| 2.5 | null | — | — |
| **3** | +0.0 | +0.0 | +0.0 |
| 3.5 | null | -48.6 | — |
| 4 | null | -44.4 | — |
| 4.5 | +0.0 | -29.9 | -29.9 |
| 5 | null | -40.0 | — |
| 5.5 | null | -57.5 | — |
| 6 | +0.0 | -23.8 | -23.8 |

Driven orders: rms **22.0 dB**, mean -18.8 dB, worst -29.9 dB on order 4.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -30.7 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 155 kg` | 86.2 | 97.5 | +13.1 % | -29.4 | 18.9 |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 666 x 0.968/(4 x (0.800 + 0.023))` | 195.8 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00124 s over 0.82 m` | 202.4 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 3.2 L` | 221.6 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 729 x 0.988/(4 x 0.418)` | 431.4 | 406.5 | -5.8 % | -28.7 | 27.3 | primary length 0.400 m → 0.424 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.019))` | 436.4 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00124 s over 0.82 m` | 607.1 | 641.2 | +5.6 % | -49.9 | 11.6 | downstream run 0.82 m → 0.779 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00124 s over 0.82 m` | 1011.9 | 871.5 | -13.9 % | -49.6 | 13.7 | downstream run 0.82 m → 0.955 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 729 x 0.988/(4 x 0.418)` | 1294.3 | 1229.0 | -5.0 % | -49.5 | 11.4 | primary length 0.400 m → 0.421 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 97.5 | -29.4 | 18.9 |
| 406.5 | -28.7 | 27.3 |
| 641.2 | -49.9 | 11.6 |
| 871.5 | -49.6 | 13.7 |
| 1229.0 | -49.5 | 11.4 |
| 2077.4 | -55.3 | 24.8 |
| 2160.1 | -67.6 | 7.6 |
| 2880.1 | -68.4 | 12.3 |
| 3240.7 | -81.5 | 7.7 |
| 3359.4 | -77.0 | 8.1 |
| 3839.6 | -74.8 | 11.1 |
| 4559.5 | -72.8 | 15.3 |
| 4798.8 | -75.0 | 8.3 |
| 5409.7 | -78.8 | 7.7 |
| 6480.1 | -87.6 | 7.5 |
| 7200.6 | -82.1 | 17.1 |
| 7920.2 | -86.1 | 8.7 |
| 9599.7 | -88.8 | 11.2 |
| 11519.8 | -93.2 | 13.2 |
| 14160.0 | -94.7 | 14.2 |
| 16799.5 | -97.4 | 14.7 |
| 18719.8 | -100.8 | 7.3 |
| 19439.9 | -99.2 | 10.7 |
| 21282.1 | -101.9 | 13.8 |

Noise floor tilt over 200 Hz-12 kHz: **-9.2 dB/octave**.


## Mercedes-AMG GT3

> 6.2L M159 cross-plane V8: earth-shaking low-frequency thunder through open side-pipes.

6.2 L  8 cyl  12.0:1  ·  firing order 4  ·  950-7497 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | +2.2 | +13.6 |
| 1 | null | -17.7 | — |
| 1.5 | -3.7 | +15.1 | +18.8 |
| 2 | null | -27.9 | — |
| 2.5 | -3.7 | +0.3 | +4.0 |
| 3 | null | — | — |
| 3.5 | -11.4 | -5.0 | +6.3 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -13.1 | -1.8 |
| 5 | null | -26.8 | — |
| 5.5 | -3.7 | -3.6 | +0.1 |
| 6 | null | -17.7 | — |
| 6.5 | -3.7 | -1.2 | +2.5 |
| 7 | null | -37.1 | — |
| 7.5 | -11.4 | -17.3 | -6.0 |
| 8 | +0.0 | -7.2 | -7.2 |

Driven orders: rms **8.3 dB**, mean +3.0 dB, worst +18.8 dB on order 1.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -17.7 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 205 kg` | 75.0 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 5.0 L` | 186.4 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 673 x 0.966/(4 x (0.600 + 0.020))` | 261.9 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00092 s over 0.62 m` | 271.1 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.240 + 0.020))` | 334.2 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 706 x 0.990/(4 x 0.519)` | 337.0 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00092 s over 0.62 m` | 813.3 | 679.4 | -16.5 % | -40.9 | 16.9 | downstream run 0.62 m → 0.743 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 706 x 0.990/(4 x 0.519)` | 1011.0 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00092 s over 0.62 m` | 1355.6 | 1276.4 | -5.8 % | -48.5 | 19.9 | downstream run 0.62 m → 0.659 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 105.0 | -21.3 | 31.5 |
| 427.0 | -41.1 | 13.7 |
| 679.4 | -40.9 | 16.9 |
| 1276.4 | -48.5 | 19.9 |
| 1477.4 | -56.2 | 7.2 |
| 1769.7 | -54.8 | 18.6 |
| 2058.8 | -58.0 | 10.3 |
| 2299.6 | -57.4 | 17.1 |
| 2638.4 | -59.9 | 10.3 |
| 2880.7 | -63.8 | 8.2 |
| 3106.1 | -61.0 | 14.8 |
| 3648.2 | -66.6 | 10.8 |
| 4452.7 | -71.0 | 12.4 |
| 5040.8 | -74.4 | 9.1 |
| 5667.0 | -76.8 | 10.2 |
| 6146.8 | -80.1 | 9.5 |
| 7539.7 | -84.8 | 7.0 |
| 10105.6 | -92.7 | 7.3 |
| 12095.3 | -96.7 | 7.6 |
| 16979.9 | -104.9 | 7.5 |
| 19526.9 | -107.9 | 10.9 |
| 21622.2 | -110.2 | 14.0 |
| 22852.4 | -121.7 | 6.9 |
| 23213.5 | -124.7 | 7.6 |

Noise floor tilt over 200 Hz-12 kHz: **-7.3 dB/octave**.


## Ferrari 458 Italia GT3

> 4.5L flat-plane V8 screaming to 9200 rpm: tuned 4-into-1 race extractors and pure tenor howl.

4.5 L  8 cyl  13.0:1  ·  firing order 4  ·  1100-8946 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -14.9 | — |
| 1 | null | -14.9 | — |
| 1.5 | null | -29.7 | — |
| 2 | +0.0 | -2.9 | -2.9 |
| 2.5 | null | -34.4 | — |
| 3 | null | -24.3 | — |
| 3.5 | null | -25.3 | — |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | null | -30.7 | — |
| 5 | null | -26.9 | — |
| 5.5 | null | -27.0 | — |
| 6 | +0.0 | -20.9 | -20.9 |
| 6.5 | null | -28.3 | — |
| 7 | null | -35.4 | — |
| 7.5 | null | — | — |
| 8 | +0.0 | -11.0 | -11.0 |

Driven orders: rms **11.9 dB**, mean -8.7 dB, worst -20.9 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -14.9 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 170 kg` | 82.3 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 653 x 0.979/(4 x (0.700 + 0.028))` | 219.7 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00111 s over 0.73 m` | 224.4 | 244.9 | +9.2 % | -37.0 | 16.0 | downstream run 0.73 m → 0.667 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 692 x 0.990/(4 x 0.417)` | 410.8 | 440.6 | +7.3 % | -44.7 | 12.0 | primary length 0.400 m → 0.373 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.150 + 0.015))` | 526.9 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00111 s over 0.73 m` | 673.1 | 789.9 | +17.4 % | -52.8 | 10.4 | downstream run 0.73 m → 0.620 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00111 s over 0.73 m` | 1121.8 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 692 x 0.990/(4 x 0.417)` | 1232.3 | 1295.3 | +5.1 % | -63.8 | 8.5 | primary length 0.400 m → 0.381 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 244.9 | -37.0 | 16.0 |
| 440.6 | -44.7 | 12.0 |
| 789.9 | -52.8 | 10.4 |
| 1295.3 | -63.8 | 8.5 |
| 1459.3 | -64.0 | 12.3 |
| 2241.6 | -67.6 | 5.8 |
| 2480.7 | -67.1 | 14.6 |
| 3189.9 | -75.4 | 8.5 |
| 3422.7 | -77.4 | 7.7 |
| 3892.1 | -81.4 | 8.8 |
| 4401.7 | -77.5 | 16.8 |
| 4799.3 | -82.3 | 7.1 |
| 5498.0 | -80.6 | 8.1 |
| 7535.7 | -88.2 | 13.1 |
| 8227.8 | -96.2 | 6.5 |
| 9598.8 | -94.2 | 10.7 |
| 10546.6 | -95.8 | 7.6 |
| 12722.6 | -99.6 | 13.1 |
| 15072.5 | -104.2 | 6.3 |
| 15838.0 | -103.4 | 10.4 |
| 18933.3 | -106.6 | 9.9 |
| 20157.7 | -111.6 | 5.8 |
| 20878.2 | -108.7 | 9.8 |
| 22454.1 | -114.4 | 5.6 |

Noise floor tilt over 200 Hz-12 kHz: **-8.5 dB/octave**.


## Audi R8 LMS GT3

> 5.2L 90 deg V10 at 8800 rpm: uneven-bank acoustic fire and straight-cut race gear scream.

5.2 L  10 cyl  12.5:1  ·  firing order 5  ·  1050-8796 rpm over 8.0 s

Reference: the crank itself: 10 firings a cycle on 2 banks

### Order balance [dB relative to order 5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -22.3 | -9.3 | — |
| 1 | -8.4 | +12.8 | +21.2 |
| 1.5 | -5.6 | -20.7 | -15.1 |
| 2 | -12.6 | -9.3 | — |
| 2.5 | -14.0 | -24.6 | — |
| 3 | -12.6 | -5.8 | — |
| 3.5 | -5.6 | -22.8 | -17.1 |
| 4 | -8.4 | -21.7 | -13.3 |
| 4.5 | -22.3 | -40.1 | — |
| **5** | +0.0 | +0.0 | +0.0 |
| 5.5 | -22.3 | -40.3 | — |
| 6 | -8.4 | -28.5 | -20.1 |
| 6.5 | -5.6 | -29.2 | -23.5 |
| 7 | -12.6 | -23.2 | — |
| 7.5 | -14.0 | -27.1 | — |
| 8 | -12.6 | -31.6 | — |
| 8.5 | -5.6 | -35.8 | -30.1 |
| 9 | -8.4 | -36.0 | -27.6 |
| 9.5 | -22.3 | -29.0 | — |
| 10 | +0.0 | -13.5 | -13.5 |

Driven orders: rms **19.9 dB**, mean -13.9 dB, worst -30.1 dB on order 8.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -5.8 dB on order 3 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 210 kg` | 74.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 661 x 0.957/(4 x (0.800 + 0.025))` | 191.7 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00125 s over 0.83 m` | 200.3 | 239.0 | +19.3 % | -43.4 | 8.6 | downstream run 0.83 m → 0.692 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.019))` | 436.4 | 411.7 | -5.7 % | -44.1 | 17.4 | runner length 0.180 m → 0.211 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 710 x 0.986/(4 x 0.366)` | 478.4 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00125 s over 0.83 m` | 600.8 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00125 s over 0.83 m` | 1001.4 | 958.4 | -4.3 % | -57.7 | 14.0 | downstream run 0.83 m → 0.863 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 710 x 0.986/(4 x 0.366)` | 1435.2 | 1306.9 | -8.9 % | -59.1 | 15.4 | primary length 0.350 m → 0.384 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 136.7 | -34.0 | 16.7 |
| 239.0 | -43.4 | 8.6 |
| 411.7 | -44.1 | 17.4 |
| 798.4 | -59.2 | 8.5 |
| 958.4 | -57.7 | 14.0 |
| 1306.9 | -59.1 | 15.4 |
| 2025.2 | -66.3 | 16.4 |
| 2878.1 | -71.5 | 8.8 |
| 3076.3 | -70.4 | 12.6 |
| 3421.2 | -82.3 | 9.5 |
| 4139.0 | -77.2 | 20.8 |
| 4824.5 | -83.9 | 10.8 |
| 5519.0 | -84.1 | 8.0 |
| 6456.8 | -89.7 | 15.6 |
| 7200.6 | -93.4 | 7.7 |
| 7920.2 | -95.2 | 7.8 |
| 9448.7 | -96.8 | 8.6 |
| 10559.3 | -97.9 | 9.0 |
| 12000.5 | -102.6 | 10.2 |
| 15119.8 | -105.5 | 9.0 |
| 15838.6 | -106.1 | 11.1 |
| 18479.5 | -109.5 | 10.2 |
| 21509.4 | -110.8 | 9.2 |
| 23279.1 | -124.6 | 8.1 |

Noise floor tilt over 200 Hz-12 kHz: **-9.2 dB/octave**.

