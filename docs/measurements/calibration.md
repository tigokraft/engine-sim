# Calibration

Stage 16 of [the implementation plan](../IMPLEMENTATION_PLAN.md): the synth measured against a reference it did not produce. Regenerate with:

```bash
cargo run --release --example calibrate -- --markdown docs/measurements/calibration.md
```

Every figure below is *relative*: an order against the firing order, a measured frequency against a predicted one, a slope against an octave. That is deliberate and it is the point of the stage. A gain constant anywhere in the chain moves every order by the same number of decibels and cancels out of all of it, so nothing here can be closed by turning something up — only by changing a length, a volume or a radius.

| Engine | Firing order | Order balance rms | Worst order | Crank nulls | Modes placed | Floor tilt |
|---|---:|---:|---:|---:|---:|---:|
| [Inline-4](#inline-4) | 2 | 4.4 dB | -6.2 dB on 4 | -13.3 dB on 1 | 1/2 of 11 | -10.7 dB/oct |
| [Cross-plane V8](#cross-plane-v8) | 4 | 15.2 dB | -34.9 dB on 7.5 | -10.4 dB on 1, not enforced | 1/2 of 11 | -10.2 dB/oct |
| [Flat-plane V8](#flat-plane-v8) | 4 | 12.6 dB | -22.6 dB on 6 | -12.6 dB on 0.5, not enforced | 4/5 of 10 | -9.9 dB/oct |
| [V10](#v10) | 5 | 17.7 dB | -28.6 dB on 8.5 | +0.6 dB on 3, not enforced | 2/5 of 10 | -8.1 dB/oct |
| [V12](#v12) | 6 | 8.1 dB | -11.7 dB on 9 | +3.9 dB on 3.5, not enforced | 3/6 of 10 | -11.9 dB/oct |
| [2-Rotor Wankel](#2-rotor-wankel) | 2 | 0.3 dB | -0.4 dB on 4 | -18.9 dB on 0.5 | 3/4 of 9 | -9.4 dB/oct |
| [Turbo Inline-4](#turbo-inline-4) | 2 | 7.0 dB | -9.8 dB on 4 | -23.0 dB on 1, not enforced | 1/3 of 11 | -8.4 dB/oct |
| [Twin-turbo V8](#twin-turbo-v8) | 4 | 20.7 dB | -41.9 dB on 7.5 | -18.4 dB on 1, not enforced | 2/2 of 11 | -7.8 dB/oct |
| [Turbo Inline-6](#turbo-inline-6) | 3 | 5.0 dB | -7.0 dB on 6 | -20.2 dB on 0.5, not enforced | 1/2 of 10 | -10.1 dB/oct |
| [Turbodiesel I4](#turbodiesel-i4) | 2 | 1.4 dB | -2.0 dB on 4 | +5.7 dB on 1, not enforced | 2/3 of 12 | -5.0 dB/oct |
| [Big Single](#big-single) | 0.5 | 9.3 dB | +13.2 dB on 1 | — | 4/6 of 9 | -8.6 dB/oct |
| [Porsche 911 GT3 Cup (992)](#porsche-911-gt3-cup-992) | 3 | 17.8 dB | -26.7 dB on 4.5 | -23.4 dB on 1, not enforced | 2/5 of 9 | -8.8 dB/oct |
| [Porsche 911 GT3 Cup (997.2)](#porsche-911-gt3-cup-9972) | 3 | 20.0 dB | -28.4 dB on 1.5 | -28.3 dB on 1, not enforced | 4/5 of 9 | -8.2 dB/oct |
| [Mercedes-AMG GT3](#mercedes-amg-gt3) | 4 | 8.8 dB | -23.0 dB on 7.5 | -16.6 dB on 6, not enforced | 1/4 of 9 | -8.2 dB/oct |
| [Ferrari 458 Italia GT3](#ferrari-458-italia-gt3) | 4 | 10.2 dB | -18.6 dB on 6 | -12.6 dB on 1, not enforced | 6/6 of 8 | -8.1 dB/oct |
| [Audi R8 LMS GT3](#audi-r8-lms-gt3) | 5 | 19.8 dB | -28.8 dB on 8.5 | -1.0 dB on 3, not enforced | 3/4 of 8 | -8.7 dB/oct |

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
| 0.5 | null | -18.1 | — |
| 1 | null | -13.3 | — |
| 1.5 | null | -26.1 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -36.7 | — |
| 3 | null | -28.0 | — |
| 3.5 | null | -54.0 | — |
| 4 | +0.0 | -6.2 | -6.2 |

Driven orders: rms **4.4 dB**, mean -3.1 dB, worst -6.2 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -13.3 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00329 s over 2.12 m` | 76.0 | not found | — | — | — |  |
| block, first bending mode | `mass law on 110 kg` | 102.3 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 625 x 0.949/(4 x (1.200 + 0.017))` | 122.0 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.5 L` | 164.9 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00329 s over 2.12 m` | 228.0 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.260 + 0.018))` | 312.1 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00329 s over 2.12 m` | 380.0 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 722 x 0.983/(4 x 0.416)` | 426.6 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 687/(2 x 0.450)` | 763.4 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 722 x 0.983/(4 x 0.416)` | 1279.7 | 1095.0 | -14.4 % | -58.8 | 12.5 | primary length 0.400 m → 0.467 m |
| expansion chamber, second pass band | `nc/2L = 2 x 687/(2 x 0.450)` | 1526.7 | 1557.5 | +2.0 % | -64.9 | 21.6 | chamber length 0.450 m, volume 9.3 L → 0.441 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 1095.0 | -58.8 | 12.5 |
| 1557.5 | -64.9 | 21.6 |
| 3022.1 | -82.2 | 14.7 |
| 3752.6 | -80.6 | 16.6 |
| 4202.1 | -81.0 | 17.7 |
| 4953.2 | -84.9 | 11.7 |
| 5279.1 | -84.6 | 14.2 |
| 5683.8 | -86.6 | 13.6 |
| 6398.8 | -89.8 | 12.5 |
| 8335.6 | -96.1 | 18.1 |
| 8684.3 | -99.8 | 11.7 |
| 9062.1 | -96.4 | 17.2 |
| 9469.7 | -98.9 | 15.8 |
| 9793.8 | -98.2 | 12.0 |
| 10242.8 | -99.6 | 12.7 |
| 13619.3 | -105.2 | 13.6 |
| 14349.3 | -104.9 | 14.8 |
| 14699.0 | -109.3 | 11.6 |
| 15066.8 | -107.6 | 17.2 |
| 15481.0 | -111.5 | 13.4 |
| 18412.9 | -112.0 | 11.4 |
| 19140.4 | -111.4 | 16.3 |
| 19593.3 | -112.4 | 13.0 |
| 22933.1 | -121.6 | 13.9 |

Noise floor tilt over 200 Hz-12 kHz: **-10.7 dB/octave**.


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
| 0.5 | null | -12.6 | — |
| 1 | null | -15.2 | — |
| 1.5 | null | -22.9 | — |
| 2 | +0.0 | +1.0 | +1.0 |
| 2.5 | null | -39.6 | — |
| 3 | null | -19.5 | — |
| 3.5 | null | -27.9 | — |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | null | -28.3 | — |
| 5 | null | -25.7 | — |
| 5.5 | null | -28.4 | — |
| 6 | +0.0 | -22.6 | -22.6 |
| 6.5 | null | -31.4 | — |
| 7 | null | -35.4 | — |
| 7.5 | null | — | — |
| 8 | +0.0 | -11.2 | -11.2 |

Driven orders: rms **12.6 dB**, mean -8.2 dB, worst -22.6 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -12.6 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 180 kg` | 80.0 | 80.6 | +0.8 % | -46.5 | 7.6 |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00262 s over 1.72 m` | 95.4 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 645 x 0.978/(4 x (0.900 + 0.020))` | 171.4 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00262 s over 1.72 m` | 286.3 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 701 x 0.990/(4 x 0.437)` | 396.8 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.015))` | 445.8 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00262 s over 1.72 m` | 477.1 | 516.3 | +8.2 % | -48.6 | 6.8 | downstream run 1.72 m → 1.590 m |
| expansion chamber, first pass band | `nc/2L = 1 x 680/(2 x 0.400)` | 850.3 | 864.5 | +1.7 % | -62.7 | 8.5 | chamber length 0.400 m, volume 9.3 L → 0.393 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 701 x 0.990/(4 x 0.437)` | 1190.5 | 1073.1 | -9.9 % | -60.8 | 11.2 | primary length 0.420 m → 0.466 m |
| expansion chamber, second pass band | `nc/2L = 2 x 680/(2 x 0.400)` | 1700.7 | 1366.7 | -19.6 % | -70.4 | 10.0 | chamber length 0.400 m, volume 9.3 L → 0.498 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 80.6 | -46.5 | 7.6 |
| 516.3 | -48.6 | 6.8 |
| 864.5 | -62.7 | 8.5 |
| 1073.1 | -60.8 | 11.2 |
| 1366.7 | -70.4 | 10.0 |
| 2081.1 | -69.4 | 15.8 |
| 2399.9 | -77.3 | 7.3 |
| 2819.1 | -73.9 | 12.9 |
| 2880.0 | -80.1 | 6.8 |
| 3120.2 | -80.6 | 6.9 |
| 3932.1 | -85.2 | 8.9 |
| 4752.7 | -85.1 | 15.9 |
| 5520.3 | -88.4 | 11.1 |
| 7441.0 | -92.9 | 16.1 |
| 9839.9 | -99.5 | 13.4 |
| 10800.0 | -105.8 | 7.0 |
| 12480.4 | -107.0 | 9.3 |
| 13439.6 | -106.7 | 10.6 |
| 15120.7 | -110.4 | 6.6 |
| 16078.4 | -109.8 | 8.9 |
| 17760.5 | -113.4 | 6.6 |
| 18719.6 | -112.3 | 9.6 |
| 21359.6 | -113.4 | 12.2 |
| 23279.7 | -126.9 | 13.4 |

Noise floor tilt over 200 Hz-12 kHz: **-9.9 dB/octave**.


## V10

> 72 deg firing, unevenly split across the banks: metallic and hard.

5.2 L  10 cyl  12.7:1  ·  firing order 5  ·  900-8496 rpm over 8.0 s

Reference: the crank itself: 10 firings a cycle on 2 banks

### Order balance [dB relative to order 5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -22.3 | -6.1 | — |
| 1 | -8.4 | +15.3 | +23.7 |
| 1.5 | -5.6 | -19.3 | -13.7 |
| 2 | -12.6 | -8.4 | — |
| 2.5 | -14.0 | -0.6 | — |
| 3 | -12.6 | +0.6 | — |
| 3.5 | -5.6 | -19.7 | -14.1 |
| 4 | -8.4 | -18.0 | -9.6 |
| 4.5 | -22.3 | -36.2 | — |
| **5** | +0.0 | +0.0 | +0.0 |
| 5.5 | -22.3 | -31.1 | — |
| 6 | -8.4 | -24.3 | -15.9 |
| 6.5 | -5.6 | -28.9 | -23.2 |
| 7 | -12.6 | -24.5 | — |
| 7.5 | -14.0 | -25.2 | — |
| 8 | -12.6 | -11.1 | — |
| 8.5 | -5.6 | -34.2 | -28.6 |
| 9 | -8.4 | -26.3 | -17.9 |
| 9.5 | -22.3 | -45.0 | — |
| 10 | +0.0 | -12.5 | -12.5 |

Driven orders: rms **17.7 dB**, mean -11.2 dB, worst -28.6 dB on order 8.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest +0.6 dB on order 3 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 220 kg` | 72.4 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00290 s over 1.92 m` | 86.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 647 x 0.976/(4 x (1.100 + 0.019))` | 141.1 | 132.0 | -6.4 % | -34.2 | 17.9 | tailpipe length 1.100 m, mouth radius 0.031 m → 1.196 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00290 s over 1.92 m` | 258.2 | 321.7 | +24.6 % | -47.1 | 15.4 | downstream run 1.92 m → 1.540 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.019))` | 363.3 | 383.3 | +5.5 % | -45.9 | 6.6 | runner length 0.220 m → 0.226 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00290 s over 1.92 m` | 430.3 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 722 x 0.993/(4 x 0.376)` | 476.5 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 694/(2 x 0.400)` | 867.0 | 720.5 | -16.9 % | -67.5 | 6.4 | chamber length 0.400 m, volume 8.5 L → 0.481 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 722 x 0.993/(4 x 0.376)` | 1429.4 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 694/(2 x 0.400)` | 1734.0 | 2075.1 | +19.7 % | -59.6 | 13.8 | chamber length 0.400 m, volume 8.5 L → 0.334 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 132.0 | -34.2 | 17.9 |
| 321.7 | -47.1 | 15.4 |
| 383.3 | -45.9 | 6.6 |
| 720.5 | -67.5 | 6.4 |
| 1035.3 | -57.5 | 14.1 |
| 2075.1 | -59.6 | 13.8 |
| 2490.2 | -67.2 | 8.4 |
| 3159.0 | -70.0 | 11.2 |
| 3838.0 | -77.4 | 9.0 |
| 4354.5 | -74.4 | 13.7 |
| 4800.2 | -86.3 | 6.5 |
| 6942.6 | -92.7 | 6.9 |
| 7439.8 | -92.5 | 10.7 |
| 8160.2 | -97.1 | 7.7 |
| 9038.5 | -99.1 | 11.3 |
| 11039.9 | -103.8 | 7.7 |
| 13200.9 | -104.9 | 9.0 |
| 13920.7 | -107.2 | 6.5 |
| 15600.0 | -111.6 | 6.8 |
| 16079.6 | -109.5 | 8.2 |
| 18239.8 | -112.7 | 11.4 |
| 20399.5 | -117.7 | 6.6 |
| 21120.5 | -113.4 | 12.7 |
| 23280.7 | -124.7 | 24.8 |

Noise floor tilt over 200 Hz-12 kHz: **-8.1 dB/octave**.


## V12

> 60 deg firing, even on both banks: no beat left to hear, only pitch.

6.5 L  12 cyl  11.8:1  ·  firing order 6  ·  800-8496 rpm over 8.0 s

Reference: the crank itself: 12 firings a cycle on 2 banks

### Order balance [dB relative to order 6]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -14.7 | — |
| 1 | null | -19.6 | — |
| 1.5 | null | -20.6 | — |
| 2 | null | -21.2 | — |
| 2.5 | null | -18.4 | — |
| 3 | +0.0 | +7.0 | +7.0 |
| 3.5 | null | +3.9 | — |
| 4 | null | -1.8 | — |
| 4.5 | null | -23.3 | — |
| 5 | null | -21.0 | — |
| 5.5 | null | — | — |
| **6** | +0.0 | +0.0 | +0.0 |
| 6.5 | null | — | — |
| 7 | null | -32.3 | — |
| 7.5 | null | -32.7 | — |
| 8 | null | -22.3 | — |
| 8.5 | null | -32.2 | — |
| 9 | +0.0 | -11.7 | -11.7 |
| 9.5 | null | -27.7 | — |
| 10 | null | -25.2 | — |
| 10.5 | null | -30.5 | — |
| 11 | null | -27.6 | — |
| 11.5 | null | -31.9 | — |
| 12 | +0.0 | -8.9 | -8.9 |

Driven orders: rms **8.1 dB**, mean -3.4 dB, worst -11.7 dB on order 9 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest +3.9 dB on order 3.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 260 kg` | 66.6 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00213 s over 1.37 m` | 117.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 633 x 0.944/(4 x (1.000 + 0.017))` | 147.0 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00213 s over 1.37 m` | 351.3 | 275.1 | -21.7 % | -40.5 | 15.0 | downstream run 1.37 m → 1.746 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 694 x 0.989/(4 x 0.314)` | 545.2 | 542.8 | -0.4 % | -42.0 | 10.1 | primary length 0.300 m → 0.301 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.140 + 0.017))` | 552.0 | 480.8 | -12.9 % | -39.1 | 24.8 | runner length 0.140 m → 0.181 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00213 s over 1.37 m` | 585.6 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 661/(2 x 0.350)` | 944.5 | 822.8 | -12.9 % | -48.3 | 14.9 | chamber length 0.350 m, volume 2.5 L → 0.402 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 694 x 0.989/(4 x 0.314)` | 1635.6 | 1656.5 | +1.3 % | -53.7 | 26.6 | primary length 0.300 m → 0.296 m |
| expansion chamber, second pass band | `nc/2L = 2 x 661/(2 x 0.350)` | 1889.0 | 1849.8 | -2.1 % | -73.2 | 7.3 | chamber length 0.350 m, volume 2.5 L → 0.357 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 275.1 | -40.5 | 15.0 |
| 480.8 | -39.1 | 24.8 |
| 542.8 | -42.0 | 10.1 |
| 822.8 | -48.3 | 14.9 |
| 1217.6 | -56.1 | 12.9 |
| 1656.5 | -53.7 | 26.6 |
| 1849.8 | -73.2 | 7.3 |
| 2508.0 | -75.1 | 12.8 |
| 3311.1 | -74.8 | 12.6 |
| 3652.7 | -75.4 | 10.2 |
| 4141.9 | -73.5 | 14.6 |
| 4885.7 | -74.4 | 16.5 |
| 5816.1 | -79.6 | 19.2 |
| 6617.9 | -88.5 | 9.0 |
| 8139.3 | -88.5 | 14.0 |
| 9034.1 | -87.7 | 16.0 |
| 9812.0 | -93.8 | 7.5 |
| 12376.6 | -98.6 | 10.9 |
| 13281.1 | -98.6 | 23.3 |
| 16592.7 | -103.6 | 13.2 |
| 17420.3 | -104.5 | 10.2 |
| 18246.3 | -108.2 | 8.0 |
| 20861.9 | -110.8 | 8.0 |
| 21561.0 | -109.5 | 11.9 |

Noise floor tilt over 200 Hz-12 kHz: **-11.9 dB/octave**.


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
| 0.5 | null | -29.4 | — |
| 1 | null | -23.0 | — |
| 1.5 | null | — | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -46.9 | — |
| 3 | null | -34.4 | — |
| 3.5 | null | -35.4 | — |
| 4 | +0.0 | -9.8 | -9.8 |

Driven orders: rms **7.0 dB**, mean -4.9 dB, worst -9.8 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -23.0 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 125 kg` | 96.0 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00243 s over 1.62 m` | 102.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 654 x 0.993/(4 x (1.200 + 0.018))` | 133.3 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.5 L` | 163.7 | 199.9 | +22.1 % | -35.4 | 21.8 |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00243 s over 1.62 m` | 308.6 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.240 + 0.017))` | 337.4 | 394.7 | +17.0 % | -47.5 | 18.7 | runner length 0.240 m → 0.220 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00243 s over 1.62 m` | 514.3 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 759 x 0.996/(4 x 0.366)` | 517.2 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 703/(2 x 0.400)` | 879.4 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 759 x 0.996/(4 x 0.366)` | 1551.7 | 1645.2 | +6.0 % | -60.6 | 28.0 | primary length 0.350 m → 0.330 m |
| expansion chamber, second pass band | `nc/2L = 2 x 703/(2 x 0.400)` | 1758.7 | not found | — | — | — |  |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 199.9 | -35.4 | 21.8 |
| 394.7 | -47.5 | 18.7 |
| 1132.0 | -52.3 | 14.8 |
| 1645.2 | -60.6 | 28.0 |
| 3204.4 | -75.3 | 10.5 |
| 3668.8 | -75.6 | 9.5 |
| 3959.2 | -73.6 | 15.6 |
| 4736.4 | -74.2 | 14.2 |
| 6368.7 | -71.5 | 17.3 |
| 8913.8 | -93.8 | 9.0 |
| 9097.4 | -92.0 | 11.3 |
| 10048.3 | -86.2 | 21.6 |
| 13137.4 | -85.9 | 23.3 |
| 14197.4 | -103.1 | 14.8 |
| 14640.4 | -104.9 | 11.8 |
| 14955.9 | -104.0 | 15.0 |
| 15454.6 | -105.2 | 10.5 |
| 15747.6 | -104.0 | 18.3 |
| 16242.6 | -105.2 | 13.7 |
| 20058.8 | -110.6 | 11.1 |
| 20830.3 | -107.4 | 16.3 |
| 21321.1 | -111.0 | 9.5 |
| 21609.8 | -111.1 | 13.0 |
| 22093.0 | -110.8 | 15.6 |

Noise floor tilt over 200 Hz-12 kHz: **-8.4 dB/octave**.


## Twin-turbo V8

> Hot-vee twins over the 90-180-270-180 burble: offbeat, but muffled.

4.0 L  8 cyl  10.0:1  ·  firing order 4  ·  760-7097 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -18.7 | -7.4 |
| 1 | null | -18.4 | — |
| 1.5 | -3.7 | -20.6 | -16.9 |
| 2 | null | -27.3 | — |
| 2.5 | -3.7 | -17.2 | -13.5 |
| 3 | null | -46.0 | — |
| 3.5 | -11.4 | -20.7 | -9.4 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -42.1 | -30.8 |
| 5 | null | -26.0 | — |
| 5.5 | -3.7 | -25.0 | -21.3 |
| 6 | null | -28.5 | — |
| 6.5 | -3.7 | -19.9 | -16.2 |
| 7 | null | -33.1 | — |
| 7.5 | -11.4 | -53.2 | -41.9 |
| 8 | +0.0 | -15.9 | -15.9 |

Driven orders: rms **20.7 dB**, mean -17.3 dB, worst -41.9 dB on order 7.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -18.4 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00410 s over 2.52 m` | 61.0 | not found | — | — | — |  |
| block, first bending mode | `mass law on 235 kg` | 70.0 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 598 x 0.985/(4 x (1.400 + 0.020))` | 103.8 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 4.5 L` | 173.8 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00410 s over 2.52 m` | 183.0 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00410 s over 2.52 m` | 305.0 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.260 + 0.018))` | 312.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 686 x 0.994/(4 x 0.468)` | 364.0 | 369.1 | +1.4 % | -47.6 | 11.7 | primary length 0.450 m → 0.444 m |
| expansion chamber, first pass band | `nc/2L = 1 x 653/(2 x 0.550)` | 593.8 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 686 x 0.994/(4 x 0.468)` | 1092.1 | 1112.6 | +1.9 % | -53.4 | 22.7 | primary length 0.450 m → 0.442 m |
| expansion chamber, second pass band | `nc/2L = 2 x 653/(2 x 0.550)` | 1187.6 | not found | — | — | — |  |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 369.1 | -47.6 | 11.7 |
| 1112.6 | -53.4 | 22.7 |
| 1551.3 | -70.9 | 18.3 |
| 3021.7 | -71.1 | 19.2 |
| 3759.5 | -72.9 | 12.9 |
| 5682.3 | -82.8 | 25.2 |
| 8337.6 | -91.3 | 13.3 |
| 9462.8 | -88.3 | 19.0 |
| 11707.1 | -100.9 | 18.5 |
| 12088.1 | -103.6 | 14.0 |
| 12432.2 | -101.3 | 20.2 |
| 12855.7 | -102.6 | 14.7 |
| 15793.7 | -109.8 | 14.5 |
| 16236.1 | -107.2 | 12.4 |
| 16975.4 | -107.1 | 23.9 |
| 18411.9 | -113.2 | 14.4 |
| 18843.9 | -119.2 | 13.4 |
| 20315.1 | -113.5 | 11.3 |
| 21036.8 | -112.9 | 19.1 |
| 21417.0 | -114.0 | 11.7 |
| 21747.1 | -114.5 | 11.4 |
| 22172.5 | -117.0 | 14.8 |
| 22477.2 | -117.8 | 12.9 |
| 22917.1 | -123.5 | 12.8 |

Noise floor tilt over 200 Hz-12 kHz: **-7.8 dB/octave**.


## Turbo Inline-6

> Even 120 deg firing, no gaps at all, and a big lazy single over it.

3.0 L  6 cyl  9.2:1  ·  firing order 3  ·  780-7197 rpm over 8.0 s

Reference: the crank itself: 6 firings a cycle on 1 bank

### Order balance [dB relative to order 3]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -20.2 | — |
| 1 | null | -23.8 | — |
| 1.5 | null | -27.7 | — |
| 2 | null | -31.6 | — |
| 2.5 | null | — | — |
| **3** | +0.0 | +0.0 | +0.0 |
| 3.5 | null | -40.4 | — |
| 4 | null | -34.4 | — |
| 4.5 | null | -36.3 | — |
| 5 | null | -33.1 | — |
| 5.5 | null | — | — |
| 6 | +0.0 | -7.0 | -7.0 |

Driven orders: rms **5.0 dB**, mean -3.5 dB, worst -7.0 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -20.2 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00363 s over 2.22 m` | 68.9 | not found | — | — | — |  |
| block, first bending mode | `mass law on 195 kg` | 76.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 602 x 0.980/(4 x (1.600 + 0.021))` | 91.0 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 3.5 L` | 159.4 | 137.6 | -13.7 % | -32.1 | 23.5 |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00363 s over 2.22 m` | 206.8 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.300 + 0.018))` | 272.9 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 687 x 0.995/(4 x 0.497)` | 343.6 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00363 s over 2.22 m` | 344.6 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 642/(2 x 0.600)` | 534.9 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 687 x 0.995/(4 x 0.497)` | 1030.9 | 1026.6 | -0.4 % | -53.4 | 16.0 | primary length 0.480 m → 0.482 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 137.6 | -32.1 | 23.5 |
| 697.9 | -46.8 | 17.1 |
| 1026.6 | -53.4 | 16.0 |
| 1412.9 | -66.7 | 13.5 |
| 1727.7 | -65.5 | 19.1 |
| 2823.0 | -67.3 | 17.0 |
| 4212.8 | -81.2 | 14.2 |
| 4493.7 | -82.9 | 15.2 |
| 5549.7 | -82.7 | 13.8 |
| 6648.3 | -91.2 | 14.0 |
| 6960.7 | -94.9 | 12.7 |
| 8399.6 | -105.7 | 12.6 |
| 9049.2 | -101.8 | 13.1 |
| 10133.6 | -101.8 | 16.2 |
| 10798.3 | -101.4 | 25.2 |
| 11167.9 | -106.0 | 13.4 |
| 11467.8 | -105.3 | 15.4 |
| 12171.4 | -109.5 | 12.3 |
| 14639.6 | -112.6 | 15.3 |
| 15360.8 | -111.1 | 19.1 |
| 20160.7 | -118.0 | 13.2 |
| 20572.4 | -118.6 | 12.1 |
| 20851.1 | -117.9 | 13.4 |
| 21265.4 | -116.8 | 20.4 |

Noise floor tilt over 200 Hz-12 kHz: **-10.1 dB/octave**.


## Turbodiesel I4

> No spark at all: a premixed spike, an iron block, and clatter.

2.0 L  4 cyl  21.5:1  ·  firing order 2  ·  800-4998 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -1.8 | — |
| 1 | null | +5.7 | — |
| 1.5 | null | -6.8 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -16.7 | — |
| 3 | null | -4.8 | — |
| 3.5 | null | — | — |
| 4 | +0.0 | -2.0 | -2.0 |

Driven orders: rms **1.4 dB**, mean -1.0 dB, worst -2.0 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest +5.7 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 46.4 | not found | — | — | — |  |
| block, first bending mode | `mass law on 190 kg` | 77.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 512 x 0.992/(4 x (1.300 + 0.017))` | 96.5 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 139.2 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.8 L` | 153.7 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 232.1 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.016))` | 367.1 | 400.2 | +9.0 % | -55.4 | 16.7 | runner length 0.220 m → 0.217 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 594 x 0.997/(4 x 0.315)` | 470.1 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 569/(2 x 0.550)` | 517.6 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 551/(2 x 0.500)` | 550.5 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 551/(2 x 0.500)` | 1101.1 | 1083.4 | -1.6 % | -72.5 | 11.5 | chamber length 0.500 m, volume 22.4 L → 0.508 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 594 x 0.997/(4 x 0.315)` | 1410.2 | 1680.4 | +19.2 % | -85.2 | 12.5 | primary length 0.300 m → 0.252 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 400.2 | -55.4 | 16.7 |
| 847.8 | -67.8 | 10.9 |
| 1083.4 | -72.5 | 11.5 |
| 1680.4 | -85.2 | 12.5 |
| 2977.4 | -74.3 | 7.7 |
| 3043.2 | -73.9 | 11.4 |
| 3592.6 | -70.0 | 14.6 |
| 4188.2 | -68.9 | 32.0 |
| 4748.0 | -72.9 | 17.1 |
| 5494.2 | -86.5 | 10.4 |
| 5695.7 | -91.5 | 7.8 |
| 5824.7 | -90.5 | 10.7 |
| 5954.4 | -91.9 | 8.5 |
| 6219.3 | -88.2 | 8.0 |
| 6352.8 | -86.6 | 8.5 |
| 7184.9 | -81.9 | 20.5 |
| 7767.7 | -86.4 | 8.1 |
| 8374.7 | -94.0 | 10.8 |
| 8847.5 | -88.7 | 8.5 |
| 10008.2 | -82.1 | 27.9 |
| 13112.1 | -104.0 | 27.2 |
| 16904.2 | -110.1 | 27.3 |
| 20554.6 | -116.0 | 18.0 |
| 23334.0 | -131.7 | 10.6 |

Noise floor tilt over 200 Hz-12 kHz: **-5.0 dB/octave**.


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
| 0.5 | null | -34.6 | — |
| 1 | null | -28.3 | — |
| 1.5 | +0.0 | -28.4 | -28.4 |
| 2 | null | -39.1 | — |
| 2.5 | null | — | — |
| **3** | +0.0 | +0.0 | +0.0 |
| 3.5 | null | -52.6 | — |
| 4 | null | -42.6 | — |
| 4.5 | +0.0 | -21.4 | -21.4 |
| 5 | null | -35.7 | — |
| 5.5 | null | — | — |
| 6 | +0.0 | -18.4 | -18.4 |

Driven orders: rms **20.0 dB**, mean -17.1 dB, worst -28.4 dB on order 1.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -28.3 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 155 kg` | 86.2 | 97.7 | +13.3 % | -34.6 | 11.2 |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 627 x 0.976/(4 x (0.800 + 0.023))` | 186.0 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00131 s over 0.82 m` | 190.6 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 3.2 L` | 221.6 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 676 x 0.992/(4 x 0.418)` | 401.5 | 399.9 | -0.4 % | -26.6 | 29.7 | primary length 0.400 m → 0.402 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.019))` | 436.4 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00131 s over 0.82 m` | 571.7 | 621.3 | +8.7 % | -42.2 | 17.9 | downstream run 0.82 m → 0.757 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00131 s over 0.82 m` | 952.8 | 868.0 | -8.9 % | -44.7 | 10.8 | downstream run 0.82 m → 0.903 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 676 x 0.992/(4 x 0.418)` | 1204.4 | 1207.3 | +0.2 % | -44.4 | 11.8 | primary length 0.400 m → 0.399 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 97.7 | -34.6 | 11.2 |
| 399.9 | -26.6 | 29.7 |
| 621.3 | -42.2 | 17.9 |
| 868.0 | -44.7 | 10.8 |
| 1207.3 | -44.4 | 11.8 |
| 1419.7 | -61.2 | 6.5 |
| 1658.8 | -62.7 | 9.2 |
| 2014.6 | -53.0 | 21.9 |
| 2791.0 | -60.5 | 12.9 |
| 3587.0 | -70.1 | 8.4 |
| 4564.9 | -69.2 | 12.1 |
| 4756.8 | -69.9 | 5.9 |
| 5441.5 | -73.0 | 9.1 |
| 5998.9 | -88.3 | 6.6 |
| 7200.0 | -78.3 | 17.8 |
| 7922.0 | -83.2 | 5.9 |
| 9051.5 | -86.0 | 6.4 |
| 9600.7 | -85.8 | 10.5 |
| 11519.1 | -90.4 | 11.3 |
| 14232.8 | -92.7 | 11.6 |
| 16799.3 | -96.4 | 13.1 |
| 18717.8 | -98.1 | 6.5 |
| 19440.9 | -97.2 | 10.4 |
| 21277.7 | -99.9 | 14.1 |

Noise floor tilt over 200 Hz-12 kHz: **-8.2 dB/octave**.


## Mercedes-AMG GT3

> 6.2L M159 cross-plane V8: earth-shaking low-frequency thunder through open side-pipes.

6.2 L  8 cyl  12.0:1  ·  firing order 4  ·  950-7497 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -11.1 | +0.3 |
| 1 | null | -27.4 | — |
| 1.5 | -3.7 | +2.8 | +6.5 |
| 2 | null | -33.2 | — |
| 2.5 | -3.7 | -8.8 | -5.1 |
| 3 | null | — | — |
| 3.5 | -11.4 | -14.8 | -3.4 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -11.3 | +0.0 |
| 5 | null | — | — |
| 5.5 | -3.7 | -1.2 | +2.5 |
| 6 | null | -16.6 | — |
| 6.5 | -3.7 | -5.7 | -2.0 |
| 7 | null | — | — |
| 7.5 | -11.4 | -34.3 | -23.0 |
| 8 | +0.0 | -12.2 | -12.2 |

Driven orders: rms **8.8 dB**, mean -3.6 dB, worst -23.0 dB on order 7.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -16.6 dB on order 6 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 205 kg` | 75.0 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 5.0 L` | 186.4 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 702 x 0.971/(4 x (0.600 + 0.020))` | 274.8 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00088 s over 0.62 m` | 282.9 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.240 + 0.020))` | 334.2 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 743 x 0.991/(4 x 0.519)` | 354.9 | 428.1 | +20.6 % | -34.2 | 10.7 | primary length 0.500 m → 0.415 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00088 s over 0.62 m` | 848.8 | 672.7 | -20.8 % | -33.1 | 21.8 | downstream run 0.62 m → 0.783 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 743 x 0.991/(4 x 0.519)` | 1064.7 | 1236.7 | +16.2 % | -45.7 | 19.3 | primary length 0.500 m → 0.430 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00088 s over 0.62 m` | 1414.7 | 1469.4 | +3.9 % | -50.5 | 10.3 | downstream run 0.62 m → 0.597 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 105.4 | -24.5 | 29.4 |
| 428.1 | -34.2 | 10.7 |
| 672.7 | -33.1 | 21.8 |
| 1236.7 | -45.7 | 19.3 |
| 1469.4 | -50.5 | 10.3 |
| 1773.4 | -50.9 | 22.5 |
| 2061.1 | -54.0 | 14.4 |
| 2300.5 | -51.7 | 20.9 |
| 2539.1 | -57.7 | 10.0 |
| 3058.2 | -59.6 | 13.8 |
| 3626.2 | -63.4 | 12.4 |
| 4424.6 | -66.5 | 13.5 |
| 4992.2 | -70.1 | 10.4 |
| 5560.5 | -73.9 | 12.4 |
| 6359.9 | -76.0 | 9.2 |
| 6890.6 | -79.9 | 6.9 |
| 10103.8 | -89.2 | 8.2 |
| 10758.9 | -90.6 | 7.1 |
| 12109.0 | -94.3 | 7.4 |
| 17340.5 | -103.3 | 7.8 |
| 17759.6 | -104.2 | 7.2 |
| 19323.1 | -105.4 | 12.1 |
| 21626.6 | -108.4 | 14.2 |
| 22856.2 | -119.2 | 8.7 |

Noise floor tilt over 200 Hz-12 kHz: **-8.2 dB/octave**.


## Ferrari 458 Italia GT3

> 4.5L flat-plane V8 screaming to 9200 rpm: tuned 4-into-1 race extractors and pure tenor howl.

4.5 L  8 cyl  13.0:1  ·  firing order 4  ·  1100-8946 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -14.0 | — |
| 1 | null | -12.6 | — |
| 1.5 | null | -27.5 | — |
| 2 | +0.0 | -4.7 | -4.7 |
| 2.5 | null | -32.4 | — |
| 3 | null | -25.2 | — |
| 3.5 | null | -19.7 | — |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | null | -31.8 | — |
| 5 | null | -28.5 | — |
| 5.5 | null | -22.3 | — |
| 6 | +0.0 | -18.6 | -18.6 |
| 6.5 | null | -25.1 | — |
| 7 | null | -31.8 | — |
| 7.5 | null | — | — |
| 8 | +0.0 | -7.2 | -7.2 |

Driven orders: rms **10.2 dB**, mean -7.6 dB, worst -18.6 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -12.6 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 170 kg` | 82.3 | 82.3 | -0.0 % | -41.1 | 7.6 |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 657 x 0.977/(4 x (0.700 + 0.028))` | 220.6 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00111 s over 0.73 m` | 225.8 | 247.1 | +9.5 % | -37.4 | 14.7 | downstream run 0.73 m → 0.665 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 700 x 0.989/(4 x 0.417)` | 414.6 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.150 + 0.015))` | 526.9 | 552.7 | +4.9 % | -41.1 | 14.5 | runner length 0.150 m → 0.157 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00111 s over 0.73 m` | 677.3 | 638.8 | -5.7 % | -48.6 | 8.7 | downstream run 0.73 m → 0.772 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00111 s over 0.73 m` | 1128.9 | 1126.8 | -0.2 % | -55.6 | 7.7 | downstream run 0.73 m → 0.729 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 700 x 0.989/(4 x 0.417)` | 1243.7 | 1276.7 | +2.7 % | -60.8 | 10.3 | primary length 0.400 m → 0.390 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 82.3 | -41.1 | 7.6 |
| 247.1 | -37.4 | 14.7 |
| 552.7 | -41.1 | 14.5 |
| 638.8 | -48.6 | 8.7 |
| 781.8 | -44.2 | 13.4 |
| 957.4 | -49.3 | 7.4 |
| 1126.8 | -55.6 | 7.7 |
| 1276.7 | -60.8 | 10.3 |
| 1496.6 | -59.7 | 11.2 |
| 2344.0 | -58.3 | 22.5 |
| 2479.2 | -65.7 | 8.9 |
| 3185.1 | -68.8 | 7.4 |
| 3469.0 | -67.3 | 16.6 |
| 4100.0 | -76.1 | 10.5 |
| 4443.8 | -72.6 | 19.3 |
| 6339.6 | -85.0 | 8.1 |
| 7593.1 | -84.2 | 13.8 |
| 8637.7 | -90.4 | 10.3 |
| 9594.9 | -90.1 | 12.1 |
| 12743.0 | -96.7 | 11.5 |
| 15028.6 | -99.3 | 12.0 |
| 15830.6 | -99.6 | 8.0 |
| 18850.1 | -104.1 | 8.1 |
| 20818.2 | -106.4 | 12.9 |

Noise floor tilt over 200 Hz-12 kHz: **-8.1 dB/octave**.


## Audi R8 LMS GT3

> 5.2L 90 deg V10 at 8800 rpm: uneven-bank acoustic fire and straight-cut race gear scream.

5.2 L  10 cyl  12.5:1  ·  firing order 5  ·  1050-8796 rpm over 8.0 s

Reference: the crank itself: 10 firings a cycle on 2 banks

### Order balance [dB relative to order 5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -22.3 | -8.8 | — |
| 1 | -8.4 | +11.2 | +19.6 |
| 1.5 | -5.6 | -23.2 | -17.6 |
| 2 | -12.6 | -12.8 | — |
| 2.5 | -14.0 | -14.8 | — |
| 3 | -12.6 | -1.0 | — |
| 3.5 | -5.6 | -23.9 | -18.3 |
| 4 | -8.4 | -23.0 | -14.6 |
| 4.5 | -22.3 | -44.4 | — |
| **5** | +0.0 | +0.0 | +0.0 |
| 5.5 | -22.3 | -31.2 | — |
| 6 | -8.4 | -30.5 | -22.1 |
| 6.5 | -5.6 | -31.5 | -25.9 |
| 7 | -12.6 | -14.5 | — |
| 7.5 | -14.0 | -25.9 | — |
| 8 | -12.6 | -29.6 | — |
| 8.5 | -5.6 | -34.4 | -28.8 |
| 9 | -8.4 | -33.5 | -25.1 |
| 9.5 | -22.3 | -27.5 | — |
| 10 | +0.0 | -8.9 | -8.9 |

Driven orders: rms **19.8 dB**, mean -14.2 dB, worst -28.8 dB on order 8.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -1.0 dB on order 3 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 210 kg` | 74.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 641 x 0.969/(4 x (0.800 + 0.025))` | 188.1 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00129 s over 0.83 m` | 194.1 | 234.8 | +21.0 % | -41.4 | 10.7 | downstream run 0.83 m → 0.682 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.019))` | 436.4 | 409.8 | -6.1 % | -38.5 | 21.7 | runner length 0.180 m → 0.212 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 685 x 0.990/(4 x 0.366)` | 463.4 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00129 s over 0.83 m` | 582.2 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00129 s over 0.83 m` | 970.4 | 952.1 | -1.9 % | -56.1 | 5.9 | downstream run 0.83 m → 0.841 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 685 x 0.990/(4 x 0.366)` | 1390.2 | 1262.4 | -9.2 % | -52.2 | 15.6 | primary length 0.350 m → 0.385 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 136.0 | -34.0 | 17.5 |
| 234.8 | -41.4 | 10.7 |
| 409.8 | -38.5 | 21.7 |
| 800.5 | -51.8 | 13.0 |
| 952.1 | -56.1 | 5.9 |
| 1262.4 | -52.2 | 15.6 |
| 2026.4 | -60.5 | 17.8 |
| 2881.8 | -65.5 | 14.0 |
| 3968.3 | -78.1 | 13.6 |
| 4799.2 | -79.4 | 9.4 |
| 5498.0 | -79.6 | 7.9 |
| 6445.8 | -88.8 | 8.4 |
| 7200.0 | -87.9 | 14.0 |
| 7921.4 | -90.1 | 7.9 |
| 9839.1 | -94.0 | 10.0 |
| 10605.2 | -97.5 | 5.6 |
| 12368.0 | -100.5 | 8.2 |
| 13439.0 | -102.2 | 5.8 |
| 15057.3 | -103.2 | 9.1 |
| 16025.6 | -103.5 | 8.0 |
| 17800.5 | -107.7 | 5.9 |
| 18478.6 | -107.2 | 7.3 |
| 21382.4 | -109.6 | 9.9 |
| 23191.5 | -122.9 | 5.3 |

Noise floor tilt over 200 Hz-12 kHz: **-8.7 dB/octave**.

