# Calibration

Stage 16 of [the implementation plan](../IMPLEMENTATION_PLAN.md): the synth measured against a reference it did not produce. Regenerate with:

```bash
cargo run --release --example calibrate -- --markdown docs/measurements/calibration.md
```

Every figure below is *relative*: an order against the firing order, a measured frequency against a predicted one, a slope against an octave. That is deliberate and it is the point of the stage. A gain constant anywhere in the chain moves every order by the same number of decibels and cancels out of all of it, so nothing here can be closed by turning something up — only by changing a length, a volume or a radius.

| Engine | Firing order | Order balance rms | Worst order | Crank nulls | Modes placed | Floor tilt |
|---|---:|---:|---:|---:|---:|---:|
| [Inline-4](#inline-4) | 2 | 4.3 dB | -6.0 dB on 4 | -16.3 dB on 1 | 2/3 of 11 | -10.6 dB/oct |
| [Cross-plane V8](#cross-plane-v8) | 4 | 12.2 dB | -34.0 dB on 7.5 | -1.2 dB on 3, not enforced | 2/4 of 9 | -10.9 dB/oct |
| [Big Cam Chopping V8](#big-cam-chopping-v8) | 4 | 8.7 dB | +15.8 dB on 2.5 | -9.9 dB on 2, not enforced | 1/7 of 9 | -10.4 dB/oct |
| [Flat-plane V8](#flat-plane-v8) | 4 | 11.3 dB | -20.4 dB on 6 | -12.6 dB on 0.5, not enforced | 3/4 of 10 | -9.7 dB/oct |
| [V10](#v10) | 5 | 17.0 dB | -27.2 dB on 8.5 | +3.0 dB on 3, not enforced | 1/4 of 10 | -7.9 dB/oct |
| [V12](#v12) | 6 | 7.8 dB | -10.9 dB on 9 | +4.7 dB on 3.5, not enforced | 3/6 of 10 | -11.8 dB/oct |
| [2-Rotor Wankel](#2-rotor-wankel) | 2 | 0.3 dB | -0.5 dB on 4 | -19.1 dB on 0.5 | 3/4 of 9 | -9.5 dB/oct |
| [Turbo Inline-4](#turbo-inline-4) | 2 | 7.2 dB | -10.1 dB on 4 | -26.0 dB on 1, not enforced | 1/3 of 11 | -8.2 dB/oct |
| [Twin-turbo V8](#twin-turbo-v8) | 4 | 19.2 dB | -37.4 dB on 7.5 | -19.4 dB on 1, not enforced | 2/2 of 11 | -7.7 dB/oct |
| [Turbo Inline-6](#turbo-inline-6) | 3 | 5.2 dB | -7.4 dB on 6 | -21.1 dB on 0.5, not enforced | 1/2 of 10 | -10.1 dB/oct |
| [Turbodiesel I4](#turbodiesel-i4) | 2 | 2.1 dB | -2.9 dB on 4 | +3.2 dB on 1, not enforced | 2/3 of 12 | -4.3 dB/oct |
| [Big Single](#big-single) | 0.5 | 8.8 dB | +12.4 dB on 1 | — | 4/6 of 9 | -8.6 dB/oct |
| [Porsche 911 GT3 Cup (992)](#porsche-911-gt3-cup-992) | 3 | 17.7 dB | -26.5 dB on 4.5 | -22.8 dB on 1, not enforced | 2/5 of 9 | -8.7 dB/oct |
| [Porsche 911 GT3 Cup (997.2)](#porsche-911-gt3-cup-9972) | 3 | 19.9 dB | -28.2 dB on 1.5 | -28.1 dB on 1, not enforced | 4/5 of 9 | -8.2 dB/oct |
| [Mercedes-AMG GT3](#mercedes-amg-gt3) | 4 | 8.2 dB | -20.9 dB on 7.5 | -16.7 dB on 6, not enforced | 1/4 of 9 | -8.2 dB/oct |
| [Ferrari 458 Italia GT3](#ferrari-458-italia-gt3) | 4 | 9.7 dB | -17.4 dB on 6 | -11.7 dB on 1, not enforced | 6/6 of 8 | -8.1 dB/oct |
| [Audi R8 LMS GT3](#audi-r8-lms-gt3) | 5 | 20.1 dB | -28.0 dB on 8.5 | -0.4 dB on 3, not enforced | 4/5 of 8 | -8.5 dB/oct |

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
| 0.5 | null | -21.0 | — |
| 1 | null | -16.3 | — |
| 1.5 | null | -29.9 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -38.6 | — |
| 3 | null | -28.4 | — |
| 3.5 | null | — | — |
| 4 | +0.0 | -6.0 | -6.0 |

Driven orders: rms **4.3 dB**, mean -3.0 dB, worst -6.0 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -16.3 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00329 s over 2.12 m` | 76.0 | not found | — | — | — |  |
| block, first bending mode | `mass law on 110 kg` | 102.3 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 625 x 0.949/(4 x (1.200 + 0.017))` | 122.0 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.5 L` | 164.9 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00329 s over 2.12 m` | 228.0 | 238.4 | +4.6 % | -46.9 | 12.5 | downstream run 2.12 m → 2.024 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.260 + 0.018))` | 312.1 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00329 s over 2.12 m` | 380.0 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 722 x 0.983/(4 x 0.416)` | 426.6 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 687/(2 x 0.450)` | 763.4 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 722 x 0.983/(4 x 0.416)` | 1279.7 | 1094.5 | -14.5 % | -58.8 | 12.6 | primary length 0.400 m → 0.468 m |
| expansion chamber, second pass band | `nc/2L = 2 x 687/(2 x 0.450)` | 1526.7 | 1557.5 | +2.0 % | -65.0 | 23.0 | chamber length 0.450 m, volume 9.3 L → 0.441 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 238.4 | -46.9 | 12.5 |
| 1094.5 | -58.8 | 12.6 |
| 1557.5 | -65.0 | 23.0 |
| 3022.3 | -82.4 | 15.3 |
| 3752.6 | -80.6 | 18.0 |
| 4202.1 | -81.0 | 18.1 |
| 5279.0 | -84.6 | 14.4 |
| 5683.8 | -86.7 | 13.5 |
| 6398.8 | -89.9 | 12.9 |
| 8335.6 | -96.1 | 19.5 |
| 8684.3 | -99.9 | 12.6 |
| 9062.1 | -96.4 | 17.9 |
| 9469.6 | -99.1 | 12.4 |
| 9794.0 | -98.2 | 16.6 |
| 10242.8 | -99.7 | 14.1 |
| 13619.3 | -105.4 | 13.8 |
| 14349.3 | -105.0 | 15.9 |
| 14699.0 | -109.3 | 12.4 |
| 15066.8 | -107.6 | 17.8 |
| 15481.0 | -111.5 | 14.6 |
| 19140.4 | -111.5 | 16.3 |
| 19593.1 | -112.4 | 13.5 |
| 20312.6 | -114.5 | 12.5 |
| 22933.1 | -121.7 | 14.2 |

Noise floor tilt over 200 Hz-12 kHz: **-10.6 dB/octave**.


## Cross-plane V8

> 90-180-270-180 gaps on each bank: the offbeat American burble.

5.0 L  8 cyl  11.0:1  ·  firing order 4  ·  750-6997 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -11.7 | -0.3 |
| 1 | null | -13.1 | — |
| 1.5 | -3.7 | -0.2 | +3.5 |
| 2 | null | -14.9 | — |
| 2.5 | -3.7 | +2.0 | +5.7 |
| 3 | null | -1.2 | — |
| 3.5 | -11.4 | +1.7 | +13.0 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -21.2 | -9.8 |
| 5 | null | -29.7 | — |
| 5.5 | -3.7 | -3.0 | +0.7 |
| 6 | null | -25.3 | — |
| 6.5 | -3.7 | +0.1 | +3.8 |
| 7 | null | -29.6 | — |
| 7.5 | -11.4 | -45.4 | -34.0 |
| 8 | +0.0 | -1.9 | -1.9 |

Driven orders: rms **12.2 dB**, mean -1.9 dB, worst -34.0 dB on order 7.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -1.2 dB on order 3 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 210 kg` | 74.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 618 x 0.982/(4 x (1.500 + 0.018))` | 99.9 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00246 s over 1.52 m` | 101.7 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 4.8 L` | 140.3 | 139.3 | -0.7 % | -36.1 | 24.0 |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.380 + 0.018))` | 218.0 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00246 s over 1.52 m` | 305.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 730 x 0.995/(4 x 0.568)` | 319.6 | 394.1 | +23.3 % | -54.0 | 6.8 | primary length 0.550 m → 0.446 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00246 s over 1.52 m` | 508.6 | 606.5 | +19.3 % | -52.7 | 10.4 | downstream run 1.52 m → 1.273 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 730 x 0.995/(4 x 0.568)` | 958.7 | 885.8 | -7.6 % | -55.1 | 9.7 | primary length 0.550 m → 0.595 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 139.3 | -36.1 | 24.0 |
| 394.1 | -54.0 | 6.8 |
| 606.5 | -52.7 | 10.4 |
| 885.8 | -55.1 | 9.7 |
| 1199.8 | -64.5 | 7.5 |
| 1535.2 | -71.3 | 17.0 |
| 1791.9 | -71.5 | 7.4 |
| 2681.9 | -79.6 | 7.4 |
| 2997.0 | -78.9 | 7.7 |
| 3358.6 | -79.7 | 8.5 |
| 3901.0 | -80.0 | 14.7 |
| 4318.9 | -88.7 | 6.8 |
| 4471.9 | -84.1 | 13.1 |
| 4799.8 | -89.2 | 10.0 |
| 5870.9 | -90.8 | 11.9 |
| 7541.1 | -97.0 | 8.0 |
| 8451.9 | -100.7 | 7.2 |
| 9000.0 | -100.1 | 11.0 |
| 9600.0 | -102.2 | 7.4 |
| 11760.0 | -112.6 | 7.7 |
| 12240.8 | -108.9 | 6.7 |
| 13199.8 | -105.7 | 14.7 |
| 16800.7 | -111.2 | 6.7 |
| 20489.9 | -111.2 | 12.8 |

Noise floor tilt over 200 Hz-12 kHz: **-10.9 dB/octave**.


## Big Cam Chopping V8

> 82 deg cam overlap, aggressive solid roller ramps and open headers: violent idle chop and raw bark.

7.0 L  8 cyl  11.5:1  ·  firing order 4  ·  900-6997 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -20.1 | -8.8 |
| 1 | null | -22.2 | — |
| 1.5 | -3.7 | +4.2 | +7.9 |
| 2 | null | -9.9 | — |
| 2.5 | -3.7 | +12.1 | +15.8 |
| 3 | null | -24.3 | — |
| 3.5 | -11.4 | -18.2 | -6.8 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -22.8 | -11.4 |
| 5 | null | -46.4 | — |
| 5.5 | -3.7 | -1.2 | +2.5 |
| 6 | null | — | — |
| 6.5 | -3.7 | +5.7 | +9.4 |
| 7 | null | — | — |
| 7.5 | -11.4 | — | — |
| 8 | +0.0 | -3.7 | -3.7 |

Driven orders: rms **8.7 dB**, mean +0.5 dB, worst +15.8 dB on order 2.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -9.9 dB on order 2 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 230 kg` | 70.8 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 5.2 L` | 169.9 | 192.6 | +13.4 % | -25.0 | 33.0 |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.280 + 0.020))` | 289.6 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 745 x 0.998/(4 x 0.576)` | 322.7 | 386.3 | +19.7 % | -45.7 | 4.8 | primary length 0.556 m → 0.465 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 745 x 0.998/(4 x 0.576)` | 968.2 | 851.9 | -12.0 % | -50.4 | 4.8 | primary length 0.556 m → 0.632 m |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 738 x 0.996/(4 x (0.060 + 0.023))` | 2206.8 | 2003.8 | -9.2 % | -59.2 | 14.7 | tailpipe length 0.060 m, mouth radius 0.038 m → 0.092 m |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00011 s over 0.08 m` | 2215.9 | 2532.5 | +14.3 % | -67.7 | 10.0 | downstream run 0.08 m → 0.073 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00011 s over 0.08 m` | 6647.8 | 7319.5 | +10.1 % | -84.8 | 6.6 | downstream run 0.08 m → 0.076 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00011 s over 0.08 m` | 11079.7 | 9514.0 | -14.1 % | -88.7 | 13.1 | downstream run 0.08 m → 0.097 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 192.6 | -25.0 | 33.0 |
| 386.3 | -45.7 | 4.8 |
| 480.0 | -54.9 | 4.6 |
| 659.2 | -37.8 | 21.1 |
| 851.9 | -50.4 | 4.8 |
| 1114.6 | -47.3 | 11.9 |
| 1497.4 | -63.1 | 7.5 |
| 1694.9 | -66.1 | 4.1 |
| 2003.8 | -59.2 | 14.7 |
| 2532.5 | -67.7 | 10.0 |
| 2835.9 | -71.8 | 5.2 |
| 3429.6 | -71.3 | 7.1 |
| 4002.1 | -77.7 | 8.0 |
| 4316.9 | -79.9 | 6.4 |
| 4710.7 | -81.2 | 5.8 |
| 5758.9 | -84.7 | 6.6 |
| 7319.5 | -84.8 | 6.6 |
| 9514.0 | -88.7 | 13.1 |
| 13155.1 | -100.0 | 14.2 |
| 13645.2 | -100.2 | 4.0 |
| 16688.1 | -105.9 | 5.6 |
| 20255.7 | -108.8 | 10.0 |

Noise floor tilt over 200 Hz-12 kHz: **-10.4 dB/octave**.


## Flat-plane V8

> Even 180 deg on both banks: two inline-fours sharing a crank.

4.5 L  8 cyl  12.5:1  ·  firing order 4  ·  900-8596 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -12.6 | — |
| 1 | null | -15.3 | — |
| 1.5 | null | -21.9 | — |
| 2 | +0.0 | +0.6 | +0.6 |
| 2.5 | null | -40.4 | — |
| 3 | null | -17.7 | — |
| 3.5 | null | -26.9 | — |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | null | -27.9 | — |
| 5 | null | -25.9 | — |
| 5.5 | null | -28.6 | — |
| 6 | +0.0 | -20.4 | -20.4 |
| 6.5 | null | -29.8 | — |
| 7 | null | -35.0 | — |
| 7.5 | null | — | — |
| 8 | +0.0 | -9.7 | -9.7 |

Driven orders: rms **11.3 dB**, mean -7.4 dB, worst -20.4 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -12.6 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 180 kg` | 80.0 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00262 s over 1.72 m` | 95.4 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 645 x 0.978/(4 x (0.900 + 0.020))` | 171.4 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00262 s over 1.72 m` | 286.3 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 701 x 0.990/(4 x 0.437)` | 396.8 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.015))` | 445.8 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00262 s over 1.72 m` | 477.1 | 516.4 | +8.2 % | -51.6 | 9.0 | downstream run 1.72 m → 1.589 m |
| expansion chamber, first pass band | `nc/2L = 1 x 680/(2 x 0.400)` | 850.3 | 851.2 | +0.1 % | -64.2 | 7.3 | chamber length 0.400 m, volume 9.3 L → 0.400 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 701 x 0.990/(4 x 0.437)` | 1190.5 | 1200.5 | +0.8 % | -74.4 | 10.5 | primary length 0.420 m → 0.417 m |
| expansion chamber, second pass band | `nc/2L = 2 x 680/(2 x 0.400)` | 1700.7 | 2081.2 | +22.4 % | -69.7 | 17.5 | chamber length 0.400 m, volume 9.3 L → 0.327 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 120.5 | -46.8 | 8.9 |
| 516.4 | -51.6 | 9.0 |
| 851.2 | -64.2 | 7.3 |
| 1071.9 | -61.7 | 12.1 |
| 1200.5 | -74.4 | 10.5 |
| 2081.2 | -69.7 | 17.5 |
| 2400.2 | -77.6 | 8.0 |
| 2819.0 | -74.0 | 13.4 |
| 3120.2 | -80.7 | 7.2 |
| 3932.3 | -85.4 | 10.5 |
| 4752.4 | -85.6 | 16.7 |
| 5520.3 | -88.5 | 12.0 |
| 7200.3 | -95.3 | 16.3 |
| 8640.4 | -105.7 | 7.5 |
| 9840.0 | -100.3 | 13.0 |
| 10800.0 | -105.9 | 7.5 |
| 12480.4 | -107.7 | 9.8 |
| 13439.6 | -106.8 | 11.0 |
| 15120.7 | -110.8 | 8.3 |
| 16078.5 | -110.1 | 8.9 |
| 17814.7 | -112.0 | 7.3 |
| 18719.6 | -112.8 | 10.8 |
| 21359.6 | -113.7 | 13.4 |
| 23279.7 | -127.3 | 13.2 |

Noise floor tilt over 200 Hz-12 kHz: **-9.7 dB/octave**.


## V10

> 72 deg firing, unevenly split across the banks: metallic and hard.

5.2 L  10 cyl  12.7:1  ·  firing order 5  ·  900-8496 rpm over 8.0 s

Reference: the crank itself: 10 firings a cycle on 2 banks

### Order balance [dB relative to order 5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -22.3 | -6.6 | — |
| 1 | -8.4 | +14.7 | +23.1 |
| 1.5 | -5.6 | -19.8 | -14.1 |
| 2 | -12.6 | -8.7 | — |
| 2.5 | -14.0 | +1.8 | — |
| 3 | -12.6 | +3.0 | — |
| 3.5 | -5.6 | -20.0 | -14.3 |
| 4 | -8.4 | -17.9 | -9.5 |
| 4.5 | -22.3 | -37.7 | — |
| **5** | +0.0 | +0.0 | +0.0 |
| 5.5 | -22.3 | -28.6 | — |
| 6 | -8.4 | -22.9 | -14.5 |
| 6.5 | -5.6 | -29.2 | -23.6 |
| 7 | -12.6 | -22.5 | — |
| 7.5 | -14.0 | -23.1 | — |
| 8 | -12.6 | -8.7 | — |
| 8.5 | -5.6 | -32.8 | -27.2 |
| 9 | -8.4 | -23.9 | -15.5 |
| 9.5 | -22.3 | -40.4 | — |
| 10 | +0.0 | -11.1 | -11.1 |

Driven orders: rms **17.0 dB**, mean -10.7 dB, worst -27.2 dB on order 8.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest +3.0 dB on order 3 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 220 kg` | 72.4 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00290 s over 1.92 m` | 86.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 647 x 0.976/(4 x (1.100 + 0.019))` | 141.1 | 132.0 | -6.4 % | -37.1 | 17.9 | tailpipe length 1.100 m, mouth radius 0.031 m → 1.196 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00290 s over 1.92 m` | 258.2 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.019))` | 363.3 | 321.7 | -11.5 % | -47.2 | 16.9 | runner length 0.220 m → 0.270 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00290 s over 1.92 m` | 430.3 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 722 x 0.993/(4 x 0.376)` | 476.5 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 694/(2 x 0.400)` | 867.0 | 720.6 | -16.9 % | -67.8 | 7.3 | chamber length 0.400 m, volume 8.5 L → 0.481 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 722 x 0.993/(4 x 0.376)` | 1429.4 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 694/(2 x 0.400)` | 1734.0 | 2074.6 | +19.6 % | -59.6 | 13.8 | chamber length 0.400 m, volume 8.5 L → 0.334 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 132.0 | -37.1 | 17.9 |
| 321.7 | -47.2 | 16.9 |
| 720.6 | -67.8 | 7.3 |
| 1022.3 | -57.4 | 14.0 |
| 2074.6 | -59.6 | 13.8 |
| 2490.4 | -67.2 | 8.4 |
| 3158.9 | -70.0 | 11.3 |
| 3838.1 | -77.4 | 9.0 |
| 4354.2 | -74.4 | 13.6 |
| 4800.2 | -86.9 | 7.2 |
| 5279.6 | -86.7 | 7.5 |
| 7440.0 | -93.4 | 11.8 |
| 8160.2 | -97.2 | 8.8 |
| 10319.4 | -100.7 | 10.7 |
| 11039.8 | -104.0 | 7.7 |
| 12480.3 | -105.5 | 8.3 |
| 13200.9 | -104.9 | 10.1 |
| 13920.6 | -108.3 | 6.9 |
| 15360.6 | -110.5 | 7.4 |
| 15600.0 | -111.6 | 6.8 |
| 16079.7 | -109.8 | 8.4 |
| 18239.8 | -112.8 | 11.3 |
| 21120.7 | -113.7 | 12.8 |
| 23280.7 | -124.9 | 25.2 |

Noise floor tilt over 200 Hz-12 kHz: **-7.9 dB/octave**.


## V12

> 60 deg firing, even on both banks: no beat left to hear, only pitch.

6.5 L  12 cyl  11.8:1  ·  firing order 6  ·  800-8496 rpm over 8.0 s

Reference: the crank itself: 12 firings a cycle on 2 banks

### Order balance [dB relative to order 6]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -16.9 | — |
| 1 | null | -21.2 | — |
| 1.5 | null | -22.7 | — |
| 2 | null | -21.1 | — |
| 2.5 | null | -17.7 | — |
| 3 | +0.0 | +7.8 | +7.8 |
| 3.5 | null | +4.7 | — |
| 4 | null | -1.0 | — |
| 4.5 | null | -22.5 | — |
| 5 | null | -20.6 | — |
| 5.5 | null | — | — |
| **6** | +0.0 | +0.0 | +0.0 |
| 6.5 | null | — | — |
| 7 | null | -32.2 | — |
| 7.5 | null | -32.5 | — |
| 8 | null | -21.5 | — |
| 8.5 | null | -31.5 | — |
| 9 | +0.0 | -10.9 | -10.9 |
| 9.5 | null | -27.0 | — |
| 10 | null | -24.4 | — |
| 10.5 | null | -29.8 | — |
| 11 | null | -26.8 | — |
| 11.5 | null | -31.2 | — |
| 12 | +0.0 | -8.1 | -8.1 |

Driven orders: rms **7.8 dB**, mean -2.8 dB, worst -10.9 dB on order 9 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest +4.7 dB on order 3.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 260 kg` | 66.6 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00213 s over 1.37 m` | 117.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 633 x 0.944/(4 x (1.000 + 0.017))` | 147.0 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00213 s over 1.37 m` | 351.3 | 275.2 | -21.7 % | -40.5 | 17.7 | downstream run 1.37 m → 1.745 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 694 x 0.989/(4 x 0.314)` | 545.2 | 542.8 | -0.4 % | -42.1 | 10.9 | primary length 0.300 m → 0.301 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.140 + 0.017))` | 552.0 | 480.8 | -12.9 % | -39.1 | 26.9 | runner length 0.140 m → 0.181 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00213 s over 1.37 m` | 585.6 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 661/(2 x 0.350)` | 944.5 | 823.1 | -12.9 % | -48.6 | 18.3 | chamber length 0.350 m, volume 2.5 L → 0.402 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 694 x 0.989/(4 x 0.314)` | 1635.6 | 1656.4 | +1.3 % | -53.6 | 28.4 | primary length 0.300 m → 0.296 m |
| expansion chamber, second pass band | `nc/2L = 2 x 661/(2 x 0.350)` | 1889.0 | 1849.8 | -2.1 % | -73.2 | 7.3 | chamber length 0.350 m, volume 2.5 L → 0.357 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 275.2 | -40.5 | 17.7 |
| 480.8 | -39.1 | 26.9 |
| 542.8 | -42.1 | 10.9 |
| 823.1 | -48.6 | 18.3 |
| 1217.6 | -56.2 | 13.6 |
| 1656.4 | -53.6 | 28.4 |
| 1849.8 | -73.2 | 7.3 |
| 2507.8 | -74.6 | 13.2 |
| 3311.3 | -74.8 | 12.7 |
| 3652.9 | -75.4 | 10.4 |
| 4142.3 | -73.5 | 14.8 |
| 4885.7 | -74.4 | 16.8 |
| 5816.6 | -79.7 | 19.2 |
| 6617.8 | -89.0 | 9.9 |
| 7317.2 | -90.8 | 8.1 |
| 8139.3 | -88.5 | 14.2 |
| 9034.1 | -87.8 | 17.0 |
| 12376.5 | -98.7 | 11.8 |
| 13281.1 | -98.8 | 24.0 |
| 16592.8 | -103.9 | 13.8 |
| 17420.3 | -104.5 | 10.4 |
| 18246.3 | -108.2 | 7.9 |
| 20861.9 | -110.8 | 8.0 |
| 21561.0 | -109.5 | 13.0 |

Noise floor tilt over 200 Hz-12 kHz: **-11.8 dB/octave**.


## 2-Rotor Wankel

> Four firings per cycle from two rotors: no valvetrain, no beat, just buzz.

2.6 L  4 cyl  10.0:1  ·  firing order 2  ·  950-8196 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -19.1 | — |
| 1 | null | -22.4 | — |
| 1.5 | null | -33.2 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -26.1 | — |
| 3 | null | -24.7 | — |
| 3.5 | null | -43.0 | — |
| 4 | +0.0 | -0.5 | -0.5 |

Driven orders: rms **0.3 dB**, mean -0.2 dB, worst -0.5 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -19.1 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 95 kg` | 110.1 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00222 s over 1.52 m` | 112.5 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 673 x 0.976/(4 x (1.000 + 0.018))` | 161.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 746 x 0.995/(4 x 0.620)` | 299.7 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00222 s over 1.52 m` | 337.6 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.200 + 0.016))` | 401.9 | 404.5 | +0.6 % | -53.3 | 8.2 | runner length 0.200 m → 0.215 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00222 s over 1.52 m` | 562.6 | 530.0 | -5.8 % | -39.7 | 10.6 | downstream run 1.52 m → 1.612 m |
| absorptive silencer, first pass band | `c/2L = 707/(2 x 0.500)` | 706.7 | 670.2 | -5.2 % | -37.9 | 22.2 | silencer length 0.500 m → 0.527 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 746 x 0.995/(4 x 0.620)` | 899.1 | 991.6 | +10.3 % | -48.8 | 9.8 | primary length 0.600 m → 0.544 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 59.0 | -42.5 | 11.3 |
| 213.3 | -34.9 | 23.0 |
| 404.5 | -53.3 | 8.2 |
| 530.0 | -39.7 | 10.6 |
| 670.2 | -37.9 | 22.2 |
| 801.2 | -39.7 | 7.5 |
| 991.6 | -48.8 | 9.8 |
| 1856.8 | -60.1 | 14.3 |
| 2684.7 | -72.2 | 14.2 |
| 3467.9 | -75.6 | 11.4 |
| 4079.8 | -81.4 | 5.5 |
| 6720.0 | -89.3 | 6.2 |
| 8661.4 | -95.6 | 8.9 |
| 9683.2 | -95.3 | 8.6 |
| 12061.0 | -102.3 | 7.4 |
| 13675.0 | -100.8 | 10.0 |
| 14471.0 | -104.3 | 11.1 |
| 16056.6 | -104.2 | 7.7 |
| 16875.3 | -103.8 | 11.5 |
| 18453.0 | -113.2 | 6.8 |
| 19247.8 | -111.5 | 7.8 |
| 20035.2 | -106.0 | 15.3 |
| 20837.6 | -106.5 | 9.3 |
| 23212.6 | -118.2 | 8.4 |

Noise floor tilt over 200 Hz-12 kHz: **-9.5 dB/octave**.


## Turbo Inline-4

> Even 180 deg firing under a small fast single: bark, whistle, flutter.

2.0 L  4 cyl  9.6:1  ·  firing order 2  ·  820-6897 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -31.4 | — |
| 1 | null | -26.0 | — |
| 1.5 | null | — | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -46.4 | — |
| 3 | null | -34.7 | — |
| 3.5 | null | -35.7 | — |
| 4 | +0.0 | -10.1 | -10.1 |

Driven orders: rms **7.2 dB**, mean -5.1 dB, worst -10.1 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -26.0 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 125 kg` | 96.0 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00243 s over 1.62 m` | 102.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 654 x 0.993/(4 x (1.200 + 0.018))` | 133.3 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.5 L` | 163.7 | 200.2 | +22.3 % | -35.2 | 24.8 |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00243 s over 1.62 m` | 308.6 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.240 + 0.017))` | 337.4 | 394.5 | +16.9 % | -47.5 | 18.8 | runner length 0.240 m → 0.220 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00243 s over 1.62 m` | 514.3 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 759 x 0.996/(4 x 0.366)` | 517.2 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 703/(2 x 0.400)` | 879.4 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 759 x 0.996/(4 x 0.366)` | 1551.7 | 1645.2 | +6.0 % | -60.6 | 29.3 | primary length 0.350 m → 0.330 m |
| expansion chamber, second pass band | `nc/2L = 2 x 703/(2 x 0.400)` | 1758.7 | not found | — | — | — |  |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 200.2 | -35.2 | 24.8 |
| 394.5 | -47.5 | 18.8 |
| 1131.9 | -52.2 | 15.7 |
| 1645.2 | -60.6 | 29.3 |
| 3204.4 | -75.3 | 10.6 |
| 3668.8 | -75.6 | 9.4 |
| 3959.2 | -73.6 | 15.8 |
| 4736.4 | -74.2 | 14.2 |
| 6368.5 | -71.5 | 17.3 |
| 8913.8 | -93.8 | 9.0 |
| 9097.4 | -92.0 | 11.3 |
| 10048.2 | -86.2 | 21.6 |
| 13137.5 | -85.9 | 23.3 |
| 14197.4 | -103.1 | 15.4 |
| 14640.5 | -104.9 | 12.0 |
| 14955.9 | -104.0 | 15.7 |
| 15454.6 | -105.2 | 10.5 |
| 15747.6 | -104.0 | 18.3 |
| 16242.6 | -105.3 | 13.9 |
| 20058.8 | -110.6 | 11.4 |
| 20830.3 | -107.4 | 17.1 |
| 21320.9 | -111.0 | 9.5 |
| 21609.8 | -111.1 | 13.6 |
| 22093.0 | -110.8 | 15.6 |

Noise floor tilt over 200 Hz-12 kHz: **-8.2 dB/octave**.


## Twin-turbo V8

> Hot-vee twins over the 90-180-270-180 burble: offbeat, but muffled.

4.0 L  8 cyl  10.0:1  ·  firing order 4  ·  760-7097 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -20.7 | -9.4 |
| 1 | null | -19.4 | — |
| 1.5 | -3.7 | -20.8 | -17.1 |
| 2 | null | -31.1 | — |
| 2.5 | -3.7 | -16.5 | -12.8 |
| 3 | null | -49.4 | — |
| 3.5 | -11.4 | -19.2 | -7.8 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -38.3 | -27.0 |
| 5 | null | -26.1 | — |
| 5.5 | -3.7 | -24.8 | -21.1 |
| 6 | null | -28.6 | — |
| 6.5 | -3.7 | -19.4 | -15.7 |
| 7 | null | -33.6 | — |
| 7.5 | -11.4 | -48.8 | -37.4 |
| 8 | +0.0 | -15.7 | -15.7 |

Driven orders: rms **19.2 dB**, mean -16.4 dB, worst -37.4 dB on order 7.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -19.4 dB on order 1 (pass).

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
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 686 x 0.994/(4 x 0.468)` | 364.0 | 368.5 | +1.2 % | -47.7 | 11.4 | primary length 0.450 m → 0.445 m |
| expansion chamber, first pass band | `nc/2L = 1 x 653/(2 x 0.550)` | 593.8 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 686 x 0.994/(4 x 0.468)` | 1092.1 | 1112.8 | +1.9 % | -53.4 | 22.9 | primary length 0.450 m → 0.442 m |
| expansion chamber, second pass band | `nc/2L = 2 x 653/(2 x 0.550)` | 1187.6 | not found | — | — | — |  |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 368.5 | -47.7 | 11.4 |
| 1112.8 | -53.4 | 22.9 |
| 1551.4 | -70.9 | 18.4 |
| 3021.8 | -71.1 | 19.5 |
| 3759.5 | -72.9 | 12.9 |
| 5682.3 | -82.8 | 25.2 |
| 8337.6 | -91.3 | 13.4 |
| 9463.0 | -88.3 | 19.0 |
| 11707.1 | -100.9 | 19.5 |
| 12088.1 | -103.6 | 14.0 |
| 12432.2 | -101.4 | 20.2 |
| 12855.7 | -102.6 | 15.1 |
| 15793.7 | -109.8 | 15.4 |
| 16236.2 | -107.3 | 13.0 |
| 16975.4 | -107.2 | 24.0 |
| 18411.9 | -113.2 | 15.1 |
| 18843.9 | -119.2 | 13.9 |
| 20315.1 | -113.7 | 12.1 |
| 21036.9 | -113.0 | 19.8 |
| 21417.0 | -114.0 | 12.4 |
| 21747.2 | -114.6 | 11.8 |
| 22172.5 | -117.0 | 14.9 |
| 22477.2 | -117.8 | 12.9 |
| 22917.1 | -123.5 | 13.3 |

Noise floor tilt over 200 Hz-12 kHz: **-7.7 dB/octave**.


## Turbo Inline-6

> Even 120 deg firing, no gaps at all, and a big lazy single over it.

3.0 L  6 cyl  9.2:1  ·  firing order 3  ·  780-7197 rpm over 8.0 s

Reference: the crank itself: 6 firings a cycle on 1 bank

### Order balance [dB relative to order 3]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -21.1 | — |
| 1 | null | -26.5 | — |
| 1.5 | null | -28.5 | — |
| 2 | null | -33.9 | — |
| 2.5 | null | — | — |
| **3** | +0.0 | +0.0 | +0.0 |
| 3.5 | null | -42.3 | — |
| 4 | null | -37.4 | — |
| 4.5 | null | -39.1 | — |
| 5 | null | -34.6 | — |
| 5.5 | null | — | — |
| 6 | +0.0 | -7.4 | -7.4 |

Driven orders: rms **5.2 dB**, mean -3.7 dB, worst -7.4 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -21.1 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00363 s over 2.22 m` | 68.9 | not found | — | — | — |  |
| block, first bending mode | `mass law on 195 kg` | 76.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 602 x 0.980/(4 x (1.600 + 0.021))` | 91.0 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 3.5 L` | 159.4 | 137.6 | -13.7 % | -32.2 | 26.0 |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00363 s over 2.22 m` | 206.8 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.300 + 0.018))` | 272.9 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 687 x 0.995/(4 x 0.497)` | 343.6 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00363 s over 2.22 m` | 344.6 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 642/(2 x 0.600)` | 534.9 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 687 x 0.995/(4 x 0.497)` | 1030.9 | 1026.0 | -0.5 % | -53.4 | 16.3 | primary length 0.480 m → 0.482 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 137.6 | -32.2 | 26.0 |
| 698.3 | -47.2 | 18.8 |
| 1026.0 | -53.4 | 16.3 |
| 1412.9 | -66.7 | 13.6 |
| 1727.7 | -65.5 | 19.6 |
| 2822.9 | -67.3 | 17.0 |
| 4212.8 | -81.2 | 14.7 |
| 4493.6 | -82.9 | 15.2 |
| 5549.6 | -82.7 | 13.8 |
| 6648.4 | -91.3 | 15.6 |
| 6960.6 | -95.0 | 13.4 |
| 8399.6 | -105.9 | 14.0 |
| 9049.5 | -102.1 | 14.5 |
| 9715.1 | -103.5 | 13.2 |
| 10133.6 | -102.0 | 17.1 |
| 10798.3 | -101.4 | 25.9 |
| 11168.0 | -105.6 | 13.4 |
| 11467.8 | -105.3 | 15.9 |
| 14639.6 | -112.6 | 16.1 |
| 14951.7 | -112.6 | 13.3 |
| 15360.8 | -111.3 | 19.4 |
| 20572.5 | -118.6 | 13.3 |
| 20851.1 | -117.9 | 13.4 |
| 21265.4 | -116.8 | 20.6 |

Noise floor tilt over 200 Hz-12 kHz: **-10.1 dB/octave**.


## Turbodiesel I4

> No spark at all: a premixed spike, an iron block, and clatter.

2.0 L  4 cyl  21.5:1  ·  firing order 2  ·  800-4998 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -4.2 | — |
| 1 | null | +3.2 | — |
| 1.5 | null | -11.1 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -21.3 | — |
| 3 | null | -7.2 | — |
| 3.5 | null | — | — |
| 4 | +0.0 | -2.9 | -2.9 |

Driven orders: rms **2.1 dB**, mean -1.5 dB, worst -2.9 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest +3.2 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 46.4 | not found | — | — | — |  |
| block, first bending mode | `mass law on 190 kg` | 77.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 512 x 0.992/(4 x (1.300 + 0.017))` | 96.5 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 139.2 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.8 L` | 153.7 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 232.1 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.016))` | 367.1 | 399.9 | +8.9 % | -56.4 | 18.7 | runner length 0.220 m → 0.217 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 594 x 0.997/(4 x 0.315)` | 470.1 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 569/(2 x 0.550)` | 517.6 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 551/(2 x 0.500)` | 550.5 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 551/(2 x 0.500)` | 1101.1 | 1082.9 | -1.6 % | -74.3 | 10.2 | chamber length 0.500 m, volume 22.4 L → 0.508 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 594 x 0.997/(4 x 0.315)` | 1410.2 | 1680.3 | +19.2 % | -85.6 | 13.2 | primary length 0.300 m → 0.252 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 399.9 | -56.4 | 18.7 |
| 847.8 | -67.8 | 12.3 |
| 1082.9 | -74.3 | 10.2 |
| 1680.3 | -85.6 | 13.2 |
| 3043.2 | -73.9 | 11.4 |
| 3592.6 | -70.0 | 14.7 |
| 4188.1 | -69.1 | 32.0 |
| 4748.0 | -72.9 | 17.3 |
| 5494.3 | -86.5 | 10.4 |
| 5695.7 | -91.6 | 8.3 |
| 5824.7 | -90.6 | 10.9 |
| 5954.5 | -92.0 | 8.8 |
| 6219.2 | -88.4 | 8.4 |
| 6352.9 | -86.7 | 9.2 |
| 6488.4 | -86.7 | 8.1 |
| 7184.8 | -83.2 | 19.9 |
| 7767.9 | -86.7 | 8.8 |
| 8374.8 | -94.0 | 11.1 |
| 8847.2 | -88.9 | 9.0 |
| 10008.0 | -82.3 | 27.9 |
| 13082.3 | -107.1 | 24.9 |
| 16904.2 | -112.6 | 26.9 |
| 20554.4 | -117.3 | 17.2 |
| 23334.2 | -132.1 | 10.4 |

Noise floor tilt over 200 Hz-12 kHz: **-4.3 dB/octave**.


## Big Single

> One firing every two turns: the crank itself is the rhythm.

0.7 L  1 cyl  12.0:1  ·  firing order 0.5  ·  1250-6697 rpm over 8.0 s

Reference: the crank itself: 1 firings a cycle on 1 bank

### Order balance [dB relative to order 0.5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| **0.5** | +0.0 | +0.0 | +0.0 |
| 1 | +0.0 | +12.4 | +12.4 |

Driven orders: rms **8.8 dB**, mean +6.2 dB, worst +12.4 dB on order 1 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: none resolvable.

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 45 kg` | 120.0 | 115.5 | -3.8 % | -29.8 | 24.1 |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 736 x 0.715/(4 x 0.637)` | 206.4 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00103 s over 0.72 m` | 241.6 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 690 x 0.853/(4 x (0.350 + 0.015))` | 403.5 | 315.8 | -21.7 % | -45.8 | 4.3 | tailpipe length 0.350 m, mouth radius 0.024 m → 0.466 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.020))` | 434.6 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 736 x 0.715/(4 x 0.637)` | 619.1 | 551.7 | -10.9 % | -38.6 | 21.1 | primary length 0.620 m → 0.696 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00103 s over 0.72 m` | 724.8 | 720.3 | -0.6 % | -50.1 | 4.3 | downstream run 0.72 m → 0.729 m |
| absorptive silencer, first pass band | `c/2L = 711/(2 x 0.360)` | 987.5 | 1007.0 | +2.0 % | -53.1 | 6.8 | silencer length 0.360 m → 0.353 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00103 s over 0.72 m` | 1208.0 | 1287.7 | +6.6 % | -55.2 | 21.4 | downstream run 0.72 m → 0.680 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 115.5 | -29.8 | 24.1 |
| 315.8 | -45.8 | 4.3 |
| 551.7 | -38.6 | 21.1 |
| 720.3 | -50.1 | 4.3 |
| 792.3 | -39.1 | 15.7 |
| 1007.0 | -53.1 | 6.8 |
| 1287.7 | -55.2 | 21.4 |
| 1439.0 | -59.9 | 4.4 |
| 1916.6 | -63.4 | 5.0 |
| 2156.0 | -62.6 | 12.1 |
| 2386.0 | -63.4 | 4.2 |
| 2872.3 | -63.2 | 5.1 |
| 3611.2 | -67.8 | 6.3 |
| 4353.8 | -78.6 | 14.2 |
| 4932.3 | -83.2 | 4.4 |
| 5555.9 | -82.4 | 4.6 |
| 6000.8 | -83.3 | 8.0 |
| 8707.6 | -92.6 | 12.1 |
| 12002.9 | -102.1 | 13.4 |
| 15601.6 | -105.6 | 17.4 |
| 18135.4 | -109.3 | 10.3 |
| 18920.0 | -110.0 | 4.4 |
| 20625.6 | -115.7 | 5.2 |
| 22133.4 | -113.6 | 11.0 |

Noise floor tilt over 200 Hz-12 kHz: **-8.6 dB/octave**.


## Porsche 911 GT3 Cup (992)

> 4.0L flat-six at 8750 rpm: screaming boxer harmonics and sequential gear whine.

4.0 L  6 cyl  13.3:1  ·  firing order 3  ·  1100-8746 rpm over 8.0 s

Reference: the crank itself: 6 firings a cycle on 2 banks

### Order balance [dB relative to order 3]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -30.4 | — |
| 1 | null | -22.8 | — |
| 1.5 | +0.0 | -13.9 | -13.9 |
| 2 | null | -33.9 | — |
| 2.5 | null | -38.0 | — |
| **3** | +0.0 | +0.0 | +0.0 |
| 3.5 | null | -39.8 | — |
| 4 | null | -39.4 | — |
| 4.5 | +0.0 | -26.5 | -26.5 |
| 5 | null | -37.5 | — |
| 5.5 | null | -55.5 | — |
| 6 | +0.0 | -19.1 | -19.1 |

Driven orders: rms **17.7 dB**, mean -14.9 dB, worst -26.5 dB on order 4.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -22.8 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 145 kg` | 89.1 | 111.1 | +24.7 % | -27.1 | 31.0 |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 658 x 0.978/(4 x (0.750 + 0.025))` | 207.7 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00118 s over 0.78 m` | 212.2 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 3.5 L` | 233.0 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 708 x 0.990/(4 x 0.398)` | 439.6 | 387.1 | -11.9 % | -34.4 | 11.4 | primary length 0.380 m → 0.431 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.160 + 0.020))` | 483.0 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00118 s over 0.78 m` | 636.7 | 675.6 | +6.1 % | -49.3 | 13.6 | downstream run 0.78 m → 0.731 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00118 s over 0.78 m` | 1061.2 | 928.2 | -12.5 % | -48.7 | 14.6 | downstream run 0.78 m → 0.887 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 708 x 0.990/(4 x 0.398)` | 1318.7 | 1288.3 | -2.3 % | -61.5 | 6.5 | primary length 0.380 m → 0.389 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 111.1 | -27.1 | 31.0 |
| 387.1 | -34.4 | 11.4 |
| 675.6 | -49.3 | 13.6 |
| 928.2 | -48.7 | 14.6 |
| 1288.3 | -61.5 | 6.5 |
| 1371.4 | -63.2 | 8.8 |
| 1720.0 | -66.7 | 7.9 |
| 2303.9 | -62.1 | 18.0 |
| 3120.0 | -68.2 | 7.9 |
| 3320.4 | -75.0 | 6.4 |
| 3870.4 | -73.2 | 9.9 |
| 4170.9 | -72.2 | 11.1 |
| 4799.8 | -75.0 | 10.5 |
| 5039.5 | -75.1 | 7.1 |
| 5760.2 | -84.7 | 8.2 |
| 6001.3 | -89.8 | 8.5 |
| 6960.4 | -81.7 | 18.6 |
| 7680.7 | -85.3 | 7.3 |
| 9599.8 | -89.4 | 10.7 |
| 11761.0 | -93.0 | 14.1 |
| 14532.0 | -94.6 | 12.6 |
| 17323.2 | -97.8 | 13.2 |
| 19201.2 | -99.5 | 10.3 |
| 21199.8 | -101.9 | 14.4 |

Noise floor tilt over 200 Hz-12 kHz: **-8.7 dB/octave**.


## Porsche 911 GT3 Cup (997.2)

> 3.8L Mezger flat-six at 8500 rpm: dry-sump mechanical clatter and GT1 lineage.

3.8 L  6 cyl  12.6:1  ·  firing order 3  ·  1150-8496 rpm over 8.0 s

Reference: the crank itself: 6 firings a cycle on 2 banks

### Order balance [dB relative to order 3]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -36.0 | — |
| 1 | null | -28.1 | — |
| 1.5 | +0.0 | -28.2 | -28.2 |
| 2 | null | -39.4 | — |
| 2.5 | null | — | — |
| **3** | +0.0 | +0.0 | +0.0 |
| 3.5 | null | -53.6 | — |
| 4 | null | -42.9 | — |
| 4.5 | +0.0 | -21.4 | -21.4 |
| 5 | null | -35.7 | — |
| 5.5 | null | — | — |
| 6 | +0.0 | -18.1 | -18.1 |

Driven orders: rms **19.9 dB**, mean -16.9 dB, worst -28.2 dB on order 1.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -28.1 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 155 kg` | 86.2 | 97.8 | +13.4 % | -34.6 | 11.3 |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 627 x 0.976/(4 x (0.800 + 0.023))` | 186.0 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00131 s over 0.82 m` | 190.6 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 3.2 L` | 221.6 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 676 x 0.992/(4 x 0.418)` | 401.5 | 399.9 | -0.4 % | -26.7 | 32.2 | primary length 0.400 m → 0.402 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.019))` | 436.4 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00131 s over 0.82 m` | 571.7 | 621.5 | +8.7 % | -42.4 | 20.4 | downstream run 0.82 m → 0.757 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00131 s over 0.82 m` | 952.8 | 868.0 | -8.9 % | -44.8 | 10.2 | downstream run 0.82 m → 0.903 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 676 x 0.992/(4 x 0.418)` | 1204.4 | 1207.5 | +0.3 % | -44.4 | 12.0 | primary length 0.400 m → 0.399 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 97.8 | -34.6 | 11.3 |
| 399.9 | -26.7 | 32.2 |
| 621.5 | -42.4 | 20.4 |
| 868.0 | -44.8 | 10.2 |
| 1207.5 | -44.4 | 12.0 |
| 1419.7 | -61.3 | 6.4 |
| 1658.8 | -62.6 | 9.3 |
| 2014.7 | -53.0 | 21.8 |
| 2790.9 | -60.5 | 12.9 |
| 3587.1 | -70.1 | 8.4 |
| 4564.9 | -69.2 | 12.1 |
| 4756.8 | -69.9 | 5.9 |
| 5441.6 | -72.9 | 9.1 |
| 5999.1 | -88.7 | 6.9 |
| 7200.0 | -78.2 | 19.0 |
| 7922.0 | -83.2 | 5.9 |
| 9051.6 | -86.1 | 6.5 |
| 9600.6 | -85.8 | 10.6 |
| 11519.2 | -90.5 | 11.3 |
| 14232.8 | -92.7 | 11.6 |
| 16799.3 | -96.4 | 13.2 |
| 18717.9 | -98.1 | 6.5 |
| 19440.9 | -97.2 | 10.4 |
| 21277.7 | -99.9 | 14.2 |

Noise floor tilt over 200 Hz-12 kHz: **-8.2 dB/octave**.


## Mercedes-AMG GT3

> 6.2L M159 cross-plane V8: earth-shaking low-frequency thunder through open side-pipes.

6.2 L  8 cyl  12.0:1  ·  firing order 4  ·  950-7497 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -11.1 | +0.3 |
| 1 | null | -27.5 | — |
| 1.5 | -3.7 | +2.8 | +6.5 |
| 2 | null | -32.8 | — |
| 2.5 | -3.7 | -8.8 | -5.1 |
| 3 | null | — | — |
| 3.5 | -11.4 | -14.8 | -3.4 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -11.4 | -0.0 |
| 5 | null | — | — |
| 5.5 | -3.7 | -1.2 | +2.5 |
| 6 | null | -16.7 | — |
| 6.5 | -3.7 | -5.8 | -2.1 |
| 7 | null | — | — |
| 7.5 | -11.4 | -32.2 | -20.9 |
| 8 | +0.0 | -12.2 | -12.2 |

Driven orders: rms **8.2 dB**, mean -3.5 dB, worst -20.9 dB on order 7.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -16.7 dB on order 6 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 205 kg` | 75.0 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 5.0 L` | 186.4 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 702 x 0.971/(4 x (0.600 + 0.020))` | 274.8 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00088 s over 0.62 m` | 282.9 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.240 + 0.020))` | 334.2 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 743 x 0.991/(4 x 0.519)` | 354.9 | 428.1 | +20.6 % | -34.2 | 10.7 | primary length 0.500 m → 0.415 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00088 s over 0.62 m` | 848.8 | 672.7 | -20.7 % | -33.1 | 21.7 | downstream run 0.62 m → 0.783 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 743 x 0.991/(4 x 0.519)` | 1064.7 | 1236.7 | +16.2 % | -45.7 | 19.3 | primary length 0.500 m → 0.430 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00088 s over 0.62 m` | 1414.7 | 1469.4 | +3.9 % | -50.6 | 10.3 | downstream run 0.62 m → 0.597 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 105.4 | -24.5 | 31.4 |
| 428.1 | -34.2 | 10.7 |
| 672.7 | -33.1 | 21.7 |
| 1236.7 | -45.7 | 19.3 |
| 1469.4 | -50.6 | 10.3 |
| 1773.4 | -50.9 | 22.5 |
| 2061.1 | -54.0 | 14.4 |
| 2300.5 | -51.7 | 20.9 |
| 2539.1 | -57.7 | 10.0 |
| 3058.2 | -59.6 | 13.8 |
| 3626.2 | -63.4 | 12.4 |
| 4424.6 | -66.5 | 13.7 |
| 4992.2 | -70.1 | 10.4 |
| 5560.5 | -73.9 | 12.4 |
| 6359.9 | -76.0 | 9.1 |
| 10103.6 | -89.2 | 8.4 |
| 10758.9 | -90.6 | 7.2 |
| 12109.0 | -94.4 | 7.4 |
| 17340.5 | -103.4 | 8.2 |
| 17759.6 | -104.2 | 7.2 |
| 19323.1 | -105.4 | 12.3 |
| 21626.6 | -108.5 | 14.3 |
| 22856.2 | -119.3 | 9.2 |
| 23594.4 | -123.9 | 7.4 |

Noise floor tilt over 200 Hz-12 kHz: **-8.2 dB/octave**.


## Ferrari 458 Italia GT3

> 4.5L flat-plane V8 screaming to 9200 rpm: tuned 4-into-1 race extractors and pure tenor howl.

4.5 L  8 cyl  13.0:1  ·  firing order 4  ·  1100-8946 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -14.2 | — |
| 1 | null | -11.7 | — |
| 1.5 | null | -25.1 | — |
| 2 | +0.0 | -5.9 | -5.9 |
| 2.5 | null | -29.9 | — |
| 3 | null | -25.0 | — |
| 3.5 | null | -18.6 | — |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | null | -30.6 | — |
| 5 | null | -28.6 | — |
| 5.5 | null | -21.2 | — |
| 6 | +0.0 | -17.4 | -17.4 |
| 6.5 | null | -23.9 | — |
| 7 | null | -30.8 | — |
| 7.5 | null | — | — |
| 8 | +0.0 | -6.2 | -6.2 |

Driven orders: rms **9.7 dB**, mean -7.4 dB, worst -17.4 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -11.7 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 170 kg` | 82.3 | 82.4 | +0.1 % | -41.8 | 8.3 |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 657 x 0.977/(4 x (0.700 + 0.028))` | 220.6 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00111 s over 0.73 m` | 225.8 | 246.9 | +9.4 % | -37.5 | 17.5 | downstream run 0.73 m → 0.666 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 700 x 0.989/(4 x 0.417)` | 414.6 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.150 + 0.015))` | 526.9 | 553.4 | +5.0 % | -42.1 | 15.0 | runner length 0.150 m → 0.157 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00111 s over 0.73 m` | 677.3 | 638.8 | -5.7 % | -48.4 | 9.3 | downstream run 0.73 m → 0.772 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00111 s over 0.73 m` | 1128.9 | 1126.8 | -0.2 % | -56.2 | 8.2 | downstream run 0.73 m → 0.729 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 700 x 0.989/(4 x 0.417)` | 1243.7 | 1276.4 | +2.6 % | -60.9 | 10.4 | primary length 0.400 m → 0.390 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 82.4 | -41.8 | 8.3 |
| 246.9 | -37.5 | 17.5 |
| 553.4 | -42.1 | 15.0 |
| 638.8 | -48.4 | 9.3 |
| 781.7 | -44.2 | 13.6 |
| 1126.8 | -56.2 | 8.2 |
| 1276.4 | -60.9 | 10.4 |
| 1496.7 | -59.7 | 11.5 |
| 2344.0 | -58.4 | 23.4 |
| 2479.1 | -65.7 | 9.0 |
| 3185.1 | -68.8 | 7.6 |
| 3469.1 | -67.3 | 16.7 |
| 4100.3 | -76.2 | 11.0 |
| 4443.8 | -72.7 | 19.4 |
| 6375.0 | -85.1 | 8.7 |
| 7593.3 | -84.2 | 13.9 |
| 8637.8 | -90.4 | 10.5 |
| 9594.9 | -90.2 | 13.5 |
| 10640.4 | -92.4 | 8.0 |
| 12799.7 | -96.6 | 13.8 |
| 15028.6 | -99.3 | 13.7 |
| 15830.8 | -99.5 | 8.1 |
| 18850.0 | -104.4 | 8.7 |
| 20818.1 | -106.4 | 14.3 |

Noise floor tilt over 200 Hz-12 kHz: **-8.1 dB/octave**.


## Audi R8 LMS GT3

> 5.2L 90 deg V10 at 8800 rpm: uneven-bank acoustic fire and straight-cut race gear scream.

5.2 L  10 cyl  12.5:1  ·  firing order 5  ·  1050-8796 rpm over 8.0 s

Reference: the crank itself: 10 firings a cycle on 2 banks

### Order balance [dB relative to order 5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -22.3 | -10.3 | — |
| 1 | -8.4 | +8.8 | +17.2 |
| 1.5 | -5.6 | -24.6 | -19.0 |
| 2 | -12.6 | -14.7 | — |
| 2.5 | -14.0 | -14.2 | — |
| 3 | -12.6 | -0.4 | — |
| 3.5 | -5.6 | -25.9 | -20.3 |
| 4 | -8.4 | -25.3 | -16.9 |
| 4.5 | -22.3 | -55.5 | — |
| **5** | +0.0 | +0.0 | +0.0 |
| 5.5 | -22.3 | -30.7 | — |
| 6 | -8.4 | -30.4 | -22.0 |
| 6.5 | -5.6 | -33.0 | -27.3 |
| 7 | -12.6 | -14.0 | — |
| 7.5 | -14.0 | -25.4 | — |
| 8 | -12.6 | -29.0 | — |
| 8.5 | -5.6 | -33.6 | -28.0 |
| 9 | -8.4 | -33.0 | -24.6 |
| 9.5 | -22.3 | -26.8 | — |
| 10 | +0.0 | -8.5 | -8.5 |

Driven orders: rms **20.1 dB**, mean -14.9 dB, worst -28.0 dB on order 8.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -0.4 dB on order 3 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 210 kg` | 74.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 641 x 0.969/(4 x (0.800 + 0.025))` | 188.1 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00129 s over 0.83 m` | 194.1 | 234.8 | +21.0 % | -41.5 | 11.5 | downstream run 0.83 m → 0.682 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.019))` | 436.4 | 409.8 | -6.1 % | -38.6 | 23.6 | runner length 0.180 m → 0.212 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 685 x 0.990/(4 x 0.366)` | 463.4 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00129 s over 0.83 m` | 582.2 | 639.6 | +9.9 % | -46.8 | 8.1 | downstream run 0.83 m → 0.751 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00129 s over 0.83 m` | 970.4 | 955.3 | -1.6 % | -56.5 | 6.1 | downstream run 0.83 m → 0.839 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 685 x 0.990/(4 x 0.366)` | 1390.2 | 1263.0 | -9.2 % | -52.2 | 16.9 | primary length 0.350 m → 0.385 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 136.0 | -36.9 | 17.3 |
| 234.8 | -41.5 | 11.5 |
| 409.8 | -38.6 | 23.6 |
| 639.6 | -46.8 | 8.1 |
| 800.4 | -51.8 | 12.9 |
| 955.3 | -56.5 | 6.1 |
| 1263.0 | -52.2 | 16.9 |
| 2026.4 | -60.8 | 18.9 |
| 2881.7 | -65.5 | 14.4 |
| 3968.2 | -78.2 | 13.8 |
| 4799.2 | -79.5 | 9.7 |
| 5498.1 | -79.6 | 8.2 |
| 6445.7 | -89.2 | 8.4 |
| 7200.0 | -88.3 | 14.0 |
| 7921.4 | -90.4 | 7.9 |
| 9839.1 | -94.3 | 10.7 |
| 10605.1 | -97.5 | 5.8 |
| 12368.0 | -100.9 | 10.3 |
| 13438.9 | -102.2 | 6.0 |
| 15057.3 | -103.5 | 11.1 |
| 16025.5 | -103.6 | 7.9 |
| 17800.5 | -107.7 | 6.3 |
| 18478.6 | -107.4 | 8.1 |
| 21382.3 | -109.8 | 11.4 |

Noise floor tilt over 200 Hz-12 kHz: **-8.5 dB/octave**.

