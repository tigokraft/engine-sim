# Calibration

Stage 16 of [the implementation plan](../IMPLEMENTATION_PLAN.md): the synth measured against a reference it did not produce. Regenerate with:

```bash
cargo run --release --example calibrate -- --markdown docs/measurements/calibration.md
```

Every figure below is *relative*: an order against the firing order, a measured frequency against a predicted one, a slope against an octave. That is deliberate and it is the point of the stage. A gain constant anywhere in the chain moves every order by the same number of decibels and cancels out of all of it, so nothing here can be closed by turning something up — only by changing a length, a volume or a radius.

| Engine | Firing order | Order balance rms | Worst order | Crank nulls | Modes placed | Floor tilt |
|---|---:|---:|---:|---:|---:|---:|
| [Inline-4](#inline-4) | 2 | 6.2 dB | -8.8 dB on 4 | -9.7 dB on 1 | 1/3 of 11 | -10.1 dB/oct |
| [Cross-plane V8](#cross-plane-v8) | 4 | 11.7 dB | -16.3 dB on 3.5 | -12.0 dB on 1, not enforced | 1/3 of 11 | -10.4 dB/oct |
| [Flat-plane V8](#flat-plane-v8) | 4 | 16.5 dB | -30.6 dB on 6 | -13.2 dB on 0.5, not enforced | 2/4 of 10 | -10.7 dB/oct |
| [V10](#v10) | 5 | 19.5 dB | -28.6 dB on 8.5 | -7.6 dB on 0.5, not enforced | 1/4 of 10 | -8.4 dB/oct |
| [V12](#v12) | 6 | 13.0 dB | -19.7 dB on 12 | -11.0 dB on 0.5, not enforced | 2/5 of 10 | -14.1 dB/oct |
| [2-Rotor Wankel](#2-rotor-wankel) | 2 | 10.3 dB | -14.6 dB on 4 | -24.0 dB on 0.5 | 3/3 of 9 | -10.1 dB/oct |
| [Turbo Inline-4](#turbo-inline-4) | 2 | 4.4 dB | -6.2 dB on 4 | -13.4 dB on 1, not enforced | 2/5 of 11 | -7.7 dB/oct |
| [Twin-turbo V8](#twin-turbo-v8) | 4 | 15.6 dB | -29.3 dB on 7.5 | -18.1 dB on 1, not enforced | 3/3 of 11 | -7.2 dB/oct |
| [Turbo Inline-6](#turbo-inline-6) | 3 | 11.4 dB | -16.1 dB on 6 | -21.4 dB on 1, not enforced | 3/3 of 10 | -9.7 dB/oct |
| [Turbodiesel I4](#turbodiesel-i4) | 2 | 3.9 dB | -5.5 dB on 4 | +5.4 dB on 1, not enforced | 1/3 of 12 | -4.7 dB/oct |
| [Big Single](#big-single) | 0.5 | 10.7 dB | +15.2 dB on 1 | — | 6/7 of 9 | -7.5 dB/oct |
| [Porsche 911 GT3 Cup (992)](#porsche-911-gt3-cup-992) | 3 | 17.3 dB | -25.0 dB on 4.5 | -21.1 dB on 1, not enforced | 2/5 of 9 | -9.1 dB/oct |
| [Porsche 911 GT3 Cup (997.2)](#porsche-911-gt3-cup-9972) | 3 | 21.5 dB | -28.0 dB on 4.5 | -27.9 dB on 1, not enforced | 4/5 of 9 | -9.5 dB/oct |
| [Mercedes-AMG GT3](#mercedes-amg-gt3) | 4 | 7.4 dB | +16.8 dB on 1.5 | -15.8 dB on 6, not enforced | 1/2 of 9 | -7.2 dB/oct |
| [Ferrari 458 Italia GT3](#ferrari-458-italia-gt3) | 4 | 11.7 dB | -20.8 dB on 6 | -11.4 dB on 0.5, not enforced | 3/4 of 8 | -8.9 dB/oct |
| [Audi R8 LMS GT3](#audi-r8-lms-gt3) | 5 | 21.9 dB | -35.1 dB on 8.5 | -8.5 dB on 3, not enforced | 2/4 of 8 | -9.4 dB/oct |

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
| 0.5 | null | -14.9 | — |
| 1 | null | -9.7 | — |
| 1.5 | null | -19.8 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -37.4 | — |
| 3 | null | -28.8 | — |
| 3.5 | null | -29.7 | — |
| 4 | +0.0 | -8.8 | -8.8 |

Driven orders: rms **6.2 dB**, mean -4.4 dB, worst -8.8 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -9.7 dB on order 1 (**fail**).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00331 s over 2.12 m` | 75.6 | not found | — | — | — |  |
| block, first bending mode | `mass law on 110 kg` | 102.3 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 621 x 0.951/(4 x (1.200 + 0.017))` | 121.5 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.2 L` | 162.1 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00331 s over 2.12 m` | 226.9 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.280 + 0.017))` | 292.0 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00331 s over 2.12 m` | 378.2 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 723 x 0.983/(4 x 0.416)` | 427.4 | 475.7 | +11.3 % | -60.3 | 8.4 | primary length 0.400 m → 0.359 m |
| expansion chamber, first pass band | `nc/2L = 1 x 686/(2 x 0.450)` | 762.7 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 723 x 0.983/(4 x 0.416)` | 1282.2 | 1047.5 | -18.3 % | -59.9 | 17.4 | primary length 0.400 m → 0.490 m |
| expansion chamber, second pass band | `nc/2L = 2 x 686/(2 x 0.450)` | 1525.4 | 1483.1 | -2.8 % | -68.7 | 21.2 | chamber length 0.450 m, volume 9.3 L → 0.463 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 475.7 | -60.3 | 8.4 |
| 1047.5 | -59.9 | 17.4 |
| 1483.1 | -68.7 | 21.2 |
| 2160.8 | -78.5 | 14.9 |
| 2878.8 | -84.5 | 15.4 |
| 3302.1 | -88.0 | 10.7 |
| 3597.9 | -81.3 | 19.6 |
| 4011.7 | -87.0 | 14.6 |
| 4347.1 | -86.6 | 12.7 |
| 4701.6 | -87.6 | 16.3 |
| 5109.4 | -93.1 | 11.6 |
| 5380.7 | -93.8 | 9.5 |
| 5833.4 | -94.3 | 10.2 |
| 6344.5 | -97.8 | 10.5 |
| 8354.5 | -106.6 | 13.2 |
| 8690.6 | -108.9 | 10.4 |
| 9040.5 | -107.8 | 13.3 |
| 9741.5 | -107.2 | 14.2 |
| 12000.4 | -113.8 | 9.6 |
| 14782.6 | -117.3 | 10.6 |
| 15175.6 | -117.1 | 14.8 |
| 18000.3 | -118.9 | 10.2 |
| 19440.5 | -117.6 | 18.5 |
| 22559.6 | -128.3 | 10.3 |

Noise floor tilt over 200 Hz-12 kHz: **-10.1 dB/octave**.


## Cross-plane V8

> 90-180-270-180 gaps on each bank: the offbeat American burble.

5.0 L  8 cyl  11.0:1  ·  firing order 4  ·  750-6997 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -11.3 | +0.0 |
| 1 | null | -12.0 | — |
| 1.5 | -3.7 | -11.4 | -7.7 |
| 2 | null | -14.3 | — |
| 2.5 | -3.7 | -14.5 | -10.8 |
| 3 | null | -22.8 | — |
| 3.5 | -11.4 | -27.7 | -16.3 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -27.2 | -15.9 |
| 5 | null | -23.2 | — |
| 5.5 | -3.7 | -19.8 | -16.1 |
| 6 | null | -13.7 | — |
| 6.5 | -3.7 | -16.4 | -12.7 |
| 7 | null | -26.9 | — |
| 7.5 | -11.4 | — | — |
| 8 | +0.0 | -10.6 | -10.6 |

Driven orders: rms **11.7 dB**, mean -10.0 dB, worst -16.3 dB on order 3.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -12.0 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00446 s over 2.82 m` | 56.0 | not found | — | — | — |  |
| block, first bending mode | `mass law on 210 kg` | 74.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 610 x 0.986/(4 x (1.500 + 0.018))` | 99.0 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 4.8 L` | 140.3 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00446 s over 2.82 m` | 168.1 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.380 + 0.018))` | 218.0 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00446 s over 2.82 m` | 280.2 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 720 x 0.996/(4 x 0.568)` | 315.7 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 679/(2 x 0.650)` | 522.2 | 405.7 | -22.3 % | -53.6 | 19.4 | chamber length 0.650 m, volume 22.1 L → 0.837 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 720 x 0.996/(4 x 0.568)` | 947.1 | 905.2 | -4.4 % | -61.8 | 12.3 | primary length 0.550 m → 0.575 m |
| expansion chamber, second pass band | `nc/2L = 2 x 679/(2 x 0.650)` | 1044.5 | 1199.8 | +14.9 % | -65.0 | 12.1 | chamber length 0.650 m, volume 22.1 L → 0.566 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 405.7 | -53.6 | 19.4 |
| 905.2 | -61.8 | 12.3 |
| 1199.8 | -65.0 | 12.1 |
| 1535.6 | -71.5 | 19.6 |
| 2679.8 | -80.1 | 12.6 |
| 2993.8 | -80.9 | 13.2 |
| 3359.5 | -83.3 | 16.9 |
| 3900.3 | -80.3 | 20.2 |
| 4472.6 | -86.1 | 14.7 |
| 4799.7 | -90.4 | 17.0 |
| 5116.0 | -94.3 | 15.1 |
| 5712.2 | -94.6 | 22.1 |
| 7540.4 | -99.8 | 12.8 |
| 8449.9 | -101.2 | 19.0 |
| 8995.9 | -104.8 | 13.8 |
| 9359.3 | -105.5 | 12.0 |
| 9599.8 | -103.6 | 17.5 |
| 11760.1 | -114.8 | 14.6 |
| 13199.8 | -109.8 | 22.9 |
| 15600.0 | -117.9 | 14.4 |
| 16799.8 | -116.1 | 20.6 |
| 18638.6 | -124.9 | 11.8 |
| 21359.8 | -120.0 | 21.1 |
| 21912.4 | -122.5 | 12.2 |

Noise floor tilt over 200 Hz-12 kHz: **-10.4 dB/octave**.


## Flat-plane V8

> Even 180 deg on both banks: two inline-fours sharing a crank.

4.5 L  8 cyl  12.5:1  ·  firing order 4  ·  900-8596 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -13.2 | — |
| 1 | null | -16.1 | — |
| 1.5 | null | -29.3 | — |
| 2 | +0.0 | +0.1 | +0.1 |
| 2.5 | null | -34.1 | — |
| 3 | null | -24.8 | — |
| 3.5 | null | -28.5 | — |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | null | -37.8 | — |
| 5 | null | -27.5 | — |
| 5.5 | null | -28.8 | — |
| 6 | +0.0 | -30.6 | -30.6 |
| 6.5 | null | -34.3 | — |
| 7 | null | -36.7 | — |
| 7.5 | null | — | — |
| 8 | +0.0 | -12.1 | -12.1 |

Driven orders: rms **16.5 dB**, mean -10.7 dB, worst -30.6 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -13.2 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 180 kg` | 80.0 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00260 s over 1.72 m` | 96.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 649 x 0.985/(4 x (0.900 + 0.020))` | 173.5 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00260 s over 1.72 m` | 288.3 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 709 x 0.993/(4 x 0.437)` | 402.6 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.015))` | 445.8 | 434.1 | -2.6 % | -48.6 | 12.8 | runner length 0.180 m → 0.200 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00260 s over 1.72 m` | 480.4 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 687/(2 x 0.400)` | 858.4 | 1002.6 | +16.8 % | -62.0 | 12.0 | chamber length 0.400 m, volume 9.3 L → 0.342 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 709 x 0.993/(4 x 0.437)` | 1207.8 | 1395.8 | +15.6 % | -71.6 | 11.1 | primary length 0.420 m → 0.363 m |
| expansion chamber, second pass band | `nc/2L = 2 x 687/(2 x 0.400)` | 1716.8 | 1661.5 | -3.2 % | -73.7 | 10.3 | chamber length 0.400 m, volume 9.3 L → 0.413 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 434.1 | -48.6 | 12.8 |
| 1002.6 | -62.0 | 12.0 |
| 1395.8 | -71.6 | 11.1 |
| 1661.5 | -73.7 | 10.3 |
| 2159.9 | -75.0 | 12.6 |
| 2880.1 | -83.1 | 9.1 |
| 3839.8 | -93.1 | 10.8 |
| 4079.8 | -95.7 | 9.5 |
| 4319.9 | -90.8 | 9.1 |
| 4799.7 | -91.2 | 14.6 |
| 5519.8 | -93.9 | 10.0 |
| 6959.9 | -96.6 | 19.0 |
| 8160.1 | -104.8 | 9.5 |
| 9839.7 | -103.4 | 17.6 |
| 10559.7 | -109.1 | 9.7 |
| 10799.9 | -110.2 | 10.3 |
| 12480.1 | -110.3 | 11.4 |
| 13440.1 | -112.1 | 14.1 |
| 15599.8 | -120.0 | 9.3 |
| 16079.2 | -114.4 | 14.1 |
| 18000.2 | -121.0 | 9.1 |
| 18720.3 | -116.5 | 12.8 |
| 21359.8 | -117.9 | 14.0 |
| 23280.1 | -131.1 | 13.2 |

Noise floor tilt over 200 Hz-12 kHz: **-10.7 dB/octave**.


## V10

> 72 deg firing, unevenly split across the banks: metallic and hard.

5.2 L  10 cyl  12.7:1  ·  firing order 5  ·  900-8496 rpm over 8.0 s

Reference: the crank itself: 10 firings a cycle on 2 banks

### Order balance [dB relative to order 5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -22.3 | -7.6 | — |
| 1 | -8.4 | +15.6 | +24.0 |
| 1.5 | -5.6 | -19.3 | -13.6 |
| 2 | -12.6 | -8.3 | — |
| 2.5 | -14.0 | -25.7 | — |
| 3 | -12.6 | -15.4 | — |
| 3.5 | -5.6 | -20.2 | -14.6 |
| 4 | -8.4 | -18.8 | -10.4 |
| 4.5 | -22.3 | -44.1 | — |
| **5** | +0.0 | +0.0 | +0.0 |
| 5.5 | -22.3 | -35.5 | — |
| 6 | -8.4 | -26.1 | -17.7 |
| 6.5 | -5.6 | -29.5 | -23.8 |
| 7 | -12.6 | -29.1 | — |
| 7.5 | -14.0 | -30.0 | — |
| 8 | -12.6 | -18.9 | — |
| 8.5 | -5.6 | -34.2 | -28.6 |
| 9 | -8.4 | -35.3 | -26.9 |
| 9.5 | -22.3 | -45.6 | — |
| 10 | +0.0 | -16.6 | -16.6 |

Driven orders: rms **19.5 dB**, mean -12.8 dB, worst -28.6 dB on order 8.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -7.6 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 220 kg` | 72.4 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 82.6 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 622 x 0.982/(4 x (1.100 + 0.019))` | 136.5 | 132.1 | -3.2 % | -34.1 | 17.0 | tailpipe length 1.100 m, mouth radius 0.031 m → 1.156 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 247.7 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.019))` | 363.3 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 412.8 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 686 x 0.995/(4 x 0.376)` | 453.5 | 500.8 | +10.4 % | -52.0 | 14.0 | primary length 0.360 m → 0.326 m |
| expansion chamber, first pass band | `nc/2L = 1 x 662/(2 x 0.400)` | 827.3 | 960.4 | +16.1 % | -63.7 | 16.0 | chamber length 0.400 m, volume 8.5 L → 0.345 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 686 x 0.995/(4 x 0.376)` | 1360.5 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 662/(2 x 0.400)` | 1654.5 | 2057.7 | +24.4 % | -62.4 | 12.6 | chamber length 0.400 m, volume 8.5 L → 0.322 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 132.1 | -34.1 | 17.0 |
| 500.8 | -52.0 | 14.0 |
| 960.4 | -63.7 | 16.0 |
| 2057.7 | -62.4 | 12.6 |
| 2400.3 | -72.7 | 11.5 |
| 3119.9 | -75.9 | 11.3 |
| 3839.8 | -80.5 | 11.8 |
| 4363.7 | -77.2 | 16.1 |
| 5279.7 | -89.8 | 10.5 |
| 5759.9 | -98.5 | 9.4 |
| 5999.6 | -98.6 | 9.4 |
| 7440.1 | -96.0 | 11.3 |
| 8160.0 | -102.7 | 10.8 |
| 8400.2 | -104.5 | 10.9 |
| 10320.0 | -103.6 | 14.2 |
| 10799.9 | -109.6 | 9.9 |
| 11040.1 | -108.6 | 11.8 |
| 13200.1 | -108.7 | 14.5 |
| 13920.0 | -111.0 | 9.9 |
| 16079.5 | -113.1 | 13.1 |
| 18960.3 | -115.1 | 13.9 |
| 20400.1 | -121.1 | 9.8 |
| 21120.1 | -116.8 | 14.1 |
| 23279.2 | -128.1 | 23.5 |

Noise floor tilt over 200 Hz-12 kHz: **-8.4 dB/octave**.


## V12

> 60 deg firing, even on both banks: no beat left to hear, only pitch.

6.5 L  12 cyl  11.8:1  ·  firing order 6  ·  800-8496 rpm over 8.0 s

Reference: the crank itself: 12 firings a cycle on 2 banks

### Order balance [dB relative to order 6]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -11.0 | — |
| 1 | null | -16.9 | — |
| 1.5 | null | -17.1 | — |
| 2 | null | -23.4 | — |
| 2.5 | null | -38.4 | — |
| 3 | +0.0 | +7.2 | +7.2 |
| 3.5 | null | -19.6 | — |
| 4 | null | -13.7 | — |
| 4.5 | null | -27.1 | — |
| 5 | null | -26.7 | — |
| 5.5 | null | — | — |
| **6** | +0.0 | +0.0 | +0.0 |
| 6.5 | null | -56.7 | — |
| 7 | null | -32.4 | — |
| 7.5 | null | -36.2 | — |
| 8 | null | -33.9 | — |
| 8.5 | null | -39.8 | — |
| 9 | +0.0 | -15.3 | -15.3 |
| 9.5 | null | -43.5 | — |
| 10 | null | -38.1 | — |
| 10.5 | null | -44.3 | — |
| 11 | null | -41.1 | — |
| 11.5 | null | -38.6 | — |
| 12 | +0.0 | -19.7 | -19.7 |

Driven orders: rms **13.0 dB**, mean -6.9 dB, worst -19.7 dB on order 12 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -11.0 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 260 kg` | 66.6 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 119.8 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 648 x 0.946/(4 x (1.000 + 0.017))` | 150.7 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 359.5 | 275.4 | -23.4 % | -44.5 | 10.7 | downstream run 1.37 m → 1.784 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.140 + 0.017))` | 552.0 | 480.5 | -12.9 % | -54.3 | 7.6 | runner length 0.140 m → 0.181 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 713 x 0.989/(4 x 0.314)` | 561.1 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 599.1 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 678/(2 x 0.350)` | 968.6 | 1185.8 | +22.4 % | -63.1 | 17.8 | chamber length 0.350 m, volume 2.5 L → 0.286 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 713 x 0.989/(4 x 0.314)` | 1683.2 | 1553.5 | -7.7 % | -72.9 | 17.5 | primary length 0.300 m → 0.325 m |
| expansion chamber, second pass band | `nc/2L = 2 x 678/(2 x 0.350)` | 1937.2 | 1887.5 | -2.6 % | -81.8 | 10.5 | chamber length 0.350 m, volume 2.5 L → 0.359 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 275.4 | -44.5 | 10.7 |
| 480.5 | -54.3 | 7.6 |
| 1185.8 | -63.1 | 17.8 |
| 1553.5 | -72.9 | 17.5 |
| 1887.5 | -81.8 | 10.5 |
| 2082.1 | -84.0 | 5.8 |
| 2446.5 | -87.9 | 5.2 |
| 3599.9 | -88.3 | 11.4 |
| 3839.7 | -95.9 | 5.4 |
| 4081.7 | -94.3 | 5.9 |
| 4319.8 | -97.7 | 6.0 |
| 4559.5 | -96.7 | 5.9 |
| 4799.8 | -93.1 | 12.8 |
| 5039.8 | -102.4 | 5.1 |
| 5280.0 | -102.9 | 9.6 |
| 5519.8 | -99.3 | 11.8 |
| 7920.7 | -108.1 | 5.7 |
| 8159.6 | -104.8 | 7.5 |
| 9119.8 | -103.6 | 9.6 |
| 13200.9 | -110.6 | 15.7 |
| 15707.8 | -112.3 | 8.1 |
| 17759.7 | -115.9 | 4.8 |
| 18479.9 | -118.9 | 5.3 |
| 21046.7 | -119.9 | 7.9 |

Noise floor tilt over 200 Hz-12 kHz: **-14.1 dB/octave**.


## 2-Rotor Wankel

> Four firings per cycle from two rotors: no valvetrain, no beat, just buzz.

2.6 L  4 cyl  10.0:1  ·  firing order 2  ·  950-8796 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -24.0 | — |
| 1 | null | -24.9 | — |
| 1.5 | null | -31.9 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -37.8 | — |
| 3 | null | -34.4 | — |
| 3.5 | null | -60.0 | — |
| 4 | +0.0 | -14.6 | -14.6 |

Driven orders: rms **10.3 dB**, mean -7.3 dB, worst -14.6 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -24.0 dB on order 0.5 (pass).

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
| absorptive silencer, first pass band | `c/2L = 674/(2 x 0.500)` | 673.9 | 640.2 | -5.0 % | -49.4 | 13.4 | silencer length 0.500 m → 0.526 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 706 x 0.991/(4 x 0.620)` | 847.0 | 799.4 | -5.6 % | -50.6 | 8.9 | primary length 0.600 m → 0.636 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 53.7 | -31.6 | 26.2 |
| 208.3 | -31.9 | 11.4 |
| 385.5 | -50.7 | 8.0 |
| 640.2 | -49.4 | 13.4 |
| 799.4 | -50.6 | 8.9 |
| 960.7 | -54.0 | 8.0 |
| 1144.6 | -50.1 | 15.0 |
| 1567.6 | -66.0 | 6.2 |
| 1826.1 | -63.7 | 14.9 |
| 2771.7 | -75.0 | 11.8 |
| 3359.7 | -77.0 | 12.3 |
| 4080.0 | -81.8 | 8.8 |
| 4767.4 | -84.7 | 7.1 |
| 5389.4 | -87.5 | 7.5 |
| 6479.7 | -88.9 | 8.6 |
| 8160.8 | -103.4 | 10.1 |
| 9119.9 | -102.4 | 7.3 |
| 9600.0 | -101.3 | 12.2 |
| 10559.6 | -101.8 | 5.9 |
| 13200.6 | -107.8 | 6.1 |
| 13680.9 | -105.8 | 9.3 |
| 14401.0 | -111.1 | 7.5 |
| 16800.1 | -109.0 | 12.4 |
| 20159.5 | -112.6 | 13.1 |

Noise floor tilt over 200 Hz-12 kHz: **-10.1 dB/octave**.


## Turbo Inline-4

> Even 180 deg firing under a small fast single: bark, whistle, flutter.

2.0 L  4 cyl  9.6:1  ·  firing order 2  ·  820-6897 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -19.2 | — |
| 1 | null | -13.4 | — |
| 1.5 | null | -29.1 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -33.7 | — |
| 3 | null | -25.9 | — |
| 3.5 | null | — | — |
| 4 | +0.0 | -6.2 | -6.2 |

Driven orders: rms **4.4 dB**, mean -3.1 dB, worst -6.2 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -13.4 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 125 kg` | 96.0 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 101.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 643 x 0.996/(4 x (1.200 + 0.018))` | 131.3 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.5 L` | 163.7 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 303.2 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.240 + 0.017))` | 337.4 | 388.1 | +15.0 % | -51.7 | 9.6 | runner length 0.240 m → 0.224 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 505.3 | 479.3 | -5.1 % | -58.2 | 8.6 | downstream run 1.62 m → 1.706 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 748 x 0.998/(4 x 0.366)` | 510.0 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 692/(2 x 0.400)` | 864.9 | 958.8 | +10.9 % | -67.5 | 8.3 | chamber length 0.400 m, volume 3.1 L → 0.361 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 748 x 0.998/(4 x 0.366)` | 1530.0 | 1149.7 | -24.9 % | -54.3 | 22.2 | primary length 0.350 m → 0.466 m |
| expansion chamber, second pass band | `nc/2L = 2 x 692/(2 x 0.400)` | 1729.7 | 1636.2 | -5.4 % | -63.5 | 28.5 | chamber length 0.400 m, volume 3.1 L → 0.423 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 56.4 | -43.5 | 13.0 |
| 388.1 | -51.7 | 9.6 |
| 479.3 | -58.2 | 8.6 |
| 958.8 | -67.5 | 8.3 |
| 1149.7 | -54.3 | 22.2 |
| 1636.2 | -63.5 | 28.5 |
| 3585.7 | -75.7 | 12.2 |
| 4187.2 | -74.6 | 17.9 |
| 4736.4 | -76.8 | 15.2 |
| 6483.5 | -71.5 | 21.0 |
| 7337.2 | -90.9 | 8.0 |
| 7848.9 | -92.4 | 8.5 |
| 8374.7 | -98.7 | 10.6 |
| 8915.0 | -93.8 | 9.5 |
| 9473.7 | -88.0 | 8.2 |
| 10047.5 | -86.2 | 23.0 |
| 13125.1 | -85.9 | 26.8 |
| 14639.6 | -113.0 | 11.2 |
| 14964.1 | -113.2 | 9.9 |
| 15725.0 | -111.9 | 19.2 |
| 17762.0 | -112.9 | 9.1 |
| 20849.1 | -117.3 | 7.9 |
| 21602.0 | -118.7 | 8.0 |
| 22080.5 | -118.9 | 10.8 |

Noise floor tilt over 200 Hz-12 kHz: **-7.7 dB/octave**.


## Twin-turbo V8

> Hot-vee twins over the 90-180-270-180 burble: offbeat, but muffled.

4.0 L  8 cyl  10.0:1  ·  firing order 4  ·  760-7097 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -12.5 | -1.2 |
| 1 | null | -18.1 | — |
| 1.5 | -3.7 | -10.5 | -6.8 |
| 2 | null | -20.6 | — |
| 2.5 | -3.7 | -9.6 | -5.9 |
| 3 | null | -47.0 | — |
| 3.5 | -11.4 | -29.7 | -18.4 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -32.5 | -21.2 |
| 5 | null | -26.1 | — |
| 5.5 | -3.7 | -22.9 | -19.2 |
| 6 | null | -29.5 | — |
| 6.5 | -3.7 | -14.3 | -10.6 |
| 7 | null | -30.8 | — |
| 7.5 | -11.4 | -40.7 | -29.3 |
| 8 | +0.0 | -14.5 | -14.5 |

Driven orders: rms **15.6 dB**, mean -12.7 dB, worst -29.3 dB on order 7.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -18.1 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00409 s over 2.52 m` | 61.2 | not found | — | — | — |  |
| block, first bending mode | `mass law on 235 kg` | 70.0 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 599 x 0.990/(4 x (1.400 + 0.020))` | 104.5 | 104.0 | -0.5 % | -46.5 | 9.9 | tailpipe length 1.400 m, mouth radius 0.033 m → 1.427 m |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 4.5 L` | 173.8 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00409 s over 2.52 m` | 183.6 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00409 s over 2.52 m` | 306.0 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.260 + 0.018))` | 312.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 691 x 0.996/(4 x 0.468)` | 367.6 | 370.5 | +0.8 % | -49.8 | 23.9 | primary length 0.450 m → 0.446 m |
| expansion chamber, first pass band | `nc/2L = 1 x 657/(2 x 0.550)` | 597.3 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 691 x 0.996/(4 x 0.468)` | 1102.7 | 1112.3 | +0.9 % | -61.8 | 21.2 | primary length 0.450 m → 0.446 m |
| expansion chamber, second pass band | `nc/2L = 2 x 657/(2 x 0.550)` | 1194.6 | not found | — | — | — |  |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 104.0 | -46.5 | 9.9 |
| 370.5 | -49.8 | 23.9 |
| 885.4 | -64.1 | 13.9 |
| 1112.3 | -61.8 | 21.2 |
| 1531.3 | -76.6 | 16.1 |
| 1781.4 | -78.7 | 7.5 |
| 2997.1 | -77.0 | 8.2 |
| 3615.5 | -73.5 | 22.3 |
| 4209.8 | -73.2 | 10.4 |
| 4776.6 | -76.8 | 12.1 |
| 5279.9 | -97.8 | 11.1 |
| 5860.2 | -94.4 | 6.6 |
| 7231.5 | -89.4 | 21.4 |
| 9422.2 | -88.5 | 22.9 |
| 11698.0 | -115.6 | 8.4 |
| 11759.9 | -119.9 | 8.6 |
| 12095.2 | -118.5 | 7.1 |
| 12480.2 | -115.8 | 6.8 |
| 13199.0 | -112.3 | 18.6 |
| 16793.4 | -116.7 | 7.0 |
| 17279.9 | -116.4 | 16.7 |
| 21028.8 | -121.4 | 16.2 |
| 22161.9 | -129.7 | 8.5 |
| 22559.3 | -133.1 | 7.6 |

Noise floor tilt over 200 Hz-12 kHz: **-7.2 dB/octave**.


## Turbo Inline-6

> Even 120 deg firing, no gaps at all, and a big lazy single over it.

3.0 L  6 cyl  9.2:1  ·  firing order 3  ·  780-7197 rpm over 8.0 s

Reference: the crank itself: 6 firings a cycle on 1 bank

### Order balance [dB relative to order 3]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -21.5 | — |
| 1 | null | -21.4 | — |
| 1.5 | null | -24.1 | — |
| 2 | null | -28.5 | — |
| 2.5 | null | — | — |
| **3** | +0.0 | +0.0 | +0.0 |
| 3.5 | null | -41.1 | — |
| 4 | null | -32.8 | — |
| 4.5 | null | -33.6 | — |
| 5 | null | -28.7 | — |
| 5.5 | null | — | — |
| 6 | +0.0 | -16.1 | -16.1 |

Driven orders: rms **11.4 dB**, mean -8.1 dB, worst -16.1 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -21.4 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00362 s over 2.22 m` | 69.1 | not found | — | — | — |  |
| block, first bending mode | `mass law on 195 kg` | 76.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 603 x 0.991/(4 x (1.600 + 0.021))` | 92.2 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 3.5 L` | 159.4 | 148.5 | -6.8 % | -32.6 | 24.6 |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00362 s over 2.22 m` | 207.3 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.300 + 0.018))` | 272.9 | 268.8 | -1.5 % | -39.0 | 11.4 | runner length 0.300 m → 0.323 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00362 s over 2.22 m` | 345.5 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 691 x 0.997/(4 x 0.497)` | 346.7 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 644/(2 x 0.600)` | 537.1 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 691 x 0.997/(4 x 0.497)` | 1040.0 | 1025.0 | -1.4 % | -53.5 | 19.9 | primary length 0.480 m → 0.487 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 148.5 | -32.6 | 24.6 |
| 268.8 | -39.0 | 11.4 |
| 705.5 | -58.2 | 9.4 |
| 1025.0 | -53.5 | 19.9 |
| 1407.5 | -64.1 | 22.4 |
| 1727.8 | -69.1 | 13.1 |
| 2826.6 | -67.5 | 18.5 |
| 3511.3 | -84.1 | 10.7 |
| 4889.1 | -87.0 | 13.3 |
| 5555.9 | -83.3 | 17.0 |
| 6000.0 | -94.5 | 13.4 |
| 6960.3 | -100.5 | 9.9 |
| 7680.4 | -104.4 | 11.3 |
| 8400.2 | -109.6 | 12.3 |
| 10135.1 | -106.9 | 11.8 |
| 10799.9 | -104.6 | 25.9 |
| 11177.3 | -111.3 | 9.3 |
| 11459.3 | -111.2 | 10.7 |
| 12539.2 | -113.0 | 9.3 |
| 14640.1 | -115.8 | 18.0 |
| 15360.0 | -114.8 | 14.9 |
| 20160.6 | -120.5 | 12.1 |
| 20879.7 | -123.8 | 10.9 |
| 21931.8 | -122.2 | 16.9 |

Noise floor tilt over 200 Hz-12 kHz: **-9.7 dB/octave**.


## Turbodiesel I4

> No spark at all: a premixed spike, an iron block, and clatter.

2.0 L  4 cyl  21.5:1  ·  firing order 2  ·  800-4998 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | +0.7 | — |
| 1 | null | +5.4 | — |
| 1.5 | null | -5.6 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -10.1 | — |
| 3 | null | -7.4 | — |
| 3.5 | null | -34.1 | — |
| 4 | +0.0 | -5.5 | -5.5 |

Driven orders: rms **3.9 dB**, mean -2.7 dB, worst -5.5 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest +5.4 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 46.4 | not found | — | — | — |  |
| block, first bending mode | `mass law on 190 kg` | 77.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 511 x 0.995/(4 x (1.300 + 0.017))` | 96.5 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 139.1 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.8 L` | 153.7 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 231.8 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.016))` | 367.1 | 408.7 | +11.3 % | -50.8 | 23.9 | runner length 0.220 m → 0.212 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 595 x 0.998/(4 x 0.315)` | 471.7 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 570/(2 x 0.550)` | 518.2 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 551/(2 x 0.500)` | 550.6 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 551/(2 x 0.500)` | 1101.1 | 1118.9 | +1.6 % | -68.8 | 13.6 | chamber length 0.500 m, volume 22.4 L → 0.492 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 595 x 0.998/(4 x 0.315)` | 1415.1 | 1641.0 | +16.0 % | -78.3 | 16.5 | primary length 0.300 m → 0.259 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 408.7 | -50.8 | 23.9 |
| 849.3 | -57.9 | 18.0 |
| 1118.9 | -68.8 | 13.6 |
| 1641.0 | -78.3 | 16.5 |
| 1678.0 | -79.2 | 8.9 |
| 2036.4 | -76.5 | 15.8 |
| 3043.1 | -73.9 | 11.7 |
| 3592.6 | -69.9 | 14.4 |
| 4187.7 | -68.8 | 32.6 |
| 4748.4 | -73.0 | 16.6 |
| 5494.3 | -86.6 | 9.2 |
| 5824.7 | -90.7 | 11.0 |
| 5954.8 | -91.9 | 8.2 |
| 6086.4 | -91.2 | 8.4 |
| 6219.1 | -88.3 | 8.4 |
| 7184.6 | -83.0 | 19.9 |
| 7768.2 | -87.0 | 8.9 |
| 8375.2 | -93.9 | 10.7 |
| 8845.9 | -88.7 | 8.8 |
| 10002.2 | -82.5 | 27.6 |
| 13725.4 | -106.6 | 25.5 |
| 16798.8 | -111.6 | 25.2 |
| 20543.8 | -117.1 | 17.5 |
| 22781.6 | -131.0 | 11.5 |

Noise floor tilt over 200 Hz-12 kHz: **-4.7 dB/octave**.


## Big Single

> One firing every two turns: the crank itself is the rhythm.

0.7 L  1 cyl  12.0:1  ·  firing order 0.5  ·  1250-7597 rpm over 8.0 s

Reference: the crank itself: 1 firings a cycle on 1 bank

### Order balance [dB relative to order 0.5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| **0.5** | +0.0 | +0.0 | +0.0 |
| 1 | +0.0 | +15.2 | +15.2 |

Driven orders: rms **10.7 dB**, mean +7.6 dB, worst +15.2 dB on order 1 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: none resolvable.

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 45 kg` | 120.0 | 117.3 | -2.3 % | -30.1 | 23.8 |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 259.2 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 808 x 1.000/(4 x 0.637)` | 317.1 | 299.5 | -5.6 % | -40.1 | 12.9 | primary length 0.620 m → 0.657 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.020))` | 434.6 | 422.0 | -2.9 % | -41.8 | 9.3 | runner length 0.180 m → 0.206 m |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 735 x 1.000/(4 x (0.350 + 0.015))` | 503.7 | 479.4 | -4.8 % | -40.5 | 13.5 | tailpipe length 0.350 m, mouth radius 0.024 m → 0.383 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 777.5 | 777.5 | -0.0 % | -42.4 | 9.0 | downstream run 0.72 m → 0.725 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 808 x 1.000/(4 x 0.637)` | 951.4 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 769/(2 x 0.360)` | 1067.8 | 1197.3 | +12.1 % | -53.0 | 13.9 | silencer length 0.360 m → 0.321 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 1295.9 | 1256.1 | -3.1 % | -52.7 | 7.3 | downstream run 0.72 m → 0.748 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 117.3 | -30.1 | 23.8 |
| 299.5 | -40.1 | 12.9 |
| 358.4 | -43.3 | 8.0 |
| 422.0 | -41.8 | 9.3 |
| 479.4 | -40.5 | 13.5 |
| 539.2 | -40.5 | 8.5 |
| 777.5 | -42.4 | 9.0 |
| 1197.3 | -53.0 | 13.9 |
| 1256.1 | -52.7 | 7.3 |
| 1916.1 | -57.1 | 9.0 |
| 2034.9 | -55.3 | 8.5 |
| 2096.0 | -54.3 | 17.6 |
| 2155.7 | -55.8 | 10.6 |
| 2874.4 | -57.0 | 9.7 |
| 4454.1 | -72.6 | 12.8 |
| 5316.6 | -73.0 | 7.6 |
| 8807.0 | -81.0 | 11.8 |
| 11172.9 | -87.7 | 7.9 |
| 13003.0 | -91.9 | 8.7 |
| 15578.5 | -94.5 | 13.5 |
| 18058.4 | -97.0 | 13.6 |
| 19962.4 | -100.5 | 7.5 |
| 20697.1 | -99.4 | 11.0 |
| 22375.3 | -104.9 | 11.4 |

Noise floor tilt over 200 Hz-12 kHz: **-7.5 dB/octave**.


## Porsche 911 GT3 Cup (992)

> 4.0L flat-six at 8750 rpm: screaming boxer harmonics and sequential gear whine.

4.0 L  6 cyl  13.3:1  ·  firing order 3  ·  1100-8746 rpm over 8.0 s

Reference: the crank itself: 6 firings a cycle on 2 banks

### Order balance [dB relative to order 3]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -27.4 | — |
| 1 | null | -21.1 | — |
| 1.5 | +0.0 | -14.7 | -14.7 |
| 2 | null | -33.8 | — |
| 2.5 | null | -45.1 | — |
| **3** | +0.0 | +0.0 | +0.0 |
| 3.5 | null | -42.4 | — |
| 4 | null | -37.4 | — |
| 4.5 | +0.0 | -25.0 | -25.0 |
| 5 | null | -35.9 | — |
| 5.5 | null | -77.5 | — |
| 6 | +0.0 | -19.0 | -19.0 |

Driven orders: rms **17.3 dB**, mean -14.7 dB, worst -25.0 dB on order 4.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -21.1 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 145 kg` | 89.1 | 110.6 | +24.1 % | -27.8 | 26.8 |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 653 x 0.984/(4 x (0.750 + 0.025))` | 207.3 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00119 s over 0.78 m` | 210.7 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 3.5 L` | 233.0 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 703 x 0.993/(4 x 0.398)` | 438.0 | 387.1 | -11.6 % | -35.2 | 12.2 | primary length 0.380 m → 0.430 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.160 + 0.020))` | 483.0 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00119 s over 0.78 m` | 632.0 | 672.6 | +6.4 % | -49.1 | 11.2 | downstream run 0.78 m → 0.729 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00119 s over 0.78 m` | 1053.3 | 928.5 | -11.8 % | -49.1 | 13.5 | downstream run 0.78 m → 0.880 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 703 x 0.993/(4 x 0.398)` | 1314.1 | 1380.6 | +5.1 % | -64.1 | 9.1 | primary length 0.380 m → 0.362 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 110.6 | -27.8 | 26.8 |
| 387.1 | -35.2 | 12.2 |
| 672.6 | -49.1 | 11.2 |
| 928.5 | -49.1 | 13.5 |
| 1380.6 | -64.1 | 9.1 |
| 1722.3 | -68.1 | 10.2 |
| 2158.1 | -67.7 | 6.4 |
| 2257.0 | -66.0 | 16.0 |
| 3120.1 | -71.7 | 8.0 |
| 3321.0 | -77.1 | 6.2 |
| 3833.7 | -75.2 | 8.0 |
| 4194.4 | -76.0 | 7.7 |
| 4799.9 | -78.1 | 10.2 |
| 5122.0 | -76.3 | 7.1 |
| 6000.4 | -91.3 | 7.0 |
| 6959.9 | -83.4 | 17.0 |
| 7680.0 | -87.4 | 7.1 |
| 9024.6 | -92.2 | 6.4 |
| 9599.7 | -92.4 | 8.2 |
| 11760.1 | -95.0 | 12.9 |
| 14549.7 | -98.0 | 8.7 |
| 17280.3 | -100.9 | 12.4 |
| 19199.8 | -102.4 | 9.7 |
| 21186.9 | -105.0 | 11.0 |

Noise floor tilt over 200 Hz-12 kHz: **-9.1 dB/octave**.


## Porsche 911 GT3 Cup (997.2)

> 3.8L Mezger flat-six at 8500 rpm: dry-sump mechanical clatter and GT1 lineage.

3.8 L  6 cyl  12.6:1  ·  firing order 3  ·  1150-8496 rpm over 8.0 s

Reference: the crank itself: 6 firings a cycle on 2 banks

### Order balance [dB relative to order 3]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -33.3 | — |
| 1 | null | -27.9 | — |
| 1.5 | +0.0 | -23.1 | -23.1 |
| 2 | null | -36.5 | — |
| 2.5 | null | -49.8 | — |
| **3** | +0.0 | +0.0 | +0.0 |
| 3.5 | null | -53.8 | — |
| 4 | null | -41.3 | — |
| 4.5 | +0.0 | -28.0 | -28.0 |
| 5 | null | -39.3 | — |
| 5.5 | null | — | — |
| 6 | +0.0 | -23.3 | -23.3 |

Driven orders: rms **21.5 dB**, mean -18.6 dB, worst -28.0 dB on order 4.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -27.9 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 155 kg` | 86.2 | 96.9 | +12.4 % | -29.8 | 19.5 |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 650 x 0.977/(4 x (0.800 + 0.023))` | 193.1 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00126 s over 0.82 m` | 197.7 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 3.2 L` | 221.6 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 711 x 0.992/(4 x 0.418)` | 422.0 | 408.0 | -3.3 % | -30.4 | 25.9 | primary length 0.400 m → 0.414 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.019))` | 436.4 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00126 s over 0.82 m` | 593.0 | 639.9 | +7.9 % | -50.8 | 11.4 | downstream run 0.82 m → 0.762 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00126 s over 0.82 m` | 988.3 | 1077.7 | +9.0 % | -55.4 | 8.7 | downstream run 0.82 m → 0.754 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 711 x 0.992/(4 x 0.418)` | 1266.1 | 1210.9 | -4.4 % | -52.7 | 11.2 | primary length 0.400 m → 0.418 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 96.9 | -29.8 | 19.5 |
| 408.0 | -30.4 | 25.9 |
| 639.9 | -50.8 | 11.4 |
| 863.0 | -49.7 | 14.7 |
| 1077.7 | -55.4 | 8.7 |
| 1210.9 | -52.7 | 11.2 |
| 2085.5 | -57.7 | 22.6 |
| 2160.6 | -70.4 | 8.3 |
| 2880.9 | -70.8 | 11.7 |
| 3839.6 | -76.0 | 11.0 |
| 4559.2 | -74.8 | 13.1 |
| 4797.6 | -76.6 | 8.4 |
| 5519.4 | -82.5 | 8.4 |
| 6240.3 | -89.7 | 7.2 |
| 6479.8 | -89.2 | 7.8 |
| 7200.0 | -84.7 | 17.4 |
| 7920.4 | -88.4 | 8.3 |
| 9599.3 | -91.4 | 10.5 |
| 11520.0 | -95.9 | 11.4 |
| 12479.9 | -95.7 | 7.2 |
| 14160.5 | -97.0 | 12.3 |
| 16799.5 | -100.6 | 13.1 |
| 19439.7 | -102.4 | 8.5 |
| 21281.2 | -104.7 | 11.4 |

Noise floor tilt over 200 Hz-12 kHz: **-9.5 dB/octave**.


## Mercedes-AMG GT3

> 6.2L M159 cross-plane V8: earth-shaking low-frequency thunder through open side-pipes.

6.2 L  8 cyl  12.0:1  ·  firing order 4  ·  950-7497 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | +0.4 | +11.8 |
| 1 | null | -26.3 | — |
| 1.5 | -3.7 | +13.1 | +16.8 |
| 2 | null | -26.9 | — |
| 2.5 | -3.7 | -1.0 | +2.7 |
| 3 | null | -51.1 | — |
| 3.5 | -11.4 | -6.0 | +5.3 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -12.9 | -1.5 |
| 5 | null | -29.1 | — |
| 5.5 | -3.7 | -4.0 | -0.3 |
| 6 | null | -15.8 | — |
| 6.5 | -3.7 | -2.1 | +1.6 |
| 7 | null | -30.8 | — |
| 7.5 | -11.4 | -17.1 | -5.8 |
| 8 | +0.0 | -7.5 | -7.5 |

Driven orders: rms **7.4 dB**, mean +2.3 dB, worst +16.8 dB on order 1.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -15.8 dB on order 6 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 205 kg` | 75.0 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 5.0 L` | 186.4 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 666 x 0.973/(4 x (0.600 + 0.020))` | 261.3 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00093 s over 0.62 m` | 268.5 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.240 + 0.020))` | 334.2 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 700 x 0.992/(4 x 0.519)` | 334.4 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00093 s over 0.62 m` | 805.4 | 680.3 | -15.5 % | -40.1 | 18.6 | downstream run 0.62 m → 0.734 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 700 x 0.992/(4 x 0.519)` | 1003.2 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00093 s over 0.62 m` | 1342.3 | 1277.2 | -4.9 % | -47.4 | 24.1 | downstream run 0.62 m → 0.652 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 103.9 | -21.9 | 31.1 |
| 418.2 | -41.3 | 14.3 |
| 639.6 | -42.9 | 6.2 |
| 680.3 | -40.1 | 18.6 |
| 1277.2 | -47.4 | 24.1 |
| 1478.0 | -55.2 | 8.4 |
| 1752.3 | -55.3 | 18.8 |
| 2037.3 | -57.9 | 11.1 |
| 2335.6 | -57.7 | 16.7 |
| 2585.4 | -60.2 | 10.5 |
| 2905.7 | -65.6 | 6.9 |
| 3080.9 | -62.6 | 15.6 |
| 3664.3 | -67.5 | 10.5 |
| 4317.8 | -72.6 | 11.6 |
| 5046.8 | -75.6 | 8.4 |
| 5640.9 | -77.6 | 9.8 |
| 6147.3 | -81.5 | 6.9 |
| 7538.0 | -86.4 | 6.3 |
| 10092.9 | -93.0 | 7.3 |
| 11950.7 | -97.8 | 7.2 |
| 16745.7 | -105.5 | 9.2 |
| 19526.7 | -107.9 | 10.6 |
| 21656.1 | -111.4 | 14.0 |
| 23378.7 | -125.2 | 7.4 |

Noise floor tilt over 200 Hz-12 kHz: **-7.2 dB/octave**.


## Ferrari 458 Italia GT3

> 4.5L flat-plane V8 screaming to 9200 rpm: tuned 4-into-1 race extractors and pure tenor howl.

4.5 L  8 cyl  13.0:1  ·  firing order 4  ·  1100-8946 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -11.4 | — |
| 1 | null | -15.0 | — |
| 1.5 | null | -28.4 | — |
| 2 | +0.0 | -1.8 | -1.8 |
| 2.5 | null | -29.1 | — |
| 3 | null | -26.2 | — |
| 3.5 | null | -26.0 | — |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | null | -34.0 | — |
| 5 | null | -28.5 | — |
| 5.5 | null | -23.9 | — |
| 6 | +0.0 | -20.8 | -20.8 |
| 6.5 | null | -27.2 | — |
| 7 | null | -34.0 | — |
| 7.5 | null | — | — |
| 8 | +0.0 | -10.6 | -10.6 |

Driven orders: rms **11.7 dB**, mean -8.3 dB, worst -20.8 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -11.4 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 170 kg` | 82.3 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 653 x 0.985/(4 x (0.700 + 0.028))` | 220.7 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00112 s over 0.73 m` | 224.2 | 244.0 | +8.9 % | -37.4 | 16.6 | downstream run 0.73 m → 0.669 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 693 x 0.993/(4 x 0.417)` | 412.2 | 440.5 | +6.9 % | -45.7 | 12.9 | primary length 0.400 m → 0.374 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.150 + 0.015))` | 526.9 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00112 s over 0.73 m` | 672.5 | 789.1 | +17.3 % | -53.5 | 10.7 | downstream run 0.73 m → 0.620 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00112 s over 0.73 m` | 1120.8 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 693 x 0.993/(4 x 0.417)` | 1236.7 | 1278.8 | +3.4 % | -65.1 | 8.3 | primary length 0.400 m → 0.387 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 244.0 | -37.4 | 16.6 |
| 440.5 | -45.7 | 12.9 |
| 789.1 | -53.5 | 10.7 |
| 1278.8 | -65.1 | 8.3 |
| 1638.7 | -64.1 | 12.2 |
| 2333.7 | -70.3 | 12.0 |
| 3185.3 | -76.2 | 7.6 |
| 3501.4 | -78.2 | 8.2 |
| 4041.8 | -82.9 | 8.1 |
| 4402.7 | -79.1 | 17.2 |
| 4799.3 | -85.0 | 7.1 |
| 5460.4 | -81.9 | 8.8 |
| 6427.0 | -90.8 | 5.9 |
| 7440.9 | -90.2 | 13.5 |
| 8228.3 | -97.9 | 6.6 |
| 8640.6 | -98.8 | 6.1 |
| 9598.3 | -96.6 | 9.6 |
| 10562.9 | -96.6 | 9.5 |
| 12000.0 | -103.2 | 5.8 |
| 12721.8 | -102.8 | 11.4 |
| 14999.6 | -105.2 | 8.6 |
| 15837.7 | -105.4 | 10.0 |
| 18848.9 | -108.4 | 7.9 |
| 21537.6 | -110.4 | 9.7 |

Noise floor tilt over 200 Hz-12 kHz: **-8.9 dB/octave**.


## Audi R8 LMS GT3

> 5.2L 90 deg V10 at 8800 rpm: uneven-bank acoustic fire and straight-cut race gear scream.

5.2 L  10 cyl  12.5:1  ·  firing order 5  ·  1050-8796 rpm over 8.0 s

Reference: the crank itself: 10 firings a cycle on 2 banks

### Order balance [dB relative to order 5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -22.3 | -8.5 | — |
| 1 | -8.4 | +11.5 | +19.9 |
| 1.5 | -5.6 | -23.6 | -18.0 |
| 2 | -12.6 | -11.3 | — |
| 2.5 | -14.0 | -25.1 | — |
| 3 | -12.6 | -8.5 | — |
| 3.5 | -5.6 | -24.0 | -18.4 |
| 4 | -8.4 | -23.6 | -15.2 |
| 4.5 | -22.3 | -37.9 | — |
| **5** | +0.0 | +0.0 | +0.0 |
| 5.5 | -22.3 | -43.2 | — |
| 6 | -8.4 | -30.5 | -22.2 |
| 6.5 | -5.6 | -31.3 | -25.7 |
| 7 | -12.6 | -23.6 | — |
| 7.5 | -14.0 | -29.2 | — |
| 8 | -12.6 | -34.0 | — |
| 8.5 | -5.6 | -40.7 | -35.1 |
| 9 | -8.4 | -38.5 | -30.1 |
| 9.5 | -22.3 | -29.9 | — |
| 10 | +0.0 | -14.7 | -14.7 |

Driven orders: rms **21.9 dB**, mean -15.9 dB, worst -35.1 dB on order 8.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -8.5 dB on order 3 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 210 kg` | 74.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 653 x 0.967/(4 x (0.800 + 0.025))` | 191.2 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00126 s over 0.83 m` | 197.8 | 238.5 | +20.6 % | -43.5 | 9.0 | downstream run 0.83 m → 0.684 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.019))` | 436.4 | 411.8 | -5.6 % | -45.2 | 16.5 | runner length 0.180 m → 0.211 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 701 x 0.989/(4 x 0.366)` | 474.2 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00126 s over 0.83 m` | 593.4 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00126 s over 0.83 m` | 988.9 | 958.5 | -3.1 % | -56.2 | 15.7 | downstream run 0.83 m → 0.852 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 701 x 0.989/(4 x 0.366)` | 1422.6 | 1278.8 | -10.1 % | -61.7 | 10.3 | primary length 0.350 m → 0.389 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 137.3 | -34.1 | 17.5 |
| 238.5 | -43.5 | 9.0 |
| 411.8 | -45.2 | 16.5 |
| 798.0 | -59.4 | 10.1 |
| 958.5 | -56.2 | 15.7 |
| 1126.4 | -63.5 | 8.2 |
| 1198.7 | -65.3 | 6.9 |
| 1278.8 | -61.7 | 10.3 |
| 2016.2 | -69.6 | 12.4 |
| 2396.9 | -76.9 | 7.0 |
| 2880.0 | -73.0 | 11.7 |
| 3074.9 | -73.9 | 7.1 |
| 3987.5 | -79.2 | 18.4 |
| 4798.1 | -84.8 | 7.8 |
| 5520.4 | -86.5 | 7.8 |
| 6432.4 | -90.7 | 16.8 |
| 9443.2 | -98.3 | 9.3 |
| 10559.9 | -100.0 | 7.4 |
| 12000.2 | -103.4 | 9.9 |
| 14998.9 | -106.2 | 9.0 |
| 16193.6 | -107.8 | 11.6 |
| 18480.0 | -111.0 | 10.9 |
| 21457.0 | -112.1 | 8.4 |
| 23279.8 | -125.8 | 7.3 |

Noise floor tilt over 200 Hz-12 kHz: **-9.4 dB/octave**.

