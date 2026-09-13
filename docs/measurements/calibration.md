# Calibration

Stage 16 of [the implementation plan](../IMPLEMENTATION_PLAN.md): the synth measured against a reference it did not produce. Regenerate with:

```bash
cargo run --release --example calibrate -- --markdown docs/measurements/calibration.md
```

Every figure below is *relative*: an order against the firing order, a measured frequency against a predicted one, a slope against an octave. That is deliberate and it is the point of the stage. A gain constant anywhere in the chain moves every order by the same number of decibels and cancels out of all of it, so nothing here can be closed by turning something up — only by changing a length, a volume or a radius.

| Engine | Firing order | Order balance rms | Worst order | Crank nulls | Modes placed | Floor tilt |
|---|---:|---:|---:|---:|---:|---:|
| [Inline-4](#inline-4) | 2 | 11.3 dB | -16.0 dB on 4 | -18.2 dB on 1 | 2/3 of 11 | -9.7 dB/oct |
| [Cross-plane V8](#cross-plane-v8) | 4 | 10.0 dB | -21.6 dB on 7.5 | -12.5 dB on 1, not enforced | 1/4 of 11 | -10.9 dB/oct |
| [Flat-plane V8](#flat-plane-v8) | 4 | 20.6 dB | -29.0 dB on 6 | -13.6 dB on 0.5, not enforced | 2/3 of 10 | -9.7 dB/oct |
| [V10](#v10) | 5 | 17.5 dB | -28.5 dB on 9 | -7.9 dB on 0.5, not enforced | 0/2 of 10 | -8.3 dB/oct |
| [V12](#v12) | 6 | 12.6 dB | -19.3 dB on 12 | -11.1 dB on 0.5, not enforced | 2/5 of 10 | -14.3 dB/oct |
| [2-Rotor Wankel](#2-rotor-wankel) | 2 | 10.5 dB | -14.8 dB on 4 | -24.2 dB on 0.5 | 3/3 of 9 | -10.1 dB/oct |
| [Turbo Inline-4](#turbo-inline-4) | 2 | 3.8 dB | -5.4 dB on 4 | -14.1 dB on 1, not enforced | 2/6 of 11 | -7.9 dB/oct |
| [Twin-turbo V8](#twin-turbo-v8) | 4 | 13.0 dB | -28.0 dB on 7.5 | -19.4 dB on 1, not enforced | 3/6 of 11 | -8.2 dB/oct |
| [Turbo Inline-6](#turbo-inline-6) | 3 | 11.4 dB | -16.1 dB on 6 | -21.4 dB on 1, not enforced | 3/3 of 10 | -9.7 dB/oct |
| [Turbodiesel I4](#turbodiesel-i4) | 2 | 11.9 dB | -16.9 dB on 4 | -6.8 dB on 1, not enforced | 1/4 of 12 | -4.6 dB/oct |
| [Big Single](#big-single) | 0.5 | 7.9 dB | +11.1 dB on 1 | — | 6/7 of 9 | -7.5 dB/oct |

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
| 0.5 | null | -23.1 | — |
| 1 | null | -18.2 | — |
| 1.5 | null | -29.0 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -46.4 | — |
| 3 | null | -35.0 | — |
| 3.5 | null | -38.3 | — |
| 4 | +0.0 | -16.0 | -16.0 |

Driven orders: rms **11.3 dB**, mean -8.0 dB, worst -16.0 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -18.2 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00331 s over 2.12 m` | 75.6 | not found | — | — | — |  |
| block, first bending mode | `mass law on 110 kg` | 102.3 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 621 x 0.951/(4 x (1.200 + 0.017))` | 121.5 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.2 L` | 162.1 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00331 s over 2.12 m` | 226.9 | 216.0 | -4.8 % | -41.1 | 15.8 | downstream run 2.12 m → 2.224 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.280 + 0.017))` | 292.0 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00331 s over 2.12 m` | 378.2 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 723 x 0.983/(4 x 0.416)` | 427.4 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 686/(2 x 0.450)` | 762.7 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 723 x 0.983/(4 x 0.416)` | 1282.2 | 1047.4 | -18.3 % | -59.7 | 13.3 | primary length 0.400 m → 0.490 m |
| expansion chamber, second pass band | `nc/2L = 2 x 686/(2 x 0.450)` | 1525.4 | 1483.2 | -2.8 % | -68.6 | 20.9 | chamber length 0.450 m, volume 9.3 L → 0.463 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 216.0 | -41.1 | 15.8 |
| 1047.4 | -59.7 | 13.3 |
| 1483.2 | -68.6 | 20.9 |
| 2160.8 | -78.5 | 12.0 |
| 2878.8 | -84.4 | 14.9 |
| 3301.9 | -87.6 | 9.5 |
| 3597.8 | -81.3 | 19.4 |
| 3823.2 | -89.5 | 7.9 |
| 4011.6 | -86.6 | 10.7 |
| 4347.3 | -86.2 | 10.2 |
| 4701.5 | -86.9 | 12.5 |
| 5380.4 | -89.9 | 9.2 |
| 6344.3 | -90.7 | 13.6 |
| 6614.2 | -94.1 | 9.3 |
| 8354.4 | -104.3 | 10.8 |
| 8690.8 | -108.6 | 8.1 |
| 9040.3 | -107.6 | 12.0 |
| 11279.7 | -105.8 | 15.6 |
| 12000.4 | -107.2 | 9.6 |
| 14782.5 | -117.1 | 10.8 |
| 15175.6 | -117.1 | 13.0 |
| 18000.5 | -113.6 | 18.2 |
| 19440.6 | -116.0 | 8.0 |
| 22277.6 | -124.0 | 7.8 |

Noise floor tilt over 200 Hz-12 kHz: **-9.7 dB/octave**.


## Cross-plane V8

> 90-180-270-180 gaps on each bank: the offbeat American burble.

5.0 L  8 cyl  11.0:1  ·  firing order 4  ·  750-6997 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -12.0 | -0.7 |
| 1 | null | -12.5 | — |
| 1.5 | -3.7 | -9.7 | -6.0 |
| 2 | null | -16.2 | — |
| 2.5 | -3.7 | -4.4 | -0.7 |
| 3 | null | -18.6 | — |
| 3.5 | -11.4 | -16.9 | -5.5 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -26.2 | -14.8 |
| 5 | null | -25.4 | — |
| 5.5 | -3.7 | -12.5 | -8.8 |
| 6 | null | -15.5 | — |
| 6.5 | -3.7 | -11.3 | -7.6 |
| 7 | null | -27.6 | — |
| 7.5 | -11.4 | -33.0 | -21.6 |
| 8 | +0.0 | -10.6 | -10.6 |

Driven orders: rms **10.0 dB**, mean -7.6 dB, worst -21.6 dB on order 7.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -12.5 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00446 s over 2.82 m` | 56.0 | not found | — | — | — |  |
| block, first bending mode | `mass law on 210 kg` | 74.1 | 82.9 | +11.9 % | -44.4 | 12.1 |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 610 x 0.986/(4 x (1.500 + 0.018))` | 99.0 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 4.8 L` | 140.3 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00446 s over 2.82 m` | 168.1 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.380 + 0.018))` | 218.0 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00446 s over 2.82 m` | 280.2 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 720 x 0.996/(4 x 0.568)` | 315.7 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 679/(2 x 0.650)` | 522.2 | 405.8 | -22.3 % | -53.1 | 13.5 | chamber length 0.650 m, volume 22.1 L → 0.837 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 720 x 0.996/(4 x 0.568)` | 947.1 | 905.5 | -4.4 % | -61.9 | 12.3 | primary length 0.550 m → 0.575 m |
| expansion chamber, second pass band | `nc/2L = 2 x 679/(2 x 0.650)` | 1044.5 | 1199.9 | +14.9 % | -64.5 | 10.8 | chamber length 0.650 m, volume 22.1 L → 0.566 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 82.9 | -44.4 | 12.1 |
| 405.8 | -53.1 | 13.5 |
| 905.5 | -61.9 | 12.3 |
| 1199.9 | -64.5 | 10.8 |
| 1535.6 | -71.5 | 18.9 |
| 2679.2 | -79.8 | 11.2 |
| 2994.3 | -80.7 | 12.1 |
| 3359.5 | -82.9 | 11.4 |
| 3900.3 | -80.3 | 18.6 |
| 4472.6 | -86.0 | 14.1 |
| 4799.7 | -90.4 | 15.5 |
| 5115.6 | -94.1 | 12.6 |
| 5712.2 | -94.0 | 18.2 |
| 7540.4 | -99.5 | 10.9 |
| 8449.8 | -101.1 | 15.3 |
| 8996.4 | -103.9 | 12.2 |
| 9359.2 | -105.2 | 10.6 |
| 9599.8 | -103.3 | 15.3 |
| 11760.1 | -114.8 | 13.1 |
| 13199.8 | -109.6 | 19.8 |
| 15600.0 | -117.1 | 12.0 |
| 16799.9 | -115.6 | 15.1 |
| 18638.5 | -124.3 | 10.4 |
| 21359.8 | -119.6 | 17.4 |

Noise floor tilt over 200 Hz-12 kHz: **-10.9 dB/octave**.


## Flat-plane V8

> Even 180 deg on both banks: two inline-fours sharing a crank.

4.5 L  8 cyl  12.5:1  ·  firing order 4  ·  900-8596 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -13.6 | — |
| 1 | null | -16.5 | — |
| 1.5 | null | -23.2 | — |
| 2 | +0.0 | -26.8 | -26.8 |
| 2.5 | null | -31.2 | — |
| 3 | null | -25.2 | — |
| 3.5 | null | -29.2 | — |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | null | -38.4 | — |
| 5 | null | -27.9 | — |
| 5.5 | null | -29.4 | — |
| 6 | +0.0 | -29.0 | -29.0 |
| 6.5 | null | -34.7 | — |
| 7 | null | -37.0 | — |
| 7.5 | null | — | — |
| 8 | +0.0 | -11.8 | -11.8 |

Driven orders: rms **20.6 dB**, mean -16.9 dB, worst -29.0 dB on order 6 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -13.6 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 180 kg` | 80.0 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00275 s over 1.82 m` | 90.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 649 x 0.985/(4 x (0.900 + 0.020))` | 173.5 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00275 s over 1.82 m` | 272.8 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 709 x 0.993/(4 x 0.437)` | 402.6 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.015))` | 445.8 | 429.5 | -3.6 % | -48.4 | 15.9 | runner length 0.180 m → 0.202 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00275 s over 1.82 m` | 454.6 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 687/(2 x 0.450)` | 763.0 | not found | — | — | — |  |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 709 x 0.993/(4 x 0.437)` | 1207.8 | 1041.4 | -13.8 % | -60.5 | 13.5 | primary length 0.420 m → 0.487 m |
| expansion chamber, second pass band | `nc/2L = 2 x 687/(2 x 0.450)` | 1526.1 | 1643.6 | +7.7 % | -73.3 | 10.4 | chamber length 0.450 m, volume 11.9 L → 0.418 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 114.8 | -43.4 | 9.4 |
| 429.5 | -48.4 | 15.9 |
| 1041.4 | -60.5 | 13.5 |
| 1374.9 | -68.6 | 14.1 |
| 1643.6 | -73.3 | 10.4 |
| 2159.9 | -75.1 | 14.0 |
| 2880.1 | -82.9 | 9.8 |
| 3119.9 | -83.9 | 8.9 |
| 3922.7 | -89.6 | 10.9 |
| 4320.0 | -90.7 | 9.0 |
| 4799.8 | -91.2 | 14.8 |
| 5519.9 | -93.4 | 9.6 |
| 6959.7 | -96.5 | 17.8 |
| 8160.1 | -104.7 | 9.4 |
| 9839.6 | -103.1 | 15.1 |
| 10559.6 | -108.1 | 8.8 |
| 10799.8 | -109.6 | 9.4 |
| 12480.0 | -110.3 | 9.7 |
| 13440.1 | -112.1 | 12.5 |
| 15599.8 | -120.0 | 9.3 |
| 16079.2 | -113.6 | 14.1 |
| 18720.2 | -116.2 | 12.6 |
| 21359.8 | -117.8 | 12.0 |
| 23280.0 | -130.3 | 10.2 |

Noise floor tilt over 200 Hz-12 kHz: **-9.7 dB/octave**.


## V10

> 72 deg firing, unevenly split across the banks: metallic and hard.

5.2 L  10 cyl  12.7:1  ·  firing order 5  ·  900-8496 rpm over 8.0 s

Reference: the crank itself: 10 firings a cycle on 2 banks

### Order balance [dB relative to order 5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -22.3 | -7.9 | — |
| 1 | -8.4 | -12.5 | -4.1 |
| 1.5 | -5.6 | -18.2 | -12.6 |
| 2 | -12.6 | -19.3 | — |
| 2.5 | -14.0 | -21.1 | — |
| 3 | -12.6 | -16.6 | — |
| 3.5 | -5.6 | -17.1 | -11.5 |
| 4 | -8.4 | -16.7 | -8.4 |
| 4.5 | -22.3 | -43.9 | — |
| **5** | +0.0 | +0.0 | +0.0 |
| 5.5 | -22.3 | -34.1 | — |
| 6 | -8.4 | -27.0 | -18.6 |
| 6.5 | -5.6 | -29.0 | -23.4 |
| 7 | -12.6 | -27.9 | — |
| 7.5 | -14.0 | -28.6 | — |
| 8 | -12.6 | -18.8 | — |
| 8.5 | -5.6 | -32.7 | -27.1 |
| 9 | -8.4 | -36.9 | -28.5 |
| 9.5 | -22.3 | -43.6 | — |
| 10 | +0.0 | -15.6 | -15.6 |

Driven orders: rms **17.5 dB**, mean -15.0 dB, worst -28.5 dB on order 9 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -7.9 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 220 kg` | 72.4 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 82.6 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 622 x 0.982/(4 x (1.100 + 0.019))` | 136.5 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 247.7 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.019))` | 363.3 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00303 s over 1.92 m` | 412.8 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 686 x 0.995/(4 x 0.376)` | 453.5 | 539.8 | +19.0 % | -52.2 | 11.5 | primary length 0.360 m → 0.302 m |
| expansion chamber, first pass band | `nc/2L = 1 x 662/(2 x 0.400)` | 827.3 | 960.6 | +16.1 % | -63.6 | 15.5 | chamber length 0.400 m, volume 8.5 L → 0.344 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 686 x 0.995/(4 x 0.376)` | 1360.5 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 662/(2 x 0.400)` | 1654.5 | not found | — | — | — |  |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 539.8 | -52.2 | 11.5 |
| 960.6 | -63.6 | 15.5 |
| 2091.4 | -62.2 | 12.2 |
| 2400.4 | -72.5 | 11.1 |
| 3119.8 | -75.7 | 11.2 |
| 3841.3 | -80.4 | 10.1 |
| 4347.8 | -77.3 | 14.3 |
| 4800.0 | -89.1 | 8.3 |
| 5279.7 | -89.0 | 9.3 |
| 5999.6 | -97.7 | 8.8 |
| 7440.2 | -96.0 | 9.6 |
| 8160.0 | -102.3 | 9.3 |
| 8400.1 | -104.4 | 10.3 |
| 10320.0 | -103.6 | 11.5 |
| 10800.0 | -109.3 | 9.0 |
| 11040.0 | -107.9 | 11.0 |
| 12480.4 | -108.5 | 8.9 |
| 12960.3 | -112.3 | 9.3 |
| 13200.2 | -108.3 | 13.6 |
| 16079.6 | -112.8 | 11.4 |
| 18239.8 | -116.4 | 12.1 |
| 20400.3 | -120.7 | 8.2 |
| 21120.3 | -116.2 | 13.1 |
| 23280.2 | -126.7 | 21.7 |

Noise floor tilt over 200 Hz-12 kHz: **-8.3 dB/octave**.


## V12

> 60 deg firing, even on both banks: no beat left to hear, only pitch.

6.5 L  12 cyl  11.8:1  ·  firing order 6  ·  800-8496 rpm over 8.0 s

Reference: the crank itself: 12 firings a cycle on 2 banks

### Order balance [dB relative to order 6]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -11.1 | — |
| 1 | null | -17.0 | — |
| 1.5 | null | -17.2 | — |
| 2 | null | -23.5 | — |
| 2.5 | null | -39.9 | — |
| 3 | +0.0 | +9.3 | +9.3 |
| 3.5 | null | -19.7 | — |
| 4 | null | -13.8 | — |
| 4.5 | null | -27.2 | — |
| 5 | null | -26.8 | — |
| 5.5 | null | — | — |
| **6** | +0.0 | +0.0 | +0.0 |
| 6.5 | null | -51.3 | — |
| 7 | null | -32.5 | — |
| 7.5 | null | -36.3 | — |
| 8 | null | -33.0 | — |
| 8.5 | null | -40.1 | — |
| 9 | +0.0 | -13.3 | -13.3 |
| 9.5 | null | -43.9 | — |
| 10 | null | -37.3 | — |
| 10.5 | null | -44.3 | — |
| 11 | null | -40.5 | — |
| 11.5 | null | -37.1 | — |
| 12 | +0.0 | -19.3 | -19.3 |

Driven orders: rms **12.6 dB**, mean -5.8 dB, worst -19.3 dB on order 12 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -11.1 dB on order 0.5 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 260 kg` | 66.6 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 119.8 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 648 x 0.946/(4 x (1.000 + 0.017))` | 150.7 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 359.5 | 274.7 | -23.6 % | -41.9 | 13.4 | downstream run 1.37 m → 1.789 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.140 + 0.017))` | 552.0 | 480.5 | -12.9 % | -54.3 | 7.6 | runner length 0.140 m → 0.181 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 713 x 0.989/(4 x 0.314)` | 561.1 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00209 s over 1.37 m` | 599.1 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 678/(2 x 0.350)` | 968.6 | 1185.8 | +22.4 % | -61.3 | 18.4 | chamber length 0.350 m, volume 2.5 L → 0.286 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 713 x 0.989/(4 x 0.314)` | 1683.2 | 1553.3 | -7.7 % | -72.7 | 17.6 | primary length 0.300 m → 0.325 m |
| expansion chamber, second pass band | `nc/2L = 2 x 678/(2 x 0.350)` | 1937.2 | 1886.6 | -2.6 % | -79.1 | 11.5 | chamber length 0.350 m, volume 2.5 L → 0.359 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 274.7 | -41.9 | 13.4 |
| 480.5 | -54.3 | 7.6 |
| 1185.8 | -61.3 | 18.4 |
| 1553.3 | -72.7 | 17.6 |
| 1886.6 | -79.1 | 11.5 |
| 2082.1 | -81.8 | 6.4 |
| 2445.8 | -86.3 | 6.6 |
| 3359.6 | -87.4 | 5.0 |
| 3599.8 | -87.9 | 11.0 |
| 4081.9 | -94.2 | 5.7 |
| 4319.8 | -96.9 | 5.3 |
| 4559.5 | -96.1 | 5.3 |
| 4799.8 | -93.0 | 12.1 |
| 5280.0 | -102.6 | 8.8 |
| 5998.7 | -97.5 | 11.0 |
| 8159.3 | -103.2 | 7.8 |
| 9119.9 | -103.2 | 8.4 |
| 13200.0 | -109.8 | 14.2 |
| 14637.9 | -114.6 | 5.0 |
| 15707.6 | -111.0 | 7.8 |
| 16423.9 | -112.3 | 5.0 |
| 20064.6 | -117.7 | 5.2 |
| 21046.6 | -118.1 | 7.8 |
| 22774.1 | -122.0 | 5.1 |

Noise floor tilt over 200 Hz-12 kHz: **-14.3 dB/octave**.


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
| absorptive silencer, first pass band | `c/2L = 674/(2 x 0.500)` | 673.9 | 640.2 | -5.0 % | -49.4 | 13.4 | silencer length 0.500 m → 0.526 m |
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
| 1826.1 | -63.7 | 14.9 |
| 2771.7 | -75.0 | 11.8 |
| 3359.7 | -77.0 | 12.3 |
| 4080.0 | -81.8 | 8.8 |
| 4767.4 | -84.7 | 7.2 |
| 5389.4 | -87.5 | 7.5 |
| 6479.7 | -88.9 | 8.6 |
| 8160.8 | -103.4 | 10.1 |
| 9119.9 | -102.4 | 7.3 |
| 9600.0 | -101.3 | 12.2 |
| 10559.6 | -101.8 | 5.9 |
| 13200.6 | -107.8 | 6.1 |
| 13680.9 | -105.8 | 9.2 |
| 14401.0 | -111.1 | 7.5 |
| 16800.1 | -109.0 | 12.4 |
| 20159.5 | -112.6 | 13.0 |

Noise floor tilt over 200 Hz-12 kHz: **-10.1 dB/octave**.


## Turbo Inline-4

> Even 180 deg firing under a small fast single: bark, whistle, flutter.

2.0 L  4 cyl  9.6:1  ·  firing order 2  ·  820-6897 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -19.3 | — |
| 1 | null | -14.1 | — |
| 1.5 | null | -45.0 | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | -29.3 | — |
| 3 | null | -22.1 | — |
| 3.5 | null | -49.6 | — |
| 4 | +0.0 | -5.4 | -5.4 |

Driven orders: rms **3.8 dB**, mean -2.7 dB, worst -5.4 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -14.1 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 125 kg` | 96.0 | not found | — | — | — |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 101.1 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 643 x 0.996/(4 x (1.200 + 0.018))` | 131.3 | not found | — | — | — |  |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.5 L` | 163.7 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 303.2 | 251.2 | -17.2 % | -42.9 | 7.8 | downstream run 1.62 m → 1.954 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.240 + 0.017))` | 337.4 | 387.9 | +15.0 % | -51.8 | 7.8 | runner length 0.240 m → 0.224 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00247 s over 1.62 m` | 505.3 | 479.3 | -5.1 % | -58.1 | 8.3 | downstream run 1.62 m → 1.706 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 748 x 0.998/(4 x 0.366)` | 510.0 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 692/(2 x 0.400)` | 864.9 | 958.8 | +10.9 % | -66.5 | 7.6 | chamber length 0.400 m, volume 3.1 L → 0.361 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 748 x 0.998/(4 x 0.366)` | 1530.0 | 1149.6 | -24.9 % | -54.3 | 21.0 | primary length 0.350 m → 0.466 m |
| expansion chamber, second pass band | `nc/2L = 2 x 692/(2 x 0.400)` | 1729.7 | 1636.3 | -5.4 % | -63.4 | 27.4 | chamber length 0.400 m, volume 3.1 L → 0.423 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 52.5 | -40.5 | 16.3 |
| 251.2 | -42.9 | 7.8 |
| 387.9 | -51.8 | 7.8 |
| 479.3 | -58.1 | 8.3 |
| 958.8 | -66.5 | 7.6 |
| 1149.6 | -54.3 | 21.0 |
| 1636.3 | -63.4 | 27.4 |
| 3585.6 | -75.6 | 11.5 |
| 4187.2 | -74.6 | 17.0 |
| 4736.2 | -76.4 | 12.9 |
| 6482.1 | -71.5 | 19.4 |
| 7336.8 | -90.4 | 8.1 |
| 7848.9 | -92.4 | 7.8 |
| 8374.7 | -98.2 | 9.9 |
| 8914.8 | -93.8 | 9.3 |
| 9473.8 | -88.0 | 8.2 |
| 10047.7 | -86.2 | 22.4 |
| 13136.6 | -85.9 | 24.7 |
| 14639.3 | -113.1 | 10.7 |
| 14964.7 | -113.1 | 10.4 |
| 15722.9 | -111.1 | 8.5 |
| 17761.8 | -110.6 | 20.7 |
| 21601.2 | -118.0 | 7.9 |
| 22080.2 | -117.8 | 10.5 |

Noise floor tilt over 200 Hz-12 kHz: **-7.9 dB/octave**.


## Twin-turbo V8

> Hot-vee twins over the 90-180-270-180 burble: offbeat, but muffled.

4.0 L  8 cyl  10.0:1  ·  firing order 4  ·  760-7097 rpm over 8.0 s

Reference: the crank itself: 8 firings a cycle on 2 banks

### Order balance [dB relative to order 4]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | -11.4 | -12.9 | -1.5 |
| 1 | null | -19.4 | — |
| 1.5 | -3.7 | -4.4 | -0.7 |
| 2 | null | -22.2 | — |
| 2.5 | -3.7 | -3.1 | +0.6 |
| 3 | null | — | — |
| 3.5 | -11.4 | -20.7 | -9.3 |
| **4** | +0.0 | +0.0 | +0.0 |
| 4.5 | -11.4 | -33.2 | -21.8 |
| 5 | null | -28.9 | — |
| 5.5 | -3.7 | -18.5 | -14.8 |
| 6 | null | -32.9 | — |
| 6.5 | -3.7 | -8.8 | -5.1 |
| 7 | null | -32.2 | — |
| 7.5 | -11.4 | -39.4 | -28.0 |
| 8 | +0.0 | -9.4 | -9.4 |

Driven orders: rms **13.0 dB**, mean -9.0 dB, worst -28.0 dB on order 7.5 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -19.4 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00409 s over 2.52 m` | 61.2 | not found | — | — | — |  |
| block, first bending mode | `mass law on 235 kg` | 70.0 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 599 x 0.990/(4 x (1.400 + 0.020))` | 104.5 | 104.4 | -0.0 % | -40.7 | 15.7 | tailpipe length 1.400 m, mouth radius 0.033 m → 1.420 m |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 4.5 L` | 173.8 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00409 s over 2.52 m` | 183.6 | 207.2 | +12.9 % | -44.9 | 7.4 | downstream run 2.52 m → 2.233 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00409 s over 2.52 m` | 306.0 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.260 + 0.018))` | 312.1 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 691 x 0.996/(4 x 0.468)` | 367.6 | 370.6 | +0.8 % | -49.7 | 16.7 | primary length 0.450 m → 0.446 m |
| expansion chamber, first pass band | `nc/2L = 1 x 657/(2 x 0.550)` | 597.3 | 730.2 | +22.3 % | -56.3 | 10.2 | chamber length 0.550 m, volume 13.1 L → 0.450 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 691 x 0.996/(4 x 0.468)` | 1102.7 | 1112.3 | +0.9 % | -61.1 | 18.9 | primary length 0.450 m → 0.446 m |
| expansion chamber, second pass band | `nc/2L = 2 x 657/(2 x 0.550)` | 1194.6 | 909.9 | -23.8 % | -56.6 | 16.3 | chamber length 0.550 m, volume 13.1 L → 0.722 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 104.4 | -40.7 | 15.7 |
| 207.2 | -44.9 | 7.4 |
| 370.6 | -49.7 | 16.7 |
| 730.2 | -56.3 | 10.2 |
| 909.9 | -56.6 | 16.3 |
| 1112.3 | -61.1 | 18.9 |
| 1545.0 | -74.0 | 11.7 |
| 1786.2 | -73.2 | 15.1 |
| 2129.6 | -81.7 | 6.7 |
| 2997.4 | -76.8 | 7.5 |
| 3615.5 | -73.4 | 20.1 |
| 4209.8 | -73.2 | 10.3 |
| 4776.7 | -76.8 | 11.8 |
| 5279.8 | -97.6 | 8.9 |
| 7231.4 | -89.4 | 17.1 |
| 7816.9 | -91.4 | 6.8 |
| 9422.3 | -88.5 | 20.5 |
| 11698.0 | -115.4 | 7.5 |
| 11759.9 | -119.5 | 7.8 |
| 13199.1 | -111.5 | 15.9 |
| 16793.6 | -116.2 | 6.5 |
| 17280.0 | -116.0 | 12.9 |
| 20436.3 | -120.0 | 13.1 |
| 22077.9 | -128.3 | 6.3 |

Noise floor tilt over 200 Hz-12 kHz: **-8.2 dB/octave**.


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
| 6239.9 | -98.5 | 9.2 |
| 6960.2 | -100.6 | 10.1 |
| 7680.4 | -104.4 | 11.3 |
| 8400.2 | -109.6 | 12.3 |
| 10135.1 | -106.9 | 11.8 |
| 10799.9 | -104.6 | 26.0 |
| 11177.2 | -111.3 | 9.1 |
| 11459.5 | -111.2 | 10.2 |
| 14640.1 | -115.8 | 18.1 |
| 15360.0 | -114.8 | 14.9 |
| 20160.6 | -120.5 | 11.9 |
| 20879.7 | -123.8 | 10.8 |
| 21931.8 | -122.2 | 16.9 |

Noise floor tilt over 200 Hz-12 kHz: **-9.7 dB/octave**.


## Turbodiesel I4

> No spark at all: a premixed spike, an iron block, and clatter.

2.0 L  4 cyl  21.5:1  ·  firing order 2  ·  800-4998 rpm over 8.0 s

Reference: the crank itself: 4 firings a cycle on 1 bank

### Order balance [dB relative to order 2]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| 0.5 | null | -11.7 | — |
| 1 | null | -6.8 | — |
| 1.5 | null | — | — |
| **2** | +0.0 | +0.0 | +0.0 |
| 2.5 | null | — | — |
| 3 | null | -19.9 | — |
| 3.5 | null | — | — |
| 4 | +0.0 | -16.9 | -16.9 |

Driven orders: rms **11.9 dB**, mean -8.4 dB, worst -16.9 dB on order 4 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: loudest -6.8 dB on order 1 (pass).

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 46.4 | not found | — | — | — |  |
| block, first bending mode | `mass law on 190 kg` | 77.9 | not found | — | — | — |  |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 511 x 0.995/(4 x (1.300 + 0.017))` | 96.5 | not found | — | — | — |  |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 139.1 | 121.7 | -12.5 % | -47.7 | 10.6 | downstream run 2.87 m → 3.276 m |
| intake plenum, helmholtz | `(c/2pi)sqrt(A/VL) with V = 2.8 L` | 153.7 | not found | — | — | — |  |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00539 s over 2.87 m` | 231.8 | not found | — | — | — |  |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.220 + 0.016))` | 367.1 | 408.7 | +11.3 % | -50.9 | 23.0 | runner length 0.220 m → 0.212 m |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 595 x 0.998/(4 x 0.315)` | 471.7 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 570/(2 x 0.550)` | 518.2 | not found | — | — | — |  |
| expansion chamber, first pass band | `nc/2L = 1 x 551/(2 x 0.500)` | 550.6 | not found | — | — | — |  |
| expansion chamber, second pass band | `nc/2L = 2 x 551/(2 x 0.500)` | 1101.1 | 1119.0 | +1.6 % | -68.6 | 10.8 | chamber length 0.500 m, volume 22.4 L → 0.492 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 595 x 0.998/(4 x 0.315)` | 1415.1 | 1598.7 | +13.0 % | -78.2 | 15.1 | primary length 0.300 m → 0.266 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 121.7 | -47.7 | 10.6 |
| 408.7 | -50.9 | 23.0 |
| 849.2 | -57.9 | 14.6 |
| 1119.0 | -68.6 | 10.8 |
| 1598.7 | -78.2 | 15.1 |
| 1677.9 | -78.7 | 8.6 |
| 2036.4 | -76.4 | 15.8 |
| 3043.1 | -73.9 | 11.7 |
| 3592.6 | -69.9 | 26.3 |
| 4187.7 | -68.8 | 14.4 |
| 4748.3 | -72.9 | 16.3 |
| 5695.9 | -91.6 | 8.3 |
| 5824.8 | -90.6 | 11.0 |
| 6219.1 | -88.3 | 8.3 |
| 7184.7 | -82.9 | 19.1 |
| 7768.1 | -87.0 | 9.0 |
| 8375.3 | -93.9 | 10.7 |
| 8845.9 | -88.7 | 8.8 |
| 10002.2 | -82.5 | 27.1 |
| 12108.8 | -113.7 | 8.7 |
| 13725.3 | -106.6 | 19.1 |
| 16798.6 | -111.5 | 24.8 |
| 20543.9 | -117.2 | 12.7 |
| 22797.9 | -130.5 | 8.5 |

Noise floor tilt over 200 Hz-12 kHz: **-4.6 dB/octave**.


## Big Single

> One firing every two turns: the crank itself is the rhythm.

0.7 L  1 cyl  12.0:1  ·  firing order 0.5  ·  1250-7597 rpm over 8.0 s

Reference: the crank itself: 1 firings a cycle on 1 bank

### Order balance [dB relative to order 0.5]

| Order | Reference | Synth | Delta |
|---:|---:|---:|---:|
| **0.5** | +0.0 | +0.0 | +0.0 |
| 1 | +0.0 | +11.1 | +11.1 |

Driven orders: rms **7.9 dB**, mean +5.6 dB, worst +11.1 dB on order 1 (tolerance 8 dB, pass). Orders the crank cannot drive are marked `null` and are held under -12 dB instead of compared: none resolvable.

### Resonance placement

| Mode | Formula | Predicted [Hz] | Measured [Hz] | Error | Level [dBFS] | Prominence [dB] | Implies |
|---|---|---:|---:|---:|---:|---:|---|
| block, first bending mode | `mass law on 45 kg` | 120.0 | 116.3 | -3.1 % | -31.8 | 21.5 |  |
| collector to mouth, quarter wave | `1/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 259.2 | not found | — | — | — |  |
| exhaust primary, quarter wave | `c(1-M^2)/4(L+d) = 1 x 808 x 1.000/(4 x 0.637)` | 317.1 | 299.5 | -5.6 % | -40.1 | 12.8 | primary length 0.620 m → 0.657 m |
| intake runner, ram quarter wave | `c/4(L+d) = 347/(4 x (0.180 + 0.020))` | 434.6 | 422.0 | -2.9 % | -41.8 | 9.3 | runner length 0.180 m → 0.206 m |
| tailpipe, quarter wave | `c(1-M^2)/4(L+d) = 735 x 1.000/(4 x (0.350 + 0.015))` | 503.7 | 479.4 | -4.8 % | -40.5 | 13.1 | tailpipe length 0.350 m, mouth radius 0.024 m → 0.383 m |
| collector to mouth, third mode | `3/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 777.5 | 777.5 | -0.0 % | -42.4 | 9.0 | downstream run 0.72 m → 0.725 m |
| exhaust primary, third mode | `3c(1-M^2)/4(L+d) = 3 x 808 x 1.000/(4 x 0.637)` | 951.4 | not found | — | — | — |  |
| absorptive silencer, first pass band | `c/2L = 769/(2 x 0.360)` | 1067.8 | 1197.3 | +12.1 % | -53.0 | 14.0 | silencer length 0.360 m → 0.321 m |
| collector to mouth, fifth mode | `5/4T with T = sum(L/c) = 0.00096 s over 0.72 m` | 1295.9 | 1256.1 | -3.1 % | -52.6 | 7.3 | downstream run 0.72 m → 0.748 m |

### Peaks in the sweep average

| Hz | dBFS | Prominence [dB] |
|---:|---:|---:|
| 116.3 | -31.8 | 21.5 |
| 299.5 | -40.1 | 12.8 |
| 358.4 | -43.3 | 8.0 |
| 422.0 | -41.8 | 9.3 |
| 479.4 | -40.5 | 13.1 |
| 539.2 | -40.5 | 8.5 |
| 777.5 | -42.4 | 9.0 |
| 1197.3 | -53.0 | 14.0 |
| 1256.1 | -52.6 | 7.3 |
| 1916.1 | -57.1 | 8.9 |
| 2034.8 | -55.4 | 8.4 |
| 2096.0 | -54.3 | 17.6 |
| 2155.7 | -55.8 | 10.6 |
| 2874.4 | -57.0 | 9.7 |
| 4454.1 | -72.6 | 12.8 |
| 5316.6 | -73.0 | 7.6 |
| 8807.0 | -81.0 | 11.7 |
| 11172.9 | -87.7 | 7.9 |
| 13003.0 | -91.9 | 8.7 |
| 15599.6 | -94.0 | 13.4 |
| 18058.3 | -97.0 | 13.6 |
| 19962.6 | -100.6 | 7.4 |
| 20697.1 | -99.4 | 11.1 |
| 22375.0 | -104.8 | 11.2 |

Noise floor tilt over 200 Hz-12 kHz: **-7.5 dB/octave**.

