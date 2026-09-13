# Engine Configuration Guide: Authoring & Tuning

This document describes how to create, edit, and acoustically tune engine configurations in the simulator.

Every engine is defined as a standalone, human-readable `.toml` file located in the `engines/` directory (e.g., `engines/cross_plane_v8.toml`, `engines/twin_turbo_v8.toml`). These files completely specify the thermodynamics, mechanical dimensions, gas-dynamic pipe networks, induction systems, mechanical noise sources, and 3D spatial acoustic apertures.

---

## Table of Contents
1. [Configuration Architecture & Units](#1-configuration-architecture--units)
2. [Quickstart: Creating a New Engine](#2-quickstart-creating-a-new-engine)
3. [Complete Field Reference](#3-complete-field-reference)
   - [Identity (`[identity]`)](#identity-identity)
   - [Block & Inertia (`[block]`)](#block--inertia-block)
   - [Cylinder Geometry & Valves (`[cylinder]`)](#cylinder-geometry--valves-cylinder)
   - [Combustion & Heat Release (`[combustion]`)](#combustion--heat-release-combustion)
   - [Firing Order & Cylinder Banks (`[firing]`)](#firing-order--cylinder-banks-firing)
   - [Exhaust Network (`[exhaust]`)](#exhaust-network-exhaust)
   - [Intake Network (`[intake]`)](#intake-network-intake)
   - [Forced Induction (`[induction]`)](#forced-induction-induction)
   - [Mechanical Noise Sources (`[mechanical]`)](#mechanical-noise-sources-mechanical)
   - [3D Acoustic Apertures (`[acoustics]`)](#3d-acoustic-apertures-acoustics)
4. [Acoustic Tuning & Physics Handbook](#4-acoustic-tuning--physics-handbook)
   - [Header Runner Resonance Tuning](#header-runner-resonance-tuning)
   - [Muffler & Silencer Selection](#muffler--silencer-selection)
   - [Exhaust Modes: Open Headers vs Straight Pipe vs Muffled](#exhaust-modes-open-headers-vs-straight-pipe-vs-muffled)
   - [Rev Limiter Pops, Bangs & Cadence](#rev-limiter-pops-bangs--cadence)
   - [Single Throttle vs Individual Throttle Bodies (ITBs)](#single-throttle-vs-individual-throttle-bodies-itbs)
5. [Validation & Diagnostic Benchmarking](#5-validation--diagnostic-benchmarking)

---

## 1. Configuration Architecture & Units

The engine simulator obeys strict physical laws:
- **SI Units Throughout**:
  - Length / Diameter / Lift / Bore / Stroke: **metres** ($m$) (e.g., $85\text{ mm} = 0.085$).
  - Mass / Inertia: **kilograms** ($kg$) and **kilogram-metres squared** ($kg\cdot m^2$).
  - Temperature: **Kelvin** ($K$) ($0^\circ\text{C} = 273.15\text{ K}$, $800^\circ\text{C} = 1073.15\text{ K}$).
  - Angles: **Crankshaft degrees** ($^\circ$) in a $720^\circ$ four-stroke cycle ($0^\circ = \text{TDC combustion}$).
  - Pressure: **Pascals** ($Pa$) ($1\text{ atm} \approx 101325\text{ Pa}$, $1\text{ bar boost} = 201325\text{ Pa}$).
  - Volume: **Cubic metres** ($m^3$) ($1\text{ L} = 0.001\text{ m}^3$).
  - Energy: **Joules per kilogram** ($J/kg$).
- **Dual-Domain Solving**:
  - The thermodynamics cycle runs in 64-bit float (`f64`) in the physical domain.
  - The acoustic synthesizer evaluates digital waveguides, non-linear shock wave steepening, and biquad resonators in 32-bit float (`f32`).
  - The narrowing occurs automatically at the snapshot boundary.

---

## 2. Quickstart: Creating a New Engine

1. Copy an existing engine from `engines/` that has the closest cylinder count or architecture:
   ```bash
   cp engines/inline_4.toml engines/my_custom_engine.toml
   ```
2. Edit `[identity]` to give your engine a unique name and note.
3. Adjust the cylinder bore, stroke, compression ratio, and redline.
4. Verify syntax and configuration validity using the built-in test suite:
   ```bash
   cargo test --lib bench::config
   ```
5. Run the acoustic diagnostic bench to hear your engine and measure its transients:
   ```bash
   cargo run --release --example acoustic_bench -- --engine my_custom_engine
   ```

---

## 3. Complete Field Reference

### Identity (`[identity]`)

Metadata displayed in the UI dashboard and logs.

```toml
[identity]
name = "Naturally Aspirated 4.0L V8"
note = "Flat-plane crankshaft, 9000 RPM screamer with equal-length 4-into-1 headers."
```

| Field | Type | Description |
|---|---|---|
| `name` | `string` | Display name of the engine. |
| `note` | `string` | Historical context, acoustic character, or technical summary. |

---

### Block & Inertia (`[block]`)

Defines the structural properties of the engine block, rotational inertia, idle/redline limits, dyno load profile, and rev limiter characteristics.

```toml
[block]
mass = 160.0
bore_spacing = 0.098
inertia = 0.18
redline = 8800.0
idle = 950.0
load = [5.0, 0.015, 0.00008]
anti_lag = false
limiter_mode = "RotatingStutter"
limiter_cut = "Spark"
```

| Field | Type | Default | Physical Meaning & Tuning |
|---|---|---|---|
| `mass` | `f64` | *Required* | Thermal mass of the cylinder block in kilograms ($kg$). Controls the warm-up rate from cold start to operating temperature ($363\text{ K}$). |
| `bore_spacing` | `f64` | *Required* | Distance between adjacent cylinder bore centerlines in metres ($m$) (e.g. `0.098` for 98 mm). |
| `inertia` | `f64` | *Required* | Crankshaft, flywheel, and accessory rotational inertia ($kg\cdot m^2$). Lower values (e.g. `0.10 - 0.15`) yield rapid race-engine throttle response; higher values (`0.30 - 0.50`) yield lazy truck/cruiser revving. |
| `redline` | `f64` | *Required* | Maximum operating engine speed in RPM. Governs where the rev limiter engages and UI warning lights flash. |
| `idle` | `f64` | *Required* | Target idle speed in RPM. The closed-loop idle governor targets this speed. |
| `load` | `[f64; 3]` | *Required* | Dyno resistance polynomial $[C_0, C_1, C_2]$: $\text{Torque} = C_0 + C_1 \cdot \omega + C_2 \cdot \omega^2$. $C_0$ is static friction, $C_1$ is viscous drag, $C_2$ is aerodynamic drag. |
| `anti_lag` | `bool` | `false` | When `true`, enables rally-style anti-lag (retards ignition and bleeds air into exhaust manifold on throttle lift to keep the turbo spooling). |
| `limiter_mode` | `string` | `"RotatingStutter"` | Rev limiter algorithm: <br>• `"RotatingStutter"`: Aggressive machine-gun 2-step cut rotating across cylinder sequence.<br>• `"Hard"`: Instantaneous hard cut at redline.<br>• `"Soft"`: Progressive timing retard before cutoff. |
| `limiter_cut` | `string` | `"Spark"` | Limiter cut mechanism: <br>• `"Spark"`: Cuts ignition spark only. Unburnt fuel blows into hot exhaust manifold, producing supersonic backfire bangs.<br>• `"Fuel"`: Cuts fuel injection only. Smooth, clean cut without backfires.<br>• `"Both"`: Cuts both fuel and spark simultaneously. |

---

### Cylinder Geometry & Valves (`[cylinder]`)

Defines the physical combustion chamber, piston kinematic dimensions, and intake/exhaust valve event timing.

```toml
[cylinder]
bore = 0.084
stroke = 0.090
rod_length = 0.145
compression_ratio = 11.5
intake_open_deg = 700.0
intake_duration_deg = 245.0
intake_lift = 0.011
intake_diameter = 0.038
intake_discharge_coeff = 0.65
exhaust_open_deg = 495.0
exhaust_duration_deg = 240.0
exhaust_lift = 0.010
exhaust_diameter = 0.032
exhaust_discharge_coeff = 0.60
```

| Field | Type | Default | Units | Description |
|---|---|---|---|---|
| `bore` | `f64` | *Required* | $m$ | Cylinder bore diameter. Single cylinder displacement $V_d = \frac{\pi}{4} \cdot \text{bore}^2 \cdot \text{stroke}$. |
| `stroke` | `f64` | *Required* | $m$ | Piston stroke length. Governs mean piston speed $U_p = 2 \cdot \text{stroke} \cdot \frac{\text{RPM}}{60}$. |
| `rod_length` | `f64` | *Required* | $m$ | Center-to-center connecting rod length. Rod/stroke ratio ($\lambda = \text{stroke}/(2\cdot\text{rod})$) determines piston non-sinusoidal motion and dwell time at TDC/BDC. |
| `compression_ratio` | `f64` | *Required* | Unitless | Geometric compression ratio $r = (V_d + V_c) / V_c$ (e.g. `10.5` for 10.5:1). |
| `intake_open_deg` | `f64` | *Required* | deg ($^\circ$) | Crank angle where intake valve opens in the $720^\circ$ cycle. (e.g., `700.0` is $20^\circ$ Before Top Dead Center Combustion / BTDC). |
| `intake_duration_deg` | `f64` | *Required* | deg ($^\circ$) | Total angular duration intake valve remains off its seat (e.g. `240.0`). |
| `intake_lift` | `f64` | *Required* | $m$ | Maximum intake valve lift off seat (e.g. `0.011` for 11 mm). |
| `intake_diameter` | `f64` | *Required* | $m$ | Outer valve head diameter (e.g. `0.038` for 38 mm). |
| `intake_discharge_coeff` | `f64` | `0.65` | Unitless | Discharge coefficient $C_d$ for isentropic orifice flow equation. |
| `exhaust_open_deg` | `f64` | *Required* | deg ($^\circ$) | Crank angle where exhaust valve opens (EVO). (e.g., `495.0` is $45^\circ$ Before Bottom Dead Center / BBDC). |
| `exhaust_duration_deg` | `f64` | *Required* | deg ($^\circ$) | Total angular duration exhaust valve remains open. |
| `exhaust_lift` | `f64` | *Required* | $m$ | Maximum exhaust valve lift (e.g. `0.010` for 10 mm). |
| `exhaust_diameter` | `f64` | *Required* | $m$ | Exhaust valve head diameter (e.g. `0.032` for 32 mm). |
| `exhaust_discharge_coeff` | `f64` | `0.60` | Unitless | Discharge coefficient $C_d$ for exhaust blowdown. |

> **Valve Timing Cycle Reference**:
> In this simulator, $0^\circ$ is Top Dead Center (TDC) of the power/expansion stroke.
> - $0^\circ - 180^\circ$: Power / Expansion stroke.
> - $180^\circ - 360^\circ$: Exhaust stroke (EVO typically $480^\circ - 510^\circ$; EVC typically $710^\circ - 730^\circ$).
> - $360^\circ - 540^\circ$: Intake stroke (IVO typically $690^\circ - 710^\circ$; IVC typically $190^\circ - 230^\circ$).
> - $540^\circ - 720^\circ$: Compression stroke (Spark typically $330^\circ - 350^\circ$ / $10^\circ - 30^\circ$ BTDC).

---

### Combustion & Heat Release (`[combustion]`)

The thermodynamic heat release rate is modeled via either a Wiebe function (Spark ignition) or a dual-stage kinetic/diffusion model (Compression ignition).

#### Spark Ignition (`type = "Spark"`)
```toml
[combustion]
type = "Spark"
spark_angle_deg = 342.0
duration_deg = 55.0
efficiency_parameter = 5.0
form_factor = 2.0
combustion_efficiency = 0.96
fuel_lhv = 44000000.0
air_fuel_ratio = 14.7
```

| Field | Type | Default | Description |
|---|---|---|---|
| `type` | `string` | `"Spark"` | Must be `"Spark"`. |
| `spark_angle_deg` | `f64` | *Required* | Spark timing in degrees ($342^\circ = 18^\circ$ BTDC). |
| `duration_deg` | `f64` | *Required* | $10\% - 90\%$ burn duration in crank degrees (typically $45^\circ - 60^\circ$). |
| `efficiency_parameter`| `f64` | `5.0` | Wiebe parameter $a$. Governs completeness of combustion window ($a \approx 5.0$ represents $99.3\%$ fuel mass burned). |
| `form_factor` | `f64` | `2.0` | Wiebe form exponent $m$. Dictates peak heat release location ($m = 2.0$ produces typical bell-curved burn rate). |
| `combustion_efficiency`| `f64`| `0.95` | Fraction of chemical fuel energy converted into thermal expansion ($0.92 - 0.98$). |
| `fuel_lhv` | `f64` | `44.0e6` | Lower Heating Value of fuel in $J/kg$ (Gasoline: $4.40 \times 10^7\text{ J/kg}$, E85: $2.92 \times 10^7\text{ J/kg}$, Methanol: $1.99 \times 10^7\text{ J/kg}$). |
| `air_fuel_ratio` | `f64` | `14.7` | Stoichiometric Air/Fuel Ratio (Gasoline: `14.7`, E85: `9.8`, Methanol: `6.4`). |

#### Compression Ignition (`type = "Compression"`)
```toml
[combustion]
type = "Compression"
injection_angle_deg = 352.0
injection_duration_deg = 26.0
premixed_duration_deg = 9.0
diffusion_duration_deg = 65.0
premixed_form_factor = 0.6
diffusion_form_factor = 1.2
efficiency_parameter = 5.0
combustion_efficiency = 0.95
fuel_lhv = 42600000.0
air_fuel_ratio = 22.0
```

| Field | Type | Default | Description |
|---|---|---|---|
| `type` | `string` | `"Compression"`| Must be `"Compression"`. |
| `injection_angle_deg` | `f64` | *Required* | Start of injection (SOI) in degrees ($352^\circ = 8^\circ$ BTDC). |
| `injection_duration_deg`| `f64` | *Required* | Total nozzle open duration ($^\circ$). |
| `premixed_duration_deg` | `f64` | *Required* | Fast premixed explosive combustion duration ($^\circ$), creating classic diesel clatter. |
| `diffusion_duration_deg`| `f64` | *Required* | Slower mixing-controlled diffusion burn duration ($^\circ$). |
| `premixed_form_factor` | `f64` | `0.6` | Wiebe exponent for premixed phase. |
| `diffusion_form_factor`| `f64` | `1.2` | Wiebe exponent for diffusion phase. |
| `fuel_lhv` | `f64` | `42.6e6` | Diesel fuel Lower Heating Value ($4.26 \times 10^7\text{ J/kg}$). |
| `air_fuel_ratio` | `f64` | `22.0` | Lean operating AFR for diesel cycle (`18.0 - 25.0`). |

---

### Firing Order & Cylinder Banks (`[firing]`)

Defines the cylinder ignition sequence and bank physical assignment.

```toml
[firing]
sequence = [1, 5, 4, 8, 6, 3, 7, 2]
banks = [0, 1, 0, 1, 1, 0, 1, 0]
```

| Field | Type | Description |
|---|---|---|
| `sequence` | `[u8]` | Array of 1-based cylinder numbers in the order they fire. The simulator computes the even firing interval as: $\Delta\theta = 720^\circ / N$. |
| `banks` | `[u8]` | Array of cylinder bank indices (`0` or `1`) mapped **1:1 to the sequence array slot**. <br>• Inline engines: all `0`.<br>• V-engines / Boxers: `0` for Bank 1 (Left), `1` for Bank 2 (Right). |

#### Common Firing Order Configurations:
- **Inline-4**:
  ```toml
  sequence = [1, 3, 4, 2]
  banks = [0, 0, 0, 0]
  ```
- **Cross-plane V8 (Classic American 90-180-270-180 burble)**:
  ```toml
  sequence = [1, 8, 7, 2, 6, 5, 4, 3]
  banks = [0, 1, 0, 1, 1, 0, 1, 0]
  ```
- **Flat-plane V8 (180° screamer, Ferrari / GT350)**:
  ```toml
  sequence = [1, 5, 2, 6, 3, 7, 4, 8]
  banks = [0, 1, 0, 1, 0, 1, 0, 1]
  ```
- **Inline-6 (Even 120° acoustic balance)**:
  ```toml
  sequence = [1, 5, 3, 6, 2, 4]
  banks = [0, 0, 0, 0, 0, 0]
  ```
- **60° / 90° V10**:
  ```toml
  sequence = [1, 6, 5, 10, 2, 7, 3, 8, 4, 9]
  banks = [0, 1, 0, 1, 0, 1, 0, 1, 0, 1]
  ```
- **60° V12**:
  ```toml
  sequence = [1, 12, 4, 9, 2, 11, 6, 7, 3, 10, 5, 8]
  banks = [0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0]
  ```

---

### Exhaust Network (`[exhaust]`)

The exhaust system is modeled as a 1D acoustic waveguide network with non-linear characteristic shock wave steepening.

```toml
[exhaust]
tailpipe_flanged = false
cutout = false
mode = "muffled"

[[exhaust.primaries]]
length = 0.45
diameter = 0.044
temperature = 900.0

# ... repeated for each cylinder (1 to N) in numerical order

[exhaust.collector]
inlets = 4
outlet_diameter = 0.060
taper_length = 0.12

[exhaust.crossover]
type = "XPipe"
position = 0.65

[[exhaust.silencers]]
type = "ExpansionChamber"
length = 0.48
area_ratio = 4.2
stages = 2

[exhaust.tailpipe]
length = 1.35
diameter = 0.065
temperature = 650.0
```

#### Exhaust Root Fields:
| Field | Type | Default | Description |
|---|---|---|---|
| `tailpipe_flanged`| `bool` | `false` | When `true`, models an acoustic flange at the tailpipe exit ($k a$ radiation impedance transition). When `false`, models an open, sharp pipe end. |
| `cutout` | `bool` | `false` | When `true`, an electric cutout valve vents exhaust gas directly before the silencers, unleashing loud raw pulses and enabling tailpipe wave steepening. |
| `mode` | `string` | `None` | **Acoustic Preset Override**: <br>• `"open_headers"`: Strips silencers and collector, cuts tailpipe down to $0.15\text{ m}$. Full raw race header sound.<br>• `"straight_pipe"`: Retains full primary, collector, and tailpipe waveguide lengths, but strips all silencer chambers.<br>• `"muffled"` (or omitted): Uses all silencers defined in `[[exhaust.silencers]]`. |

#### Primary Runners (`[[exhaust.primaries]]`):
Must contain one entry per cylinder, in cylinder order ($1, 2, \dots, N$).
- `length`: Runner length ($m$) from exhaust valve to collector.
- `diameter`: Inner tube diameter ($m$).
- `temperature`: Steady gas temperature in runner ($K$) (typically $850 - 1050\text{ K}$).

#### Collector (`[exhaust.collector]`):
- `inlets`: Number of merged primaries (e.g. `4` for 4-to-1, `3` for 3-to-1, `8` for 8-to-1).
- `outlet_diameter`: Collector outlet throat diameter ($m$).
- `taper_length`: Length ($m$) of converging/diverging taper cone.

#### Crossover (`[exhaust.crossover]`):
Used on multi-bank engines (V6, V8, V10, V12) to interconnect banks:
- `type = "None"`: Completely independent true dual exhaust.
- `type = "XPipe"`:
  ```toml
  [exhaust.crossover]
  type = "XPipe"
  position = 0.70  # Fractional distance along secondary pipe (0.0 to 1.0)
  ```
- `type = "HPipe"`:
  ```toml
  [exhaust.crossover]
  type = "HPipe"
  position = 0.65  # Fractional distance along secondary pipe
  diameter = 0.044 # Balance tube inner diameter (m)
  ```
- `type = "Balance180"`: 180° crossover manifold pairing opposite-phase cylinders.

#### Silencers (`[[exhaust.silencers]]`):
Zero, one, or multiple silencers can be chained in sequence:
1. **Expansion Chamber** (Reactive OEM muffler):
   ```toml
   [[exhaust.silencers]]
   type = "ExpansionChamber"
   length = 0.50     # Chamber length (m)
   area_ratio = 4.5  # Chamber cross-sectional area / pipe area
   stages = 2        # Number of internal baffled chambers
   ```
2. **Helmholtz Resonator** (Side-branch drone cancellation chamber):
   ```toml
   [[exhaust.silencers]]
   type = "Helmholtz"
   chamber_volume = 0.0035  # Cavity volume (m^3, e.g. 3.5 L)
   neck_diameter = 0.038    # Neck connecting pipe diameter (m)
   neck_length = 0.080      # Neck connecting pipe length (m)
   q = 8.0                  # Quality factor / resonance sharpness
   resonant_mix = 0.85      # Attenuation mix fraction (0.0 to 1.0)
   ```
3. **Absorptive Silencer** (Perforated-core glasspack / bullet muffler):
   ```toml
   [[exhaust.silencers]]
   type = "Absorptive"
   length = 0.35              # Muffler casing length (m)
   diameter = 0.055            # Perforated core diameter (m)
   packing_thickness = 0.035   # Fiberglass packing thickness (m)
   packing_absorption = 0.70   # High-frequency absorption coefficient (0.0 - 1.0)
   ```
4. **Quarter-Wave Stub** (J-pipe):
   ```toml
   [[exhaust.silencers]]
   type = "QuarterWaveStub"
   length = 0.62   # Stub length (m) tuned to cancel specific frequency
   diameter = 0.045
   ```
5. **Straight Section**:
   ```toml
   [[exhaust.silencers]]
   type = "Straight"
   ```

#### Tailpipe (`[exhaust.tailpipe]`):
- `length`: Distance from muffler outlet to exhaust tip ($m$).
- `diameter`: Tailpipe exit diameter ($m$).
- `temperature`: Gas temperature at vehicle rear ($K$) (typically $500 - 700\text{ K}$).

#### Tuning the Exhaust: the Resonance Sweep

`ExhaustNetwork` computes every one of the relationships above already; `acoustic_bench --sweep`
just reads them back as a table instead of a spectrum you have to squint at:

- **Primary length** sets the quarter-wave peak, $f_1 = c(1 - M^2) / 4(L + \delta)$, with $\delta$
  the Karal-Flugge end correction at the collector junction and $c$ the speed of sound *in that
  primary's own gas*, which is not the tailpipe's — an 880 K primary and a 650 K tailpipe of the
  same length tune 16 % apart.
- **Primary diameter** sets $Z_0 = \rho c / A$, the collector junction's characteristic impedance,
  and therefore how hard the junction reflects.
- **Collector taper length** is the reflection ramp: short is peaky, long is broadband. It does not
  move the primary's tuned frequency (the reflection sits at the junction, before the taper), so
  a healthy sweep should show the *predicted* frequency essentially unchanged across it — watch
  the `Prom.` (prominence) column fall instead, which is the broadband claim actually made visible.
- **Tailpipe length and diameter** set the tailpipe's own quarter-wave and its radiation corner,
  $f_c = c / (2 \pi a)$ — the brightness control, and the most under-used parameter in the
  catalogue.
- **Crossover position** is a delay in wavelengths, and is what separates a flat-plane rasp from a
  crossplane burble.

Run it against one preset and one parameter:

```bash
cargo run --release --example acoustic_bench -- --engine flat_plane_v8 --sweep primary-length
```

`--sweep <parameter>` accepts `primary-length`, `primary-diameter`, `collector-taper`,
`tailpipe-length`, `tailpipe-diameter` or `crossover-position`; `--sweep-steps <n>` widens or
narrows the table (default 5, log-spaced from half to double the preset's own value). The sweep
ignores `--exhaust`, since forcing open-headers or straight-pipe first would overwrite the very
geometry being swept.

Every row prints the analytic prediction beside whatever the render actually produced, and says so
plainly when the two disagree by more than a few percent, or when no peak was found near the
prediction at all — that is the point of the tool, not a bug in it:

```
Value [m]  |    Predicted |     Measured |     Error |      Prom. | Note
--------------------------------------------------------------------------------
0.2100     |     800.6 Hz |     920.2 Hz |    +14.9% |    14.7 dB | diverges +14.9% -- mean-flow bias, end correction, or a junction the formula omits
0.2970     |     570.8 Hz |     554.4 Hz |     -2.9% |    12.2 dB | agrees with the analytic mode
0.4200     |     402.6 Hz |           -- |        -- |         -- | no peak found within the search window
0.5940     |     280.8 Hz |           -- |        -- |         -- | no peak found within the search window
0.8400     |     194.2 Hz |     170.2 Hz |    -12.4% |    16.6 dB | diverges -12.4% -- mean-flow bias, end correction, or a junction the formula omits
```

**Worked example.** The flat-plane V8 ships an 0.42 m primary at 880 K
(`engines/flat_plane_v8.toml`, `[[exhaust.primaries]]`). Appendix A's formula, with the mean-flow
term the gas is actually carrying at that geometry's own sweep midpoint, predicts **402.6 Hz** — not
the round 354 Hz an offline hand calculation might suggest, because the collector here expands from
a 41 mm primary into a 65 mm outlet, which the model gives the *flanged* end correction rather than
the unflanged one, and because the mean flow at load is not zero. That much the sweep confirms on
its own: it is why the tool computes the prediction from the same rendered gas state instead of a
nominal temperature.

What it *cannot* confirm at the baseline length is where the peak actually sits, because there
isn't one within the search window: the row above shows `no peak found`. Reading the row either
side of it (0.297 m agrees to −2.9 %, 0.84 m diverges by −12.4 %) shows why — the flat-plane V8's
eight primaries feed one collector with a genuine area step, and downstream of that step the taper,
the silencer chain and the tailpipe behave as their *own* compound quarter-wave run from the
collector to the mouth, not as the primary's independent continuation of it.
`examples/calibrate.rs`'s fuller resonance-placement table names this mode directly — "collector to
mouth, quarter wave" — and finds it at 209.7 Hz against a 176.3 Hz prediction, close to where this
preset's tailpipe-length sweep also lands. The naive per-segment formula is not wrong about the
primary or the tailpipe in isolation; it is silent about the mode the two of them make together once
a real area mismatch couples them, and the sweep's job is to say so rather than to quietly report
the nearest peak as if it agreed.

A single-cylinder preset (no collector area mismatch to couple through) tunes far more cleanly —
compare `--engine big_single --sweep primary-length`, where most rows land within a few percent.

`--mode` runs open-headers, straight-pipe and muffled on one preset and tabulates the share of
total energy each leaves in the top of the spectrum (`analysis::orders::octave_bands`, not absolute
level, since the three modes do not radiate the same overall loudness):

```bash
cargo run --release --example acoustic_bench -- --engine cross_plane_v8 --mode
```

The claim is that this share rises muffled → straight-pipe → open-headers, and the tool says so when
it does not hold, rather than only printing the numbers: on the cross-plane V8 today it does not —
straight-pipe currently reads brighter than open-headers. That preset's `ExpansionChamber` silencer
has no loss term yet ([`docs/TIMBRE_PLAN.md`](TIMBRE_PLAN.md), Stage T5), so a bench run over it is
measuring the gap T5 exists to close, and the tool flags exactly that rather than smoothing it into
a pass.

---

### Intake Network (`[intake]`)

Models the intake runner waveguides, Helmholtz plenum chamber, throttle assembly, airbox, and snorkel.

```toml
[intake]
plenum_volume = 0.0040
trumpet_flanged = true

[[intake.runners]]
length = 0.28
diameter = 0.045
temperature = 310.0

# ... repeated for each cylinder runner (1 to N)

[intake.throttle]
type = "Single"
bore = 0.075

[intake.airbox]
length = 0.22
diameter = 0.080
temperature = 305.0

[intake.snorkel]
length = 0.35
diameter = 0.075
temperature = 295.0
```

| Field | Type | Default | Description |
|---|---|---|---|
| `plenum_volume` | `f64` | *Required* | Manifold plenum volume in $m^3$ (e.g., `0.0040` for 4.0 litres). Interacts with runner lengths to form the primary intake Helmholtz acoustic resonance. |
| `trumpet_flanged`| `bool` | `true` | When `true`, models smooth velocity stacks / bellmouths inside the plenum or ITB entries. |
| `runners` | Array | *Required* | One runner per cylinder ($1 \dots N$). Contains `length` ($m$), `diameter` ($m$), and `temperature` ($K$, typically $300 - 320\text{ K}$). |
| `throttle` | Table | *Required* | Throttle body geometry: <br>• Single throttle: `{ type = "Single", bore = 0.075 }`<br>• Individual Throttle Bodies: `{ type = "IndividualBodies", bore = 0.046 }` |
| `airbox` | Table | `None` | Optional upstream airbox resonator pipe section (`length`, `diameter`, `temperature`). |
| `snorkel` | Table | `None` | Optional intake snorkel tube drawing ambient air into the airbox (`length`, `diameter`, `temperature`). |

---

### Forced Induction (`[induction]`)

Configures naturally aspirated breathing or forced induction physics and voicing (shaft whistling, blow-off valves, and wastegate flutter).

#### 1. Naturally Aspirated (`type = "NaturallyAspirated"`)
```toml
[induction]
type = "NaturallyAspirated"
```

#### 2. Turbocharged (`type = "Turbocharged"`)
```toml
[induction]
type = "Turbocharged"
max_shaft_rpm = 175000.0
reference_engine_rpm = 6800.0
spool_up = 0.45
spool_down = 0.30
order = 2.0
reference_rpm = 140000.0
level = 0.035

[induction.blow_off]
threshold = 0.25
decay_time = 0.22
center_hz = 2200.0
level = 0.06

[induction.wastegate]
flutter_hz = 60.0
rattle_hz = 1750.0
threshold = 0.70
level = 0.03
```

| Field | Type | Description |
|---|---|---|
| `max_shaft_rpm` | `f64` | Maximum compressor wheel rotational speed (e.g. `160000 - 220000`). |
| `reference_engine_rpm`| `f64` | Engine RPM at which maximum exhaust gas drive and boost are achieved. |
| `spool_up` | `f64` | Spool acceleration time constant in seconds. Lower values simulate small, fast-spooling turbos; higher values (`0.6 - 1.0`) simulate massive laggy single turbos. |
| `spool_down` | `f64` | Spool deceleration time constant in seconds on throttle release. |
| `order` | `f64` | Compressor blade pass frequency multiplier relative to shaft speed. |
| `reference_rpm` | `f64` | Reference shaft RPM where turbo whistle reaches unity nominal pitch. |
| `level` | `f64` | Whistle audio synthesis volume level ($0.01 - 0.08$). |
| `blow_off` | Table | Blow-Off Valve (BOV) venting on sudden throttle lift: <br>• `threshold`: Throttle closing rate required to trigger ($0.15 - 0.40$).<br>• `decay_time`: Discharge duration in seconds ($0.15 - 0.35$).<br>• `center_hz`: Center acoustic jet frequency ($1800 - 3200\text{ Hz}$).<br>• `level`: Discharge audio volume. |
| `wastegate` | Table | Wastegate flutter and flap rattle under boost: <br>• `flutter_hz`: Diaphragm flutter frequency ($50 - 80\text{ Hz}$).<br>• `rattle_hz`: High-frequency metallic flap rattle ($1400 - 2400\text{ Hz}$).<br>• `threshold`: Boost pressure threshold fraction ($0.6 - 0.85$).<br>• `level`: Wastegate audio volume. |

#### 3. Roots Supercharged (`type = "RootsSupercharged"`)
Positive displacement blower (e.g. Eaton, Weiand).
```toml
[induction]
type = "RootsSupercharged"
belt_ratio = 2.1
lobes = 4
level = 0.045
```
- `belt_ratio`: Crank-to-blower pulley ratio (e.g., `2.1` spins the blower 2.1× engine speed).
- `lobes`: Rotor lobe count (`3` or `4`), dictating the meshing whine fundamental order.
- `level`: Supercharger whine volume level.

#### 4. Centrifugal Supercharged (`type = "CentrifugalSupercharged"`)
Gear-driven centrifugal impeller (e.g. ProCharger, Vortech).
```toml
[induction]
type = "CentrifugalSupercharged"
gear_ratio = 4.1
order = 2.0
level = 0.035

[induction.blow_off]
threshold = 0.20
decay_time = 0.18
center_hz = 2600.0
level = 0.05
```

---

### Mechanical Noise Sources (`[mechanical]`)

Controls the synthesis of mechanical, tactile noise generated by metal-on-metal engine components.

```toml
[mechanical.intake_valve]
timing = "per_cylinder"
order = 0.0
level = 0.45

[mechanical.exhaust_valve]
timing = "per_cylinder"
order = 0.0
level = 0.45

[mechanical.piston_slap]
timing = "per_cylinder"
order = 0.0
level = 0.35

[mechanical.injector]
timing = "per_cylinder"
order = 0.0
level = 0.30

[mechanical.timing_chain]
timing = "crank_order"
order = 18.0
level = 0.20

[mechanical.gear_whine]
timing = "crank_order"
order = 32.0
level = 0.18

[mechanical.accessory]
timing = "crank_order"
order = 1.35
level = 0.15
```

Each mechanical source supports:
- `timing`:
  - `"per_cylinder"`: Impulses trigger synchronously with each individual cylinder's mechanical valve or piston stroke event.
  - `"crank_order"`: Continuous tone oscillating at a fixed multiple (`order`) of crankshaft rotational frequency.
- `order`: Crank harmonic multiplier (only used when `timing = "crank_order"`).
- `level`: Amplitude level in mix ($0.0$ to $1.0$).

---

### 3D Acoustic Apertures (`[acoustics]`)

Defines the physical 3D positions in space (vehicle coordinate system) where sound waves radiate into the environment:
- **X**: Lateral axis ($+X$ = vehicle right, $-X$ = vehicle left).
- **Y**: Longitudinal axis ($+Y$ = vehicle front, $-Y$ = vehicle rear).
- **Z**: Vertical axis ($+Z$ = vehicle roof/up, $-Z$ = ground/down).

```toml
[acoustics]
tailpipes = [
    [-0.40, -2.40, 0.32],
    [ 0.40, -2.40, 0.32],
]
intake = [0.0, 1.40, 0.65]
block  = [0.0, 0.80, 0.48]
```

- `tailpipes`: Array of $[X, Y, Z]$ points for each exhaust tip. Dual exhaust configurations specify two apertures, creating wide stereo imaging when listener moves around the car.
- `intake`: $[X, Y, Z]$ position of the intake snorkel or velocity stacks under the hood.
- `block`: $[X, Y, Z]$ acoustic center of the engine block vibrating radiating casing noise.

---

## 4. Acoustic Tuning & Physics Handbook

### Header Runner Resonance Tuning
Primary exhaust runner length directly tunes the acoustic quarter-wave reflection that scavenges the cylinder during valve overlap.

The resonant fundamental frequency of a primary pipe closed at the valve and open at the collector is:
$$f_1 = \frac{c}{4 L}$$
where $c \approx \sqrt{\gamma R T} \approx 600\text{ m/s}$ in hot exhaust gas ($900\text{ K}$).

To tune primary length $L$ for peak scavenging at engine speed $N$ (RPM):
$$L = \frac{(720^\circ - \text{EVO}) \times c}{12 \times N}$$
- **Shorter Primaries ($0.30 - 0.45\text{ m}$)**: Tune high-RPM power ($6500 - 8500\text{ RPM}$), producing a sharper, higher-pitched, brassier exhaust bark.
- **Longer Primaries ($0.70 - 1.05\text{ m}$)**: Tune low-to-midrange torque ($3000 - 5000\text{ RPM}$), producing a deep, throaty, hollow resonance.

### Muffler & Silencer Selection
- **To get an OEM / Street Sound**: Use an `ExpansionChamber` with `area_ratio = 4.0 - 5.5` and `stages = 2` followed by a `Helmholtz` resonator tuned to cruise drone RPM.
- **To get a Tuner / Performance Sound**: Use an `Absorptive` glasspack with `length = 0.35` and `packing_absorption = 0.60`. This eats harsh high frequencies above $2.5\text{ kHz}$ while letting deep bass fundamentals punch through.
- **To get a Track Day / Screamer Sound**: Set `mode = "straight_pipe"`. All expansion chambers are eliminated; only natural tube reflections and wave steepening color the sound.
- **To get a Raw Dragster / Dyno Sound**: Set `mode = "open_headers"`. Headers dump straight to atmosphere with zero muffling or secondary reflection.

### Exhaust Modes: Open Headers vs Straight Pipe vs Muffled
In `[exhaust]`, you can set the `mode` parameter:
```toml
[exhaust]
mode = "open_headers"   # Options: "muffled", "straight_pipe", "open_headers"
```
- `"muffled"`: Authentic road car sound utilizing all silencer stages.
- `"straight_pipe"`: Race car sound with intact header collection and long tailpipe column resonance, but no damping mufflers.
- `"open_headers"`: Drag / tractor pull sound with immediate short blowdown off the cylinder heads.

### Rev Limiter Pops, Bangs & Cadence
To achieve the aggressive "machine-gun firecracker" limiter sound:
1. In `[block]`:
   ```toml
   limiter_mode = "RotatingStutter"
   limiter_cut = "Spark"
   ```
2. In `[exhaust]`:
   ```toml
   cutout = true  # Enables tailpipe shock wave steepening
   ```
This cuts spark while continuing fuel flow. The unburnt mixture is pumped into the red-hot exhaust runners where it instantly detonates, producing sub-100 microsecond explosive shock wavefronts and high crest-factor bangs.

### Single Throttle vs Individual Throttle Bodies (ITBs)
- **Single Throttle Body (`[intake.throttle] type = "Single"`):**
  Air enters through a single restrictor into a large common plenum. The plenum acts as an acoustic low-pass filter, dampening intake bark and emphasizing deep, muffled Helmholtz induction thrum.
- **Individual Throttle Bodies (`[intake.throttle] type = "IndividualBodies"`):**
  Every cylinder has its own independent throttle butterfly directly at the intake runner mouth. On opening, unfiltered high-velocity suction pulses radiate directly into atmosphere, giving razor-sharp throttle crack and raw induction scream at high RPM.

---

## 5. Validation & Diagnostic Benchmarking

After creating or modifying an engine configuration, validate it with the test suite and diagnostic benchmark tool.

### 1. Automated Schema & Presets Verification
Run the configuration unit tests:
```bash
cargo test --lib bench::config
```
This tests:
- Round-trip serialization/deserialization to ensure no fields are lost or scrambled.
- Bank sequence ordering across all cylinders.
- Mode overrides (`open_headers`, `straight_pipe`).

### 2. Acoustic Diagnostic Tool
Run `acoustic_bench` to simulate 4.8 seconds of high-RPM limiter revs and measure acoustics:
```bash
cargo run --release --example acoustic_bench -- --engine <engine_name>
```

#### Diagnostic Flags:
- `--engine <name>`: Loads `engines/<name>.toml`.
- `--exhaust open-headers`: Overrides exhaust geometry to raw open headers.
- `--exhaust straight-pipe`: Overrides exhaust geometry to straight pipe.
- `--listener dyno`: Sets listener monitoring mode (`dyno`, `exhaust`, `cockpit`, `bystander`).
- `--compare <path_to_audio.mp3>`: Compares 10-octave spectral balance (31.5 Hz to 16 kHz) against a real-world reference recording.
- `--sweep <parameter>`: Sweeps one exhaust geometry parameter and prints its resonance table instead of rendering the dyno pull — see [Tuning the Exhaust: the Resonance Sweep](#tuning-the-exhaust-the-resonance-sweep) above.
- `--sweep-steps <n>`: Points in the sweep (default `5`).
- `--mode`: Compares open-headers, straight-pipe and muffled on one engine in a single table, instead of rendering the dyno pull.

#### Benchmark Output Metrics:
- **Peak Attack Rise Time**: Evaluates supersonic shock wavefront sharpness (Target: $< 200\text{ }\mu\text{s}$).
- **Crest Factor**: Evaluates dynamic punch of limiter bangs (Target: $> 14.0\text{ dB}$).
- **Pop Dynamic Contrast**: Ratio of backfire pop peak to pre-pop baseline floor (Target: $> 17.0\text{ dB}$).
- **10-Octave Spectral Delta**: Energy balance across Sub-Bass, Low-Bass, Midrange, Presence, and Bite/Crack bands.
