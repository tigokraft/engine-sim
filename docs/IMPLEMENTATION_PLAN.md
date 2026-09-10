# Engine audio realism — implementation plan

Roadmap for taking the synth from *physically motivated* to *physically
derived*: every engine acoustically distinct because its plumbing is distinct,
intake and exhaust notes emerging from real pipe calculation, and each claim
checkable against an analytic result or a recording.

Read the stage you are working on in full before touching code. Each stage
carries its files, its formulas, the tests that prove it, its commit sequence,
and a self-contained prompt.

The commit rule in [AGENTS.md](../AGENTS.md) is not optional and applies to every
stage: one change, one commit, one line, no attribution trailers, ever.

---

## Where the code stands today

Baseline at the first commit of this plan: **183 tests passing**, `cargo test
--release` clean.

```
per cylinder ─┐
              ├─▶ PulsePool ─▶ ExhaustRunner ─▶ Muffler ─▶ pan ─┐
per cylinder ─┘   (one per bank, shared)                        │
                                                                ├─▶ DC ─▶ Block ─▶ clip
IntakeVoice (filtered noise, no pipe) ──────────────────────────┤
TurboVoice / MechanicalVoice / BackfireVoice ───────────────────┘
```

What is wrong with it, in the order this plan fixes it:

| # | Problem | Where |
|---|---|---|
| 1 | No measurement harness, so no change can be proven | — |
| 2 | Knock is solved and displayed, never heard | `physics/thermodynamics.rs:173` |
| 3 | Blowdown smoothed over 15 ms — six firings at 6000 rpm — so every cylinder gets the same amplitude | `audio/dsp.rs:1372` |
| 4 | Crank speed is smoothed constant; no intra-cycle ripple | `audio/dsp.rs:1551` |
| 5 | Mechanical layer is one click plus one noise bed | `audio/dsp.rs:970` |
| 6 | Every engine shares one runner length, reflection, muffler, block mass | `audio/mod.rs:472` |
| 7 | No radiation model; tailpipe is a lowpass where physics wants a differentiator | `audio/filters.rs:702` |
| 8 | One shared runner per bank, hand-set reflection coefficient | `audio/dsp.rs:749` |
| 9 | No intake pipe at all, so no intake note | `audio/dsp.rs:846` |
| 10 | Nothing from combustion reaches the block; structure is one biquad on the bus | `audio/filters.rs:1008` |
| 11 | No thermal state: wall temperature fixed, no cold engine | `physics/thermodynamics.rs:236` |
| 12 | Bulk exhaust temperature for all delays; no mean-flow convection | `audio/filters.rs:863` |
| 13 | Exhaust panned, everything else centred; no space, no apertures | `audio/dsp.rs:1626` |
| 14 | AFR and spark timing are constants | `physics/thermodynamics.rs` |
| 15 | Physics pipe is linear-acoustic at Mach 0.5 | `physics/engine_block.rs:683` |

---

## Ordering and dependencies

```
S0 measurement harness ─────────────────────────────┐ (gate for every stage)
                                                    │
S1 free wins  ─── S2 mechanical set ────────────────┤ independent, do first
                                                    │
S3 geometry spec ─┬─ S4 radiation ──┐               │
                  ├─ S5 exhaust network ─┬─ S6 intake network
                  │                      └─ S10 pipe numerics
                  └─ S9 thermal state
S7 excitation tables ─── supersedes the S1 amplitude work
S8 structural path ───── needs S7 for dP/dtheta
S11 propagation ──────── needs S4 (apertures radiate)
S12 fuelling/control ─── independent
S13 induction hardware ─ independent
S14 diesel ───────────── needs S8
S15 physics pipe solver  needs S0, replaces part of S5's assumptions
S16 calibration ──────── needs S0, meaningful after S5+S6
```

Do **S0 first**. Everything after it is measured against it. S1 and S2 are the
cheapest audible wins and need no new architecture — run them while S3's design
is being settled.

## Definition of done, every stage

1. `cargo test --release` green, with the stage's new analytic tests included.
2. `cargo fmt` and `cargo clippy` clean.
3. `cargo run --release --example engine_audio -- --offline` reports no
   discontinuity and no silent block.
4. The stage's measurement (order spectrum, CPU, or resonance table) recorded in
   `docs/measurements/`.
5. Nothing in the audio callback allocates, locks, or panics.
6. Commits are one line each, granular, no trailers.

---

# Stage 0 — Measurement harness

**Goal.** Make every later claim falsifiable. Nothing else starts until a change
in timbre can be seen as a number.

**Why.** Every stage after this asserts something about a spectrum — "the V12
sits an octave above the four", "the collector reflection is 0.31", "the intake
note peaks at the ram frequency". Without order analysis those are opinions.

**Files.** New `src/analysis/mod.rs`, `src/analysis/orders.rs`,
`examples/measure.rs`; extend `examples/engine_audio.rs`'s existing
`render_offline` and `write_wav` rather than duplicating them.

**Design.**

- `RenderScript` — deterministic rpm/throttle/gear profile, seeded RNG, so two
  runs of the same build are bit-identical and two builds are comparable.
- Fixed profiles: `idle_hold`, `sweep_up`, `sweep_down`, `tip_in`, `overrun_cut`,
  `limiter_bounce`. One WAV per preset per profile.
- `orders::track(samples, rpm_curve)` — STFT, then for each engine order `n`
  extract the magnitude at `f = n * rpm / 60`. A four-stroke's firing order is
  `N_cyl / 2`, so a V8 is order 4, an inline-four order 2, a V12 order 6. Report
  orders 0.5 through 24 in dB.
- `orders::resonances(samples)` — long-term average spectrum with peak picking,
  for reading pipe resonances back out of the audio.
- CPU bench: samples rendered per second of wall clock, as a multiple of
  real time, per preset. Record the headroom now, before 17 delay lines exist.

**Tests.**

- A synthesised pure tone at `4 * rpm / 60` reads as order 4 at the right dB and
  nothing at order 3 or 5.
- The harness is deterministic: two renders of one script are bit-identical.
- A known two-tone signal's resonance picker finds both peaks within 1 %.

**Commits.**

```
add analysis module root
add order tracking
add resonance peak picking
add deterministic render script
add measure example
add measurement baseline for the catalogue
```

**Prompt.**

> In the Rust crate at this repo root, build the measurement harness described as
> Stage 0 in `docs/IMPLEMENTATION_PLAN.md`. Read that stage first, and read
> `AGENTS.md` for the commit rule — one change per commit, one line per message,
> never a `Co-Authored-By` trailer.
>
> Add `src/analysis/` with order tracking and resonance peak picking, and an
> `examples/measure.rs` that renders every preset in
> `EnginePreset::catalogue()` through fixed deterministic scripts, writes the
> WAVs, and prints an order table in dB plus a real-time multiple for CPU.
> Reuse the offline render and WAV writer already in
> `examples/engine_audio.rs` — factor them out rather than copying them.
>
> Prove it with tests: a synthetic tone at order 4 must read as order 4 and not
> its neighbours; two renders of one script must be bit-identical. Then commit
> the measured baseline for all six presets under `docs/measurements/` so later
> stages have something to regress against. Keep the physics and the synth
> untouched in this stage.

---

# Stage 1 — Wins the physics already paid for

**Goal.** Three things the solver computes and the synth discards: knock,
per-cylinder blowdown pressure, and intra-cycle crank speed.

**Why.** No new architecture, no new geometry, and the largest change in
character per line of code in the whole plan.

**Files.** `src/audio/dsp.rs` (snapshot, new `KnockVoice`, `advance_crank`),
`src/audio/mod.rs` (`SnapshotSource::sample`), `src/physics/engine_block.rs`
(expose per-cylinder state and instantaneous torque).

### 1a — Knock

Livengood–Wu already runs; `knock_integral` and `knocking` reach the TUI and stop.
Knock is the burned gas ringing in the bore cavity. First circumferential mode:

```
f_knock = rho_10 * c / (pi * B)        rho_10 = 1.8412
c       = sqrt(gamma * R * T_burned)
```

So a 94 mm bore pings near 5.5 kHz and a 78 mm bore near 6.6 kHz — **pitch from
bore**, free engine differentiation. Add `knock_intensity` and `knock_bore` to
`EngineSnapshot`; a `KnockVoice` of two or three high-Q bandpasses at
`rho_10`, `rho_20 = 3.0542`, `rho_01 = 3.8317` times `c / (pi B)`, excited by a
short noise burst whose amplitude comes from how far past 1.0 the integral went,
decaying in 2–5 ms. Route it through the structural path, not the exhaust — knock
is heard through the block.

### 1b — Per-cylinder blowdown

`blowdown_pa: Smoothed::new(0.0, fs, 0.015)` is six times slower than the firing
interval at 6000 rpm, so cylinder-to-cylinder differences are averaged away and
then re-invented by `CycleVariation`. Replace the scalar with
`blowdown_delta: [f32; MAX_CYLINDERS]`, sampled from the phase ring at each
cylinder's own EVO phase — `SnapshotSource::sample` already does exactly this for
one cylinder, and `block.sample_of(i)` gives the rest. Keep a short glide per
cylinder, not one shared smoother. Then reduce `CycleVariation`'s amplitude sigma
to what remains genuinely stochastic and let the physics carry the rest.

### 1c — Intra-cycle crank ripple

`advance_crank` uses `cycle_hz` smoothed over 30 ms: a perfectly uniform crank. A
real crank accelerates on each power stroke and decelerates against compression —
several percent within one revolution at idle. Integrate at audio rate:

```
dw/dt = (T_indicated(theta) - T_load - T_friction) / I
```

`PhaseRing::indicated_torque` and the preset's `inertia` are both already there;
pass the torque curve and inertia in the snapshot and let the synth run its own
crank. Optionally add crank–flywheel torsional compliance as a two-inertia model:

```
f_torsional = (1 / 2pi) * sqrt(k * (1/I_crank + 1/I_flywheel))
```

**Tests.**

- Knock pitch scales as `1/B`: doubling bore halves the mode frequency.
- Knock is silent when `knock_integral < 1`.
- Two cylinders given different EVO pressures produce different pulse amplitudes
  in the same cycle — the test the current code cannot pass.
- Firing intervals are non-uniform at idle and converge toward uniform at the
  limiter; every cylinder still fires exactly once per cycle.

**Commits.**

```
expose per-cylinder evo pressure on the block
carry per-cylinder blowdown in the snapshot
fire each cylinder from its own pressure
reduce synthetic amplitude variation to the stochastic remainder
test cylinders fire at different amplitudes
carry knock intensity and bore in the snapshot
add knock resonance voice
tune knock modes from bore and gas temperature
test knock pitch scales inversely with bore
carry indicated torque and inertia in the snapshot
integrate crank speed at audio rate
test firing intervals ripple at idle
```

**Prompt.**

> Implement Stage 1 of `docs/IMPLEMENTATION_PLAN.md` in this Rust crate. Read
> that stage and `AGENTS.md` first — commit granularly, one line per message, no
> `Co-Authored-By` ever.
>
> Three separate pieces, committed separately:
>
> 1. **Knock.** `KnockModel` in `src/physics/thermodynamics.rs` already runs the
>    Livengood–Wu integral and it reaches the TUI but never the audio. Carry
>    `knock_intensity` and bore on `EngineSnapshot`, add a `KnockVoice` to
>    `src/audio/dsp.rs` with modes at `rho * sqrt(gamma R T) / (pi * bore)` for
>    `rho` = 1.8412, 3.0542, 3.8317, excited by a short burst scaled by how far
>    past 1.0 the integral went. Route it through the block resonator, not the
>    exhaust bus.
> 2. **Per-cylinder blowdown.** The single `blowdown_pa` smoother at
>    `src/audio/dsp.rs:1372` has a 15 ms time constant, slower than the firing
>    interval — it erases real cylinder-to-cylinder pressure differences.
>    Replace it with a per-cylinder array sampled from the phase ring at each
>    cylinder's own EVO phase, and cut `CycleVariation`'s amplitude sigma to
>    match what is genuinely stochastic once the physics carries the rest.
> 3. **Crank ripple.** `advance_crank` uses a 30 ms-smoothed constant speed.
>    Integrate `dw/dt = (T_indicated - T_load - T_friction)/I` at audio rate from
>    the torque curve and inertia so firing intervals breathe on their own.
>
> Keep `EngineSnapshot` `Copy` and free of heap indirection — it crosses an
> `rtrb` queue into the audio callback. Add the analytic tests the stage lists,
> and re-run the Stage 0 measurement to show the order spectrum changed.

---

# Stage 2 — The mechanical source set

**Goal.** Replace one uniform valve click plus one noise bed with the actual set
of order-locked mechanical sources.

**Why.** Strip combustion out of an idle recording and what remains is most of
the sound. This is the largest single idle-realism win available, and each source
is ten lines in the existing envelope-plus-biquad idiom.

**Files.** `src/audio/dsp.rs` (`MechanicalVoice` → `MechanicalRig`),
`src/audio/filters.rs` (a small modal bank), `src/bench.rs` (per-preset
mechanical spec).

**Design.** One impulsive-source primitive — rate, jitter, envelope, body filter,
level law — instanced per source:

| source | rate | notes |
|---|---|---|
| intake valve seating | 1/cycle/cyl | lighter valve, brighter, earlier |
| exhaust valve seating | 1/cycle/cyl | heavier, hotter, duller |
| piston slap | 1/cycle/cyl at TDC compression | scales with peak pressure, low and thuddy |
| injector | 1/cycle/cyl | sharp, ~4 kHz, dominant on diesel |
| timing chain or belt | chain-pitch order | continuous whirr |
| gear whine | tooth-count order | near-pure tone: oil pump, straight-cut gears, dog box |
| accessory drive | **non**-integer crank order | belt ratio is not commensurate — this is what stops an idle sounding synchronised |

Split the existing single click into intake and exhaust seatings with their own
bodies. Give every source per-instance jitter — real lash and tolerances differ
across a head, which `MechanicalVoice` already gestures at with one random
amplitude.

**Tests.**

- Each source's event rate matches its order at three speeds.
- The accessory order is irrational relative to the crank: no event coincides
  twice within one measured cycle.
- Piston slap scales with peak cylinder pressure and vanishes on a cut cylinder,
  while valve seatings do not.
- Total mechanical RMS still sits at unity so `mechanical_level` keeps meaning
  the same thing (extend `mechanical_layers_sit_at_unity`).

**Commits.**

```
add impulsive mechanical source primitive
split valve seating into intake and exhaust events
add piston slap keyed to peak pressure
add injector tick
add timing chain whirr
add gear whine at tooth-count order
add accessory drive at non-integer order
add per-preset mechanical spec
test mechanical event rates track their orders
test mechanical rig sits at unity rms
```

**Prompt.**

> Implement Stage 2 of `docs/IMPLEMENTATION_PLAN.md`. Read it and `AGENTS.md`
> first; commit each source separately with a one-line message and no trailers.
>
> `MechanicalVoice` in `src/audio/dsp.rs` is currently one 3.2 kHz valve click at
> a uniform rate plus a twice-lowpassed noise bed. Generalise it into a rig of
> order-locked impulsive sources sharing one primitive (rate, jitter, envelope,
> body resonance, level law): separate intake and exhaust valve seatings, piston
> slap at TDC compression scaled by peak cylinder pressure, injector tick, timing
> chain, gear whine at a tooth-count order, and an accessory drive at a
> deliberately non-integer crank order. Put the per-engine choices (which sources
> exist, their orders and levels) on `EnginePreset` in `src/bench.rs`.
>
> Preserve the unity-RMS calibration so `SynthConfig::mechanical_level` keeps its
> meaning — extend the existing `mechanical_layers_sit_at_unity` test rather than
> replacing it. Nothing may allocate in the callback.

---

# Stage 3 — Acoustic geometry, and engines that differ by construction

**Goal.** One geometry description per engine, read by both the physics pipe and
the audio path, so no acoustic constant is ever chosen by ear again.

**Why.** `SynthConfig::from_block` copies only the firing taps out of the block;
everything else falls through to `uniform()`. The inline-four, the V12 and the
Wankel all get `runner_length: 0.45`, `runner_reflection: 0.55`, the same 8-litre
muffler and `block_mass: 180.0`. They are rhythm variations on one exhaust
system. This stage is mostly plumbing and it changes more than any filter will.

**Files.** New `src/physics/plumbing.rs` (geometry, shared by both sides),
`src/bench.rs` (per-preset systems), `src/audio/mod.rs` (`from_block` reads it),
`src/audio/dsp.rs` (`SynthConfig` carries it instead of loose scalars).

**Design.** Geometry as data, in SI, with no acoustic coefficients in it — those
are *derived*:

```rust
pub struct PipeSection { pub length: f64, pub area: f64, pub wall_temperature: f64 }
pub struct Collector    { pub inlets: usize, pub outlet_area: f64, pub taper_length: f64 }

pub enum Crossover { None, HPipe { position: f64, area: f64 }, XPipe { position: f64 }, Balance180 }

pub enum Silencer {
    Straight,                                     // no silencer fitted
    Helmholtz(MufflerGeometry),                   // what exists today
    ExpansionChamber { length: f64, area_ratio: f64, stages: usize },
    Absorptive { length: f64, area: f64, loss_db_per_m: f64 },
    QuarterWaveStub { length: f64, area: f64 },   // drone killer
}

pub struct ExhaustSystem {
    pub primaries: Vec<PipeSection>,   // one per cylinder, unequal lengths allowed
    pub collector: Collector,
    pub secondary: Vec<PipeSection>,
    pub crossover: Crossover,
    pub silencers: Vec<Silencer>,
    pub tailpipe: PipeSection,
    pub tailpipe_flanged: bool,
}

pub enum ThrottleLayout { Single { bore: f64 }, IndividualBodies { bore: f64 } }

pub struct IntakeSystem {
    pub runners: Vec<PipeSection>,     // one per cylinder
    pub plenum_volume: f64,
    pub throttle: ThrottleLayout,
    pub airbox: Option<PipeSection>,
    pub snorkel: Option<PipeSection>,
    pub trumpet_flanged: bool,
}
```

The reflection coefficient, the resonance frequencies and the losses all come out
of this; none of them is stored. Delete `runner_length`, `runner_reflection` and
the bare `muffler` from `SynthConfig` in the same stage so there is no second
source of truth. Keep `MufflerGeometry` — it becomes one `Silencer` variant.

Also derive `block_mass` per preset rather than defaulting all six to 180 kg.

**The catalogue this produces** (starting values, to be calibrated in S16):

| preset | primaries | collector | crossover | silencer | intake |
|---|---|---|---|---|---|
| Inline-4 | 4 × 0.40 m, 38 mm | 4-1 | — | expansion chamber | single throttle + airbox |
| Cross-plane V8 | 2 × 4 unequal ≈ 0.55 m, 44 mm | 4-1 per bank | H-pipe | large chamber | plenum, long runners |
| Flat-plane V8 | 2 × 4 equal 0.42 m, 41 mm | 4-2-1 | X-pipe | straight-through | ITBs, short |
| V10 | 2 × 5 mixed 0.36 m, 40 mm | 5-1 | X-pipe | twin chamber | ITBs |
| V12 | 2 × 6 short 0.30 m, 35 mm | 6-1 | none | twin small | ITBs, very short |
| Wankel | 2 ports 0.60 m, 48 mm | 2-1 | — | absorptive | peripheral, huge |

**Tests.**

- Every preset's derived collector reflection differs, and each equals
  `(A_primary - A_outlet) / (A_primary + A_outlet)`.
- Each preset's primary quarter-wave fundamental `c / 4L` lands where the table
  says, on hot gas.
- No two presets in the catalogue produce the same set of derived acoustic
  constants — the regression test against "all engines sound alike".
- The physics pipe and the audio path, asked for the same pipe's round-trip time,
  agree within a sample.

**Commits.**

```
add plumbing geometry types
derive collector reflection from area ratio
add exhaust system to the engine preset
add intake system to the engine preset
give each preset its own block mass
read plumbing geometry in synth config
drop hand-tuned runner length and reflection
test every preset derives distinct acoustics
```

**Prompt.**

> Implement Stage 3 of `docs/IMPLEMENTATION_PLAN.md`. Read that stage and
> `AGENTS.md` first. Commit each piece separately, one line, no trailers.
>
> Today `SynthConfig::from_block` (`src/audio/mod.rs:472`) copies only the firing
> taps out of the block, so every engine in the catalogue inherits
> `runner_length: 0.45`, `runner_reflection: 0.55`, one default muffler and
> `block_mass: 180.0`. Fix that at the root: add `src/physics/plumbing.rs` with
> the `ExhaustSystem` / `IntakeSystem` types the stage specifies, put one of each
> on every `EnginePreset` in `src/bench.rs` using the table in the stage, and make
> `SynthConfig` read the geometry instead of carrying tuned scalars.
>
> The rule that matters: geometry is stored, acoustics are derived. A reflection
> coefficient must be computed as `(A1-A2)/(A1+A2)` from areas, never stored as a
> number someone liked. Remove `runner_length`, `runner_reflection` and the bare
> `muffler` field in the same change so there is no second source of truth, and
> keep the existing `MufflerGeometry` as one `Silencer` variant.
>
> The important test: assert that no two presets derive the same set of acoustic
> constants. That is the regression test for "all the engines sound the same".
> Behaviour may change audibly here — re-run the Stage 0 measurement and commit
> the new baseline.

---

# Stage 4 — Radiation: the mouth, not the pipe

**Goal.** Model what leaves the tailpipe and the intake mouth, instead of
lowpassing the inside of the pipe.

**Why.** `Muffler` ends in a `tailpipe_cutoff` lowpass. Physically a pipe mouth
is a *differentiator* (monopole radiation goes as `dQ/dt`, +6 dB/octave) behind a
frequency-dependent reflection that keeps the low end *in* the pipe. Getting the
tilt backwards is the single biggest "synth versus recording" tell, and it is
cheap to fix.

**Files.** New `src/audio/radiation.rs`; `src/audio/filters.rs` (the mouth
termination replaces the ad-hoc open end in `ExhaustRunner`).

**Design.** An open end has three parts, all derived from mouth radius `a`:

1. **Reflection** `R(omega)`: `|R| -> 1` for `ka << 1`, `-> 0` for `ka >> 1`, with
   the corner at `ka = 1`, i.e. `f_c = c / (2 pi a)`. A first-order fit of
   Levine–Schwinger is enough; DC gain is `-1` (pressure release).
2. **End correction**: the pipe is acoustically longer than it is —
   `delta = 0.6133 a` unflanged, `0.8216 a` flanged. Add it to the delay.
3. **Radiated signal** = the transmitted part `(1 + R)`, differentiated for
   monopole radiation, scaled by mouth area.

**Tests.**

- A closed–open pipe of length `L` with the mouth fitted resonates at
  `c / (4 (L + delta))`, not `c / 4L` — the end correction is measurable.
- Reflection magnitude falls monotonically through `ka = 1`.
- Radiated spectrum of a flat-spectrum excitation rises at +6 dB/octave below the
  mouth corner and flattens above it.
- A wider mouth radiates the low end better: doubling `a` lowers the corner by an
  octave.

**Commits.**

```
add mouth radiation module
add levine-schwinger reflection fit
apply open-end length correction
radiate the transmitted wave with monopole tilt
terminate the runner at a real mouth
test end correction lowers the pipe fundamental
test radiated tilt is six db per octave
```

**Prompt.**

> Implement Stage 4 of `docs/IMPLEMENTATION_PLAN.md`. Read it and `AGENTS.md`
> first; granular one-line commits, no trailers.
>
> Add `src/audio/radiation.rs` with a proper open-end termination and use it in
> place of the ad-hoc open end inside `ExhaustRunner`
> (`src/audio/filters.rs:772`) and the `tailpipe_cutoff` lowpass in `Muffler`.
> Three parts, all from mouth radius: a reflection filter with `|R|->1` at
> `ka<<1` and `->0` at `ka>>1` cornering at `f = c/(2 pi a)` with DC gain `-1`; an
> end correction of `0.6133a` unflanged or `0.8216a` flanged added to the delay;
> and a radiated output equal to the transmitted part `(1+R)` differentiated for
> monopole radiation and scaled by mouth area.
>
> The current lowpass tilts the wrong way — the radiated field rises with
> frequency below the mouth corner. Prove the fix with tests: the fundamental
> must drop to `c/(4(L+delta))`, and the radiated tilt must measure +6 dB/octave
> below the corner. Keep the loop a contraction at every frequency so no
> temperature glide can make it ring away.

---

# Stage 5 — The exhaust waveguide network

**Goal.** Replace one lumped runner per bank with a network of pipes joined by
scattering junctions: per-cylinder primaries, a real collector, a real crossover,
composed silencers, and the Stage 4 mouth.

**Why.** This is the pipe calculation. A header's character is unequal primaries
meeting at an area discontinuity; one shared delay line per bank cannot produce
it, and a hand-set reflection coefficient cannot respond to geometry.

**Files.** New `src/audio/waveguide.rs`; `src/audio/dsp.rs` (`ExhaustBank`
becomes a network instance).

**Design.** Four primitives, everything else composed from them.

**Pipe** — bidirectional delay line carrying `p+` and `p-`, fractional read
(`DelayLine` and `Smoothed` already exist), per-segment loss filter.

**Scattering junction** — N pipes meeting, admittance `Y_i = A_i / (rho c)`:

```
p_J   = 2 * sum(Y_i * p_i_plus) / sum(Y_i)
p_i_minus = p_J - p_i_plus
```

For two pipes this collapses to `r = (A1 - A2) / (A1 + A2)`: expansion inverts
the sign, `A2 -> infinity` gives `r -> -1`. **The collector reflection stops
being a knob and becomes arithmetic.**

**Terminations** — the valve end's reflection glides with valve effective area
against pipe area (rigid when shut, transmitting when open), driven from the
`ValveEvent` lift the solver already computes. The open end is Stage 4's mouth.

**Wall loss** — per-segment one-pole, scaled the way physics says:

```
alpha = (1 / (a c)) * sqrt(pi f nu) * (1 + (gamma - 1) / sqrt(Pr))
```

`alpha ~ sqrt(f) / a`, so a 38 mm bike primary is dramatically duller than a
76 mm truck pipe — free differentiation, currently absent.

**Composed elements**, no special cases: expansion chamber = two area steps a
length apart; quarter-wave stub = a delay with a rigid end on a 3-port junction;
tapered collector or megaphone = a chain of short cylinders with rising area;
H-pipe or X-pipe = a junction linking the two banks, which is precisely what
separates a flat-plane from a cross-plane V8 at equal firing order.

**Budget.** A V8 is ~8 primaries + 2 collectors + crossover + 2 silencers + 2
tailpipes ≈ 17 delay lines and a handful of junctions per sample: a few percent
of one core at 48 kHz. Hold the Stage 0 CPU number as the gate, and keep the
network allocated once at construction.

**Tests.**

- A single closed–open pipe resonates at `c / 4L` within 1 %.
- A two-pipe junction reflects exactly `(A1 - A2) / (A1 + A2)`.
- An N-port junction conserves volume flow and is passive: total outgoing energy
  never exceeds incoming.
- Expansion chamber transmission loss matches the analytic result
  `TL = 10 log10[1 + (1/4)(m - 1/m)^2 sin^2(kL)]`, `m = A2/A1`.
- A quarter-wave stub notches at `c / 4L_stub`.
- A narrower pipe is measurably duller at the mouth than a wide one of equal
  length.
- Raising gas temperature raises every resonance by `sqrt(T2/T1)`.
- An X-pipe measurably transfers energy between banks; `Crossover::None` does not.
- CPU stays inside the Stage 0 budget; no allocation in the callback.

**Commits.**

```
add waveguide pipe primitive
add scattering junction
test junction reflects the area ratio
add viscothermal wall loss
add valve-end termination from lift
compose expansion chamber from area steps
compose quarter-wave stub
compose tapered collector
add bank crossover junction
build the network from the exhaust system
replace the per-bank runner with the network
test pipe resonates at c over four l
test chamber transmission loss matches theory
test crossover transfers energy between banks
```

**Prompt.**

> Implement Stage 5 of `docs/IMPLEMENTATION_PLAN.md` — the exhaust waveguide
> network. Read that stage, Stage 3 (whose geometry types you consume) and
> `AGENTS.md` first. One commit per primitive, one line each, no trailers.
>
> Add `src/audio/waveguide.rs` with four primitives — bidirectional pipe with
> fractional delay, N-port scattering junction, terminations, viscothermal loss —
> and compose everything else from them: expansion chamber, quarter-wave stub,
> tapered collector, bank crossover. Then replace the single `ExhaustRunner` per
> bank in `src/audio/dsp.rs` with a network instantiated from the
> `ExhaustSystem` geometry: one primary per cylinder, a collector junction, the
> crossover, the silencer chain, the tailpipe and the Stage 4 mouth.
>
> The junction is the point of the stage. Use `p_J = 2*sum(Y_i p_i+)/sum(Y_i)`
> with `Y_i = A_i/(rho c)` and `p_i- = p_J - p_i+`, so a two-pipe case gives
> `r = (A1-A2)/(A1+A2)` and the collector reflection is arithmetic rather than a
> tuned constant. Wall loss must scale as `sqrt(f)/a` so a narrow pipe is duller
> than a wide one.
>
> Every element gets an analytic test, not a golden waveform: `c/4L` for a
> closed-open pipe, the exact junction reflection, expansion-chamber transmission
> loss against `10log10[1+(1/4)(m-1/m)^2 sin^2(kL)]`, a stub notch at
> `c/4L_stub`, and `sqrt(T2/T1)` scaling with temperature. Allocate the whole
> network at construction — nothing in the callback may allocate. Check CPU
> against the Stage 0 baseline and report the real-time multiple.
>
> The existing `waveguide_damping` Transit-Time Decision Rule exists to hide a
> comb artefact that a properly lossy network with a real mouth should not
> produce. Measure whether it is still needed; if it is not, say so and remove it
> in its own commit rather than leaving a workaround in place.

---

# Stage 6 — The intake network and the intake note

**Goal.** Give the intake side the same network, and excite it with the events
that actually make induction noise.

**Why.** `IntakeVoice` is bandpassed noise with a flow-driven centre frequency —
nothing in it is pitched, which is exactly why there is no intake note. Once
Stage 5's primitives exist, the intake is the same network mirrored, and the note
falls out.

**Files.** New `src/audio/intake_voice.rs` (replacing `IntakeVoice`),
`src/audio/waveguide.rs` (reused as-is), `src/audio/mod.rs` (per-cylinder port
flow in the snapshot).

**Design.** Network:

```
intake valve -> port -> runner -> plenum junction (N runners) -> throttle
    -> airbox -> snorkel -> mouth radiation -> listener
```

The throttle is a **time-varying area restriction inside the network**, so
throttle response is geometric rather than a gain law. ITBs are short undamped
runners straight to atmosphere; a single throttle with a big airbox is muted with
a honk at the box resonance. Same code, different geometry.

Three excitations, all from state the solver already has:

1. **Induction gulp** — a *rarefaction* at IVO, `p' = -rho c u = -c * mdot / A`.
2. **Valve-closing slam** — the loudest single intake event and completely absent
   today. At IVC the moving column hits a closed end and its inertia becomes a
   hard positive pulse that travels up the runner and out of the mouth. Water
   hammer; amplitude from `d(mdot)/dt` at IVC. **This is the bark of an ITB
   engine** — without it the intake can only ever hiss.
3. **Turbulent broadband** — what exists now, but the level law is wrong.
   Orifice noise is dipole: power goes as `u^6`, so amplitude goes as `u^3`.
   `IntakeVoice::tune` uses `flow^1.5`, which is power as `u^3` — about half the
   exponent, which is why induction never comes alive under load.

**Tests.**

- The mouth output is pitched at the firing order, not merely noisy: order
  analysis shows a peak at `N_cyl/2` order.
- The plenum Helmholtz ram peak appears at
  `f = (c / 2pi) sqrt(A_runner / (V_plenum L_runner))`.
- A shorter runner raises the intake note by `1/L`.
- Closing the throttle attenuates the mouth output through the area term without
  any explicit gain law.
- Valve-slam amplitude scales with `d(mdot)/dt` at IVC and with runner length.
- ITB geometry produces measurably more high-order content than single-throttle
  geometry on the same engine.

**Commits.**

```
carry per-cylinder port flow in the snapshot
add intake network from the intake system
add induction rarefaction at valve opening
add valve-closing water hammer
correct orifice noise to a dipole level law
model the throttle as a network area restriction
retire the filtered-noise intake voice
test intake note is pitched at firing order
test plenum ram peak lands at the helmholtz frequency
test itbs are brighter than a single throttle
```

**Prompt.**

> Implement Stage 6 of `docs/IMPLEMENTATION_PLAN.md` — the intake network. Read
> that stage, Stage 5 (whose waveguide primitives you reuse unchanged) and
> `AGENTS.md`. Granular one-line commits, no trailers.
>
> Replace `IntakeVoice` (`src/audio/dsp.rs:846`), which is bandpassed noise with
> no pipe in it, with a real network built from `IntakeSystem`: port, per-cylinder
> runner, plenum junction, throttle as a time-varying area restriction, airbox,
> snorkel, mouth radiation. Excite it with three things, each its own commit: the
> rarefaction the piston pulls at IVO (`p' = -c*mdot/A`), the valve-closing water
> hammer at IVC from `d(mdot)/dt` — the loudest intake event and entirely missing
> today — and turbulent broadband corrected to a dipole law, amplitude as `u^3`
> rather than the present `flow^1.5`.
>
> The test that matters: the mouth output must be *pitched* at the firing order,
> and the plenum ram peak must land at
> `(c/2pi) sqrt(A_runner/(V_plenum*L_runner))`. Also assert that a shorter runner
> raises the note as `1/L`, that closing the throttle attenuates through the area
> term with no explicit gain law, and that ITB geometry is brighter than
> single-throttle geometry on the same engine. Per-cylinder port flow will need to
> reach the snapshot — keep it `Copy`, fixed-size, no heap.

---

# Stage 7 — Phase-resolved excitation

**Goal.** Stop synthesising pulse shapes. Drive both networks from the solver's
own pressure and flow curves.

**Why.** `advance_crank` builds a two-exponential envelope with `attack =
0.00016` and `noise_depth = 0.5` hardcoded, from one scalar per firing. The real
shape depends on port area, valve lift rate, bore and pressure ratio: a 4-valve
bike engine cracks, a peripheral-port rotary is a wide slow hot event. The
`PhaseRing` already holds the whole cycle in 720 cells — the information exists
and is being thrown away.

**Files.** `src/audio/dsp.rs` (`EngineSnapshot`, excitation path),
`src/audio/mod.rs` (`SnapshotSource::sample`), `src/physics/engine_block.rs`
(downsampled cycle export).

**Design.** Add fixed-size tables to the snapshot — cylinder pressure and port
mass flow versus crank phase, downsampled from 720 to 128 or 256 points:

```rust
pub const CYCLE_TABLE: usize = 128;
pub cylinder_pressure: [f32; CYCLE_TABLE],   // Pa
pub exhaust_port_flow: [f32; CYCLE_TABLE],   // kg/s
pub intake_port_flow:  [f32; CYCLE_TABLE],   // kg/s
```

512 bytes each; the struct stays `Copy`, `rtrb` stays happy, and at 240 Hz it is
~120 kB/s. The audio thread reads them at crank rate with interpolation, so
excitation is the gas dynamics rather than an approximation of it — for both
sides, and it starts differing per engine automatically. Interpolate *between*
successive snapshots as well, rather than stepping.

With this in place, `blowdown_degrees`, the fixed attack and `noise_depth` all
go away, and `CycleVariation` shrinks to what is genuinely stochastic.

**Tests.**

- Table round-trip: a known analytic pressure curve survives downsampling and
  playback to within a stated tolerance.
- Playback rate follows crank speed exactly; no drift over 10^6 samples.
- Two engines with different valve timing produce measurably different pulse
  spectra with no per-engine tuning constant anywhere.
- A snapshot arriving late produces no discontinuity (extend
  `no_discontinuity_when_state_jumps` and `starvation_keeps_the_engine_running`).
- `EngineSnapshot` is still `Copy` and contains no indirection.

**Commits.**

```
export a downsampled cycle from the phase ring
carry cycle tables in the snapshot
play the exhaust excitation from the pressure table
play the intake excitation from the flow table
interpolate between successive snapshots
drop the parametric pulse envelope
test excitation playback tracks crank speed
test valve timing changes the pulse spectrum
```

**Prompt.**

> Implement Stage 7 of `docs/IMPLEMENTATION_PLAN.md`. Read it and `AGENTS.md`
> first; one-line commits, no trailers.
>
> `advance_crank` in `src/audio/dsp.rs` synthesises every exhaust pulse as two
> exponentials with a hardcoded 0.16 ms attack and a fixed noise depth, from a
> single scalar. Meanwhile `PhaseRing` in `src/physics/engine_block.rs` holds the
> entire cycle in 720 cells. Carry that instead: downsample cylinder pressure and
> both port flows to fixed-size `[f32; 128]` tables on `EngineSnapshot`, and drive
> both the exhaust and intake networks by playing them back at crank rate with
> interpolation — including interpolation between successive snapshots so a 240 Hz
> physics loop does not step the excitation.
>
> `EngineSnapshot` must stay `Copy` with no heap indirection; it crosses an `rtrb`
> queue into the callback. Once the tables drive the excitation, remove
> `blowdown_degrees`, the fixed attack and `noise_depth`, and cut `CycleVariation`
> to the genuinely stochastic remainder.
>
> Prove the pulse shape is now physical: two presets with different valve timing
> must show measurably different pulse spectra with no per-engine tuning constant
> involved. Extend the existing starvation and state-jump tests rather than
> replacing them.

---

# Stage 8 — The structural path

**Goal.** Give combustion a route to the listener that is not the exhaust pipe:
cylinder pressure rise rate driving a modal block.

**Why.** `BlockResonator` is one peaking biquad sitting on the *output bus*,
excited by the exhaust mix. Physically the dominant structure-borne path is
`dP/dtheta` hammering the block — that is what "combustion noise" means in NVH,
and it is most of what a diesel is. Right now nothing from combustion reaches the
structure except as a filtered copy of the exhaust.

**Files.** New `src/audio/structure.rs`; `src/audio/filters.rs` (retire the single
`BlockResonator`), `src/audio/dsp.rs` (route knock and the mechanical rig through
it), `src/bench.rs` (per-preset structural spec).

**Design.** A modal filterbank — 4 to 8 resonators standing for block bending,
torsional, bore-wall and pan-drumming modes, with frequencies and Qs derived from
dressed mass, material stiffness and bore spacing rather than typed in. Keep the
existing `block_resonance_hz(mass)` law as the anchor for the first mode so the
current calibration is not lost.

Three excitations into the bank:

- `dP/dtheta` per cylinder from the Stage 7 tables — the combustion path.
- The Stage 2 mechanical impulses — they radiate through the block, not the air.
- Stage 1's knock burst — knock is heard through the structure.

An alloy block and an iron block of the same size differ mostly in mass, and mass
is already a preset field; stiffness barely changes. That asymmetry is exactly why
an alloy V8 rings higher than an iron one, and it should come out of the formula.

**Tests.**

- Mode frequencies scale as `1/sqrt(mass)`; a heavier block rings lower (carry
  over `a_heavier_block_rumbles_lower`).
- A steeper `dP/dtheta` at constant peak pressure raises radiated level — the
  diesel-clatter mechanism.
- Knock and the mechanical rig appear in the structural output and not on the
  exhaust bus.
- Structural output is still bounded and DC-free.

**Commits.**

```
add modal structure module
derive mode frequencies from mass and stiffness
excite the structure from pressure rise rate
route the mechanical rig through the structure
route knock through the structure
add per-preset structural spec
retire the single block resonator
test modes scale with block mass
test steeper pressure rise radiates more
```

**Prompt.**

> Implement Stage 8 of `docs/IMPLEMENTATION_PLAN.md`. Read it, Stage 7 (whose
> pressure tables you differentiate) and `AGENTS.md`. One-line granular commits,
> no trailers.
>
> Replace the single `BlockResonator` biquad (`src/audio/filters.rs:1008`) with a
> modal filterbank in a new `src/audio/structure.rs`: 4–8 modes whose frequencies
> and Qs come from dressed mass, material stiffness and bore spacing, anchored so
> the first mode reproduces the existing `block_resonance_hz(mass)` law. Excite it
> from three sources — per-cylinder `dP/dtheta` off the Stage 7 tables, the
> Stage 2 mechanical impulses, and the Stage 1 knock burst — so the structure
> becomes a second radiating path rather than a filter on the exhaust bus.
>
> The mechanism to get right: a steeper pressure rise at the same peak pressure
> must radiate more. That is combustion noise, and it is what makes a diesel
> clatter. Keep the `a_heavier_block_rumbles_lower` and
> `block_rumble_is_strongest_at_idle_and_gone_at_speed` behaviours or explain in
> the commit why the physical model supersedes them.

---

# Stage 9 — Thermal state and warm-up

**Goal.** One state variable that makes a cold engine sound cold.

**Why.** `WoschniModel::wall_temperature` is a constant, so there is no cold
start. Warm-up moves several things at once, all of which the audio path already
reads: cooler walls give cooler exhaust, so a lower `c`, so **every pipe
resonance drops in pitch**; thicker oil raises FMEP, so the mechanical floor is
louder; the idle is fast; the mixture is rich. Sixty seconds of continuously
changing sound for one ODE.

**Files.** New `src/physics/thermal.rs`; `src/physics/thermodynamics.rs` (wall
temperature becomes state), `src/bench.rs` (thermal mass, oil viscosity law,
cold-idle target), `src/audio/waveguide.rs` (pipe wall temperature per section).

**Design.** Lumped thermal masses with the heat the solver already computes:

```
C_block  dT_block/dt = Q_wall(t) - h_coolant (T_block - T_coolant)
C_pipe_i dT_pipe/dt  = Q_gas_i(t)  - h_air     (T_pipe  - T_ambient)
```

`Q_wall` is Woschni's, already integrated per cycle. Friction rises as oil
viscosity falls with temperature — a Vogel or Walther law on the Chen–Flynn
coefficients. Each pipe section carries its own temperature, which Stage 10 then
uses for the gradient.

**Tests.**

- From cold, exhaust temperature rises monotonically to a plateau.
- Every derived resonance rises with it by `sqrt(T2/T1)`.
- FMEP falls monotonically as the block warms; the mechanical floor follows.
- Cold idle sits above warm idle and converges.
- A stopped hot engine cools toward ambient at the modelled time constant.

**Commits.**

```
add lumped thermal masses
make cylinder wall temperature a state
warm the pipes from gas heat
scale friction with oil viscosity
raise cold idle from thermal state
test resonances rise as the engine warms
```

**Prompt.**

> Implement Stage 9 of `docs/IMPLEMENTATION_PLAN.md`. Read it and `AGENTS.md`.
> Granular one-line commits, no trailers.
>
> `WoschniModel::wall_temperature` is a fixed constant, so this simulator has no
> cold engine. Add `src/physics/thermal.rs` with lumped thermal masses for the
> block and for each pipe section, integrated from the wall heat flow Woschni
> already computes, and make cylinder wall temperature a state variable rather
> than a parameter. Scale the Chen–Flynn friction coefficients with oil viscosity
> against block temperature (Vogel or Walther), and raise the idle target when
> cold.
>
> The audible payoff is that every pipe resonance rises by `sqrt(T2/T1)` as the
> engine warms — assert exactly that, plus monotonic warm-up, falling FMEP, cold
> idle above warm idle, and a hot stopped engine cooling at the modelled time
> constant. Give each pipe section its own temperature; Stage 10 needs the
> gradient.

---

# Stage 10 — Pipe numerics: mean flow, gradient, steepening

**Goal.** Four corrections that separate a pipe model from a delay line.

**Why.** Each is small, each is currently wrong, and together they are the
difference between a note that is loud and a note that is *hard*.

**Files.** `src/audio/waveguide.rs`, `src/audio/filters.rs`, `src/audio/dsp.rs`.

### 10a — Mean-flow convection

Waves run downstream at `c + u` and upstream at `c - u`, so a pipe's effective
tuning length is **asymmetric and load-dependent** — an effect the current model
cannot produce at all. Two lines: bias the forward and backward delays separately.
The prediction to test:

```
f_1 = c (1 - M^2) / (4 L)        M = u / c
```

### 10b — Temperature gradient along the pipe

One bulk `exhaust_temperature` sets every delay, but a real system runs ~900 C at
the port and ~400 C at the tailpipe. `c` varies about 35 % along the length, so
tuning computed from a bulk temperature is simply wrong. Use Stage 9's per-section
temperatures.

### 10c — Wave steepening

At blowdown the pressure ratio exceeds 2 and the crest travels faster than the
trough, so the front steepens toward a shock down the primary. Amplitude-dependent
delay, or a saturating shaper with level-dependent HF emphasis per segment. This
is the mechanism behind a race engine sounding hard rather than merely loud.

### 10d — Interpolation and aliasing

Linear interpolation inside a feedback loop is a lowpass whose cutoff moves with
the fractional part — audible as a breathing dullness while the delay glides with
temperature. Move to Thiran allpass or Lagrange-3. Separately, a 0.16 ms attack
into a soft clipper at 48 kHz folds; oversample the nonlinear stages 2x or
bandlimit the excitation.

Also: `CONTROL_BLOCK = 32` is 1.5 ms at 48 kHz, while a V12 at 8000 rpm fires
every 1.25 ms — every control-rate schedule is quantised coarser than the events
it tracks. Either shorten the block or move the affected schedules to per-sample.

**Tests.**

- Fundamental shifts as `(1 - M^2)` with mean flow, in both directions.
- A pipe with a hot end and a cold end resonates between the two bulk
  predictions, not at either.
- Steepening raises high-order content with amplitude at constant fundamental,
  and is absent at low amplitude.
- Glide with the new interpolator shows no measurable amplitude modulation.
- No aliasing products above the excitation's bandlimit after oversampling.

**Commits.**

```
bias pipe delays by mean flow
test fundamental shifts with mach number
give each pipe section its own gas temperature
add amplitude-dependent wave steepening
replace linear delay interpolation with thiran allpass
oversample the nonlinear stages
shorten the control block below the firing interval
test steepening raises high orders with amplitude
```

**Prompt.**

> Implement Stage 10 of `docs/IMPLEMENTATION_PLAN.md`. Read it, Stage 5 and
> Stage 9, plus `AGENTS.md`. Four independent corrections, each its own commit,
> one line, no trailers.
>
> 1. **Mean flow.** Bias forward and backward delays separately by `c+u` and
>    `c-u` so tuning becomes load-dependent and asymmetric. Test that the
>    fundamental moves as `c(1-M^2)/(4L)`.
> 2. **Gradient.** Stop deriving every delay from one bulk exhaust temperature;
>    use the per-section temperatures from Stage 9. A pipe hot at the port and
>    cool at the tailpipe must resonate between the two bulk predictions.
> 3. **Steepening.** Make propagation amplitude-dependent so a large pulse
>    steepens toward a shock down the primary — high-order content must rise with
>    amplitude at a fixed fundamental, and be absent when quiet.
> 4. **Interpolation and aliasing.** Replace linear fractional delay with Thiran
>    allpass or Lagrange-3 (linear interpolation in a feedback loop modulates
>    brightness as the delay glides), oversample the nonlinear stages 2x, and deal
>    with `CONTROL_BLOCK = 32` being slower than the firing interval of a V12 at
>    8000 rpm.
>
> Check CPU against the Stage 0 baseline after each one and report the cost.

---

# Stage 11 — Propagation: apertures, directivity, space

**Goal.** Stop mixing sources into a stereo bus and start placing real radiators
in real positions relative to a listener.

**Why.** Today the exhaust is panned by bank index and everything else is centred
(`src/audio/dsp.rs:1626`). Tailpipe, intake mouth and block are metres apart on a
real car; getting that *geometry* right does more for "this is a physical object"
than any single filter.

**Files.** New `src/audio/propagation.rs`; `src/audio/dsp.rs` (the mix becomes an
aperture list), `src/bench.rs` (aperture positions per preset).

**Design.**

- **Apertures.** Each radiator — tailpipe(s), intake mouth, block — has a
  position, an area and a facing. Each gets its own path delay (1 m ≈ 3 ms, an
  audible comb), its own `1/r`, and its own air absorption.
- **Directivity.** A mouth is a monopole to `ka = 1` and beams above it, so
  off-axis loses the top. This is why a car sounds different from behind than
  from alongside.
- **Ground reflection.** Outdoors this is the dominant coloration:

```
delta = sqrt((h_s + h_r)^2 + d^2) - sqrt((h_s - h_r)^2 + d^2)
```

- **Doppler**, if this becomes a vehicle: per aperture, because the tailpipe and
  the intake are ~3 m apart and genuinely arrive differently on a pass-by.
- **Cabin path**, if inside: at low frequency the structure-borne route through
  the mounts dominates and is *not* a filtered version of the exterior sound.
- **Environment**: geometry-derived early reflections — tunnel, garage, wall.

**Tests.**

- Aperture delays match `r / c` and the sum shows the expected comb.
- Moving the listener behind the car attenuates the tailpipe's high end and not
  its low end.
- Ground reflection notches at the frequency the path difference predicts.
- Doppler shift matches `f' = f c / (c - v_r)`.
- Total level falls as `1/r` with distance.

**Commits.**

```
add propagation module
place apertures with position area and facing
delay each aperture by its path length
add distance attenuation and air absorption
add mouth directivity
add ground reflection
add per-aperture doppler
add aperture positions to the presets
test ground notch matches the path difference
```

**Prompt.**

> Implement Stage 11 of `docs/IMPLEMENTATION_PLAN.md`. Read it and `AGENTS.md`.
> Granular one-line commits, no trailers.
>
> The mix in `src/audio/dsp.rs:1626` pans the exhaust by bank index and puts
> everything else in the centre. Replace it with a real propagation model in
> `src/audio/propagation.rs`: an aperture list (tailpipes, intake mouth, block)
> each with position, area and facing, each delayed by its own path length,
> attenuated by `1/r` and by air absorption, and filtered by mouth directivity
> (monopole to `ka=1`, beaming above). Add ground reflection using the path
> difference `sqrt((hs+hr)^2+d^2) - sqrt((hs-hr)^2+d^2)`, and per-aperture Doppler
> so a pass-by shifts the tailpipe and the intake mouth separately.
>
> Test what is predictable: aperture delays equal `r/c`, the ground notch lands
> where the path difference says, Doppler matches `f' = f c/(c - v_r)`, level falls
> as `1/r`, and a listener behind the car loses the tailpipe's high end but not its
> low end. Put aperture positions on `EnginePreset`.

---

# Stage 12 — Fuelling, timing and the driver's controls

**Goal.** Make the two remaining constants — AFR and spark angle — into the
control signals they are, and give the limiter and misfire real character.

**Why.** `air_fuel_ratio: STOICH_AFR` for every engine always, and
`WiebeProfile::spark_angle` is a preset constant. Both feed channels the audio
path already reads, so plumbing them costs little and unlocks a lot.

**Files.** New `src/physics/control.rs`; `src/physics/thermodynamics.rs`,
`src/bench.rs`, `src/audio/dsp.rs` (per-cylinder health, limiter modes).

**Design.**

- **AFR** as a function of load and speed: WOT enrichment near 12.5:1, lean
  cruise, accel enrichment, and **decel fuel cut-off** — which is why a real
  overrun goes quiet and then bangs on tip-in. AFR moves `gamma`, `R`, flame speed
  (Wiebe duration) and EGT, all of which the synth already consumes.
- **Spark map** in advance versus load and speed, with knock-driven retard closing
  the loop on Stage 1's knock integral. Heavy deliberate retard is what anti-lag
  and launch control *are*.
- **Limiter modes**: hard cut, soft progressive cut, rotating per-cylinder
  stutter, and fuel-cut versus spark-cut — fuel cut sends no unburnt charge to the
  exhaust, so it cannot bang, while spark cut can. Among the most identifiable
  sounds a car makes.
- **Per-cylinder health**: a dead plug or fouled injector as a hole in the firing
  pattern — the classic hunting idle. `CycleVariation` is already per-cylinder.

**Tests.**

- Enrichment lowers EGT and measurably lowers every pipe resonance.
- Decel fuel cut-off silences combustion while the mechanical floor and pumping
  continue; tip-in produces unburnt fuel and a bang.
- Knock retard reduces the knock integral below 1 within a bounded number of
  cycles.
- A fuel-cut limiter produces no backfires; a spark-cut limiter does.
- A single dead cylinder shows as a missing order component and a lope at the
  cycle rate.

**Commits.**

```
add fuelling and spark control module
schedule afr against load and speed
add decel fuel cut-off
add spark advance map
close knock retard on the knock integral
add limiter modes
add per-cylinder health
test enrichment lowers the pipe resonances
test fuel cut cannot backfire
test a dead cylinder lopes at the cycle rate
```

**Prompt.**

> Implement Stage 12 of `docs/IMPLEMENTATION_PLAN.md`. Read it and `AGENTS.md`;
> granular one-line commits, no trailers.
>
> Two constants are doing real damage: `air_fuel_ratio` is `STOICH_AFR` for every
> engine at every operating point, and `WiebeProfile::spark_angle` is fixed. Add
> `src/physics/control.rs` scheduling AFR against load and speed (WOT enrichment,
> lean cruise, accel enrichment, decel fuel cut-off) and spark advance against
> load and speed, with knock-driven retard closing the loop on the Livengood–Wu
> integral from Stage 1. Then add limiter modes — hard cut, soft cut, rotating
> per-cylinder stutter, and fuel-cut versus spark-cut — and per-cylinder health so
> a dead plug becomes a hole in the firing pattern.
>
> These all reach the ear through channels the synth already reads, so assert the
> chain: enrichment must lower EGT and therefore every pipe resonance; a fuel-cut
> limiter must produce no backfires while a spark-cut limiter does; decel fuel
> cut-off must silence combustion while the mechanical floor continues; a dead
> cylinder must show as a missing order and a lope at the cycle rate.

---

# Stage 13 — Induction hardware beyond the turbo

**Goal.** The other compressors, and the valves that make noise.

**Why.** `Induction` has exactly two variants, and `TurboVoice` is the only
compressor in the crate. Several very recognisable sounds are simply absent.

**Files.** `src/audio/mod.rs` (`Induction`), `src/audio/dsp.rs` (new voices),
`src/bench.rs` (fit them to presets).

**Design.**

- **Roots / twin-screw supercharger.** Its whine is a **crank** order — belt ratio
  times rotor lobe count — not a shaft order. No lag, rises perfectly with rpm: a
  completely different sound from a turbo, but the same code shape as `TurboVoice`.
- **Centrifugal supercharger.** Shaft-order like a turbo, but belt-locked, so it
  has a turbo's pitch behaviour with none of its lag.
- **Blow-off / dump valve.** A broadband whoosh on lift, distinct from the surge
  flutter that exists today, and triggered by the same closed-throttine-with-boost
  condition.
- **Wastegate chatter.** A rattling flutter at high boost as the gate hunts.
- **Exhaust cutout / active valve flap.** A step change in the silencer chain —
  trivial once Stage 5's network is geometry-driven, since it is a bypass junction
  opening.
- **Anti-lag**, once Stage 12's retard exists: fuel and spark into the exhaust,
  keeping the turbine lit. Loud, and it falls out of two existing mechanisms.

**Tests.**

- Supercharger whine tracks crank speed exactly and shows no spool lag.
- A turbo's whistle still lags; the two are distinguishable in an order analysis.
- The dump valve fires on lift with boost present and never without boost.
- Opening the cutout raises high-order content and lowers back pressure.

**Commits.**

```
add roots supercharger voice at crank order
add centrifugal supercharger voice
add blow-off valve
add wastegate chatter
add exhaust cutout as a bypass junction
add anti-lag from spark retard
fit induction hardware to the presets
test supercharger whine has no lag
```

**Prompt.**

> Implement Stage 13 of `docs/IMPLEMENTATION_PLAN.md`. Read it and `AGENTS.md`;
> one-line granular commits, no trailers.
>
> `Induction` in `src/audio/mod.rs` has only naturally-aspirated and turbocharged
> variants. Add the rest: a Roots/twin-screw supercharger whose whine is locked to
> a **crank** order (belt ratio times lobe count) with no spool lag — structurally
> like `TurboVoice` but driven from engine speed, not shaft speed — a centrifugal
> supercharger (shaft-order but belt-locked), a blow-off valve as a broadband
> whoosh on lift distinct from the existing surge flutter, wastegate chatter, and
> an exhaust cutout implemented as a bypass junction in the Stage 5 network. If
> Stage 12 is in, add anti-lag from spark retard plus exhaust fuelling.
>
> The test that separates them: order analysis must show the supercharger tracking
> crank speed with no lag while the turbo lags. Fit each to the presets that would
> plausibly carry it.

---

# Stage 14 — Diesel and other combustion topologies

**Goal.** An engine whose sound is dominated by the structural path rather than
the pipe.

**Why.** The catalogue is six petrol engines. A diesel is a genuinely different
acoustic object: no spark, compression ignition with a premixed spike giving a
very high `dP/dtheta`, and the characteristic clatter comes from that pressure
rise exciting the structure — not from the exhaust. It is the best possible test
that Stage 8's structural path is real.

**Files.** `src/physics/thermodynamics.rs` (two-stage heat release),
`src/bench.rs` (presets), `src/audio/structure.rs` (already the mechanism).

**Design.**

- **Two-stage Wiebe**: a short premixed burn superimposed on a long diffusion
  burn, with an ignition-delay period computed rather than assumed — the knock
  model's Arrhenius integral is the same physics.
- Injector noise dominant (Stage 2), high compression ratio, turbocharged
  (Stage 13), low redline, huge mass.
- Worth adding alongside: **two-stroke** (port timing, no valve events, expansion
  chamber that genuinely works acoustically) and a **big single or twin** where
  the crank ripple of Stage 1c becomes the dominant character.

**Tests.**

- Two-stage heat release produces a pressure trace with the premixed spike ahead
  of the diffusion hump.
- Ignition delay lengthens with lower compression temperature.
- Radiated sound is dominated by the structural path, unlike a petrol preset
  measured the same way.

**Commits.**

```
add two-stage diesel heat release
compute ignition delay from arrhenius integral
add diesel preset
add two-stroke port timing
add big single preset
test diesel radiates mostly through the structure
```

**Prompt.**

> Implement Stage 14 of `docs/IMPLEMENTATION_PLAN.md`. Read it, Stage 8 and
> `AGENTS.md`. Granular one-line commits, no trailers.
>
> Add compression ignition: a two-stage Wiebe (short premixed spike plus long
> diffusion burn) with an ignition delay computed from an Arrhenius integral rather
> than assumed — the same physics `KnockModel` already uses. Then add a diesel
> preset with high compression, a turbo, a low redline, a heavy block and dominant
> injector noise, and if the effort is small, a two-stroke with port timing and a
> real expansion chamber plus a big single where Stage 1's crank ripple dominates.
>
> The test that validates Stage 8 as much as this stage: measure the radiated
> output with the exhaust network muted and with the structural path muted, and
> show the diesel is structure-dominated where a petrol preset is pipe-dominated.

---

# Stage 15 — Physics-grade pipe solver

**Goal.** Replace the linear-acoustic wave splitter in the physics with a solver
valid at the amplitudes an exhaust actually reaches.

**Why.** `AcousticPipe::integrate` injects `p' = rho c u` and propagates with a
scalar damping factor. At blowdown the pressure ratio exceeds 2 and Mach reaches
0.3–0.6, where that is the wrong equation. This is the reference-grade answer, and
it is deliberately last: Stages 5 and 10 already capture most of what is audible,
so this is for correctness of the *boundary condition* the solver feeds back into
the cylinder.

**Files.** `src/physics/engine_block.rs` (`AcousticPipe`), possibly a new
`src/physics/gasdyn.rs`.

**Design.** Method of Characteristics on Riemann variables, or a TVD
finite-volume Euler solver — what GT-Power and Ricardo WAVE do. What it buys:
shock formation, mean-flow convection as a consequence rather than a correction,
temperature stratification along the pipe, and correct partial reflection at
junctions under flow. Keep unit-CFL stepping and the existing accumulator idiom so
frame-rate independence survives.

Keep the audio-rate waveguide separate. The two have different jobs: the physics
pipe is a boundary condition at control rate in `f64`; the audio network is
radiation at sample rate in `f32`. They must read **one** geometry (Stage 3) and
should be asserted against each other, not merged.

**Tests.**

- A Sod shock-tube problem matches the analytic solution.
- Linear-amplitude behaviour reproduces the current model within tolerance —
  the new solver must not lose the old correct case.
- A finite-amplitude pulse steepens into a shock over the predicted distance.
- Mass, momentum and energy are conserved to solver tolerance over 10^4 steps.
- The audio network and the physics solver agree on a pipe's fundamental within
  a percent at low amplitude.

**Commits.**

```
add riemann variable gas dynamics
solve the pipe by method of characteristics
test sod shock tube against theory
reproduce linear acoustics at low amplitude
convect junction reflection with mean flow
test the audio and physics pipes agree
```

**Prompt.**

> Implement Stage 15 of `docs/IMPLEMENTATION_PLAN.md`. Read it and `AGENTS.md`.
> Granular one-line commits, no trailers.
>
> `AcousticPipe::integrate` (`src/physics/engine_block.rs:683`) is linear
> acoustics — it injects `p' = rho c u` and propagates with a scalar damping
> factor. Exhaust blowdown runs at pressure ratios above 2 and Mach 0.3–0.6, where
> that model does not hold. Replace it with Method of Characteristics on Riemann
> variables (or a TVD finite-volume Euler solver), keeping the unit-CFL stepping
> and the accumulator so frame-rate independence survives.
>
> Two hard requirements. First, it must reproduce the present model at low
> amplitude — do not lose the case that was already right. Second, verify against
> a Sod shock tube with the analytic solution. Also assert conservation over 10^4
> steps and that a finite-amplitude pulse steepens over the predicted distance.
>
> Do **not** merge this with the audio-rate waveguide. They have different jobs:
> `f64` control-rate boundary condition versus `f32` sample-rate radiation. They
> share the Stage 3 geometry and should be cross-checked against each other on a
> low-amplitude pipe fundamental.

---

# Stage 16 — Calibration against recordings

**Goal.** Turn "physically motivated" into "measured against reality".

**Why.** `engine.wav` exists in this repo only as an *output* target. The
strongest available realism tool is to use real recordings as *input*: order-track
a reference sweep, and compare.

**Files.** `src/analysis/orders.rs` (from Stage 0), new
`examples/calibrate.rs`, `docs/measurements/`.

**Design.**

- Order-track a reference recording with a known rpm curve: extract each engine
  order's amplitude in dB versus rpm, plus the long-term average spectrum to find
  where the pipe resonances actually sit.
- Compare against the synth on the same script. The comparison metrics that
  matter: **order balance** (is order 4 dominant where it should be, is the
  half-order content right), **resonance placement**, and **noise-floor tilt**.
- Where they disagree, the fix is a *geometry* correction — a primary length, a
  chamber volume, a mouth radius — never a new gain constant. That constraint is
  what keeps the model physical.
- Record every calibration in `docs/measurements/` with the recording it came
  from, so a later regression is attributable.

**Tests.**

- Synth order spectrum matches a reference within a stated dB tolerance across
  the sweep.
- Resonance placement matches within a stated percentage.
- A CI-friendly fixed sweep per preset regresses timbre, not just crashes.

**Commits.**

```
add reference order extraction
add calibrate example
record reference measurements
calibrate primary lengths against the reference
calibrate chamber volumes against the reference
add timbre regression to the test suite
```

**Prompt.**

> Implement Stage 16 of `docs/IMPLEMENTATION_PLAN.md`. Read it, Stage 0 and
> `AGENTS.md`. Granular one-line commits, no trailers.
>
> Build the calibration loop. Extend `src/analysis/orders.rs` to order-track a
> *reference recording* with a supplied rpm curve, add `examples/calibrate.rs` that
> renders the matching synth script and prints a per-order dB comparison plus
> resonance placement, and record the results under `docs/measurements/`.
>
> One rule, and it is the point of the stage: when the synth disagrees with the
> reference, fix a **geometric** parameter — primary length, chamber volume, mouth
> radius — never add a gain or EQ constant. If a disagreement cannot be explained
> geometrically, write that down in the measurement notes as an open question
> instead of papering over it.
>
> Finish by adding a fixed-sweep timbre regression to the test suite so future
> changes are caught as spectral drift, not just as crashes.

---

# Appendix A — Formula reference

Everything the stages assert against, in one place.

```
speed of sound            c = sqrt(gamma R T)
pipe fundamental          f1 = c / (4 L)                      (closed-open)
with end correction       f1 = c / (4 (L + delta))
  unflanged               delta = 0.6133 a
  flanged                 delta = 0.8216 a
with mean flow            f1 = c (1 - M^2) / (4 L)            M = u/c
temperature scaling       f2/f1 = sqrt(T2/T1)

junction (N pipes)        Y_i = A_i / (rho c)
                          p_J = 2 sum(Y_i p_i+) / sum(Y_i)
                          p_i- = p_J - p_i+
two-pipe reflection       r = (A1 - A2) / (A1 + A2)

expansion chamber TL      TL = 10 log10[1 + (1/4)(m - 1/m)^2 sin^2(kL)]
                          m = A2 / A1
quarter-wave stub notch   f = c / (4 L_stub)
helmholtz resonance       f = (c / 2pi) sqrt(A_neck / (V L_neck))

viscothermal loss         alpha = (1/(a c)) sqrt(pi f nu) (1 + (gamma-1)/sqrt(Pr))
                          alpha ~ sqrt(f) / a
mouth radiation corner    f_c = c / (2 pi a)                  (ka = 1)
monopole radiation        p ~ dQ/dt                           (+6 dB/octave)

knock cavity modes        f = rho_mn c / (pi B)
                          rho_10 = 1.8412  (first circumferential)
                          rho_20 = 3.0542
                          rho_01 = 3.8317  (first radial)

firing frequency          f = (rpm / 120) N_cyl               (four-stroke)
engine order of firing    n = N_cyl / 2                       (four-stroke)
firing interval per bank  tau = (120 / rpm) / N_cyl_in_bank
crank dynamics            dw/dt = (T_ind - T_load - T_fric) / I
torsional resonance       f = (1/2pi) sqrt(k (1/I1 + 1/I2))

orifice noise (dipole)    power ~ u^6, amplitude ~ u^3
water hammer at IVC       p' ~ d(mdot)/dt                     (column inertia)
induction rarefaction     p' = -c mdot / A

doppler                   f' = f c / (c - v_r)
ground path difference    delta = sqrt((hs+hr)^2 + d^2) - sqrt((hs-hr)^2 + d^2)
distance                  p ~ 1/r
```

# Appendix B — Budgets and invariants

**CPU.** Record the real-time multiple per preset at Stage 0 and check it after
every stage that adds DSP. Target: the whole synth under 5 % of one core at
48 kHz for the largest preset. Stage 5 is the one that will cost — ~17 delay lines
for a V8 — and Stage 10d's oversampling roughly doubles the nonlinear stages.

**Real-time safety.** No allocation, no lock, no `panic!`, no unbounded loop in
the audio callback, ever. Every buffer allocated at construction; every new
`Vec` in a voice is a bug. `EngineSnapshot` stays `Copy` with no indirection
however many tables Stage 7 adds.

**Numerical safety.** Keep the NaN discipline: `EngineSnapshot::sanitized` guards
every field, filter loops stay contractions at all frequencies, and any new state
gets the same treatment. A NaN in a delay line is permanent and renders as
full-scale noise.

**No tone knobs.** Every stage removes more hand-tuned acoustic constants than it
adds. If a stage ends with a new coefficient that has no physical name, that is a
finding to write down, not a value to tune.

# Appendix C — Running things

```bash
cargo run --release                                    # interactive dashboard
cargo run --release --example v8_bench                 # pressure and torque profile
cargo run --release --example engine_audio             # live audio
cargo run --release --example engine_audio -- --offline out.wav   # deterministic render
cargo test --release                                   # 183 tests at plan start
cargo run --release --example measure                  # Stage 0 onward: order tables and CPU
cargo run --release --example calibrate                # Stage 16 onward: compare to a reference
```
