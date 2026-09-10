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
