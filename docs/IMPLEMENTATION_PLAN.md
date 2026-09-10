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
