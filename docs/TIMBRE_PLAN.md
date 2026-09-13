# Timbre plan

A second plan, narrower than [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md).
That one built the machine; this one fixes what the machine sounds like.

The complaint it exists to close is "it sounds like a can rumbling", and the
measurement behind it, taken 2026-09-13 on the 12 s drive cycle, octave-band
share of total energy:

| band | pre-Stage-0 render | current | delta |
|---|---|---|---|
| 63 Hz | -9.9 dB | -2.6 dB | **+7.3** |
| 2 kHz | -10.2 dB | -18.4 dB | **-8.2** |
| 4 kHz | -14.6 dB | -23.0 dB | **-8.4** |

Crest factor over the same render fell 20 dB to 14 dB.

Seven decibels of energy share moved *into* the bottom octave and eight came
*out* of the 500 Hz - 4 kHz region where an engine's bark lives, while the
transients flattened. That is the whole of it. It is not a missing voice and not
a missing physical effect; it is a spectral balance that drifted across fourteen
stages with nothing watching.

The reason nothing watched is structural, and it is in the harness rather than
the synth. [`Fingerprint`](../src/analysis/timbre.rs) records exactly three
things — order balance, resonance peak positions, and noise-floor tilt over
200 Hz - 12 kHz. None of them can fail on this. A tilt figure in particular is
blind to it: lose all of 2 kHz and a proportionate share of 4 kHz and the slope
is unchanged. So every stage optimised order placement, every stage passed, and
the midrange drained out underneath.

**Nothing in this plan is provable until Stage T0 lands.** Do it first.

---

## Working method

These stages are cheap to get wrong expensively. The failure mode is reading
code to form an opinion about sound, and this repo is large enough that doing so
burns a session before a single measurement exists. The recipe below is not
style advice; following it is part of the definition of done.

### Diagnose by rendering, never by reading

The doc comments in `src/audio` are more confident than the code, and several
constants there were calibrated against a filter that was broken at the time and
kept their values after it was fixed. A comment explaining why a number is right
is not evidence the number is right.

To find where a tonal property went:

```bash
python3 -m pip install numpy        # not installed by default
cargo run --release --example measure -- --preset "Flat-plane V8" --out /tmp/m
```

then compare on three numbers — RMS, **crest factor**, and the octave-band
table. Crest factor is the one that catches "hollow"; RMS barely moves when the
character does.

To find *when* it went: `git worktree add` at candidate commits, copy in
`examples/diagnose_sound.rs`, render, compare. Bisect the numbers. Do not bisect
by reading diffs.

### Read narrowly

Whole-file reads are the largest avoidable cost in this repo:

| file | lines |
|---|---|
| `src/audio/dsp.rs` | 6331 |
| `src/audio/waveguide.rs` | 4004 |
| `src/physics/thermodynamics.rs` | 2464 |
| `src/physics/engine_block.rs` | 2427 |
| `src/analysis/orders.rs` | 2051 |

None of them should ever be read end to end. The pattern that works:

1. `grep -n 'pub fn\|pub struct\|pub enum' <file>` for the map.
2. `grep -n '<symbol>' <file>` for the site.
3. `sed -n '<start>,<end>p' <file>` for the span, forty lines at a time.

Read the stage in this document and the spans it names. Nothing else.

### One stage per session

Each stage below is sized to be finished — code, tests, commits, measurement —
without a context summary in the middle. Start a fresh session per stage and
give it the stage's Prompt verbatim. Two stages in one session means the second
one is planned against a compressed memory of the first.

### Do not re-derive what is already recorded

Before investigating whether something exists, check this list. As of
2026-09-13 the following are **implemented**, and rediscovering them is pure
cost:

- Lock-free `rtrb` SPSC audio bridge; no mutex, no `RwLock`, no wall-clock
  physics at audio rate. Physics runs at 480 Hz and the crank integrates in the
  audio thread. See the contract at the top of `src/audio/stream.rs`.
- Lagrange-3 fractional delay interpolation (`read_lagrange3`).
- Amplitude-dependent wave steepening (`WaveguidePipe::set_steepening`).
- Mean-flow convection, forward and backward delays biased separately
  (`set_mach`).
- Per-section gas temperature along the exhaust.
- Karal-Flugge end corrections, `0.6133 a` unflanged and `0.8216 a` flanged.
- Cycle-to-cycle combustion variation, per-cylinder amplitude and phase,
  rerolled per cycle and scheduled against rpm.
- Valve-overlap reversion in the intake plumbing.
- Turbo whistle, compressor surge, blow-off, wastegate flutter, backfire, knock,
  gear whine, and the mechanical impulsive set.
- 2x oversampled soft clipping on the output bus.

### Measure one preset, not the catalogue

A full catalogue render is eleven presets times six profiles. For a diagnostic
loop use `--preset "Flat-plane V8"` alone; it has the most midrange to lose.
Record the full catalogue only at the end of a stage, for `docs/measurements/`.

### Say what you did not do

Unchanged from [AGENTS.md](../AGENTS.md), and it matters more here because these
stages are judged on a number rather than on whether something compiles. If a
stage's metric did not move, say the metric did not move.

---

## Definition of done, every stage

The six rules in [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md#definition-of-done-every-stage)
apply unchanged, plus one:

7. The stage's effect on the octave-band share and crest factor is recorded in
   `docs/measurements/`, against the pre-Stage-0 reference, whether or not it
   moved in the intended direction.

---

## Ordering

```
T0 broadband metrics ──── gate for everything below
   │
   ├── T1 midrange audit ──── the actual fix; needs T0 to be provable
   │
   ├── T2 reciprocating inertia ─── independent of T1, different mechanism
   │      └── T3 valve float ────── cheap, reuses T2's kinematics
   │
   └── T4 exhaust tuning surface ── independent; makes the pipe model usable
```

T0 then T1 is the critical path. T2 through T4 add character that is currently
absent; they do not fix the balance and must not be mistaken for it.

---

# Stage T0 — Broadband metrics in the fingerprint

**Goal.** Make the spectral balance regression visible to a test.

**Why.** It is the one thing standing between this plan and every claim in it
being an opinion. Fourteen stages passed their tests while the midrange
disappeared, because the three quantities `Fingerprint` records cannot express
"the energy moved octaves". Until a `cargo test` can fail on this, nothing below
can be proven to have helped.

**Files.** `src/analysis/timbre.rs`, `src/analysis/orders.rs`,
`examples/calibrate.rs`.

**Design.**

- `orders::octave_bands(samples) -> [f64; 10]` — energy share per octave band,
  centres 31.5 Hz through 16 kHz, each in dB relative to total energy. Share,
  not absolute level, so it is invariant to output gain exactly the way the
  existing order balance is invariant to it. That invariance is the point: a
  stage that changes overall loudness must not shift these numbers.
- `orders::crest_db(samples) -> f64` — `20 log10(peak / rms)` over the whole
  render. One number, and the one that tracks "hollow" when RMS does not.
- Extend `Fingerprint` with `octave_share: &'static [f64; 10]` and
  `crest_db: f64`, and `Measured` with the owned equivalents.
- Extend `Fingerprint::drift` to report a band whose share moved more than
  `BAND_TOLERANCE_DB` (start at 2.0) and a crest that moved more than
  `CREST_TOLERANCE_DB` (start at 1.5). The existing drift reporting style —
  one human sentence per drifted quantity — carries over unchanged.
- Record the pre-Stage-0 reference as a named constant, not as the current
  measurement. The recorded fingerprints are what the synth *should* produce,
  and right now it does not; committing today's numbers as the target would
  freeze the bug into the test suite.

**Tests.**

- Pink noise reads as equal share in every band within 1 dB.
- A pure sine at 1 kHz puts more than 90 % of its energy in the 1 kHz band and
  the two neighbours below `SILENCE_DB`.
- A square wave and a sine of equal RMS differ in `crest_db` by the analytic
  3.0 dB, within 0.1.
- `octave_share` is unchanged when the input is scaled by 6 dB; `crest_db` too.
- `drift` names the band when a fingerprint's 2 kHz share is moved by 3 dB, and
  stays silent when it is moved by 1 dB.

**Commits.**

```
add octave band energy share
add crest factor measurement
record octave share and crest in the fingerprint
report band and crest drift
record the pre-stage-0 reference fingerprint
```

**Prompt.**

> Implement Stage T0 of `docs/TIMBRE_PLAN.md`. Read that stage, the Working
> method section above it, and `AGENTS.md` for the commit rule — one change per
> commit, one line per message, lower case, no trailers ever.
>
> Add `octave_bands` and `crest_db` to `src/analysis/orders.rs`, then widen
> `Fingerprint` and `Measured` in `src/analysis/timbre.rs` to carry them and
> `drift` to report on them. Follow the existing style in that file exactly: a
> tolerance constant per quantity, and one plain sentence per drifted quantity.
>
> The recorded fingerprint values are the **pre-Stage-0 targets** in the table at
> the top of this plan, not today's measurements. The suite is expected to fail
> against the current synth; that failure is the deliverable. Say so plainly in
> your final message and do not adjust the tolerances to make it pass.
>
> Do not touch `src/audio` or `src/physics` in this stage.

---

# Stage T1 — Find and return the midrange

**Goal.** Move the 2 kHz and 4 kHz octave shares back within tolerance of the
pre-Stage-0 reference without giving up the order structure the plan built.

**Why.** This is the complaint. Everything else in this plan is character; this
is correctness.

**Files.** Determined by the bisect, not in advance. Expect `src/audio/filters.rs`,
`src/audio/waveguide.rs`, `src/audio/dsp.rs`.

**Design.** This stage is an investigation with a commit at the end of it, so it
is specified as a procedure rather than a diff.

Bisect first, on the numbers from T0. Then check the four suspects, in this
order, because this is descending order of how much energy each can remove from
1 - 4 kHz:

1. **Viscothermal wall loss.** `ViscothermalLoss` with `set_wall_enhancement` is
   a frequency-dependent damper *inside a feedback loop*, so its error compounds
   per round trip. An enhancement factor a factor of two high is inaudible on a
   single pass and removes the bark after six. Check the constant's history —
   `git log -p` on the file — for a value set while the filter it was tuned
   against was broken.
2. **Radiation corner placement.** `corner_hz = c / (2 pi a)`. A 65 mm tailpipe
   at 650 K gives `a = 0.0325`, `c ~ 510 m/s`, so `f_c ~ 2.5 kHz`: the mouth is a
   mirror below it and a window above. Everything this plan is missing lives
   right at that corner, so an over-steep mouth response or a mis-normalised
   `radiation_gain` discards the bark at the last stage before the mix.
3. **Output clipper drive.** 2x oversampled soft clipping fed an already
   bass-heavy signal compresses exactly the peaks that carry midrange transient
   energy. A direct crest-factor cost, and the cheapest of the four to test:
   render with the clipper bypassed and read the band table.
4. **Structural path level.** `StructuralPath` is a modal bank on bending, pan
   and bore-wall modes, all of them low. If its level rose relative to the
   exhaust mouth the result is precisely this spectrum — boom up, bark down.

**Tests.**

- The catalogue fingerprints from T0 pass, which is the stage.
- Whichever constant moved gets an analytic test pinning it to its derivation
  rather than to its value, so it cannot drift back silently.
- Order balance for every catalogue preset stays within the existing tolerance:
  the midrange must come back *without* redistributing the orders.

**Commits.** Not predictable. One per constant or filter corrected, one line
each, and a final one recording the measurement.

**Prompt.**

> Implement Stage T1 of `docs/TIMBRE_PLAN.md`. Read that stage and the Working
> method section. Stage T0 must already be committed; if the octave-share
> fingerprint is not in `src/analysis/timbre.rs`, stop and say so.
>
> Diagnose by rendering and measuring, not by reading code. Bisect the commit
> range with worktrees and `examples/diagnose_sound.rs` to find where the 2 kHz
> share fell, then work the four suspects in the order the stage lists them.
> Treat doc comments in `src/audio` as claims to check, not as facts — several
> constants there were calibrated against a filter that was broken at the time.
>
> Fix the cause, not the symptom. Do not add a corrective EQ, a shelf, or a
> makeup gain anywhere in the chain; if the only way to pass is a broadband tilt
> then the cause has not been found and you should say that instead of shipping
> the tilt.
>
> Report the before and after octave-band table in your final message.

---

# Stage T2 — Reciprocating inertia

**Goal.** Give the block the second-order shake it currently cannot feel.

**Why.** `indicated_torque` is `(P_cyl - P_crankcase) * dV/dtheta` and nothing
else, and `StructuralPath` is driven by `combustion_drive(pressure_rate, bore)`
and nothing else. There is no reciprocating mass anywhere in the crate. So the
engine has no inertia torque ripple and no shaking force, which means an
inline-four has no secondary buzz, and crossplane and flatplane V8s differ only
in firing order and never in how they shake the block they are bolted to. That
difference is a large part of what makes those two engines sound like different
objects rather than the same object played in a different rhythm.

Two distinct effects, two commits, and they must not be conflated:

- **Inertia torque** acts on the crank and ripples the speed.
- **Inertia force resultant** acts on the block and shakes it. This is the
  audible one, and it is the one that cancels or does not cancel depending on
  crank layout.

**Files.** `src/physics/cylinder.rs`, `src/physics/engine_block.rs`,
`src/audio/dsp.rs`, `src/bench/config.rs`.

**Design.**

- Add `reciprocating_mass: f64` to `CylinderGeometry` — piston, rings, pin, and
  about a third of the rod. Default it from bore rather than requiring it in
  every preset: `m ~ 620 * B^3` kg puts a 94 mm bore near 0.51 kg, which is the
  right neighbourhood for an aluminium slug. Expose it in `[cylinder]` as an
  optional TOML field.
- The exact slider-crank is already there, so do not use the two-term
  approximation. Differentiate `piston_position` twice with respect to theta
  analytically and add `dposition_dtheta` and `d2position_dtheta2` next to the
  existing `dvolume_dtheta`.
- Piston acceleration at near-constant speed is `x_ddot = omega^2 * x''(theta)`,
  and taking it as exactly that is correct to the degree the crank speed is
  steady over one revolution. The alpha term is second order in an already small
  quantity; leave it out and say so in the doc comment.
- Inertia torque on the crank: `tau_i = -m * omega^2 * x''(theta) * x'(theta)`.
  It does no net work over a cycle, which is the test.
- Shaking force along the cylinder axis: `F_i = -m * omega^2 * x''(theta)`.
  Resolve each cylinder's force onto the block axes using its bank angle and sum
  over cylinders at their own phases, the same way `fill_excitations` already
  sums `pressure_slope_at`. The resultant is a vector; the structural drive
  wants its magnitude along the modes the bank already has.
- Sum the resultant into `StructuralPath`'s drive alongside `combustion_drive`,
  scaled so that at idle it is well below the combustion term and at redline it
  is comparable. It grows as `omega^2` on its own, so the scaling is a single
  constant, not a schedule.

**Tests.**

- `x'` and `x''` match a central difference of `piston_position` to 1e-6 across
  a full revolution.
- Inertia torque integrates to zero over 720 degrees, to within 1e-9 of the
  peak.
- The second-order component of inertia torque grows as `omega^2`.
- An inline-four's shaking-force resultant has a non-zero second order; a
  crossplane V8's second order cancels to below 1 % of an inline-four's at the
  same speed and mass. This is the test that proves the layout is being read.
- A flat-plane and a crossplane V8 differ in resultant, so the two presets are
  distinguishable by this path alone.
- Structural drive is unchanged from today when `reciprocating_mass` is zero,
  which keeps every existing fingerprint valid.

**Commits.**

```
add piston position derivatives
add reciprocating mass to cylinder geometry
add inertia torque to the crank
sum the reciprocating shaking force over the block
drive the structural path from the shaking force
expose reciprocating mass in the engine config
test inertia torque does no net work
test inline four secondary does not cancel
```

**Prompt.**

> Implement Stage T2 of `docs/TIMBRE_PLAN.md`. Read that stage, the Working
> method section, and `AGENTS.md`.
>
> Two separate mechanisms, and keep them separate: inertia *torque* ripples the
> crank, inertia *force* shakes the block. The audible one is the force
> resultant, and it is what distinguishes a crossplane V8 from a flat-plane one
> structurally rather than only in firing order.
>
> Use the exact slider-crank already in `src/physics/cylinder.rs`. Differentiate
> it analytically; do not introduce the `cos theta + (r/l) cos 2theta`
> approximation when the closed form is sitting in the file.
>
> Default `reciprocating_mass` from bore so no existing preset has to change,
> and make the whole path a no-op at zero mass so the recorded fingerprints stay
> valid. Then re-record them with the mass on, and report what the octave-band
> share did — this stage adds midrange, so T0's metrics should move, and if they
> do not, say so.

---

# Stage T3 — Valve float

**Goal.** The one mechanical voice the plan never reached.

**Why.** Above the speed where spring force can no longer hold the lifter to the
cam, the valve leaves the profile, flies, and re-seats at a velocity the cam
never intended. It is loud, it is only present near the limiter, and it is a
large part of why an engine at redline sounds like it is being hurt. Cheap now
that T2 has the kinematics.

**Files.** `src/physics/cylinder.rs`, `src/audio/dsp.rs`, `src/bench/config.rs`.

**Design.**

- Float speed follows from the cam: the valve separates when the required
  acceleration exceeds what the spring can supply, so
  `omega_float^2 ~ k_spring * preload / (m_valve * lift * shape)`. Rather than
  model the spring, parameterise it as `float_rpm` per preset with a default
  derived from redline — floating at about 1.06x redline is where a road engine
  sits, a race valvetrain higher.
- Above `float_rpm`, blend in a re-seating impulse on the exhaust and intake
  valve events: the existing `ImpulsiveConfig` machinery already schedules
  per-cylinder clicks, so this is an amplitude and a brightness schedule on
  those, not a new voice type.
- Impact velocity grows with `(rpm - float_rpm)`, so amplitude and spectral
  centroid both rise past the threshold. Below it, exactly zero — the effect
  must not leak into the midrange of an engine that is not floating, or it will
  read as a T1 regression.

**Tests.**

- Below `float_rpm` the float contribution is bit-zero.
- Impact level rises monotonically above the threshold.
- The catalogue fingerprints, which are measured below float speed, are
  unchanged.
- A limiter bounce render shows the float band appearing and disappearing with
  each cut.

**Commits.**

```
add valve float threshold speed
scale valve impact with float overspeed
expose float speed in the engine config
test float is silent below threshold
```

**Prompt.**

> Implement Stage T3 of `docs/TIMBRE_PLAN.md`. Read that stage and the Working
> method section. Stage T2 should be committed first.
>
> This is a schedule on the existing impulsive valve voices, not a new voice.
> Reuse the `ImpulsiveConfig` path in `src/audio/dsp.rs` rather than adding a
> parallel one.
>
> Below the float speed the contribution must be exactly zero, not small — every
> recorded fingerprint is measured below float speed and must be untouched. Prove
> that with a test before you tune anything.

---

# Stage T4 — Exhaust tuning surface

**Goal.** Make the pipe model's tuning readable as a number, so exhaust geometry
can be designed rather than guessed.

**Why.** `ExhaustNetwork` is the most complete part of the crate — a real
bidirectional scattering network with valve terminations, tapered collectors,
crossovers, four silencer types, and a radiating mouth, all of it configurable
per section in TOML. None of that is usable as a design tool, because there is
no way to ask "what does this pipe tune to" short of rendering and squinting at
a spectrum. The relationships are all in the code and none of them are exposed:

- Primary length sets the quarter-wave peak, `f_1 = c (1 - M^2) / (4 L)`, and
  because `c` depends on the section temperature a 880 K primary and a 650 K
  tailpipe are tuned 16 % apart for the same length.
- Primary diameter sets `Z_0 = rho c / A` and therefore the collector junction's
  reflection strength.
- Collector taper length is the reflection ramp: short is peaky, long is
  broadband.
- Tailpipe radius sets the radiation corner `c / (2 pi a)`, which is the system's
  brightness control and the most under-used parameter in the catalogue.
- Crossover position is a delay in wavelengths, and is what separates a
  flat-plane rasp from a crossplane burble.

**Files.** `examples/acoustic_bench.rs`, `src/bench/config.rs`,
`docs/ENGINE_CONFIGURATION_GUIDE.md`.

**Design.**

- Extend `examples/acoustic_bench.rs` with a `--sweep <parameter>` mode that
  rebuilds one preset's exhaust across a range of one geometric parameter and
  prints the resulting resonance table — primary length, primary diameter,
  collector taper, tailpipe length, tailpipe diameter, crossover position.
- Print the analytic prediction beside the measured peak for each point. Where
  they disagree by more than a few percent, the model is telling you something:
  that is the mean-flow term, the end correction, or a junction the formula does
  not know about, and it is worth reading rather than hiding.
- Add a `--mode` flag exercising the three exhaust modes on one preset, so
  muffled, straight pipe and open headers are comparable in one table.
- Document the sweep output in the configuration guide, under the existing
  exhaust section, with a worked example: take the flat-plane V8's 0.42 m
  primary at 880 K, predict 354 Hz, and show the measured peak.

**Tests.**

- A single pipe with no silencers and no collector resonates at
  `c (1 - M^2) / (4 L)` within 2 %, at three lengths.
- Doubling primary length halves the tuning peak.
- The unflanged tailpipe's effective length exceeds its physical length by
  `0.6133 a`, read back off the measured peak.
- The three exhaust modes produce monotonically increasing high-order content on
  the same preset.

**Commits.**

```
add geometry sweep to the acoustic bench
print analytic tuning beside measured peaks
add exhaust mode comparison to the bench
document exhaust tuning sweeps
test single pipe resonates at the quarter wave prediction
```

**Prompt.**

> Implement Stage T4 of `docs/TIMBRE_PLAN.md`. Read that stage and the Working
> method section.
>
> This stage adds no physics. `ExhaustNetwork` is complete; the work is exposing
> what it already computes as a table someone can design against. Extend
> `examples/acoustic_bench.rs` rather than writing a new example.
>
> Print the analytic prediction next to every measured peak. Where they diverge,
> say so in the output instead of smoothing it over — the divergence is the
> mean-flow bias, the end correction, or a junction, and a bench that hides it is
> worth less than one that flags it.
>
> Finish by documenting the sweep in `docs/ENGINE_CONFIGURATION_GUIDE.md` under
> the exhaust section, with the flat-plane V8 worked through as an example.

---

## Not in this plan

Named so they are not rediscovered as ideas:

- **Process-wide FTZ/DAZ.** The per-filter `DENORMAL_FLOOR` covers the recursions
  that matter, and the target is Apple silicon, where NEON flushes subnormals in
  the default mode. Revisit only if profiling shows it.
- **Burgers-equation shock propagation.** `set_steepening` already produces the
  audible effect. A full non-linear solver is a large amount of work for a
  refinement of something that is not currently wrong.
- **Thiran allpass interpolation.** Lagrange-3 is in place and the breathing
  artefact it was meant to fix is gone. No measured reason to change it.
- **Overrun burble as a new voice.** DFCO, anti-lag and the backfire voice
  already cover it. If it is inadequate, that is a tuning stage against T0's
  metrics, not a new mechanism.
