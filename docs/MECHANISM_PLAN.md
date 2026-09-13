# Mechanism plan

A third plan. [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) built the
machine, [TIMBRE_PLAN.md](TIMBRE_PLAN.md) fixes what it sounds like, and this one
is about what it cannot currently say.

Every stage here has the same shape. A real engine has a mechanism; the solver
does not; so two engines that a listener can tell apart come out of this
simulator sounding the same. That is a different failure from the timbre one and
it is not fixed by balancing a spectrum — a turbo engine with a perfect
spectrum is still not a turbo engine if there is no turbine in the pipe.

**Read [TIMBRE_PLAN.md's Working method](TIMBRE_PLAN.md#working-method) before
any stage here.** It records what is already implemented, which is most of what
looks missing at first glance, and how to diagnose by measurement rather than by
reading code. The commit rules in [AGENTS.md](../AGENTS.md) apply unchanged.

---

## What is already there

Checked 2026-09-13. Do not rediscover these.

| Mechanism | State |
|---|---|
| Brake load | Quadratic in rpm, `load: (c0, c1, c2)` per preset |
| Dyno | `DynoMode`: `FreeRev`, `RpmHold`, `SweepPull`, `Motoring`, with recorded pulls |
| Engine braking | `CLOSED_THROTTLE_PMEP` pumping term |
| Spark and AFR schedules | `schedule_spark_advance(load, rpm)`, `schedule_afr(load, rpm, throttle, dt)` — **both already take load** |
| Limiter strategies | `HardCut`, `SoftCut`, `RotatingStutter` |
| Cut mechanisms | `LimiterCut::{None, Fuel, Spark}` |
| Anti-lag, DFCO | `EngineControlUnit`, with tip-in tracking |
| Per-cylinder health | `CylinderHealth`: dead plug, dead injector, both |
| Backfire trigger | Gated on unburnt fuel mass **and** exhaust temperature |
| Turbo shaft speed | First-order lag with asymmetric spool, surge margin |
| Turbo voices | Whistle, surge flutter, blow-off, wastegate |
| Valve events | Raised-cosine lift, per-valve open angle, duration, lift, diameter, `C_d` |
| Overlap reversion | Modelled in the intake plumbing, with a test |
| Cycle-to-cycle variation | Per-cylinder amplitude and phase, rerolled per cycle, scheduled on rpm |
| Thermal state | Lumped block, oil, coolant and exhaust masses; Vogel oil viscosity; thermostat |
| Cold idle | `cold_idle_rise = 0.6`, decaying on `cold_fraction`, not on a timer |
| Rotary preset | Exists as a deliberate approximation — see M5 |

---

## Ordering

```
M1 vehicle load ──┬── lights up spark and AFR schedules that already take load
                  │
                  ├── M2 boost and turbine ──── needs M1 for a real load line
                  │
                  └── M4 cam and the idle limit cycle
                            │
                            └── M5 rotary ports ─── same area-vs-angle machinery

M3 backfire taxonomy ─── independent, but do TIMBRE T6 first
M6 cold start and cranking ─ independent
```

M1 first: it is small, and three other stages are measured against a load line
that does not exist yet. M3 must not start before [TIMBRE_PLAN.md's
T6](TIMBRE_PLAN.md) — adding four kinds of backfire to a backfire that aliases
gets four kinds of rasp.

---

# Stage M1 — Vehicle load

**Goal.** Let the engine be loaded by something other than its own drag curve.

**Why.** Load is a quadratic in rpm and nothing else, so the engine has exactly
one load line and it is a function of speed alone. There is no gear, no vehicle
mass, no road, no grade, no clutch. That means the simulator cannot produce the
single most common thing anybody listens to an engine do: **hold a speed under
load**. Cruising at 2500 rpm in sixth and pulling at 2500 rpm in second are the
same sound today, and they are nothing like each other in life — different spark
advance, different AFR, different trapped mass, different exhaust temperature,
and therefore a different pipe tuning and a different note.

The machinery to use it is already built and starved.
`schedule_spark_advance(load, rpm)` and `schedule_afr(load, rpm, throttle, dt)`
both take a load argument. Giving them a real one is most of this stage's value.

**Files.** New `src/physics/vehicle.rs`; `src/bench.rs` (`Driveline`),
`src/physics/control.rs`, `src/bench/config.rs`, `src/ui/telemetry.rs`.

**Design.**

- `Gearbox` — a ratio list plus a final drive, and a currently selected gear.
  Neutral is a gear, and it is the one that makes the existing free-rev
  behaviour a special case rather than a separate code path.
- `RoadLoad` — the standard three terms, which is the same shape as the existing
  `load` tuple and should replace it rather than sit beside it:

  ```text
  F = m g C_rr + 0.5 rho C_d A v^2 + m g sin(grade)
  ```

  Reflect it to the crank through the ratio and the wheel radius. The existing
  quadratic stays available as the dyno's absorber curve, which is what it
  actually was.
- **Load fraction** is what the schedules want, and it must be defined once and
  stated: normalised trapped mass against what the cylinder would trap at
  atmospheric pressure and ambient temperature at that speed. Not throttle
  position — a throttle is a valve, not a load — and this distinction is the
  whole point of the stage.
- `Clutch` — engaged, slipping, or open, with a torque capacity. Slipping is
  needed for a standing start and for the moment of a shift, and it is where the
  driveline's inertia changes, which the block already integrates against.
- Shift events are a driveline concern, not an audio one: the existing cut
  machinery covers the ignition cut on an upshift.

**Tests.**

- Steady-state speed in a gear on a level road solves where road load equals
  brake torque through the ratio; check against a hand-computed case.
- The same engine speed in two gears yields different load fractions, and
  therefore different scheduled spark advance.
- A grade raises load fraction at constant speed; the AFR schedule enriches.
- Neutral reproduces today's free-rev behaviour to within integration error, so
  every existing measurement remains comparable.
- Clutch slip dissipates the torque difference and does not create energy.
- Exhaust temperature is higher at the same rpm under higher load, and the
  primaries' tuning frequency moves with it.

**Commits.**

```
add gearbox ratios and final drive
add road load with rolling aero and grade terms
add a clutch with a torque capacity
define load fraction from trapped mass
feed load fraction to the spark and afr schedules
expose the driveline in the engine config
test the same rpm in two gears schedules different spark
```

**Prompt.**

> Implement Stage M1 of `docs/MECHANISM_PLAN.md`. Read that stage and
> `docs/TIMBRE_PLAN.md`'s Working method section first, then `AGENTS.md`.
>
> The important definition is load fraction, and it is not throttle position.
> Define it once, from trapped mass against an atmospheric reference at the same
> speed, document it where it is defined, and feed it to the two ECU schedules
> that already take a `load` argument and are currently being handed something
> else.
>
> Neutral must reproduce today's free-revving behaviour to integration error, so
> every recorded fingerprint and dyno pull stays comparable. Prove that with a
> test before you wire anything else in.
>
> Do not touch `src/audio` in this stage. The note changes because the exhaust
> gets hotter and the spark moves, not because anything in the synth was told
> about gears.

---

# Stage M2 — Boost and the turbine

**Goal.** Make a turbocharged engine a turbocharged engine rather than an
atmospheric one with a whistle.

**Why.** `src/audio/mod.rs` says it plainly: *"The block itself is atmospheric
either way — there is no turbine, no compressor and no boost anywhere in the
solver."* `TurboModel` is a first-order lag on engine speed times throttle,
producing a number that pitches a whistle. Nothing else.

Three consequences, and the third is the one people actually hear:

1. A turbo engine traps the same mass as an atmospheric one, so it makes the
   same torque and the same blowdown pressure. The preset's extra power is not
   simulated, it is asserted.
2. Shaft speed is a proxy of a proxy, so there is no lag from an actual pulse
   train: no spool on a downshift, no fall-off when the wastegate opens, no
   difference between a small housing and a large one beyond a time constant
   somebody typed.
3. **There is no turbine in the exhaust path.** A turbine is a large acoustic
   obstruction between the collector and the rest of the system — it scatters,
   absorbs and smears the blowdown pulse. That is *why* a turbo car sounds muted
   and whooshy rather than hard, and it is entirely absent here. This is the
   single largest reason the turbo presets do not sound turbocharged, and it is
   a waveguide element, not a voice.

**Files.** `src/physics/intake.rs`, `src/physics/plumbing.rs`,
`src/audio/waveguide.rs`, `src/audio/mod.rs`, `src/bench/config.rs`.

**Design.**

- **Turbine as a network element.** A `Turbine` in the exhaust chain between the
  collector and the downpipe, built the way `ExpansionChamber` and
  `AbsorptiveSilencer` already are: a two-port that returns
  `(upstream, transmitted)`. It is mostly a resistive termination with a
  frequency-dependent transmission — high orders scatter into the wheel and are
  lost, low orders pass. Give it a real loss term from the start; see what
  [TIMBRE_PLAN.md's T5](TIMBRE_PLAN.md) had to fix in `ExpansionChamber`.
- **Compressor map, coarsely.** Pressure ratio as a function of corrected shaft
  speed and corrected mass flow, with a surge line and a choke line. A crude map
  with the right topology beats an accurate time constant, because surge and
  choke are what the driver hears.
- **Close the loop.** Compressor outlet pressure feeds the intake plenum, which
  raises trapped mass, which raises torque and exhaust enthalpy, which drives
  the turbine, which accelerates the shaft. That loop *is* lag; delete the
  asymmetric time constants once it runs, rather than keeping both.
- **Wastegate and blow-off become real.** Both currently exist only as voices.
  A wastegate bleeds turbine flow to hold a target pressure ratio; a blow-off
  vents compressor outlet on a shut throttle. Each should move the physics and
  let the existing voice key off the actual event.
- **Charge cooling.** An intercooler is a temperature drop between compressor
  and plenum, and it changes trapped mass and knock margin. One parameter,
  large effect, and knock is already solved and voiced.
- Keep `TurboModel` as the fallback for a preset that does not define a map, so
  nothing in the catalogue breaks mid-stage.

**Tests.**

- Boost raises trapped mass and brake torque at constant speed and throttle.
- Shaft speed lags a throttle step, and the lag is longer for a larger wheel
  inertia — not because a constant says so.
- A turbine in the chain attenuates high orders more than low ones; total
  radiated energy downstream is strictly less than without it.
- Opening the wastegate lowers boost and lowers shaft speed.
- Surge is entered by crossing the map's surge line, not by a throttle
  threshold.
- Intercooling lowers charge temperature and raises the knock margin.

**Commits.**

```
add a turbine element to the exhaust network
add a compressor map with surge and choke lines
feed compressor outlet pressure to the intake plenum
drive the turbine from exhaust enthalpy
make the wastegate bleed turbine flow
make the blow-off valve vent compressor outlet
add intercooler charge cooling
test boost raises trapped mass
test the turbine attenuates high orders
```

**Prompt.**

> Implement Stage M2 of `docs/MECHANISM_PLAN.md`. Read that stage, Stage M1, and
> `docs/TIMBRE_PLAN.md`'s Working method section.
>
> Build the turbine element **first** and measure it before touching the
> compressor. It is the half that changes what a turbo engine sounds like, it is
> independent of the boost loop, and it is testable on its own. Give it a real
> dissipative term — read what T5 of the timbre plan had to fix in
> `ExpansionChamber` and do not repeat it.
>
> Then close the boost loop and delete the asymmetric spool constants in
> `TurboModel`; keeping both a real loop and a hand-tuned lag is how the two
> disagree later. Leave `TurboModel` reachable as the fallback for presets with
> no map so the catalogue keeps building.
>
> Say in your final message what the turbine did to the octave-band table of a
> turbo preset, against the same preset without one.

---

# Stage M3 — What kind of backfire

**Goal.** Make a crackle, a bang, an afterfire and an anti-lag burst four
different sounds.

**Why.** There is one `BackfireVoice`, one pulse shape — attack `0.00018`, one
decay, one noise ratio — and a severity scalar. Every pop in the catalogue is
the same event at a different volume, which is why they all sound alike and all
sound like each other's rasp.

What actually separates them is **where in the pipe the charge lights**, and the
exhaust is already a network of addressable nodes, so this is expressible with
what is built:

| Kind | Ignites | Heard as |
|---|---|---|
| Crackle / pop | Tailpipe or mid-pipe, far from the port | Small, soft, heavily coloured by the pipe behind it — almost no direct path |
| Bang | Collector | Loud, short path, most of the network still ahead of it |
| Afterfire | Large accumulated charge, slow deflagration anywhere | Low, broad *whoomp* rather than a crack |
| Anti-lag | At the exhaust port, continuously | Periodic, locked to firing, upstream of the entire system |

A pop that lights in the tailpipe has almost no pipe left to travel through, so
it arrives raw and short. One that lights at the port travels the whole network
and arrives with every resonance the system has. Today both are injected at the
same node, which is why the pipe's character never varies with the event.

**Files.** `src/audio/dsp.rs`, `src/audio/waveguide.rs`, `src/physics/control.rs`.

**Design.**

- `ExhaustNetwork::inject(node, pressure)` — add a forward wave at a named node:
  port, collector, post-silencer, tailpipe. The chain already holds its
  interface returns per bank, so the nodes exist; this exposes them.
- `BackfireKind` chosen from the physics that produced it, not from a preset
  field. Unburnt mass, exhaust temperature and time since the cut already
  determine it: a small charge that has had time to travel lights downstream; a
  large one that has just been dumped lights at the port.
- **Ignition delay.** Fuel accumulates, then lights, then the flame propagates.
  Today a cut produces one pulse per trigger. A real cut produces a burst with a
  random inter-event time, which is what a crackle tune *is*. Model the delay,
  and let a single cut spawn several events at different nodes.
- One pulse *shape* per kind: a deflagration in a large volume is not a fast
  attack made quieter, it is genuinely slower. Derive the attack from the
  charge mass and the volume it lights in, so the shape follows the event.
- Anti-lag already has its own condition in the ECU; route it to port injection
  and let it be periodic rather than triggered.

**Tests.**

- Injection at the tailpipe reaches the mouth earlier and with less resonant
  colouring than injection at the port; the delay difference matches the
  network's transit time.
- A larger unburnt mass produces a slower attack, not only a louder one.
- One cut produces more than one event, with non-constant spacing.
- Anti-lag produces a periodic train locked to firing; a limiter cut does not.
- Every kind is bandlimited to the same standard T6 set.

**Commits.**

```
expose exhaust network injection nodes
choose a backfire kind from charge mass and temperature
derive the backfire attack from the charge volume
add ignition delay so one cut fires several events
inject anti-lag at the exhaust port
test tailpipe and port injection differ in transit
```

**Prompt.**

> Implement Stage M3 of `docs/MECHANISM_PLAN.md`. Read that stage and
> `docs/TIMBRE_PLAN.md`'s Working method section.
>
> **Do not start until T6 of the timbre plan is committed.** The current
> backfire aliases; four kinds of aliased backfire is four kinds of rasp. If T6
> is not in `git log`, stop and say so.
>
> The mechanism is injection node, not voicing. Add injection at named nodes in
> `ExhaustNetwork` and let the physics choose the node; do not add a
> `BackfireKind` field to the preset and switch on it. A pop that lights in the
> tailpipe should sound different because it travelled a shorter pipe, which is
> a thing the network already computes.
>
> Report the measured transit-time difference between port and tailpipe
> injection in your final message.

---

# Stage M4 — Cam character and the idle limit cycle

**Goal.** Make a big cam lope.

**Why.** Two halves, and the second is the one that is actually missing.

The **cam** half is partly there. `ValveEvent` is a raised cosine with an open
angle, a duration, a lift and a port diameter, so duration and overlap are
already expressible and reversion is already modelled. What is missing is the
vocabulary: a cam is specified by **lobe separation angle** and **advance**, not
by two absolute opening angles, and the profile's **aggressiveness** — how fast
it comes off the seat — is fixed at a raised cosine for every engine. A solid
roller and a stock hydraulic have very different valvetrain noise for the same
duration, and that difference is entirely in the ramp.

The **idle** half is not there at all, and it is the whole of the chop. A lopey
idle is not noise and it is not just a rough burn — it is a **limit cycle**:

```
big overlap -> reversion -> diluted charge -> weak or partial burn
   -> torque drops -> speed drops -> less reversion -> good burn
   -> torque rises -> speed rises -> more reversion -> ...
```

Every term in that loop except the last link exists. What closes it is an idle
governor with real lag: the sim currently holds `idle` as a target with no
controller that can overshoot, so it settles instead of hunting. Cycle-to-cycle
variation adds roughness, but roughness is not lope — lope is periodic, at a
rate well under the firing frequency, and a listener hears the period.

**Files.** `src/physics/thermodynamics.rs` (`ValveEvent`),
`src/physics/control.rs`, `src/bench/config.rs`, `src/audio/dsp.rs`.

**Design.**

- **Cam specification.** Add lobe separation angle and cam advance as the
  primary parameters, with the absolute open angles derived. Keep the existing
  fields working by deriving them the other way, so no preset breaks.
- **Profile aggressiveness.** One parameter interpolating the lift curve from
  the current raised cosine toward a faster-ramping polynomial at the same
  duration and lift. It changes the flow curve slightly and the valvetrain
  impulse a lot; wire it to the existing `ImpulsiveConfig` level for the valve
  voices.
- **A real idle governor.** A PI controller on an idle air bypass, with an
  actuator lag and an authority limit. Give it the lag a real one has. Its
  overshoot against the reversion loop is the lope, and it must be allowed to
  fail to converge — an engine with a cam too big for its idle *hunts*, and the
  governor should not be permitted to hide that.
- **Partial-burn feedback.** Charge dilution from reversion must reduce burn
  completeness, not merely torque. The Wiebe efficiency parameter and burn
  duration are the places for it; a diluted charge burns slower and less
  completely, which sends unburnt fuel out of the port — and that is the same
  quantity the backfire voice keys off, which is why a lopey engine crackles at
  idle on its own.
- Make the loop's period observable in telemetry, so the chop can be measured
  rather than argued about.

**Tests.**

- With a stock cam the governor converges; with a large overlap cam at the same
  target it enters a limit cycle instead.
- The limit cycle's period is well below the firing frequency and stable across
  a render.
- Reversion mass rises with overlap, and burn completeness falls with reversion
  mass.
- Unburnt fuel out of the port is non-zero at a lopey idle and near zero at a
  stock one.
- Lobe separation angle and cam advance reproduce the existing presets' absolute
  angles exactly, so nothing in the catalogue moves.
- A more aggressive ramp raises the valvetrain impulse at the same duration and
  lift.

**Commits.**

```
specify cams by lobe separation and advance
add a cam profile aggressiveness parameter
scale valvetrain impulse with ramp rate
add an idle governor with actuator lag
reduce burn completeness with charge dilution
publish the idle limit cycle period
test a big overlap cam fails to settle at idle
```

**Prompt.**

> Implement Stage M4 of `docs/MECHANISM_PLAN.md`. Read that stage and
> `docs/TIMBRE_PLAN.md`'s Working method section.
>
> The chop is a limit cycle, not roughness, and the missing link is a governor
> that can overshoot. Cycle-to-cycle variation is already implemented and is not
> it — do not turn it up and call the result a lope.
>
> Build the loop in this order: dilution reduces burn completeness, then the
> governor with lag, then check whether it oscillates. If it converges with a
> large cam, the dilution feedback is too weak and that is the thing to fix, not
> the governor's gains.
>
> The cam respecification must be exactly neutral on the catalogue. Prove that
> with a test that derives every existing preset's absolute angles from its new
> lobe separation and advance before you change any behaviour.
>
> Report the measured limit-cycle period for a large-cam preset.

---

# Stage M5 — Rotary, properly

**Goal.** A Wankel that is a Wankel, and port types that differ because the
ports differ.

**Why.** The current rotary is an honest costume, and `src/bench.rs` says so:
a very long rod to flatten the motion, a stretched Wiebe for the long thin
chamber, a low compression ratio and huge "valve" overlap standing in for
peripheral porting. It gets the firing order right — four firings per 720
degrees, which is the correct order 2 — and that is genuinely most of the way
to the buzz.

What it cannot express is everything the user is asking about:

- **The eccentric shaft turns three times per rotor revolution.** Every
  mechanical noise source on a rotary — bearings, seals, the rotor itself — is
  at rotor rate, one third of the shaft rate the whistle and the firing sit at.
  A reciprocating solver has no such ratio, so the rotary's mechanical bed is
  currently at the wrong order. This is a real and cheap timbral signature.
- **Ports are uncovered, not lifted.** The distinction between port types is
  almost entirely **how fast the port opens**, and that is geometry:

  | Port | Uncovered by | Opening rate | Character |
  |---|---|---|---|
  | Side | The rotor's flat side face | Gradual, over many degrees | Streetable, idles, softest |
  | Bridge | A larger side port with a bridge of material | Longer duration, much more overlap | The lope; brap at idle |
  | Half / J-bridge | Between the two | | |
  | Peripheral | The **apex seal** sweeping past a hole in the rotor housing | Near-instantaneous | Enormous overlap, barely idles, the scream |

  A peripheral port's blowdown edge is close to a step because an apex seal
  crosses the port in a few degrees. A side port ramps. That difference in
  `dA/dtheta` is the whole sound, and it is the same machinery Stage M4 needs
  for cam ramp rate — build it once.
- **Two plugs per rotor.** Leading and trailing, fired roughly ten to fifteen
  degrees apart, so heat release is two overlapping events rather than one. The
  solver takes a single Wiebe.
- **Idle.** A rotary idles differently, and a peripheral port barely idles at
  all. `idle` is a preset number so the *speed* is settable, but the
  instability is not — which is Stage M4's governor, reused.

**Files.** `src/physics/cylinder.rs` or a new `src/physics/rotor.rs`,
`src/physics/thermodynamics.rs`, `src/bench.rs`, `src/bench/config.rs`,
`src/audio/dsp.rs`.

**Design.**

Decide the scope question first and state the answer in the commit: **does the
solver get real epitrochoid geometry, or does the costume get better?** Both are
defensible. The honest chamber volume is

```text
V(theta) = V_min + V_d/2 * (1 - cos(theta_rotor))    to first order
```

with `theta_rotor = theta_shaft / 3`, which is not far from what a long rod
already produces — so the geometry is the *cheap* part. The expensive part is
porting, and porting is where the sound is.

The recommendation is: keep a near-sinusoidal chamber, but make it a first-class
`RotorGeometry` rather than a distorted slider-crank, so the 3:1 shaft ratio is
in the type rather than in a comment, and spend the stage on ports.

- `PortGeometry` replacing `ValveEvent` for a rotary: an area-versus-angle curve
  parameterised by opening rate, duration and peak area, with named
  constructors for side, bridge, half-bridge and peripheral. Share the ramp
  machinery with Stage M4's cam aggressiveness.
- Overlap follows from the port geometry rather than being asserted, so a
  peripheral port's idle problem is *emergent* — it should be hard to make a
  peripherally ported engine idle, and that is correct.
- Two-plug heat release: two Wiebe events at a stated split, with the trailing
  plug's contribution and delay both parameters.
- Mechanical voices at rotor rate, not shaft rate. Check `MechanicalSpec::rotary`
  and fix its orders.
- Exhaust already runs hot in the preset and should stay hot; a rotary's
  still-burning charge at port opening is why.

**Tests.**

- A peripheral port's `dA/dtheta` at opening exceeds a side port's by the stated
  ratio, and the resulting blowdown edge has measurably more high-order content.
- The four port types are distinguishable in the octave-band table, in the
  expected order.
- Mechanical voice orders land at one third of the shaft order.
- Firing order stays at four events per 720 degrees of eccentric shaft for a
  two-rotor.
- The trailing plug's heat release is second and smaller, and removing it
  measurably changes the pressure trace.
- A peripherally ported engine does not settle at the idle a side-ported one
  does, with the Stage M4 governor unchanged between them.

**Commits.**

```
add rotor geometry with the eccentric shaft ratio
add port area curves for the rotary
add named side bridge and peripheral port profiles
add trailing plug heat release
put rotary mechanical voices at rotor rate
test peripheral porting opens faster than side porting
```

**Prompt.**

> Implement Stage M5 of `docs/MECHANISM_PLAN.md`. Read that stage, Stage M4, and
> `docs/TIMBRE_PLAN.md`'s Working method section. Read the doc comment on
> `EnginePreset::two_rotor_wankel` in `src/bench.rs` first — it states exactly
> what the current approximation is and why, and it is accurate.
>
> Spend the stage on **ports**, not on epitrochoid geometry. The chamber volume
> curve is nearly sinusoidal and the long-rod trick already approximates it; the
> thing a listener can hear is how fast the port uncovers, and that is what
> separates a side port from a bridge port from a peripheral port.
>
> Build the area-versus-angle ramp so Stage M4's cam aggressiveness can use the
> same code. If M4 is already committed, reuse its ramp rather than writing a
> second one.
>
> Do put the 3:1 eccentric shaft ratio in a type rather than in a comment, and
> fix `MechanicalSpec::rotary`'s orders while you are there — the mechanical bed
> is currently at shaft rate and belongs at rotor rate.
>
> A peripherally ported engine should be *hard to idle*. If yours idles
> smoothly, the overlap is not reaching the charge, and that is a bug to report
> rather than a result to accept.

---

# Stage M6 — Cold start and cranking

**Goal.** The first thirty seconds.

**Why.** The thermal model is one of the better-built parts of the crate —
lumped block, oil, coolant and exhaust masses, Vogel oil viscosity, a
thermostat, a `cold_fraction` that decays on heat rather than on a timer, and a
`cold_idle_rise` of 0.6 that puts an 850 rpm idle at 1360 cold. All of it is
real and none of it is wasted: a cold engine's exhaust is cooler, so the pipe's
speed of sound is lower, so the tuning moves. That much already happens.

What is missing is at both ends of it.

- **`cold_fraction` never reaches the synth.** Grep it: `src/audio` sees
  `oil_temperature` once, for the friction correlation, and nothing else. So no
  voice knows the engine is cold. A cold engine is *louder* mechanically, not
  quieter — cold clearances mean more piston slap, noisier tappets, a harder
  injector tick — and none of that can happen.
- **There is no cranking.** `EngineBlock::cold_start()` sets the thermal state
  of an engine that is already turning. There is no starter motor, no
  compression-by-compression chug, no catch, no first fire, no flare and settle.
  That sequence is most of what people mean by "cold start" as a sound.

**Files.** `src/physics/engine_block.rs`, `src/physics/control.rs`,
`src/audio/dsp.rs`, `src/audio/mod.rs`, `src/bench.rs`, `src/main.rs`.

**Design.**

- **Put `cold_fraction` in the snapshot.** It is one `f32` and it is the gate for
  everything below. It crosses at the `EngineSnapshot` boundary like every other
  physics value, in `f32`, once.
- **Cold-scaled mechanical levels.** Piston slap, tappet and injector levels rise
  with `cold_fraction` from the existing `ImpulsiveConfig` levels. Clearances
  close as the block warms, so the scaling is on the impulse, not on a filter.
- **A starter.** A torque source with its own speed, its own whine order, and an
  engagement and release. Cranking speed is set by the starter against the
  compression torque the solver already computes, so the chug is emergent — the
  engine slows on every compression stroke and the starter drags it over. Do not
  synthesise the chug.
- **Catch and first fire.** Fuel is enabled, the first cylinder to see spark at
  a fireable condition catches, and the engine accelerates away from the starter.
  The starter releases on an overspeed condition, as a real one does.
- **Cold enrichment and its misfires.** A cold engine runs rich and still
  misfires, because fuel condenses on cold port walls. Model the wall film as a
  first-order lag on delivered fuel against commanded fuel, keyed on
  `cold_fraction`. Its misfires feed unburnt fuel to the exhaust, which the
  backfire voice already reads — so a cold engine pops on its own, correctly.
- **The flare.** `cold_idle_rise` already gives the raised idle; what is missing
  is the *overshoot* on catch and the settle into it. Stage M4's governor is the
  right home for that; if M4 is done, reuse it.

**Tests.**

- Cranking speed oscillates with compression events and does not converge to a
  constant.
- The starter releases after catch and its whine stops.
- `cold_fraction` at 1 raises the mechanical impulse levels by the stated factor
  and at 0 leaves them exactly as today, so every recorded fingerprint stays
  valid.
- Wall film delays delivered fuel behind commanded fuel, and the lag shortens as
  the engine warms.
- A cold start produces unburnt fuel at the port; a warm idle does not.
- Idle speed decays from the cold value to the warm one as heat goes in, not on
  a timer.

**Commits.**

```
publish cold fraction in the engine snapshot
scale mechanical impulse levels with cold fraction
add a starter motor torque source
release the starter on catch
add a cold fuel wall film lag
test cranking speed ripples on compression
```

**Prompt.**

> Implement Stage M6 of `docs/MECHANISM_PLAN.md`. Read that stage and
> `docs/TIMBRE_PLAN.md`'s Working method section.
>
> The thermal model is already good and you should change very little of it. The
> two gaps are that `cold_fraction` never crosses into `src/audio`, and that
> there is no cranking sequence at all.
>
> Do not synthesise the cranking chug. The solver computes compression torque;
> a starter with a finite torque against that torque produces the chug for free,
> and a chug that is emergent will speed up correctly as the oil thins. If yours
> needs a shaped envelope to sound right, the starter torque is wrong.
>
> Everything must be exactly neutral at `cold_fraction == 0`, so the recorded
> fingerprints — all of which are measured warm — stay valid. Prove that with a
> test first.

---

## Not in this plan

- **A full 1-D gas dynamics solver in the cylinder.** Stage 15 of
  [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) covers the physics pipe; this
  plan assumes it either lands there or does not.
- **Tyre, suspension and body noise.** The propagation model radiates from
  apertures on a chassis, but the chassis is not a vehicle and this plan does not
  make it one. M1 adds a load, not a car.
- **Gearbox and driveline acoustics.** Straight-cut gear whine already exists as
  a mechanical voice. M1 gives it a real input speed; voicing it further is a
  separate piece of work.
- **Electric and hybrid.** Out of scope for a combustion simulator.
