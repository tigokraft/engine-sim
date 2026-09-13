# Turbo plan

A dedicated plan for forced induction, replacing
[MECHANISM_PLAN.md's Stage M2](MECHANISM_PLAN.md) with the full version.

It is separate from the other three plans because it is the only piece of work
that changes the **physics, the acoustics and the control system at once**, and
because doing a third of it is worse than doing none: a compressor with no
turbine is a boost number nothing pays for, and a turbine with no compressor is
a restriction with no reward.

Read [TIMBRE_PLAN.md's Working method](TIMBRE_PLAN.md#working-method) before any
stage here. The commit rules in [AGENTS.md](../AGENTS.md) apply unchanged.

---

## Prerequisites

| Gate | Why |
|---|---|
| [TIMBRE T5](TIMBRE_PLAN.md) | The silencer chain is currently bypassed on two presets. Adding a turbine element to a chain that is being skipped measures nothing. |
| [TIMBRE T0](TIMBRE_PLAN.md) | Every claim in this plan is "the note changed". Without octave-band and crest metrics that is an opinion. |
| [TIMBRE T1](TIMBRE_PLAN.md) | The midrange is 8 dB down. A turbine's whole audible job is in the midrange and above, so its effect cannot be judged until the band exists. |
| [MECHANISM M1](MECHANISM_PLAN.md) | A turbo is a load device. Without a vehicle load there is no way to hold an engine at a boost threshold, which is where every interesting turbo behaviour lives. |

**Do not start TB1 before those four.** TB0 is the exception and is explicitly
allowed to go early — see its stage.

---

## Where the code stands

`src/audio/mod.rs` states it exactly, and the statement is accurate:

> The block itself is atmospheric either way — there is no turbine, no
> compressor and no boost anywhere in the solver.

What exists today:

| Piece | State |
|---|---|
| `TurboModel` | A first-order lag on `rpm x throttle`, with asymmetric spool constants. Produces a shaft speed and nothing else. |
| `TurboVoicing` | Whistle order, reference speed, level. |
| `BlowOffVoicing`, `WastegateVoicing` | Voices with thresholds. No device behind either. |
| `TurboModel::surge` | A heuristic on shaft speed and shut throttle. Not a map. |
| `Induction` | Lives in `src/audio/mod.rs`. Induction is currently an **audio** concept. |
| Anti-lag | Real in the ECU: retards ignition, enriches on overrun, raises manifold temperature. Feeds the backfire voice. The one genuinely-modelled forced-induction behaviour. |

So a turbocharged preset traps the same mass as an atmospheric one, makes the
same torque, launches the same blowdown pulse into the same unobstructed pipe,
and has a whistle painted over the top.

## The three insertion points

The reason this is tractable is that the boundaries already exist and are clean.

**1. The compressor replaces one argument.**

```rust
IntakePlenum::advance(&mut self, dt, tps, upstream: &PortState, draws: &[ValveDraw])
```

`upstream` is currently `IntakePlenum::ambient_upstream(env, gas)`. A compressor
is a `PortState` at a higher pressure and temperature. That is the entire
plumbing change on the intake side — and `ThrottleBody::mass_flow` already
handles `p_man / p_up` in both directions, with a doc comment that names the
boost case. The boundary was built for this.

**2. The turbine adds a loss coefficient.**

`ExhaustSystem::back_pressure(mass_flow, cutout_open)` already sums `k / A^2`
terms over the primaries, collector, silencers and tailpipe. A turbine is a
large `k`. That gives the physics-side back pressure the engine pays for boost.

**3. The turbine is a two-port in the acoustic network.**

`ExpansionChamber`, `QuarterWaveStub` and `AbsorptiveSilencer` are all
`step(p_in_plus, p_out_minus) -> (upstream, transmitted)`. A `Turbine` is the
same shape. This is the half that changes what a turbo engine *sounds* like, and
it is independent of everything else in this plan.

## What a turbine does to the sound

Worth stating plainly, because it is the thing most often got wrong.

A turbine **splits the exhaust into two acoustic domains**. Upstream of it sits
a short, hot, high-pressure manifold whose pulses now reflect off a near-closed
end instead of running down a header — so the manifold becomes its own resonant
system with its own, much higher, tuning. Downstream sits a comparatively smooth
flow feeding the rest of the system, with the blowdown edge scattered into the
wheel and the high orders gone.

That is why a turbo car sounds muted and whooshy rather than hard, why the
tailpipe note loses its crack, and why the *intake* becomes the loud end of the
car. None of it is a voice. All of it is one two-port element and a boundary
condition.

---

## Ordering

```
TB0 turbine as an acoustic element ── allowed early; the audible half
      │
      ├── TB1 compressor map ──┐
      ├── TB2 turbine map and shaft dynamics ──┤
      │                                        │
      └────────────────────────────────────────┴── TB3 close the loop
                                                        │
                                    TB4 boost control and the valves
                                                        │
                                    TB5 transients and architectures
                                                        │
                                    TB6 voicing from real state
                                                        │
                                    TB7 calibration and catalogue
```

TB1 and TB2 are independent of each other and both are useless until TB3. TB6
deliberately comes late: the voices work today off proxies, and rewiring them
before the state they should key off exists means doing it twice.

---

# Stage TB0 — The turbine as an acoustic element

**Goal.** Put a turbine in the exhaust path. Nothing else.

**Why.** It is the half of a turbocharger that a listener hears, it is
independent of boost, it is one element in a chain that already has four of
them, and it can be measured on its own. It is also the cheapest large change in
character in this plan, so it goes first even though it is physically backwards
to have a turbine that extracts no work.

**This stage may start as soon as [TIMBRE T5](TIMBRE_PLAN.md) lands**, ahead of
T0 and T1, because its effect is large enough to see without them. Do not
re-level anything against it until T1 is done.

**Files.** `src/audio/waveguide.rs`, `src/physics/plumbing.rs`,
`src/bench/config.rs`.

**Design.**

- `Turbine` as a `SilencerElement`-shaped two-port:
  `step(p_in_plus, p_out_minus) -> (upstream, transmitted)`.
- Three physical behaviours, all of which matter:
  - **Reflection.** The nozzle ring is a large area contraction, so a
    substantial fraction of the incident wave reflects. Use the same
    `ScatteringJunction` machinery the collector uses, sized from the effective
    nozzle area.
  - **Dissipation.** Energy that enters the wheel is converted or lost and does
    not come back. Give it a real loss term from the first commit — read what
    [TIMBRE T5](TIMBRE_PLAN.md) had to fix in `ExpansionChamber`, which had none
    and made mufflers *louder*.
  - **Smearing.** The wheel passages are many short paths of differing length,
    so a sharp edge arrives dispersed. A short allpass chain or a small diffuse
    network, not a lowpass — a lowpass makes it dull, dispersion makes it
    *whooshy*, and those are different sounds.
- **Blade-pass content.** The wheel modulates the flow at blade count times
  shaft speed. This is a real acoustic source on the exhaust side and is
  distinct from the compressor whistle on the intake side. Take shaft speed from
  the existing `TurboModel` for now; TB6 rewires it.
- **Housing A/R** as the element's tuning parameter. A small A/R is a tighter
  nozzle: more reflection, more back pressure, faster spool, more muffling. It
  is the one number that most changes both the sound and the behaviour, so it
  should exist from the start even though nothing uses it for spool yet.
- Physics side: add the turbine's `k` term to `back_pressure` so the engine
  starts paying for it immediately.
- Wire it into `ExhaustNetwork` between the collector and the downstream chain,
  so the manifold upstream of it becomes its own domain.

**Tests.**

- A turbine in the chain attenuates high orders more than low ones.
- Total radiated energy downstream is strictly less than without it, at every
  frequency — the element cannot add energy anywhere.
- A smaller A/R reflects more and transmits less.
- The manifold upstream of the turbine resonates at its own, higher frequency,
  and that frequency is absent without the turbine fitted.
- A step edge arrives dispersed rather than merely quieter: rise time lengthens
  while total energy in the band is preserved.
- Blade-pass content appears at blade count times shaft speed and tracks it.
- `back_pressure` rises with a turbine fitted and falls with a larger A/R.

**Commits.**

```
add a turbine element to the exhaust network
scatter and dissipate at the turbine nozzle
disperse the blowdown edge through the wheel
add turbine blade pass content
add the turbine loss coefficient to back pressure
test the turbine attenuates high orders
test the pre-turbine manifold has its own resonance
```

**Prompt.**

> Implement Stage TB0 of `docs/TURBO_PLAN.md`. Read that stage, the "What a
> turbine does to the sound" section above it, and
> `docs/TIMBRE_PLAN.md`'s Working method section.
>
> This stage adds no boost and no compressor. It is one two-port element in the
> exhaust network, shaped exactly like `ExpansionChamber` and
> `AbsorptiveSilencer`, plus a loss coefficient in `back_pressure`.
>
> Three behaviours and they are not interchangeable: reflect at the nozzle,
> dissipate in the wheel, and **disperse** the edge. Dispersion is the one that
> makes it sound like a turbo. If you implement it as a lowpass it will sound
> dull instead of whooshy, and that is the wrong answer — use an allpass chain.
>
> Give it a real dissipative term in the first commit. `ExpansionChamber`
> shipped without one and made mufflers louder; do not repeat it.
>
> Report the before-and-after octave-band table for a turbo preset in your final
> message, and say what happened to the pre-turbine manifold resonance.

---

# Stage TB1 — The compressor map

**Goal.** Pressure ratio from corrected speed and corrected flow, with real
surge and choke boundaries.

**Why.** Surge and choke are not thresholds on throttle position, they are
places on a map, and every dramatic compressor behaviour is a map excursion. A
heuristic can produce a flutter on lift; only a map produces flutter on *the
wrong gear at the wrong rpm with the wrong wheel*, which is the thing that makes
one turbo setup feel different from another.

**Files.** New `src/physics/compressor.rs`; `src/bench/config.rs`.

**Design.**

- Corrected quantities, because a map is only portable in them:

  ```text
  N_corr = N / sqrt(T_in / T_ref)
  m_corr = m sqrt(T_in / T_ref) / (P_in / P_ref)
  ```

- Map representation: speed lines as `(m_corr, pressure_ratio, efficiency)`
  triples, interpolated. Do **not** fit an analytic surface; real maps are not
  analytic and the interesting behaviour is at the edges where a fit is worst.
  Ship a small number of honest points per speed line.
- **Surge line** as the locus of each speed line's minimum-flow point, and
  **choke** as its maximum-flow point. Crossing the surge line is an event, not
  a gradient: flow reverses, the wheel unloads, pressure collapses, flow
  re-establishes, and the cycle repeats at a rate set by the charge-pipe volume
  and the wheel inertia. That cycle is the *stu-tu-tu*, and it should emerge
  from the loop in TB3 rather than being oscillated by hand here.
- **Efficiency** is not decoration: it sets discharge temperature through

  ```text
  T_out = T_in (1 + (PR^((gamma-1)/gamma) - 1) / eta)
  ```

  and discharge temperature sets charge density and knock margin, both of which
  are already modelled downstream. A map without efficiency gives free power.
- Overspeed as a hard limit on the map, not a clamp buried in an update
  function.
- Provide two or three stock maps by frame size, parameterised enough that a
  preset can say "small single" or "large single" without typing a map.

**Tests.**

- Corrected quantities are invariant: the same physical point at two inlet
  temperatures lands on the same map location.
- Interpolation is monotone along a speed line and does not overshoot between
  them.
- A point left of the surge line is reported as surging; one right of choke is
  reported as choked; the boundaries are read off the map, not from constants.
- Discharge temperature follows the isentropic relation at the map's efficiency,
  and a lower efficiency gives a hotter charge at the same pressure ratio.
- Requesting a point outside the map fails loudly rather than extrapolating.

**Commits.**

```
add corrected speed and corrected flow
add a compressor map with speed lines
interpolate pressure ratio and efficiency
report surge and choke from the map boundaries
derive discharge temperature from map efficiency
add stock compressor maps by frame size
test corrected quantities are inlet invariant
```

---

# Stage TB2 — The turbine map and the shaft

**Goal.** Drive the shaft from exhaust enthalpy against a real inertia.

**Why.** Shaft speed is currently a first-order lag on a proxy, so lag is a
constant somebody typed rather than a consequence. Every turbo cliché — spool
threshold, the difference a downshift makes, why a big wheel is lazy and a small
one is frantic — is the power balance on a rotating inertia, and none of it can
appear until that balance is solved.

**Files.** New `src/physics/turbine.rs`; `src/physics/engine_block.rs`.

**Design.**

- Turbine map as reduced mass flow and efficiency against expansion ratio and
  corrected speed. Turbine maps are far flatter than compressor maps, so fewer
  points suffice, but keep the same representation so one interpolator serves
  both.
- Power balance on the shaft:

  ```text
  I dw/dt = (P_turbine * eta_mech - P_compressor) / w - P_bearing(w)
  ```

  with `I` the polar inertia of wheel, shaft and compressor together. **This is
  the lag.** Delete `spool_up` and `spool_down` once it runs; keeping a solved
  balance *and* hand-tuned constants guarantees they disagree later.
- Bearing drag as a real term, and let it differ by bearing type — a ball
  bearing cartridge spools measurably faster than a journal, which is a thing
  people buy turbos for.
- **The turbine is driven by a pulse train, not by mean flow.** Feed it the
  instantaneous manifold state, not a cycle average. Pulse energy recovery is
  why a small-displacement engine with a well-separated manifold spools sooner
  than its mean flow says it should, and it is the physical basis of twin-scroll
  in TB5.
- Turbine inlet temperature as a reported quantity with a limit, because it
  gates anti-lag and overboost realism later.

**Tests.**

- The shaft accelerates when turbine power exceeds compressor power plus drag,
  and not otherwise.
- A larger polar inertia lags a given step by longer, with no constant changed.
- A ball bearing spools faster than a journal at the same inertia.
- Pulse-fed turbine power exceeds mean-flow-fed power at the same average mass
  flow — the pulse energy is real and must not average away.
- Steady state matches a hand-computed power balance.
- Shaft speed is bounded by the map's overspeed limit and the limit is reported.

**Commits.**

```
add a turbine map with reduced flow and efficiency
add the turbocharger shaft power balance
add bearing drag by bearing type
feed the turbine the instantaneous manifold state
report turbine inlet temperature
test a larger wheel inertia lags longer
```

---

# Stage TB3 — Close the loop

**Goal.** Boost that the engine breathes and pays for.

**Why.** TB1 and TB2 are inert until the compressor's outlet is the intake's
upstream and the engine's exhaust is the turbine's inlet. This is the stage
where a turbocharged engine starts making more torque than an atmospheric one
for a physical reason.

**Files.** `src/physics/intake.rs`, `src/physics/engine_block.rs`,
`src/audio/mod.rs`, `src/bench/config.rs`.

**Design.**

- **The compressor becomes the plenum's upstream.** Replace
  `ambient_upstream(env, gas)` with the compressor's discharge `PortState`. As
  noted above, `ThrottleBody::mass_flow` already handles a manifold above its
  upstream, so reversion and blow-off both work through the existing path.
- **Charge pipe volume** between compressor and throttle. It is not cosmetic:
  it is the capacitance that sets how fast pressure collapses when the throttle
  shuts, which sets whether the compressor surges and at what rate. Model it as
  a plenum, the same type the intake already uses.
- **Intercooler** as a pressure drop and a heat exchanger between them.
  Effectiveness and core volume; the volume adds to the charge pipe's, which is
  why a big front-mount changes throttle response as well as temperature.
- **Move `Induction` into the physics.** It currently lives in `src/audio/mod.rs`
  because it described a sound. It now describes hardware, and the voicing
  should hang off it rather than contain it. Leave a re-export so the audio side
  keeps compiling in one commit.
- The exhaust side already has `back_pressure`; feed the turbine's actual
  operating point into it rather than the fixed `k` from TB0.
- Knock is already solved and voiced. Boost raises trapped mass and charge
  temperature, so knock should now appear on its own under high load — check
  that it does rather than adding anything.

**Tests.**

- Boost raises trapped mass, and torque rises with it.
- Boost raises exhaust back pressure, and the pumping penalty appears in the
  torque balance — boost is not free.
- Shutting the throttle at boost collapses charge pipe pressure at a rate set by
  its volume.
- A larger intercooler lowers charge temperature and raises the knock margin.
- An atmospheric preset is bit-identical to before this stage.
- Sustained high boost produces knock without any knock-specific change.

**Commits.**

```
feed compressor discharge to the intake plenum
add the charge pipe volume
add an intercooler pressure drop and heat exchange
move induction into the physics module
feed the turbine operating point to back pressure
test boost raises trapped mass and back pressure
```

---

# Stage TB4 — Boost control, the wastegate and the blow-off valve

**Goal.** Make the two valves devices instead of voices.

**Why.** `WastegateVoicing` and `BlowOffVoicing` are sounds with thresholds and
nothing behind them. Once boost is real they become the only things standing
between the engine and its own destruction, and their behaviour is most of what
distinguishes one setup from another.

**Files.** `src/physics/turbine.rs`, `src/physics/control.rs`,
`src/physics/intake.rs`, `src/bench/config.rs`.

**Design.**

- **Wastegate** as a bypass around the turbine with a real flow area, a spring
  preload and an actuator. Two fitments, and they sound entirely different:
  - **Internal**, dumping into the downpipe after the turbine, so its flow
    rejoins the main exhaust and is silenced with it.
  - **External**, optionally dumping to atmosphere through a screamer pipe —
    a *second radiating aperture*, unsilenced, upstream of everything. The
    propagation model already supports multiple apertures; this is one more.
- **Boost control** as a closed loop on the actuator: a target pressure ratio,
  a controller with real authority limits, and the failure modes that follow —
  **overshoot** on a fast tip-in, and **boost creep** when the gate cannot flow
  enough at high engine speed and boost rises whatever the controller wants.
  Both are behaviours people know by ear.
- **Blow-off valve** as a real vent from the charge pipe, in three fitments
  because they are three different sounds:
  - **Atmospheric** — vents outside, the classic release. Another aperture.
  - **Recirculating** — vents back ahead of the compressor, nearly silent, and
    the flow returns to the inlet so the compressor stays loaded.
  - **None** — the charge pipe has nowhere to go, the compressor is driven into
    surge, and the flutter is the result rather than an effect that was added.
  The third case must fall out of TB1's surge line and TB3's charge pipe volume.
  If it has to be synthesised, something upstream is wrong.
- Boost-by-gear and a driver-set target belong here, since M1 supplies the gear.

**Tests.**

- The wastegate holds the target pressure ratio in steady state.
- A fast tip-in overshoots the target and settles, and the overshoot grows with
  controller gain.
- Boost creep appears at high engine speed with an undersized gate and is absent
  with an adequate one.
- An external gate venting to atmosphere adds a radiating aperture whose signal
  is unsilenced and arrives before the tailpipe's.
- With no blow-off fitted, shutting the throttle at boost drives the compressor
  across the surge line, and the resulting oscillation has a period set by
  charge pipe volume and wheel inertia.
- A recirculating valve prevents surge and radiates almost nothing.

**Commits.**

```
add a wastegate bypass with spring preload
add internal and external wastegate fitments
add a screamer pipe aperture
add closed loop boost control
add boost creep from insufficient gate flow
add atmospheric and recirculating blow-off fitments
test no blow-off drives the compressor into surge
```

---

# Stage TB5 — Transients and architectures

**Goal.** The behaviours people actually describe when they talk about turbos.

**Why.** Everything to here is a steady-state machine that happens to have
inertia. What is left is the set of transient and layout behaviours that make
one turbocharged engine recognisably a different thing from another.

**Files.** `src/physics/turbine.rs`, `src/physics/plumbing.rs`,
`src/audio/waveguide.rs`, `src/physics/control.rs`, `src/bench/config.rs`.

**Design.**

- **Twin-scroll.** Divide the manifold so that cylinders whose exhaust events
  would otherwise interfere feed separate scrolls, and give the turbine two
  inlets. This is a real acoustic change as well as a spool change: the
  pre-turbine manifold becomes two shorter, better-separated resonators, which
  is audible. It needs the firing order, which the block already carries.
- **Sequential and compound.** A small turbo feeding a large one, or two turbos
  with a changeover. The changeover is a discrete event with its own transient
  and its own sound, and it is worth modelling because nothing else in the
  catalogue has a step change in induction character mid-pull.
- **Twin turbo, one per bank.** The catalogue already has `twin_turbo_v8`
  asserting this; make it true, with two shafts that can differ.
- **Variable geometry.** Moving nozzle vanes: effective A/R becomes a controlled
  variable rather than a fitting. This is the diesel case and the catalogue has a
  turbodiesel.
- **Anti-lag, properly.** It is already real in the ECU — retard, overrun
  fuelling, raised manifold temperature — and it already feeds the backfire
  voice. What it lacks is the consequence: combustion in the manifold drives the
  turbine directly, so the shaft should *hold speed at a shut throttle*, which
  is the entire point. Connect it, and connect its cost: turbine inlet
  temperature climbs, and the limit from TB2 should be reachable.
- **Heat soak.** The existing thermal model has lumped masses; the turbo is
  another one. A heat-soaked compressor makes hotter air for the same pressure
  ratio, which is why a car is slower on its third run.

**Tests.**

- Twin-scroll raises turbine power at the same mean flow, by more than a
  divided-manifold pressure change alone accounts for.
- Twin-scroll changes the pre-turbine manifold resonance to the divided length.
- A sequential changeover is a bounded transient with no discontinuity in shaft
  speed or torque.
- Two turbos on one V8 spool independently and can be given different maps.
- Closing VGT vanes spools sooner and raises back pressure.
- Anti-lag holds shaft speed at a shut throttle and raises turbine inlet
  temperature toward its limit.
- A heat-soaked turbo delivers a lower charge density at the same pressure
  ratio.

**Commits.**

```
divide the manifold for twin scroll turbines
add a two inlet turbine
add sequential turbo changeover
give each bank its own turbocharger
add variable turbine geometry
drive the turbine from anti-lag combustion
add turbocharger heat soak
test twin scroll recovers more pulse energy
```

---

# Stage TB6 — Voicing from real state

**Goal.** Retire every proxy.

**Why.** The voices work today off quantities that will by now be fiction:
`TurboVoicing::reference_rpm` against a shaft speed that is solved, a surge
heuristic against a map that has a surge line, a blow-off threshold against a
valve that exists. Rewiring them earlier would mean doing it twice, which is why
this is last but one.

**Files.** `src/audio/dsp.rs`, `src/audio/mod.rs`.

**Design.**

- Whistle pitch from solved shaft speed times a real blade count, not from an
  order chosen to sound right. Blade count is a compressor property and belongs
  on the map.
- Whistle level from compressor **work**, not from a load proxy: a compressor
  moving no air is silent however fast it turns.
- Surge flutter keyed to actual surge-line crossings, with its rate taken from
  the oscillation the loop produces rather than from `11.0 + 17.0 * surge`.
- Blow-off level and duration from vented mass and the pressure it vented from.
  A small vent and a big one are the same sound today.
- Wastegate chatter from actual actuator movement, which means it appears when
  the gate is hunting and not when it is open and stable.
- **Delete `TurboModel`.** It is the last proxy, and the rule in `AGENTS.md` is
  that a stage is not done until the thing it replaces is gone from the path.
- Re-level the whole induction group against the exhaust, once, with the
  [TIMBRE T0](TIMBRE_PLAN.md) metrics.

**Tests.**

- Whistle frequency equals shaft speed times blade count within a tone.
- A compressor at speed against a closed throttle is quieter than the same speed
  flowing air.
- Flutter rate matches the loop's measured surge period, not a constant.
- Vent duration scales with vented mass.
- `TurboModel` no longer exists anywhere in the crate.

**Commits.**

```
pitch the whistle from solved shaft speed and blade count
level the whistle from compressor work
key surge flutter to the map crossing
scale blow-off from vented mass
key wastegate chatter to actuator movement
delete the turbo model proxy
```

---

# Stage TB7 — Calibration and the catalogue

**Goal.** Make the five forced-induction presets true.

**Why.** `turbo_inline_4`, `turbo_inline_6`, `twin_turbo_v8`, `turbodiesel_i4`
and the two supercharged variants currently assert their character in a `note`
string and a whistle order. Every one of them needs a real map, a real wheel
inertia, a real A/R and a real boost target, and every recorded fingerprint will
move.

**Files.** `src/bench.rs`, `engines/*.toml`,
`docs/ENGINE_CONFIGURATION_GUIDE.md`, `docs/measurements/`.

**Design.**

- One frame size per preset, chosen so the engine's airflow sits in the middle
  of the map at its torque peak — which is what choosing a turbo *is*, and the
  documentation should say so.
- Document the whole `[induction]` section in the configuration guide with the
  same depth the exhaust section has, including a worked example: take the
  inline-four's displacement and volumetric efficiency, compute its airflow at
  the torque peak, and show which speed line that lands on.
- Add a `--sweep` mode to the acoustic bench over boost target and A/R, the way
  [MECHANISM M4](MECHANISM_PLAN.md) and [TIMBRE T4](TIMBRE_PLAN.md) do for their
  parameters.
- Re-record every fingerprint and say in the commit what moved.
- The superchargers are not turbos and are out of scope here, but they share the
  compressor map machinery — a centrifugal is a compressor on a belt. Note in
  the guide which parts apply.

**Tests.**

- Every preset's operating point at its torque peak lies inside its map, away
  from both boundaries.
- No preset can reach compressor overspeed at redline at its boost target.
- Each turbo preset's spool threshold is within a stated band of the rpm its
  note claims.
- The catalogue renders without a surge or overspeed event during a normal
  sweep.

**Commits.**

```
give each turbo preset a real compressor map
size wheel inertia and housing from the frame
set boost targets from the preset torque curves
add a boost and housing sweep to the acoustic bench
document the induction section in the configuration guide
re-record the catalogue fingerprints
```

---

## Not in this plan

- **Oil and water cooling circuits, coking, bearing failure.** Real, and
  inaudible.
- **Compressor and turbine blade aeroelasticity.** The blade-pass content in TB0
  is a modulation, not a structural model, and that is the right level here.
- **Electric turbochargers and e-assist.** Out of scope for now; the shaft power
  balance in TB2 is where they would attach if that changes.
- **Supercharger physics.** Roots and centrifugal voices exist and stay as they
  are. TB1's map machinery would serve a centrifugal directly, and TB7 notes it,
  but converting them is separate work.
