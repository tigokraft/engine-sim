# Stage M4 — cam character and the idle limit cycle

Stage M4 of [the mechanism plan](../MECHANISM_PLAN.md): cams specified by lobe
separation and advance rather than by two absolute angles, a profile
aggressiveness parameter, a real PI idle governor on an air bypass with
actuator lag, and charge dilution costing burn completeness. Per
[Definition of done, rule 7](../TIMBRE_PLAN.md#definition-of-done-every-stage),
this records the stage's effect on octave-band share and crest factor against
the pre-Stage-0 reference, whether or not it moved as intended.

## It did not move

Measured by running `no_preset_has_drifted_from_its_recorded_timbre` at the
commit before this stage (`91b23f5`) and at the end of it, and differencing the
two drift reports — both are stated against the same pre-Stage-0 fingerprints,
so the difference is this stage's own contribution.

| preset | 63 Hz | 2 kHz | 4 kHz | crest |
|---|---|---|---|---|
| 2-Rotor Wankel | +0.0 | +0.1 | +0.0 | +0.0 |
| Big Single | -0.1 | -0.1 | +0.0 | +0.1 |
| Cross-plane V8 | +0.0 | +0.0 | +0.0 | -0.1 |
| Flat-plane V8 | **+0.5** | -0.1 | +0.1 | +0.0 |
| Inline-4 | +0.0 | +0.0 | -0.1 | **+1.2** |
| Turbo Inline-4 | +0.0 | +0.0 | +0.0 | +0.5 |
| Turbo Inline-6 | +0.1 | +0.0 | +0.0 | -0.1 |
| Turbodiesel I4 | +0.0 | +0.0 | +0.0 | +0.0 |
| Twin-turbo V8 | +0.2 | +0.0 | -0.1 | -0.3 |
| V10 | +0.2 | +0.1 | +0.1 | +0.0 |
| V12 | **+0.5** | -0.3 | +0.0 | +0.0 |

All figures dB, after minus before. Nothing in the 500 Hz - 4 kHz region the
timbre plan is about moved by more than 0.3 dB on any preset, and the largest
movement anywhere is the Inline-4's crest factor, up 1.2 dB — in the direction
the timbre plan wants, and small enough to be a by-product rather than a fix.

That is the intended result rather than a disappointing one. The catalogue is
measured on a drive cycle, and this stage changes what an engine does at *idle*
with a cam none of the presets has. Three things were built to keep it that
way, and each has its own test:

- **The cam respecification is exactly neutral.** Every preset's absolute valve
  angles are re-derived from its own lobe separation and advance and come back
  within a nanodegree (`every_preset_derives_its_valve_angles_from_lobe_separation_and_advance`).
- **The default lift curve is bit-identical.** The two-flank profile at a half
  ramp fraction is the same raised cosine, to the last bit on the opening flank
  and to a few ulp on the closing one, where the angle is formed by reflection
  (`a_stock_ramp_reproduces_the_raised_cosine_it_replaced`).
- **A wide-open plate is unrestricted.** The bypass is in parallel with the
  plate and the sum is capped at the bore, so at full throttle the air path is
  the same unrestricted one every fingerprint was recorded against.

## What the stage did move: the idle

The new measurement, and the one the stage exists for. Cross-plane V8, idling
under its own governor with no throttle input, 15 s to settle then 120 s
recorded at a 480 Hz physics rate. The cam is the preset's own, re-ground with
`ValveTrain::with_cam_timing` to add duration at the stock 100 degree lobe
separation, so the extra duration lands as overlap and nothing else moves.

| overlap | mean rpm | peak-to-peak | limit cycle | governor's own figure |
|---|---|---|---|---|
| 40 deg (stock) | 750.0 | 5.0 rpm | none — settled | settled |
| 60 deg | 750.0 | 13.7 rpm | none — settled | settled |
| 80 deg | 750.0 | 35.4 rpm | 1.96 s (0.51 Hz) | 1.95 s, 18 rpm |
| 100 deg | 749.9 | 69.3 rpm | 2.04 s (0.49 Hz) | 1.92 s, 36 rpm |
| 120 deg | 749.9 | 118.3 rpm | ~2.3 s (0.44 Hz) | 2.26 s, 55 rpm |

"Limit cycle" is the dominant rate of the recorded speed trace, found by
evaluating the DFT over a 0.1 - 4 Hz grid; "the governor's own figure" is what
[`IdleHunt`](../../src/physics/control.rs) publishes to telemetry, which is a
Schmitt-triggered crossing count. The two agree at 80 and 100 degrees. At 120
the spectral estimate is unreliable — its two halves disagree, 3.63 s against
2.32 s, because the cycle there is deep enough to be strongly non-sinusoidal —
so the crossing count is the figure to quote.

**The measured limit-cycle period for the large-cam preset is 2.3 s, 0.44 Hz,
55 rpm peak to peak**, against a firing frequency of 50 Hz — a hundred and
thirteen times slower than the thing it modulates.

The governor's gains are identical across every row. Nothing was retuned for
the big cam; the same controller that settles a stock cam in a couple of
seconds cannot settle that one at all, which is the claim the stage was
written to make.

## Where the mechanism is weaker than it should be

Honest note for whoever does M5 or revisits this.

The loop closes through the governor's phase lag against a very steep
air-to-torque characteristic near idle, and through the cylinder's own residual
fraction costing burn completeness. The second of those is much weaker than it
should be. Trapped residual at a 120 degree overlap idle measures **6 %**,
where a real engine with that cam carries 20-30 %, because reversion out of the
intake valve is pushed into a plenum that reports itself as clean air and is
re-inhaled as fresh charge on the next stroke. The relevant comment in
`IntakePlenum::port_state` states this and has always been accurate.

Tracking the intake charge's composition was built and measured, and it is
recorded here because it very nearly works:

- It produces the mechanism properly. At a 120 degree overlap idle the residual
  reaches 20-50 %, the charge genuinely partial-burns, unburnt fuel at the port
  goes from 0.23 mg at a stock idle to 5.4 mg at a lopey one — a factor of 24 —
  and the speed swing grows from 24 to 776 rpm peak to peak.
- It is unaffordable as a single shared plenum. The lumped intake volume is
  common to every cylinder, so a contamination that should live in one runner
  for a few cycles at idle instead reaches the whole engine at every operating
  point. At wide-open throttle and 7000 rpm the same balance settles at 15 %
  burned gas in the manifold, which costs the Big Single all of its torque at
  4000 rpm and stops the Inline-4 reaching its limiter.

The fix is a per-cylinder runner control volume rather than the shared plenum:
its residence time is the runner's mass over the throughput, which is several
cycles at an idle drawing a gram a second and under half a cycle at full load,
so the contamination would persist exactly where it should and wash out exactly
where it should. That is more model than this stage needed and it is not in
here.

One consequence of leaving it out: **unburnt fuel at the port does not
discriminate a lopey idle from a stock one** in the shipped model, and the
plan's test bullet for it is not delivered. Two things stand in the way, and
both are recorded rather than worked around:

1. The residual is too small, as above — 6 % against a dilution limit of 35 %
   costs about a tenth of a percentage point of burn completeness.
2. The solver adds the Wiebe's fresh-charge burned fraction to a state that
   already carries the residual, so every cycle saturates at exactly 1 however
   badly it burned, and a cylinder that always reports a fully burned charge
   can never report unburnt fuel. Scaling the Wiebe by what is left to burn is
   a one-line correction and it is right, but it makes every engine report the
   0.67 % the `a = 5` convention leaves unburnt, which breaks the
   "a burnt charge should carry no fuel" invariant three tests in the backfire
   path assert exactly. Renegotiating that invariant belongs with Stage M3's
   code, not here, and there is nothing to gain from it until the residual is
   the right size.
