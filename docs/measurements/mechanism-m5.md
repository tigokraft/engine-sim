# Stage M5 — rotary, properly

Stage M5 of [the mechanism plan](../MECHANISM_PLAN.md): the eccentric shaft's
3:1 ratio in a type, four named rotary port profiles sharing Stage M4's ramp
machinery, the mechanical noise floor moved to rotor rate, and a two-plug
heat release. Per
[Definition of done, rule 7](../TIMBRE_PLAN.md#definition-of-done-every-stage),
this records the stage's effect on octave-band share and crest factor against
the pre-Stage-0 reference, whether or not it moved as intended.

## It did not move, except where it was supposed to

`no_preset_has_drifted_from_its_recorded_timbre` was run before this stage
(`57bc8c2`) and after it. Every preset's drift report is byte-identical
between the two runs **except 2-Rotor Wankel**, which is the only preset this
stage's commits touch. Nothing here moved a valve angle, a Wiebe, a pipe or a
mechanical order on any other engine.

## What the stage did move: the Wankel's own fingerprint

| band | before | after | delta |
|---|---:|---:|---:|
| 31.5 Hz | -16.5 | -22.9 | -6.4 |
| 63 Hz | -11.7 | -15.1 | -3.4 |
| 125 Hz | -10.5 | -11.1 | -0.6 |
| 250 Hz | -3.3 | -7.4 | -4.1 |
| 500 Hz | -15.6 | -4.1 | **+11.5** |
| 1000 Hz | -15.8 | -5.3 | **+10.5** |
| 2000 Hz | -19.7 | -18.6 | +1.1 |
| 4000 Hz | -29.4 | -23.7 | +5.7 |
| 8000 Hz | -47.8 | -35.9 | **+11.9** |
| 16000 Hz | -60.4 | -46.4 | **+14.0** |
| crest factor | 34.4 dB | 17.2 dB | **-17.2 dB** |

All figures dB, against the pre-Stage-0 fingerprint recorded in
[`timbre::RECORDED`](../../src/analysis/timbre.rs). This is the calibration
sweep, not idle — it is dominated by what changed in the mid and upper bands
and by crest factor collapsing 17 dB. Both are the expected shape of this
change: a port network with far more overlap and (for three of the four named
profiles) a faster edge fills in the gaps between blowdown events instead of
leaving them as short, high-crest pulses, and the two-plug burn adds a second,
smaller heat-release event that broadens the pressure trace the same way.
Splitting the two causes: recording the fingerprint after the port change
alone (before the two-plug commit) already reads -23.8 / -15.7 / -10.9 / -7.2
/ -4.2 / -5.3 / -18.6 / -23.7 / -35.9 / -46.5, crest 17.1 dB — the ports are
essentially the whole effect, and the two-plug burn commit moves the
fingerprint by a few tenths of a dB and 0.1 dB of crest on top of it.

## The port calibration grid

The plan's own prompt says to spend the stage on ports and to expect a
peripheral port to be hard to idle. What it does not say, and what this stage
found by rendering the block rather than by assuming, is that **duration and
opening rate interact multiplicatively on reversion mass**, and this
preset's block mass, inertia, load and idle target cannot survive both pushed
to their individual extremes at once — not just at idle, at wide-open
throttle too.

Measured by running a 20 s wide-open-throttle hold (limiter engaged, matching
`every_preset_can_reach_its_own_limiter`) and a closed-throttle idle hold (15
s to settle, 20-30 s recorded, matching
`a_big_overlap_cam_fails_to_settle_at_idle`'s method) over a grid of
`(duration, ramp_fraction)` pairs, holding lift, diameter and discharge
coefficient fixed at the two-rotor preset's own values:

| duration | ramp 0.50 | ramp 0.35 | ramp 0.28 | ramp 0.22 |
|---:|---:|---:|---:|---:|
| 230° | 8773 rpm | 8779 | 8774 | 8803 |
| 240° | — | — | — | 8789 |
| 250° | 8783 | 8785 | 8793 | 8809 |
| 260° | 8791 | 8799 | 8786 | 8810 |
| 270° | 8795 | 8801 | 8796 | **682** |
| 280° | 8786 | 8799 | 8790 | **400** |
| 300° | 8789 | 8788 | — | — |
| 330° | 8791 | **919** | — | — |

Figures are the rpm reached at the end of a 20 s wide-open-throttle hold with
the limiter engaged (redline 8800), a dash an untested cell; bold is a hold
that failed to sustain combustion at all. Separately, at `ramp = 0.20` and
`0.10` and `duration = 230°` the same hold reaches 8773 and **400**
respectively — steepening the ramp alone, with the overlap held at this
stage's mildest, crosses a cliff between `0.13` and `0.12`. The safe region is
a frontier, not a rectangle: a narrow ramp at minimal overlap survives past
`FASTEST_RAMP`, and a raised-cosine ramp survives past 300° of duration, but
nothing tested combined a duration past ~270° with a ramp much steeper than
~0.30 without collapsing to the block's stall floor before the hold finished.

The four named profiles were chosen from this grid, verified individually
against both the WOT hold and the idle hold, with margin rather than pinned
to the edge of the frontier:

| profile | duration | ramp | overlap | WOT (limiter) | free-rev (redline off, 25 s) | idle |
|---|---:|---:|---:|---:|---:|---|
| `side_port` | 230° | 0.50 | 30° | 8773 | 8756 | alive, 2.03 s period, 215 rpm p-p, settles near 935 |
| `bridge_port` | 240° | 0.35 | 40° | 8793 | 9345 | collapses to the 400 rpm stall floor |
| `half_bridge_port` | 250° | 0.28 | 50° | 8803 | 9748 | collapses to the 400 rpm stall floor |
| `peripheral_port` | 260° | 0.22 | 60° | 8793 | 9755 | collapses to the 400 rpm stall floor |

`peripheral_port` uses `0.22`, not `PERIPHERAL_PORT_RAMP`'s nominal floor of
`0.025` — that value is real, tested, and reachable through
`ValveEvent::with_port_ramp_fraction` (a port has no spring to survive, unlike
a poppet cam bound by `FASTEST_RAMP`), but no duration this preset's block
will sustain combustion at, at any throttle, pairs with it. Widening that
frontier is a driveline-and-breathing question — bigger primaries, a
different load curve, more inertia — not a port-geometry one, and it is out
of this stage's scope. It is recorded here rather than quietly worked around.

That three of the four named profiles fail to hold the idle governor at all,
rather than merely hunting the way Stage M4's big-overlap V8 cam did, is a
stronger result than the plan asked for, not a weaker one: a real bridge or
peripheral port genuinely does not idle. `side_port` alone holds a
recognisable, if rough, idle under the unmodified Stage M4 governor, which is
the comparison `a_peripheral_port_does_not_settle_at_the_idle_a_side_port_does`
checks.

## Mechanical voices at rotor rate

`MechanicalSpec::rotary`'s gear whine was `ImpulsiveSpec::order(3.0, 0.30)` —
three events per *eccentric shaft* revolution. The phasing gear meshes with
the rotor, not the shaft, so that was one order too fast by exactly the
factor the plan names: `RotorGeometry::shaft_order(3.0)` gives `1.0`, and the
gear whine now sits at rotor rate, a third of what it was. The accessory
drive is left alone — it is a belt on the shaft nose, genuinely at shaft rate
on a real rotary the same as on a reciprocating engine.

## Two-plug heat release

`TwoPlugCombustion` sums a leading and a trailing Wiebe, the trailing plug
delayed 12 degrees and weighted 0.35 against the leading plug's 0.65.
`removing_the_trailing_plug_measurably_changes_the_pressure_trace` records
the effect directly: on a synthetic test profile (20° BTDC leading spark, 60°
duration), zeroing `trailing_share` moves both the peak cylinder pressure and
the pressure trace itself by more than 10 kPa at every angle where the two
plugs' contributions do not coincide.

## Where the mechanism is weaker than it should be

Honest notes for whoever revisits this.

**A pre-existing continuity failure, not caused by this stage.** Running
`cargo run --release --example measure -- --preset "2-Rotor Wankel"` reports
`tip_in` and `limiter_bounce` as clipping (`peak` at or past 1.0) both before
and after this stage's changes — checked by running the same command against
`57bc8c2`, the commit immediately before Stage M5 started. Re-running the
full catalogue shows the same failure on eight other presets already:
Flat-plane V8, Turbo Inline-4, Big Single and all five GT3 cars. This is a
`master_gain` calibration question across a set of presets this stage never
touches, not a port or combustion regression, and fixing it is out of scope
here. `docs/measurements/README.md` and the per-preset measurement files were
found to be stale in the same pass — missing the GT3 presets entirely — and
were deliberately left alone rather than folded into this stage's diff.

**The duration/ramp frontier above.** A true peripheral port's blowdown edge
is closer to a step than `0.22` reproduces; this preset's reversion model
cannot currently sustain combustion at the duration a real peripheral port
would want alongside an edge that fast. The port machinery
(`ValveEvent::with_port_ramp_fraction`, `PERIPHERAL_PORT_RAMP`) is not the
limit — it was built to and tested at the full range — the two-rotor
preset's other parameters are.
