use super::*;
use crate::physics::engine_block::PHASE_CELLS;
use crate::physics::vehicle::Gear;

/// Runs a preset at a fixed speed until its phase ring has filled.
fn primed(preset: &EnginePreset, rpm: f64) -> EngineBlock {
    let mut block = preset.block(Environment::default());
    for _ in 0..900 {
        block.update(1.0 / 240.0, rpm);
    }
    block
}

#[test]
fn every_preset_is_physically_sane() {
    for preset in EnginePreset::catalogue() {
        let displacement = preset.displacement() * 1e3;
        assert!(
            (0.5..8.0).contains(&displacement),
            "{}: implausible displacement {displacement:.2} L",
            preset.name
        );
        assert!(
            (5_000.0..9_000.0).contains(&preset.redline),
            "{}: redline {} leaves the dashboard's 0-9000 needle no travel",
            preset.name,
            preset.redline
        );
        assert!(
            preset.idle > STALL_RPM,
            "{}: idles below stall",
            preset.name
        );
        // A rod shorter than the crank radius has no slider-crank solution;
        // `CylinderGeometry::new` clamps it, so check the clamp was never
        // needed rather than that it worked.
        assert!(
            preset.model.geometry.rod_ratio() > 1.5,
            "{}: rod ratio {:.2} is not a real engine",
            preset.name,
            preset.model.geometry.rod_ratio()
        );
    }
}

#[test]
fn every_preset_derives_its_valve_angles_from_lobe_separation_and_advance() {
    // The cam respecification has to be exactly neutral on the catalogue,
    // and this is the proof. For every preset, read the lobe separation
    // and the advance back off the absolute angles it ships, then rebuild
    // the absolute angles from those two figures alone: the train that
    // comes back must be the train that went in. Anything else means a
    // preset moved when the vocabulary changed, which is a silent retune
    // of an engine nobody asked to retune.
    //
    // The tolerance is the round trip through a mean and a half-difference
    // in binary floating point, not a modelling slop: a nanodegree.
    const TOLERANCE_DEG: f64 = 1e-9;
    for preset in EnginePreset::catalogue() {
        let original = preset.model.valves;
        let rebuilt = original.with_cam_timing(original.lobe_separation(), original.advance());
        for (name, before, after) in [
            ("intake", original.intake, rebuilt.intake),
            ("exhaust", original.exhaust, rebuilt.exhaust),
        ] {
            let moved = (after.open_angle - before.open_angle).to_degrees().abs();
            assert!(
                moved < TOLERANCE_DEG,
                "{}: {name} valve opening moved {moved:.3e} deg rebuilding it from \
                 LSA {:.4} deg and advance {:.4} deg",
                preset.name,
                original.lobe_separation().to_degrees(),
                original.advance().to_degrees(),
            );
            assert_eq!(
                before.duration, after.duration,
                "{}: {name} duration is the lobe's own and must not move",
                preset.name
            );
            assert_eq!(
                before.max_lift, after.max_lift,
                "{}: {name} lift is the lobe's own and must not move",
                preset.name
            );
        }

        // And the separation itself has to be a cam a grinder would cut.
        let lsa = original.lobe_separation().to_degrees();
        assert!(
            (95.0..=120.0).contains(&lsa),
            "{}: lobe separation {lsa:.1} deg is not a production cam",
            preset.name
        );
    }
}

#[test]
fn an_aggressive_ramp_raises_the_valvetrain_impulse_at_the_same_duration() {
    // The audible half of the profile parameter. Same cam card — same
    // duration, same lift, same timing — and a faster flank, so the valve
    // arrives at its seat harder and the seating impulse the mechanical
    // rig plays is louder for it. A stock cam must be left exactly alone,
    // because every recorded fingerprint has one.
    let preset = EnginePreset::cross_plane_v8();
    let stock_block = preset.block(Environment::default());
    let stock = preset.synth_config(&stock_block, 48_000.0);

    let mut roller_preset = preset.clone();
    roller_preset.model.valves = preset.model.valves.with_aggressiveness(1.0);
    assert_eq!(
        roller_preset.model.valves.intake.duration, preset.model.valves.intake.duration,
        "re-grinding the flanks must not change the duration"
    );
    let roller_block = roller_preset.block(Environment::default());
    let roller = roller_preset.synth_config(&roller_block, 48_000.0);

    for (name, before, after) in [
        (
            "intake",
            stock.mechanical.intake_valve,
            roller.mechanical.intake_valve,
        ),
        (
            "exhaust",
            stock.mechanical.exhaust_valve,
            roller.mechanical.exhaust_valve,
        ),
    ] {
        let before = before.expect("the V8 has valve voices").level;
        let after = after.expect("the V8 has valve voices").level;
        assert!(
            after > before * 3.0,
            "{name} valve impulse should follow the flank: {before:.3} to {after:.3}"
        );
    }

    // And the stock cam is untouched, to the bit.
    let reference = preset.mechanical.intake_valve.expect("V8 has valve voices");
    assert_eq!(
        stock
            .mechanical
            .intake_valve
            .expect("V8 has valve voices")
            .level,
        reference.level,
        "a raised-cosine cam must leave the mechanical levels exactly as they were"
    );
}

#[test]
fn a_tighter_lobe_separation_buys_overlap() {
    // The reason the vocabulary is worth having: overlap is not a free
    // parameter, it is what is left over when the two lobes are ground
    // closer together. Closing the separation by a camshaft degree opens
    // two crank degrees of overlap and nothing else changes.
    let train = crate::physics::thermodynamics::ValveTrain::default();
    let before = train.overlap().to_degrees();
    let tighter = train.with_cam_timing(train.lobe_separation() - deg(4.0), train.advance());
    let after = tighter.overlap().to_degrees();
    assert!(
        (after - before - 8.0).abs() < 1e-6,
        "four camshaft degrees tighter must buy eight crank degrees of \
         overlap: {before:.3} deg to {after:.3} deg"
    );
    assert_eq!(
        tighter.intake.duration, train.intake.duration,
        "re-timing a cam must not re-grind it"
    );
}

#[test]
fn overlap_raises_reversion_and_reversion_costs_burn_completeness() {
    // The first two links of the lope loop, measured rather than asserted:
    // a wider overlap traps more of last cycle's exhaust, and a charge
    // with more of last cycle's exhaust in it burns less of its fuel.
    // Same engine, same speed, same throttle — only the cam is re-ground,
    // and it is re-ground with the vocabulary this stage added.
    let dt = 1.0 / 480.0;
    let residual_at = |extra_duration: f64| -> (f64, f64) {
        let mut preset = EnginePreset::cross_plane_v8();
        let stock = preset.model.valves;
        let mut wider = stock;
        wider.intake.duration += extra_duration;
        wider.exhaust.duration += extra_duration;
        // Re-timed onto the stock separation and advance, so the extra
        // duration lands as overlap and nothing else moves.
        preset.model.valves = wider.with_cam_timing(stock.lobe_separation(), stock.advance());
        let mut block = preset.block(Environment::default());
        // A shut plate: an idle, which is the only place a manifold is far
        // enough below the exhaust for overlap to push gas the wrong way.
        block.throttle = 0.0;
        for _ in 0..(8.0 / dt) as usize {
            block.update(dt, 900.0);
        }
        let residual = block.master.latch.dilution;
        let wiebe = block
            .model
            .combustion
            .spark()
            .expect("the cross-plane V8 has spark plugs")
            .diluted(residual);
        (residual, 1.0 - (-wiebe.efficiency_parameter).exp())
    };

    // Thirty degrees, not sixty: past this the shut plate stops being the
    // only path between the two runners. At large enough overlap the two
    // valves are open together for long enough that exhaust pressure can
    // push straight through the cylinder and out the intake valve into
    // the plenum, which is a real effect (see `TURBO_PLAN.md` TB3's
    // reversion work) but a different one from what this test measures —
    // it re-pressurises the manifold past the shut throttle altogether
    // and, past around forty-five degrees, actually erodes the residual
    // *fraction* by flooding the cylinder with more trapped mass overall.
    let (stock_residual, stock_completeness) = residual_at(0.0);
    let (lopey_residual, lopey_completeness) = residual_at(deg(30.0));
    assert!(
        lopey_residual > stock_residual * 1.05,
        "thirty degrees more overlap must trap meaningfully more residual: \
         {stock_residual:.4} to {lopey_residual:.4}"
    );
    assert!(
        lopey_completeness < stock_completeness,
        "more residual must cost burn completeness: {stock_completeness:.5} \
         to {lopey_completeness:.5}"
    );
    assert!(
        stock_residual > 0.0,
        "a real engine always traps some residual"
    );
}

#[test]
fn every_preset_makes_torque_and_stays_finite() {
    for preset in EnginePreset::catalogue() {
        let block = primed(&preset, 4_000.0);
        let torque = block.mean_brake_torque(4_000.0);
        assert!(
            torque.is_finite() && torque > 0.0,
            "{} makes no torque at 4000 rpm: {torque:.1} N m",
            preset.name
        );
        let peak = block.ring.peak_pressure();
        assert!(
            (5e5..3e7).contains(&peak),
            "{}: implausible peak pressure {:.1} bar",
            preset.name,
            peak / 1e5
        );
    }
}

#[test]
fn every_preset_free_revs_past_its_own_limiter() {
    // Reaching the limiter is not enough: the engine has to be able to
    // *overshoot* it, or it hovers just underneath, the cut chatters on and
    // off, and the limiter bounce — with the backfires that come with it —
    // never really happens. This measures the free-revving ceiling with the
    // limiter taken out, which is the headroom the brake load leaves.
    for preset in EnginePreset::catalogue() {
        let mut block = preset.block(Environment::default());
        let mut driveline = Driveline::new(&preset);
        driveline.redline = f64::INFINITY;
        // The block's own limiter has to come out as well as the
        // driveline's. On a spark-cut engine it makes no difference to the
        // torque the block solves, but a fuel cut really does stop the
        // combustion — so leaving it in would measure the limiter rather
        // than the brake load, on exactly the engines that have one.
        block.ecu.redline = f64::INFINITY;
        driveline.throttle_target = 1.0;
        let dt = 1.0 / 240.0;

        // A single cylinder's torque impulse is a large fraction of what
        // the flywheel sees each firing event, so the free-revving ceiling
        // is not a fixed point, it is a shallow limit cycle the flywheel
        // settles into — and a single instantaneous sample can land on
        // either side of that cycle for a while after the throttle first
        // snaps open. 60 seconds and a 5-second trailing average is enough
        // margin for the slowest of these (a single, with its
        // proportionally huge flywheel) to have settled into its own
        // cycle and be measured across it rather than at one phase of it.
        let mut tail = std::collections::VecDeque::with_capacity(240 * 5);
        for _ in 0..(240 * 60) {
            driveline.update(&mut block, dt);
            block.update(dt, driveline.rpm);
            if tail.len() == tail.capacity() {
                tail.pop_front();
            }
            tail.push_back(driveline.rpm);
        }
        let settled_rpm = tail.iter().sum::<f64>() / tail.len() as f64;
        let margin = settled_rpm - preset.redline;
        assert!(
            margin > 150.0,
            "{} tops out at {settled_rpm:.0} rpm, only {margin:.0} over its \
             {:.0} rpm limiter — too little to bounce off it",
            preset.name,
            preset.redline,
        );
    }
}

#[test]
fn neutral_reproduces_the_free_revving_flywheel_to_integration_error() {
    // Gearbox, road load and clutch exist now, but a gear has to be
    // selected to reach any of them. In neutral, `update_in_gear` is never
    // called, and the flywheel formula below is a literal copy of what
    // `Driveline::update`'s `FreeRev` arm does — so this proves free
    // revving is still the plain flywheel against its own drag curve and
    // has not quietly acquired a driveline.
    //
    // Two things moved under it since Stage M1 and both are read off the
    // driveline rather than recomputed here, because neither is what this
    // test is about: the governor, which is a PI controller on an air
    // bypass now rather than a proportional term on the pedal, and the
    // pedal itself, which swings the block's throttle plate instead of
    // multiplying its torque afterwards. See the comment in the `None`
    // arm.
    let preset = EnginePreset::cross_plane_v8();
    let mut block = preset.block(Environment::default());
    let mut driveline = Driveline::new(&preset);
    assert_eq!(driveline.gearbox.gear, Gear::Neutral);
    driveline.throttle_target = 0.6;
    let dt = 1.0 / 240.0;

    // An independent reference flywheel, stepped by the same formula and
    // nothing else — no gearbox, no road load, no clutch.
    let mut reference_rpm = driveline.rpm;
    let mut reference_throttle = driveline.throttle;

    for _ in 0..(240 * 8) {
        let reference_block = block.clone();
        // The bypass the governor is about to command, taken before the
        // step so the reference sees what the driveline saw.
        let mut reference_governor = driveline.idle_governor;
        let target = driveline.idle_target(&reference_block);
        let bypass = reference_governor.update(target, reference_rpm, dt);

        driveline.update(&mut block, dt);
        block.update(dt, driveline.rpm);

        let slew = 1.0 - (-dt / 0.12_f64).exp();
        reference_throttle += (driveline.throttle_target - reference_throttle) * slew;
        let effective = reference_throttle.max(bypass * IDLE_BYPASS_PEDAL_AUTHORITY);

        let torque = reference_block.mean_brake_torque(reference_rpm);
        let drive = if driveline.manual_cut
            || reference_block.ecu.active_cut != LimiterCut::None
            || (reference_rpm >= driveline.redline
                && reference_block.ecu.limiter_cut_type != LimiterCut::None)
        {
            0.0
        } else {
            torque
        };
        let (a, b, c) = driveline.load;
        let omega = reference_rpm * PI / 30.0;
        let load = a + b * omega + c * omega * omega + (1.0 - effective) * driveline.pumping;
        let alpha = (drive - load) / driveline.inertia.max(1e-3);
        let omega = (omega + alpha * dt).max(STALL_RPM * PI / 30.0);
        reference_rpm = omega * 30.0 / PI;

        assert!(
            (driveline.rpm - reference_rpm).abs() < 1e-9,
            "neutral drifted from the free-revving flywheel: {} vs reference {}",
            driveline.rpm,
            reference_rpm
        );
    }
}

#[test]
fn every_preset_can_reach_its_own_limiter() {
    // The brake load is tuned per preset, and getting it wrong is silent:
    // the engine simply plateaus below the redline and the limiter, the
    // backfires and the whole top end of the dashboard become unreachable.
    for preset in EnginePreset::catalogue() {
        let mut block = preset.block(Environment::default());
        let mut driveline = Driveline::new(&preset);
        driveline.throttle_target = 1.0;
        let dt = 1.0 / 240.0;

        let mut reached = false;
        for _ in 0..(240 * 20) {
            driveline.update(&mut block, dt);
            block.update(dt, driveline.rpm);
            reached |= driveline.on_the_limiter();
        }
        assert!(
            reached,
            "{} never reached its {:.0} rpm limiter (stalled at {:.0})",
            preset.name, preset.redline, driveline.rpm
        );
    }
}

#[test]
fn muffled_and_straight_pipe_renders_differ_for_every_silenced_preset() {
    // The test that would have caught it: two of the catalogue's presets
    // shipped their exhaust cutout permanently open, so `--exhaust muffled`
    // and `--exhaust straight-pipe` rendered byte-identical WAVs. Presets
    // whose silencer chain is `Silencer::Straight` are excluded on
    // purpose — for those, muffled and straight-pipe are the same mode by
    // construction, which is a separate, already-documented fact, not this
    // bug.
    use crate::analysis::render::RenderPlan;
    use crate::analysis::script::{RenderScript, Segment};

    for preset in EnginePreset::catalogue() {
        let has_real_silencer = preset
            .exhaust
            .silencers
            .iter()
            .any(|s| !matches!(s, Silencer::Straight));
        if !has_real_silencer {
            continue;
        }

        let mut straight = preset.clone();
        straight.exhaust = straight.exhaust.into_straight_pipe();

        let script = RenderScript::new(
            "t5_muffled_vs_straight",
            "short idle hold to catch a bypassed silencer chain",
            vec![Segment::hold(0.5, preset.idle, 0.15)],
        );

        let muffled = RenderPlan::new(&preset, &script).render();
        let unbaffled = RenderPlan::new(&straight, &script).render();

        assert_ne!(
            muffled.samples, unbaffled.samples,
            "{}: muffled and straight-pipe render identically",
            preset.name
        );
    }
}

#[test]
fn the_four_port_profiles_are_distinguishable_in_the_octave_band_table() {
    // Stage M5's other claim: it is the port, not a fudge factor, that a
    // listener can tell apart. Rendered at a speed every named profile
    // actually sustains — idle is where the peripheral port is *supposed*
    // to fail, which is a different test — the four render distinct
    // audio, and the fast-opening end of the range carries more
    // high-frequency content than the gentle end.
    use crate::analysis::orders::octave_bands;
    use crate::analysis::render::RenderPlan;
    use crate::analysis::script::{RenderScript, Segment};

    let profiles: [(&str, ValveTrain); 4] = [
        ("side", crate::physics::rotor::side_port()),
        ("bridge", crate::physics::rotor::bridge_port()),
        ("half_bridge", crate::physics::rotor::half_bridge_port()),
        ("peripheral", crate::physics::rotor::peripheral_port()),
    ];

    let mut renders: Vec<(&str, Vec<f32>, [f64; 10])> = Vec::new();
    for (name, valves) in profiles {
        let mut preset = EnginePreset::two_rotor_wankel();
        preset.model.valves = valves;
        let script = RenderScript::new(
            "t_m5_port_profiles",
            "a steady hold well above idle, where every named profile runs",
            vec![Segment::hold(1.0, 4_500.0, 1.0)],
        );
        let render = RenderPlan::new(&preset, &script).render();
        let mono = render.mono();
        let table = octave_bands(&mono, render.sample_rate as f64);
        renders.push((name, mono, table));
    }

    for i in 0..renders.len() {
        for j in (i + 1)..renders.len() {
            assert_ne!(
                renders[i].1, renders[j].1,
                "{} and {} render identically",
                renders[i].0, renders[j].0
            );
        }
    }

    // The top two octave bands (8 kHz, 16 kHz) are where a sharper
    // blowdown edge shows up first; the peripheral port's edge is the
    // fastest of the four (see `physics::rotor::tests::\
    // peripheral_port_opens_faster_than_side_port`) and should read
    // higher there than the side port's.
    // The 2 kHz band is where the ordering shows most cleanly: a
    // sharper blowdown edge puts more energy up there, and it rises
    // monotonically side < bridge < half-bridge < peripheral, tracking
    // the opening-rate ordering measured in
    // `physics::rotor::tests::peripheral_port_opens_faster_than_side_port`.
    const BAND_2K: usize = 6;
    let band_2k: Vec<f64> = renders.iter().map(|(_, _, t)| t[BAND_2K]).collect();
    assert!(
        band_2k.windows(2).all(|w| w[0] < w[1]),
        "the 2 kHz band must rise side < bridge < half-bridge < peripheral: {:?}",
        renders
            .iter()
            .map(|(n, _, t)| (*n, t[BAND_2K]))
            .collect::<Vec<_>>()
    );
    assert!(
        band_2k[3] > band_2k[0] + 2.0,
        "the peripheral port must carry measurably more 2 kHz content than the side port: \
         {:.1} dB against {:.1} dB",
        band_2k[3],
        band_2k[0]
    );
}

#[test]
fn find_by_name_fails_loudly_on_an_unknown_preset() {
    assert!(EnginePreset::find_by_name("not-a-real-engine").is_none());
    assert!(EnginePreset::find_by_name("cross-plane v8").is_some());
    // Case-insensitive, and a substring is enough — the same rule
    // `examples/measure.rs` filters the catalogue with.
    assert_eq!(
        EnginePreset::find_by_name("INLINE-4").map(|p| p.name),
        Some("Inline-4")
    );
}

#[test]
fn the_starter_releases_after_the_engine_catches_and_its_whine_stops() {
    // Catch first, release second, and in that order for every engine in
    // the catalogue. The engine is not started by the starter letting go —
    // it fires under the motor, outruns it, and the motor notices.
    for preset in EnginePreset::catalogue() {
        let mut block = preset.block(Environment::default());
        block.cold_start();
        let mut driveline = Driveline::cranking(&preset, &mut block);
        let dt = 1.0 / 480.0;

        assert!(
            driveline.starter.whine_hz(driveline.rpm.max(200.0)) > 0.0,
            "{} was not whining with the pinion in",
            preset.name
        );

        let mut released_at = None;
        for frame in 0..(480 * 8) {
            let meshed = driveline.starter.engaged;
            driveline.update(&mut block, dt);
            block.update(dt, driveline.rpm);
            if meshed && !driveline.starter.engaged {
                released_at = Some((frame as f64 * dt, driveline.rpm));
            }
        }

        let (released, release_rpm) = released_at
            .unwrap_or_else(|| panic!("{} never caught: {:.0} rpm", preset.name, driveline.rpm));
        assert!(
            released > 0.0 && released < 5.0,
            "{} took {released:.1} s to catch",
            preset.name
        );
        assert!(
            release_rpm > driveline.starter.free_speed,
            "{} let go at {release_rpm:.0} rpm without outrunning the pinion",
            preset.name
        );
        // An engine with a loping or brapping port swings across a wide
        // limit cycle; 0.70 covers the trough of a peripheral port brap
        // and an aggressive race cam chop while still ensuring the engine
        // is running rather than dying back to the 400 rpm stall floor.
        assert!(
            driveline.rpm > preset.idle * 0.70,
            "{} caught and then died back to {:.0} rpm",
            preset.name,
            driveline.rpm
        );

        // And the whine is gone, at any speed, because the pinion is out of
        // mesh rather than faded down.
        assert_eq!(
            driveline.starter.whine_hz(driveline.rpm),
            0.0,
            "{} is still whining after the pinion came out",
            preset.name
        );
        assert_eq!(driveline.starter.torque(driveline.rpm), 0.0);
    }
}

/// Cranks a preset that cannot fire, and returns the speed trace [rev/min].
///
/// A fuel-cut limiter pinned at one rev/min is an engine being turned over
/// with nothing to burn — a compression test, or a car with the pump relay
/// out. It is the only way to listen to cranking on its own, because an
/// engine that can start does so inside a second and is then a different
/// thing entirely.
#[cfg(test)]
fn crank_without_fuel(preset: &EnginePreset, seconds: f64) -> Vec<f64> {
    let mut block = preset.block(Environment::default());
    block.cold_start();
    let mut driveline = Driveline::cranking(preset, &mut block);
    block.ecu.limiter_mode = LimiterMode::HardCut;
    block.ecu.limiter_cut_type = LimiterCut::Fuel;
    block.ecu.redline = 1.0;

    let dt = 1.0 / 480.0;
    let mut trace = Vec::new();
    for _ in 0..(seconds / dt) as usize {
        driveline.update(&mut block, dt);
        block.update(dt, driveline.rpm);
        trace.push(driveline.rpm);
    }
    assert!(
        driveline.starter.engaged,
        "{} caught with its fuel cut off",
        preset.name
    );
    trace
}

#[test]
fn cranking_speed_ripples_on_compression_and_never_settles() {
    // The stage's whole claim, and the reason none of it is synthesised.
    // Nothing here has a period in it: the starter has one torque curve with
    // no time in it, and the ripple is the engine's own compression strokes
    // arriving underneath it. If this ever converges to a constant, then
    // something has started handing the crank a torque averaged over a
    // cycle — which is exactly the number that has nothing to say here.
    for preset in EnginePreset::catalogue() {
        let trace = crank_without_fuel(&preset, 3.0);
        // The first second is the spin-up from rest, where the speed is
        // still climbing; the ripple under test is what the crank does once
        // the motor and the drag have found each other.
        let tail = &trace[480..];
        let mean = tail.iter().sum::<f64>() / tail.len() as f64;
        let lo = tail.iter().cloned().fold(f64::INFINITY, f64::min);
        let hi = tail.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        assert!(
            lo > 1.0,
            "{} stalled the starter on its own compression",
            preset.name
        );
        assert!(
            hi - lo > 0.04 * mean,
            "{} cranked at a flat {mean:.0} rpm: {lo:.0}..{hi:.0}",
            preset.name
        );

        // And it is a ripple, not a drift: the speed crosses its own mean
        // over and over, which a curve settling onto a constant does not do.
        let crossings = tail
            .windows(2)
            .filter(|w| (w[0] - mean) * (w[1] - mean) < 0.0)
            .count();
        assert!(
            crossings >= 8,
            "{} crossed its own mean only {crossings} times in two seconds, \
             which is a drift and not a chug",
            preset.name
        );

        // The chug is one compression stroke, so its rate has to be the
        // engine's own event rate rather than any number written down: half
        // a crank order per cylinder, at whatever speed the crank reached.
        let expected_hz = mean / 60.0 * preset.firing.len() as f64 * 0.5;
        let measured_hz = crossings as f64 / 2.0 / 2.0;
        assert!(
            measured_hz > 0.25 * expected_hz && measured_hz < 2.5 * expected_hz,
            "{} chugged at {measured_hz:.1} Hz on {} cylinders at {mean:.0} rpm, \
             nowhere near the {expected_hz:.1} Hz its compressions arrive at",
            preset.name,
            preset.firing.len()
        );
    }
}

#[test]
fn the_chug_speeds_up_as_the_oil_thins() {
    // The test that says the chug is emergent rather than drawn. Nothing
    // about the starter changes between these two runs and nothing about
    // the compression does either — only the oil the bearings are shearing,
    // which is the friction the same motor is working against. A shaped
    // envelope could not do this without being told to.
    let preset = EnginePreset::inline_four();

    let crank_at = |soaked: bool| -> f64 {
        let mut block = preset.block(Environment::default());
        if !soaked {
            block.cold_start();
        }
        let mut driveline = Driveline::cranking(&preset, &mut block);
        block.ecu.limiter_mode = LimiterMode::HardCut;
        block.ecu.limiter_cut_type = LimiterCut::Fuel;
        block.ecu.redline = 1.0;
        let dt = 1.0 / 480.0;
        let mut trace = Vec::new();
        for _ in 0..(480 * 3) {
            driveline.update(&mut block, dt);
            block.update(dt, driveline.rpm);
            trace.push(driveline.rpm);
        }
        let tail = &trace[480..];
        tail.iter().sum::<f64>() / tail.len() as f64
    };

    let stone_cold = crank_at(false);
    let warm = crank_at(true);
    assert!(
        warm > stone_cold * 1.02,
        "the same starter cranked a warm engine at {warm:.0} rpm and a cold one \
         at {stone_cold:.0}, so the oil is not reaching the crank"
    );
}

#[test]
fn a_cold_start_puts_unburnt_fuel_in_the_exhaust_and_a_warm_idle_does_not() {
    // The cold engine's pops, arriving at the exhaust the way the backfire
    // voice already reads them: not a new event type, just fuel that did
    // not burn. Nothing schedules a misfire — the port wall takes the first
    // injections, the charge that reaches the cylinder is past the lean
    // limit, and the flame does not propagate.
    use crate::audio::{EngineControls, SnapshotSource};

    let preset = EnginePreset::inline_four();

    let mut block = preset.block(Environment::default());
    block.cold_start();
    let mut driveline = Driveline::cranking(&preset, &mut block);
    let mut source = SnapshotSource::new(&block);
    let dt = 1.0 / 480.0;
    let mut misfired = 0usize;
    let mut cold_unburnt = 0.0f32;
    for _ in 0..(480 * 3) {
        driveline.update(&mut block, dt);
        block.update(dt, driveline.rpm);
        let snapshot = source.sample(&block, driveline.rpm, dt, EngineControls::default());
        if block.ecu.misfiring {
            misfired += 1;
        }
        cold_unburnt = cold_unburnt.max(snapshot.unburnt_fuel_mass);
    }
    assert!(
        misfired > 0,
        "a stone-cold start lit every charge it was given"
    );
    assert!(
        cold_unburnt > 1.0e-6,
        "a cold start sent {cold_unburnt:.3e} kg of fuel out unburnt"
    );

    // The same engine warm, idling: the port is dry, the mixture is what
    // the schedule asked for, and nothing goes out unlit.
    let mut block = preset.block(Environment::default());
    let mut driveline = Driveline::new(&preset);
    // A block constructed this instant has an unprimed ring: the first
    // torque figures it reports are not yet real combustion data, and the
    // flywheel can flare well past idle chasing them before the governor
    // and the ring both settle — for this engine, briefly past the DFCO
    // threshold, which is a real ECU behaviour and not a bug, but it is
    // not the "warm idle" this half of the test means to measure. Let it
    // settle first, the same way every other idle measurement in this
    // file does.
    for _ in 0..(480 * 2) {
        driveline.update(&mut block, dt);
        block.update(dt, driveline.rpm);
    }
    let mut source = SnapshotSource::new(&block);
    let mut warm_unburnt = 0.0f32;
    for _ in 0..(480 * 3) {
        driveline.update(&mut block, dt);
        block.update(dt, driveline.rpm);
        let snapshot = source.sample(&block, driveline.rpm, dt, EngineControls::default());
        assert!(!block.ecu.misfiring, "a warm idle misfired");
        // Not asserted bit-exact here, and deliberately: a block seeded
        // exactly on its thermostat dips a few nanokelvin under it on the
        // first frames, before the ring holds a cycle for the chamber heat
        // to come out of, so `cold_fraction` is a billionth rather than a
        // zero. The exactness guarantee is the one in
        // `a_warm_port_has_no_film_and_delivers_exactly_what_was_metered`,
        // which is about the law itself; this is about the engine.
        assert!(
            (block.ecu.fuel_delivery - 1.0).abs() < 1e-6,
            "a warm port held back {:.9} of what was metered",
            1.0 - block.ecu.fuel_delivery
        );
        warm_unburnt = warm_unburnt.max(snapshot.unburnt_fuel_mass);
    }
    assert!(
        warm_unburnt < 0.01 * cold_unburnt,
        "a warm idle put {warm_unburnt:.3e} kg out against the cold start's \
         {cold_unburnt:.3e} kg"
    );
}

#[test]
fn the_idle_target_falls_from_its_cold_value_as_the_block_takes_heat() {
    // Not a timer. The governor is holding a speed that is a function of
    // block temperature, so it comes down exactly as fast as the engine
    // warms and no faster.
    let preset = EnginePreset::inline_four();
    let mut block = preset.block(Environment::default());
    block.cold_start();
    let driveline = Driveline::cranking(&preset, &mut block);

    let stone_cold = driveline.idle_target(&block);
    assert!(
        stone_cold > preset.idle * 1.4,
        "a stone-cold engine was only given {stone_cold:.0} rpm against a warm {:.0}",
        preset.idle
    );

    let mut driveline = driveline;
    let dt = 1.0 / 480.0;
    let mut targets = vec![stone_cold];
    let mut temperatures = vec![block.thermal.block_temperature()];
    for step in 0..(480 * 120) {
        driveline.update(&mut block, dt);
        block.update(dt, driveline.rpm);
        if step % (480 * 10) == 0 {
            targets.push(driveline.idle_target(&block));
            temperatures.push(block.thermal.block_temperature());
        }
    }

    // Monotone down, and monotone up behind it: one is the cause of the
    // other, which is what "not on a timer" means.
    for pair in targets.windows(2) {
        assert!(
            pair[1] <= pair[0] + 1.0,
            "the idle target went back up: {:.0} then {:.0}",
            pair[0],
            pair[1]
        );
    }
    for pair in temperatures.windows(2) {
        assert!(
            pair[1] >= pair[0] - 0.1,
            "the block cooled while it was being run: {:.1} then {:.1} K",
            pair[0],
            pair[1]
        );
    }
    let settled = *targets.last().unwrap();
    assert!(
        settled < stone_cold,
        "two minutes of running left the idle target at {settled:.0} rpm, \
         where it started"
    );
}

#[test]
fn a_momentary_overrun_does_not_throw_the_pinion_out() {
    // What the release hold is for. A single-cylinder engine spends two
    // revolutions accelerating into nothing between compressions and passes
    // the motor's free speed doing it, having fired nothing at all.
    let mut starter = Starter::for_engine(0.7e-3, 200.0, 0.10);
    starter.engage();
    let dt = 1.0 / 480.0;

    // A tenth of a second over the top, then back under it.
    for _ in 0..48 {
        starter.update(starter.free_speed + 50.0, dt);
    }
    assert!(starter.engaged, "let go of a coast between compressions");
    starter.update(starter.free_speed - 50.0, dt);
    assert_eq!(starter.overrun_time, 0.0, "the hold did not reset");

    // Sustained, which is an engine that is running.
    for _ in 0..480 {
        starter.update(starter.free_speed + 50.0, dt);
    }
    assert!(!starter.engaged, "held on to an engine that was running");
}

#[test]
fn a_starter_makes_less_torque_the_faster_it_is_turning() {
    // The one property the chug depends on: a motor that gives back more
    // torque the more it is slowed cannot be stopped by a compression
    // stroke, only delayed by one.
    let mut starter = Starter::for_engine(2.0e-3, 150.0, 0.22);
    starter.engage();
    assert!(starter.torque(0.0) > starter.torque(200.0));
    assert!(starter.torque(200.0) > starter.torque(400.0));
    assert_eq!(starter.torque(starter.free_speed), 0.0);
    assert_eq!(starter.torque(starter.free_speed * 2.0), 0.0);

    starter.engaged = false;
    assert_eq!(starter.torque(0.0), 0.0, "a pinion that is out made torque");
}

#[test]
fn a_starter_is_sized_against_the_compression_it_has_to_cross() {
    // Both halves of the rule, each shown binding on its own. A big flywheel
    // swallows the hump and leaves the drag term in charge; a light one
    // leaves the motor to find the whole of it.
    use crate::physics::control::STARTER_TORQUE_PER_LITRE;

    let heavy = Starter::for_engine(2.0e-3, 150.0, 2.0);
    assert_eq!(
        heavy.stall_torque,
        STARTER_TORQUE_PER_LITRE * 2.0,
        "a flywheel that swallows the hump should leave the drag term in charge"
    );

    let light = Starter::for_engine(0.7e-3, 400.0, 0.02);
    assert!(
        light.stall_torque > STARTER_TORQUE_PER_LITRE * 0.7,
        "a big single on a light flywheel was sized on drag alone"
    );
    assert!(
        light.stall_torque > 400.0,
        "sized at {:.0} N m against a 400 N m compression it has to cross",
        light.stall_torque
    );
}

#[test]
fn a_shut_throttle_falls_back_to_idle_without_stalling() {
    let preset = EnginePreset::cross_plane_v8();
    let mut block = preset.block(Environment::default());
    let mut driveline = Driveline::new(&preset);
    let dt = 1.0 / 240.0;

    driveline.throttle_target = 1.0;
    for _ in 0..(240 * 6) {
        driveline.update(&mut block, dt);
        block.update(dt, driveline.rpm);
    }
    assert!(driveline.rpm > 3_000.0, "never pulled away from idle");

    driveline.throttle_target = 0.0;
    for _ in 0..(240 * 15) {
        driveline.update(&mut block, dt);
        block.update(dt, driveline.rpm);
    }
    assert!(
        driveline.rpm > STALL_RPM,
        "the governor let it stall at {:.0} rpm",
        driveline.rpm
    );
    assert!(
        driveline.rpm < preset.idle + 400.0,
        "never came back down to idle: {:.0} rpm",
        driveline.rpm
    );
}

#[test]
fn a_big_overlap_cam_fails_to_settle_at_idle() {
    // The stage's claim, and it is a claim about a *limit cycle*, not
    // about roughness. Same engine, same idle target, same governor with
    // the same gains — a governor is calibrated on the engine it ships
    // with and is not retuned for a cam it was never given. Only the
    // lobes are re-ground, and they are re-ground with the vocabulary
    // this stage added: eighty degrees more duration a side at the stock
    // lobe separation, which lands as overlap and nothing else.
    let dt = 1.0 / 480.0;
    let idle = |extra_duration: f64, seconds: f64| -> (Vec<f64>, IdleHunt, f64) {
        let mut preset = EnginePreset::cross_plane_v8();
        let stock = preset.model.valves;
        let mut wider = stock;
        wider.intake.duration += extra_duration;
        wider.exhaust.duration += extra_duration;
        preset.model.valves = wider.with_cam_timing(stock.lobe_separation(), stock.advance());
        let mut block = preset.block(Environment::default());
        let mut driveline = Driveline::new(&preset);
        // Long enough for the governor to find the engine before anything
        // is measured; a start-up transient is not a limit cycle.
        let settle = (15.0 / dt) as usize;
        let mut rpms = Vec::with_capacity((seconds / dt) as usize);
        for i in 0..(settle + (seconds / dt) as usize) {
            driveline.update(&mut block, dt);
            block.update(dt, driveline.rpm);
            if i >= settle {
                rpms.push(driveline.rpm);
            }
        }
        assert_eq!(
            driveline.throttle_target, 0.0,
            "the test drove the throttle instead of letting the governor do it"
        );
        let overlap = preset.model.valves.overlap().to_degrees();
        (rpms, driveline.idle_hunt, overlap)
    };

    let swing = |rpms: &[f64]| -> f64 {
        let hi = rpms.iter().cloned().fold(f64::MIN, f64::max);
        let lo = rpms.iter().cloned().fold(f64::MAX, f64::min);
        hi - lo
    };
    let mean = |rpms: &[f64]| rpms.iter().sum::<f64>() / rpms.len() as f64;

    let (stock_rpms, stock_hunt, stock_overlap) = idle(0.0, 40.0);
    let (lopey_rpms, lopey_hunt, lopey_overlap) = idle(deg(80.0), 40.0);
    assert!(
        lopey_overlap > stock_overlap + 70.0,
        "the re-ground cam must actually have the overlap: {stock_overlap:.0} to \
         {lopey_overlap:.0} deg"
    );

    // The stock cam settles: the governor finds the speed and holds it.
    assert_eq!(
        stock_hunt.period, 0.0,
        "a stock cam must settle, not hunt: {:.2} s at {:.0} rpm peak to peak",
        stock_hunt.period, stock_hunt.amplitude
    );
    assert!(
        swing(&stock_rpms) < 15.0,
        "a settled idle must hold its speed: {:.1} rpm peak to peak",
        swing(&stock_rpms)
    );

    // The big cam does not. Same target, same gains, and it hunts instead.
    assert!(
        lopey_hunt.period > 0.0,
        "a big overlap cam must enter a limit cycle at the same idle target"
    );
    assert!(
        swing(&lopey_rpms) > 4.0 * swing(&stock_rpms),
        "the lope must be a different thing from the stock idle's ripple, not a \
         louder one: {:.1} rpm against {:.1}",
        swing(&lopey_rpms),
        swing(&stock_rpms)
    );

    // And it is a lope rather than roughness: the period is far below the
    // firing frequency, which is what a listener hears as a rate at all.
    let firing_hz = mean(&lopey_rpms) / 120.0 * EnginePreset::cross_plane_v8().firing.len() as f64;
    assert!(
        lopey_hunt.hunt_hz() < firing_hz / 20.0,
        "the limit cycle must be well under the firing frequency: {:.2} Hz against \
         {firing_hz:.0} Hz firing",
        lopey_hunt.hunt_hz()
    );

    // Stable across the render: the second half of the trace swings as
    // much as the first. A transient decays; a limit cycle does not.
    let half = lopey_rpms.len() / 2;
    let (first, second) = (swing(&lopey_rpms[..half]), swing(&lopey_rpms[half..]));
    assert!(
        second > 0.5 * first,
        "the limit cycle must not be a decaying transient: {first:.1} rpm in the \
         first half against {second:.1} in the second"
    );

    // It is still an idle, not a stall and not a flare.
    assert!(
        mean(&lopey_rpms) > STALL_RPM
            && mean(&lopey_rpms) < 2.0 * EnginePreset::cross_plane_v8().idle,
        "a lopey engine still idles: {:.0} rpm",
        mean(&lopey_rpms)
    );
}

#[test]
fn a_peripheral_port_does_not_settle_at_the_idle_a_side_port_does() {
    // Stage M5's idle claim, on the same governor Stage M4 built and did
    // not retune here: reversion through a bigger, faster-opening port
    // dilutes the charge before the governor ever gets a vote. Only the
    // ports move between these two runs — same block, same idle target,
    // same [`IdleGovernor`] gains.
    let dt = 1.0 / 480.0;
    let idle = |valves: ValveTrain, seconds: f64| -> (Vec<f64>, IdleHunt) {
        let mut preset = EnginePreset::two_rotor_wankel();
        preset.model.valves = valves;
        let mut block = preset.block(Environment::default());
        let mut driveline = Driveline::new(&preset);
        let settle = (15.0 / dt) as usize;
        let mut rpms = Vec::with_capacity((seconds / dt) as usize);
        for i in 0..(settle + (seconds / dt) as usize) {
            driveline.update(&mut block, dt);
            block.update(dt, driveline.rpm);
            if i >= settle {
                rpms.push(driveline.rpm);
            }
        }
        assert_eq!(
            driveline.throttle_target, 0.0,
            "the test drove the throttle instead of letting the governor do it"
        );
        (rpms, driveline.idle_hunt)
    };

    let swing = |rpms: &[f64]| -> f64 {
        let hi = rpms.iter().cloned().fold(f64::MIN, f64::max);
        let lo = rpms.iter().cloned().fold(f64::MAX, f64::min);
        hi - lo
    };
    let mean = |rpms: &[f64]| rpms.iter().sum::<f64>() / rpms.len() as f64;

    let (side_rpms, _side_hunt) = idle(crate::physics::rotor::side_port(), 30.0);
    let (peripheral_rpms, _peripheral_hunt) = idle(crate::physics::rotor::peripheral_port(), 30.0);

    // The side port holds a recognisable idle: it stays alive, well clear
    // of the stall floor, near its own target.
    assert!(
        mean(&side_rpms) > STALL_RPM + 200.0,
        "a side-ported rotary must hold a real idle, not hover near stall: {:.0} rpm",
        mean(&side_rpms)
    );

    // The peripheral port does not settle at that idle. This preset's
    // reversion model does not merely hunt for it — see
    // `crate::physics::rotor::peripheral_port`'s own doc comment — it
    // cannot sustain combustion against the governor at all, and the
    // block's own stall floor is where it ends up. Either outcome is
    // "does not settle"; a collapse to the stall floor is accepted
    // alongside a wide limit cycle because both are the same failure,
    // not two different ones.
    let not_settled = peripheral_rpms.iter().all(|&r| r <= STALL_RPM + 1.0)
        || swing(&peripheral_rpms) > 4.0 * swing(&side_rpms);
    assert!(
        not_settled,
        "a peripheral port must not hold the idle a side port does: side swings {:.1} rpm \
         around {:.0}, peripheral swings {:.1} rpm around {:.0}",
        swing(&side_rpms),
        mean(&side_rpms),
        swing(&peripheral_rpms),
        mean(&peripheral_rpms)
    );
}

#[test]
fn big_cam_chopping_v8_chops_at_idle() {
    let preset = EnginePreset::big_cam_chopping_v8();
    assert_eq!(preset.firing.len(), 8);
    assert!(preset.model.valves.overlap().to_degrees() > 60.0);
    assert!(preset.model.valves.aggressiveness() >= 0.45 - 1e-4);

    let dt = 1.0 / 480.0;
    let mut block = preset.block(Environment::default());
    let mut driveline = Driveline::new(&preset);

    let settle = (15.0 / dt) as usize;
    let record = (25.0 / dt) as usize;
    let mut rpms = Vec::with_capacity(record);

    for i in 0..(settle + record) {
        driveline.update(&mut block, dt);
        block.update(dt, driveline.rpm);
        if i >= settle {
            rpms.push(driveline.rpm);
        }
    }

    let mean = rpms.iter().sum::<f64>() / rpms.len() as f64;
    let hi = rpms.iter().cloned().fold(f64::MIN, f64::max);
    let lo = rpms.iter().cloned().fold(f64::MAX, f64::min);
    let swing = hi - lo;

    assert!(mean > STALL_RPM, "engine stalled at idle: {mean:.0} rpm");
    assert!(
        swing > 30.0,
        "the big cam must chop at idle: swing was only {swing:.1} rpm"
    );
    assert!(
        driveline.idle_hunt.period > 0.0,
        "big cam must enter an idle limit cycle"
    );
    let firing_hz = mean / 120.0 * 8.0;
    assert!(
        driveline.idle_hunt.hunt_hz() < firing_hz / 10.0,
        "hunt frequency must be well below firing frequency: {:.2} Hz vs {:.1} Hz firing",
        driveline.idle_hunt.hunt_hz(),
        firing_hz
    );
}

#[test]
fn a_cold_engine_idles_above_a_warm_one_and_converges() {
    let preset = EnginePreset::cross_plane_v8();
    let dt = 1.0 / 120.0;

    let mut block = preset.block(Environment::default());
    block.cold_start();
    let mut driveline = Driveline::new(&preset);

    // Ten seconds of a cold start: the governor is holding a fast idle.
    let idle_for = |block: &mut EngineBlock, driveline: &mut Driveline, seconds: f64| {
        for _ in 0..(seconds / dt) as usize {
            driveline.update(block, dt);
            block.update(dt, driveline.rpm);
        }
        driveline.rpm
    };

    let cold = idle_for(&mut block, &mut driveline, 10.0);
    assert!(
        cold > preset.idle * 1.25,
        "a stone-cold engine idled at {cold:.0} rpm against a warm {:.0}",
        preset.idle
    );

    // And it comes back down on its own, on the block's temperature rather
    // than on a timer: no throttle input anywhere in this test.
    let warm = idle_for(&mut block, &mut driveline, 120.0);
    assert!(
        warm < cold,
        "the fast idle never came down: {warm:.0} rpm after two minutes"
    );
    assert!(
        (warm - preset.idle).abs() < 120.0,
        "converged to {warm:.0} rpm rather than the {:.0} rpm it idles at warm",
        preset.idle
    );
    assert!(
        driveline.throttle_target == 0.0,
        "the test drove the throttle instead of letting the governor do it"
    );
}

#[test]
fn a_presets_shaft_and_its_voice_always_agree() {
    // The two halves of a turbo are wired up separately — one into the
    // snapshot source, one into the synth — and disagreeing is silent in
    // both directions: a shaft nobody listens to, or a voice with nothing
    // driving it, both of which just sound like a missing turbo.
    let mut turbocharged = 0;
    for preset in EnginePreset::catalogue() {
        let block = preset.block(Environment::default());
        let fitted = preset.is_turbocharged();
        assert_eq!(
            preset.snapshot_source(&block).has_turbo(),
            fitted,
            "{}: the shaft disagrees with the preset",
            preset.name
        );
        assert_eq!(
            preset.synth_config(&block, 48_000.0).turbo.is_some(),
            fitted,
            "{}: the voice disagrees with the preset",
            preset.name
        );
        turbocharged += usize::from(fitted);
    }
    // Both kinds have to be represented, or the distinction is untested in
    // every other test in this file.
    assert!(turbocharged > 0, "no turbocharged engine in the catalogue");
    assert!(
        turbocharged < EnginePreset::catalogue().len(),
        "every engine is turbocharged"
    );
}

#[test]
fn induction_hardware_is_fitted_across_presets() {
    let v8 = EnginePreset::cross_plane_v8();
    assert!(
        matches!(v8.induction, Induction::RootsSupercharged { .. }),
        "cross_plane_v8 should carry a Roots supercharger"
    );
    assert!(
        v8.exhaust.cutout_fitted,
        "cross_plane_v8 should have an exhaust cutout fitted"
    );

    let v10 = EnginePreset::v10();
    assert!(
        matches!(v10.induction, Induction::CentrifugalSupercharged { .. }),
        "v10 should carry a centrifugal supercharger"
    );

    let t_i4 = EnginePreset::turbo_inline_four();
    assert!(
        t_i4.anti_lag,
        "turbo_inline_four should have anti-lag armed"
    );
    let t_i4_block = t_i4.block(Environment::default());
    assert!(
        t_i4_block.ecu.anti_lag,
        "block ECU must inherit preset's anti-lag setting"
    );
    assert!(
        matches!(
            t_i4.induction,
            Induction::Turbocharged {
                blow_off: Some(_),
                ..
            }
        ),
        "turbo_inline_four should carry a blow-off valve"
    );

    let tt_v8 = EnginePreset::twin_turbo_v8();
    assert!(
        tt_v8.exhaust.cutout_fitted,
        "twin_turbo_v8 should have an exhaust cutout fitted"
    );
    assert!(
        matches!(
            tt_v8.induction,
            Induction::Turbocharged {
                wastegate: Some(_),
                ..
            }
        ),
        "twin_turbo_v8 should carry wastegate chatter"
    );

    let t_i6 = EnginePreset::turbo_inline_six();
    assert!(
        matches!(
            t_i6.induction,
            Induction::Turbocharged {
                blow_off: Some(_),
                wastegate: Some(_),
                ..
            }
        ),
        "turbo_inline_six should carry both blow-off and wastegate chatter"
    );

    // Check that SynthConfig accurately mirrors every preset's induction voicing
    for preset in EnginePreset::catalogue() {
        let block = preset.block(Environment::default());
        let config = preset.synth_config(&block, 48_000.0);
        match preset.induction {
            Induction::NaturallyAspirated => {
                assert!(config.turbo.is_none());
                assert!(config.roots.is_none());
                assert!(config.centrifugal.is_none());
            }
            Induction::Turbocharged {
                voice,
                blow_off,
                wastegate,
                ..
            } => {
                assert_eq!(config.turbo, Some(voice));
                assert_eq!(config.blow_off, blow_off);
                assert_eq!(config.wastegate, wastegate);
            }
            Induction::RootsSupercharged { voice } => {
                assert_eq!(config.roots, Some(voice));
                assert!(config.turbo.is_none());
            }
            Induction::CentrifugalSupercharged { voice, blow_off } => {
                assert_eq!(config.centrifugal, Some(voice));
                assert_eq!(config.blow_off, blow_off);
                assert!(config.turbo.is_none());
            }
        }
    }
}

#[test]
fn the_wankel_breathes_like_a_2_6_litre_four_stroke() {
    // The mapping in `FiringOrder::two_rotor_wankel` is only meaningful if
    // the displacement per cycle comes out right; a chamber sized as though
    // it were a piston would quietly halve the engine.
    let wankel = EnginePreset::two_rotor_wankel();
    let chamber = wankel.model.geometry.displacement() * 1e6;
    assert!(
        (600.0..700.0).contains(&chamber),
        "chamber is {chamber:.0} cc, not a 654 cc rotor face"
    );
    assert!((2.5..2.7).contains(&(wankel.displacement() * 1e3)));
}

#[test]
fn every_presets_derived_collector_reflection_differs_and_matches_area_ratio() {
    let catalogue = EnginePreset::catalogue();
    let mut reflections = Vec::new();

    for preset in &catalogue {
        let a_primary = preset.exhaust.primary_area();
        let a_outlet = preset.exhaust.collector.outlet_area;
        let expected = (a_primary - a_outlet) / (a_primary + a_outlet);
        let derived = preset.exhaust.collector_reflection();

        assert!(
            (derived - expected).abs() < 1e-12,
            "{}: derived reflection {derived} != expected {expected}",
            preset.name
        );
        assert!(
            derived < 0.0,
            "{}: primary expands into larger collector, reflection must invert phase",
            preset.name
        );
        reflections.push((preset.name, derived));
    }

    // Assert all presets derive distinct reflection coefficients.
    for i in 0..reflections.len() {
        for j in (i + 1)..reflections.len() {
            let (name_a, r_a) = reflections[i];
            let (name_b, r_b) = reflections[j];
            assert!(
                (r_a - r_b).abs() > 1e-4,
                "{name_a} and {name_b} share collector reflection {r_a:.4}"
            );
        }
    }
}

#[test]
fn primary_quarter_wave_fundamental_lands_on_hot_gas() {
    // Hot exhaust gas: gamma = 1.33, R = 287 J/(kg K), T = 900 K
    let c = (1.33 * 287.0 * 900.0_f64).sqrt();

    for preset in EnginePreset::catalogue() {
        let length = preset.exhaust.primary_length();
        let expected = c / (4.0 * length);
        let derived = preset.exhaust.primary_quarter_wave_hz(c);

        assert!(
            (derived - expected).abs() < 1e-9,
            "{}: quarter-wave {derived} Hz != expected {expected} Hz",
            preset.name
        );
        // Sane fundamental frequency bounds for physical exhaust primaries (0.30 - 0.60 m)
        assert!(
            (200.0..600.0).contains(&derived),
            "{}: quarter-wave {derived} Hz is out of plausible range",
            preset.name
        );
    }
}

#[test]
fn every_preset_is_cast_on_production_bore_centres() {
    use crate::audio::structure::MIN_BORE_WALL;

    for preset in EnginePreset::catalogue() {
        let bore = preset.model.geometry.bore;
        let ratio = preset.bore_spacing / bore;
        assert!(
            (1.05..=1.35).contains(&ratio),
            "{}: {:.3} bores between centres is not a block anyone casts",
            preset.name,
            ratio
        );
        // And the wall the bore-wall modes ring on is real metal, not the
        // model's floor: a spacing that had to be clamped is a spacing that
        // was not describing this engine.
        let wall = 0.5 * (preset.bore_spacing - bore);
        assert!(
            wall >= MIN_BORE_WALL as f64,
            "{}: {:.1} mm of wall between the bores",
            preset.name,
            wall * 1e3
        );
    }
}

#[test]
fn no_two_presets_share_the_same_derived_acoustic_constants() {
    use crate::audio::filters::block_resonance_hz;

    let catalogue = EnginePreset::catalogue();
    let c = (1.33 * 287.0 * 900.0_f64).sqrt();

    for i in 0..catalogue.len() {
        for j in (i + 1)..catalogue.len() {
            let a = &catalogue[i];
            let b = &catalogue[j];

            let r_a = a.exhaust.collector_reflection();
            let r_b = b.exhaust.collector_reflection();
            let f_qw_a = a.exhaust.primary_quarter_wave_hz(c);
            let f_qw_b = b.exhaust.primary_quarter_wave_hz(c);
            let tau_rt_a = a.exhaust.primary_round_trip_seconds(c);
            let tau_rt_b = b.exhaust.primary_round_trip_seconds(c);
            let f_block_a = block_resonance_hz(a.block_mass as f32);
            let f_block_b = block_resonance_hz(b.block_mass as f32);

            let identical = (r_a - r_b).abs() < 1e-6
                && (f_qw_a - f_qw_b).abs() < 1e-3
                && (tau_rt_a - tau_rt_b).abs() < 1e-6
                && (f_block_a - f_block_b).abs() < 1e-3;

            assert!(
                !identical,
                "regression: {} and {} share identical derived acoustics",
                a.name, b.name
            );
        }
    }
}

#[test]
fn physics_pipe_and_audio_path_round_trip_times_agree() {
    use crate::audio::dsp::EngineSynth;
    use crate::audio::filters::round_trip_seconds;

    let fs = 48_000.0;
    let env = Environment::default();

    for preset in EnginePreset::catalogue() {
        let block = preset.block(env);
        let config = preset.synth_config(&block, fs);
        let synth = EngineSynth::new(config);

        let gamma = block.model.gas.gamma_burned;
        let gas_constant = block.model.gas.r_burned;
        let temperature = 900.0;
        let c = (gamma * gas_constant * temperature).sqrt();

        for (bank_idx, manifold) in block.exhaust_banks.iter().enumerate() {
            let phys_round_trip = manifold.pipe.transit_time(c);
            let runner_length = preset
                .exhaust
                .primary_length_for_cylinders(&block.firing.cylinders_on_bank(bank_idx as u8));

            // Direct analytical calculation in audio filters:
            let audio_filter_round_trip = round_trip_seconds(
                runner_length as f32,
                gamma as f32,
                gas_constant as f32,
                temperature as f32,
            ) as f64;

            let diff_filter_samples = (phys_round_trip - audio_filter_round_trip).abs() * fs as f64;
            assert!(
                diff_filter_samples < 1.0,
                "{}, bank {}: physics round trip ({:.6} s) and audio filter ({:.6} s) disagree by {:.3} samples",
                preset.name,
                bank_idx,
                phys_round_trip,
                audio_filter_round_trip,
                diff_filter_samples
            );

            // Initialized audio synth runner agreement at default ambient
            // conditions. No end correction here: a primary in the network
            // stops at the collector junction, which scatters off an area
            // ratio, and only a pipe stopping in open air is lengthened by
            // the slug of gas outside its mouth.
            let c_ambient = (1.33 * 287.0 * 300.0_f64).sqrt();
            let phys_ambient_round_trip = 2.0 * runner_length / c_ambient;
            let audio_synth_round_trip = synth.runner_round_trip_seconds(bank_idx) as f64;
            let diff_synth_samples =
                (phys_ambient_round_trip - audio_synth_round_trip).abs() * fs as f64;
            assert!(
                diff_synth_samples < 1.0,
                "{}, bank {}: physics round trip ({:.6} s) and synth runner ({:.6} s) disagree by {:.3} samples",
                preset.name,
                bank_idx,
                phys_ambient_round_trip,
                audio_synth_round_trip,
                diff_synth_samples
            );
        }
    }
}

/// Which of the engine's two radiating paths is left alive.
#[derive(Clone, Copy)]
enum Path {
    /// Only the exhaust network: pipes, collector, silencers, tailpipe.
    Pipe,
    /// Only the block: the modal structure and the three sources into it.
    Structure,
}

/// Radiated RMS of one preset with everything but one path muted [-].
///
/// Both measurements are of the same engine, in the same environment, at
/// the same speed and throttle, through the same propagation model; the
/// only difference between them is which of the two buses is turned off.
/// The compressors are off in both, because a whistle is neither a pipe nor
/// a block and the diesel has one where the atmospheric four does not.
fn radiated_through(preset: &EnginePreset, rpm: f64, path: Path) -> f64 {
    use crate::audio::dsp::EngineSynth;

    let fs = 48_000.0;
    let block = primed(preset, rpm);
    let mut source = preset.snapshot_source(&block);
    let snapshot = source.sample(&block, rpm, 1.0 / 240.0, EngineControls::wide_open());

    let mut config = preset.synth_config(&block, fs);
    config.intake_level = 0.0;
    config.turbo = None;
    config.roots = None;
    config.centrifugal = None;
    config.blow_off = None;
    config.wastegate = None;
    match path {
        Path::Pipe => config.structure_level = 0.0,
        Path::Structure => config.exhaust_level = 0.0,
    }

    let mut synth = EngineSynth::new(config);
    synth.set_snapshot(&snapshot);
    let mut buffer = vec![0.0f32; 2 * 48_000];
    synth.render(&mut buffer, 2); // settle the pipes and the smoothers
    synth.render(&mut buffer, 2);
    let power: f64 = buffer
        .chunks(2)
        .map(|frame| {
            let mono = 0.5 * (frame[0] + frame[1]) as f64;
            mono * mono
        })
        .sum();
    (power / (buffer.len() / 2) as f64).sqrt()
}

#[test]
fn the_diesel_radiates_through_its_block_and_the_petrol_four_through_its_pipe() {
    // The test that validates Stage 8 as much as Stage 14. Combustion has
    // two ways out of an engine — down the exhaust as gas, and through the
    // castings as vibration — and which one carries the sound is not a
    // mixing decision. It is decided by how fast the pressure rises and by
    // what is standing in the way of the pulse, and those are both physics
    // this crate solves rather than parameters anyone typed.
    //
    // A diesel puts a premixed spike into the first crank degrees of its
    // burn and hangs a turbine in front of its tailpipe; an atmospheric
    // petrol four does neither. Nothing below tells either engine which
    // answer to give.
    let diesel = EnginePreset::turbo_diesel_four();
    let petrol = EnginePreset::inline_four();

    let split = |preset: &EnginePreset, rpm: f64| {
        let structure = radiated_through(preset, rpm, Path::Structure);
        let pipe = radiated_through(preset, rpm, Path::Pipe);
        assert!(
            structure > 0.0 && pipe > 0.0,
            "{} at {rpm:.0}: a path went silent — structure {structure:.6}, pipe {pipe:.6}",
            preset.name
        );
        (structure / pipe, structure, pipe)
    };

    // The band a diesel is actually driven in: it idles at eight hundred
    // and is on the limiter at five thousand, and everything it does for a
    // living happens between the two.
    let mut diesel_ratios = Vec::new();
    for rpm in [1_500.0, 2_250.0, 3_000.0] {
        let (diesel_ratio, _, _) = split(&diesel, rpm);
        let (petrol_ratio, petrol_structure, petrol_pipe) = split(&petrol, rpm);
        diesel_ratios.push(diesel_ratio);

        assert!(
            petrol_ratio < 1.0,
            "at {rpm:.0} rpm the petrol four is not pipe-dominated: block \
             {petrol_structure:.5} against pipe {petrol_pipe:.5}"
        );
        // And the gap is not a rounding accident. It used to measure four
        // and a half, consistently, across the range; Stage T5 gave the
        // inline-four's own expansion chamber a real loss term, which
        // quietens exactly its pipe path and so narrows the gap — to as
        // little as 2.8x at 3000 rpm — without touching the diesel side,
        // which has no reactive chamber to lose energy from. 2.5x is
        // comfortably under the new range and still an order away from
        // "too much alike".
        assert!(
            diesel_ratio > 2.5 * petrol_ratio,
            "at {rpm:.0} rpm the two engines radiate too much alike: diesel \
             {diesel_ratio:.3} against petrol {petrol_ratio:.3}"
        );
    }

    // Structure-dominated across the range it works in. Taken over the
    // range and not speed by speed, because at the bottom of it the pipe
    // still just edges the block — 0.96 against 1.0 — and the reason is
    // named in the preset: the network has no turbine in it. A VGT takes
    // most of what a blowdown pulse carries and turns it into shaft work,
    // and what stands in for one here is the housing volume alone, which
    // can only reflect. The absolute claim used to hold at 1500 rpm
    // because the exhaust was damped everywhere else instead — a wall loss
    // several times what a pipe measures, and a valve that bled the bottom
    // of the band into the cylinder. Those were wrong, they are fixed, and
    // what they were covering for is this.
    let mean = diesel_ratios.iter().sum::<f64>() / diesel_ratios.len() as f64;
    assert!(
        mean > 1.0,
        "the diesel is not structure-dominated across its range: {diesel_ratios:?}"
    );
}

#[test]
fn the_single_rocks_its_crank_where_the_twelve_barely_ripples() {
    // Stage 1c integrates crank speed at audio rate from the indicated
    // torque curve and the rotating inertia, so how much the crank hunts
    // inside its own cycle is a property of the engine rather than a
    // setting. A single is the extreme case in both terms at once: one
    // firing per two revolutions instead of twelve, and a tenth of the
    // inertia to smooth it with. It should not be a subtle difference, and
    // it is not.
    use crate::audio::dsp::EngineSynth;

    let rpm = 1_250.0;
    let ripple = |preset: &EnginePreset| {
        let block = primed(preset, rpm);
        let mut source = preset.snapshot_source(&block);
        let snapshot = source.sample(&block, rpm, 1.0 / 240.0, EngineControls::wide_open());
        let mut synth = EngineSynth::new(preset.synth_config(&block, 48_000.0));
        synth.set_snapshot(&snapshot);

        let mut frame = [0.0f32; 2];
        // A second to settle the speed smoother, then two cycles of it.
        for _ in 0..48_000 {
            synth.render(&mut frame, 2);
        }
        let (mut low, mut high) = (f32::MAX, f32::MIN);
        for _ in 0..24_000 {
            synth.render(&mut frame, 2);
            let delta = synth.crank_omega_delta();
            low = low.min(delta);
            high = high.max(delta);
        }
        let nominal = 4.0 * std::f32::consts::PI * (rpm as f32 / 120.0);
        (high - low) / nominal
    };

    let single = ripple(&EnginePreset::big_single());
    let twelve = ripple(&EnginePreset::v12());

    assert!(
        single > 0.02,
        "the single's crank barely moved: {:.3} % of peak-to-peak ripple",
        single * 100.0
    );
    assert!(
        single > 5.0 * twelve,
        "one cylinder on a light flywheel ripples {:.2} % against the V12's {:.2} %, \
         which is not the difference between a thumper and a twelve",
        single * 100.0,
        twelve * 100.0
    );
}

/// The solved cylinder pressure of one preset, cell by crank degree, with
/// the motored isentrope of the same cycle taken off it.
///
/// What is left is the pressure combustion put there and nothing else: no
/// compression ramp, no expansion. Its slope is what hammers the block.
fn combustion_overpressure(preset: &EnginePreset, rpm: f64) -> (Vec<f64>, f64) {
    let block = primed(preset, rpm);
    let latch = block.master.latch;
    let ignition = latch
        .autoignition
        .expect("a compression-ignition preset must have lit")
        .angle
        .to_degrees();
    let trace = (0..PHASE_CELLS)
        .map(|cell| {
            let theta = deg(cell as f64 + 0.5);
            block.ring.cell(cell).pressure
                - latch.motored_pressure(preset.model.geometry.safe_volume(theta))
        })
        .collect();
    (trace, ignition)
}

#[test]
fn the_diesel_pressure_trace_spikes_before_it_humps() {
    // The shape a two-stage release puts on a real solved cycle, as
    // distinct from the shape the profile has on its own. Everything here
    // comes out of the RK4 solver's own phase ring; nothing samples the
    // Wiebe.
    let rpm = 1_800.0;
    let preset = EnginePreset::turbo_diesel_four();
    let (trace, ignition) = combustion_overpressure(&preset, rpm);
    let slope = |k: usize| trace[k] - trace[k - 1];

    // The spike: the steepest pressure rise of the whole cycle lands within
    // a few degrees of the angle the Arrhenius integral picked, not at the
    // injector opening and not at top dead centre by coincidence.
    let (spike_at, spike_rate) =
        (1..PHASE_CELLS)
            .map(|k| (k as f64, slope(k)))
            .fold(
                (0.0, f64::MIN),
                |best, now| if now.1 > best.1 { now } else { best },
            );
    assert!(
        (spike_at - ignition).abs() < 5.0,
        "the steepest rise is at {spike_at:.0} degrees but the charge lit at \
         {ignition:.1}"
    );

    // The hump: long after the spike has died back to a twentieth of
    // itself, the diffusion burn is still adding pressure, and the
    // overpressure does not reach its own maximum until well past it.
    let spike_over = (spike_at as usize..PHASE_CELLS)
        .find(|&k| slope(k) < 0.05 * spike_rate)
        .expect("the spike must end");
    let (hump_at, _) = (spike_over..spike_over + 60)
        .map(|k| (k as f64, trace[k]))
        .fold(
            (0.0, f64::MIN),
            |best, now| if now.1 > best.1 { now } else { best },
        );
    assert!(
        hump_at > spike_at + 8.0,
        "nothing burned after the spike: the overpressure peaked at \
         {hump_at:.0} degrees against a spike at {spike_at:.0}"
    );
    assert!(
        trace[hump_at as usize] > trace[spike_over],
        "the diffusion burn added no pressure at all"
    );

    // And the spike is the premixed stage's doing. Collapsing the two
    // stages into one — same fuel, same ignition angle, same everything
    // else — takes most of the rate out of the cycle while leaving the
    // burn itself intact, which is the whole claim of a two-stage model.
    let mut single_stage = EnginePreset::turbo_diesel_four();
    let diesel = *preset
        .model
        .combustion
        .compression()
        .expect("the diesel preset must be compression-ignition");
    single_stage.model.combustion = HeatRelease::Compression(DieselCombustion {
        premixed_duration: diesel.diffusion_duration,
        premixed_form_factor: diesel.diffusion_form_factor,
        ..diesel
    });
    let (flat, _) = combustion_overpressure(&single_stage, rpm);
    let flat_rate = (1..PHASE_CELLS)
        .map(|k| flat[k] - flat[k - 1])
        .fold(f64::MIN, f64::max);
    assert!(
        spike_rate > 2.0 * flat_rate,
        "the premixed stage is not what makes the rate: {:.2} bar/deg with it \
         against {:.2} without",
        spike_rate / 1e5,
        flat_rate / 1e5
    );
}

#[test]
fn a_diesel_has_no_spark_to_cut_and_so_never_pops() {
    // The driver holding the ignition cut, and the engine sitting on its
    // limiter, are the two ways a petrol engine sends a cylinder's worth of
    // raw fuel down a red-hot pipe. Neither exists on a diesel: there is no
    // coil to interrupt, and its limiter takes the fuel away instead — so
    // the charge that would have popped was never metered in the first
    // place.
    let cut = EngineControls {
        spark_cut: true,
        ..EngineControls::wide_open()
    };

    let unburnt = |preset: &EnginePreset, rpm: f64| {
        let block = primed(preset, rpm);
        let mut source = preset.snapshot_source(&block);
        source
            .sample(&block, rpm, 1.0 / 240.0, cut)
            .unburnt_fuel_mass
    };

    let diesel = EnginePreset::turbo_diesel_four();
    let petrol = EnginePreset::inline_four();
    assert_eq!(
        unburnt(&diesel, 3_000.0),
        0.0,
        "a cut coil sent fuel out of an engine that has no coil"
    );
    assert!(
        unburnt(&petrol, 3_000.0) > 0.0,
        "the petrol four should be pumping its charge out unburnt on a cut"
    );

    // And on the limiter the same holds for the opposite reason: the
    // diesel's cut is a fuel cut, so its cylinders are empty.
    let mut on_the_limiter = diesel.block(Environment::default());
    let mut driveline = Driveline::new(&diesel);
    driveline.throttle_target = 1.0;
    let dt = 1.0 / 240.0;
    let mut reached = false;
    let mut cut_kind = crate::physics::control::LimiterCut::None;
    for _ in 0..(240 * 20) {
        driveline.update(&mut on_the_limiter, dt);
        on_the_limiter.update(dt, driveline.rpm);
        reached |= driveline.on_the_limiter();
        if on_the_limiter.ecu.active_cut != crate::physics::control::LimiterCut::None {
            cut_kind = on_the_limiter.ecu.active_cut;
        }
    }
    assert!(reached, "the diesel never got to its limiter");
    assert_eq!(
        cut_kind,
        crate::physics::control::LimiterCut::Fuel,
        "a diesel's limiter has to take the fuel, not a spark it does not have"
    );
}

#[test]
fn every_preset_has_valid_aperture_positions() {
    for preset in EnginePreset::catalogue() {
        assert!(
            !preset.aperture_positions.tailpipes.is_empty(),
            "{}: has no tailpipes configured",
            preset.name
        );
        for (i, tp) in preset.aperture_positions.tailpipes.iter().enumerate() {
            assert!(
                tp[2] > 0.0,
                "{}: tailpipe {i} height <= 0 ({:?})",
                preset.name,
                tp
            );
        }
        assert!(
            preset.aperture_positions.intake[2] > 0.0,
            "{}: intake height <= 0 ({:?})",
            preset.name,
            preset.aperture_positions.intake
        );
        assert!(
            preset.aperture_positions.block[2] > 0.0,
            "{}: block height <= 0 ({:?})",
            preset.name,
            preset.aperture_positions.block
        );
    }
}

#[test]
fn every_preset_configures_limiter_mode_and_cut_type() {
    for preset in EnginePreset::catalogue() {
        let block = preset.block(Environment::default());
        assert_eq!(
            block.ecu.limiter_mode, preset.limiter_mode,
            "{}: limiter_mode mismatch",
            preset.name
        );
        assert_eq!(
            block.ecu.limiter_cut_type, preset.limiter_cut,
            "{}: limiter_cut mismatch",
            preset.name
        );
    }
}

#[test]
fn dyno_absorber_holds_target_rpm() {
    let preset = EnginePreset::inline_four();
    let mut block = preset.block(Environment::default());
    let mut driveline = Driveline::new(&preset);
    let target_rpm = 3_500.0;
    driveline.dyno_mode = DynoMode::RpmHold { target_rpm };
    driveline.throttle_target = 0.80; // 80 % throttle applied

    let dt = 1.0 / 240.0;
    for _ in 0..(240 * 3) {
        driveline.update(&mut block, dt);
        block.update(dt, driveline.rpm);
    }

    assert!(
        (driveline.rpm - target_rpm).abs() < 50.0,
        "rpm hold failed to lock target: {:.1} rpm vs target {:.1} rpm",
        driveline.rpm,
        target_rpm
    );
    assert!(
        driveline.dyno_absorber_torque > 50.0,
        "dyno absorber must be producing opposing torque to hold 80% throttle: {:.1} N m",
        driveline.dyno_absorber_torque
    );
}

#[test]
fn dyno_sweep_pull_records_clean_curve() {
    let preset = EnginePreset::inline_four();
    let mut block = preset.block(Environment::default());
    let mut driveline = Driveline::new(&preset);
    driveline.rpm = 2_000.0;
    driveline.dyno_mode = DynoMode::SweepPull {
        start_rpm: 2_000.0,
        rate_rpm_s: 600.0,
    };

    let dt = 1.0 / 240.0;
    while driveline.dyno_mode != DynoMode::FreeRev {
        driveline.update(&mut block, dt);
        block.update(dt, driveline.rpm);
    }

    let pull = driveline
        .last_pull
        .as_ref()
        .expect("pull must be finalized in last_pull");
    assert!(
        !pull.points.is_empty(),
        "dyno pull must contain recorded points"
    );
    let peak_t = pull.peak_torque.expect("must have peak torque");
    let peak_p = pull.peak_power.expect("must have peak power");
    assert!(
        peak_t.torque > 100.0,
        "peak torque must be plausible: {:.1} N m",
        peak_t.torque
    );
    assert!(
        peak_p.power_kw > 50.0,
        "peak power must be plausible: {:.1} kW",
        peak_p.power_kw
    );
    assert!(
        peak_p.rpm >= peak_t.rpm,
        "peak power rpm ({:.0}) must sit at or above peak torque rpm ({:.0})",
        peak_p.rpm,
        peak_t.rpm
    );
}

#[test]
fn dyno_sweep_pull_catalogue_from_idle() {
    for preset in EnginePreset::catalogue() {
        let mut block = preset.block(Environment::default());
        let mut driveline = Driveline::new(&preset);
        driveline.rpm = preset.idle;
        let dt = 1.0 / 240.0;
        // Settle at idle
        for _ in 0..(240 * 2) {
            driveline.update(&mut block, dt);
            block.update(dt, driveline.rpm);
        }

        driveline.trigger_sweep_pull();
        for _ in 0..(240 * 30) {
            driveline.update(&mut block, dt);
            block.update(dt, driveline.rpm);
            if driveline.last_pull.is_some() {
                break;
            }
        }
        let pull = driveline.last_pull.as_ref().unwrap_or_else(|| {
            panic!(
                "preset '{}' failed to complete pull: rpm was {:.1}, mode was {:?}",
                preset.name, driveline.rpm, driveline.dyno_mode
            )
        });
        assert!(
            !pull.points.is_empty(),
            "preset '{}' pull has no points",
            preset.name
        );
        let pt = pull
            .peak_torque
            .unwrap_or_else(|| panic!("preset '{}' has no peak torque", preset.name));
        let pp = pull
            .peak_power
            .unwrap_or_else(|| panic!("preset '{}' has no peak power", preset.name));
        assert!(pt.torque > 0.0, "preset '{}' peak torque <= 0", preset.name);
        assert!(
            pp.power_kw > 0.0,
            "preset '{}' peak power <= 0",
            preset.name
        );
    }
}

#[test]
fn sae_j1349_correction_factors() {
    // At standard reference (25 °C = 298.15 K, 99.0 kPa), CF must equal 1.0
    let standard = sae_j1349_correction(99_000.0, 298.15);
    assert!((standard - 1.0).abs() < 1e-6);

    // Lower pressure (high altitude) reduces air density -> CF > 1.0 to correct back up
    let high_altitude = sae_j1349_correction(85_000.0, 298.15);
    assert!(high_altitude > 1.0);

    // Hotter day reduces air density -> CF > 1.0
    let hot_day = sae_j1349_correction(99_000.0, 315.0);
    assert!(hot_day > 1.0);
}

#[test]
fn dyno_controls_toggle_and_sweep() {
    let preset = EnginePreset::inline_four();
    let mut driveline = Driveline::new(&preset);
    driveline.rpm = 3_456.0;

    driveline.toggle_rpm_hold();
    match driveline.dyno_mode {
        DynoMode::RpmHold { target_rpm } => {
            assert_eq!(target_rpm, 3_500.0);
        }
        _ => panic!("expected RpmHold mode"),
    }

    driveline.nudge_held_rpm(100.0);
    match driveline.dyno_mode {
        DynoMode::RpmHold { target_rpm } => {
            assert_eq!(target_rpm, 3_600.0);
        }
        _ => panic!("expected RpmHold mode with nudged target"),
    }

    driveline.toggle_rpm_hold();
    assert_eq!(driveline.dyno_mode, DynoMode::FreeRev);

    driveline.trigger_sweep_pull();
    match driveline.dyno_mode {
        DynoMode::SweepPull { rate_rpm_s, .. } => {
            assert_eq!(rate_rpm_s, 300.0);
        }
        _ => panic!("expected SweepPull mode"),
    }
}

#[test]
fn gt3_cup_collection_contains_valid_race_engines() {
    let race_engines = EnginePreset::gt3_cup_collection();
    assert_eq!(race_engines.len(), 5);

    for engine in &race_engines {
        assert!(
            engine.redline >= 7_500.0,
            "race engine {} redline too low: {}",
            engine.name,
            engine.redline
        );
        assert!(
            engine.inertia <= 0.25,
            "race engine {} inertia too high: {}",
            engine.name,
            engine.inertia
        );
        assert!(
            engine.exhaust.cutout_fitted,
            "race engine {} should have a cutout fitted",
            engine.name
        );
        assert_eq!(engine.limiter_mode, LimiterMode::RotatingStutter);
        assert_eq!(engine.limiter_cut, LimiterCut::Spark);

        let block = engine.block(Environment::default());
        assert_eq!(block.firing.len(), engine.firing.len());
        assert_eq!(block.ecu.redline, engine.redline);
    }

    // Check specifically the 911 GT3 Cup engines
    let cup_992 = race_engines
        .iter()
        .find(|e| e.name.contains("992"))
        .expect("992 cup");
    assert_eq!(cup_992.firing.len(), 6);
    assert!((cup_992.displacement() * 1e3 - 4.0).abs() < 0.1);
    assert_eq!(cup_992.redline, 8_750.0);

    let cup_997 = race_engines
        .iter()
        .find(|e| e.name.contains("997"))
        .expect("997 cup");
    assert_eq!(cup_997.firing.len(), 6);
    assert!((cup_997.displacement() * 1e3 - 3.8).abs() < 0.1);
    assert_eq!(cup_997.redline, 8_500.0);
}
