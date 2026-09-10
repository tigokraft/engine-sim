//! Fixed drive scripts: what the engine is asked to do while it is measured.
//!
//! A measurement is only comparable to the one before it if the engine was
//! doing the same thing both times, so a script is a pure function of elapsed
//! time — no wall clock, no flywheel, no driver. `block.update` is handed the
//! speed the script says, exactly as the offline mode of the `engine_audio`
//! example already does, which is what makes two runs of one script identical
//! sample for sample.
//!
//! A script says three things: how fast the engine is turning, how far the
//! throttle is open, and whether the ignition is cutting. There is no gear in
//! it because there is no gearbox in the simulator — the bench's
//! [`Driveline`](crate::bench::Driveline) is a flywheel against a dyno brake,
//! and a ratio between it and the road would change nothing any of these
//! measurements can see.
//!
//! The six fixed profiles are chosen to corner different parts of the audio
//! path rather than to be a realistic lap:
//!
//! | Profile | What it is there to expose |
//! |---|---|
//! | [`idle_hold`] | the noise floor, the mechanical layer, and the low orders |
//! | [`sweep_up`] | resonances, as every order is dragged through every pipe mode |
//! | [`sweep_down`] | the same modes approached from above, and the overrun |
//! | [`tip_in`] | transient response: a throttle step with the engine still slow |
//! | [`overrun_cut`] | unburnt fuel down a hot pipe — the backfire path |
//! | [`limiter_bounce`] | the hardest thing the synth is ever asked to do |
//!
//! Every profile is built from the preset's own idle and redline, so a V12 is
//! swept over a V12's rev range rather than over a borrowed one.

use crate::audio::EngineControls;
use crate::bench::EnginePreset;

/// Throttle position that holds an idle.
const IDLE_THROTTLE: f64 = 0.06;

/// One leg of a script: a linear ramp in speed and throttle.
///
/// Linear because a measurement wants a sweep whose rate is known, not one that
/// is shaped to feel like a driver.
#[derive(Debug, Clone, Copy)]
pub struct Segment {
    /// How long the leg lasts [s].
    pub seconds: f64,
    /// Speed at the start and the end of the leg [rev/min].
    pub rpm: (f64, f64),
    /// Throttle at the start and the end of the leg, `0..=1`.
    pub throttle: (f64, f64),
    /// Whether the ignition is cutting for the whole leg.
    pub spark_cut: bool,
    /// Depth and rate of a speed oscillation under the ramp, `(rev/min, Hz)`.
    ///
    /// A rev limiter cuts ignition and lets the engine fall away from the
    /// redline, so the oscillation only ever subtracts: the ramp is the ceiling
    /// the engine bounces off, not a line it sits on.
    pub bounce: (f64, f64),
}

impl Segment {
    /// A leg that holds one speed and one throttle.
    pub fn hold(seconds: f64, rpm: f64, throttle: f64) -> Self {
        Self {
            seconds,
            rpm: (rpm, rpm),
            throttle: (throttle, throttle),
            spark_cut: false,
            bounce: (0.0, 0.0),
        }
    }

    /// A leg that ramps from one speed and throttle to another.
    pub fn ramp(seconds: f64, rpm: (f64, f64), throttle: (f64, f64)) -> Self {
        Self {
            seconds,
            rpm,
            throttle,
            spark_cut: false,
            bounce: (0.0, 0.0),
        }
    }

    /// The same leg with the ignition cutting.
    pub fn cutting(mut self) -> Self {
        self.spark_cut = true;
        self
    }

    /// The same leg with the engine bouncing `depth` rev/min at `hz`.
    pub fn bouncing(mut self, depth: f64, hz: f64) -> Self {
        self.bounce = (depth, hz);
        self
    }

    /// Speed and controls `elapsed` seconds — a fraction `u` — into the leg.
    ///
    /// The bounce is phased from the start of its own leg rather than from the
    /// start of the script, so it enters at zero and the speed stays continuous
    /// across the boundary into it.
    fn at(&self, u: f64, elapsed: f64) -> (f64, EngineControls) {
        let u = u.clamp(0.0, 1.0);
        let (depth, hz) = self.bounce;
        let rpm = self.rpm.0 + (self.rpm.1 - self.rpm.0) * u
            - depth * (std::f64::consts::TAU * hz * elapsed).sin().abs();
        let throttle = (self.throttle.0 + (self.throttle.1 - self.throttle.0) * u).clamp(0.0, 1.0);
        (
            rpm,
            EngineControls {
                throttle,
                spark_cut: self.spark_cut,
            },
        )
    }
}

/// A named, deterministic drive cycle.
#[derive(Debug, Clone)]
pub struct RenderScript {
    /// Short slug, used for the WAV's filename and the table's row.
    pub name: &'static str,
    /// One line on what the profile is there to expose.
    pub note: &'static str,
    /// The legs, run back to back.
    segments: Vec<Segment>,
}

impl RenderScript {
    /// Builds a script from its legs.
    pub fn new(name: &'static str, note: &'static str, segments: Vec<Segment>) -> Self {
        Self {
            name,
            note,
            segments,
        }
    }

    /// Total length [s].
    pub fn seconds(&self) -> f64 {
        self.segments.iter().map(|s| s.seconds).sum()
    }

    /// Speed the engine starts at [rev/min], which is what the block is primed to.
    pub fn start_rpm(&self) -> f64 {
        self.segments.first().map_or(0.0, |s| s.rpm.0)
    }

    /// Speed and controls at elapsed time `t` [s].
    ///
    /// Past the end of the script the last leg's final state is held, so a
    /// render that runs long tails off rather than falling to zero rpm.
    pub fn at(&self, t: f64) -> (f64, EngineControls) {
        let mut start = 0.0;
        for segment in &self.segments {
            if t < start + segment.seconds || segment.seconds <= 0.0 {
                let elapsed = t - start;
                return segment.at(elapsed / segment.seconds.max(f64::MIN_POSITIVE), elapsed);
            }
            start += segment.seconds;
        }
        match self.segments.last() {
            Some(last) => last.at(1.0, t - (self.seconds() - last.seconds)),
            None => (0.0, EngineControls::default()),
        }
    }

    /// The same script stretched or squeezed to a total length.
    ///
    /// Every leg keeps its share of the whole, so the shape of the cycle
    /// survives. A bounce rate does not scale: a limiter bounces at the rate a
    /// limiter bounces at, however long the script around it runs.
    pub fn scaled_to(mut self, seconds: f64) -> Self {
        let total = self.seconds();
        if total > 0.0 && seconds > 0.0 {
            let factor = seconds / total;
            for segment in &mut self.segments {
                segment.seconds *= factor;
            }
        }
        self
    }
}

// ---------------------------------------------------------------------------
// The fixed profiles
// ---------------------------------------------------------------------------

/// The six profiles every preset is measured through.
pub fn catalogue(preset: &EnginePreset) -> Vec<RenderScript> {
    vec![
        idle_hold(preset),
        sweep_up(preset),
        sweep_down(preset),
        tip_in(preset),
        overrun_cut(preset),
        limiter_bounce(preset),
    ]
}

/// Idle, held. The noise floor and the mechanical layer with nothing over them.
pub fn idle_hold(preset: &EnginePreset) -> RenderScript {
    RenderScript::new(
        "idle_hold",
        "idle held, for the noise floor and the mechanical layer",
        vec![Segment::hold(4.0, preset.idle, IDLE_THROTTLE)],
    )
}

/// A pull from idle to the redline.
///
/// The one profile that reads a resonance: every order is dragged through every
/// fixed mode in the exhaust and the intake, so anything that stays put while
/// the orders sweep past it is plumbing.
pub fn sweep_up(preset: &EnginePreset) -> RenderScript {
    RenderScript::new(
        "sweep_up",
        "idle to redline, for the resonances the orders sweep past",
        vec![
            Segment::hold(1.0, preset.idle, 0.1),
            Segment::ramp(8.0, (preset.idle, preset.redline), (0.15, 1.0)),
        ],
    )
}

/// The redline back down to idle, on a closing throttle.
pub fn sweep_down(preset: &EnginePreset) -> RenderScript {
    RenderScript::new(
        "sweep_down",
        "redline to idle, the same modes approached from above",
        vec![
            Segment::hold(1.0, preset.redline, 1.0),
            Segment::ramp(8.0, (preset.redline, preset.idle), (1.0, 0.1)),
        ],
    )
}

/// A throttle step with the engine still slow.
///
/// The step is at a leg boundary and is therefore instantaneous, which is the
/// point: everything downstream has to reach its new state on its own time
/// constants rather than being ramped there.
pub fn tip_in(preset: &EnginePreset) -> RenderScript {
    let top = preset.idle + 0.6 * (preset.redline - preset.idle);
    RenderScript::new(
        "tip_in",
        "a throttle step from idle, for transient response",
        vec![
            Segment::hold(2.0, preset.idle, 0.05),
            Segment::ramp(3.0, (preset.idle, top), (1.0, 1.0)),
            Segment::hold(1.0, top, 1.0),
        ],
    )
}

/// A lift with the ignition cut: unburnt fuel down a hot pipe.
pub fn overrun_cut(preset: &EnginePreset) -> RenderScript {
    let high = preset.idle + 0.85 * (preset.redline - preset.idle);
    let low = preset.idle + 0.35 * (preset.redline - preset.idle);
    RenderScript::new(
        "overrun_cut",
        "a lift on a cut ignition, for the backfire path",
        vec![
            Segment::hold(2.0, high, 1.0),
            Segment::ramp(3.0, (high, low), (0.0, 0.0)).cutting(),
            Segment::ramp(2.0, (low, preset.idle), (0.05, IDLE_THROTTLE)),
        ],
    )
}

/// Wide open against the limiter, then off it.
///
/// The hardest thing the synth is asked to do: the highest firing rate the
/// engine reaches, a cut ignition, and a speed that reverses several times a
/// second underneath it.
pub fn limiter_bounce(preset: &EnginePreset) -> RenderScript {
    RenderScript::new(
        "limiter_bounce",
        "wide open on the limiter, the hardest case for the synth",
        vec![
            Segment::ramp(
                1.5,
                (
                    preset.idle + 0.9 * (preset.redline - preset.idle),
                    preset.redline,
                ),
                (0.8, 1.0),
            ),
            Segment::hold(4.0, preset.redline, 1.0)
                .cutting()
                .bouncing(0.02 * preset.redline, 6.0),
            Segment::ramp(1.5, (preset.redline, preset.idle), (0.0, IDLE_THROTTLE)),
        ],
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v8() -> EnginePreset {
        EnginePreset::cross_plane_v8()
    }

    #[test]
    fn the_catalogue_is_the_six_named_profiles() {
        let preset = v8();
        let scripts = catalogue(&preset);
        let names: Vec<_> = scripts.iter().map(|s| s.name).collect();
        assert_eq!(
            names,
            [
                "idle_hold",
                "sweep_up",
                "sweep_down",
                "tip_in",
                "overrun_cut",
                "limiter_bounce"
            ]
        );
        assert!(scripts.iter().all(|s| s.seconds() > 0.0));
    }

    /// A script is a function of time and nothing else, so the same instant
    /// gives the same answer however many times it is asked.
    #[test]
    fn a_script_is_a_pure_function_of_time() {
        let preset = v8();
        let script = sweep_up(&preset);
        for step in 0..500 {
            let t = step as f64 * 0.01;
            let (rpm, controls) = script.at(t);
            let (again, controls_again) = script.at(t);
            assert_eq!(rpm.to_bits(), again.to_bits());
            assert_eq!(
                controls.throttle.to_bits(),
                controls_again.throttle.to_bits()
            );
            assert_eq!(controls.spark_cut, controls_again.spark_cut);
        }
    }

    /// Every profile stays inside the engine it was built for.
    #[test]
    fn no_profile_leaves_the_rev_range() {
        for preset in EnginePreset::catalogue() {
            for script in catalogue(&preset) {
                let steps = 2_000;
                for step in 0..=steps {
                    let t = script.seconds() * step as f64 / steps as f64;
                    let (rpm, controls) = script.at(t);
                    assert!(
                        rpm > 0.0 && rpm <= preset.redline + 1e-9,
                        "{} on {} asked for {rpm:.0} rpm against a {:.0} redline",
                        preset.name,
                        script.name,
                        preset.redline
                    );
                    assert!((0.0..=1.0).contains(&controls.throttle));
                }
            }
        }
    }

    /// The limiter bounce dips below the redline and comes back, several times.
    #[test]
    fn the_limiter_bounce_bounces() {
        let preset = v8();
        let script = limiter_bounce(&preset);

        // Sample the leg that is on the limiter, which starts 1.5 s in.
        let speeds: Vec<f64> = (0..400)
            .map(|i| script.at(1.5 + i as f64 * 0.01).0)
            .collect();
        let reversals = speeds
            .windows(3)
            .filter(|w| (w[1] - w[0]).signum() != (w[2] - w[1]).signum())
            .count();
        assert!(
            reversals > 20,
            "the limiter leg reversed {reversals} times in four seconds"
        );

        let lowest = speeds.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!(
            lowest < preset.redline - 0.01 * preset.redline,
            "the bounce never left the redline"
        );
    }

    /// Scaling changes how long a cycle takes, not what it does.
    #[test]
    fn scaling_preserves_the_shape_of_a_script() {
        let preset = v8();
        let full = sweep_up(&preset);
        let half = sweep_up(&preset).scaled_to(full.seconds() / 2.0);

        assert!((half.seconds() - full.seconds() / 2.0).abs() < 1e-9);
        for step in 0..=100 {
            let u = step as f64 / 100.0;
            let (fast, _) = half.at(u * half.seconds());
            let (slow, _) = full.at(u * full.seconds());
            assert!(
                (fast - slow).abs() < 1.0,
                "at {:.0} % the scaled script was at {fast:.0} rpm and the \
                 original at {slow:.0}",
                u * 100.0
            );
        }
    }

    /// Past its end a script holds, rather than falling to a stalled engine.
    #[test]
    fn a_script_holds_past_its_end() {
        let preset = v8();
        let script = idle_hold(&preset);
        let (rpm, _) = script.at(script.seconds() * 10.0);
        assert!((rpm - preset.idle).abs() < 1e-9);
    }
}
