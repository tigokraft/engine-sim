//! The Wankel rotor: the eccentric shaft's fixed ratio to the rotor it
//! drives, and the port profiles that stand in for an epitrochoid housing's
//! porting.
//!
//! [`EnginePreset::two_rotor_wankel`](crate::bench::EnginePreset::two_rotor_wankel)
//! states the scope decision this module makes: the chamber volume curve
//! stays the long-rod slider-crank approximation, because it is already close
//! to the sinusoid a rotor's volume genuinely traces, and re-deriving it from
//! an epitrochoid would spend a stage on geometry nobody can hear. What a
//! listener *can* hear is how fast a port uncovers, and that is porting, not
//! geometry — so this module is almost entirely about ports.

/// The eccentric shaft's fixed ratio to the rotor it drives.
///
/// Not a tuning parameter: every Wankel's phasing gears are cut 2:3, so the
/// eccentric shaft turns exactly three times for every revolution the rotor
/// itself makes. Firing, and every port timing in this crate's cycle
/// coordinates, already sits at the correct shaft rate — see
/// [`FiringOrder::two_rotor_wankel`](crate::physics::engine_block::FiringOrder::two_rotor_wankel),
/// which puts four firings across 720 degrees of *shaft* rotation because
/// that is what the block's cycle actually is. What was *not* at the right
/// rate is the mechanical noise floor: a rotor's bearings, its seals and its
/// own mass physically vibrate at rotor speed, one third of the shaft's, and
/// [`RotorGeometry::shaft_order`] is the conversion
/// [`crate::audio::dsp::MechanicalSpec::rotary`] uses to put them there
/// instead of at shaft rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RotorGeometry;

impl RotorGeometry {
    /// Eccentric shaft revolutions per rotor revolution [-].
    pub const SHAFT_TO_ROTOR_RATIO: f64 = 3.0;

    /// Converts a mechanical order specified at rotor rate — where a rotor's
    /// bearings, seals and its own mass actually vibrate — to the equivalent
    /// order at eccentric shaft rate, which is what
    /// [`ImpulsiveSpec::order`](crate::audio::dsp::ImpulsiveSpec::order) is
    /// quoted against for every other engine in the catalogue.
    pub fn shaft_order(rotor_order: f32) -> f32 {
        rotor_order / Self::SHAFT_TO_ROTOR_RATIO as f32
    }
}

use crate::physics::cylinder::deg;
use crate::physics::thermodynamics::{ValveEvent, ValveTrain, RAISED_COSINE_RAMP};

/// Lobe separation every named port profile shares [rad, camshaft-style mean
/// of the two centrelines — see [`ValveTrain::lobe_separation`]].
///
/// Held fixed, at the two-rotor preset's original timing, so the four
/// profiles below differ only in duration (and so overlap) and opening rate
/// — the two things Stage M5 of `docs/MECHANISM_PLAN.md` actually claims a
/// port type differs in — and not in when the ports are centred, which would
/// confound the comparison.
fn port_lobe_separation() -> f64 {
    deg(100.0)
}

/// Builds a port pair at a stated duration and opening rate.
///
/// Realised as a [`ValveTrain`] because the solver already integrates gas
/// exchange through a curtain-area [`ValveEvent`], and a housing port's
/// instantaneous open area grows the same way a poppet valve's does: the
/// same two-flank raised cosine [`ValveEvent::lift`] already is, only
/// reached over a handful of degrees instead of sixty for the aggressive
/// profiles. [`ValveEvent::with_port_ramp_fraction`] is what makes that
/// legal — a housing port has no spring and no lifter to survive, so it is
/// not held to the valvetrain limit [`ValveEvent::with_ramp_fraction`] is.
///
/// Lift, diameter and discharge coefficient are the two-rotor preset's
/// original values, held fixed across every profile on purpose: peak area is
/// not the claim here, opening rate and overlap are, and changing area too
/// would confound the comparison the way moving the lobe separation would.
fn port_train(duration: f64, ramp_fraction: f64) -> ValveTrain {
    ValveTrain {
        intake: ValveEvent::new(0.0, duration, 0.014, 0.048, 0.70)
            .with_port_ramp_fraction(ramp_fraction),
        exhaust: ValveEvent::new(0.0, duration, 0.013, 0.042, 0.68)
            .with_port_ramp_fraction(ramp_fraction),
    }
    .with_cam_timing(port_lobe_separation(), 0.0)
}

/// Side port: uncovered by the rotor's own flat face, gradually, over many
/// degrees. The softest and most streetable profile — the one a stock idle
/// governor is expected to hold without hunting.
///
/// Calibrated by measurement, not guessed: this is the mildest point on the
/// duration/ramp grid this preset's reversion model will run at all, verified
/// by rendering the two-rotor block through both a full free-rev and a
/// closed-throttle idle hold before it was picked. See
/// [`peripheral_port`] for why the other three profiles stop well short of
/// [`PERIPHERAL_PORT_RAMP`](crate::physics::thermodynamics::PERIPHERAL_PORT_RAMP)'s nominal floor.
pub fn side_port() -> ValveTrain {
    port_train(deg(230.0), RAISED_COSINE_RAMP)
}

/// Bridge port: a larger side port with a bridge of material still holding
/// the housing together across the opening. Longer duration and a faster
/// edge than a side port — the beginning of the lope, and unable to hold the
/// governor's idle target at all.
pub fn bridge_port() -> ValveTrain {
    port_train(deg(240.0), 0.35)
}

/// Half (J-) bridge port: between a bridge and a peripheral port in both
/// duration and opening rate.
pub fn half_bridge_port() -> ValveTrain {
    port_train(deg(250.0), 0.28)
}

/// Peripheral port: uncovered by the apex seal sweeping past a hole in the
/// rotor housing rather than by the rotor's own face, which is why it opens
/// far faster than any of the other three profiles.
///
/// [`PERIPHERAL_PORT_RAMP`](crate::physics::thermodynamics::PERIPHERAL_PORT_RAMP) (0.025) is the floor
/// [`ValveEvent::with_port_ramp_fraction`] allows a port to reach, and it is
/// not what this profile uses: measurement showed this preset's reversion
/// model cannot sustain combustion at *any* throttle, WOT included, once
/// duration and ramp are both pushed to their individual extremes at once —
/// the two axes interact multiplicatively on reversion mass, not
/// additively. `0.22` is the steepest edge, at this duration, this
/// particular preset's block mass, inertia and load actually run on; going
/// further made the free-rev test fail outright rather than produce a
/// peripheral port that merely idles badly. That the four-way comparison
/// still lands cleanly — this profile alone fails to hold the idle governor
/// while running clean to redline — is Stage M5's actual headline result,
/// and it came from rendering the block, not from asserting a number.
pub fn peripheral_port() -> ValveTrain {
    port_train(deg(260.0), 0.22)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shaft_order_is_a_third_of_rotor_order() {
        assert!((RotorGeometry::shaft_order(3.0) - 1.0).abs() < 1e-9);
        assert!((RotorGeometry::shaft_order(9.0) - 3.0).abs() < 1e-9);
    }

    #[test]
    fn named_profiles_widen_in_the_stated_order() {
        let overlaps = [
            side_port().overlap(),
            bridge_port().overlap(),
            half_bridge_port().overlap(),
            peripheral_port().overlap(),
        ];
        assert!(
            overlaps.windows(2).all(|w| w[0] < w[1]),
            "port overlap must widen side < bridge < half-bridge < peripheral: {:?}",
            overlaps.map(f64::to_degrees)
        );
    }

    /// Peak `dA/dtheta` across a valve event's opening flank, by finite
    /// difference on [`ValveEvent::effective_area`] itself rather than on the
    /// closed-form [`ValveEvent::ramp_rate`] ratio — measured, not asserted.
    fn peak_opening_slope(event: &ValveEvent) -> f64 {
        let step = 1e-5;
        let flank = event.ramp_fraction * event.duration;
        let samples = 500;
        (0..=samples)
            .map(|i| {
                let theta = event.open_angle + flank * (i as f64 / samples as f64);
                let a1 = event.effective_area(theta - step);
                let a2 = event.effective_area(theta + step);
                ((a2 - a1) / (2.0 * step)).abs()
            })
            .fold(0.0, f64::max)
    }

    #[test]
    fn peripheral_port_opens_faster_than_side_port() {
        // The stage's headline claim: how fast the port uncovers, measured
        // as the steepest instantaneous flow-area slope either port's
        // intake event reaches on its way to full open. Measured at ~2.0x;
        // the assertion leaves margin rather than pinning the exact figure.
        let side = peak_opening_slope(&side_port().intake);
        let peripheral = peak_opening_slope(&peripheral_port().intake);
        let ratio = peripheral / side;
        assert!(
            ratio > 1.5,
            "a peripheral port must uncover area far faster than a side port: \
             {ratio:.2}x ({peripheral:.5} against {side:.5} m^2/rad)"
        );
    }
}
