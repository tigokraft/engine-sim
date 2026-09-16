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
use crate::physics::thermodynamics::{
    ValveEvent, ValveTrain, PERIPHERAL_PORT_RAMP, RAISED_COSINE_RAMP,
};

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
pub fn side_port() -> ValveTrain {
    port_train(deg(230.0), RAISED_COSINE_RAMP)
}

/// Bridge port: a larger side port with a bridge of material still holding
/// the housing together across the opening. Longer duration and a faster
/// edge than a side port — the beginning of the lope, and a brap at idle.
pub fn bridge_port() -> ValveTrain {
    port_train(deg(270.0), 0.20)
}

/// Half (J-) bridge port: between a bridge and a peripheral port in both
/// duration and opening rate.
pub fn half_bridge_port() -> ValveTrain {
    port_train(deg(300.0), 0.08)
}

/// Peripheral port: uncovered by the apex seal sweeping past a hole in the
/// rotor housing rather than by the rotor's own face, which is why it opens
/// in a handful of degrees instead of dozens. Enormous overlap and the
/// steepest edge a port in this crate is allowed — see
/// [`PERIPHERAL_PORT_RAMP`]. A peripherally ported rotary is notorious for
/// barely idling at all, and that is expected to fall straight out of this
/// profile once it is fed through the reversion and idle-governor machinery
/// Stage M4 built, not something asserted separately.
pub fn peripheral_port() -> ValveTrain {
    port_train(deg(330.0), PERIPHERAL_PORT_RAMP)
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
}
