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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shaft_order_is_a_third_of_rotor_order() {
        assert!((RotorGeometry::shaft_order(3.0) - 1.0).abs() < 1e-9);
        assert!((RotorGeometry::shaft_order(9.0) - 3.0).abs() < 1e-9);
    }
}
