//! The vehicle: a gearbox, the road it pushes back with, and the clutch that
//! couples the crank to both — including through the moment it cannot.
//!
//! [`crate::bench::Driveline`] integrates a bare flywheel against a load. Until
//! now that load was a quadratic in engine speed and nothing else, so the
//! engine had exactly one load line regardless of gear, road, or grade. This
//! module gives it a real one: [`Gearbox`] picks a ratio, a road load prices
//! the road at that ratio, and a clutch is the limited-torque coupling between
//! the two, which is what a standing start and a shift both are.

use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Gearbox
// ---------------------------------------------------------------------------

/// Which gear the box is in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Gear {
    /// No drive path: the crank spins free of the road entirely. This is the
    /// gear that makes today's free-revving behaviour a special case of a
    /// gearbox rather than a separate code path — see [`Gearbox::overall_ratio`].
    Neutral,
    /// A numbered gear, one-indexed the way a shift pattern is printed, into
    /// [`Gearbox::ratios`].
    Engaged(usize),
}

/// A ratio list plus a final drive, and the gear currently selected.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Gearbox {
    /// Gear ratios, first gear first: engine speed divided by gearbox output
    /// speed, before the final drive.
    pub ratios: Vec<f64>,
    /// Final drive (differential) ratio, applied after whichever gear ratio.
    pub final_drive: f64,
    /// Currently selected gear.
    pub gear: Gear,
}

impl Gearbox {
    /// Builds a gearbox in neutral from a ratio list and a final drive.
    pub fn new(ratios: Vec<f64>, final_drive: f64) -> Self {
        Self {
            ratios,
            final_drive,
            gear: Gear::Neutral,
        }
    }

    /// A generic close-ratio six-speed behind a 3.90 final drive.
    ///
    /// Not tuned per engine — Stage M1 is about the architecture existing at
    /// all, not about fitting a gearbox to each car in the catalogue.
    pub fn generic_six_speed() -> Self {
        Self::new(vec![3.36, 2.09, 1.47, 1.14, 1.00, 0.85], 3.90)
    }

    /// Overall ratio (engine turns per output turn) for the selected gear, or
    /// `None` in neutral — there is no ratio because there is no drive path.
    pub fn overall_ratio(&self) -> Option<f64> {
        match self.gear {
            Gear::Neutral => None,
            Gear::Engaged(g) => self
                .ratios
                .get(g.checked_sub(1)?)
                .map(|r| r * self.final_drive),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() <= tol, "expected {b}, got {a} (tol {tol})");
    }

    #[test]
    fn neutral_has_no_overall_ratio() {
        let gearbox = Gearbox::generic_six_speed();
        assert_eq!(gearbox.overall_ratio(), None);
    }

    #[test]
    fn overall_ratio_is_gear_times_final_drive() {
        let mut gearbox = Gearbox::new(vec![3.36, 2.09, 1.47, 1.14, 1.00, 0.85], 3.90);
        gearbox.gear = Gear::Engaged(1);
        approx(gearbox.overall_ratio().unwrap(), 3.36 * 3.90, 1e-9);
        gearbox.gear = Gear::Engaged(6);
        approx(gearbox.overall_ratio().unwrap(), 0.85 * 3.90, 1e-9);
    }

    #[test]
    fn a_gear_past_the_ratio_list_has_no_overall_ratio() {
        let mut gearbox = Gearbox::new(vec![3.0, 2.0], 4.0);
        gearbox.gear = Gear::Engaged(3);
        assert_eq!(gearbox.overall_ratio(), None);
        gearbox.gear = Gear::Engaged(0);
        assert_eq!(gearbox.overall_ratio(), None);
    }
}
