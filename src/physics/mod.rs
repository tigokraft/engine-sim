//! Zero-dimensional engine physics.
//!
//! - [`cylinder`] — slider-crank kinematics and the `[theta, T, m, x_b]` state.
//! - [`intake`] — the throttle body orifice and the 0D intake plenum whose
//!   pressure is the cylinders' intake boundary condition.
//! - [`thermodynamics`] — Wiebe/Woschni/valve derivatives and the RK4 solver.
//! - [`engine_block`] — the 720-cell phase ring, firing order, friction and
//!   manifolds that turn one cylinder into a whole engine.
//! - [`thermal`] — the lumped masses that give the block and every pipe section
//!   a temperature of their own, so the engine can be cold.
//! - [`vehicle`] — the gearbox, road load, and clutch that give the flywheel
//!   something to pull against besides its own drag curve.

pub mod compressor;
pub mod control;
pub mod cylinder;
pub mod engine_block;
pub mod intake;
pub mod plumbing;
pub mod rotor;
pub mod thermal;
pub mod thermodynamics;
pub mod turbine;
pub mod vehicle;

pub use control::*;
pub use plumbing::*;
