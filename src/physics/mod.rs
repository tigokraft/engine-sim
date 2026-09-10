//! Zero-dimensional engine physics.
//!
//! - [`cylinder`] — slider-crank kinematics and the `[theta, T, m, x_b]` state.
//! - [`intake`] — the throttle body orifice and the 0D intake plenum whose
//!   pressure is the cylinders' intake boundary condition.
//! - [`thermodynamics`] — Wiebe/Woschni/valve derivatives and the RK4 solver.
//! - [`engine_block`] — the 720-cell phase ring, firing order, friction and
//!   manifolds that turn one cylinder into a whole engine.

pub mod cylinder;
pub mod engine_block;
pub mod intake;
pub mod plumbing;
pub mod thermodynamics;

pub use plumbing::*;
