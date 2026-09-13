//! Zero-dimensional thermodynamic engine simulator.
//!
//! The crate is split so the physics can be exercised without the audio or
//! terminal front-ends:
//!
//! - [`environment`] — ambient air model (Tetens humidity, mixture density).
//! - [`physics::cylinder`] — slider-crank kinematics and the 0D cylinder state
//!   vector `[theta, T, m, x_b]`.
//! - [`physics::intake`] — the throttle body as a compressible orifice and the
//!   0D intake plenum control volume whose algebraic pressure closure is the
//!   cylinders' intake boundary condition.
//! - [`physics::thermodynamics`] — the open-system first law, Wiebe combustion,
//!   Woschni wall loss, compressible valve flow, the Livengood-Wu knock
//!   integral, and the RK4 solver that steps them.
//! - [`physics::engine_block`] — the 720-cell phase ring that turns one solved
//!   cylinder into a whole firing order, plus Chen-Flynn friction and the
//!   plenum/acoustic-pipe manifolds.
//! - [`bench`] — the rig around the engine: the selectable engine catalogue
//!   and the flywheel its torque accelerates.
//! - [`ui`] — the `ratatui` dashboard: gauges, a live P-V diagram, torque and
//!   power curves traced as the engine sweeps, and a spectrum of the audio the
//!   device is actually playing.
//! - [`analysis`] — the measurement harness: order tracking and resonance peak
//!   picking over a rendered buffer, so a change in timbre can be read as a
//!   number rather than argued about.
//! - [`audio`] — the real-time synthesis engine: blowdown excitation, runner
//!   delays, Helmholtz muffler filtering, induction noise, backfires, and a
//!   turbo on the engines that have one, fed over a lock-free queue into a
//!   `cpal` output stream.
//!
//! `cargo run --release` opens the interactive dashboard; `cargo run --release
//! --example v8_bench` runs a cross-plane V8 at 3000 rpm and prints its pressure
//! and torque profiles.
//!
//! Everything is SI unless a name says otherwise: metres, kilograms, seconds,
//! Kelvin, Pascals, radians.

pub mod analysis;
pub mod audio;
pub mod bench;
pub mod environment;
pub mod physics;
pub mod ui;
