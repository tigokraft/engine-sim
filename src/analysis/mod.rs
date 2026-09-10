//! The measurement harness: turning a change in timbre into a number.
//!
//! Everything in [`physics`](crate::physics) and [`audio`](crate::audio) makes a
//! claim about a spectrum — that a V12 sits an octave above a four, that a
//! collector reflects a third of what reaches it, that an intake peaks at its
//! ram frequency. Until those can be read back out of the rendered audio they
//! are opinions, and a change that made the engine sound *different* is
//! indistinguishable from one that made it sound *right*.
//!
//! This module exists to make them falsifiable, and it deliberately holds no
//! synthesis of its own: nothing here may change what the engine sounds like,
//! only what can be said about it.
//!
//! - [`render`] — the offline render: the physics and the synth stepped by a
//!   script against a virtual clock, and the WAV it is written to.
//! - [`script`] — the fixed drive cycles a preset is measured through, each a
//!   pure function of elapsed time.
//! - [`orders`] — a Hann-windowed STFT; the level of each engine order read off
//!   it against the speed curve the render was driven by, and peak picking on
//!   the long-term average spectrum for the pipe resonances that stay put while
//!   the orders sweep past them.
//!
//! The harness is worth something only if a rerun is comparable to the run
//! before it, so every part of it is deterministic. The drive scripts are fixed
//! functions of elapsed time rather than of the wall clock, the physics is a
//! fixed-step solve, and the synth's stochastic layers run off a seeded
//! xorshift. Two renders of one script on one build are bit-identical, which is
//! what makes a difference between two builds attributable to the build.

pub mod orders;
pub mod render;
pub mod script;
