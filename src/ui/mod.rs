//! The terminal dashboard.
//!
//! - [`terminal`] — layout, widgets and key bindings for the live dashboard.
//! - [`spectrum`] — the FFT analyser behind the audio display.
//! - [`telemetry`] — the snapshot the physics thread publishes for the UI, and
//!   the commands that go back the other way.

pub mod spectrum;
pub mod telemetry;
pub mod terminal;
