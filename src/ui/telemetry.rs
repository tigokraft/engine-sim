//! What crosses between the simulation thread and the dashboard.
//!
//! Three channels, each shaped by what it carries:
//!
//! - [`Command`] — keystrokes, over an `mpsc` channel. Rare, small, and must not
//!   be lost.
//! - [`Telemetry`] — the current state of the engine, behind a mutex. Published
//!   at the frame rate and always read whole, so a lock is the right tool: the
//!   dashboard wants a *consistent* set of numbers, not the freshest possible
//!   value of each one independently.
//! - [`SimEvent`] — the audio device coming up or failing, over an `mpsc`
//!   channel, because it happens at start-up and on preset changes rather than
//!   continuously.
//!
//! The mutex here is safe in a way the one in the audio callback would not be:
//! it is shared between two ordinary threads, and neither of them has a
//! deadline. Nothing in the real-time audio path ever touches it.

use std::sync::{Arc, Mutex};

use crate::audio::{AudioScope, StreamInfo};
use crate::bench::{DynoMode, DynoRun};
use crate::physics::control::{CylinderHealth, LimiterCut, LimiterMode};
use crate::physics::engine_block::ManifoldMode;

/// Shared handle to the published telemetry.
pub type SharedTelemetry = Arc<Mutex<Telemetry>>;

// ---------------------------------------------------------------------------
// Commands and events
// ---------------------------------------------------------------------------

/// What a keystroke asks the simulation to do.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Command {
    /// Move the pedal by a delta, clamped into `0..=1` at the far end.
    Throttle(f64),
    /// Toggle the driver's ignition cut (launch control / two-step).
    ToggleCut,
    /// Load catalogue entry `n`, rebuilding the block and the audio stream.
    SelectPreset(usize),
    /// Toggle audio mute by pausing the output stream.
    ToggleMute,
    /// Shut down: stop the stream, then the thread.
    Quit,
    /// Set dyno loading mode directly.
    SetDynoMode(DynoMode),
    /// Trigger an automated dyno sweep pull.
    TriggerDynoPull,
    /// Toggle closed-loop RPM hold on/off at current engine RPM.
    ToggleRpmHold,
    /// Adjust held RPM target by delta [rev/min].
    AdjustHeldRpm(f64),
    /// Manual spark advance trim delta [deg].
    TrimSpark(f64),
    /// Manual AFR trim delta [-].
    TrimAfr(f64),
    /// Reset manual spark and AFR trims to 0.0.
    ResetTrims,
    /// Cycle rev limiter mode (Hard -> Soft -> Rotating).
    CycleLimiterMode,
    /// Cycle rev limiter cut type (Spark -> Fuel).
    CycleLimiterCut,
    /// Toggle cylinder health for cylinder index.
    ToggleCylinder(usize),
}

/// Something that happened to the simulation which the dashboard should show.
#[derive(Debug)]
pub enum SimEvent {
    /// An output stream opened. Carries the tap the spectrum reads.
    ///
    /// `cpal::Stream` is not `Send` on every platform, so the stream is built
    /// and owned entirely by the simulation thread; only the scope crosses over.
    AudioReady {
        /// What the device negotiated.
        info: Box<StreamInfo>,
        /// The tap on the rendered output.
        scope: Box<AudioScope>,
    },
    /// No audio: the device was missing, busy, or refused every configuration.
    ///
    /// The dashboard keeps running. An engine simulator with no sound card is
    /// still a working engine simulator, and saying so is better than exiting.
    AudioFailed(String),
    /// The simulation thread has finished and the stream is closed.
    Stopped,
}

// ---------------------------------------------------------------------------
// The torque and power trace
// ---------------------------------------------------------------------------

/// Width of one bucket in the torque trace [rev/min].
pub const CURVE_BUCKET_RPM: f64 = 250.0;
/// Number of buckets, covering the dashboard's 0-9000 rpm span.
pub const CURVE_BUCKETS: usize = 36;

/// One measured point on the torque curve.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurvePoint {
    /// Bucket centre [rev/min].
    pub rpm: f64,
    /// Brake torque [N m].
    pub torque: f64,
    /// Brake power [kW].
    pub power: f64,
}

/// The torque curve, filled in as the engine sweeps through the range.
///
/// This is not a swept dyno run computed up front — it is a record of what the
/// block actually solved at each speed the engine has visited, which is why it
/// appears a section at a time as you rev. The value recorded is
/// [`EngineBlock::mean_brake_torque`], the torque the block makes at that speed
/// with the cylinder full; the pedal scales torque downstream in
/// [`Driveline`](crate::bench::Driveline) and is deliberately not applied here,
/// so what the curve shows is the engine's capability rather than the driver's
/// current request.
///
/// [`EngineBlock::mean_brake_torque`]: crate::physics::engine_block::EngineBlock::mean_brake_torque
#[derive(Debug, Clone)]
pub struct CurveTrace {
    buckets: Vec<Option<CurvePoint>>,
}

impl Default for CurveTrace {
    fn default() -> Self {
        Self::new()
    }
}

impl CurveTrace {
    /// An empty trace.
    pub fn new() -> Self {
        Self {
            buckets: vec![None; CURVE_BUCKETS],
        }
    }

    /// Forgets everything, for a preset change.
    pub fn clear(&mut self) {
        self.buckets.iter_mut().for_each(|slot| *slot = None);
    }

    /// Records a solved torque at a speed.
    ///
    /// Successive visits to the same bucket are smoothed rather than replaced.
    /// The block's torque wanders by a few percent within a cycle — it is
    /// integrated over a phase ring that is still being rewritten — and an
    /// unsmoothed trace jitters enough to be hard to read.
    pub fn record(&mut self, rpm: f64, torque: f64) {
        if !rpm.is_finite() || !torque.is_finite() || rpm < 0.0 {
            return;
        }
        let index = (rpm / CURVE_BUCKET_RPM) as usize;
        let Some(slot) = self.buckets.get_mut(index) else {
            return;
        };

        let centre = (index as f64 + 0.5) * CURVE_BUCKET_RPM;
        let power = torque * centre * std::f64::consts::PI / 30.0 / 1_000.0;
        match slot {
            Some(point) => {
                point.torque += (torque - point.torque) * 0.05;
                point.power = point.torque * centre * std::f64::consts::PI / 30.0 / 1_000.0;
            }
            None => {
                *slot = Some(CurvePoint {
                    rpm: centre,
                    torque,
                    power,
                })
            }
        }
    }

    /// The measured points, in speed order.
    pub fn points(&self) -> impl Iterator<Item = &CurvePoint> {
        self.buckets.iter().flatten()
    }

    /// How many buckets have been visited.
    pub fn len(&self) -> usize {
        self.buckets.iter().flatten().count()
    }

    /// Whether nothing has been recorded yet.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Peak torque measured so far, if any.
    pub fn peak_torque(&self) -> Option<CurvePoint> {
        self.points()
            .copied()
            .reduce(|best, p| if p.torque > best.torque { p } else { best })
    }

    /// Peak power measured so far, if any.
    pub fn peak_power(&self) -> Option<CurvePoint> {
        self.points()
            .copied()
            .reduce(|best, p| if p.power > best.power { p } else { best })
    }

    /// Copies another trace's buckets into this one without reallocating.
    pub fn copy_from(&mut self, other: &CurveTrace) {
        self.buckets.copy_from_slice(&other.buckets);
    }
}

// ---------------------------------------------------------------------------
// Health
// ---------------------------------------------------------------------------

/// What the audio path is doing, as far as the dashboard needs to know.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct AudioHealth {
    /// Device name, empty if no stream ever opened.
    pub device: String,
    /// Negotiated rate [Hz].
    pub sample_rate: u32,
    /// Negotiated channel count.
    pub channels: u16,
    /// Whether the stream is running (mute pauses it).
    pub playing: bool,
    /// Whether a stream exists at all.
    pub open: bool,
    /// Fraction of callbacks that found no fresh physics, `0..=1`.
    pub starvation: f64,
    /// Peak magnitude of the last callback.
    pub peak: f32,
    /// Snapshots dropped because the queue was full.
    pub dropped: u64,
    /// Stream errors reported by the backend.
    pub errors: u64,
    /// Whether the device has gone away.
    pub disconnected: bool,
}

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------

/// Everything the dashboard draws, published once per rendered frame.
#[derive(Debug, Clone)]
pub struct Telemetry {
    /// Index of the selected catalogue entry.
    pub preset: usize,
    /// Its display name.
    pub preset_name: String,
    /// Its one-line character note.
    pub preset_note: String,
    /// Its spec line.
    pub preset_spec: String,
    /// How the selected engine breathes, e.g. `"turbocharged"`.
    pub induction: &'static str,
    /// Cylinder or chamber count.
    pub cylinders: usize,
    /// Exhaust bank count.
    pub banks: usize,
    /// Limiter speed [rev/min].
    pub redline: f64,

    /// Crankshaft speed [rev/min].
    pub rpm: f64,
    /// Throttle actually applied [-].
    pub throttle: f64,
    /// Where the pedal is being asked to go [-].
    pub throttle_target: f64,
    /// Whether the driver is holding the cut.
    pub manual_cut: bool,
    /// Whether the limiter is cutting.
    pub limiter: bool,
    /// Whether audio is muted.
    pub muted: bool,

    /// Intake plenum pressure [Pa].
    pub map_pa: f64,
    /// Ambient pressure [Pa].
    pub ambient_pa: f64,
    /// Exhaust port pressure, averaged over the banks [Pa].
    pub exhaust_pa: f64,
    /// Exhaust gas temperature, averaged over the banks [K].
    pub exhaust_k: f64,
    /// Which acoustic mode each exhaust bank is in.
    pub manifold_modes: Vec<ManifoldMode>,

    /// Brake torque at the current speed [N m].
    pub torque: f64,
    /// Brake power at the current speed [kW].
    pub power_kw: f64,
    /// Indicated mean effective pressure [Pa].
    pub imep: f64,
    /// Brake mean effective pressure [Pa].
    pub bmep: f64,
    /// Peak cylinder pressure over the logged cycle [Pa].
    pub peak_pressure: f64,
    /// Mean piston speed [m/s]. A production engine lives below about 25.
    pub mean_piston_speed: f64,
    /// Livengood-Wu knock integral, `>= 1` is autoignition [-].
    pub knock_integral: f64,
    /// Whether the charge is knocking.
    pub knocking: bool,
    /// Ignition delay this cycle's charge took to light itself [s].
    ///
    /// `None` on a spark engine, where the coil decides and there is no delay
    /// to solve. The same Arrhenius integral as the knock one above, read the
    /// other way up: on a petrol engine reaching one is the failure mode, and
    /// on a diesel it is how the engine runs at all. See
    /// [`crate::physics::thermodynamics::Autoignition`].
    pub ignition_delay: Option<f64>,

    /// Whether this engine has a turbo at all; see [`crate::audio::Induction`].
    ///
    /// Without one there is no shaft, and the two readings below stay at zero
    /// because nothing is turning — not because nothing is happening.
    pub turbo_fitted: bool,
    /// Audio-path turbo shaft speed [rev/min]; see [`crate::audio::TurboModel`].
    pub turbo_rpm: f64,
    /// How far into surge the compressor is, `0..=1`.
    pub turbo_surge: f64,

    /// The pressure-volume loop: `(volume [cc], pressure [bar])`.
    pub pv: Vec<(f64, f64)>,
    /// The torque and power curve measured so far.
    pub curve: CurveTrace,

    /// RK4 substeps in the last frame.
    pub substeps: usize,
    /// Angular substep actually used [deg].
    pub dtheta_deg: f64,
    /// Measured simulation frame rate [Hz].
    pub physics_hz: f64,
    /// Whether the phase ring has filled; before it does, the P-V loop is
    /// partly stale and the torque numbers are not yet meaningful.
    pub ring_primed: bool,
    /// Whether the scheduler accepted a real-time priority for this thread.
    pub realtime: bool,

    /// Audio path health.
    pub audio: AudioHealth,

    // --- Dyno loading & test cell ------------------------------------------
    /// Current dyno absorber operating mode.
    pub dyno_mode: DynoMode,
    /// Torque exerted by the dyno brake absorber [N m].
    pub dyno_absorber_torque: f64,
    /// Completed dyno sweep pull run, if any.
    pub last_pull: Option<DynoRun>,

    // --- Combustion diagnostics & efficiency ------------------------------
    /// Location of peak cylinder pressure [deg ATDC].
    pub lpp_deg_atdc: f64,
    /// Volumetric efficiency of cylinder charging [-].
    pub volumetric_efficiency: f64,
    /// Brake specific fuel consumption [g / (kW h)].
    pub bsfc_g_kwh: f64,

    // --- Thermal & fluid circuits ------------------------------------------
    /// Coolant bulk temperature [K].
    pub coolant_k: f64,
    /// Oil gallery temperature [K].
    pub oil_k: f64,
    /// Cylinder head metal temperature [K].
    pub head_k: f64,
    /// Oil pressure [bar].
    pub oil_pressure_bar: f64,

    // --- Calibration & ECU trims -------------------------------------------
    /// Base ignition advance before trims [deg BTDC].
    pub spark_advance_deg: f64,
    /// Knock closed-loop retard applied [deg].
    pub knock_retard_deg: f64,
    /// Manual spark calibration trim [deg].
    pub spark_trim: f64,
    /// Manual AFR calibration trim [-].
    pub afr_trim: f64,
    /// Commanded / actual air-fuel ratio [-].
    pub actual_afr: f64,
    /// Active rev limiter strategy.
    pub limiter_mode: LimiterMode,
    /// Active rev limiter cut mechanism.
    pub limiter_cut: LimiterCut,
    /// Per-cylinder operating health status.
    pub cylinder_health: Vec<CylinderHealth>,
}

impl Default for Telemetry {
    fn default() -> Self {
        Self {
            preset: 0,
            preset_name: String::new(),
            preset_note: String::new(),
            preset_spec: String::new(),
            induction: "naturally aspirated",
            cylinders: 0,
            banks: 0,
            redline: 7_000.0,
            rpm: 0.0,
            throttle: 0.0,
            throttle_target: 0.0,
            manual_cut: false,
            limiter: false,
            muted: false,
            map_pa: 0.0,
            ambient_pa: crate::environment::STANDARD_PRESSURE,
            exhaust_pa: 0.0,
            exhaust_k: 0.0,
            manifold_modes: Vec::new(),
            torque: 0.0,
            power_kw: 0.0,
            imep: 0.0,
            bmep: 0.0,
            peak_pressure: 0.0,
            mean_piston_speed: 0.0,
            knock_integral: 0.0,
            knocking: false,
            ignition_delay: None,
            turbo_fitted: false,
            turbo_rpm: 0.0,
            turbo_surge: 0.0,
            pv: Vec::new(),
            curve: CurveTrace::new(),
            substeps: 0,
            dtheta_deg: 0.0,
            physics_hz: 0.0,
            ring_primed: false,
            realtime: false,
            audio: AudioHealth::default(),
            dyno_mode: DynoMode::FreeRev,
            dyno_absorber_torque: 0.0,
            last_pull: None,
            lpp_deg_atdc: 0.0,
            volumetric_efficiency: 0.0,
            bsfc_g_kwh: 0.0,
            coolant_k: 293.15,
            oil_k: 293.15,
            head_k: 293.15,
            oil_pressure_bar: 0.0,
            spark_advance_deg: 0.0,
            knock_retard_deg: 0.0,
            spark_trim: 0.0,
            afr_trim: 0.0,
            actual_afr: 14.7,
            limiter_mode: LimiterMode::HardCut,
            limiter_cut: LimiterCut::None,
            cylinder_health: Vec::new(),
        }
    }
}

impl Telemetry {
    /// Manifold pressure relative to ambient [Pa]; negative is vacuum.
    pub fn manifold_gauge_pa(&self) -> f64 {
        self.map_pa - self.ambient_pa
    }

    /// Manifold depression [in.Hg], zero when the plenum is at or above ambient.
    pub fn vacuum_inhg(&self) -> f64 {
        (-self.manifold_gauge_pa()).max(0.0) / 3_386.389
    }

    /// Manifold boost [psi], zero when the plenum is at or below ambient.
    pub fn boost_psi(&self) -> f64 {
        self.manifold_gauge_pa().max(0.0) / 6_894.757
    }

    /// Brake power in horsepower, for the readout.
    pub fn power_hp(&self) -> f64 {
        self.power_kw * 1.341_022
    }

    /// Whether ignition is cut, from either cause.
    pub fn cutting(&self) -> bool {
        self.manual_cut || self.limiter
    }

    /// Coolant bulk temperature [°C].
    pub fn coolant_c(&self) -> f64 {
        self.coolant_k - 273.15
    }

    /// Oil gallery temperature [°C].
    pub fn oil_c(&self) -> f64 {
        self.oil_k - 273.15
    }

    /// Cylinder head metal temperature [°C].
    pub fn head_c(&self) -> f64 {
        self.head_k - 273.15
    }

    /// Oil gallery pressure [psi].
    pub fn oil_pressure_psi(&self) -> f64 {
        self.oil_pressure_bar * 14.503_773_773
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_trace_records_into_the_bucket_for_its_speed() {
        let mut trace = CurveTrace::new();
        trace.record(3_010.0, 300.0);
        let point = trace.points().next().copied().expect("a recorded point");
        assert_eq!(
            point.rpm, 3_125.0,
            "3010 rpm belongs to the 3000-3250 bucket"
        );
        assert_eq!(point.torque, 300.0);
        // P [kW] = tau * omega / 1000, omega = rpm * pi / 30.
        let expected = 300.0 * 3_125.0 * std::f64::consts::PI / 30.0 / 1_000.0;
        assert!((point.power - expected).abs() < 1e-9);
    }

    #[test]
    fn repeat_visits_are_smoothed_towards_the_new_value() {
        let mut trace = CurveTrace::new();
        trace.record(3_000.0, 100.0);
        for _ in 0..200 {
            trace.record(3_000.0, 200.0);
        }
        let point = trace.points().next().copied().unwrap();
        assert!(
            (point.torque - 200.0).abs() < 1.0,
            "smoothing never converged: {:.1}",
            point.torque
        );
        assert!(point.torque < 200.0, "smoothing must approach, not jump");
    }

    #[test]
    fn out_of_range_and_non_finite_input_is_dropped_not_panicked_on() {
        let mut trace = CurveTrace::new();
        trace.record(f64::NAN, 100.0);
        trace.record(4_000.0, f64::INFINITY);
        trace.record(-100.0, 100.0);
        // 12000 rpm is past the last bucket: nothing to write to, and nothing
        // to panic about either.
        trace.record(12_000.0, 100.0);
        assert!(trace.is_empty());
    }

    #[test]
    fn peaks_come_from_the_measured_points() {
        let mut trace = CurveTrace::new();
        trace.record(2_000.0, 400.0);
        trace.record(7_000.0, 320.0);
        assert_eq!(trace.peak_torque().unwrap().torque, 400.0);
        // Power is torque times speed, so the peak moves up the range.
        assert_eq!(trace.peak_power().unwrap().rpm, 7_125.0);
    }

    #[test]
    fn gauge_conversions_split_vacuum_from_boost() {
        let mut telemetry = Telemetry {
            ambient_pa: 101_325.0,
            map_pa: 101_325.0 - 33_863.89, // exactly 10 in.Hg down
            ..Telemetry::default()
        };
        assert!((telemetry.vacuum_inhg() - 10.0).abs() < 1e-6);
        assert_eq!(telemetry.boost_psi(), 0.0);

        telemetry.map_pa = 101_325.0 + 68_947.57; // exactly 10 psi up
        assert!((telemetry.boost_psi() - 10.0).abs() < 1e-6);
        assert_eq!(telemetry.vacuum_inhg(), 0.0);
    }

    #[test]
    fn telemetry_temperature_and_pressure_conversions_match_physics() {
        let t = Telemetry {
            coolant_k: 363.15, // 90 °C
            oil_k: 373.15,     // 100 °C
            head_k: 383.15,    // 110 °C
            oil_pressure_bar: 4.0,
            ..Telemetry::default()
        };
        assert!((t.coolant_c() - 90.0).abs() < 1e-6);
        assert!((t.oil_c() - 100.0).abs() < 1e-6);
        assert!((t.head_c() - 110.0).abs() < 1e-6);
        assert!((t.oil_pressure_psi() - 4.0 * 14.503_773_773).abs() < 1e-6);
    }
}
