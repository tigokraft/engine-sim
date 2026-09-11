//! The live dashboard: layout, widgets and key bindings.
//!
//! ```text
//! ┌ engine · device · health ─────────────────────────────────────────────┐
//! ├──────────────────────┬────────────────────────────────────────────────┤
//! │  _   _   _   _       │                                                │
//! │ |_| |_| |_| |_|  rpm │   P-V diagram, one whole 720 degree cycle      │
//! │  _|  _| |_|  _|      │   straight out of the solver's phase ring      │
//! │ [gauge ██████░░░░░░] │                                                │
//! ├──────────────────────┼────────────────────────────────────────────────┤
//! │ throttle / ignition  │   torque and power, traced as the engine       │
//! ├──────────────────────┤   sweeps through the range                     │
//! │ manifold vacuum/boost├────────────────────────────────────────────────┤
//! ├──────────────────────┤   spectrum of the audio the device is          │
//! │ cycle vitals         │   actually playing, plus a pulse meter         │
//! ├──────────────────────┤                                                │
//! │ 1-9 [ ] engine presets│                                                │
//! ├──────────────────────┴────────────────────────────────────────────────┤
//! │ key bindings                                                          │
//! └───────────────────────────────────────────────────────────────────────┘
//! ```
//!
//! The dashboard owns no simulation state. It renders a [`Telemetry`] snapshot
//! published by the simulation thread and turns keystrokes into [`Command`]s
//! sent back to it, which is what lets the physics run at its own cadence
//! without the draw rate ever pulling on it.
//!
//! The one thing it does own is the spectrum: [`SpectrumAnalyzer`] reads the
//! audio tap directly on this thread, because the analysis is a property of the
//! sound rather than of the engine, and running it here keeps a 1024-point FFT
//! off the simulation's critical path.

use std::io::{self, Stdout, Write};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use crossterm::event::{
    self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyEvent, KeyEventKind,
    KeyModifiers,
};
use crossterm::execute;
use crossterm::terminal::{
    disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen,
};
use ratatui::backend::CrosstermBackend;
use ratatui::buffer::Buffer;
use ratatui::layout::{Alignment, Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style, Stylize};
use ratatui::symbols::Marker;
use ratatui::text::{Line as TextLine, Span};
use ratatui::widgets::canvas::{Canvas, Line as CanvasLine};
use ratatui::widgets::{
    Axis, Block, Borders, Chart, Dataset, Gauge, GraphType, LegendPosition, Paragraph, Widget, Wrap,
};
use ratatui::{Frame, Terminal};

use crate::audio::AudioScope;
use crate::bench::EnginePreset;
use crate::physics::engine_block::ManifoldMode;
use crate::ui::spectrum::SpectrumAnalyzer;
use crate::ui::telemetry::{Command, Telemetry};

/// Top of the tachometer [rev/min]. Every preset's redline sits below it.
pub const GAUGE_MAX_RPM: f64 = 9_000.0;

/// Throttle change per key press [-].
///
/// A terminal reports key presses, not a key being held, so the pedal moves in
/// steps rather than being tracked continuously. Auto-repeat then turns a held
/// key into a ramp at roughly 30 %/s, which is close enough to a real pedal to
/// drive with.
pub const THROTTLE_STEP: f64 = 0.06;

/// How long the knock warning stays lit after a knocking frame [s].
pub const KNOCK_HOLD: f64 = 0.9;

/// The smallest terminal the dashboard will draw in.
pub const MIN_WIDTH: u16 = 96;
/// The smallest terminal the dashboard will draw in.
pub const MIN_HEIGHT: u16 = 30;

// ---------------------------------------------------------------------------
// Palette
// ---------------------------------------------------------------------------

const INK: Color = Color::Rgb(120, 130, 140);
const FRAME_COLOR: Color = Color::Rgb(70, 78, 88);
const LABEL: Color = Color::Rgb(150, 160, 172);
const COOL: Color = Color::Rgb(90, 190, 220);
const LIVE: Color = Color::Rgb(120, 220, 150);
const WARN: Color = Color::Rgb(230, 180, 80);
const HOT: Color = Color::Rgb(235, 95, 85);
const ACCENT: Color = Color::Rgb(190, 150, 240);

/// The four strokes, for colouring the P-V loop.
const INTAKE_COLOR: Color = COOL;
const COMPRESSION_COLOR: Color = Color::Rgb(110, 140, 235);
const POWER_COLOR: Color = HOT;
const EXHAUST_COLOR: Color = Color::Rgb(200, 120, 190);

// ---------------------------------------------------------------------------
// Terminal session
// ---------------------------------------------------------------------------

/// Owns the terminal's raw mode and alternate screen for as long as it lives.
///
/// Both are global changes to the user's terminal: leaving raw mode on turns
/// their shell unusable, and leaving the alternate screen on hides their
/// scrollback. Restoring from a destructor rather than from the end of `main`
/// means the terminal comes back from an error path and from a panic too, and
/// the panic hook exists because a panic unwinding through the draw code would
/// otherwise print its message into a raw-mode screen that is about to be torn
/// down — invisible, on a terminal the user then has to reset by hand.
pub struct TerminalSession {
    terminal: Terminal<CrosstermBackend<Stdout>>,
}

impl TerminalSession {
    /// Takes over the terminal.
    pub fn open() -> Result<Self> {
        let previous_hook = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            let _ = restore_terminal();
            previous_hook(info);
        }));

        enable_raw_mode().context("could not put the terminal into raw mode")?;
        let mut stdout = io::stdout();
        execute!(stdout, EnterAlternateScreen, EnableMouseCapture)
            .context("could not switch to the alternate screen")?;

        let mut terminal = Terminal::new(CrosstermBackend::new(stdout))
            .context("could not initialise the terminal backend")?;
        terminal.hide_cursor().ok();
        terminal.clear().ok();

        Ok(Self { terminal })
    }

    /// The underlying terminal, for drawing.
    pub fn terminal(&mut self) -> &mut Terminal<CrosstermBackend<Stdout>> {
        &mut self.terminal
    }
}

impl Drop for TerminalSession {
    fn drop(&mut self) {
        self.terminal.show_cursor().ok();
        let _ = restore_terminal();
    }
}

/// Puts the terminal back the way it was found. Safe to call twice.
fn restore_terminal() -> io::Result<()> {
    let mut stdout = io::stdout();
    execute!(stdout, LeaveAlternateScreen, DisableMouseCapture)?;
    disable_raw_mode()?;
    stdout.flush()
}

// ---------------------------------------------------------------------------
// Dashboard
// ---------------------------------------------------------------------------

/// The dashboard's own state: everything that is a property of the display
/// rather than of the engine.
pub struct Dashboard {
    /// The most recent published telemetry.
    pub telemetry: Telemetry,
    /// Catalogue names, specs and whether each is turbocharged, for the list.
    presets: Vec<(&'static str, String, bool)>,
    /// The spectrum analyser and its window.
    analyzer: SpectrumAnalyzer,
    /// The tap on the rendered audio, once a stream has opened.
    scope: Option<AudioScope>,
    /// A line about the audio path: which device, or why there is none.
    audio_status: String,
    /// Whether audio is unavailable, as opposed to merely muted.
    audio_failed: bool,
    /// Needle position, lagged behind the true speed [rev/min].
    needle: f64,
    /// Seconds the knock badge stays lit after the last knocking frame.
    knock_hold: f64,
    /// Measured draw rate [Hz].
    fps: f64,
    /// When the last frame was drawn, for the frame time.
    last_frame: Instant,
    /// Set once the driver has asked to quit.
    quitting: bool,
}

impl Default for Dashboard {
    fn default() -> Self {
        Self::new()
    }
}

impl Dashboard {
    /// A dashboard with nothing on it yet.
    pub fn new() -> Self {
        Self {
            telemetry: Telemetry::default(),
            presets: EnginePreset::catalogue()
                .into_iter()
                .map(|preset| (preset.name, preset.spec(), preset.is_turbocharged()))
                .collect(),
            analyzer: SpectrumAnalyzer::default(),
            scope: None,
            audio_status: "opening audio device...".to_string(),
            audio_failed: false,
            needle: 0.0,
            knock_hold: 0.0,
            fps: 0.0,
            last_frame: Instant::now(),
            quitting: false,
        }
    }

    /// Whether the driver has asked to quit.
    pub fn quitting(&self) -> bool {
        self.quitting
    }

    /// Attaches a tap on a newly opened stream.
    pub fn attach_scope(&mut self, scope: AudioScope, status: String) {
        self.analyzer.set_sample_rate(scope.sample_rate() as f32);
        self.analyzer.reset();
        self.scope = Some(scope);
        self.audio_status = status;
        self.audio_failed = false;
    }

    /// Records that no stream could be opened.
    pub fn audio_failed(&mut self, reason: String) {
        self.scope = None;
        self.analyzer.reset();
        self.audio_status = reason;
        self.audio_failed = true;
    }

    /// Reads the audio tap and advances the display's own animation.
    ///
    /// Separate from [`Dashboard::draw`] because it is time-dependent and a draw
    /// is not: the terminal may redraw for a resize without any time passing.
    pub fn tick(&mut self, dt: Duration) {
        let dt = dt.as_secs_f64().clamp(1e-4, 0.25);
        self.analyzer.update(self.scope.as_mut(), dt as f32);

        // The needle lags the crankshaft, the way a real one lags its drive.
        // Without it the display flickers between adjacent digits at idle,
        // where the speed genuinely oscillates within a cycle.
        let alpha = 1.0 - (-dt / 0.055).exp();
        self.needle += (self.telemetry.rpm - self.needle) * alpha;

        // Autoignition is intermittent — it happens on some cycles and not
        // others — so a badge wired straight to the current frame strobes at
        // several hertz and reads as a rendering fault rather than as knock.
        // Latching it holds the warning up long enough to be read, and it is
        // still telling the truth: the engine did knock, within the last second.
        if self.telemetry.knocking {
            self.knock_hold = KNOCK_HOLD;
        } else {
            self.knock_hold = (self.knock_hold - dt).max(0.0);
        }

        let frame = self.last_frame.elapsed().as_secs_f64();
        self.last_frame = Instant::now();
        if frame > 1e-6 {
            // Smoothed, or the readout is unreadable noise.
            self.fps += (1.0 / frame - self.fps) * 0.1;
        }
    }

    /// Translates a key press into a command for the simulation thread.
    ///
    /// Returns `None` for keys that mean nothing here. `Repeat` is accepted
    /// alongside `Press` so that holding the throttle keys ramps the pedal;
    /// `Release` is ignored because most terminals never send it.
    pub fn on_key(&mut self, key: KeyEvent) -> Option<Command> {
        if !matches!(key.kind, KeyEventKind::Press | KeyEventKind::Repeat) {
            return None;
        }

        // Ctrl-C has to work even though the terminal is in raw mode and the
        // signal is never generated.
        if key.modifiers.contains(KeyModifiers::CONTROL)
            && matches!(key.code, KeyCode::Char('c') | KeyCode::Char('C'))
        {
            self.quitting = true;
            return Some(Command::Quit);
        }

        match key.code {
            KeyCode::Char('q') | KeyCode::Char('Q') | KeyCode::Esc => {
                self.quitting = true;
                Some(Command::Quit)
            }
            KeyCode::Up | KeyCode::Char('w') | KeyCode::Char('W') => {
                Some(Command::Throttle(THROTTLE_STEP))
            }
            KeyCode::Down | KeyCode::Char('s') | KeyCode::Char('S') => {
                Some(Command::Throttle(-THROTTLE_STEP))
            }
            KeyCode::Char(' ') => Some(Command::ToggleCut),
            KeyCode::Char('m') | KeyCode::Char('M') => Some(Command::ToggleMute),
            KeyCode::Char(c @ '1'..='9') => {
                let index = (c as u8 - b'1') as usize;
                (index < self.presets.len()).then_some(Command::SelectPreset(index))
            }
            KeyCode::Char('[') => self.step_preset(-1),
            KeyCode::Char(']') => self.step_preset(1),
            _ => None,
        }
    }

    /// The engine `delta` places along the catalogue from the one running.
    ///
    /// The digit row runs out before the catalogue does — there are ten fingers
    /// and nine digits, and more engines than either — so this is the only way
    /// to reach the ones past the ninth. It wraps, because a list you can fall
    /// off the end of is a list you have to count your way back up.
    fn step_preset(&self, delta: isize) -> Option<Command> {
        let count = self.presets.len();
        if count == 0 {
            return None;
        }
        let index = (self.telemetry.preset as isize + delta).rem_euclid(count as isize);
        Some(Command::SelectPreset(index as usize))
    }

    /// Reads whatever input is already queued, without blocking.
    ///
    /// Everything pending is drained in one go rather than one event per frame:
    /// key auto-repeat can outrun 60 fps, and a queue that is only ever drained
    /// one deep would lag further behind the keyboard the longer a key is held.
    pub fn drain_input(&mut self, mut send: impl FnMut(Command)) -> Result<()> {
        while event::poll(Duration::ZERO).context("could not poll the terminal for input")? {
            match event::read().context("could not read a terminal event")? {
                Event::Key(key) => {
                    if let Some(command) = self.on_key(key) {
                        send(command);
                    }
                }
                // A resize needs no handling: the next draw reads the new size.
                Event::Resize(_, _) => {}
                _ => {}
            }
        }
        Ok(())
    }

    // -----------------------------------------------------------------------
    // Drawing
    // -----------------------------------------------------------------------

    /// Draws one frame.
    pub fn draw(&self, frame: &mut Frame) {
        let area = frame.area();
        if area.width < MIN_WIDTH || area.height < MIN_HEIGHT {
            frame.render_widget(too_small(area), area);
            return;
        }

        let [header, body, footer] = Layout::vertical([
            Constraint::Length(3),
            Constraint::Min(0),
            Constraint::Length(3),
        ])
        .areas(area);

        // The left column is fixed: its contents are readouts of a known width,
        // and letting them stretch would only add whitespace. Everything that
        // benefits from more room — the diagrams — is on the right.
        let [left, right] =
            Layout::horizontal([Constraint::Length(46), Constraint::Min(0)]).areas(body);

        // The engine list is sized to the catalogue rather than fixed: adding a
        // preset should lengthen the panel, not silently clip the last row off
        // the bottom of it.
        let engine_rows = self.presets.len() as u16 + 2;
        let [tacho, pedal, manifold, vitals, presets] = Layout::vertical([
            Constraint::Length(7),
            Constraint::Length(5),
            Constraint::Length(6),
            Constraint::Min(6),
            Constraint::Length(engine_rows),
        ])
        .areas(left);

        let [pv, curves, audio] = Layout::vertical([
            Constraint::Percentage(42),
            Constraint::Percentage(30),
            Constraint::Min(9),
        ])
        .areas(right);

        self.draw_header(frame, header);
        self.draw_tacho(frame, tacho);
        self.draw_pedal(frame, pedal);
        self.draw_manifold(frame, manifold);
        self.draw_vitals(frame, vitals);
        self.draw_presets(frame, presets);
        self.draw_pv(frame, pv);
        self.draw_curves(frame, curves);
        self.draw_audio(frame, audio);
        self.draw_footer(frame, footer);
    }

    fn draw_header(&self, frame: &mut Frame, area: Rect) {
        let t = &self.telemetry;

        let mut spans = vec![
            Span::styled(
                format!(" {} ", t.preset_name),
                Style::default().fg(Color::Black).bg(ACCENT).bold(),
            ),
            Span::raw("  "),
            Span::styled(t.preset_spec.clone(), Style::default().fg(LABEL)),
            Span::styled("  ·  ", Style::default().fg(FRAME_COLOR)),
            Span::styled(t.induction, Style::default().fg(LABEL)),
            Span::styled("  ·  ", Style::default().fg(FRAME_COLOR)),
            Span::styled(t.preset_note.clone(), Style::default().fg(INK).italic()),
        ];

        // Only the states that are unusual get a badge. A dashboard where
        // everything is lit all the time is a dashboard nobody reads.
        if self.knock_hold > 0.0 {
            spans.push(badge(" KNOCK ", Color::Black, HOT));
        }
        if t.limiter {
            spans.push(badge(" LIMITER ", Color::Black, WARN));
        }
        if t.manual_cut {
            spans.push(badge(" 2-STEP ", Color::Black, ACCENT));
        }
        if t.muted {
            spans.push(badge(" MUTED ", Color::Black, INK));
        }
        if self.audio_failed {
            spans.push(badge(" NO AUDIO ", Color::White, HOT));
        }
        if !t.ring_primed {
            spans.push(badge(" SETTLING ", Color::Black, COOL));
        }

        let right = format!(
            "{:>5.1} fps   sim {:>5.1} Hz {}   {} sub/frame @ {:.2}°",
            self.fps,
            t.physics_hz,
            if t.realtime { "RT" } else { "  " },
            t.substeps,
            t.dtheta_deg,
        );

        frame.render_widget(
            Paragraph::new(TextLine::from(spans)).block(panel("BENCHTOP ENGINE SIMULATOR")),
            area,
        );
        // Painted over the block's own top border, which is what the title row
        // is for; `Paragraph` alignment cannot put two alignments on one line.
        let inner = Rect {
            x: area.x + 1,
            y: area.y,
            width: area.width.saturating_sub(2),
            height: 1,
        };
        frame.render_widget(
            Paragraph::new(Span::styled(right, Style::default().fg(INK)))
                .alignment(Alignment::Right),
            inner,
        );
    }

    /// Tachometer: a seven-segment readout over a bar that reddens at the top.
    fn draw_tacho(&self, frame: &mut Frame, area: Rect) {
        let t = &self.telemetry;
        let block = panel("TACHOMETER");
        let inner = block.inner(area);
        frame.render_widget(block, area);

        let [digits, gauge, scale] = Layout::vertical([
            Constraint::Length(3),
            Constraint::Length(1),
            Constraint::Length(1),
        ])
        .areas(inner);

        // The needle drives the display, the true speed drives the colour: the
        // limiter is a fact about the engine, not about the needle.
        let shown = self.needle.max(0.0);
        let colour = rpm_colour(t.rpm, t.redline);
        let [readout, units] =
            Layout::horizontal([Constraint::Length(20), Constraint::Min(0)]).areas(digits);

        frame.render_widget(seven_segment(shown, 4).fg(colour), readout);
        frame.render_widget(
            Paragraph::new(vec![
                TextLine::from(Span::styled("rev/min", Style::default().fg(LABEL))),
                TextLine::from(Span::styled(
                    format!("{:>6.1} hp", t.power_hp()),
                    Style::default().fg(LIVE),
                )),
                TextLine::from(Span::styled(
                    format!("{:>6.0} N·m", t.torque),
                    Style::default().fg(LIVE),
                )),
            ]),
            units,
        );

        frame.render_widget(
            Gauge::default()
                .gauge_style(Style::default().fg(colour).bg(Color::Rgb(38, 42, 48)))
                .ratio((shown / GAUGE_MAX_RPM).clamp(0.0, 1.0))
                // `Gauge` inverts the label where the bar covers it, so the
                // colour set here is the one seen over the *empty* part. A dark
                // label would be unreadable there and fine everywhere else,
                // which is the worst way for it to be wrong.
                .label(Span::styled(
                    format!("{:.0} rpm", shown),
                    Style::default().fg(Color::Rgb(232, 236, 242)).bold(),
                )),
            gauge,
        );

        // The scale, with the redline marked where it actually falls.
        frame.render_widget(redline_scale(t.redline, scale.width), scale);
    }

    /// Throttle position and the state of the ignition.
    fn draw_pedal(&self, frame: &mut Frame, area: Rect) {
        let t = &self.telemetry;
        let block = panel("THROTTLE / IGNITION");
        let inner = block.inner(area);
        frame.render_widget(block, area);

        let [bar, state] =
            Layout::vertical([Constraint::Length(1), Constraint::Length(2)]).areas(inner);

        let pedal = t.throttle.clamp(0.0, 1.0);
        frame.render_widget(
            Gauge::default()
                .gauge_style(Style::default().fg(LIVE).bg(Color::Rgb(38, 42, 48)))
                .ratio(pedal)
                .label(Span::styled(
                    format!("{:.0} %", pedal * 100.0),
                    Style::default().fg(Color::Rgb(232, 236, 242)).bold(),
                )),
            bar,
        );

        let (ignition, ignition_colour) = if t.limiter {
            ("CUT — limiter", WARN)
        } else if t.manual_cut {
            ("CUT — 2-step / launch", ACCENT)
        } else {
            ("firing", LIVE)
        };

        frame.render_widget(
            Paragraph::new(vec![
                TextLine::from(vec![
                    Span::styled("pedal target  ", Style::default().fg(LABEL)),
                    Span::styled(
                        format!("{:>3.0} %", t.throttle_target * 100.0),
                        Style::default().fg(INK),
                    ),
                    Span::styled("     turbo  ", Style::default().fg(LABEL)),
                    // A dash, not a zero: an atmospheric engine has no shaft
                    // standing still, it has no shaft.
                    if t.turbo_fitted {
                        Span::styled(
                            format!("{:>6.0} rpm", t.turbo_rpm),
                            Style::default().fg(if t.turbo_surge > 0.35 { WARN } else { INK }),
                        )
                    } else {
                        Span::styled("  none — NA", Style::default().fg(FRAME_COLOR))
                    },
                ]),
                TextLine::from(vec![
                    Span::styled("ignition      ", Style::default().fg(LABEL)),
                    Span::styled(ignition, Style::default().fg(ignition_colour).bold()),
                    Span::styled(
                        if t.turbo_fitted && t.turbo_surge > 0.35 {
                            "   SURGE"
                        } else {
                            ""
                        },
                        Style::default().fg(WARN),
                    ),
                ]),
            ]),
            state,
        );
    }

    /// Manifold pressure, as a gauge that reads vacuum left of zero and boost
    /// right of it — the way a real one is marked.
    fn draw_manifold(&self, frame: &mut Frame, area: Rect) {
        let t = &self.telemetry;
        let block = panel("MANIFOLD PRESSURE");
        let inner = block.inner(area);
        frame.render_widget(block, area);

        let [dial, numbers, note] = Layout::vertical([
            Constraint::Length(1),
            Constraint::Length(1),
            Constraint::Length(2),
        ])
        .areas(inner);

        frame.render_widget(BoostDial::new(t.manifold_gauge_pa()), dial);

        let reading = if t.manifold_gauge_pa() < 0.0 {
            Span::styled(
                format!("{:>5.1} in.Hg vacuum", t.vacuum_inhg()),
                Style::default().fg(COOL).bold(),
            )
        } else {
            Span::styled(
                format!("{:>5.1} psi boost", t.boost_psi()),
                Style::default().fg(WARN).bold(),
            )
        };
        frame.render_widget(
            Paragraph::new(TextLine::from(vec![
                reading,
                Span::styled(
                    format!("    MAP {:>5.1} kPa", t.map_pa / 1e3),
                    Style::default().fg(LABEL),
                ),
            ])),
            numbers,
        );

        // The block is solved atmospherically whatever the engine is, so this
        // gauge lives below zero by design. Saying so on the dashboard is
        // better than leaving a boost needle that never moves and letting it
        // read as a bug — and the reason differs between the two cases, so the
        // line does too.
        frame.render_widget(
            Paragraph::new(vec![
                TextLine::from(Span::styled(
                    format!(
                        "exhaust {:>5.0} K   {:>5.1} kPa   {}",
                        t.exhaust_k,
                        t.exhaust_pa / 1e3,
                        manifold_modes(&t.manifold_modes)
                    ),
                    Style::default().fg(INK),
                )),
                TextLine::from(Span::styled(
                    if t.turbo_fitted {
                        "turbocharged — sound only, no boost"
                    } else {
                        "naturally aspirated — no compressor"
                    },
                    Style::default().fg(FRAME_COLOR).italic(),
                )),
            ]),
            note,
        );
    }

    /// The numbers that describe the cycle itself.
    fn draw_vitals(&self, frame: &mut Frame, area: Rect) {
        let t = &self.telemetry;
        let block = panel("CYCLE");
        let inner = block.inner(area);
        frame.render_widget(block, area);

        let knock_colour = if t.knock_integral > 0.85 {
            HOT
        } else if t.knock_integral > 0.5 {
            WARN
        } else {
            INK
        };

        let rows = vec![
            vital(
                "peak cylinder",
                format!("{:>8.1} bar", t.peak_pressure / 1e5),
                INK,
            ),
            vital("IMEP", format!("{:>8.2} bar", t.imep / 1e5), INK),
            vital("BMEP", format!("{:>8.2} bar", t.bmep / 1e5), INK),
            vital("brake power", format!("{:>8.1} kW", t.power_kw), LIVE),
            vital(
                "knock integral",
                format!("{:>8.3}", t.knock_integral),
                knock_colour,
            ),
            vital(
                "firing rate",
                format!("{:>8.1} Hz", firing_hz(t.rpm, t.cylinders)),
                INK,
            ),
            vital(
                "banks",
                format!(
                    "{:>8}",
                    format!("{} × {}", t.banks, t.cylinders / t.banks.max(1))
                ),
                INK,
            ),
            vital(
                "exhaust gas",
                format!("{:>8.0} K", t.exhaust_k),
                if t.exhaust_k > 1_250.0 { WARN } else { INK },
            ),
            vital(
                "piston speed",
                format!("{:>8.1} m/s", t.mean_piston_speed),
                // Past about 25 m/s a production engine is throwing rods; the
                // colour is the only place that limit appears.
                if t.mean_piston_speed > 25.0 { HOT } else { INK },
            ),
        ];

        frame.render_widget(Paragraph::new(rows), inner);
    }

    /// The catalogue, with the selected engine marked.
    fn draw_presets(&self, frame: &mut Frame, area: Rect) {
        let block = panel("ENGINES  1-9 [ ]");
        let inner = block.inner(area);
        frame.render_widget(block, area);

        // A short terminal gets a window onto the catalogue rather than the
        // top of it: a list that cannot show the engine you are running is
        // worse than one that shows fewer engines.
        let visible = inner.height as usize;
        let current = self.telemetry.preset;
        let first = if self.presets.len() <= visible {
            0
        } else {
            current
                .saturating_sub(visible / 2)
                .min(self.presets.len() - visible)
        };

        let rows: Vec<TextLine> = self
            .presets
            .iter()
            .enumerate()
            .skip(first)
            .take(visible)
            .map(|(i, (name, spec, turbo))| {
                let selected = i == current;
                let marker = if selected { "▶" } else { " " };
                let style = if selected {
                    Style::default().fg(ACCENT).add_modifier(Modifier::BOLD)
                } else {
                    Style::default().fg(INK)
                };
                // Only the first nine have a digit to be reached by; the rest
                // are marked as being on the bracket keys and nothing else.
                let shortcut = char::from_digit(i as u32 + 1, 10).unwrap_or('.');
                TextLine::from(vec![
                    Span::styled(format!("{marker} {shortcut} "), style),
                    Span::styled(format!("{name:<15}"), style),
                    Span::styled(format!("{spec:<21}"), Style::default().fg(FRAME_COLOR)),
                    // Which engines whistle is the one thing about this list a
                    // reader cannot work out from the spec line.
                    Span::styled(
                        if *turbo { "T" } else { " " },
                        Style::default().fg(if *turbo { WARN } else { FRAME_COLOR }),
                    ),
                ])
            })
            .collect();

        frame.render_widget(Paragraph::new(rows), inner);
    }

    /// The pressure-volume loop, straight out of the solver's phase ring.
    fn draw_pv(&self, frame: &mut Frame, area: Rect) {
        let t = &self.telemetry;

        let (v_max, p_max) = t.pv.iter().fold((1.0f64, 1.0f64), |(v, p), &(vol, pres)| {
            (v.max(vol), p.max(pres))
        });
        // Headroom, and a floor so a motored engine does not get a wildly
        // magnified axis showing nothing but atmospheric noise.
        let p_top = (p_max * 1.08).max(5.0);
        let v_top = v_max * 1.04;

        let title = format!(
            "P-V DIAGRAM   {:.0} cc swept   peak {:.1} bar   {}",
            v_max - t.pv.iter().fold(f64::MAX, |m, &(v, _)| m.min(v)),
            p_max,
            if t.ring_primed {
                "one full 720° cycle"
            } else {
                "filling the phase ring..."
            }
        );

        let points = t.pv.clone();
        let canvas = Canvas::default()
            .block(panel(&title))
            // Braille packs 2x4 dots into a cell, which is what makes a curve
            // this fine legible in a terminal at all.
            .marker(Marker::Braille)
            .x_bounds([0.0, v_top])
            .y_bounds([0.0, p_top])
            .paint(move |ctx| {
                let total = points.len();
                for (i, pair) in points.windows(2).enumerate() {
                    let (v1, p1) = pair[0];
                    let (v2, p2) = pair[1];
                    ctx.draw(&CanvasLine {
                        x1: v1,
                        y1: p1,
                        x2: v2,
                        y2: p2,
                        // The loop is published in crank order, so the index
                        // into it is the crank angle and the stroke follows.
                        color: stroke_colour(i, total),
                    });
                }
            });
        frame.render_widget(canvas, area);

        // Axis labels and a legend, painted into the block's border row so they
        // cost no plot area.
        let inner = Rect {
            x: area.x + 1,
            y: area.y + area.height.saturating_sub(1),
            width: area.width.saturating_sub(2),
            height: 1,
        };
        frame.render_widget(
            Paragraph::new(TextLine::from(vec![
                Span::styled(format!(" 0 → {v_top:.0} cm³ "), Style::default().fg(LABEL)),
                Span::styled("intake ", Style::default().fg(INTAKE_COLOR)),
                Span::styled("compression ", Style::default().fg(COMPRESSION_COLOR)),
                Span::styled("power ", Style::default().fg(POWER_COLOR)),
                Span::styled("exhaust ", Style::default().fg(EXHAUST_COLOR)),
                Span::styled(format!(" 0 → {p_top:.0} bar "), Style::default().fg(LABEL)),
            ]))
            .alignment(Alignment::Center),
            inner,
        );
    }

    /// Torque and power, traced as the engine sweeps the range.
    fn draw_curves(&self, frame: &mut Frame, area: Rect) {
        let t = &self.telemetry;

        // Torque in N·m and power in horsepower share an axis. That is not a
        // trick of scaling: for engines of this size the two are numerically
        // similar, which is exactly why dyno sheets have always plotted them
        // together.
        let torque: Vec<(f64, f64)> = t.curve.points().map(|p| (p.rpm, p.torque)).collect();
        let power: Vec<(f64, f64)> = t
            .curve
            .points()
            .map(|p| (p.rpm, p.power * 1.341_022))
            .collect();

        let ceiling = torque
            .iter()
            .chain(power.iter())
            .fold(50.0f64, |m, &(_, y)| m.max(y))
            * 1.1;

        let peak_torque = t.curve.peak_torque();
        let peak_power = t.curve.peak_power();
        let title = match (peak_torque, peak_power) {
            (Some(tq), Some(pw)) => format!(
                "DYNO   peak {:.0} N·m @ {:.0}   {:.0} hp @ {:.0}   {} of {} buckets measured",
                tq.torque,
                tq.rpm,
                pw.power * 1.341_022,
                pw.rpm,
                t.curve.len(),
                crate::ui::telemetry::CURVE_BUCKETS,
            ),
            _ => "DYNO   rev the engine to trace its curve".to_string(),
        };

        // A vertical marker at the current speed, drawn as a two-point dataset
        // so it lives on the same axes as the curves.
        let cursor = vec![(t.rpm, 0.0), (t.rpm, ceiling)];

        let datasets = vec![
            Dataset::default()
                .name("cursor")
                .marker(Marker::Braille)
                .graph_type(GraphType::Line)
                .style(Style::default().fg(FRAME_COLOR))
                .data(&cursor),
            Dataset::default()
                .name("torque N·m")
                .marker(Marker::Braille)
                .graph_type(GraphType::Line)
                .style(Style::default().fg(LIVE))
                .data(&torque),
            Dataset::default()
                .name("power hp")
                .marker(Marker::Braille)
                .graph_type(GraphType::Line)
                .style(Style::default().fg(WARN))
                .data(&power),
        ];

        let chart = Chart::new(datasets)
            .block(panel(&title))
            .legend_position(Some(LegendPosition::TopLeft))
            .x_axis(
                Axis::default()
                    .style(Style::default().fg(FRAME_COLOR))
                    .bounds([0.0, GAUGE_MAX_RPM])
                    .labels(vec![
                        Span::styled("0", Style::default().fg(LABEL)),
                        Span::styled("4500", Style::default().fg(LABEL)),
                        Span::styled("9000 rpm", Style::default().fg(LABEL)),
                    ]),
            )
            .y_axis(
                Axis::default()
                    .style(Style::default().fg(FRAME_COLOR))
                    .bounds([0.0, ceiling])
                    .labels(vec![
                        Span::styled("0", Style::default().fg(LABEL)),
                        Span::styled(format!("{:.0}", ceiling / 2.0), Style::default().fg(LABEL)),
                        Span::styled(format!("{ceiling:.0}"), Style::default().fg(LABEL)),
                    ]),
            );

        frame.render_widget(chart, area);
    }

    /// The spectrum of what the sound card is playing, and a level meter.
    fn draw_audio(&self, frame: &mut Frame, area: Rect) {
        let t = &self.telemetry;
        let block = panel(&format!("AUDIO   {}", self.audio_status));
        let inner = block.inner(area);
        frame.render_widget(block, area);

        let [spectrum, meters] =
            Layout::horizontal([Constraint::Min(30), Constraint::Length(30)]).areas(inner);

        frame.render_widget(
            SpectrumBars {
                bands: self.analyzer.bands(),
                peaks: self.analyzer.peaks(),
                live: self.analyzer.is_live(),
            },
            spectrum,
        );

        let [pulse, hold, stats] = Layout::vertical([
            Constraint::Length(2),
            Constraint::Length(2),
            Constraint::Min(0),
        ])
        .areas(meters);

        let rms = self.analyzer.rms().clamp(0.0, 1.0) as f64;
        let peak = self.analyzer.peak_hold().clamp(0.0, 1.0) as f64;

        frame.render_widget(
            Paragraph::new(vec![
                TextLine::from(Span::styled("pulse  (rms)", Style::default().fg(LABEL))),
                meter_bar(rms, 28, LIVE),
            ]),
            pulse,
        );
        frame.render_widget(
            Paragraph::new(vec![
                TextLine::from(vec![
                    Span::styled("peak hold   ", Style::default().fg(LABEL)),
                    Span::styled(
                        format!("{:>5.1} dBFS", 20.0 * peak.max(1e-6).log10()),
                        Style::default().fg(if peak > 0.97 { HOT } else { INK }),
                    ),
                ]),
                meter_bar(peak, 28, if peak > 0.97 { HOT } else { WARN }),
            ]),
            hold,
        );

        frame.render_widget(
            Paragraph::new(vec![
                TextLine::from(Span::styled(
                    format!(
                        "firing {:>6.1} Hz · {} cyl",
                        firing_hz(t.rpm, t.cylinders),
                        t.cylinders
                    ),
                    Style::default().fg(INK),
                )),
                TextLine::from(Span::styled(
                    format!(
                        "starved {:>4.1} %   dropped {}",
                        t.audio.starvation * 100.0,
                        t.audio.dropped
                    ),
                    Style::default().fg(if t.audio.starvation > 0.5 {
                        WARN
                    } else {
                        FRAME_COLOR
                    }),
                )),
            ])
            .wrap(Wrap { trim: true }),
            stats,
        );
    }

    fn draw_footer(&self, frame: &mut Frame, area: Rect) {
        let key = |k: &str, what: &str| {
            vec![
                Span::styled(
                    format!(" {k} "),
                    Style::default().fg(Color::Black).bg(LABEL),
                ),
                Span::styled(format!(" {what}   "), Style::default().fg(INK)),
            ]
        };
        let mut spans = Vec::new();
        spans.extend(key("↑/W ↓/S", "throttle"));
        spans.extend(key("SPACE", "ignition cut · 2-step"));
        spans.extend(key("1-9 / [ ]", "engine"));
        spans.extend(key(
            "M",
            if self.telemetry.muted {
                "unmute"
            } else {
                "mute"
            },
        ));
        spans.extend(key("Q/ESC", "quit"));

        frame.render_widget(
            Paragraph::new(TextLine::from(spans)).block(panel("CONTROLS")),
            area,
        );
    }
}

// ---------------------------------------------------------------------------
// Small widgets and helpers
// ---------------------------------------------------------------------------

/// The dashboard's standard panel border.
fn panel(title: &str) -> Block<'static> {
    Block::default()
        .borders(Borders::ALL)
        .border_style(Style::default().fg(FRAME_COLOR))
        .title(Span::styled(
            format!(" {title} "),
            Style::default().fg(LABEL).add_modifier(Modifier::BOLD),
        ))
}

/// A coloured status chip for the header.
fn badge(text: &str, fg: Color, bg: Color) -> Span<'static> {
    Span::styled(
        format!("  {text}"),
        Style::default().fg(fg).bg(bg).add_modifier(Modifier::BOLD),
    )
}

/// One `label ......... value` row.
fn vital(label: &str, value: String, colour: Color) -> TextLine<'static> {
    TextLine::from(vec![
        Span::styled(format!("{label:<16}"), Style::default().fg(LABEL)),
        Span::styled(value, Style::default().fg(colour)),
    ])
}

/// The message shown when the terminal is too small to lay the dashboard out.
fn too_small(area: Rect) -> Paragraph<'static> {
    Paragraph::new(vec![
        TextLine::from(Span::styled(
            "terminal too small",
            Style::default().fg(HOT).bold(),
        )),
        TextLine::from(Span::styled(
            format!(
                "need {MIN_WIDTH}x{MIN_HEIGHT}, have {}x{}",
                area.width, area.height
            ),
            Style::default().fg(INK),
        )),
        TextLine::from(Span::styled("Q to quit", Style::default().fg(FRAME_COLOR))),
    ])
    .alignment(Alignment::Center)
}

/// Firing frequency: `f = (rpm / 120) * n` for a four-stroke [Hz].
fn firing_hz(rpm: f64, cylinders: usize) -> f64 {
    rpm / 120.0 * cylinders as f64
}

/// Green below the shift light, amber approaching the redline, red on it.
fn rpm_colour(rpm: f64, redline: f64) -> Color {
    let fraction = rpm / redline.max(1.0);
    if fraction >= 1.0 {
        HOT
    } else if fraction > 0.9 {
        WARN
    } else if fraction > 0.6 {
        LIVE
    } else {
        COOL
    }
}

/// A one-line rpm scale with the redline marked where it falls.
fn redline_scale(redline: f64, width: u16) -> Paragraph<'static> {
    let width = width.max(1) as usize;
    let mark = ((redline / GAUGE_MAX_RPM) * width as f64).round() as usize;
    let mut spans = Vec::new();
    let ticks: String = (0..width)
        .map(|i| {
            // A tick every thousand rpm.
            let rpm = i as f64 / width as f64 * GAUGE_MAX_RPM;
            let next = (i + 1) as f64 / width as f64 * GAUGE_MAX_RPM;
            if (rpm / 1_000.0).floor() != (next / 1_000.0).floor() {
                '┬'
            } else {
                '─'
            }
        })
        .collect();
    let cut = mark.min(ticks.chars().count());
    spans.push(Span::styled(
        ticks.chars().take(cut).collect::<String>(),
        Style::default().fg(FRAME_COLOR),
    ));
    spans.push(Span::styled(
        ticks.chars().skip(cut).collect::<String>(),
        Style::default().fg(HOT),
    ));
    Paragraph::new(TextLine::from(spans))
}

/// A filled bar with a fractional last cell.
fn meter_bar(value: f64, width: usize, colour: Color) -> TextLine<'static> {
    const EIGHTHS: [char; 9] = [' ', '▏', '▎', '▍', '▌', '▋', '▊', '▉', '█'];
    let filled = value.clamp(0.0, 1.0) * width as f64;
    let whole = filled.floor() as usize;
    let remainder = ((filled - whole as f64) * 8.0).round() as usize;

    let mut bar = "█".repeat(whole.min(width));
    if whole < width && remainder > 0 {
        bar.push(EIGHTHS[remainder.min(8)]);
    }
    let padding = width.saturating_sub(bar.chars().count());

    TextLine::from(vec![
        Span::styled(bar, Style::default().fg(colour)),
        Span::styled(
            "·".repeat(padding),
            Style::default().fg(Color::Rgb(45, 50, 58)),
        ),
    ])
}

/// The colour of the stroke that point `index` of `total` falls in.
///
/// The loop is published in crank order over a whole 720 degree cycle, so the
/// index into it *is* the crank angle, and the four strokes are its quarters:
/// intake, compression, power, exhaust, with compression TDC at 360 degrees.
fn stroke_colour(index: usize, total: usize) -> Color {
    let quarter = if total == 0 { 0 } else { index * 4 / total };
    match quarter {
        0 => INTAKE_COLOR,
        1 => COMPRESSION_COLOR,
        2 => POWER_COLOR,
        _ => EXHAUST_COLOR,
    }
}

/// A short word for each bank's acoustic state.
fn manifold_modes(modes: &[ManifoldMode]) -> String {
    if modes.is_empty() {
        return String::new();
    }
    let names: Vec<&str> = modes
        .iter()
        .map(|mode| match mode {
            ManifoldMode::Plenum => "plenum",
            ManifoldMode::Acoustic => "acoustic",
        })
        .collect();
    names.join("/")
}

// ---------------------------------------------------------------------------
// The boost dial
// ---------------------------------------------------------------------------

/// A centre-zero manifold gauge: vacuum to the left, boost to the right.
///
/// Written as a widget rather than assembled from two `Gauge`s because the
/// interesting thing about the reading is where it sits relative to atmospheric,
/// and that needs one continuous scale with zero somewhere inside it.
struct BoostDial {
    /// Gauge pressure [Pa]: negative is vacuum.
    gauge_pa: f64,
}

impl BoostDial {
    /// Full-scale vacuum, one atmosphere down [Pa].
    const VACUUM_SPAN: f64 = 101_325.0;
    /// Full-scale boost, 20 psi up [Pa].
    const BOOST_SPAN: f64 = 137_895.0;

    fn new(gauge_pa: f64) -> Self {
        Self { gauge_pa }
    }
}

impl Widget for BoostDial {
    fn render(self, area: Rect, buf: &mut Buffer) {
        if area.width < 8 || area.height == 0 {
            return;
        }
        let width = area.width as usize;
        // Zero sits where the two spans meet, so the scale is continuous but the
        // two halves have different resolutions — which is how a real gauge is
        // marked, since 20 psi of boost and 30 in.Hg of vacuum are not the same
        // amount of pressure.
        let zero = ((Self::VACUUM_SPAN / (Self::VACUUM_SPAN + Self::BOOST_SPAN)) * width as f64)
            .round() as usize;
        let zero = zero.clamp(1, width - 1);

        let position = if self.gauge_pa < 0.0 {
            let fraction = (-self.gauge_pa / Self::VACUUM_SPAN).clamp(0.0, 1.0);
            (zero as f64 * (1.0 - fraction)).round() as usize
        } else {
            let fraction = (self.gauge_pa / Self::BOOST_SPAN).clamp(0.0, 1.0);
            zero + ((width - zero) as f64 * fraction).round() as usize
        };
        let position = position.min(width - 1);

        let y = area.y;
        for x in 0..width {
            let (symbol, colour) = if x == position {
                ("◆", if self.gauge_pa < 0.0 { COOL } else { WARN })
            } else if x == zero {
                ("│", LABEL)
            } else if (x > position && x < zero) || (x < position && x > zero) {
                // The span between the needle and atmospheric, filled in.
                ("─", if self.gauge_pa < 0.0 { COOL } else { WARN })
            } else {
                ("·", Color::Rgb(45, 50, 58))
            };
            buf[(area.x + x as u16, y)]
                .set_symbol(symbol)
                .set_style(Style::default().fg(colour));
        }
    }
}

// ---------------------------------------------------------------------------
// The spectrum
// ---------------------------------------------------------------------------

/// Vertical bars with a peak-hold cap, one per analyser band.
///
/// Eighth-block glyphs give eight sub-rows of vertical resolution per terminal
/// row, so a panel eight rows deep resolves sixty-four levels — enough that the
/// display moves smoothly rather than stepping.
struct SpectrumBars<'a> {
    bands: &'a [f32],
    peaks: &'a [f32],
    live: bool,
}

impl Widget for SpectrumBars<'_> {
    fn render(self, area: Rect, buf: &mut Buffer) {
        const LEVELS: [char; 9] = [' ', '▁', '▂', '▃', '▄', '▅', '▆', '▇', '█'];
        if area.width == 0 || area.height == 0 {
            return;
        }

        if !self.live {
            let text = "no audio tap";
            let x = area.x + area.width.saturating_sub(text.len() as u16) / 2;
            let y = area.y + area.height / 2;
            for (i, ch) in text.chars().enumerate() {
                let x = x + i as u16;
                if x < area.x + area.width {
                    buf[(x, y)]
                        .set_symbol(&ch.to_string())
                        .set_style(Style::default().fg(FRAME_COLOR));
                }
            }
            return;
        }

        let columns = area.width as usize;
        let rows = area.height as usize;

        for column in 0..columns {
            // The bands are stretched across whatever width there is rather
            // than drawn one per column. In a narrow terminal that folds the
            // top of the spectrum in instead of losing it off the edge; in a
            // wide one it widens the bars rather than leaving a gap.
            let band = column * self.bands.len() / columns.max(1);
            let level = self.bands.get(band).copied().unwrap_or(0.0).clamp(0.0, 1.0);
            let peak = self.peaks.get(band).copied().unwrap_or(0.0).clamp(0.0, 1.0);

            let eighths = (level * (rows * 8) as f32).round() as usize;
            let peak_row = ((peak * rows as f32).ceil() as usize).clamp(1, rows);
            let colour = band_colour(band, self.bands.len());
            let x = area.x + column as u16;

            for row in 0..rows {
                // Row 0 is the top of the panel, so the bar is drawn from the
                // bottom up.
                let from_bottom = rows - 1 - row;
                let y = area.y + row as u16;

                let filled = eighths.saturating_sub(from_bottom * 8).min(8);
                if filled > 0 {
                    buf[(x, y)]
                        .set_symbol(&LEVELS[filled].to_string())
                        .set_style(Style::default().fg(colour));
                } else if from_bottom + 1 == peak_row {
                    buf[(x, y)]
                        .set_symbol("▁")
                        .set_style(Style::default().fg(Color::Rgb(90, 96, 108)));
                }
            }
        }
    }
}

/// Bands run cool at the bottom of the spectrum to hot at the top, so the
/// harmonic structure of the note reads as a shape rather than a wall.
fn band_colour(band: usize, total: usize) -> Color {
    let t = band as f32 / total.max(1) as f32;
    let r = (80.0 + 170.0 * t) as u8;
    let g = (200.0 - 60.0 * t) as u8;
    let b = (230.0 - 150.0 * t) as u8;
    Color::Rgb(r, g, b)
}

// ---------------------------------------------------------------------------
// The seven-segment readout
// ---------------------------------------------------------------------------

/// Segment patterns, three rows per digit, in the shape a calculator draws.
const SEGMENTS: [[&str; 3]; 10] = [
    [" _ ", "| |", "|_|"], // 0
    ["   ", "  |", "  |"], // 1
    [" _ ", " _|", "|_ "], // 2
    [" _ ", " _|", " _|"], // 3
    ["   ", "|_|", "  |"], // 4
    [" _ ", "|_ ", " _|"], // 5
    [" _ ", "|_ ", "|_|"], // 6
    [" _ ", "  |", "  |"], // 7
    [" _ ", "|_|", "|_|"], // 8
    [" _ ", "|_|", " _|"], // 9
];

/// Renders a number as a right-aligned seven-segment readout `digits` wide.
///
/// Leading zeros are blanked rather than drawn, the way an instrument cluster
/// does it: a tachometer reading `0850` looks like a fault, ` 850` looks like an
/// idle.
pub fn seven_segment(value: f64, digits: usize) -> Paragraph<'static> {
    let clamped = value.max(0.0).min(10f64.powi(digits as i32) - 1.0) as u64;
    let text = format!("{clamped:>width$}", width = digits);

    let rows: Vec<TextLine> = (0..3)
        .map(|row| {
            let mut line = String::with_capacity(digits * 4);
            for ch in text.chars() {
                match ch.to_digit(10) {
                    Some(d) => line.push_str(SEGMENTS[d as usize][row]),
                    None => line.push_str("   "),
                }
                line.push(' ');
            }
            TextLine::from(line)
        })
        .collect();

    Paragraph::new(rows)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crossterm::event::KeyEventState;
    use ratatui::backend::TestBackend;

    fn press(code: KeyCode) -> KeyEvent {
        KeyEvent {
            code,
            modifiers: KeyModifiers::NONE,
            kind: KeyEventKind::Press,
            state: KeyEventState::NONE,
        }
    }

    #[test]
    fn the_documented_bindings_all_do_what_the_footer_says() {
        let mut dashboard = Dashboard::new();

        assert_eq!(
            dashboard.on_key(press(KeyCode::Up)),
            Some(Command::Throttle(THROTTLE_STEP))
        );
        assert_eq!(
            dashboard.on_key(press(KeyCode::Char('w'))),
            Some(Command::Throttle(THROTTLE_STEP))
        );
        assert_eq!(
            dashboard.on_key(press(KeyCode::Down)),
            Some(Command::Throttle(-THROTTLE_STEP))
        );
        assert_eq!(
            dashboard.on_key(press(KeyCode::Char('S'))),
            Some(Command::Throttle(-THROTTLE_STEP))
        );
        assert_eq!(
            dashboard.on_key(press(KeyCode::Char(' '))),
            Some(Command::ToggleCut)
        );
        assert_eq!(
            dashboard.on_key(press(KeyCode::Char('M'))),
            Some(Command::ToggleMute)
        );
        for (key, index) in [('1', 0), ('3', 2), ('7', 6), ('9', 8)] {
            assert_eq!(
                dashboard.on_key(press(KeyCode::Char(key))),
                Some(Command::SelectPreset(index))
            );
        }
        // Zero is not an engine, and nor is any digit past the end of the
        // catalogue: both must select nothing rather than clamp onto the
        // nearest one.
        let unbound = std::iter::once('0').chain(
            (dashboard.presets.len() + 1..=9)
                .map(|n| char::from_digit(n as u32, 10).expect("a digit")),
        );
        for key in unbound {
            assert_eq!(
                dashboard.on_key(press(KeyCode::Char(key))),
                None,
                "'{key}' selected an engine that is not in the catalogue"
            );
        }
        // The catalogue is longer than the digit row, so every engine past the
        // ninth is reachable only by stepping — and stepping has to reach all
        // of them, in order, and wrap.
        let count = dashboard.presets.len();
        assert!(count > 0, "an empty catalogue");
        for index in 0..count {
            dashboard.telemetry.preset = index;
            assert_eq!(
                dashboard.on_key(press(KeyCode::Char(']'))),
                Some(Command::SelectPreset((index + 1) % count)),
                "']' did not step forward from engine {index}"
            );
            assert_eq!(
                dashboard.on_key(press(KeyCode::Char('['))),
                Some(Command::SelectPreset((index + count - 1) % count)),
                "'[' did not step back from engine {index}"
            );
        }
        assert!(!dashboard.quitting());
    }

    #[test]
    fn quitting_is_reachable_three_ways() {
        for code in [KeyCode::Char('q'), KeyCode::Char('Q'), KeyCode::Esc] {
            let mut dashboard = Dashboard::new();
            assert_eq!(dashboard.on_key(press(code)), Some(Command::Quit));
            assert!(dashboard.quitting(), "{code:?} did not set the quit flag");
        }

        // Raw mode swallows the interrupt, so Ctrl-C has to be handled here or
        // the only way out would be to kill the process from another terminal.
        let mut dashboard = Dashboard::new();
        let ctrl_c = KeyEvent {
            code: KeyCode::Char('c'),
            modifiers: KeyModifiers::CONTROL,
            kind: KeyEventKind::Press,
            state: KeyEventState::NONE,
        };
        assert_eq!(dashboard.on_key(ctrl_c), Some(Command::Quit));
        assert!(dashboard.quitting());
    }

    #[test]
    fn key_releases_are_ignored() {
        let mut dashboard = Dashboard::new();
        let release = KeyEvent {
            kind: KeyEventKind::Release,
            ..press(KeyCode::Char(' '))
        };
        assert_eq!(dashboard.on_key(release), None);

        // Auto-repeat is not: holding the throttle has to keep moving the pedal.
        let repeat = KeyEvent {
            kind: KeyEventKind::Repeat,
            ..press(KeyCode::Up)
        };
        assert_eq!(
            dashboard.on_key(repeat),
            Some(Command::Throttle(THROTTLE_STEP))
        );
    }

    #[test]
    fn the_readout_blanks_leading_zeros_and_clamps() {
        let render = |value: f64| {
            let mut buffer = Buffer::empty(Rect::new(0, 0, 20, 3));
            seven_segment(value, 4).render(Rect::new(0, 0, 20, 3), &mut buffer);
            (0..3)
                .map(|row| {
                    (0..20)
                        .map(|x| buffer[(x, row)].symbol().to_string())
                        .collect::<String>()
                })
                .collect::<Vec<_>>()
        };

        // An exact check on one number, so a mangled segment table cannot pass.
        assert_eq!(
            render(1_205.0),
            vec![
                "     _   _   _      ".to_string(),
                "  |  _| | | |_      ".to_string(),
                "  | |_  |_|  _|     ".to_string(),
            ],
            "1205 did not come out as 1-2-0-5"
        );

        let idle = render(850.0);
        // The leading digit is blank, not a zero: no vertical bars in column 0-3.
        assert!(idle.iter().all(|row| row[..4].trim().is_empty()));
        assert!(
            idle[2].contains("|_|"),
            "850 should draw an 8: {:?}",
            idle[2]
        );

        // Four digits cannot show five, and must not panic trying.
        let over = render(99_999.0);
        assert!(over[0].starts_with(" _ "), "clamped to 9999: {:?}", over[0]);
        // A negative speed clamps to zero, and zero is a digit like any other:
        // a tachometer at rest reads 0, it does not go blank.
        assert_eq!(render(-500.0), render(0.0));
        assert!(render(0.0)[1].contains("| |"), "0 rpm should draw a zero");
    }

    #[test]
    fn the_knock_warning_latches_and_then_clears() {
        let mut dashboard = Dashboard::new();
        let frame = Duration::from_secs_f64(1.0 / 60.0);

        dashboard.telemetry.knocking = true;
        dashboard.tick(frame);
        assert!(dashboard.knock_hold > 0.0);

        // One knocking frame has to stay visible for most of a second, or an
        // event that lasts a single cycle is never seen.
        dashboard.telemetry.knocking = false;
        for _ in 0..30 {
            dashboard.tick(frame);
        }
        assert!(
            dashboard.knock_hold > 0.0,
            "the warning cleared within half a second"
        );

        for _ in 0..90 {
            dashboard.tick(frame);
        }
        assert_eq!(dashboard.knock_hold, 0.0, "the warning never cleared");
    }

    #[test]
    fn the_strokes_are_the_quarters_of_the_cycle() {
        // 720 samples over one cycle: intake, compression, power, exhaust.
        assert_eq!(stroke_colour(0, 720), INTAKE_COLOR);
        assert_eq!(stroke_colour(179, 720), INTAKE_COLOR);
        assert_eq!(stroke_colour(180, 720), COMPRESSION_COLOR);
        assert_eq!(stroke_colour(360, 720), POWER_COLOR);
        assert_eq!(stroke_colour(540, 720), EXHAUST_COLOR);
        assert_eq!(stroke_colour(719, 720), EXHAUST_COLOR);
        // An empty loop must not divide by zero.
        assert_eq!(stroke_colour(0, 0), INTAKE_COLOR);
    }

    #[test]
    fn the_boost_dial_puts_the_needle_where_the_pressure_is() {
        let needle_at = |gauge_pa: f64| {
            let area = Rect::new(0, 0, 40, 1);
            let mut buffer = Buffer::empty(area);
            BoostDial::new(gauge_pa).render(area, &mut buffer);
            (0..40).find(|&x| buffer[(x, 0)].symbol() == "◆")
        };
        // The atmospheric mark, read off a dial whose needle is somewhere else.
        let zero = {
            let area = Rect::new(0, 0, 40, 1);
            let mut buffer = Buffer::empty(area);
            BoostDial::new(-50_000.0).render(area, &mut buffer);
            (0..40)
                .find(|&x| buffer[(x, 0)].symbol() == "│")
                .expect("the atmospheric mark")
        };

        // At exactly atmospheric the needle sits on the mark and must win it:
        // a dial with no needle at all reads as broken.
        let area = Rect::new(0, 0, 40, 1);
        let mut buffer = Buffer::empty(area);
        BoostDial::new(0.0).render(area, &mut buffer);
        assert_eq!(buffer[(zero, 0)].symbol(), "◆");

        let vacuum = needle_at(-50_000.0).expect("a needle under vacuum");
        let boost = needle_at(50_000.0).expect("a needle under boost");
        assert!(vacuum < zero, "vacuum must read left of atmospheric");
        assert!(boost > zero, "boost must read right of atmospheric");

        // Off-scale readings clamp to the ends rather than drawing outside.
        assert!(needle_at(-1e9).unwrap() < zero);
        assert!(needle_at(1e9).unwrap() >= 39 - 1);
    }

    #[test]
    fn firing_rate_follows_the_four_stroke_speed_law() {
        // A V8 at 3000 rpm fires 200 times a second.
        assert!((firing_hz(3_000.0, 8) - 200.0).abs() < 1e-9);
        // A four at the same speed fires half as often.
        assert!((firing_hz(3_000.0, 4) - 100.0).abs() < 1e-9);
        assert_eq!(firing_hz(0.0, 8), 0.0);
    }

    #[test]
    fn a_full_dashboard_draws_without_panicking_at_any_size() {
        let mut dashboard = Dashboard::new();
        dashboard.telemetry = Telemetry {
            preset_name: "Cross-plane V8".into(),
            preset_spec: "5.0 L  8 cyl  11.0:1".into(),
            preset_note: "burble".into(),
            cylinders: 8,
            banks: 2,
            redline: 7_000.0,
            rpm: 6_500.0,
            throttle: 1.0,
            map_pa: 90_000.0,
            peak_pressure: 6.2e6,
            knocking: true,
            limiter: true,
            pv: (0..720)
                .map(|i| {
                    let t = i as f64 / 720.0 * std::f64::consts::TAU;
                    (300.0 + 250.0 * t.cos(), 20.0 + 18.0 * t.sin())
                })
                .collect(),
            ..Telemetry::default()
        };
        dashboard.telemetry.curve.record(3_000.0, 400.0);
        dashboard.telemetry.curve.record(6_000.0, 380.0);

        // Comfortable, exactly minimum, and far too small.
        for (width, height) in [(200, 60), (120, 40), (MIN_WIDTH, MIN_HEIGHT), (40, 10)] {
            let backend = TestBackend::new(width, height);
            let mut terminal = Terminal::new(backend).unwrap();
            terminal
                .draw(|frame| dashboard.draw(frame))
                .expect("a draw at every size");
        }
    }

    /// The whole rendered screen as text, for asserting on what is visible.
    fn screen(dashboard: &Dashboard, width: u16, height: u16) -> String {
        let mut terminal = Terminal::new(TestBackend::new(width, height)).unwrap();
        terminal.draw(|frame| dashboard.draw(frame)).unwrap();
        let buffer = terminal.backend().buffer();
        (0..buffer.area.height)
            .map(|y| {
                (0..buffer.area.width)
                    .map(|x| buffer[(x, y)].symbol())
                    .collect::<String>()
            })
            .collect::<Vec<_>>()
            .join("\n")
    }

    #[test]
    fn the_engine_list_always_shows_the_engine_that_is_running() {
        // The catalogue outgrew the panel once already. Clipping it is silent
        // and leaves the selected engine off the bottom of a list whose whole
        // job is to say which engine is selected.
        let mut dashboard = Dashboard::new();
        let last = dashboard.presets.len() - 1;
        let (name, ..) = dashboard.presets[last];
        dashboard.telemetry.preset = last;

        // Past the ninth the row carries a dot rather than a digit, because
        // there is no digit left to reach it by.
        let shortcut = char::from_digit(last as u32 + 1, 10).unwrap_or('.');
        for (width, height) in [(200, 60), (MIN_WIDTH, MIN_HEIGHT)] {
            let screen = screen(&dashboard, width, height);
            assert!(
                screen.contains(&format!("▶ {shortcut} {name}")),
                "{name} is not on the list at {width}x{height}:\n{screen}"
            );
        }
    }

    #[test]
    fn the_turbo_readout_says_so_when_there_is_no_turbo() {
        // Zero rpm on a gauge reads as a stopped turbo, not as an engine that
        // never had one — which is the whole distinction this dashboard exists
        // to show.
        let mut dashboard = Dashboard::new();
        let atmospheric = screen(&dashboard, 140, 45);
        assert!(atmospheric.contains("none — NA"), "{atmospheric}");
        assert!(atmospheric.contains("naturally aspirated — no compressor"));

        dashboard.telemetry.turbo_fitted = true;
        dashboard.telemetry.turbo_rpm = 137_496.0;
        let turbocharged = screen(&dashboard, 140, 45);
        assert!(turbocharged.contains("137496 rpm"), "{turbocharged}");
        assert!(turbocharged.contains("turbocharged — sound only, no boost"));
    }

    #[test]
    fn an_empty_dashboard_draws_before_any_telemetry_arrives() {
        // The first frame is rendered before the simulation thread has published
        // anything, so every default has to survive the layout.
        let dashboard = Dashboard::new();
        let mut terminal = Terminal::new(TestBackend::new(140, 45)).unwrap();
        terminal.draw(|frame| dashboard.draw(frame)).unwrap();
    }
}
