use super::*;
use crate::physics::CylinderHealth;
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
    assert_eq!(
        dashboard.on_key(press(KeyCode::Char('c'))),
        Some(Command::CycleLimiterCut)
    );
    assert_eq!(
        dashboard.on_key(press(KeyCode::Char('C'))),
        Some(Command::CycleLimiterCut)
    );
    assert_eq!(
        dashboard.on_key(press(KeyCode::Char('l'))),
        Some(Command::CycleLimiterMode)
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
fn the_cycle_panel_swaps_knock_for_ignition_delay_on_a_diesel() {
    // A knock integral is a petrol engine's margin against a failure. On a
    // compression engine the same integral reaching one is the engine
    // working, so the panel has to report the other half of it — how long
    // the charge took — or it is showing a number that cannot vary.
    let mut dashboard = Dashboard::new();
    let petrol = screen(&dashboard, 140, 45);
    assert!(petrol.contains("knock integral"), "{petrol}");
    assert!(!petrol.contains("ignition delay"));

    dashboard.telemetry.ignition_delay = Some(0.850e-3);
    let diesel = screen(&dashboard, 140, 45);
    assert!(diesel.contains("ignition delay"), "{diesel}");
    assert!(diesel.contains("0.85 ms"), "{diesel}");
    assert!(!diesel.contains("knock integral"));
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

#[test]
fn layout_mode_pane_counts_and_splits() {
    let area = Rect::new(0, 0, 100, 60);
    for mode in [
        LayoutMode::Single,
        LayoutMode::SplitVertical,
        LayoutMode::SplitHorizontal,
        LayoutMode::TripleWide,
        LayoutMode::QuadGrid,
    ] {
        let rects = mode.split(area);
        assert_eq!(rects.len(), mode.pane_count());
        for r in rects {
            assert!(r.width > 0 && r.height > 0);
        }
    }
}

#[test]
fn cycling_layout_modes_and_panes() {
    let mut d = Dashboard::new();
    assert_eq!(d.layout_mode, LayoutMode::Single);
    assert_eq!(d.active_pane_idx, 0);

    d.cycle_layout_mode();
    assert_eq!(d.layout_mode, LayoutMode::SplitVertical);
    d.cycle_active_pane();
    assert_eq!(d.active_pane_idx, 1);
    d.cycle_active_pane();
    assert_eq!(d.active_pane_idx, 0);

    d.cycle_layout_mode();
    assert_eq!(d.layout_mode, LayoutMode::SplitHorizontal);
    d.cycle_layout_mode();
    assert_eq!(d.layout_mode, LayoutMode::TripleWide);
    d.cycle_layout_mode();
    assert_eq!(d.layout_mode, LayoutMode::QuadGrid);
    d.cycle_layout_mode();
    assert_eq!(d.layout_mode, LayoutMode::Single);
}

#[test]
fn switching_views_and_rendering_all_layouts() {
    let mut d = Dashboard::new();
    d.telemetry = Telemetry {
        spark_trim: 3.0,
        afr_trim: -0.4,
        lpp_deg_atdc: 14.5,
        oil_pressure_bar: 4.2,
        dyno_absorber_torque: 350.0,
        cylinder_health: vec![
            CylinderHealth {
                spark_ok: true,
                fuel_ok: true,
            },
            CylinderHealth {
                spark_ok: false,
                fuel_ok: true,
            },
        ],
        last_pull: Some(crate::bench::DynoRun {
            points: Vec::new(),
            peak_torque: Some(crate::bench::DynoPoint {
                rpm: 4500.0,
                torque: 420.0,
                power_kw: 198.0,
            }),
            peak_power: Some(crate::bench::DynoPoint {
                rpm: 6200.0,
                torque: 360.0,
                power_kw: 234.0,
            }),
            sae_correction: 1.025,
        }),
        ..Telemetry::default()
    };

    // Test each view in Single mode
    for view in [
        ViewPane::DynoCell,
        ViewPane::CombustionLab,
        ViewPane::EcuTuning,
        ViewPane::ThermalFluids,
        ViewPane::NvhOrders,
    ] {
        d.set_active_pane_view(view);
        let s = screen(&d, 160, 50);
        assert!(!s.is_empty());
    }

    // Test ECU tuning screen contains calibration indicators and cut type display
    d.set_active_pane_view(ViewPane::EcuTuning);
    d.telemetry.limiter_cut = crate::physics::LimiterCut::Spark;
    let ecu_screen = screen(&d, 160, 50);
    assert!(ecu_screen.contains("IGNITION CALIBRATION"));
    assert!(ecu_screen.contains("FUELLING CALIBRATION"));
    assert!(ecu_screen.contains("REV LIMITER"));
    assert!(ecu_screen.contains("CYLINDER HEALTH MATRIX"));
    assert!(ecu_screen.contains("SPARK CUT (Backfire)"));

    d.telemetry.limiter_cut = crate::physics::LimiterCut::Fuel;
    let ecu_fuel = screen(&d, 160, 50);
    assert!(ecu_fuel.contains("FUEL CUT (Clean)"));

    d.telemetry.limiter_cut = crate::physics::LimiterCut::None;
    let ecu_none = screen(&d, 160, 50);
    assert!(ecu_none.contains("NONE (No Cut)"));

    // Test Combustion lab contains LPP and IMEP
    d.set_active_pane_view(ViewPane::CombustionLab);
    let comb_screen = screen(&d, 160, 50);
    assert!(comb_screen.contains("INDICATOR DIAGNOSTICS"));
    assert!(comb_screen.contains("Peak Loc (LPP)"));
    assert!(comb_screen.contains("MBT Optimal"));

    // Test Dyno Cell with completed pull shows ghost overlay info
    d.set_active_pane_view(ViewPane::DynoCell);
    let dyno_screen = screen(&d, 160, 50);
    assert!(dyno_screen.contains("DYNO PULL: 420 N·m · 314 hp (SAE CF: 1.025)"));

    // Test QuadGrid layout renders without issue
    d.layout_mode = LayoutMode::QuadGrid;
    let quad_screen = screen(&d, 180, 60);
    assert!(!quad_screen.is_empty());
}
