use super::*;

#[test]
fn export_and_validate_all_catalogue_toml_files() {
    let presets = [
        ("engines/inline_4.toml", EnginePreset::inline_four()),
        (
            "engines/cross_plane_v8.toml",
            EnginePreset::cross_plane_v8(),
        ),
        (
            "engines/big_cam_chopping_v8.toml",
            EnginePreset::big_cam_chopping_v8(),
        ),
        ("engines/flat_plane_v8.toml", EnginePreset::flat_plane_v8()),
        ("engines/v10.toml", EnginePreset::v10()),
        ("engines/v12.toml", EnginePreset::v12()),
        (
            "engines/2_rotor_wankel.toml",
            EnginePreset::two_rotor_wankel(),
        ),
        (
            "engines/turbo_inline_4.toml",
            EnginePreset::turbo_inline_four(),
        ),
        ("engines/twin_turbo_v8.toml", EnginePreset::twin_turbo_v8()),
        (
            "engines/turbo_inline_6.toml",
            EnginePreset::turbo_inline_six(),
        ),
        (
            "engines/turbodiesel_i4.toml",
            EnginePreset::turbo_diesel_four(),
        ),
        ("engines/big_single.toml", EnginePreset::big_single()),
        ("engines/gt3_cup_992.toml", EnginePreset::gt3_cup_992()),
        ("engines/gt3_cup_997.toml", EnginePreset::gt3_cup_997()),
        ("engines/amg_gt3.toml", EnginePreset::amg_gt3()),
        (
            "engines/ferrari_458_gt3.toml",
            EnginePreset::ferrari_458_gt3(),
        ),
        ("engines/r8_lms_gt3.toml", EnginePreset::r8_lms_gt3()),
    ];

    for (path, preset) in &presets {
        preset
            .to_file(path)
            .expect("failed to write preset to file");
        let loaded = EnginePreset::from_file(path).expect("failed to read preset from file");
        assert_eq!(loaded.name, preset.name);
        assert_eq!(loaded.firing.len(), preset.firing.len());
        assert_eq!(loaded.redline, preset.redline);
    }
}

#[test]
fn engine_config_roundtrip_all_catalogue_presets() {
    for preset in EnginePreset::catalogue() {
        let toml_str = preset.to_toml().expect("failed to serialize preset");
        let restored = EnginePreset::from_toml(&toml_str).expect("failed to deserialize preset");

        assert_eq!(restored.name, preset.name);
        assert_eq!(restored.firing.len(), preset.firing.len());
        assert_eq!(restored.redline, preset.redline);
        assert!((restored.float_rpm - preset.float_rpm).abs() < 1e-6);
        assert_eq!(restored.limiter_mode, preset.limiter_mode);
        assert_eq!(restored.limiter_cut, preset.limiter_cut);
        assert!(
            (restored.model.geometry.displacement() - preset.model.geometry.displacement()).abs()
                < 1e-9
        );
    }
}

#[test]
fn engine_config_float_rpm_override_roundtrips() {
    let mut preset = EnginePreset::inline_four();
    preset.float_rpm = 8_200.0;
    let toml_str = preset.to_toml().expect("failed to serialize preset");
    assert!(toml_str.contains("float_rpm = 8200.0"));
    let restored = EnginePreset::from_toml(&toml_str).expect("failed to deserialize preset");
    assert_eq!(restored.float_rpm, 8_200.0);
}

#[test]
fn engine_config_open_headers_mode_override() {
    let preset = EnginePreset::inline_four();
    let mut config = EngineConfig::from_preset(&preset);
    config.exhaust.mode = Some("open_headers".to_string());

    let built = config.to_preset();
    assert!(built.exhaust.is_open_headers());
    assert!(built.exhaust.silencers.is_empty());
    // The inline-four has no cutout fitted, and the open-headers
    // conversion leaves fitment untouched rather than forcing it on.
    assert!(!built.exhaust.cutout_fitted);
}
