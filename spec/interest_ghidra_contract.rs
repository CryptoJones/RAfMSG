//! Client spatial culling / interest contract from the Ghidra 12.1.4 decompile
//! on pluto (`/mnt/hdd1/repos/OpenSourceSpaceGame/StarCitizen/functions/`).
//!
//! Sources:
//! - `FUN_142cc87e0` lines 615-685: registers paired enable/cull distance
//!   CVars for spatial relevance:
//!   - `g_foip_voiceEnableDistance` = `0x42480000` (50.0 m)
//!   - `g_foip_voiceCullDistance`   = `0x42700000` (60.0 m)
//!   - `g_foip_facialCullDistance`  = `0x42200000` (40.0 m)
//!
//!   The enable distance is strictly less than the cull distance, creating a
//!   hysteresis band `[enable, cull]` that prevents boundary flicker.
//!
//! Comparison with `ragfmsg::interest::InterestManager`:
//! - `InterestConfig` models the same dual-range hysteresis
//!   (`sensor_range < exit_range`, default `10_000.0 < 12_000.0`).

use ragfmsg::interest::{InterestConfig, InterestManager};
use ragfmsg::types::Vec3;

const VOICE_ENABLE_DIST: f32 = f32::from_bits(0x4248_0000); // 50.0
const VOICE_CULL_DIST: f32 = f32::from_bits(0x4270_0000); // 60.0
const FACIAL_CULL_DIST: f32 = f32::from_bits(0x4220_0000); // 40.0

#[test]
fn recovered_cull_cvars_form_hysteresis_band() {
    assert_eq!(VOICE_ENABLE_DIST, 50.0);
    assert_eq!(VOICE_CULL_DIST, 60.0);
    assert_eq!(FACIAL_CULL_DIST, 40.0);
    const { assert!(VOICE_ENABLE_DIST < VOICE_CULL_DIST) };
}

#[test]
fn interest_manager_honors_decompiled_hysteresis_band() {
    let cfg = InterestConfig {
        sensor_range: VOICE_ENABLE_DIST as f64,
        exit_range: VOICE_CULL_DIST as f64,
        update_interval_ticks: 1,
        hysteresis_updates: 1,
    };
    let mut mgr = InterestManager::new(cfg);
    mgr.register_client(1, Vec3::ZERO);
    // Inside enable distance -> enters view.
    mgr.set_position(10, Vec3::new(45.0, 0.0, 0.0));
    mgr.update();
    assert!(mgr.can_target(1, 10));

    // Inside hysteresis band (55.0 between 50.0 and 60.0) -> remains visible.
    mgr.set_position(10, Vec3::new(55.0, 0.0, 0.0));
    mgr.update();
    assert!(mgr.can_target(1, 10));

    // Beyond cull distance (65.0 > 60.0) -> evicted.
    mgr.set_position(10, Vec3::new(65.0, 0.0, 0.0));
    mgr.update();
    assert!(!mgr.can_target(1, 10));
}
