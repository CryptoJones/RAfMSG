use ragfmsg::floating_origin::*;
use ragfmsg::ifcs::IfcsConfig;
use ragfmsg::types::{ShipBody, ShipInput, Vec3};

fn large_world() -> WorldPoint {
    WorldPoint::new(1_000_000.0, 250_000.0, -1_000_000.0)
}

#[test]
fn origin_local_world_roundtrip_at_one_million_meters() {
    let origin = FloatingOrigin::new(large_world(), DEFAULT_REBASE_THRESHOLD);
    let world = WorldPoint::new(1_000_123.5, 250_042.25, -999_900.125);
    let local = origin.to_local(world);
    assert!(local.0.x.abs() < 1000.0);
    let restored = origin.to_world(local);
    assert_eq!(restored.0, world.0);
}

#[test]
fn origin_rebase_triggers_at_threshold() {
    let mut origin = FloatingOrigin::at_origin();
    assert!(!origin.maybe_rebase(WorldPoint::new(500.0, 0.0, 0.0)));
    assert_eq!(origin.rebase_count, 0);
    assert!(origin.maybe_rebase(WorldPoint::new(1500.0, 0.0, 0.0)));
    assert_eq!(origin.rebase_count, 1);
    assert_eq!(origin.anchor.0.x, 1500.0);
}

#[test]
fn origin_rebase_causes_no_visible_pop() {
    let mut world = FloatingOriginWorld::new(FloatingOrigin::at_origin(), IfcsConfig::default());
    let initial = WorldPoint::new(100.0, 0.0, 0.0);
    world.spawn(1, initial, ShipBody::new_cube(1.0));
    let before = world.world_body(1).unwrap();

    assert!(world.move_camera(WorldPoint::new(2000.0, 0.0, 0.0)));
    let after = world.world_body(1).unwrap();
    assert_eq!(before.pos, after.pos, "rebase must preserve world position");
    assert_eq!(world.last_rebase_delta.unwrap().0.x, 2000.0);
    assert!(world.local_body(1).unwrap().pos.x < -1000.0);
}

#[test]
fn origin_f32_precision_loss_avoided_by_local_frame() {
    let origin = FloatingOrigin::new(large_world(), DEFAULT_REBASE_THRESHOLD);
    let near = WorldPoint::new(1_000_000.1, 250_000.1, -1_000_000.1);
    let absolute_error = f32_absolute_error(near);
    let local_error = f32_local_error(&origin, near);
    assert!(
        absolute_error > 0.01,
        "absolute f32 quantization should lose precision: {absolute_error}"
    );
    assert!(
        local_error < 1e-6,
        "local f32 quantization should be precise: {local_error}"
    );
    assert!(local_error < absolute_error);
}

#[test]
fn origin_local_physics_matches_origin_frame() {
    let cfg = IfcsConfig {
        thrust_n: 100.0,
        ..Default::default()
    };
    let mut remote = FloatingOriginWorld::new(
        FloatingOrigin::new(large_world(), DEFAULT_REBASE_THRESHOLD),
        cfg,
    );
    let mut local = FloatingOriginWorld::new(FloatingOrigin::at_origin(), cfg);

    let ship = ShipBody::new_cube(10.0);
    remote.spawn(1, large_world(), ship);
    local.spawn(1, WorldPoint::new(0.0, 0.0, 0.0), ship);

    let input = ShipInput::new(1.0, 0.25, 0.0, 0.0);
    for _ in 0..120 {
        remote.step(input);
        local.step(input);
    }
    assert_eq!(remote.local_body(1).unwrap(), local.local_body(1).unwrap());
}

#[test]
fn origin_replication_sends_relative_positions() {
    let mut world = FloatingOriginWorld::new(
        FloatingOrigin::new(large_world(), DEFAULT_REBASE_THRESHOLD),
        IfcsConfig::default(),
    );
    world.spawn(7, large_world(), ShipBody::new_cube(1.0));
    let replicated = world.replicated_position(7).unwrap();
    assert_eq!(
        replicated,
        Vec3::ZERO,
        "rebased entities sit at the local origin"
    );
    assert!(replicated.length() < 1.0);
}

#[test]
fn origin_many_rebases_do_not_drift() {
    let mut world = FloatingOriginWorld::new(FloatingOrigin::at_origin(), IfcsConfig::default());
    world.spawn(
        1,
        WorldPoint::new(50.0, 50.0, 50.0),
        ShipBody::new_cube(1.0),
    );
    let expected = world.world_body(1).unwrap().pos;
    for camera_x in (0..20).map(|step| step as f64 * 2000.0) {
        world.move_camera(WorldPoint::new(camera_x, 0.0, 0.0));
    }
    let actual = world.world_body(1).unwrap().pos;
    assert_eq!(expected, actual);
    assert_eq!(world.origin.rebase_count, 19);
}
