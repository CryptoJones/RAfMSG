use ragfmsg::interest::*;
use ragfmsg::types::Vec3;

fn manager() -> InterestManager {
    InterestManager::new(InterestConfig {
        sensor_range: 100.0,
        exit_range: 120.0,
        update_interval_ticks: 1,
        hysteresis_updates: 2,
    })
}

fn tick_all(mesh: &mut InterestManager, ticks: u32) {
    for _ in 0..ticks {
        mesh.update();
    }
}

#[test]
fn interest_drops_entities_beyond_sensor_range() {
    let mut mesh = manager();
    mesh.register_client(1, Vec3::ZERO);
    mesh.set_position(10, Vec3::ZERO);
    mesh.set_position(20, Vec3::new(500.0, 0.0, 0.0));
    tick_all(&mut mesh, 4);
    let view = mesh.visible_for(1);
    assert!(view.contains(&10));
    assert!(!view.contains(&20));
    assert_eq!(mesh.client_count(), 1);
}

#[test]
fn interest_reappears_when_entities_move_closer() {
    let mut mesh = manager();
    mesh.register_client(1, Vec3::ZERO);
    mesh.set_position(20, Vec3::new(500.0, 0.0, 0.0));
    tick_all(&mut mesh, 4);
    assert!(!mesh.visible_for(1).contains(&20));

    mesh.set_position(20, Vec3::new(50.0, 0.0, 0.0));
    tick_all(&mut mesh, 4);
    assert!(mesh.visible_for(1).contains(&20));
}

#[test]
fn interest_leaving_view_is_not_destroy() {
    let mut mesh = manager();
    mesh.register_client(1, Vec3::ZERO);
    mesh.set_position(20, Vec3::ZERO);
    tick_all(&mut mesh, 2);
    assert!(mesh.visible_for(1).contains(&20));

    mesh.set_position(20, Vec3::new(500.0, 0.0, 0.0));
    tick_all(&mut mesh, 4);
    assert!(!mesh.visible_for(1).contains(&20));
    assert!(
        mesh.entity_exists(20),
        "server entity must survive leaving interest"
    );

    mesh.set_position(20, Vec3::ZERO);
    tick_all(&mut mesh, 2);
    assert!(mesh.visible_for(1).contains(&20));
}

#[test]
fn interest_hysteresis_prevents_flicker() {
    let mut mesh = manager();
    mesh.register_client(1, Vec3::ZERO);
    mesh.set_position(20, Vec3::ZERO);
    tick_all(&mut mesh, 2);
    assert!(mesh.visible_for(1).contains(&20));

    mesh.set_position(20, Vec3::new(110.0, 0.0, 0.0));
    tick_all(&mut mesh, 10);
    assert!(
        mesh.visible_for(1).contains(&20),
        "inside hysteresis band stays visible"
    );

    mesh.set_position(20, Vec3::new(130.0, 0.0, 0.0));
    mesh.update();
    assert!(
        mesh.visible_for(1).contains(&20),
        "first miss does not drop yet"
    );
    mesh.update();
    assert!(!mesh.visible_for(1).contains(&20), "second miss drops it");
}

#[test]
fn interest_targeting_enforcement() {
    let mut mesh = manager();
    mesh.register_client(1, Vec3::ZERO);
    mesh.set_position(10, Vec3::ZERO);
    mesh.set_position(20, Vec3::new(500.0, 0.0, 0.0));
    tick_all(&mut mesh, 2);
    assert!(mesh.can_target(1, 10));
    assert!(!mesh.can_target(1, 20));
    assert!(!mesh.can_target(2, 10));
}

#[test]
fn interest_replication_filter() {
    let mut mesh = manager();
    mesh.register_client(1, Vec3::ZERO);
    for entity in 0..10 {
        mesh.set_position(entity, Vec3::new(entity as f64 * 20.0, 0.0, 0.0));
    }
    tick_all(&mut mesh, 2);
    let candidates: Vec<u64> = (0..10).collect();
    let relevant = mesh.relevant_for(1, &candidates);
    assert_eq!(relevant, vec![0, 1, 2, 3, 4, 5]);
}

#[test]
fn interest_spatial_index_and_update_interval() {
    let config = InterestConfig {
        update_interval_ticks: 10,
        ..Default::default()
    };
    let mut mesh = InterestManager::new(config);
    for entity in 0..100 {
        mesh.set_position(entity, Vec3::new(entity as f64 * 100.0, 0.0, 0.0));
    }
    let near = mesh.nearby(Vec3::ZERO, 250.0);
    assert_eq!(
        near.len(),
        3,
        "rstar should find entities 0,1,2 within 250m"
    );
    for tick in 1..10 {
        assert!(
            !mesh.update(),
            "no relevance check before interval at tick {tick}"
        );
    }
    assert!(mesh.update());
    assert_eq!(mesh.updates, 1);
}
