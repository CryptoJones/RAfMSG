use bevy_ecs::prelude::*;
use lightyear::prelude::{Interpolated, Predicted};
use ragfmsg::ifcs::IfcsConfig;
use ragfmsg::physics::advance;
use ragfmsg::prediction::*;
use ragfmsg::types::{EntityId, ShipBody, ShipInput};
use std::collections::BTreeMap;

fn client() -> PredictionClient {
    PredictionClient::new(ReconcileConfig::default(), IfcsConfig::default())
}

fn ship() -> ShipBody {
    ShipBody::new_cube(10.0)
}

fn snapshot(_tick: u32, body: ShipBody) -> BTreeMap<EntityId, ShipBody> {
    BTreeMap::from([(1, body)])
}

#[test]
fn prediction_clean_run_has_no_rollbacks() {
    let ifcs = IfcsConfig::default();
    let mut server = ship();
    let mut client = client();
    client.predict(1, server);
    let input = ShipInput::new(1.0, 0.2, 0.0, 0.0);

    for _ in 0..30 {
        advance(&mut server, &input, &ifcs, 1.0 / 60.0);
        client.step(input);
        client.receive_snapshot(client.tick, snapshot(client.tick, server));
    }
    assert_eq!(client.rollbacks, 0);
    let predicted = client.predicted_body(1).unwrap();
    assert!(predicted.pos.distance(server.pos) < 0.01);
    assert!(predicted.vel.distance(server.vel) < 0.01);
}

#[test]
fn prediction_divergence_triggers_rollback() {
    let ifcs = IfcsConfig::default();
    let mut server = ship();
    let mut client = client();
    client.predict(1, server);
    let normal = ShipInput::new(1.0, 0.0, 0.0, 0.0);
    let wrong = ShipInput::new(-1.0, 0.0, 0.0, 0.0);

    for t in 0..20 {
        advance(&mut server, &normal, &ifcs, 1.0 / 60.0);
        client.step(if t == 10 { wrong } else { normal });
    }
    client.receive_snapshot(client.tick, snapshot(client.tick, server));
    assert!(
        client.rollbacks > 0,
        "mis-prediction should cause reconciliation"
    );
    let predicted = client.predicted_body(1).unwrap();
    assert!(predicted.pos.distance(server.pos) < 0.01);
}

#[test]
fn prediction_rejects_stale_snapshots() {
    let mut client = client();
    let mut body = ship();
    body.pos.x = 5.0;
    client.receive_snapshot(10, snapshot(10, body));
    let mut stale = ship();
    stale.pos.x = 99.0;
    client.receive_snapshot(9, snapshot(9, stale));
    assert_eq!(client.stale_snapshots_ignored, 1);
    assert_eq!(client.last_snapshot_tick, Some(10));
    assert_eq!(client.confirmed_bodies[&1].pos.x, 5.0);
}

#[test]
fn prediction_history_is_bounded() {
    let mut client = client();
    client.predict(1, ship());
    for _ in 0..80 {
        client.step(ShipInput::NONE);
    }
    assert!(client.history_len(1) <= client.config.history_ticks as usize + 1);
    assert_eq!(client.tick_stamp().0, 80);
}

#[test]
fn prediction_interpolates_between_snapshots() {
    let config = ReconcileConfig {
        interpolation_delay_ticks: 5,
        ..Default::default()
    };
    let mut client = PredictionClient::new(config, IfcsConfig::default());
    client.observe(2);
    let mut start = ship();
    start.pos.x = 0.0;
    let mut end = ship();
    end.pos.x = 10.0;
    client.receive_snapshot(0, BTreeMap::from([(2, start)]));
    client.receive_snapshot(10, BTreeMap::from([(2, end)]));

    client.tick = 10;
    let sampled = client.interpolated_body(2).unwrap();
    assert!(
        (sampled.pos.x - 5.0).abs() < 1e-6,
        "expected midpoint, got {}",
        sampled.pos.x
    );
    client.tick = 8;
    let sampled = client.interpolated_body(2).unwrap();
    assert!(
        (sampled.pos.x - 3.0).abs() < 1e-6,
        "expected interpolation at target tick 3, got {}",
        sampled.pos.x
    );
}

#[test]
fn prediction_interpolation_respects_delay() {
    let config = ReconcileConfig {
        interpolation_delay_ticks: 2,
        ..Default::default()
    };
    let mut client = PredictionClient::new(config, IfcsConfig::default());
    client.observe(2);
    let mut start = ship();
    start.pos.x = 0.0;
    let mut end = ship();
    end.pos.x = 10.0;
    client.receive_snapshot(0, BTreeMap::from([(2, start)]));
    client.receive_snapshot(10, BTreeMap::from([(2, end)]));

    client.tick = 7;
    let sampled = client.interpolated_body(2).unwrap();
    assert!(
        (sampled.pos.x - 5.0).abs() < 1e-6,
        "expected delayed midpoint, got {}",
        sampled.pos.x
    );
}

#[test]
fn prediction_tracks_marker_kinds() {
    let mut c1 = client();
    let mut c2 = client();
    let server = ship();
    c1.predict(1, server);
    c2.observe(2);
    c2.receive_snapshot(1, snapshot(1, server));
    assert!(c1.is_predicted(1));
    assert!(c2.is_interpolated(2));
    assert!(!c1.is_interpolated(1));
}

#[test]
fn prediction_lightyear_markers_are_bevy_components() {
    let mut world = World::new();
    let entity = world.spawn((predicted_component(),)).id();
    assert!(world.get::<Predicted>(entity).is_some());
    let entity = world.spawn((interpolated_component(),)).id();
    assert!(world.get::<Interpolated>(entity).is_some());
}
