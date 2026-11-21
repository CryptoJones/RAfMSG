use ragfmsg::net::{MockNetwork, NetConfig};
use ragfmsg::sim::{Client, InputMessage, Server};
use ragfmsg::types::ShipInput;
use ragfmsg::world::{snapshot, Entity};

fn setup(cfg: NetConfig) -> (Server, Client, MockNetwork, u64) {
    let mut server = Server::new(Default::default());
    let id = server.spawn(Entity::cube(1000.0, true, Some(1)));
    let initial = snapshot(&server.world, server.tick);
    let mut client = Client::new(1, Default::default());
    client.receive_snapshot(&initial);
    client.predict(id);
    (server, client, MockNetwork::new(cfg), id)
}

#[test]
fn seed_reproduces_the_same_drop_pattern() {
    let cfg = NetConfig {
        input_loss: 0.5,
        seed: 7,
        ..Default::default()
    };
    let mut a = MockNetwork::new(cfg);
    let mut b = MockNetwork::new(cfg);
    for tick in 0..40 {
        let msg = InputMessage {
            tick,
            client: 1,
            target: 1,
            input: ShipInput::new(1.0, 0.0, 0.0, 0.0),
        };
        a.send_input(msg);
        b.send_input(msg);
    }
    assert_eq!(a.stats, b.stats);
    assert!(a.stats.inputs_dropped > 0);
}

#[test]
fn snapshots_arrive_after_the_configured_latency() {
    let cfg = NetConfig {
        downlink_latency: 4,
        ..Default::default()
    };
    let server = Server::new(Default::default());
    let mut client = Client::new(1, Default::default());
    let mut net = MockNetwork::new(cfg);
    net.send_snapshot(snapshot(&server.world, 0));

    for _ in 0..4 {
        assert_eq!(net.deliver_snapshots(&mut client), 0);
        net.advance();
    }
    assert_eq!(net.deliver_snapshots(&mut client), 1);
    assert_eq!(client.last_snapshot_tick, Some(0));
}

#[test]
fn jitter_reordered_snapshots_are_ignored() {
    let cfg = NetConfig {
        downlink_latency: 0,
        jitter: 6,
        seed: 4,
        ..Default::default()
    };
    let (mut server, mut client, mut net, _) = setup(cfg);
    let mut highest = 0;

    for _ in 0..80 {
        net.send_snapshot(server.step());
        net.deliver_snapshots(&mut client);
        assert!(client.last_snapshot_tick >= Some(highest));
        if let Some(tick) = client.last_snapshot_tick {
            highest = tick;
        }
        net.advance();
    }
    assert!(client.stale_snapshots_ignored > 0);
    assert_eq!(client.last_snapshot_tick, Some(highest));
}

#[test]
fn lost_inputs_fall_back_to_last_known_input() {
    let cfg = NetConfig {
        input_loss: 0.5,
        seed: 5,
        ..Default::default()
    };
    let (mut server, mut client, mut net, id) = setup(cfg);
    let input = ShipInput::new(0.5, 0.0, 0.0, 0.0);

    for _ in 0..40 {
        let msg = client.send_input(id, input);
        net.send_input(msg);
        client.step(input);
        net.deliver_inputs(&mut server);
        net.send_snapshot(server.step());
        net.deliver_snapshots(&mut client);
        net.advance();
    }

    assert!(net.stats.inputs_dropped > 0);
    assert_eq!(server.world.get(id).unwrap().input, input);
    assert!(server.world.get(id).unwrap().body.vel.z < 0.0);
}

#[test]
fn confirmed_state_is_exactly_a_server_snapshot() {
    let cfg = NetConfig {
        downlink_latency: 2,
        jitter: 2,
        snapshot_loss: 0.2,
        seed: 11,
        ..Default::default()
    };
    let (mut server, mut client, mut net, id) = setup(cfg);

    for tick in 0..80 {
        let input = ShipInput::new(if tick % 7 < 4 { 1.0 } else { 0.0 }, 0.1, 0.0, 0.0);
        let msg = client.send_input(id, input);
        net.send_input(msg);
        client.step(input);
        net.deliver_inputs(&mut server);
        net.send_snapshot(server.step());
        net.deliver_snapshots(&mut client);
        net.advance();
    }

    assert!(net.stats.snapshots_dropped > 0);
    let tick = client.last_snapshot_tick.unwrap();
    assert_eq!(client.confirmed[&id], net.sent_snapshots[&tick].bodies[&id]);
}

#[test]
fn loss_and_latency_trigger_bounded_reconciliation() {
    let cfg = NetConfig {
        uplink_latency: 2,
        downlink_latency: 1,
        input_loss: 0.4,
        jitter: 1,
        seed: 9,
        ..Default::default()
    };
    let (mut server, mut client, mut net, id) = setup(cfg);

    for tick in 0..80 {
        let input = ShipInput::new(if tick % 11 < 6 { 1.0 } else { 0.0 }, 0.2, 0.0, 0.0);
        let msg = client.send_input(id, input);
        net.send_input(msg);
        client.step(input);
        net.deliver_inputs(&mut server);
        net.send_snapshot(server.step());
        net.deliver_snapshots(&mut client);
        net.advance();
    }

    assert!(client.rollbacks > 0);
    assert!(client.rollbacks < 80);
    let tick = client.last_snapshot_tick.unwrap();
    assert_eq!(client.confirmed[&id], net.sent_snapshots[&tick].bodies[&id]);
}
