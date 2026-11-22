use ragfmsg::network::*;
use ragfmsg::types::{Quat, ShipBody, Vec3};
use std::collections::BTreeMap;

fn body(x: f64) -> ShipBody {
    ShipBody {
        pos: Vec3::new(x, 0.0, 0.0),
        vel: Vec3::ZERO,
        ori: Quat::IDENTITY,
        ang_vel: Vec3::new(0.1, -0.2, 0.3),
        mass: 10.0,
    }
}

#[test]
fn net_serialization_roundtrip() {
    let input = InputMessage {
        tick: 42,
        client: 7,
        throttle: 1.0,
        yaw: 0.5,
        pitch: -0.25,
        roll: 0.0,
    };
    let bytes = encode(&input).unwrap();
    let decoded: InputMessage = decode(&bytes).unwrap();
    assert_eq!(decoded, input);
    assert_eq!(decoded.tick(), lightyear::prelude::Tick(42));

    let snapshot = SnapshotMessage {
        tick: 9,
        bodies: vec![(3, WireBody::from(body(1.5)))],
    };
    let decoded: SnapshotMessage = decode(&encode(&snapshot).unwrap()).unwrap();
    assert_eq!(decoded, snapshot);
    let restored: ShipBody = decoded.bodies[0].1.into();
    assert_eq!(restored, body(1.5));
}

#[test]
fn net_channels_and_memory_transport() {
    let mut transport = MemoryTransport::new();
    transport.send(Channel::Reliable, 1, b"reliable").unwrap();
    transport
        .send(Channel::Unreliable, 2, b"unreliable")
        .unwrap();
    assert_eq!(transport.sent.len(), 2);
    let received = transport.recv();
    assert_eq!(received.len(), 2);
    assert_eq!(received[0], (1, Channel::Reliable, b"reliable".to_vec()));
    assert_eq!(
        received[1],
        (2, Channel::Unreliable, b"unreliable".to_vec())
    );
    assert!(transport.recv().is_empty());

    transport.deliver(3, Channel::Unreliable, b"drop".to_vec());
    assert_eq!(transport.drop_pending(), 1);
    assert!(transport.recv().is_empty());
}

#[test]
fn net_sync_modes_map_to_lightyear() {
    assert!(matches!(
        SyncMode::Full.to_lightyear(),
        Some(bevy_replicon::prelude::ReplicationMode::OnChange)
    ));
    assert!(matches!(
        SyncMode::Once.to_lightyear(),
        Some(bevy_replicon::prelude::ReplicationMode::Once)
    ));
    assert!(SyncMode::None.to_lightyear().is_none());
}

#[test]
fn net_component_sync_config() {
    let mut rules = ReplicationRules::new();
    rules.register("ShipBody", SyncMode::Full, Target::All);
    rules.register("Hull", SyncMode::Once, Target::All);
    rules.register("ServerSecret", SyncMode::None, Target::All);

    for tick in 1..=3 {
        assert!(rules.should_send("ShipBody", 1, 10, tick));
        assert_eq!(rules.should_send("Hull", 1, 10, tick), tick == 1);
        assert!(!rules.should_send("ServerSecret", 1, 10, tick));
    }
    assert_eq!(rules.mode_for("ShipBody"), SyncMode::Full);
    assert_eq!(rules.mode_for("Missing"), SyncMode::None);
}

#[test]
fn net_target_filtering_and_rate() {
    let mut rules = ReplicationRules::with_rate(3);
    rules.register("ShipBody", SyncMode::Full, Target::Single(7));

    let mut bodies = BTreeMap::new();
    bodies.insert(1, body(1.0));
    assert!(build_snapshot(&bodies, 7, 0, &mut rules).bodies.len() == 1);
    assert!(build_snapshot(&bodies, 8, 0, &mut rules).bodies.is_empty());
    assert!(build_snapshot(&bodies, 7, 1, &mut rules).bodies.is_empty());
    assert!(build_snapshot(&bodies, 7, 3, &mut rules).bodies.len() == 1);
}

#[test]
fn net_delta_compression() {
    let mut baseline = BTreeMap::new();
    baseline.insert(1, WireBody::from(body(1.0)));
    baseline.insert(2, WireBody::from(body(2.0)));

    let mut current = baseline.clone();
    current.insert(2, WireBody::from(body(20.0)));
    current.insert(3, WireBody::from(body(3.0)));

    let delta = encode_delta(5, &baseline, &current);
    assert_eq!(delta.changed.len(), 2);
    assert!(delta.removed.is_empty());
    apply_delta(&mut baseline, &delta);
    assert_eq!(baseline, current);

    let full = SnapshotMessage {
        tick: 5,
        bodies: current.iter().map(|(id, b)| (*id, *b)).collect(),
    };
    let delta_bytes = encode(&delta).unwrap().len();
    let full_bytes = encode(&full).unwrap().len();
    assert!(
        delta_bytes < full_bytes,
        "delta {delta_bytes} < full {full_bytes}"
    );
}

#[test]
fn net_bandwidth_profiler() {
    let mut profiler = BandwidthProfiler::default();
    let message = InputMessage {
        tick: 1,
        client: 1,
        throttle: 1.0,
        yaw: 0.0,
        pitch: 0.0,
        roll: 0.0,
    };
    let bytes = profiler.record_send(&message, 128).unwrap();
    let decoded: InputMessage = profiler.record_receive(&bytes, 128).unwrap();
    assert_eq!(decoded, message);
    assert_eq!(profiler.sent.messages, 1);
    assert_eq!(profiler.received.messages, 1);
    assert!(profiler.sent.wire_bytes > 0);
    assert!(profiler.sent.compression_ratio() > 1.0);
}

#[test]
fn net_connection_lifecycle() {
    let mut pending = Connection::begin(1, 0, 5);
    assert_eq!(pending.state, ConnectionState::Connecting);
    assert!(!pending.update(5));
    pending.heartbeat(5);
    assert_eq!(pending.state, ConnectionState::Connected);
    assert!(pending.update(11));
    assert_eq!(pending.state, ConnectionState::TimedOut);

    let mut conn = Connection::connect(1, 0, 5);
    assert_eq!(conn.state, ConnectionState::Connected);
    assert!(!conn.update(5));
    assert!(conn.update(6));
    assert_eq!(conn.state, ConnectionState::TimedOut);
    conn.heartbeat(10);
    assert_eq!(conn.state, ConnectionState::Connected);
    conn.disconnect();
    assert_eq!(conn.state, ConnectionState::Disconnected);
}

#[test]
fn net_reliable_retransmission() {
    let mut queue = ReliableQueue::default();
    let seq = queue.send(b"packet");
    let mut transport = MemoryTransport::new();
    assert_eq!(queue.retransmit(&mut transport, 1), 1);
    assert_eq!(queue.retransmit(&mut transport, 1), 1);
    assert!(queue.ack(seq));
    assert!(!queue.ack(seq));
    assert_eq!(queue.pending_count(), 0);
    assert_eq!(queue.retransmit(&mut transport, 1), 0);
    assert_eq!(transport.sent.len(), 2);
}

#[test]
fn net_udp_transport_roundtrip() {
    let mut server = UdpTransport::bind("127.0.0.1:0").unwrap();
    let mut client = UdpTransport::bind("127.0.0.1:0").unwrap();
    let server_addr = server.local_addr();
    let client_addr = client.local_addr();
    server.add_peer(1, client_addr);
    client.add_peer(99, server_addr);

    client.send(Channel::Unreliable, 99, b"hello-udp").unwrap();
    let received = server.recv();
    assert_eq!(received.len(), 1);
    assert_eq!(received[0].2, b"hello-udp");
}
