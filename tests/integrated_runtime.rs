use ragfmsg::backend::{BackendError, BackendService, IdentityRegistry, Persistence, SessionId};
use ragfmsg::flight::FlightTuning;
use ragfmsg::floating_origin::{FloatingOrigin, FloatingOriginWorld, WorldPoint};
use ragfmsg::ifcs::IfcsConfig;
use ragfmsg::input::{InputSource, SequencedInput};
use ragfmsg::interest::InterestConfig;
use ragfmsg::meshing::Lease;
use ragfmsg::network::{InputMessage as NetInput, SnapshotMessage};
use ragfmsg::runtime::{IntegratedRuntime, RuntimeError};
use ragfmsg::test_harness::run_seam_suite;
use ragfmsg::types::{ShipBody, ShipInput, Vec3};
use ragfmsg::world::{try_client_mutate_at_tick, AuthorityError, Entity, Snapshot, World};

#[test]
fn integrated_runtime_full_eight_seam_pipeline() {
    let secret = b"runtime-secret";
    let backend = BackendService::new(secret, Persistence::open_in_memory().unwrap());
    let mut rt = IntegratedRuntime::new(
        secret,
        IfcsConfig::default(),
        InterestConfig::default(),
        backend,
    );

    let token = rt.backend.auth.issue_token_with_expiry("pilot-1", 1000);
    let spawn_pos = WorldPoint::new(5_000.0, 0.0, 0.0);
    let (client_id, entity_id) = rt.authenticate_and_join(&token, 100, 4, spawn_pos).unwrap();
    assert_eq!(rt.identity.user_of(client_id), Some("pilot-1"));

    rt.submit_input(client_id, entity_id, ShipInput::new(1.0, 0.0, 0.0, 0.0))
        .unwrap();
    for _ in 0..10 {
        rt.step(spawn_pos);
    }

    assert_eq!(rt.tick, 10);
    assert!(rt.profiler.sent.messages >= 10);
    assert!(rt.clients[&client_id].last_snapshot_tick.is_some());

    rt.persist_client_ship(&token, 200, entity_id).unwrap();
    let restored = rt
        .backend
        .reconnect_authenticated(&token, 250)
        .unwrap()
        .unwrap();
    assert!((restored.pos_x - 5_000.0).abs() < 1e-6);
}

#[test]
fn integrated_runtime_rejects_expired_token_on_join_and_persist() {
    let secret = b"runtime-secret";
    let backend = BackendService::new(secret, Persistence::open_in_memory().unwrap());
    let mut rt = IntegratedRuntime::new(
        secret,
        IfcsConfig::default(),
        InterestConfig::default(),
        backend,
    );
    let token = rt.backend.auth.issue_token_with_expiry("pilot-2", 50);
    assert_eq!(
        rt.authenticate_and_join(&token, 50, 4, WorldPoint::new(0.0, 0.0, 0.0)),
        Err(RuntimeError::Backend(BackendError::TokenExpired))
    );
    assert_eq!(
        rt.persist_client_ship(&token, 50, 1),
        Err(BackendError::TokenExpired)
    );
}

#[test]
fn security_hardening_tokens_sessions_and_signed_leases() {
    let backend = BackendService::new(b"s3cr3t", Persistence::open_in_memory().unwrap());
    let exp_token = backend.auth.issue_token_with_expiry("alice", 100);
    assert_eq!(backend.auth.verify_at(&exp_token, 99).unwrap(), "alice");
    assert_eq!(
        backend.auth.verify_at(&exp_token, 100),
        Err(BackendError::TokenExpired)
    );

    // SessionIds are random and non-sequential.
    let s1 = SessionId::new();
    let s2 = SessionId::new();
    assert_ne!(s1, s2);
    assert_ne!(s1.0.wrapping_add(1), s2.0);

    // Signed lease verifies and detects tampering.
    let lease = Lease {
        entity: 1,
        owner: 2,
        generation: 1,
        body: ShipBody::new_cube(10.0).into(),
    };
    let mut signed = lease.sign(b"mesh-key");
    assert!(signed.verify(b"mesh-key"));
    assert!(!signed.verify(b"wrong-key"));
    signed.lease.owner = 99;
    assert!(!signed.verify(b"mesh-key"));
}

#[test]
fn unified_configs_identity_shapes_and_stale_tick() {
    // Config conversion
    let ifcs = IfcsConfig {
        thrust_n: 2500.0,
        torque_nm: 300.0,
        g_limit_ms2: 50.0,
        ang_damping: 0.2,
    };
    let tuning = FlightTuning::from(ifcs);
    let back: IfcsConfig = (&tuning).into();
    assert_eq!(back.thrust_n, ifcs.thrust_n);
    assert_eq!(back.torque_nm, ifcs.torque_nm);
    assert_eq!(back.g_limit_ms2, ifcs.g_limit_ms2);
    assert_eq!(back.ang_damping, ifcs.ang_damping);

    // Message shape conversions
    let seq = SequencedInput::new(
        1,
        7,
        3,
        9,
        InputSource::Keyboard,
        ShipInput::new(0.5, -0.25, 0.1, 0.0),
    );
    let sim_msg: ragfmsg::sim::InputMessage = (&seq).into();
    let net_msg = NetInput::from(sim_msg);
    let roundtrip = net_msg.into_sim(9);
    assert_eq!(roundtrip, sim_msg);

    // Snapshot conversion + floating origin wire body
    let mut origin_world = FloatingOriginWorld::new(FloatingOrigin::at_origin(), ifcs);
    origin_world.spawn(
        1,
        WorldPoint::new(100.0, 200.0, 300.0),
        ShipBody::new_cube(10.0),
    );
    let wire = origin_world.replicated_wire_body(1).unwrap();
    assert_eq!(wire.px, 100.0);
    let net_snap: SnapshotMessage = origin_world.replicated_snapshot(4);
    let world_snap: Snapshot = net_snap.to_world_snapshot();
    assert_eq!(world_snap.tick, 4);
    assert_eq!(world_snap.bodies[&1].pos, Vec3::new(100.0, 200.0, 300.0));

    // Identity registry
    let mut reg = IdentityRegistry::new();
    let c1 = reg.get_or_assign("alice");
    let c2 = reg.get_or_assign("bob");
    assert_ne!(c1, c2);
    assert_eq!(reg.get_or_assign("alice"), c1);

    // StaleTick error produced
    let mut world = World::new();
    let id = world.spawn(Entity::cube(10.0, true, Some(c1)));
    assert_eq!(
        try_client_mutate_at_tick(&mut world, c1, id, ShipBody::new_cube(10.0), 3, 5),
        Err(AuthorityError::StaleTick)
    );
}

#[test]
fn cross_seam_harness_suite_passes_all_eight_seams() {
    let report = run_seam_suite();
    assert!(report.all_passed(), "failures: {:?}", report.failed);
    assert!(report.skipped.is_empty(), "skipped: {:?}", report.skipped);
    assert_eq!(report.passed.len(), 8);
}
