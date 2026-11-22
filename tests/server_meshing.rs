use ragfmsg::meshing::*;
use ragfmsg::types::{ShipBody, Vec3};

fn cluster() -> MeshCluster {
    let mut mesh = MeshCluster::new();
    mesh.add_server(1);
    mesh.add_server(2);
    mesh.add_server(3);
    mesh
}

fn body(x: f64) -> ShipBody {
    let mut ship = ShipBody::new_cube(10.0);
    ship.pos = Vec3::new(x, 0.0, 0.0);
    ship
}

#[test]
fn mesh_single_authority_and_shadows() {
    let mut mesh = cluster();
    mesh.spawn(10, body(1.0), 1).unwrap();
    assert_eq!(mesh.authority_of(10), Some(1));
    assert_eq!(mesh.authority_count(), 1);
    assert!(mesh.is_shadow(2, 10));
    assert!(mesh.is_shadow(3, 10));
    assert!(!mesh.is_shadow(1, 10));
    assert_eq!(mesh.shadow_body(2, 10).unwrap().pos.x, 1.0);
    assert_eq!(mesh.authoritative_body(10).unwrap().pos.x, 1.0);
    assert_eq!(mesh.spawn(10, body(2.0), 2), Err(MeshError::AlreadyOwned));
}

#[test]
fn mesh_shadow_is_read_only() {
    let mut mesh = cluster();
    mesh.spawn(10, body(1.0), 1).unwrap();
    assert!(mesh.is_shadow(2, 10));
    assert!(mesh.shadow_body(2, 10).is_some());
    assert_eq!(mesh.authority_of(10), Some(1));
}

#[test]
fn mesh_two_phase_handoff() {
    let mut mesh = cluster();
    mesh.spawn(10, body(1.0), 1).unwrap();
    let (token, lease) = mesh.begin_handoff(10, 1, 2).unwrap();
    assert_eq!(lease.owner, 2);
    assert_eq!(lease.generation, 1);
    assert_eq!(mesh.state_of(token), Some(HandoffState::Preparing));
    assert_eq!(
        mesh.authority_of(10),
        Some(1),
        "old server owns until commit"
    );

    mesh.acknowledge(token).unwrap();
    assert_eq!(
        mesh.state_of(token),
        Some(HandoffState::AwaitingAck { token })
    );
    mesh.commit(token).unwrap();
    assert_eq!(mesh.authority_of(10), Some(2));
    assert_eq!(mesh.generation_of(10), Some(1));
    assert!(!mesh.is_shadow(2, 10));
    assert!(mesh.is_shadow(1, 10));
    assert_eq!(
        mesh.state_of(token),
        Some(HandoffState::Committed { token })
    );
    assert_eq!(mesh.commits, 1);
}

#[test]
fn mesh_rejects_invalid_handoff_calls() {
    let mut mesh = cluster();
    mesh.spawn(10, body(1.0), 1).unwrap();
    assert_eq!(mesh.begin_handoff(10, 2, 3), Err(MeshError::NotAuthority));
    assert_eq!(mesh.begin_handoff(99, 1, 2), Err(MeshError::UnknownEntity));
    let (token, _) = mesh.begin_handoff(10, 1, 2).unwrap();
    assert_eq!(mesh.commit(token), Err(MeshError::BadState));
    mesh.acknowledge(token).unwrap();
    assert_eq!(mesh.acknowledge(token), Err(MeshError::BadState));
}

#[test]
fn mesh_recovers_from_failed_server() {
    let mut mesh = cluster();
    mesh.spawn(10, body(1.0), 1).unwrap();
    let (token, _) = mesh.begin_handoff(10, 1, 2).unwrap();
    mesh.acknowledge(token).unwrap();
    mesh.recover(token, 1).unwrap();
    assert_eq!(mesh.authority_of(10), Some(2));
    assert_eq!(mesh.state_of(token), Some(HandoffState::Aborted { token }));
    assert_eq!(mesh.aborts, 1);
    assert_eq!(mesh.recover(token, 1), Err(MeshError::BadState));
}

#[test]
fn mesh_coordinator_load_balances() {
    let mut mesh = cluster();
    for entity in 0..9 {
        let owner = mesh.coordinator.assign().unwrap();
        mesh.spawn(entity, body(entity as f64), owner).unwrap();
    }
    let loads: Vec<usize> = (1..=3)
        .map(|server| mesh.coordinator.load_of(server))
        .collect();
    assert_eq!(loads, vec![3, 3, 3], "entities should spread evenly");
    assert_eq!(mesh.authority_count(), 9);
}

#[test]
fn mesh_lease_serializes_with_bincode() {
    let lease = Lease {
        entity: 7,
        owner: 2,
        generation: 3,
        body: body(4.0).into(),
    };
    let bytes = ragfmsg::network::encode(&lease).unwrap();
    let decoded: Lease = ragfmsg::network::decode(&bytes).unwrap();
    assert_eq!(decoded, lease);
}
