use ragfmsg::sim::Server;
use ragfmsg::types::{ShipBody, ShipInput, Vec3};
use ragfmsg::world::{apply_snapshot, snapshot, try_client_mutate, AuthorityError, Entity};

fn server_with_cube() -> (Server, u64) {
    let mut server = Server::new(Default::default());
    let id = server.spawn(Entity::cube(1000.0, true, Some(1)));
    (server, id)
}

#[test]
fn spawn_replicates_to_client() {
    let (server, id) = server_with_cube();
    let snap = snapshot(&server.world, server.tick);
    assert_eq!(
        snap.bodies.get(&id),
        Some(&server.world.get(id).unwrap().body)
    );

    let mut client_world = ragfmsg::world::World::new();
    apply_snapshot(&mut client_world, &snap);
    assert_eq!(
        client_world.get(id).unwrap().body,
        server.world.get(id).unwrap().body
    );
}

#[test]
fn mutation_replicates_within_epsilon() {
    let (mut server, id) = server_with_cube();
    server.world.get_mut(id).unwrap().body.pos = Vec3::new(7.0, 8.0, 9.0);

    let snap = server.step();
    let mut client_world = ragfmsg::world::World::new();
    apply_snapshot(&mut client_world, &snap);

    let server_pos = server.world.get(id).unwrap().body.pos;
    let client_pos = client_world.get(id).unwrap().body.pos;
    assert!(client_pos.distance(server_pos) < ragfmsg::types::POS_EPS);
    assert!((client_pos.x - 7.0).abs() < 1e-9);
}

#[test]
fn non_replicated_state_does_not_cross() {
    let (mut server, replicated) = server_with_cube();
    let hidden = server.spawn(Entity::cube(1000.0, false, None));
    server.world.get_mut(replicated).unwrap().server_only = Some(42);

    let snap = server.step();
    assert!(snap.bodies.contains_key(&replicated));
    assert!(!snap.bodies.contains_key(&hidden));

    let mut client_world = ragfmsg::world::World::new();
    apply_snapshot(&mut client_world, &snap);
    assert!(client_world.get(hidden).is_none());
    assert_eq!(client_world.get(replicated).unwrap().server_only, None);
}

#[test]
fn client_cannot_mutate_server_authoritative_component() {
    let (mut server, owned) = server_with_cube();
    let server_owned = server.spawn(Entity::cube(1000.0, true, None));
    let mut world = server.world;
    let mut forged = ShipBody::new_cube(1000.0);
    forged.pos = Vec3::new(1000.0, 0.0, 0.0);

    assert_eq!(
        try_client_mutate(&mut world, 1, server_owned, forged),
        Err(AuthorityError::NotOwner)
    );
    assert_eq!(world.get(server_owned).unwrap().body.pos, Vec3::ZERO);

    assert!(try_client_mutate(&mut world, 1, owned, forged).is_ok());
    assert_eq!(world.get(owned).unwrap().body.pos, forged.pos);
}

#[test]
fn wrong_client_cannot_mutate_another_clients_entity() {
    let (server, id) = server_with_cube();
    let mut world = server.world;
    assert_eq!(
        try_client_mutate(&mut world, 2, id, ShipBody::new_cube(1000.0)),
        Err(AuthorityError::NotOwner)
    );
}

#[test]
fn plain_ship_input_is_zero() {
    assert_eq!(ShipInput::NONE, ShipInput::new(0.0, 0.0, 0.0, 0.0));
}
