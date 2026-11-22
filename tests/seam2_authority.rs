use ragfmsg::sim::Server;
use ragfmsg::types::{ShipInput, DT};
use ragfmsg::world::{AuthorityError, Entity};

fn server_with_cube(owner: u64) -> (Server, u64) {
    let mut server = Server::new(Default::default());
    let id = server.spawn(Entity::cube(1000.0, true, Some(owner)));
    (server, id)
}

#[test]
fn input_applies_to_correct_entity() {
    let (mut server, id) = server_with_cube(1);
    let msg = ragfmsg::sim::InputMessage {
        tick: 0,
        client: 1,
        target: id,
        input: ShipInput::new(1.0, 0.5, 0.0, 0.0),
    };
    assert!(server.submit_input(msg).is_ok());
    server.step();

    let body = server.world.get(id).unwrap().body;
    assert!((body.vel.z + DT).abs() < 1e-9);
    assert!(body.ang_vel.z > 0.0);
}

#[test]
fn authority_check_rejects_other_client() {
    let (mut server, id) = server_with_cube(2);
    let msg = ragfmsg::sim::InputMessage {
        tick: 0,
        client: 1,
        target: id,
        input: ShipInput::new(1.0, 0.0, 0.0, 0.0),
    };
    assert_eq!(server.submit_input(msg), Err(AuthorityError::NotOwner));
    server.step();
    assert_eq!(server.world.get(id).unwrap().body.vel.z, 0.0);
}

#[test]
fn lost_packets_fall_back_to_last_known_input() {
    let (mut server, id) = server_with_cube(1);
    let input = ShipInput::new(0.5, 0.0, 0.0, 0.0);
    server
        .submit_input(ragfmsg::sim::InputMessage {
            tick: 0,
            client: 1,
            target: id,
            input,
        })
        .unwrap();
    server.step();

    for _ in 0..5 {
        server.step();
        assert_eq!(server.world.get(id).unwrap().input, input);
    }
    assert!(server.world.get(id).unwrap().body.vel.z < 0.0);
}

#[test]
fn no_input_means_no_runaway() {
    let (mut server, id) = server_with_cube(1);
    for _ in 0..120 {
        server.step();
    }
    let body = server.world.get(id).unwrap().body;
    assert_eq!(body.vel, ragfmsg::types::Vec3::ZERO);
    assert_eq!(body.pos, ragfmsg::types::Vec3::ZERO);
}

#[test]
fn replayed_input_reproduces_server_state() {
    use ragfmsg::sim::Client;
    let (mut server, id) = server_with_cube(1);
    let initial = ragfmsg::world::snapshot(&server.world, server.tick);
    let mut client = Client::new(1, Default::default());
    client.receive_snapshot(&initial);
    client.predict(id);

    for _ in 0..30 {
        let input = ShipInput::new(1.0, 0.2, 0.0, 0.0);
        let msg = client.send_input(id, input);
        client.step(input);
        server.submit_input(msg).unwrap();
        let snap = server.step();
        client.receive_snapshot(&snap);
    }

    assert_eq!(client.rollbacks, 0);
    assert_eq!(
        client.body(id).unwrap().pos,
        server.world.get(id).unwrap().body.pos
    );
}
