use ragfmsg::ifcs::IfcsConfig;
use ragfmsg::physics::advance;
use ragfmsg::sim::{Client, Server};
use ragfmsg::types::{ShipBody, ShipInput, Vec3, DT, ORI_EPS, POS_EPS};
use ragfmsg::world::{snapshot, Entity};

fn paired() -> (Server, Client, u64) {
    let cfg = IfcsConfig::default();
    let mut server = Server::new(cfg);
    let id = server.spawn(Entity::cube(1000.0, true, Some(1)));
    let initial = snapshot(&server.world, server.tick);
    let mut client = Client::new(1, cfg);
    client.receive_snapshot(&initial);
    client.predict(id);
    (server, client, id)
}

#[test]
fn identical_inputs_give_identical_state() {
    let (mut a, _) = {
        let mut server = Server::new(IfcsConfig::default());
        server.spawn(Entity::cube(1000.0, true, Some(1)));
        (server, ())
    };
    let mut b = Server::new(IfcsConfig::default());
    let id = b.spawn(Entity::cube(1000.0, true, Some(1)));

    let input = ShipInput::new(1.0, 0.3, -0.2, 0.1);
    for tick in 0..60 {
        for server in [&mut a, &mut b] {
            server
                .submit_input(ragfmsg::sim::InputMessage {
                    tick,
                    client: 1,
                    target: id,
                    input,
                })
                .unwrap();
            server.step();
        }
    }
    assert_eq!(a.world.get(id).unwrap().body, b.world.get(id).unwrap().body);
}

#[test]
fn divergence_triggers_rollback_and_correction() {
    let (mut server, mut client, id) = paired();
    let input = ShipInput::new(1.0, 0.0, 0.0, 0.0);
    let msg = client.send_input(id, input);
    client.step(input);
    server.submit_input(msg).unwrap();
    let snap = server.step();

    client.world.get_mut(id).unwrap().body.pos = Vec3::new(9.0, 9.0, 9.0);
    client.receive_snapshot(&snap);

    assert_eq!(client.rollbacks, 1);
    let authoritative = server.world.get(id).unwrap().body;
    let corrected = client.body(id).unwrap();
    assert!(corrected.pos.distance(authoritative.pos) < POS_EPS);
    assert!(corrected.ori.angle_to(authoritative.ori) < ORI_EPS);
}

#[test]
fn clean_run_has_almost_no_rollbacks() {
    let (mut server, mut client, id) = paired();
    for _ in 0..120 {
        let input = ShipInput::new(1.0, 0.1, 0.0, 0.0);
        let msg = client.send_input(id, input);
        client.step(input);
        server.submit_input(msg).unwrap();
        client.receive_snapshot(&server.step());
    }
    assert_eq!(client.rollbacks, 0);
}

#[test]
fn physics_step_is_repeatable() {
    let cfg = IfcsConfig::default();
    let input = ShipInput::new(1.0, 0.4, 0.2, -0.1);
    let start = ShipBody::new_cube(1000.0);
    let mut a = start;
    let mut b = start;
    for _ in 0..30 {
        advance(&mut a, &input, &cfg, DT);
        advance(&mut b, &input, &cfg, DT);
    }
    assert_eq!(a, b);
}
