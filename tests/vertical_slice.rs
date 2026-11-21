use ragfmsg::sim::{Client, Server};
use ragfmsg::types::{ShipInput, POS_EPS};
use ragfmsg::world::{snapshot, Entity};

#[test]
fn vertical_slice_input_ifcs_physics_replicate_predict_reconcile() {
    let mut server = Server::new(Default::default());
    let id = server.spawn(Entity::cube(1000.0, true, Some(1)));
    let initial = snapshot(&server.world, server.tick);

    let mut client = Client::new(1, Default::default());
    client.receive_snapshot(&initial);
    client.predict(id);

    for tick in 0..120 {
        let input = if tick < 60 {
            ShipInput::new(1.0, 0.2, 0.0, 0.0)
        } else {
            ShipInput::NONE
        };
        let msg = client.send_input(id, input);
        client.step(input);
        server.submit_input(msg).unwrap();
        client.receive_snapshot(&server.step());
    }

    let authoritative = server.world.get(id).unwrap().body;
    let predicted = client.body(id).unwrap();
    assert!(authoritative.pos.length() > 1.0);
    assert!(predicted.pos.distance(authoritative.pos) < POS_EPS);
    assert!(predicted.ori.angle_to(authoritative.ori) < 0.001);
    assert_eq!(client.rollbacks, 0);
}
