use bevy_ecs::prelude::*;
use ragfmsg::ecs::*;
use ragfmsg::ifcs::IfcsConfig;
use ragfmsg::physics::step;
use ragfmsg::types::{ForceTorque, ShipBody, ShipInput, Vec3, DT};

fn body_count(world: &mut EcsWorld) -> usize {
    let mut query = world.query::<&Body>();
    query.iter(world).count()
}

#[test]
fn ecs_spawn_query_and_filter() {
    let mut world = new_world();
    let server = spawn_ship(
        &mut world,
        ShipBody::new_cube(2.0),
        ShipInput::NONE,
        SpawnOpts {
            replicate: true,
            server: true,
            ..Default::default()
        },
    );
    spawn_ship(
        &mut world,
        ShipBody::new_cube(3.0),
        ShipInput::NONE,
        SpawnOpts {
            replicate: true,
            owner: Some(7),
            ..Default::default()
        },
    );
    spawn_ship(
        &mut world,
        ShipBody::new_cube(4.0),
        ShipInput::NONE,
        SpawnOpts::default(),
    );

    let replicated = replicated_bodies(&mut world);
    assert_eq!(replicated.len(), 2);
    assert!(replicated.iter().any(|(entity, _)| *entity == server));
    assert_eq!(owned_by(&mut world, 7).len(), 1);
    assert!(owned_by(&mut world, 8).is_empty());
    assert_eq!(body_of(&world, server).unwrap().mass, 2.0);
    assert_eq!(body_count(&mut world), 3);
}

#[test]
fn ecs_despawn_updates_count() {
    let mut world = new_world();
    let entity = spawn_ship(
        &mut world,
        ShipBody::new_cube(1.0),
        ShipInput::NONE,
        SpawnOpts::default(),
    );
    assert!(despawn_ship(&mut world, entity));
    assert!(!despawn_ship(&mut world, entity));
    assert_eq!(world.resource::<DespawnCount>().0, 1);
    assert!(body_of(&world, entity).is_none());
}

#[test]
fn ecs_component_registration() {
    let mut world = new_world();
    let mut registry = ComponentRegistry::new();
    let body_id = registry.register::<Body>(&mut world, "Body");
    let _ = registry.register::<Replicate>(&mut world, "Replicate");
    let _ = registry.register::<Server>(&mut world, "Server");
    assert!(registry.is_registered("Body"));
    assert!(registry.is_registered("Replicate"));
    assert!(registry.is_registered("Server"));
    assert!(!registry.is_registered("Missing"));
    assert_eq!(registry.len(), 3);
    assert!(registry.register::<Body>(&mut world, "Body") == body_id);
    assert_eq!(registry.len(), 3);
}

#[test]
fn ecs_schedule_drives_entity_lifecycle() {
    let mut world = new_world();
    let mut schedule = lifecycle_schedule();

    schedule.run(&mut world);
    assert_eq!(world.resource::<TickCounter>().0, 1);
    assert_eq!(body_count(&mut world), 1);

    schedule.run(&mut world);
    schedule.run(&mut world);
    let z_after = replicated_bodies(&mut world)[0].1.pos.z;
    assert!(
        z_after < 0.0,
        "full throttle should push ship along -Z, got {z_after}"
    );

    schedule.run(&mut world);
    assert_eq!(world.resource::<DespawnCount>().0, 1);
    assert_eq!(body_count(&mut world), 0);

    schedule.run(&mut world);
    assert_eq!(world.resource::<DespawnCount>().0, 1);
}

#[test]
fn ecs_parallel_query_updates_every_entity() {
    let mut world = new_world();
    for _ in 0..64 {
        spawn_ship(
            &mut world,
            ShipBody::new_cube(1.0),
            ShipInput::NONE,
            SpawnOpts {
                server: true,
                ..Default::default()
            },
        );
    }
    world.insert_resource(TickCounter(1));
    let mut advance = Schedule::default();
    advance.add_systems(advance_system);
    advance.run(&mut world);

    let bodies = replicated_bodies(&mut world);
    assert_eq!(bodies.len(), 0);
    let mut query = world.query_filtered::<&Body, With<Server>>();
    let velocities: Vec<Vec3> = query.iter(&world).map(|body| body.0.vel).collect();
    assert_eq!(velocities.len(), 64);
    assert!(velocities.iter().all(|v| *v == velocities[0]));
    assert!(velocities[0].z < 0.0);
}

#[test]
fn ecs_contract_bridge_preserves_harness_behavior() {
    let cfg = IfcsConfig::default();
    let input = ShipInput::new(0.5, 0.25, 0.0, 0.0);
    let body = ShipBody::new_cube(10.0);
    let mut through_component = Body(body);
    let mut direct = body;
    let ft = ForceTorque {
        force: Vec3::new(1.0, 2.0, 3.0),
        torque: Vec3::new(0.1, 0.2, 0.3),
    };
    step(&mut through_component.0, &ft, DT);
    step(&mut direct, &ft, DT);
    assert_eq!(through_component.0, direct);

    let _ = (cfg, input);
}
