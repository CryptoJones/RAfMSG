use ragfmsg::physics3d::*;
use ragfmsg::types::Vec3;
use rapier3d_f64::prelude::Vector;

fn cube_world() -> PhysicsWorld3d {
    new_world(Vec3::new(0.0, -9.81, 0.0), PHYSICS_DT)
}

#[test]
fn physics_world_and_body_api() {
    let mut world = cube_world();
    let (body, collider) =
        spawn_dynamic_sphere(&mut world, Vec3::new(0.0, 5.0, 0.0), 0.5, 2.0, false);
    let (pos, vel) = body_state(&world, body).unwrap();
    assert_eq!(pos, Vec3::new(0.0, 5.0, 0.0));
    assert_eq!(vel, Vec3::ZERO);
    assert!(world.colliders.contains(collider));

    for _ in 0..10 {
        step(&mut world);
    }
    let (pos_after, vel_after) = body_state(&world, body).unwrap();
    assert!(pos_after.y < pos.y, "gravity should pull the sphere down");
    assert!(vel_after.y < 0.0);

    assert!(apply_force(&mut world, body, Vec3::new(10.0, 0.0, 0.0)));
    let (stale, _) = spawn_dynamic_sphere(&mut world, Vec3::ZERO, 0.2, 1.0, false);
    world.remove_body(stale);
    assert!(!apply_force(&mut world, stale, Vec3::ZERO));
}

#[test]
fn physics_collision_callbacks() {
    let mut world = new_world(Vec3::new(0.0, -9.81, 0.0), PHYSICS_DT);
    spawn_fixed_cuboid(&mut world, Vec3::ZERO, Vec3::new(10.0, 0.1, 10.0), true);
    spawn_dynamic_sphere(&mut world, Vec3::new(0.0, 3.0, 0.0), 0.5, 1.0, true);

    let mut started = 0;
    for _ in 0..120 {
        started += step_with_collisions(&mut world).started;
        if started > 0 {
            break;
        }
    }
    assert!(
        started > 0,
        "falling sphere should generate a Started collision event"
    );
}

#[test]
fn physics_raycast_hit() {
    let mut world = new_world(Vec3::ZERO, PHYSICS_DT);
    spawn_fixed_cuboid(&mut world, Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0), false);
    let hit = raycast(
        &mut world,
        Vec3::new(0.0, 0.0, 5.0),
        Vec3::new(0.0, 0.0, -1.0),
        10.0,
    );
    let (_, toi) = hit.expect("ray should hit the cuboid");
    assert!((toi - 4.0).abs() < 1e-6, "expected toi 4.0, got {toi}");
    assert!(raycast(
        &mut world,
        Vec3::new(0.0, 0.0, 5.0),
        Vec3::new(0.0, 0.0, 1.0),
        10.0,
    )
    .is_none());
}

#[test]
fn physics_shape_cast_hit() {
    let mut world = new_world(Vec3::ZERO, PHYSICS_DT);
    spawn_fixed_cuboid(&mut world, Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0), false);
    let hit = sweep_sphere(
        &mut world,
        Vec3::new(0.0, 0.0, 5.0),
        Vec3::new(0.0, 0.0, -10.0),
        0.5,
        1.0,
    );
    let (_, toi) = hit.expect("swept sphere should hit the cuboid");
    assert!((toi - 0.35).abs() < 1e-3, "expected toi ~0.35, got {toi}");
}

#[test]
fn physics_overlap_test() {
    let mut world = new_world(Vec3::ZERO, PHYSICS_DT);
    let (_, ball) = spawn_dynamic_sphere(&mut world, Vec3::new(2.0, 0.0, 0.0), 0.5, 1.0, false);
    spawn_fixed_cuboid(
        &mut world,
        Vec3::new(10.0, 0.0, 0.0),
        Vec3::new(1.0, 1.0, 1.0),
        false,
    );

    let hits = overlap_sphere(&mut world, Vec3::new(2.0, 0.0, 0.0), 0.6);
    assert_eq!(hits.len(), 1);
    assert_eq!(hits[0], ball);
    assert!(overlap_sphere(&mut world, Vec3::new(6.0, 0.0, 0.0), 0.5).is_empty());
}

#[test]
fn physics_determinism_bit_exact() {
    let mut a = cube_world();
    let mut b = cube_world();
    for world in [&mut a, &mut b] {
        let (body, _) = spawn_dynamic_sphere(world, Vec3::new(1.0, 4.0, -2.0), 0.5, 3.0, false);
        apply_force(world, body, Vec3::new(5.0, 0.0, -3.0));
        spawn_fixed_cuboid(
            world,
            Vec3::new(0.0, -1.0, 0.0),
            Vec3::new(8.0, 0.2, 8.0),
            false,
        );
    }
    let before = snapshot(&a);
    assert_eq!(before, snapshot(&b));
    for _ in 0..200 {
        step(&mut a);
        step(&mut b);
    }
    let after_a = snapshot(&a);
    let after_b = snapshot(&b);
    assert_eq!(
        after_a, after_b,
        "identical inputs must give bit-exact states"
    );
    assert_ne!(before, after_a, "the world should actually have advanced");
}

#[test]
fn physics_momentum_conserved_without_forces() {
    let mut world = new_world(Vec3::ZERO, PHYSICS_DT);
    let (body, _) = spawn_dynamic_sphere(&mut world, Vec3::ZERO, 0.5, 1.0, false);
    world
        .bodies
        .get_mut(body)
        .unwrap()
        .set_linvel(Vector::new(1.0, 2.0, 3.0), true);
    let initial = body_state(&world, body).unwrap().1;
    for _ in 0..300 {
        step(&mut world);
    }
    let final_vel = body_state(&world, body).unwrap().1;
    assert_eq!(
        initial, final_vel,
        "no forces in space means constant velocity"
    );
}
