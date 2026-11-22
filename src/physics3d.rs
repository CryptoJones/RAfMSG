use crate::types::Vec3;
use rapier3d_f64::math::Pose;
use rapier3d_f64::parry::query::details::ShapeCastOptions;
use rapier3d_f64::prelude::*;
use std::sync::mpsc;

pub type PhysicsWorld3d = PhysicsWorld;
pub type BodyHandle = RigidBodyHandle;
pub type ColliderHandle3d = ColliderHandle;

pub const PHYSICS_DT: f64 = 1.0 / 60.0;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CollisionReport {
    pub started: usize,
    pub stopped: usize,
}

pub fn new_world(gravity: Vec3, dt: f64) -> PhysicsWorld3d {
    let mut world = PhysicsWorld3d::new();
    world.gravity = to_vector(gravity);
    world.integration_parameters.dt = dt;
    world
}

pub fn default_world() -> PhysicsWorld3d {
    new_world(Vec3::ZERO, PHYSICS_DT)
}

fn to_vector(v: Vec3) -> Vector {
    Vector::new(v.x, v.y, v.z)
}

fn to_vec3(v: Vector) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}

pub fn spawn_dynamic_sphere(
    world: &mut PhysicsWorld3d,
    pos: Vec3,
    radius: f64,
    mass: f64,
    report_contacts: bool,
) -> (BodyHandle, ColliderHandle3d) {
    let mut collider = ColliderBuilder::ball(radius);
    if report_contacts {
        collider = collider.active_events(ActiveEvents::COLLISION_EVENTS);
    }
    world.insert(
        RigidBodyBuilder::dynamic()
            .translation(to_vector(pos))
            .additional_mass(mass),
        collider,
    )
}

pub fn spawn_fixed_cuboid(
    world: &mut PhysicsWorld3d,
    center: Vec3,
    half_extents: Vec3,
    report_contacts: bool,
) -> (BodyHandle, ColliderHandle3d) {
    let mut collider = ColliderBuilder::cuboid(half_extents.x, half_extents.y, half_extents.z);
    if report_contacts {
        collider = collider.active_events(ActiveEvents::COLLISION_EVENTS);
    }
    world.insert(
        RigidBodyBuilder::fixed().translation(to_vector(center)),
        collider,
    )
}

pub fn apply_force(world: &mut PhysicsWorld3d, handle: BodyHandle, force: Vec3) -> bool {
    match world.bodies.get_mut(handle) {
        Some(body) => {
            body.add_force(to_vector(force), true);
            true
        }
        None => false,
    }
}

pub fn body_state(world: &PhysicsWorld3d, handle: BodyHandle) -> Option<(Vec3, Vec3)> {
    world
        .bodies
        .get(handle)
        .map(|body| (to_vec3(body.translation()), to_vec3(body.linvel())))
}

pub fn step(world: &mut PhysicsWorld3d) {
    world.step();
}

pub fn step_with_collisions(world: &mut PhysicsWorld3d) -> CollisionReport {
    let (collision_tx, collision_rx) = mpsc::channel();
    let (force_tx, _force_rx) = mpsc::channel();
    let (tear_tx, _tear_rx) = mpsc::channel();
    let collector = ChannelEventCollector::new(collision_tx, force_tx, tear_tx);
    world.step_with_events(&(), &collector);

    let mut report = CollisionReport::default();
    while let Ok(event) = collision_rx.try_recv() {
        match event {
            CollisionEvent::Started(..) => report.started += 1,
            CollisionEvent::Stopped(..) => report.stopped += 1,
        }
    }
    report
}

pub fn raycast(
    world: &mut PhysicsWorld3d,
    origin: Vec3,
    direction: Vec3,
    max_toi: f64,
) -> Option<(ColliderHandle3d, f64)> {
    world.detect_collisions(&(), &());
    let ray = Ray::new(to_vector(origin), to_vector(direction).normalize());
    world.cast_ray(&ray, max_toi, true, QueryFilter::default())
}

pub fn sweep_sphere(
    world: &mut PhysicsWorld3d,
    origin: Vec3,
    velocity: Vec3,
    radius: f64,
    max_toi: f64,
) -> Option<(ColliderHandle3d, f64)> {
    world.detect_collisions(&(), &());
    let shape = Ball::new(radius);
    let pose = Pose::translation(origin.x, origin.y, origin.z);
    let options = ShapeCastOptions {
        max_time_of_impact: max_toi,
        ..Default::default()
    };
    world
        .cast_shape(
            &pose,
            to_vector(velocity),
            &shape,
            options,
            QueryFilter::default(),
        )
        .map(|(handle, hit)| (handle, hit.time_of_impact))
}

pub fn overlap_sphere(
    world: &mut PhysicsWorld3d,
    center: Vec3,
    radius: f64,
) -> Vec<ColliderHandle3d> {
    world.detect_collisions(&(), &());
    let shape = Ball::new(radius);
    let pose = Pose::translation(center.x, center.y, center.z);
    world
        .intersect_shape(pose, &shape, QueryFilter::default())
        .map(|(handle, _)| handle)
        .collect()
}

pub fn snapshot(world: &PhysicsWorld3d) -> Vec<(BodyHandle, Vec3, Vec3)> {
    let mut states: Vec<(BodyHandle, Vec3, Vec3)> = world
        .bodies
        .iter()
        .map(|(handle, body)| (handle, to_vec3(body.translation()), to_vec3(body.linvel())))
        .collect();
    states.sort_by_key(|(handle, _, _)| handle.0);
    states
}
