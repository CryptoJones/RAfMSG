use ragfmsg::flight::*;
use ragfmsg::physics3d::{self, PhysicsWorld3d};
use ragfmsg::types::{ForceTorque, ShipBody, ShipInput, Vec3, DT};

fn ship() -> ShipBody {
    ShipBody::new_cube(10.0)
}

fn controller(tuning: FlightTuning) -> FlightController {
    FlightController::new(tuning, ThrusterArray::default())
}

#[test]
fn flight_zero_input_conserves_momentum() {
    let ctrl = controller(FlightTuning::default());
    let mut body = ship();
    body.vel = Vec3::new(1.0, 2.0, 3.0);
    body.ang_vel = Vec3::new(0.1, 0.2, 0.3);
    let before = body;
    ctrl.step(&mut body, &ShipInput::NONE);
    assert_eq!(body.pos, before.pos + before.vel.scale(DT));
    assert_eq!(body.vel, before.vel);
    assert_eq!(body.ang_vel, before.ang_vel);
}

#[test]
fn flight_full_throttle_is_f_over_m() {
    let tuning = FlightTuning {
        thrust_n: 1000.0,
        g_limit_ms2: 1_000_000.0,
        ..Default::default()
    };
    let ctrl = controller(tuning);
    let mut body = ship();
    ctrl.step(&mut body, &ShipInput::new(1.0, 0.0, 0.0, 0.0));
    let expected = -1000.0 / 10.0 * DT;
    assert!(
        (body.vel.z - expected).abs() < 1e-9,
        "expected {expected}, got {}",
        body.vel.z
    );
}

#[test]
fn flight_g_limit_enforcement() {
    let tuning = FlightTuning {
        thrust_n: 100_000.0,
        g_limit_ms2: 10.0,
        ..Default::default()
    };
    let ctrl = controller(tuning);
    let mut body = ship();
    ctrl.step(&mut body, &ShipInput::new(1.0, 0.0, 0.0, 0.0));
    assert!(
        body.vel.z.abs() <= 10.0 * DT + 1e-9,
        "g-limit should cap acceleration"
    );
}

#[test]
fn flight_damping_and_stabilization_reduce_spin() {
    let mut body = ship();
    body.ang_vel = Vec3::new(0.0, 2.0, 0.0);
    let plain = controller(FlightTuning::default());
    let damped = controller(FlightTuning {
        ang_damping: 0.5,
        ..Default::default()
    });
    let stabilized = controller(FlightTuning {
        stabilization_gain: 0.5,
        ..Default::default()
    });
    let input = ShipInput::NONE;
    let mut a = body;
    let mut b = body;
    let mut c = body;
    plain.step(&mut a, &input);
    damped.step(&mut b, &input);
    stabilized.step(&mut c, &input);
    assert!(b.ang_vel.y < a.ang_vel.y);
    assert!(c.ang_vel.y < a.ang_vel.y);
}

#[test]
fn flight_linear_damping_reduces_drift() {
    let mut body = ship();
    body.vel = Vec3::new(5.0, 0.0, 0.0);
    let ctrl = controller(FlightTuning {
        linear_damping: 1.0,
        ..Default::default()
    });
    let before = body.vel.x;
    ctrl.step(&mut body, &ShipInput::NONE);
    assert!(body.vel.x < before);
}

#[test]
fn flight_thruster_allocation() {
    let array = ThrusterArray::new(vec![Thruster::new(
        Vec3::ZERO,
        Vec3::new(1.0, 0.0, 0.0),
        100.0,
    )]);
    let throttles = array.allocate(ForceTorque {
        force: Vec3::new(50.0, 0.0, 0.0),
        torque: Vec3::ZERO,
    });
    assert_eq!(throttles.len(), 1);
    assert!((throttles[0] - 0.5).abs() < 1e-6);
    let net = array.net(&throttles);
    assert!((net.force.x - 50.0).abs() < 1e-6);
}

#[test]
fn flight_allocation_respects_thruster_limits() {
    let array = ThrusterArray::new(vec![Thruster::new(
        Vec3::ZERO,
        Vec3::new(1.0, 0.0, 0.0),
        100.0,
    )]);
    let throttles = array.allocate(ForceTorque {
        force: Vec3::new(1000.0, 0.0, 0.0),
        torque: Vec3::ZERO,
    });
    assert_eq!(throttles[0], 1.0);
    assert_eq!(array.net(&throttles).force.x, 100.0);
}

#[test]
fn flight_six_dof_array_generates_torque() {
    let array = ThrusterArray::six_dof(100.0, 1.0);
    let throttles = array.allocate(ForceTorque {
        force: Vec3::ZERO,
        torque: Vec3::new(0.0, 0.0, 50.0),
    });
    assert_eq!(throttles.len(), 12);
    let net = array.net(&throttles);
    assert!(
        net.torque.z > 0.0,
        "allocated thrusters should produce +Z torque"
    );
    assert!(throttles.iter().all(|value| *value >= 0.0 && *value <= 1.0));
}

#[test]
fn flight_empty_thruster_array_is_safe() {
    let array = ThrusterArray::default();
    assert!(array
        .allocate(ForceTorque {
            force: Vec3::new(1.0, 1.0, 1.0),
            torque: Vec3::ZERO,
        })
        .is_empty());
    assert_eq!(array.net(&[]), ForceTorque::ZERO);
}

#[test]
fn flight_rapier_bridge_applies_forces() {
    let mut world: PhysicsWorld3d = physics3d::default_world();
    let (body, _) = physics3d::spawn_dynamic_sphere(&mut world, Vec3::ZERO, 0.5, 10.0, false);
    assert!(apply_to_rapier(
        &mut world,
        body,
        &ForceTorque {
            force: Vec3::new(100.0, 0.0, 0.0),
            torque: Vec3::ZERO,
        }
    ));
    physics3d::step(&mut world);
    let (_, velocity) = physics3d::body_state(&world, body).unwrap();
    assert!(velocity.x > 0.0, "IFCS force should drive the Rapier body");
}
