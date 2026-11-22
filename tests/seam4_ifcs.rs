use ragfmsg::ifcs::{control_law, IfcsConfig};
use ragfmsg::physics::step;
use ragfmsg::types::{ShipBody, ShipInput, Vec3, DT};

#[test]
fn control_law_matches_reference_values() {
    let cfg = IfcsConfig::default();
    let body = ShipBody::new_cube(1000.0);
    let ft = control_law(&ShipInput::new(1.0, 0.5, 0.0, 0.0), &body, &cfg);
    assert_eq!(ft.force, Vec3::new(0.0, 0.0, -1000.0));
    assert_eq!(ft.torque, Vec3::new(0.0, 0.0, 50.0));
}

#[test]
fn zero_input_conserves_momentum() {
    let cfg = IfcsConfig::default();
    let mut body = ShipBody::new_cube(1000.0);
    body.vel = Vec3::new(1.0, 2.0, 3.0);
    for _ in 0..60 {
        let ft = control_law(&ShipInput::NONE, &body, &cfg);
        step(&mut body, &ft, DT);
    }
    assert_eq!(body.vel, Vec3::new(1.0, 2.0, 3.0));
    assert_eq!(body.ori, ragfmsg::types::Quat::IDENTITY);
    assert_eq!(body.ang_vel, Vec3::ZERO);
    let expected = Vec3::new(1.0, 2.0, 3.0);
    assert!(body.pos.distance(expected) < 1e-12);
}

#[test]
fn full_throttle_accelerates_by_f_over_m() {
    let cfg = IfcsConfig::default();
    let mut body = ShipBody::new_cube(1000.0);
    let ft = control_law(&ShipInput::new(1.0, 0.0, 0.0, 0.0), &body, &cfg);
    step(&mut body, &ft, DT);
    assert!((body.vel.z + DT).abs() < 1e-12);
}

#[test]
fn g_limit_clamps_thrust() {
    let cfg = IfcsConfig {
        g_limit_ms2: 0.5,
        ..IfcsConfig::default()
    };
    let body = ShipBody::new_cube(1000.0);
    let ft = control_law(&ShipInput::new(1.0, 0.0, 0.0, 0.0), &body, &cfg);
    assert!((ft.force.length() - 500.0).abs() < 1e-9);
}

#[test]
fn angular_damping_removes_spin() {
    let cfg = IfcsConfig {
        ang_damping: 2.0,
        ..IfcsConfig::default()
    };
    let mut body = ShipBody::new_cube(1000.0);
    body.ang_vel = Vec3::new(0.0, 1.0, 0.0);
    let ft = control_law(&ShipInput::NONE, &body, &cfg);
    assert!((ft.torque.y + 2000.0).abs() < 1e-9);
}
