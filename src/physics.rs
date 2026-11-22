use crate::ifcs::{control_law, IfcsConfig};
use crate::types::{ForceTorque, ShipBody, ShipInput};

pub fn step(body: &mut ShipBody, ft: &ForceTorque, dt: f64) {
    let inv_mass = body.inv_mass();

    let accel = ft.force.scale(inv_mass);
    body.vel = body.vel + accel.scale(dt);
    body.pos = body.pos + body.vel.scale(dt);

    let ang_accel = ft.torque.scale(inv_mass);
    body.ang_vel = body.ang_vel + ang_accel.scale(dt);
    body.ori = body.ori.integrate(body.ang_vel, dt);
}

pub fn advance(body: &mut ShipBody, input: &ShipInput, cfg: &IfcsConfig, dt: f64) {
    let ft = control_law(input, body, cfg);
    step(body, &ft, dt);
}
