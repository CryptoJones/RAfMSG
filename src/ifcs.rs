use crate::types::{ForceTorque, ShipBody, ShipInput, Vec3};

#[derive(Clone, Copy, Debug)]
pub struct IfcsConfig {
    pub thrust_n: f64,
    pub torque_nm: f64,
    pub g_limit_ms2: f64,
    pub ang_damping: f64,
}

impl Default for IfcsConfig {
    fn default() -> Self {
        Self {
            thrust_n: 1000.0,
            torque_nm: 100.0,
            g_limit_ms2: 1000.0,
            ang_damping: 0.0,
        }
    }
}

pub fn control_law(input: &ShipInput, body: &ShipBody, cfg: &IfcsConfig) -> ForceTorque {
    let throttle = input.throttle.clamp(-1.0, 1.0);
    let local_force = Vec3::new(0.0, 0.0, -throttle * cfg.thrust_n);
    let mut force = body.ori.rotate(local_force);

    let max_force = body.mass * cfg.g_limit_ms2;
    let magnitude = force.length();
    if magnitude > max_force && magnitude > 0.0 {
        force = force.scale(max_force / magnitude);
    }

    let mut torque = Vec3::new(
        input.roll.clamp(-1.0, 1.0) * cfg.torque_nm,
        input.pitch.clamp(-1.0, 1.0) * cfg.torque_nm,
        input.yaw.clamp(-1.0, 1.0) * cfg.torque_nm,
    );
    if cfg.ang_damping > 0.0 {
        torque = torque - body.ang_vel.scale(cfg.ang_damping * body.mass);
    }

    ForceTorque { force, torque }
}
