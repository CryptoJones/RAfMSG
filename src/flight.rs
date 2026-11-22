use crate::physics::step;
use crate::types::{ForceTorque, ShipBody, ShipInput, Vec3, DT};
use nalgebra::{DMatrix, DVector};

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Thruster {
    pub position: Vec3,
    pub direction: Vec3,
    pub max_force: f64,
}

impl Thruster {
    pub fn new(position: Vec3, direction: Vec3, max_force: f64) -> Self {
        Self {
            position,
            direction: direction.normalized(),
            max_force,
        }
    }

    fn wrench(&self) -> [f64; 6] {
        let force = self.direction.scale(self.max_force);
        let torque = cross(self.position, force);
        [force.x, force.y, force.z, torque.x, torque.y, torque.z]
    }
}

#[derive(Clone, Debug, Default)]
pub struct ThrusterArray {
    pub thrusters: Vec<Thruster>,
}

impl ThrusterArray {
    pub fn new(thrusters: Vec<Thruster>) -> Self {
        Self { thrusters }
    }

    pub fn six_dof(force: f64, arm: f64) -> Self {
        Self::new(vec![
            Thruster::new(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), force),
            Thruster::new(Vec3::ZERO, Vec3::new(-1.0, 0.0, 0.0), force),
            Thruster::new(Vec3::ZERO, Vec3::new(0.0, 1.0, 0.0), force),
            Thruster::new(Vec3::ZERO, Vec3::new(0.0, -1.0, 0.0), force),
            Thruster::new(Vec3::ZERO, Vec3::new(0.0, 0.0, 1.0), force),
            Thruster::new(Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0), force),
            Thruster::new(Vec3::new(arm, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), force),
            Thruster::new(Vec3::new(arm, 0.0, 0.0), Vec3::new(0.0, -1.0, 0.0), force),
            Thruster::new(Vec3::new(0.0, arm, 0.0), Vec3::new(1.0, 0.0, 0.0), force),
            Thruster::new(Vec3::new(0.0, arm, 0.0), Vec3::new(-1.0, 0.0, 0.0), force),
            Thruster::new(Vec3::new(0.0, 0.0, arm), Vec3::new(1.0, 0.0, 0.0), force),
            Thruster::new(Vec3::new(0.0, 0.0, arm), Vec3::new(-1.0, 0.0, 0.0), force),
        ])
    }

    pub fn allocate(&self, desired: ForceTorque) -> Vec<f64> {
        if self.thrusters.is_empty() {
            return Vec::new();
        }
        let columns = self.thrusters.len();
        let matrix = DMatrix::from_fn(6, columns, |row, col| self.thrusters[col].wrench()[row]);
        let target = DVector::from_vec(vec![
            desired.force.x,
            desired.force.y,
            desired.force.z,
            desired.torque.x,
            desired.torque.y,
            desired.torque.z,
        ]);
        match matrix.clone().pseudo_inverse(1e-9) {
            Ok(pinv) => {
                let throttles = pinv * target;
                throttles
                    .iter()
                    .map(|value| value.clamp(0.0, 1.0))
                    .collect()
            }
            Err(_) => vec![0.0; columns],
        }
    }

    pub fn net(&self, throttles: &[f64]) -> ForceTorque {
        let mut force = Vec3::ZERO;
        let mut torque = Vec3::ZERO;
        for (thruster, throttle) in self.thrusters.iter().zip(throttles) {
            let thrust = thruster.direction.scale(*throttle * thruster.max_force);
            force = force + thrust;
            torque = torque + cross(thruster.position, thrust);
        }
        ForceTorque { force, torque }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlightTuning {
    pub thrust_n: f64,
    pub torque_nm: f64,
    pub g_limit_ms2: f64,
    pub linear_damping: f64,
    pub ang_damping: f64,
    pub stabilization_gain: f64,
}

impl Default for FlightTuning {
    fn default() -> Self {
        Self {
            thrust_n: 1000.0,
            torque_nm: 100.0,
            g_limit_ms2: 1000.0,
            linear_damping: 0.0,
            ang_damping: 0.0,
            stabilization_gain: 0.0,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct FlightController {
    pub tuning: FlightTuning,
    pub thrusters: ThrusterArray,
}

impl FlightController {
    pub fn new(tuning: FlightTuning, thrusters: ThrusterArray) -> Self {
        Self { tuning, thrusters }
    }

    pub fn control_law(&self, input: &ShipInput, body: &ShipBody) -> ForceTorque {
        let throttle = input.throttle.clamp(-1.0, 1.0);
        let local = Vec3::new(0.0, 0.0, -throttle * self.tuning.thrust_n);
        let mut force = body.ori.rotate(local);
        let max_force = body.mass * self.tuning.g_limit_ms2;
        let magnitude = force.length();
        if magnitude > max_force && magnitude > 0.0 {
            force = force.scale(max_force / magnitude);
        }
        if self.tuning.linear_damping > 0.0 {
            force = force - body.vel.scale(self.tuning.linear_damping * body.mass);
        }

        let commanded = Vec3::new(
            input.roll.clamp(-1.0, 1.0),
            input.pitch.clamp(-1.0, 1.0),
            input.yaw.clamp(-1.0, 1.0),
        )
        .scale(self.tuning.torque_nm);
        let stabilized = commanded
            - body.ang_vel.scale(self.tuning.ang_damping * body.mass)
            - body
                .ang_vel
                .scale(self.tuning.stabilization_gain * body.mass);

        if self.thrusters.thrusters.is_empty() {
            ForceTorque {
                force,
                torque: stabilized,
            }
        } else {
            let desired = ForceTorque {
                force,
                torque: stabilized,
            };
            let throttles = self.thrusters.allocate(desired);
            self.thrusters.net(&throttles)
        }
    }

    pub fn step(&self, body: &mut ShipBody, input: &ShipInput) {
        let forces = self.control_law(input, body);
        step(body, &forces, DT);
    }
}

pub fn apply_to_rapier(
    world: &mut crate::physics3d::PhysicsWorld3d,
    handle: crate::physics3d::BodyHandle,
    forces: &ForceTorque,
) -> bool {
    match world.bodies.get_mut(handle) {
        Some(body) => {
            use rapier3d_f64::prelude::Vector;
            body.add_force(
                Vector::new(forces.force.x, forces.force.y, forces.force.z),
                true,
            );
            body.add_torque(
                Vector::new(forces.torque.x, forces.torque.y, forces.torque.z),
                true,
            );
            true
        }
        None => false,
    }
}
