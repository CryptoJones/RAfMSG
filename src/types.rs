use std::ops::{Add, Mul, Sub};

pub type Tick = u64;
pub type EntityId = u64;
pub type ClientId = u64;

pub const TICK_HZ: u64 = 60;
pub const DT: f64 = 1.0 / TICK_HZ as f64;
pub const POS_EPS: f64 = 0.01;
pub const ORI_EPS: f64 = 0.001;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn scale(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }

    pub fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    pub fn length(self) -> f64 {
        self.dot(self).sqrt()
    }

    pub fn distance(self, rhs: Self) -> f64 {
        (self - rhs).length()
    }

    pub fn normalized(self) -> Self {
        let len = self.length();
        if len > 0.0 {
            self.scale(1.0 / len)
        } else {
            Self::ZERO
        }
    }
}

impl Add for Vec3 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Sub for Vec3 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Quat {
    pub const IDENTITY: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };

    pub const fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        Self { x, y, z, w }
    }

    pub fn from_axis_angle(axis: Vec3, angle: f64) -> Self {
        let half = angle * 0.5;
        let s = half.sin();
        let a = axis.normalized();
        Self::new(a.x * s, a.y * s, a.z * s, half.cos()).normalize()
    }

    pub fn scale(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s, self.w * s)
    }

    pub fn conjugate(self) -> Self {
        Self::new(-self.x, -self.y, -self.z, self.w)
    }

    pub fn normalize(self) -> Self {
        let n = (self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w).sqrt();
        if n > 0.0 {
            self.scale(1.0 / n)
        } else {
            Self::IDENTITY
        }
    }

    pub fn rotate(self, v: Vec3) -> Vec3 {
        let qv = Self::new(v.x, v.y, v.z, 0.0);
        let r = self * qv * self.conjugate();
        Vec3::new(r.x, r.y, r.z)
    }

    pub fn integrate(self, ang_vel: Vec3, dt: f64) -> Self {
        let omega = Self::new(ang_vel.x, ang_vel.y, ang_vel.z, 0.0);
        let delta = (omega * self).scale(0.5 * dt);
        (self + delta).normalize()
    }

    pub fn angle_to(self, other: Self) -> f64 {
        let d = (self.x * other.x + self.y * other.y + self.z * other.z + self.w * other.w)
            .abs()
            .min(1.0);
        2.0 * d.acos()
    }
}

impl Add for Quat {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self::new(
            self.x + rhs.x,
            self.y + rhs.y,
            self.z + rhs.z,
            self.w + rhs.w,
        )
    }
}

impl Mul for Quat {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self {
        Self::new(
            self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
            self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
        )
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ShipInput {
    pub throttle: f64,
    pub yaw: f64,
    pub pitch: f64,
    pub roll: f64,
}

impl ShipInput {
    pub const NONE: Self = Self {
        throttle: 0.0,
        yaw: 0.0,
        pitch: 0.0,
        roll: 0.0,
    };

    pub const fn new(throttle: f64, yaw: f64, pitch: f64, roll: f64) -> Self {
        Self {
            throttle,
            yaw,
            pitch,
            roll,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ShipBody {
    pub pos: Vec3,
    pub vel: Vec3,
    pub ori: Quat,
    pub ang_vel: Vec3,
    pub mass: f64,
}

impl ShipBody {
    pub fn new_cube(mass: f64) -> Self {
        Self {
            pos: Vec3::ZERO,
            vel: Vec3::ZERO,
            ori: Quat::IDENTITY,
            ang_vel: Vec3::ZERO,
            mass,
        }
    }

    pub fn inv_mass(self) -> f64 {
        if self.mass > 0.0 {
            1.0 / self.mass
        } else {
            0.0
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ForceTorque {
    pub force: Vec3,
    pub torque: Vec3,
}

impl ForceTorque {
    pub const ZERO: Self = Self {
        force: Vec3::ZERO,
        torque: Vec3::ZERO,
    };
}
