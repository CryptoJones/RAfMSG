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

/// Client-side IFCS contract recovered from the Ghidra 12.1.4 decompile of
/// StarCitizen.exe on pluto. Bodies are the smallest implementation consistent
/// with the recovered pseudocode, not a reference implementation; each item
/// cites its `FUN_<address>` source. See `spec/ifcs_ghidra_contract.rs` for the
/// tests that pin these.
pub mod client_contract {
    /// Quaternion (x, y, z, w).
    #[derive(Clone, Copy, Debug)]
    pub struct Quat {
        pub x: f32,
        pub y: f32,
        pub z: f32,
        pub w: f32,
    }

    const FLT_EPSILON: f32 = 1.1920929e-07;

    /// Rotate `v` by `q` — the expansion at `FUN_1447ed270` lines 119-132.
    fn quat_rotate(q: Quat, v: [f32; 3]) -> [f32; 3] {
        let (x, y, z) = (v[0], v[1], v[2]);
        let t0 = (z * q.y - y * q.z) + x * q.w;
        let t1 = (x * q.z - z * q.x) + y * q.w;
        let t2 = (y * q.x - x * q.y) + z * q.w;
        let o0 = t2 * q.y - t1 * q.z;
        let o1 = t0 * q.z - t2 * q.x;
        let o2 = t1 * q.x - t0 * q.y;
        [x + o0 + o0, y + o1 + o1, z + o2 + o2]
    }

    /// `FUN_1447ad820` lines 159-176: `expf(dt * -2)` smoother. The decay
    /// factor is `expf(dt * -2)`; the blend toward `measured` is `1 - decay`,
    /// so `dt = 0` holds `state`.
    pub fn smooth_exp2(state: f32, measured: f32, dt: f32) -> f32 {
        let decay = (-2.0 * dt).exp();
        (1.0 - decay) * (measured - state) + state
    }

    /// `FUN_1447ad820` lines 163-176: length ratio clamped to `[0, 1]`, or 0
    /// when the reference is at or below FLT_EPSILON.
    pub fn speed_fraction(value: f32, reference: f32) -> f32 {
        if reference <= FLT_EPSILON {
            return 0.0;
        }
        (value / reference).clamp(0.0, 1.0)
    }

    /// `FUN_1447ad820` lines 452-458: curve-off arm `norm * 0.3 + 0.7` clamped
    /// to `[0.7, 1]`.
    pub fn fallback_proximity_scale(norm: f32) -> f32 {
        (norm * 0.3 + 0.7).clamp(0.7, 1.0)
    }

    /// `FUN_1447e3d90` / `FUN_1447ed270` lines 48-58: a gate at or below
    /// FLT_EPSILON forces the published command to zero; otherwise each
    /// component clamps to `[-1, 1]`.
    pub fn clamp_signed_command(gate: f32, v: [f32; 3]) -> [f32; 3] {
        if gate <= FLT_EPSILON {
            return [0.0; 3];
        }
        [
            v[0].clamp(-1.0, 1.0),
            v[1].clamp(-1.0, 1.0),
            v[2].clamp(-1.0, 1.0),
        ]
    }

    /// `FUN_1447705d0` lines 47-107: gravity compensation is the gravity vector
    /// rotated by the conjugate quaternion times `(1 - blend)`; aero adds the
    /// aero vector directly. Disabled terms contribute nothing.
    pub fn compensate(
        gravity: [f32; 3],
        aero: [f32; 3],
        q: Quat,
        blend: f32,
        gravity_enabled: bool,
        aero_enabled: bool,
    ) -> [f32; 3] {
        let conj = Quat {
            x: -q.x,
            y: -q.y,
            z: -q.z,
            w: q.w,
        };
        let scale = 1.0 - blend;
        let g = if gravity_enabled {
            quat_rotate(conj, gravity)
        } else {
            [0.0; 3]
        };
        let a = if aero_enabled { aero } else { [0.0; 3] };
        [
            g[0] * scale + a[0],
            g[1] * scale + a[1],
            g[2] * scale + a[2],
        ]
    }

    /// `FUN_1447ed270` lines 119-132: rotate the solver vector by the body
    /// quaternion.
    pub fn rotate_body_command(q: Quat, v: [f32; 3]) -> [f32; 3] {
        quat_rotate(q, v)
    }

    /// `FUN_143dd0b20` lines 24-55: with proportional gain 1 and integral /
    /// derivative gains 0, the sample is `clamp(signal, low, high)`.
    pub fn axis_limit_output(signal: f32, low: f32, high: f32) -> f32 {
        signal.clamp(low, high)
    }

    /// `FUN_144821090` lines 47-67 (with tick->seconds in `FUN_140640dc0` /
    /// `FUN_1406764a0`): `alpha = clamp(seconds * rate, 0, 1)`,
    /// `value = (target - start) * alpha + start`.
    pub fn slew_damping(start: f32, target: f32, dt: f32, rate: f32) -> f32 {
        let alpha = (dt * rate).clamp(0.0, 1.0);
        (target - start) * alpha + start
    }

    // --- additions from the float-bearing sweep (2026-11-22) ---

    /// `FUN_1447eef50` line 645: an axis is active when `0.001 < ABS(value)`.
    pub fn input_active(x: f32, y: f32, z: f32) -> bool {
        0.001 < x.abs() || 0.001 < y.abs() || 0.001 < z.abs()
    }

    /// `FUN_1447eef50` line 361: m/s^2 -> g via `0.101936795` (== 1 / 9.81).
    pub fn to_g_units(a: f32) -> f32 {
        a * 0.101936795
    }

    /// `FUN_1441ea570` lines 474-481: per-axis velocity match within 0.05.
    pub fn velocity_matched(a: [f32; 3], b: [f32; 3]) -> bool {
        (a[0] - b[0]).abs() <= 0.05 && (a[1] - b[1]).abs() <= 0.05 && (a[2] - b[2]).abs() <= 0.05
    }

    /// `FUN_1447eef50` line 676: normalize with the FLT_EPSILON zero-guard so a
    /// near-zero direction yields zero, not NaN.
    pub fn normalize_direction(v: [f32; 3]) -> [f32; 3] {
        let mag2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
        if mag2 <= FLT_EPSILON {
            return [0.0; 3];
        }
        let inv = 1.0 / mag2.sqrt();
        [v[0] * inv, v[1] * inv, v[2] * inv]
    }

    /// `FUN_1447e3d90` line 259: g-safety engages when `limit * 9.81 < speed`.
    pub fn g_safety_engaged(speed: f32, limit_g: f32) -> bool {
        limit_g * 9.81 < speed
    }

    /// `FUN_1441dc3e0` line 1042 (`vminss` with `0x447a0000` == 1000.0f): the
    /// gravlev antifall correction is one-sided clamped to at most 1000.0.
    pub fn antifall_clamp(v: f32) -> f32 {
        v.min(1000.0)
    }

    /// rad -> deg. The pi / 180.0 / 0.31830987 (1/pi) immediates recur in the
    /// control path (`FUN_145ae2620`, `FUN_1447846c0`, `FUN_144b2d680`).
    pub fn rad_to_deg(r: f32) -> f32 {
        r * (180.0 / core::f32::consts::PI)
    }

    /// deg -> rad.
    pub fn deg_to_rad(d: f32) -> f32 {
        d * (core::f32::consts::PI / 180.0)
    }
}
