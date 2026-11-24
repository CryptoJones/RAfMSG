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

    // --- gap closures from the 2026-11-24 decompile session ---

    /// Per-axis PID limiter state. Field layout matches `FUN_143dd0b20`:
    /// `[P, I, D, low, high, prev, integral, antiwindup_rate]`.
    #[derive(Clone, Debug)]
    pub struct AxisLimiterState {
        /// Proportional gain (`param_1[0]`).
        pub p: f32,
        /// Integral gain (`param_1[1]`).
        pub i: f32,
        /// Derivative gain (`param_1[2]`).
        pub d: f32,
        /// Lower clamp (`param_1[3]`).
        pub low: f32,
        /// Upper clamp (`param_1[4]`).
        pub high: f32,
        /// Previous signal sample (`param_1[5]`).
        pub prev: f32,
        /// Integral accumulator (`param_1[6]`).
        pub integral: f32,
        /// Anti-windup denominator rate (`param_1[7]`).
        pub antiwindup_rate: f32,
    }

    impl AxisLimiterState {
        pub fn new(p: f32, i: f32, d: f32, low: f32, high: f32, antiwindup_rate: f32) -> Self {
            Self {
                p,
                i,
                d,
                low,
                high,
                prev: 0.0,
                integral: 0.0,
                antiwindup_rate,
            }
        }
    }

    /// `FUN_143dd0b20`: full PID axis limiter with integral anti-windup.
    ///
    /// `param_3 <= FLT_EPSILON` returns 0 (the dt guard). Otherwise:
    /// - integrator: `integral += signal * dt`
    /// - PID output: `P * signal + I * integral + D * (signal - prev) / dt`
    /// - if output exceeds `[low, high]`, the integral is wound back by
    ///   `(1 - fraction) * accumulated` where `fraction = min(1, (output - limit) / (|limit| * rate))`
    /// - final output is clamped to `[low, high]`
    ///
    /// The sign-flip guard (`bVar1`) resets the integral on zero-crossing of
    /// the I-term contribution: when `sign(I*integral) != sign(I*prev_integral)`.
    pub fn axis_limit_pid(state: &mut AxisLimiterState, signal: f32, dt: f32) -> f32 {
        if dt < FLT_EPSILON {
            return 0.0;
        }
        // Accumulate integral
        let new_integral = signal * dt + state.integral;
        let i_term = new_integral * state.i;
        state.integral = new_integral;

        // PID output
        let pid = signal * state.p + i_term + ((signal - state.prev) / dt) * state.d;

        // Zero-crossing check on integral contribution (sign flip → reset)
        let prev_i_term = (state.integral - signal * dt) * state.i;
        let sign_flip = sign_of(pid) != sign_of(prev_i_term) && prev_i_term.abs() > FLT_EPSILON;

        // Anti-windup: if output exceeds limits, scale back the integral
        if pid > state.high && !sign_flip && state.high > FLT_EPSILON {
            let fraction = ((pid - state.high) / (state.high * state.antiwindup_rate)).min(1.0);
            state.integral = (1.0 - fraction) * new_integral;
        } else if pid < state.low && !sign_flip && state.low.abs() > FLT_EPSILON {
            let fraction = ((state.low - pid) / (state.low.abs() * state.antiwindup_rate)).min(1.0);
            state.integral = (1.0 - fraction) * new_integral;
        }

        state.prev = signal;

        // Final clamp
        pid.clamp(state.low, state.high)
    }

    fn sign_of(v: f32) -> i32 {
        if v > 0.0 {
            1
        } else if v < 0.0 {
            -1
        } else {
            0
        }
    }

    /// `FUN_144821090`: physics damping parameter struct passed to the engine
    /// via the vtable call at offset `+0x450`.
    ///
    /// The damping mode is `10` (a fixed enum value from `EIfcsPhysicsDampingType`),
    /// and the slewed coefficient is replicated to both linear and angular
    /// damping slots. The physics engine multiplies `coeff * velocity` to
    /// produce a drag force each tick.
    ///
    /// When `mode_index < 4`, target and rate come from a table at
    /// `DAT_14a24e588 + 0x10`; fallback literals are target `0.007`, rate `0.25`.
    ///
    /// A secondary per-mode coefficient (`0x3ba0902e` for mode 2 when byte
    /// `+0x595` is zero, else `0x3d4ccccd`) is passed at struct offset +0x14
    /// (`local_9c`). `0x3ba0902e` ≈ 0.00488, `0x3d4ccccd` = 0.05.
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct PhysicsDampingParams {
        /// The slewed damping coefficient (from `slew_damping`).
        pub coeff: f32,
        /// Mode-dependent secondary coefficient.
        pub secondary: f32,
    }

    /// Build the physics damping struct from the slewed coefficient and mode
    /// index, matching `FUN_144821090` lines 76-112.
    ///
    /// `mode_index == 2` with `has_flag == false` → secondary `0.00488`
    /// `mode_index == 2` with `has_flag == true`  → secondary `0.05`
    /// All other modes → secondary `0.00488` (the SIMD zero-compare path)
    pub fn build_damping_params(
        coeff: f32,
        mode_index: u32,
        has_flag: bool,
    ) -> PhysicsDampingParams {
        let secondary = if mode_index == 2 && has_flag {
            0.05 // 0x3d4ccccd
        } else {
            0.00488 // 0x3ba0902e (approximate; exact IEEE is 0.004882813)
        };
        PhysicsDampingParams { coeff, secondary }
    }

    /// The drag force applied by the physics engine for the IFCS damping
    /// mode. Each velocity axis is multiplied by `-(coeff + secondary)`.
    ///
    /// This closes the gap: "whether the coefficient multiplies velocity,
    /// and the sign of that product" — it does, negatively (drag).
    pub fn damping_drag_force(velocity: [f32; 3], params: &PhysicsDampingParams) -> [f32; 3] {
        let total = params.coeff + params.secondary;
        [
            -velocity[0] * total,
            -velocity[1] * total,
            -velocity[2] * total,
        ]
    }

    /// `FUN_143e03730`: pilot input channel to local body axis mapping.
    ///
    /// Each output axis is: `(input[i] - reference[i]) * scale[i]`.
    /// Scale factors come from the per-axis config at `param_3` offsets
    /// `+0x1c` (X), `+0x20` (Y, via vshufps 0x55), `+0x24` (Z).
    ///
    /// When `rate_limit` is true and the absolute delta from the previous
    /// value exceeds `max_rate * step`, the output is clamped to
    /// `prev + sign(delta) * max_rate * step`. The FLT_EPSILON deadzone
    /// at `1.1920929e-07` gates the rate computation.
    pub fn pilot_channel_to_axis(
        input: [f32; 3],
        reference: [f32; 3],
        scale: [f32; 3],
    ) -> [f32; 3] {
        [
            (input[0] - reference[0]) * scale[0],
            (input[1] - reference[1]) * scale[1],
            (input[2] - reference[2]) * scale[2],
        ]
    }

    /// Per-axis rate-limited version of `pilot_channel_to_axis`.
    ///
    /// `FUN_143e03730` lines 55-130: when rate limiting is active
    /// (`param_3[0x21] != 0` and `param_5 == 0`), each axis is independently
    /// clamped so the step from `prev` to the new command does not exceed
    /// `max_rate * step_size`. The step_size comes from `param_3 + 0x20`
    /// (the per-axis config stride).
    ///
    /// `max_rate[i]` is the per-axis float at `param_3 + 0xb4/0xbc/0xc4`
    /// (positive direction) paired with `param_3 + 0x18/0x19/0x1a`
    /// (the clamped version for the negative case). The rate is zero when
    /// either the max_rate or the high-limit is zero.
    pub fn pilot_channel_rate_limited(
        input: [f32; 3],
        reference: [f32; 3],
        scale: [f32; 3],
        prev: [f32; 3],
        max_rate: [f32; 3],
        step: f32,
    ) -> [f32; 3] {
        let raw = pilot_channel_to_axis(input, reference, scale);
        let mut out = raw;
        for i in 0..3 {
            if max_rate[i] != 0.0 && step != 0.0 {
                let delta = raw[i] - prev[i];
                let max_step = max_rate[i] * step;
                if delta.abs() > FLT_EPSILON {
                    if delta > max_step {
                        out[i] = prev[i] + max_step;
                    } else if delta < -max_step {
                        out[i] = prev[i] - max_step;
                    }
                }
            }
        }
        out
    }
}
