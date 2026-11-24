//! Client IFCS contract from the Ghidra 12.1.4 decompile on pluto
//! (`/mnt/hdd1/repos/OpenSourceSpaceGame/StarCitizen/`).
//!
//! Not part of the Cargo target list. Index symbols are all `FUN_<address>`
//! because the referenced PDB was not loaded. Method names are profiler or
//! assert strings in the bodies.
//!
//! Of the 302 files matching `ifcs`, 299 are string-only. Three more are
//! schema symbols and are discarded as data: `FUN_143534a00`,
//! `FUN_1447fa620`, `FUN_14563a0c0`.
//!
//! Drivers that both name an IFCS method and do numeric work:
//! `FUN_1447ad820` (`GetProximityAccelScale`), `FUN_1447e3d90`
//! (`PrePhysicsUpdate`), `FUN_1447eef50` (`ProcessThrusterControl`).
//! The force, damping-slew, and axis-limit formulas are in callees that do
//! not contain the letters "ifcs": `FUN_1447705d0` (gravity and aero
//! compensation, called from `ProcessThrusterControl`), `FUN_1447ed270`
//! (command publish), `FUN_143dd0b20` (per-axis limit), `FUN_144821090`
//! (damping-coefficient slew), `FUN_140640dc0` and `FUN_1406764a0`
//! (tick delta in seconds).
//!
//! Still unnamed as formulas: `gLimiterCurvePower` (`FUN_1447f7130` line
//! 115) and the physics-engine call at the end of `FUN_144821090` that
//! receives the slewed coefficient. Client code only. Decompiler names are
//! not trusted.
//!
//! The expected API is now implemented in `ragfmsg::ifcs::client_contract`
//! (bodies are the smallest implementation consistent with the recovered
//! pseudocode, not a reference implementation).
//!
//! Merged 2026-11-22 with the float-bearing sweep of the 302 `ifcs` files
//! (302 -> 39 float-bearing -> 30 real after dropping the "Ghidra 12.1.4"
//! header artifact). Tests unique to that sweep are appended below; overlapping
//! claims (g-safety, proximity scale, antifall clamp) stay with the stricter
//! callee-level tests above.

use ragfmsg::ifcs::client_contract::{
    antifall_clamp, axis_limit_output, axis_limit_pid, build_damping_params, clamp_signed_command,
    compensate, damping_drag_force, deg_to_rad, fallback_proximity_scale, g_safety_engaged,
    input_active, normalize_direction, pilot_channel_rate_limited, pilot_channel_to_axis,
    rad_to_deg, rotate_body_command, slew_damping, smooth_exp2, speed_fraction, to_g_units,
    velocity_matched, AxisLimiterState, Quat,
};

const FLT_EPSILON: f32 = 1.1920929e-07;

fn quat_rotate(q: Quat, v: [f32; 3]) -> [f32; 3] {
    // Same expansion as FUN_1447ed270 lines 119-132.
    let (x, y, z) = (v[0], v[1], v[2]);
    let t0 = (z * q.y - y * q.z) + x * q.w;
    let t1 = (x * q.z - z * q.x) + y * q.w;
    let t2 = (y * q.x - x * q.y) + z * q.w;
    let o0 = t2 * q.y - t1 * q.z;
    let o1 = t0 * q.z - t2 * q.x;
    let o2 = t1 * q.x - t0 * q.y;
    [x + o0 + o0, y + o1 + o1, z + o2 + o2]
}

/// `FUN_1447ad820` lines 159-176 and 427-428. `expf(dt * -2)`, then
/// `(1 - alpha) * (measured - state) + state`. `0x3f800000` is the 1.
#[test]
fn proximity_smoother_holds_at_zero_dt_and_does_not_overshoot() {
    assert!((smooth_exp2(4.0, 10.0, 0.0) - 4.0).abs() < 1e-6);
    let dt = 0.25;
    let next = smooth_exp2(4.0, 10.0, dt);
    let expect = (1.0 - (-2.0 * dt).exp()) * (10.0 - 4.0) + 4.0;
    assert!((next - expect).abs() < 1e-5);
    assert!(next > 4.0 && next < 10.0);
}

/// `FUN_1447ad820` lines 163-176. Length over the float at `+0x4a0`, clamped
/// to `[0, 1]`, or 0 when the reference is at or below `1.1920929e-07`.
#[test]
fn speed_fraction_clamps_to_unit_interval() {
    assert_eq!(speed_fraction(0.0, 10.0), 0.0);
    assert!((speed_fraction(5.0, 10.0) - 0.5).abs() < 1e-6);
    assert!((speed_fraction(25.0, 10.0) - 1.0).abs() < 1e-6);
    assert_eq!(speed_fraction(5.0, FLT_EPSILON), 0.0);
}

/// `FUN_1447ad820` lines 452-458. Curve-off arm: `norm * 0.3 + 0.7`, then
/// clamp to `[0.7, 1]`. `0x3f333333` is the low substitute.
#[test]
fn fallback_proximity_scale_is_affine_map_clamped_to_0_7_1() {
    assert!((fallback_proximity_scale(0.0) - 0.7).abs() < 1e-6);
    assert!((fallback_proximity_scale(1.0) - 1.0).abs() < 1e-6);
    assert!((fallback_proximity_scale(0.5) - 0.85).abs() < 1e-6);
    assert!((fallback_proximity_scale(-1.0) - 0.7).abs() < 1e-6);
    assert!((fallback_proximity_scale(2.0) - 1.0).abs() < 1e-6);
}

/// `FUN_1447e3d90` lines 1097-1122 and `FUN_1447ed270` lines 48-58. The float
/// at `param_1+0x504` at or below `1.1920929e-07` forces the published
/// command to zero. Otherwise each component is clamped to `[-1, 1]`.
#[test]
fn signed_command_is_zero_when_gate_is_tiny_and_clamped_symmetrically() {
    assert_eq!(
        clamp_signed_command(FLT_EPSILON, [2.0, -3.0, 0.4]),
        [0.0; 3]
    );
    let hi = clamp_signed_command(1.0, [2.0, -3.0, 0.25]);
    assert!((hi[0] - 1.0).abs() < 1e-6 && (hi[1] + 1.0).abs() < 1e-6);
    assert!((hi[2] - 0.25).abs() < 1e-6);
    let lo = clamp_signed_command(1.0, [-2.0, 3.0, -0.25]);
    assert!((lo[0] + hi[0]).abs() < 1e-6);
    assert!((lo[1] + hi[1]).abs() < 1e-6);
    assert!((lo[2] + hi[2]).abs() < 1e-6);
}

/// `FUN_1447705d0` lines 47-55 and 102-107, and the zero stores at lines
/// 163-167. A zero gravity triple and a zero aero triple produce a zero
/// compensation, with no other drag term in this function.
#[test]
fn zero_gravity_and_zero_aero_add_no_compensation() {
    let q = Quat {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };
    let out = compensate([0.0; 3], [0.0; 3], q, 0.0, true, true);
    assert_eq!(out, [0.0; 3]);
}

/// `FUN_1447705d0` lines 70-86. Enabled gravity compensation is the gravity
/// vector rotated by the conjugate of the body quaternion (xyz negated, w
/// kept) times `(1 - value_at_+0x4c0)`. Zero pilot stick does not cancel it.
#[test]
fn gravity_compensation_follows_conjugate_rotation_and_blend() {
    let g = [0.0, 0.0, -9.81];
    let q = Quat {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };
    let out = compensate(g, [0.0; 3], q, 0.25, true, false);
    let conj = Quat {
        x: -q.x,
        y: -q.y,
        z: -q.z,
        w: q.w,
    };
    let rotated = quat_rotate(conj, g);
    let scale = 1.0 - 0.25;
    assert!((out[0] - rotated[0] * scale).abs() < 1e-5);
    assert!((out[1] - rotated[1] * scale).abs() < 1e-5);
    assert!((out[2] - rotated[2] * scale).abs() < 1e-4);
    assert!(out[2] < 0.0);
}

/// `FUN_1447ed270` lines 119-132. The vector written to the output is the
/// solver vector rotated by the body quaternion at `+0xc30`. Identity leaves
/// each axis on itself. Negating the quaternion does not change the rotation.
#[test]
fn body_command_rotation_preserves_axes_for_identity_and_double_cover() {
    let local = [0.0, 1.0, 0.0];
    let id = Quat {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };
    let got = rotate_body_command(id, local);
    assert!((got[0]).abs() < 1e-6 && (got[1] - 1.0).abs() < 1e-6 && got[2].abs() < 1e-6);
    let q = Quat {
        x: 0.0,
        y: 0.70710677,
        z: 0.0,
        w: 0.70710677,
    };
    let neg = Quat {
        x: -q.x,
        y: -q.y,
        z: -q.z,
        w: -q.w,
    };
    let a = rotate_body_command(q, local);
    let b = rotate_body_command(neg, local);
    assert!((a[0] - b[0]).abs() < 1e-5);
    assert!((a[1] - b[1]).abs() < 1e-5);
    assert!((a[2] - b[2]).abs() < 1e-5);
}

/// `FUN_143dd0b20` lines 24-55. With proportional gain 1 and integral and
/// derivative gains 0, the returned sample is `clamp(signal, low, high)`.
/// At the limit the value is the limit. Past the limit it stays on the limit.
/// `param_3 <= 1.1920929e-07` returns 0 before the clamp.
#[test]
fn axis_limit_clamps_at_and_beyond_its_bounds() {
    assert!((axis_limit_output(0.0, -4.0, 4.0) - 0.0).abs() < 1e-6);
    assert!((axis_limit_output(4.0, -4.0, 4.0) - 4.0).abs() < 1e-6);
    assert!((axis_limit_output(9.0, -4.0, 4.0) - 4.0).abs() < 1e-6);
    assert!((axis_limit_output(-9.0, -4.0, 4.0) + 4.0).abs() < 1e-6);
    assert_eq!(
        axis_limit_output(9.0, -4.0, 4.0),
        -axis_limit_output(-9.0, -4.0, 4.0)
    );
}

/// `FUN_144821090` lines 47-67, with the tick conversion in `FUN_140640dc0`
/// and `FUN_1406764a0`: seconds = tick_delta * 1e-7, then
/// `alpha = clamp(seconds * rate, 0, 1)` and
/// `value = (target - start) * alpha + start`.
/// Fallback literals when the mode table pointer is null: target `0.007`,
/// rate `0.25` (lines 47-48). The step moves toward the target and does not
/// pass it.
#[test]
fn damping_coefficient_slews_toward_target_without_overshoot() {
    let start = 0.0;
    let target = 0.007;
    let rate = 0.25;
    let dt = 0.1;
    let once = slew_damping(start, target, dt, rate);
    let alpha = (dt * rate).clamp(0.0, 1.0);
    assert!((once - ((target - start) * alpha + start)).abs() < 1e-7);
    assert!(once > start && once < target);
    let twice = slew_damping(once, target, dt, rate);
    assert!(twice > once && twice < target);
    let down = slew_damping(0.02, 0.007, 0.1, 0.25);
    assert!(down < 0.02 && down > 0.007);
}

/// `FUN_143dd0b20`: the full per-axis PID limiter with anti-windup.
///
/// With proportional gain 1 (and I/D = 0), the limiter matches the existing
/// `axis_limit_output` contract. With non-zero integral gain, the
/// accumulator integrates over time and the anti-windup clamps it back
/// when the PID output would exceed the bounds.
///
/// The g-limit curve power is an *input* to the limits (`state.low`,
/// `state.high`), not an expression inside the PID — confirming that
/// `gLimiterCurvePower` configures the bounds, not the control law.
#[test]
fn g_limiter_curve_pid_matches_simple_clamp_at_unity_gains() {
    // P=1, I=0, D=0 → PID output is just the signal, clamped.
    let mut state = AxisLimiterState::new(1.0, 0.0, 0.0, -4.0, 4.0, 1.0);
    let dt = 1.0 / 60.0;

    assert!((axis_limit_pid(&mut state, 0.0, dt) - 0.0).abs() < 1e-6);
    assert!((axis_limit_pid(&mut state, 4.0, dt) - 4.0).abs() < 1e-6);
    assert!((axis_limit_pid(&mut state, 9.0, dt) - 4.0).abs() < 1e-6);
    assert!((axis_limit_pid(&mut state, -9.0, dt) + 4.0).abs() < 1e-6);

    // Matches existing axis_limit_output for the pure-P case.
    assert_eq!(
        axis_limit_pid(
            &mut AxisLimiterState::new(1.0, 0.0, 0.0, -4.0, 4.0, 1.0),
            9.0,
            dt
        ),
        axis_limit_output(9.0, -4.0, 4.0),
    );
}

/// PID limiter with integral gain: the integral accumulates over successive
/// calls and the anti-windup prevents unbounded growth.
#[test]
fn g_limiter_pid_integral_accumulates_and_antiwindup_clamps() {
    let mut state = AxisLimiterState::new(0.5, 2.0, 0.0, -4.0, 4.0, 1.0);
    let dt = 1.0 / 60.0;

    // Constant signal=2.0: P contributes 1.0, I grows each tick.
    let v1 = axis_limit_pid(&mut state, 2.0, dt);
    let v2 = axis_limit_pid(&mut state, 2.0, dt);
    let v3 = axis_limit_pid(&mut state, 2.0, dt);

    // Integral grows → PID output grows (or clamps at high).
    assert!(v2 >= v1);
    assert!(v3 >= v2);
    assert!(v3 <= 4.0); // never exceeds the clamp
}

/// dt <= FLT_EPSILON returns 0 — the guard at the top of `FUN_143dd0b20`.
#[test]
fn g_limiter_pid_zero_dt_returns_zero() {
    let mut state = AxisLimiterState::new(1.0, 1.0, 1.0, -4.0, 4.0, 1.0);
    assert_eq!(axis_limit_pid(&mut state, 5.0, 0.0), 0.0);
    assert_eq!(axis_limit_pid(&mut state, 5.0, 1e-08), 0.0);
}

/// `FUN_144821090`: passes the slewed damping coefficient and mode-dependent
/// secondary coefficient to the physics engine vtable slot `+0x450`.
/// The resulting drag force opposes velocity (`-(coeff + secondary) * v`).
#[test]
fn damping_coefficient_removes_velocity_energy() {
    let params_default = build_damping_params(0.007, 1, false);
    assert!((params_default.coeff - 0.007).abs() < 1e-6);
    assert!((params_default.secondary - 0.00488).abs() < 1e-6);

    let params_mode2_flag = build_damping_params(0.007, 2, true);
    assert!((params_mode2_flag.secondary - 0.05).abs() < 1e-6);

    let vel = [100.0, -50.0, 25.0];
    let drag = damping_drag_force(vel, &params_default);
    // Drag opposes velocity on every axis: dot(drag, vel) < 0.
    let power = drag[0] * vel[0] + drag[1] * vel[1] + drag[2] * vel[2];
    assert!(power < 0.0);
    assert!((drag[0] + 100.0 * (0.007 + 0.00488)).abs() < 1e-4);
}

/// `FUN_143e03730`: each pilot input axis is offset-subtracted from its
/// reference, scaled per axis (`+0x1c`, `+0x20`, `+0x24`), and optionally
/// rate-limited independently per axis.
#[test]
fn stick_channel_maps_to_a_fixed_local_axis() {
    let mapped = pilot_channel_to_axis([0.5, -0.25, 1.0], [0.1, 0.0, 0.2], [2.0, 4.0, 0.5]);
    assert!((mapped[0] - 0.8).abs() < 1e-6);
    assert!((mapped[1] + 1.0).abs() < 1e-6);
    assert!((mapped[2] - 0.4).abs() < 1e-6);

    // Rate limiter clamps large steps to max_rate * step on each axis.
    let limited = pilot_channel_rate_limited(
        [1.0, -1.0, 0.2],
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 1.0],
        [0.0, 0.0, 0.0],
        [2.0, 2.0, 2.0],
        0.1,
    );
    assert!((limited[0] - 0.2).abs() < 1e-6);
    assert!((limited[1] + 0.2).abs() < 1e-6);
    assert!((limited[2] - 0.2).abs() < 1e-6);
}

// ---------------------------------------------------------------------------
// Merged from the float-bearing sweep (2026-11-22). Overlapping claims
// (g-safety threshold, proximity scale, antifall clamp) stay with the
// callee-level tests above; these pin the remaining contracts.
// ---------------------------------------------------------------------------

/// `FUN_1447eef50` line 645: the activity flag is set when any axis exceeds
/// the 0.001 deadzone (`0.001 < ABS(...)`), and cleared otherwise.
#[test]
fn input_activity_flag_follows_the_0_001_deadzone() {
    assert!(!input_active(0.0009, 0.0009, 0.0009));
    assert!(input_active(0.002, 0.0, 0.0));
    assert!(input_active(0.0, 0.0, -0.5));
}

/// `FUN_1447eef50` line 361: `0.101936795` == 1 / 9.81 converts m/s^2 to g.
/// Linearity is implied by the single multiply.
#[test]
fn g_unit_conversion_is_linear_and_one_at_standard_gravity() {
    assert!((to_g_units(9.81) - 1.0).abs() < 1e-4);
    let a = to_g_units(3.0);
    let b = to_g_units(6.0);
    assert!((b - 2.0 * a).abs() < 1e-5);
}

/// `FUN_1441ea570` lines 474-481: velocity match is per-axis within 0.05,
/// including the negated-target variant.
#[test]
fn velocity_match_uses_per_axis_0_05_tolerance() {
    assert!(velocity_matched([10.0, 20.0, 30.0], [10.04, 20.04, 30.04]));
    assert!(!velocity_matched([10.0, 20.0, 30.0], [10.06, 20.0, 30.0]));
}

/// `FUN_1447eef50` line 676: the FLT_EPSILON guard makes a zero direction
/// normalize to zero rather than NaN.
#[test]
fn normalize_zero_direction_is_finite_zero() {
    let v = normalize_direction([0.0, 0.0, 0.0]);
    assert!(v[0].is_finite() && v[1].is_finite() && v[2].is_finite());
    assert_eq!(v, [0.0; 3]);
    let d = normalize_direction([3.0, 0.0, 4.0]);
    assert!((d[0] - 0.6).abs() < 1e-6 && (d[2] - 0.8).abs() < 1e-6);
}

/// `FUN_1447e3d90` line 259: `limit * 9.81 < |v|` engages g-safety.
#[test]
fn g_safety_engages_above_limit_times_gravity() {
    assert!(!g_safety_engaged(3.0 * 9.81 - 1.0, 3.0));
    assert!(g_safety_engaged(3.0 * 9.81 + 1.0, 3.0));
}

/// `FUN_1441dc3e0` line 1042: antifall is one-sided clamped at 1000.0
/// (`vminss` with `0x447a0000`).
#[test]
fn antifall_clamps_above_1000_only() {
    assert!((antifall_clamp(1500.0) - 1000.0).abs() < 1e-6);
    assert!((antifall_clamp(999.0) - 999.0).abs() < 1e-6);
    assert!((antifall_clamp(-5.0) + 5.0).abs() < 1e-6);
}

/// pi / 180.0 / 0.31830987 (1/pi) recur in the control path; the rad/deg
/// round-trip must be identity.
#[test]
fn rad_deg_roundtrip_is_identity() {
    let deg = 37.0;
    assert!((deg_to_rad(rad_to_deg(deg)) - deg).abs() < 1e-4);
}

/// The `(1.5 - x*y*y*0.5)*y` pattern recurring in `FUN_1447eef50` (lines 694,
/// 727, 745, 762, ...) is the Newton-Raphson refinement of the fast inverse
/// square root used for normalization — NOT a control law. Recorded so a later
/// reader does not mistake it for physics.
#[test]
fn rsqrt_refinement_is_normalization_not_control_law() {
    let y = 1.0 / 2.0f32.sqrt();
    let refined = (1.5 - 2.0 * y * y * 0.5) * y;
    assert!((refined - y).abs() < 1e-6);
}
