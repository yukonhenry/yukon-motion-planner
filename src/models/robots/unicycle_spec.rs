use crate::models::robot::SpecError;

/// Specification parameters for a specific differential drive robot asset
///
/// Read from `robots.capabilities` by [`UnicycleSpec::from_capabilities`] rather than
/// hardcoded, because `capabilities` is free-form on purpose — see
/// [`RobotSpec`](crate::models::robot::RobotSpec). A physical spec is exactly the "new
/// capability is a new key rather than a migration" case that design was for.
///
/// `Copy` so [`RobotSpec`](crate::models::robot::RobotSpec) can carry one and stay `Copy`
/// itself; it is six `f64`s and gets read in the setup path, never a hot loop.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UnicycleSpec {
    pub max_linear_vel: f64,       // m/s
    pub min_linear_vel: f64,       // m/s (negative for reverse)
    pub max_angular_vel: f64,      // rad/s
    pub max_linear_accel: f64,     // m/s^2
    pub max_angular_accel: f64,    // rad/s^2
    pub track_width: f64,          // Distance between wheels (b) in meters
}

/// The `robots.capabilities` keys that describe the machine physically.
///
/// Named constants for the same reason [`MAX_VELOCITY`](crate::models::robot::MAX_VELOCITY)
/// is one: these strings are a wire contract, and a typo in one of two spellings of the same
/// key reads as "absent" and silently substitutes a default.
pub const MAX_LINEAR_VEL: &str = "max_linear_vel";
pub const MIN_LINEAR_VEL: &str = "min_linear_vel";
pub const MAX_ANGULAR_VEL: &str = "max_angular_vel";
pub const MAX_LINEAR_ACCEL: &str = "max_linear_accel";
pub const MAX_ANGULAR_ACCEL: &str = "max_angular_accel";
pub const TRACK_WIDTH: &str = "track_width";

impl Default for UnicycleSpec {
    fn default() -> Self {
        Self {
            max_linear_vel: 2.0,     // 2.0 m/s max speed
            min_linear_vel: -0.5,    // 0.5 m/s max reverse speed
            max_angular_vel: 1.5,    // ~86 deg/s max spin
            max_linear_accel: 2.5,   // m/s^2 acceleration limit
            max_angular_accel: 3.0,  // rad/s^2 rotational acceleration limit
            track_width: 0.45,       // 45 cm wheelbase width
        }
    }
}

impl UnicycleSpec {
    /// The radius of the disc this body is treated as occupying, in metres.
    ///
    /// Half the track width, which is the best available answer while the spec has no *length*:
    /// a differential-drive base is roughly as long as it is wide, and the wheelbase is the one
    /// dimension actually recorded. Deliberately a named method rather than `track_width / 2.0`
    /// scattered at call sites, so that adding a body length later changes the circumscribed
    /// radius in one place — and everything that clears obstacles by it follows.
    pub fn radius(&self) -> f64 {
        self.track_width / 2.0
    }

    /// The `(v, omega)` pair the wheels can actually produce, nearest to the one asked for.
    ///
    /// The limits on a differential drive live on the *wheels*, not on `v` and `omega`
    /// separately. With `v_L = v - omega*b/2` and `v_R = v + omega*b/2` each bounded, the
    /// feasible set in `(v, omega)` is a **diamond**, not the box that clamping the two
    /// independently describes. The box is wrong in both directions:
    ///
    /// * **Too permissive at speed.** At the default spec, `v = 2.0, omega = 1.5` passes a box
    ///   check while demanding 2.34 m/s of a wheel that tops out at 2.0 — 17% over. A planner
    ///   working from that emits trajectories the robot cannot execute, which is the one
    ///   failure a kinodynamic planner exists to prevent.
    /// * **Too restrictive at rest.** Spinning on the spot needs one wheel to reverse, so the
    ///   reverse bound is what caps it: `2 * 0.5 / 0.45 ≈ 2.2 rad/s` against the box's 1.5.
    ///
    /// Saturating each wheel independently and mapping back — rather than scaling `(v, omega)`
    /// toward the origin, which would preserve the turn radius — because that is what the
    /// hardware does: two motors, each clipping its own command. A commanded spin too fast for
    /// the reversing wheel comes back with the robot also drifting forward, which is exactly
    /// what a real base does when asked for more than it has.
    ///
    /// [`max_angular_vel`](Self::max_angular_vel) still applies on top, as the policy cap it
    /// is — a commanded yaw-rate limit independent of what the wheels could manage. Applying
    /// it after is safe: reducing `|omega|` moves both wheel speeds toward `v`, which the
    /// saturation above already left in range.
    pub fn saturate_velocity(&self, v: f64, omega: f64) -> (f64, f64) {
        self.saturate(v, omega, self.min_linear_vel, self.max_linear_vel, self.max_angular_vel)
    }

    /// The `(a, alpha)` pair the motors can actually produce, nearest to the one asked for.
    ///
    /// The same diamond one derivative up: wheel accelerations are `a -/+ alpha*b/2`, and
    /// bounding those is not the same as bounding `a` and `alpha` apart. The consequence is
    /// worth stating plainly — at full linear acceleration a differential drive **cannot turn
    /// at all**, because both motors are already saturated pushing it forward.
    ///
    /// Symmetric bounds, unlike velocity: the spec records one `max_linear_accel`, and braking
    /// as hard as accelerating is the right default for a motor.
    pub fn saturate_acceleration(&self, a: f64, alpha: f64) -> (f64, f64) {
        self.saturate(a, alpha, -self.max_linear_accel, self.max_linear_accel, self.max_angular_accel)
    }

    /// Clips a `(linear, angular)` pair to what two independently-bounded wheels can produce.
    fn saturate(
        &self,
        linear: f64,
        angular: f64,
        min_wheel: f64,
        max_wheel: f64,
        angular_cap: f64,
    ) -> (f64, f64) {
        let half = self.radius();
        let left = (linear - angular * half).clamp(min_wheel, max_wheel);
        let right = (linear + angular * half).clamp(min_wheel, max_wheel);

        // Back out of wheel space. The inverse of the mixing above, so a pair that was already
        // feasible comes through untouched.
        (
            (left + right) / 2.0,
            ((right - left) / self.track_width).clamp(-angular_cap, angular_cap),
        )
    }

    /// Reads the physical spec out of a robot's capabilities, defaulting each key it does not
    /// find.
    ///
    /// Defaulting rather than demanding, unlike [`RobotSpec::from_capabilities`], because
    /// every robot row predates these keys: requiring them would invalidate the entire stored
    /// fleet at once. A key that is *present* still has to make sense, though — a stored
    /// `track_width` of zero is a typo rather than a robot, and silently substituting 0.45 for
    /// it would hide the mistake behind plausible behavior.
    pub fn from_capabilities(capabilities: &serde_json::Value) -> Result<Self, SpecError> {
        let defaults = Self::default();

        // `Ok(fallback)` on absence, so a missing key and a present-but-wrong one are
        // different outcomes rather than both quietly becoming the default.
        let read = |key: &'static str, fallback: f64, rule: &'static str, ok: fn(f64) -> bool| {
            let Some(raw) = capabilities.get(key) else {
                return Ok(fallback);
            };
            let value = raw.as_f64().ok_or(SpecError::Missing(key))?;
            if !value.is_finite() || !ok(value) {
                return Err(SpecError::OutOfRange(key, value, rule));
            }
            Ok(value)
        };

        Ok(Self {
            max_linear_vel: read(MAX_LINEAR_VEL, defaults.max_linear_vel, "positive", |v| {
                v > 0.0
            })?,
            // Reverse is the one bound that is not positive: zero means a robot that cannot
            // back up at all, which is a legitimate machine, and a *positive* value here is
            // almost certainly a sign error that would let the robot reverse at speed.
            min_linear_vel: read(MIN_LINEAR_VEL, defaults.min_linear_vel, "zero or negative", |v| {
                v <= 0.0
            })?,
            max_angular_vel: read(MAX_ANGULAR_VEL, defaults.max_angular_vel, "positive", |v| {
                v > 0.0
            })?,
            max_linear_accel: read(MAX_LINEAR_ACCEL, defaults.max_linear_accel, "positive", |v| {
                v > 0.0
            })?,
            max_angular_accel: read(
                MAX_ANGULAR_ACCEL,
                defaults.max_angular_accel,
                "positive",
                |v| v > 0.0,
            )?,
            track_width: read(TRACK_WIDTH, defaults.track_width, "positive", |v| v > 0.0)?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn an_empty_capabilities_blob_reads_as_the_default_machine() {
        // Every stored robot predates these keys, so this is the case the whole fleet takes.
        assert_eq!(
            UnicycleSpec::from_capabilities(&json!({})).unwrap(),
            UnicycleSpec::default(),
        );
    }

    #[test]
    fn a_present_key_overrides_only_itself() {
        let spec = UnicycleSpec::from_capabilities(&json!({ "max_linear_vel": 3.25 })).unwrap();
        assert_eq!(spec.max_linear_vel, 3.25);
        assert_eq!(spec.track_width, UnicycleSpec::default().track_width);
    }

    #[test]
    fn a_present_but_nonsense_value_is_an_error_rather_than_a_default() {
        // The distinction this whole function turns on. Falling back to the default here
        // would take a typo and give it plausible behavior, which is the one outcome that
        // never gets noticed.
        for bad in [
            json!({ "max_linear_vel": 0 }),
            json!({ "max_linear_vel": -1 }),
            json!({ "track_width": 0 }),
            json!({ "max_angular_vel": -0.5 }),
            json!({ "max_linear_accel": "quick" }),
            json!({ "max_linear_vel": f64::INFINITY }),
        ] {
            assert!(
                UnicycleSpec::from_capabilities(&bad).is_err(),
                "{bad} should not describe a robot",
            );
        }
    }

    #[test]
    fn reverse_speed_is_bounded_the_other_way_round() {
        // A positive min_linear_vel is a sign error, and a permissive check would read it as
        // a robot that reverses faster than it drives forward.
        assert!(UnicycleSpec::from_capabilities(&json!({ "min_linear_vel": 0.5 })).is_err());

        assert_eq!(
            UnicycleSpec::from_capabilities(&json!({ "min_linear_vel": -1.5 }))
                .unwrap()
                .min_linear_vel,
            -1.5,
        );
        // Zero is a robot that only drives forward — unusual, not invalid.
        assert_eq!(
            UnicycleSpec::from_capabilities(&json!({ "min_linear_vel": 0 }))
                .unwrap()
                .min_linear_vel,
            0.0,
        );
    }

    /// The wheel speeds a `(v, omega)` pair demands — the quantity the bounds are actually on.
    fn wheels(spec: &UnicycleSpec, linear: f64, angular: f64) -> (f64, f64) {
        let half = spec.radius();
        (linear - angular * half, linear + angular * half)
    }

    #[test]
    fn a_feasible_command_passes_through_unchanged() {
        // Saturation must not disturb anything the robot can already do, or every trajectory
        // gets quietly bent and the dynamics stop being the dynamics.
        let spec = UnicycleSpec::default();
        for &(v, omega) in &[(0.0, 0.0), (1.0, 1.0), (1.5, 0.5), (-0.4, 0.2), (0.0, 1.5)] {
            let (out_v, out_omega) = spec.saturate_velocity(v, omega);
            assert!(
                (out_v - v).abs() < 1e-12 && (out_omega - omega).abs() < 1e-12,
                "({v}, {omega}) was feasible but came back as ({out_v}, {out_omega})",
            );
        }
    }

    #[test]
    fn no_command_can_ask_a_wheel_for_more_than_it_has() {
        // The property the diamond exists to guarantee, swept rather than sampled at a few
        // convenient points. Under the old independent clamping this fails at (2.0, 1.5) —
        // which the box called legal while demanding 2.34 m/s of a 2.0 m/s wheel.
        let spec = UnicycleSpec::default();

        for i in -30..=30 {
            for j in -30..=30 {
                let (v, omega) = (i as f64 * 0.2, j as f64 * 0.2);
                let (out_v, out_omega) = spec.saturate_velocity(v, omega);
                let (left, right) = wheels(&spec, out_v, out_omega);

                for (name, speed) in [("left", left), ("right", right)] {
                    assert!(
                        speed >= spec.min_linear_vel - 1e-9 && speed <= spec.max_linear_vel + 1e-9,
                        "({v}, {omega}) -> ({out_v}, {out_omega}) drives the {name} wheel at {speed}",
                    );
                }
                assert!(out_omega.abs() <= spec.max_angular_vel + 1e-9);
            }
        }
    }

    #[test]
    fn no_command_can_ask_a_motor_for_more_acceleration_than_it_has() {
        // The same diamond one derivative up. Independent clamping let a command for full
        // linear and full angular acceleration through, which needs one motor at 2.5 + 3.0 *
        // 0.225 = 3.18 m/s^2 against a 2.5 limit.
        let spec = UnicycleSpec::default();

        for i in -20..=20 {
            for j in -20..=20 {
                let (a, alpha) = (i as f64 * 0.5, j as f64 * 0.5);
                let (out_a, out_alpha) = spec.saturate_acceleration(a, alpha);
                let (left, right) = wheels(&spec, out_a, out_alpha);

                for (name, rate) in [("left", left), ("right", right)] {
                    assert!(
                        rate.abs() <= spec.max_linear_accel + 1e-9,
                        "({a}, {alpha}) -> ({out_a}, {out_alpha}) drives the {name} motor at {rate}",
                    );
                }
                assert!(out_alpha.abs() <= spec.max_angular_accel + 1e-9);
            }
        }
    }

    #[test]
    fn turning_at_full_speed_costs_speed() {
        // The concrete case the box got wrong. At 2.0 m/s both wheels are already at their
        // limit, so any turn at all has to come out of the forward speed — the robot slows to
        // 1.83 m/s and turns at 0.75 rad/s rather than holding 2.0 and turning at 1.5.
        let spec = UnicycleSpec::default();
        let (v, omega) = spec.saturate_velocity(2.0, 1.5);

        assert!((v - 1.83125).abs() < 1e-9, "v = {v}");
        assert!((omega - 0.75).abs() < 1e-9, "omega = {omega}");
        assert!(v < 2.0, "the robot turned without giving up any speed");

        // Going straight at the limit is untouched — the speed cap itself still means what it
        // always did.
        assert_eq!(spec.saturate_velocity(2.0, 0.0), (2.0, 0.0));
    }

    #[test]
    fn a_spin_too_fast_for_the_reversing_wheel_drifts_forward() {
        // Spinning on the spot needs one wheel to run backwards, so it is the *reverse* bound
        // that limits it, not the forward one. Asked for more than that, the two wheels end up
        // asymmetric and the robot creeps forward while it turns — which is what a real base
        // does, and what scaling (v, omega) toward the origin would have hidden.
        let spec = UnicycleSpec::default();
        let (v, omega) = spec.saturate_velocity(0.0, 5.0);

        assert!(v > 0.0, "a clipped spin should drift, got v = {v}");
        assert!((v - 0.3125).abs() < 1e-9, "v = {v}");
        assert!((omega - spec.max_angular_vel).abs() < 1e-12);

        let (left, right) = wheels(&spec, v, omega);
        assert!(left >= spec.min_linear_vel - 1e-9, "left wheel at {left}");
        assert!(right <= spec.max_linear_vel + 1e-9, "right wheel at {right}");
    }

    #[test]
    fn a_narrower_robot_turns_harder_for_the_same_wheels() {
        // Track width is what converts a wheel-speed difference into a yaw rate, so it has to
        // appear in the constraint rather than only in the kinematics. A robot half as wide
        // reaches the same spin with half the wheel difference.
        let wide = UnicycleSpec::default();
        let narrow = UnicycleSpec { track_width: 0.20, ..UnicycleSpec::default() };

        let (_, wide_omega) = wide.saturate_velocity(0.0, 10.0);
        let (_, narrow_omega) = narrow.saturate_velocity(0.0, 10.0);
        // Both hit the policy cap here, so compare the wheel demand instead.
        assert_eq!(wide_omega, narrow_omega);
        let (wide_left, _) = wheels(&wide, 0.0, wide_omega);
        let (narrow_left, _) = wheels(&narrow, 0.0, narrow_omega);
        assert!(
            narrow_left.abs() < wide_left.abs(),
            "the narrower robot should need less wheel speed: {narrow_left} vs {wide_left}",
        );
    }

    #[test]
    fn the_radius_is_half_the_track_width() {
        let spec = UnicycleSpec::from_capabilities(&json!({ "track_width": 1.2 })).unwrap();
        assert_eq!(spec.radius(), 0.6);
    }
}
