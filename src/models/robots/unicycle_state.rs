use crate::models::kino_dynamic_base::KinodynamicBase;
use crate::models::robots::unicycle_spec::UnicycleSpec;
use std::ops::{Add, Mul};

/// Folds an angle into `[-pi, pi)`.
///
/// `rem_euclid` rather than `%`: Rust's float remainder takes the sign of the dividend, so the
/// obvious `(theta + PI) % (2 * PI) - PI` is an identity for every negative angle — a robot
/// turning clockwise would wind `theta` off toward -inf instead of wrapping, and nothing that
/// compares headings would work again.
///
/// Shared rather than inlined where it is needed, because the planner's distance metric has to
/// measure heading differences the same way the dynamics wrap them. Two spellings of this is
/// how a state ends up "far" from itself.
pub fn wrap_angle(theta: f64) -> f64 {
    (theta + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI) - std::f64::consts::PI
}

/// The 5-DOF State Vector: [x, y, theta, v, omega]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnicycleState {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
    pub v: f64,
    pub omega: f64,
}

/// The 2-DOF Control Input Vector: [a, alpha]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnicycleControl {
    pub accel: f64,
    pub angular_accel: f64,
}

// Implement Addition for state mixing inside RK4 engine
impl Add for UnicycleState {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            theta: self.theta + other.theta,
            v: self.v + other.v,
            omega: self.omega + other.omega,
        }
    }
}

// Implement Scalar Multiplication for integration scaling steps
impl Mul<f64> for UnicycleState {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
            theta: self.theta * scalar,
            v: self.v * scalar,
            omega: self.omega * scalar,
        }
    }
}

/// Implementation of the Kinodynamic Model trait for our unicycle specifications
pub struct SecondOrderUnicycle {
    pub spec: UnicycleSpec,
}

impl KinodynamicBase for SecondOrderUnicycle {
    type State = UnicycleState;
    type Control = UnicycleControl;

    /// Evaluates the core 5-DOF continuous ODE equations: x_dot = f(x, u)
    fn dynamics(&self, state: &Self::State, control: &Self::Control) -> Self::State {
        Self::State {
            x: state.v * state.theta.cos(),
            y: state.v * state.theta.sin(),
            theta: state.omega,
            v: control.accel,
            omega: control.angular_accel,
        }
    }

    /// Enforces the actuator saturation limits (Control Box Constraints)
    fn clamp_inputs(&self, control: &Self::Control) -> Self::Control {
        let (accel, angular_accel) = self
            .spec
            .saturate_acceleration(control.accel, control.angular_accel);
        Self::Control {
            accel,
            angular_accel,
        }
    }

    /// Enforces the mechanical velocity safety caps (State Box Constraints)
    fn clamp_states(&self, state: &Self::State) -> Self::State {
        let (v, omega) = self.spec.saturate_velocity(state.v, state.omega);
        Self::State {
            x: state.x,
            y: state.y,
            theta: wrap_angle(state.theta),
            v,
            omega,
        }
    }
}

impl SecondOrderUnicycle {
    /// Helper to convert global unicycle states into raw wheel targets (m/s)
    pub fn to_wheel_velocities(&self, state: &UnicycleState) -> (f64, f64) {
        let half_b = self.spec.track_width / 2.0;
        let left_wheel_vel = state.v - (state.omega * half_b);
        let right_wheel_vel = state.v + (state.omega * half_b);
        (left_wheel_vel, right_wheel_vel)
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    fn robot() -> SecondOrderUnicycle {
        SecondOrderUnicycle {
            spec: UnicycleSpec::default(),
        }
    }

    fn heading(theta: f64) -> f64 {
        robot()
            .clamp_states(&UnicycleState {
                x: 0.0,
                y: 0.0,
                theta,
                v: 0.0,
                omega: 0.0,
            })
            .theta
    }

    /// Wrapping has to work in *both* directions.
    ///
    /// Rust's `%` on floats takes the sign of the dividend, so the obvious
    /// `(theta + PI) % (2 * PI) - PI` is an identity for every negative angle — a robot
    /// turning clockwise winds `theta` off toward -inf rather than wrapping, and nothing
    /// downstream that compares headings works again. Hence `rem_euclid`, and hence the
    /// negative cases here.
    #[test]
    fn a_heading_wraps_into_a_half_open_turn_from_either_side() {
        let pi = std::f64::consts::PI;

        for (raw, expected) in [
            (0.5, 0.5),
            (-0.5, -0.5),
            (3.5, 3.5 - 2.0 * pi),
            (-3.5, 2.0 * pi - 3.5),
            (7.0, 7.0 - 2.0 * pi),
            (-7.0, 2.0 * pi - 7.0),
        ] {
            assert!(
                (heading(raw) - expected).abs() < 1e-12,
                "{raw} wrapped to {} rather than {expected}",
                heading(raw),
            );
        }

        // The interval is [-pi, pi): half open, so exactly one of the two representations of
        // a half turn survives. A closed interval would let the same heading compare unequal
        // to itself depending on which way the robot arrived at it.
        assert!((heading(pi) + pi).abs() < 1e-12, "pi should fold onto -pi");
        assert!((heading(-pi) + pi).abs() < 1e-12, "-pi should stay put");
    }

    /// Spinning one way for long enough is exactly the case the truncated-remainder bug hid:
    /// each individual step looks fine, and the error only shows once the heading crosses
    /// -pi.
    #[test]
    fn spinning_clockwise_stays_inside_one_turn() {
        use crate::simulators::simulator_engine::SimulatorEngine;

        let model = robot();
        let mut state = UnicycleState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            v: 0.0,
            omega: -1.0,
        };
        let control = UnicycleControl {
            accel: 0.0,
            angular_accel: 0.0,
        };

        // 4 radians clockwise — comfortably past -pi, so an unwrapped heading is unmistakable.
        for _ in 0..400 {
            state = SimulatorEngine::step(&model, &state, &control, 0.01);
        }

        let pi = std::f64::consts::PI;
        assert!(
            state.theta >= -pi && state.theta < pi,
            "theta escaped one turn: {}",
            state.theta,
        );
        assert!(
            (state.theta - (2.0 * pi - 4.0)).abs() < 1e-9,
            "theta = {} rather than the wrapped -4 rad",
            state.theta,
        );
    }

    /// The differential-drive constraint: a pure spin turns the wheels equal and opposite,
    /// and driving straight turns them together. Getting the sign of the `omega * b/2` term
    /// backwards swaps left for right, which is invisible in simulation and destructive on
    /// hardware.
    #[test]
    fn wheel_velocities_split_around_the_body_speed() {
        let model = robot();
        let half_b = model.spec.track_width / 2.0;

        let straight = UnicycleState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            v: 1.0,
            omega: 0.0,
        };
        assert_eq!(model.to_wheel_velocities(&straight), (1.0, 1.0));

        let spin = UnicycleState {
            omega: 2.0,
            v: 0.0,
            ..straight
        };
        let (left, right) = model.to_wheel_velocities(&spin);
        assert!((left + right).abs() < 1e-12, "a spin should not translate");
        assert!(
            (right - 2.0 * half_b).abs() < 1e-12,
            "a positive omega should drive the right wheel forward, got {right}",
        );
    }
}
