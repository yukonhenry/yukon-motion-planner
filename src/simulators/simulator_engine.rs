use crate::models::kino_dynamic_base::KinodynamicBase;

pub struct SimulatorEngine;

impl SimulatorEngine {
    /// Advances any kinodynamic model by dt seconds using RK4 integration
    pub fn step<M: KinodynamicBase>(model: &M, state: &M::State, control: &M::Control, dt: f64) -> M::State {
        let safe_control = model.clamp_inputs(control);

        let k1 = model.dynamics(state, &safe_control);

        let state_k2 = state.clone() + k1.clone() * (0.5 * dt);
        let k2 = model.dynamics(&state_k2, &safe_control);

        let state_k3 = state.clone() + k2.clone() * (0.5 * dt);
        let k3 = model.dynamics(&state_k3, &safe_control);

        let state_k4 = state.clone() + k3.clone() * dt;
        let k4 = model.dynamics(&state_k4, &safe_control);

        let next_state = state.clone() + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
        model.clamp_states(&next_state)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::robots::unicycle_spec::UnicycleSpec;
    use crate::models::robots::unicycle_state::{
        SecondOrderUnicycle, UnicycleControl, UnicycleState,
    };

    fn robot() -> SecondOrderUnicycle {
        SecondOrderUnicycle {
            spec: UnicycleSpec::default(),
        }
    }

    /// Coasting: no control input at all, so `v` and `omega` are whatever the state carries.
    const COAST: UnicycleControl = UnicycleControl {
        accel: 0.0,
        angular_accel: 0.0,
    };

    fn at(x: f64, y: f64, theta: f64, v: f64, omega: f64) -> UnicycleState {
        UnicycleState {
            x,
            y,
            theta,
            v,
            omega,
        }
    }

    fn integrate(
        model: &SecondOrderUnicycle,
        mut state: UnicycleState,
        control: &UnicycleControl,
        dt: f64,
        steps: usize,
    ) -> UnicycleState {
        for _ in 0..steps {
            state = SimulatorEngine::step(model, &state, control, dt);
        }
        state
    }

    /// Forward Euler over the same model, as the thing RK4 has to beat.
    ///
    /// Written out here rather than shared, because its only job is to be the *worse*
    /// integrator: if it ever stops being worse, the comparison below should fail rather
    /// than quietly agree.
    fn euler_step(
        model: &SecondOrderUnicycle,
        state: &UnicycleState,
        control: &UnicycleControl,
        dt: f64,
    ) -> UnicycleState {
        let derivative = model.dynamics(state, &model.clamp_inputs(control));
        model.clamp_states(&(state.clone() + derivative * dt))
    }

    #[test]
    fn coasting_straight_covers_speed_times_time() {
        // The one case with a closed form simple enough to assert exactly: heading fixed at
        // zero and no acceleration makes dx/dt a constant, and RK4 is exact on a constant.
        // A sign slip or a mis-scaled stage would show up here before anything subtler does.
        let model = robot();
        let dt = 0.02;
        let steps = 50; // 1.0s
        let end = integrate(&model, at(0.0, 0.0, 0.0, 1.5, 0.0), &COAST, dt, steps);

        assert!((end.x - 1.5).abs() < 1e-12, "x = {}", end.x);
        assert!(end.y.abs() < 1e-12, "y = {}", end.y);
        assert!(end.theta.abs() < 1e-12, "theta = {}", end.theta);
        assert!((end.v - 1.5).abs() < 1e-12, "coasting should not change speed");
    }

    /// The unit-radius arc this suite measures against: constant `v` and `omega` of 1 from
    /// the origin, whose exact solution is x = sin t, y = 1 - cos t, theta = t.
    ///
    /// Deliberately *not* stopped at a quarter turn, even though (1, 1) is the tidier
    /// number. A quarter turn is symmetric about the diagonal, so swapping the sin and the
    /// cos in the dynamics — putting the robot's forward axis at right angles to its heading
    /// — lands on the very same point and goes unnoticed. A third of a turn is asymmetric,
    /// and tells the two apart.
    const ARC_TIME: f64 = std::f64::consts::FRAC_PI_3;
    const ARC_STEPS: usize = 64;

    fn exact_arc() -> (f64, f64) {
        (ARC_TIME.sin(), 1.0 - ARC_TIME.cos())
    }

    /// Constant `v` and `omega` trace a circle of radius `v / omega`. This is the property
    /// that actually pins the integrator: it couples `theta` into `x` and `y`, so an
    /// integrator that got the stage weights, the ordering, or the sin/cos pairing wrong
    /// still travels the right *distance* while ending up somewhere else entirely.
    #[test]
    fn a_constant_turn_traces_the_arc_it_should() {
        let model = robot();
        let (x, y) = exact_arc();
        let end = integrate(
            &model,
            at(0.0, 0.0, 0.0, 1.0, 1.0),
            &COAST,
            ARC_TIME / ARC_STEPS as f64,
            ARC_STEPS,
        );

        assert!((end.x - x).abs() < 1e-9, "x = {} rather than {x}", end.x);
        assert!((end.y - y).abs() < 1e-9, "y = {} rather than {y}", end.y);
        assert!((end.theta - ARC_TIME).abs() < 1e-9, "theta = {}", end.theta);
    }

    /// Why this is RK4 and not Euler.
    ///
    /// Asserted as a ratio rather than two absolute bounds because the absolute numbers are a
    /// function of the step size: what has to hold is that fourth-order accuracy is *being
    /// obtained*, and a "simplification" of the stages into something first-order would pass
    /// a loose absolute bound while failing this.
    #[test]
    fn rk4_tracks_the_arc_far_closer_than_euler_does() {
        let model = robot();
        let dt = ARC_TIME / ARC_STEPS as f64;
        let (x, y) = exact_arc();
        let start = at(0.0, 0.0, 0.0, 1.0, 1.0);

        let rk4 = integrate(&model, start.clone(), &COAST, dt, ARC_STEPS);

        let mut euler = start;
        for _ in 0..ARC_STEPS {
            euler = euler_step(&model, &euler, &COAST, dt);
        }

        let error = |s: &UnicycleState| ((s.x - x).powi(2) + (s.y - y).powi(2)).sqrt();
        let (rk4_error, euler_error) = (error(&rk4), error(&euler));

        assert!(rk4_error < 1e-9, "RK4 error {rk4_error} is not fourth-order");
        assert!(
            euler_error > rk4_error * 1e6,
            "RK4 error {rk4_error} is not decisively better than Euler's {euler_error}",
        );
    }

    #[test]
    fn a_control_beyond_the_spec_saturates_instead_of_applying() {
        // What `clamp_inputs` is for. Over one step from rest RK4 on a constant derivative is
        // exact, so the saturated acceleration is readable straight off the velocities.
        //
        // Commanded one axis at a time, because the wheel diamond means a robot asked for
        // everything at once cannot have it — see the combined case below.
        let model = robot();
        let spec = robot().spec;
        let step = |accel, angular_accel| {
            SimulatorEngine::step(
                &model,
                &at(0.0, 0.0, 0.0, 0.0, 0.0),
                &UnicycleControl { accel, angular_accel },
                0.1,
            )
        };

        let straight = step(100.0, 0.0);
        assert!((straight.v - spec.max_linear_accel * 0.1).abs() < 1e-12, "v = {}", straight.v);
        assert!(straight.omega.abs() < 1e-12, "a straight command should not turn");

        let spin = step(0.0, 100.0);
        assert!(spin.v.abs() < 1e-12, "a pure spin command should not translate");
        assert!(
            (spin.omega - spec.max_angular_accel * 0.1).abs() < 1e-12,
            "omega = {}",
            spin.omega,
        );
    }

    #[test]
    fn full_linear_acceleration_leaves_nothing_over_to_turn_with() {
        // The wheel diamond, stated as the behavior it implies. Both motors saturated pushing
        // the robot forward have nothing left to differ by, so a command for maximum linear
        // *and* maximum angular acceleration yields maximum linear and zero angular — not
        // both, which is what clamping the two independently used to report.
        let model = robot();
        let spec = robot().spec;
        let both = SimulatorEngine::step(
            &model,
            &at(0.0, 0.0, 0.0, 0.0, 0.0),
            &UnicycleControl { accel: 100.0, angular_accel: 100.0 },
            0.1,
        );

        assert!((both.v - spec.max_linear_accel * 0.1).abs() < 1e-12, "v = {}", both.v);
        assert!(
            both.omega.abs() < 1e-12,
            "the robot turned while both wheels were already at full acceleration: omega = {}",
            both.omega,
        );
    }

    #[test]
    fn sustained_acceleration_stops_at_the_velocity_the_hardware_allows() {
        // Clamping the *input* bounds how fast the robot can change speed; it says nothing
        // about the speed itself. Without the state cap a robot accelerating for long enough
        // would outrun its own spec, which is the number every downstream cost model trusts.
        let model = robot();
        let spec = robot().spec;

        let forward = integrate(
            &model,
            at(0.0, 0.0, 0.0, 0.0, 0.0),
            &UnicycleControl {
                accel: 100.0,
                angular_accel: 0.0,
            },
            0.05,
            200, // 10s, far longer than saturation needs
        );
        assert!((forward.v - spec.max_linear_vel).abs() < 1e-12, "v = {}", forward.v);

        // Spin is capped separately, and by the policy limit rather than by the wheels: at
        // `max_angular_vel` of 1.5 rad/s the wheels are turning +/-0.34 m/s, well inside their
        // own bounds, so it is `max_angular_vel` that binds first.
        let spinning = integrate(
            &model,
            at(0.0, 0.0, 0.0, 0.0, 0.0),
            &UnicycleControl {
                accel: 0.0,
                angular_accel: 100.0,
            },
            0.05,
            200,
        );
        assert!(
            (spinning.omega - spec.max_angular_vel).abs() < 1e-12,
            "omega = {}",
            spinning.omega,
        );
        assert!(spinning.v.abs() < 1e-12, "a pure spin drifted: v = {}", spinning.v);

        // Reverse is capped separately and much lower — a robot that reversed as fast as it
        // drove would be a different machine.
        let backward = integrate(
            &model,
            at(0.0, 0.0, 0.0, 0.0, 0.0),
            &UnicycleControl {
                accel: -100.0,
                angular_accel: 0.0,
            },
            0.05,
            200,
        );
        assert!((backward.v - spec.min_linear_vel).abs() < 1e-12, "v = {}", backward.v);
    }
}
