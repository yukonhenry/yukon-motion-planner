/// Abstract trait defining any type of robotic system dynamics
pub trait KinodynamicBase {
    type State: Clone + std::ops::Add<Output=Self::State> + std::ops::Mul<f64, Output=Self::State>;
    type Control;

    /// Computes the derivative dx/dt given the current state and applied control
    fn dynamics(&self, state: &Self::State, control: &Self::Control) -> Self::State;

    /// Enforces hardware constraints on controls (torques/forces/voltages) and states (joint limits)
    fn clamp_inputs(&self, control: &Self::Control) -> Self::Control;
    fn clamp_states(&self, state: &Self::State) -> Self::State;
}
