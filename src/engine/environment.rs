use crate::qtable::Action;

pub struct TimeStep {
    pub(super) step_type: StepType,
    /// `None` when `step_type` is `StepType::First`, i.e. at the start of a sequence
    pub(super) reward: Option<f64>,
    /// `None` when `step_type` is `StepType::First`, i.e. at the start of a sequence
    pub(super) discount: Option<f64>,
    pub(super) observation: Vec<f64>,
}
impl TimeStep {
    fn is_first(&self) -> bool {
        matches!(self.step_type, StepType::First)
    }
    fn is_mid(&self) -> bool {
        matches!(self.step_type, StepType::Mid)
    }
    fn is_last(&self) -> bool {
        matches!(self.step_type, StepType::Last)
    }
}

pub enum StepType {
    First,
    Mid,
    Last,
}

pub trait Environment<A> {
    /// starts a  sequence and returns the first time step of this sequence
    fn reset(&mut self) -> TimeStep;

    /// updates the environment according to the action and returns a new time step
    fn step(&mut self, action: A) -> TimeStep;

    /// frees resources used by the environment
    fn close(&mut self) {}
}
