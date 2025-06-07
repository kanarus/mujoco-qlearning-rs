mod environment;

use crate::qtable::Action;

pub trait Physics {
    /// updates the state with repeatetion of `n` steps.
    fn step(&mut self, n: usize);

    /// elapsed time in seconds since the start of the physics
    fn time(&self) -> f64;

    /// current timestamp in seconds
    fn timestamp(&self) -> f64;

    /// resets the physics to its initial state
    fn reset(&mut self);
    fn after_reset(&mut self);

    fn with_reset<F: FnOnce(&mut Self)>(&mut self, f: F) {
        self.reset();
        f(self);
        self.after_reset();
    }
}

pub trait Task<P: Physics> {
    /// sets the state of the environment at the beginning of each episode
    fn initialize_episode(&mut self, physycs: &mut P);

    /// updates the task from the action
    fn before_step(&mut self, action: Action, physics: &mut P);
    /// Optional method to update the task after the physics engine has stepped
    fn after_step(&mut self, physics: &mut P);

    /// returns an observation of the current state
    fn get_observation(&self, physics: &P) -> Vec<f64>;
    /// returns a reward for the current state
    fn get_reward(&self, physics: &P) -> f64;
}

pub struct ControlEnvironment<P: Physics, T: Task<P>> {
    pub physics: P,
    pub task: T,
}
