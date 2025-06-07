use super::environment::{Environment, TimeStep, StepType};
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
    /// returns a final discount if the episode should end, otherwise `None`
    fn get_final_discount(&self, _physics: &P) -> Option<f64> {
        None
    }
}

pub struct ControlEnvironment<P: Physics, T: Task<P>> {
    physics: P,
    task: T,
    n_sub_steps: usize,
    step_count: usize,
    reset_next_step: bool,
}

impl<P: Physics, T: Task<P>> ControlEnvironment<P, T> {
    pub fn new(physics: P, task: T) -> Self {
        Self::new_with_control_timestamp(1., physics, task)
    }
    pub fn new_with_control_timestamp(control_timestamp: f64, physics: P, task: T) -> Self {
        let n_sub_steps = compute_n_steps(control_timestamp, physics.timestamp());
        Self {
            physics,
            task,
            n_sub_steps,
            step_count: 0,
            reset_next_step: true,
        }
    }
}

impl<P: Physics, T: Task<P>> ControlEnvironment<P, T> {
    pub fn physics(&self) -> &P {
        &self.physics
    }

    pub fn task(&self) -> &T {
        &self.task
    }

    pub fn control_timestamp(&self) -> f64 {
        (self.n_sub_steps as f64) * self.physics.timestamp()
    }
}

impl<P: Physics, T: Task<P>> Environment for ControlEnvironment<P, T> {
    fn reset(&mut self) -> TimeStep {
        self.reset_next_step = false;
        self.step_count = 0;
        self.physics.with_reset(|physics| self.task.initialize_episode(physics));

        TimeStep {
            step_type: StepType::First,
            reward: None,
            discount: None,
            observation: self.task.get_observation(&self.physics),
        }
    }

    fn step(&mut self, action: Action) -> TimeStep {
        if self.reset_next_step {
            return self.reset();
        }

        self.task.before_step(action, &mut self.physics);
        self.physics.step(self.n_sub_steps);
        self.task.after_step(&mut self.physics);

        let reward = self.task.get_reward(&self.physics);
        let observation = self.task.get_observation(&self.physics);

        self.step_count += 1;

        match self.task.get_final_discount(&self.physics) {
            Some(final_discount) => {
                self.reset_next_step = true;
                TimeStep {
                    step_type: StepType::Last,
                    reward: Some(reward),
                    discount: Some(final_discount),
                    observation,
                }
            }
            None => {
                TimeStep {
                    step_type: StepType::Mid,
                    reward: Some(reward),
                    discount: Some(1.0),
                    observation,
                }
            }
        }
    }
}

fn compute_n_steps(
    control_timestamp: f64,
    physics_timestamp: f64,
) -> usize {
    assert! {
        control_timestamp >= physics_timestamp,
        "Control timestamp ({control_timestamp}) must not exceeds physics timestamp ({physics_timestamp})",
    }

    let div = control_timestamp / physics_timestamp;
    let rounded_div = div.round();

    assert! {
        f64::abs(rounded_div - div) < f64::EPSILON,
        "Control timestamp ({control_timestamp}) must be an integer-multiple of physics timestamp ({physics_timestamp})",
    }

    rounded_div as usize
}
