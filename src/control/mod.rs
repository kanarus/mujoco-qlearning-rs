use crate::mujoco::{self, MjModel, MjData};

pub struct Physics {
    mjmodel: MjModel,
    mjdata: MjData,
}

impl std::ops::Deref for Physics {
    type Target = MjModel;

    fn deref(&self) -> &Self::Target {
        &self.mjmodel
    }
}

impl Physics {
    /// sets the control signal for the actuators
    pub fn set_control(&mut self, control: impl IntoIterator<Item = f64>) {
        self.mjdata.set_ctrl(control);
    }

    pub fn foward(&mut self) {
        mujoco::foward(&self.mjmodel, &mut self.mjdata);
    }
}

impl Physics {
    /// updates the state with repeatetion of `n` steps.
    fn step(&mut self, n: usize) {
        todo!()
    }

    /// elapsed time in seconds since the start of the physics
    fn time(&self) -> f64 {
        todo!()
    }

    /// current timestamp in seconds
    fn timestamp(&self) -> f64 {
        todo!()
    }

    /// resets the physics to its initial state
    fn reset(&mut self) {
        mujoco::reset_data(&self.mjmodel, &mut self.mjdata);
    }
    fn after_reset(&mut self) {
        todo!()
    }

    fn with_reset(&mut self, f: impl FnOnce(&mut Self)) {
        self.reset();
        f(self);
        self.after_reset();
    }
}


#[allow(unused_variables)]
pub trait Task {
    type Action;

    /// sets the state of the environment at the beginning of each episode
    fn initialize_episode(&mut self, physycs: &mut Physics);

    /// updates the task from the action
    fn before_step(&mut self, action: Self::Action, physics: &mut Physics);
    /// Optional method to update the task after the physics engine has stepped
    fn after_step(&mut self, physics: &mut Physics) {}

    fn action_spec(&self, physics: &Physics) -> BoundedArraySpec {
        let num_actions = physics.nu();
        BoundedArraySpec { shape: [num_actions, 1] }
    }
    fn step_spec(&self, physics: &Physics) -> BoundedArraySpec {
        unimplemented!("Step spec is not implemented for this task")
    }

    /// returns an observation of the current state
    fn get_observation(&self, physics: &Physics) -> Vec<f64>;
    /// returns a reward for the current state
    fn get_reward(&self, physics: &Physics) -> f64;
    /// returns a final discount if the episode should end, otherwise `None`
    fn get_final_discount(&self, _physics: &Physics) -> Option<f64> {
        None
    }
    fn observation_spec(&self, physics: &Physics) -> BoundedArraySpec {
        unimplemented!("Observation spec is not implemented for this task")
    }
}

pub struct BoundedArraySpec {
    pub shape: [usize; 2],
}

pub struct Environment<S: State, T: Task> {
    __state__: std::marker::PhantomData<S>,
    physics: Physics,
    task: T,
    n_sub_steps: usize,
    step_count: usize,
    reset_next_step: bool,
}

impl<S: State, T: Task> Environment<S, T> {
    pub fn new(physics: Physics, task: T) -> Self {
        Self::new_with_control_timestamp(1., physics, task)
    }
    pub fn new_with_control_timestamp(control_timestamp: f64, physics: Physics, task: T) -> Self {
        let n_sub_steps = compute_n_steps(control_timestamp, physics.timestamp());
        Self {
            __state__: std::marker::PhantomData,
            physics,
            task,
            n_sub_steps,
            step_count: 0,
            reset_next_step: true,
        }
    }
}

impl<S: State, T: Task> Environment<S, T> {
    pub fn physics(&self) -> &Physics {
        &self.physics
    }

    pub fn task(&self) -> &T {
        &self.task
    }

    pub fn control_timestamp(&self) -> f64 {
        (self.n_sub_steps as f64) * self.physics.timestamp()
    }
}

impl<S: State, T: Task> Environment<S, T> {
    fn reset(&mut self) -> TimeStep<S> {
        self.reset_next_step = false;
        self.step_count = 0;
        self.physics.with_reset(|physics| self.task.initialize_episode(physics));

        TimeStep {
            step_type: StepType::First,
            state: S::from_base(BaseState {
                reward: None,
                discount: None,
                observation: self.task.get_observation(&self.physics),
            }),
        }
    }

    fn step(&mut self, action: T::Action) -> TimeStep<S> {
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
                    state: S::from_base(BaseState {
                        reward: Some(reward),
                        discount: Some(final_discount),
                        observation,
                    }),
                }
            }
            None => {
                TimeStep {
                    step_type: StepType::Mid,
                    state: S::from_base(BaseState {
                        reward: Some(reward),
                        discount: Some(1.0),
                        observation,
                    }),
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

pub struct TimeStep<S: State> {
    pub step_type: StepType,
    pub state: S,
}
pub enum StepType {
    First,
    Mid,
    Last,
}
pub struct BaseState {
    /// `None` when `step_type` is `StepType::First`, i.e. at the start of a sequence
    pub reward: Option<f64>,
    pub discount: Option<f64>,
    /// `None` when `step_type` is `StepType::First`, i.e. at the start of a sequence
    pub observation: Vec<f64>,
}
pub trait State {
    fn from_base(base: BaseState) -> Self;
    fn reward(&self) -> Option<f64>;
    fn discount(&self) -> Option<f64>;
    fn observation(&self) -> &[f64];
}

impl<S: State> TimeStep<S> {
    fn is_first(&self) -> bool {
        matches!(self.step_type, StepType::First)
    }
    fn is_mid(&self) -> bool {
        matches!(self.step_type, StepType::Mid)
    }
    fn is_last(&self) -> bool {
        matches!(self.step_type, StepType::Last)
    }

    fn state(&self) -> &S {
        &self.state
    }
}
