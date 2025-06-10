use crate::mujoco::{self, MjModel, MjData};
use std::ops::{Deref, DerefMut};

pub trait Physics: Deref<Target = PhysicsBase> + DerefMut<Target = PhysicsBase> {
    fn derive(base: PhysicsBase) -> Self;

    /// ```
    /// {
    ///     self.reset();
    ///     f(self);
    ///     self.after_reset();
    /// }
    /// ```
    fn with_reset(&mut self, f: impl FnOnce(&mut Self)) {
        self.reset();
        f(self);
        self.after_reset();
    }
}

pub struct PhysicsBase {
    pub model: MjModel,
    pub data: MjData,
}

#[derive(Debug)]
pub enum PhysicsError {
    SizeNotMatching { expected: usize, actual: usize },
}
impl std::fmt::Display for PhysicsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PhysicsError::SizeNotMatching { expected, actual } => {
                write!(f, "Size not matching: expected {}, got {}", expected, actual)
            }
        }
    }
}
impl std::error::Error for PhysicsError {}

macro_rules! collect_property_elements {
    ($physics:ident: $get_property:ident * $size:ident) => {
        (0..$physics.model.$size())
            .map(|i| unsafe {
                // SAFETY: i < $size
                $physics.data.$get_property(i)
            })
            .collect()
    };
}
impl PhysicsBase {
    pub fn foward(&mut self) {
        mujoco::foward(&self.model, &mut self.data);
    }

    /// sets the control signal for the actuators
    pub fn set_control(&mut self, control: impl IntoIterator<Item = f64>) -> Result<(), PhysicsError> {
        let control: Vec<f64> = control.into_iter().collect();
        if control.len() != self.model.nu() {
            return Err(PhysicsError::SizeNotMatching {
                expected: self.model.nu(),
                actual: control.len(),
            });
        }
        // SAFETY: control has the correct size `nu`
        unsafe { self.data.set_ctrl(control); }
        Ok(())
    }

    pub fn control(&self) -> Vec<f64> {
        collect_property_elements!(self: get_ctrl * nu)
    }
    pub fn activation(&self) -> Vec<f64> {
        collect_property_elements!(self: get_act * na)
    }

    pub fn velocity(&self) -> Vec<f64> {
        collect_property_elements!(self: get_qvel * nv)
    }

    pub fn position(&self) -> Vec<f64> {
        collect_property_elements!(self: get_qpos * nq)
    }

    /// updates the state with repeatetion of `n` steps.
    pub fn step(&mut self, n: usize) {
        for _ in 0..n {
            mujoco::step(&self.model, &mut self.data);
        }
    }

    /// elapsed time in seconds since the start of the physics
    pub fn time(&self) -> f64 {
        self.data.time()
    }

    /// current timestamp in seconds
    pub fn timestamp(&self) -> f64 {
        self.model.opt().timestamp()
    }

    /// resets the physics to its initial state
    pub fn reset(&mut self) {
        mujoco::reset_data(&self.model, &mut self.data);
        self.model.with_disable(
            &[mujoco::DisableBit::mjDSBL_ACTUATION],
            |mjmodel| {mujoco::foward(mjmodel, &mut self.data);}
        );
    }
    pub fn after_reset(&mut self) {
        self.model.with_disable(
            &[mujoco::DisableBit::mjDSBL_ACTUATION],
            |mjmodel| {mujoco::foward(mjmodel, &mut self.data);}
        );
    }
}

pub trait Action {
    type Physics: Physics;
}

/// ## required
/// 
/// - `type Physics`
/// - `type Action`
/// - `fn initialize_episode`
/// - `fn before_step`
/// - `fn get_observation`
/// - `fn get_reward`
#[allow(unused_variables)]
pub trait Task {
    type Physics: Physics;
    type Action: Action<Physics = Self::Physics>;

    /// sets the state of the environment at the beginning of each episode
    fn initialize_episode(&mut self, physycs: &mut Self::Physics);

    /// updates the task from the action
    fn before_step(&mut self, action: Self::Action, physics: &mut Self::Physics);
    /// Optional method to update the task after the physics engine has stepped
    fn after_step(&mut self, physics: &mut Self::Physics) {}

    fn action_spec(&self, physics: &Self::Physics) -> BoundedArraySpec {
        let num_actions = physics.model.nu();
        BoundedArraySpec { shape: [num_actions, 1] }
    }
    fn step_spec(&self, physics: &Self::Physics) -> BoundedArraySpec {
        unimplemented!("Step spec is not implemented for this task")
    }

    /// returns an observation of the current state
    fn get_observation(&self, physics: &Self::Physics) -> Vec<f64>;
    /// returns a reward for the current state
    fn get_reward(&self, physics: &Self::Physics) -> f64;
    /// returns a final discount if the episode should end, otherwise `None`
    fn get_final_discount(&self, _physics: &Self::Physics) -> Option<f64> {
        None
    }
    fn observation_spec(&self, physics: &Self::Physics) -> BoundedArraySpec {
        unimplemented!("Observation spec is not implemented for this task")
    }
}

pub struct BoundedArraySpec {
    pub shape: [usize; 2],
}

pub trait State: Deref<Target = StateBase> + DerefMut<Target = StateBase> {
    fn derive(base: StateBase) -> Self;
}
pub struct StateBase {
    /// in `TimeStep`, this is `None` when `step_type` is `StepType::First`, i.e. at the start of a sequence
    pub reward: Option<f64>,
    pub discount: Option<f64>,
    /// in `TimeStep`, this is `None` when `step_type` is `StepType::First`, i.e. at the start of a sequence
    pub observation: Vec<f64>,
}

pub struct Environment<S: State, T: Task> {
    __state__: std::marker::PhantomData<S>,
    physics: T::Physics,
    task: T,
    n_sub_steps: usize,
    step_count: usize,
    reset_next_step: bool,
}

impl<S: State, T: Task> Environment<S, T> {
    pub fn new(physics: T::Physics, task: T) -> Self {
        Self::new_with_control_timestamp(1., physics, task)
    }
    pub fn new_with_control_timestamp(control_timestamp: f64, physics: T::Physics, task: T) -> Self {
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
    pub fn physics(&self) -> &T::Physics {
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
    pub fn reset(&mut self) -> TimeStep<S> {
        self.reset_next_step = false;
        self.step_count = 0;
        self.physics.with_reset(|physics| self.task.initialize_episode(physics));

        TimeStep {
            step_type: StepType::First,
            state: S::derive(StateBase {
                reward: None,
                discount: None,
                observation: self.task.get_observation(&self.physics),
            }),
        }
    }

    pub fn step(&mut self, action: T::Action) -> TimeStep<S> {
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
                    state: S::derive(StateBase {
                        reward: Some(reward),
                        discount: Some(final_discount),
                        observation,
                    }),
                }
            }
            None => {
                TimeStep {
                    step_type: StepType::Mid,
                    state: S::derive(StateBase {
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
