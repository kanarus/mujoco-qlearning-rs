mod environment;
mod control;

use crate::mujoco::{MjModel, MjData};

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
}
