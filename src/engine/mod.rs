mod environment;
mod control;

use crate::mujoco::{mjData};

pub struct Physics {
    mjdata: mjData,
}

impl Physics {
    /// sets the control signal for the actuators
    pub fn set_control(&mut self, control: ()) {
        self.mjdata.ctrl = control;
    }
}
