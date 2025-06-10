use crate::mujoco;
use crate::control::{BasePhysics, Physics, Task};

struct Acrobot(BasePhysics);

impl Physics for Acrobot {
    fn from_base(base: BasePhysics) -> Self {
        Acrobot(base)
    }
}
impl std::ops::Deref for Acrobot {
    type Target = BasePhysics;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl std::ops::DerefMut for Acrobot {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl Acrobot {
    /// Returns horizontal (x) component of body frame z-axes
    fn horizontal(&self) -> [f64; 2] {
        ["upper_arm", "lower_arm"]
            .map(|name| self.model.object_id_of(mujoco::ObjectType::mjOBJ_BODY, name).unwrap())
            .map(|id| unsafe {self.data.get_xmat(id, mujoco::Axis3::XZ)})
    }

    /// Returns vertical (y) component of body frame z-axes
    fn vertical(&self) -> [f64; 2] {
        ["upper_arm", "lower_arm"]
            .map(|name| self.model.object_id_of(mujoco::ObjectType::mjOBJ_BODY, name).unwrap())
            .map(|id| unsafe {self.data.get_xmat(id, mujoco::Axis3::YZ)})
    }
}

struct Balance {
    sparse: bool,
    do_swing: bool,
}

// impl Task for Balance {
//     
// }
