use crate::mujoco;
use crate::control::{PhysicsBase, Physics, Task};

struct Acrobot(PhysicsBase);

impl Physics for Acrobot {
    fn derive(base: PhysicsBase) -> Self {
        Acrobot(base)
    }
}
impl std::ops::Deref for Acrobot {
    type Target = PhysicsBase;
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
            .map(|id| self.data.get_xmat(id, mujoco::MatrixComponent::XZ).unwrap())// `id` is a body id
    }

    /// Returns vertical (y) component of body frame z-axes
    fn vertical(&self) -> [f64; 2] {
        ["upper_arm", "lower_arm"]
            .map(|name| self.model.object_id_of(mujoco::ObjectType::mjOBJ_BODY, name).unwrap())
            .map(|id| self.data.get_xmat(id, mujoco::MatrixComponent::ZZ).unwrap())// `id` is a body id
    }

    fn distance_tip_target(&self) -> f64 {

    }
}

struct Balance {
    sparse: bool,
    do_swing: bool,
}

// impl Task for Balance {
//     
// }
