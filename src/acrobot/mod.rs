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
            .map(|id| self.data.xmat(id).unwrap().xz)
    }

    /// Returns vertical (y) component of body frame z-axes
    fn vertical(&self) -> [f64; 2] {
        ["upper_arm", "lower_arm"]
            .map(|name| self.model.object_id_of(mujoco::ObjectType::mjOBJ_BODY, name).unwrap())
            .map(|id| self.data.xmat(id).unwrap().zz)
    }

    fn distance_tip_target(&self) -> f64 {
        let [tip, target] = ["tip", "target"]
            .map(|name| self.model.object_id_of(mujoco::ObjectType::mjOBJ_SITE, name).unwrap())
            .map(|id| self.data.site_xpos(id).unwrap());
        f64::sqrt(
            (tip.x - target.x).powi(2) +
            (tip.y - target.y).powi(2) +
            (tip.z - target.z).powi(2)
        )
    }
}

struct Balance {
    sparse: bool,
    do_swing: bool,
}

// impl Task for Balance {
//     
// }
