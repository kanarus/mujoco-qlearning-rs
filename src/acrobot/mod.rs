use crate::mujoco;
use crate::control::{Action, Physics, PhysicsBase, Task};

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

    fn orientations(&self) -> Orientations {
        let [elbow_rad, shoulder_rad] = ["elbow", "shoulder"]
            .map(|name| self.model.object_id_of(mujoco::ObjectType::mjOBJ_JOINT, name).unwrap())
            .map(|id| self.position_of(id))
            .map(|mut pos_vec| {
                assert_eq!(pos_vec.len(), 1, "Expected a single position value for joint");
                pos_vec.pop().unwrap()
            });
        Orientations {
            elbow: Orientation { sin: elbow_rad.sin(), cos: elbow_rad.cos() },
            shoulder: Orientation { sin: shoulder_rad.sin(), cos: shoulder_rad.cos() },
        }
    }
}

pub struct Orientations {
    pub elbow: Orientation,
    pub shoulder: Orientation,
}

pub struct Orientation {
    pub sin: f64,
    pub cos: f64,
}

struct Balance {
    sparse: bool,
    do_swing: bool,
}


impl Balance {
    pub fn new() -> Self {
        Balance {
            sparse: false,
            do_swing: true,
        }
    }

    pub fn sparse(mut self, sparse: bool) -> Self {
        self.sparse = sparse;
        self
    }
    
    pub fn do_swing(mut self, do_swing: bool) -> Self {
        self.do_swing = do_swing;
        self
    }
}

struct BalanceAction;

impl Action for BalanceAction {
    type Physics = Acrobot;
}

struct BalanceObservation {
    orientations: Orientations,
    velocity: mujoco::Vector,
}

impl Task for Balance {
    type Physics = Acrobot;

    type Action = BalanceAction;

    type Observation = BalanceObservation;

    fn initialize_episode(&mut self, physycs: &mut Self::Physics) {
        use std::f64::consts::PI;
        use rand::Rng;

        let (elbow_id, shoulder_id) = (
            physycs.model.object_id_of(mujoco::ObjectType::mjOBJ_JOINT, "elbow").unwrap(),
            physycs.model.object_id_of(mujoco::ObjectType::mjOBJ_JOINT, "shoulder").unwrap(),
        );
        if self.do_swing {
            physycs.set_position(elbow_id, [rand::rng().random_range(-(PI/2.)..(PI/2.))]).unwrap();
        } else {
            physycs.set_position(elbow_id, [rand::rng().random_range(-(PI/10.)..(PI/10.))]).unwrap();
        }
        physycs.set_position(shoulder_id, [0.0]).unwrap();
    }

    fn before_step(&mut self, _action: Self::Action, _physics: &mut Self::Physics) {
        // do nothing
    }

    fn get_observation(&self, physics: &Self::Physics) -> Self::Observation {

        todo!()
    }

    fn get_reward(&self, physics: &Self::Physics) -> f64 {
        todo!()
    }
}
