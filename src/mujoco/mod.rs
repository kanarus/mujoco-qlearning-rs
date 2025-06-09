mod bindgen;

pub use bindgen::{
    mjtDisableBit as DisableBit,
};

pub struct MjModel {
    mjmodel: bindgen::mjModel,
}

macro_rules! model_n {
    ($($n_name:ident),* $(,)?) => {
        impl MjModel {
            $(
                pub fn $n_name(&self) -> usize {
                    self.mjmodel.$n_name as usize
                }
            )*  
        }
    };
}
model_n! {
    nu, na, nv, nq
}

impl MjModel {
    pub fn with_disable(&mut self, flags: &[DisableBit], f: impl FnOnce(&mut Self)) {
        let old_bitmask = self.mjmodel.opt.disableflags;
        let mut new_bitmask = old_bitmask;
        for flag in flags {
            new_bitmask |= *flag as i32;
        }
        self.mjmodel.opt.disableflags = new_bitmask;
        f(self);
        self.mjmodel.opt.disableflags = old_bitmask;
    }
}

impl MjModel {
    pub fn opt(&self) -> MjOpt {
        MjOpt { mjopt: self.mjmodel.opt }
    }
}

pub struct MjOpt {
    mjopt: bindgen::mjOption,
}

impl MjOpt {
    pub fn timestamp(&self) -> f64 {
        self.mjopt.timestep
    }
}

pub struct MjData {
    mjdata: bindgen::mjData,
}

impl MjData {
    pub fn time(&self) -> f64 {
        self.mjdata.time
    }

    pub fn set_ctrl(&mut self, ctrl: impl IntoIterator<Item = f64>) {
        let mut ctrl_ptr = self.mjdata.ctrl;
        for c in ctrl {
            unsafe {
                ctrl_ptr.write(c);
                ctrl_ptr = ctrl_ptr.add(1);
            }
        }
    }

    /// get one of ctrl signals of the MjData by the index
    /// 
    /// SAFETY: `index` < corresponded model's `nu` property
    pub unsafe fn get_ctrl(&self, index: usize) -> f64 {
        unsafe {self.mjdata.ctrl.add(index).read()}
    }

    /// get one of actuators of the MjData by the index
    /// 
    /// SAFETY: `index` < corresponded model's `na` property
    pub unsafe fn get_act(&self, index: usize) -> f64 {
        unsafe {self.mjdata.act.add(index).read()}
    }

    /// get one element of velocity-vector of the MjData by the index
    /// 
    /// SAFETY: `index` < corresponded model's `nv` property
    pub unsafe fn get_qvel(&self, index: usize) -> f64 {
        unsafe {self.mjdata.qvel.add(index).read()}
    }

    /// get one element of position-vector of the MjData by the index
    /// 
    /// SAFETY: `index` < corresponded model's `nv` property
    pub unsafe fn get_qpos(&self, index: usize) -> f64 {
        unsafe {self.mjdata.qpos.add(index).read()}
    }
}

pub fn foward(mjmodel: &MjModel, mjdata: &mut MjData) {
    unsafe {
        bindgen::mj_forward(&mjmodel.mjmodel, &mut mjdata.mjdata);
    }
}

pub fn reset_data(mjmodel: &MjModel, mjdata: &mut MjData) {
    unsafe {
        bindgen::mj_resetData(&mjmodel.mjmodel, &mut mjdata.mjdata);
    }
}

pub fn step(mjmodel: &MjModel, mjdata: &mut MjData) {
    unsafe {
        bindgen::mj_step(&mjmodel.mjmodel, &mut mjdata.mjdata)
    }
}
