mod bindgen;

pub use bindgen::{
    mjtDisableBit as DisableBit,
};

pub struct MjModel {
    mjmodel: bindgen::mjModel,
}

impl MjModel {
    pub fn nu(&self) -> usize {
        self.mjmodel.nu as usize
    }
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

pub struct MjData {
    mjdata: bindgen::mjData,
}

impl MjData {
    pub fn set_ctrl(&mut self, ctrl: impl IntoIterator<Item = f64>) {
        let mut ctrl_ptr = self.mjdata.ctrl;
        for c in ctrl {
            unsafe {
                ctrl_ptr.write(c);
                ctrl_ptr = ctrl_ptr.add(1);
            }
        }
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
