mod bindgen;

pub use bindgen::{
    mjtDisableBit as DisableBit,
    mjtObj as ObjectType,
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
    nu, na, nv, nq,
    nbody, njnt, nsite, ncam, nmesh, ngeom, nlight, nkey, nplugin,
    nuser_body, nuser_jnt, nuser_site, nuser_cam, nuser_geom, nuser_sensor, nuser_actuator,
}

impl MjModel {
    pub fn object_id_of(&self, objtype: ObjectType, name: &str) -> Option<usize> {
        let c_name = std::ffi::CString::new(name).ok()?;
        let id = unsafe {
            bindgen::mj_name2id(
                &self.mjmodel,
                objtype as i32,
                c_name.as_ptr()
            )
        };
        (id >= 0).then_some(id as usize)
    }
    pub fn object_name_of(&self, objtype: ObjectType, id: usize) -> Option<String> {
        let c_name_ptr = unsafe {
            bindgen::mj_id2name(
                &self.mjmodel,
                objtype as i32,
                id as i32
            )
        };
        if c_name_ptr.is_null() {
            None
        } else {
            unsafe {std::ffi::CStr::from_ptr(c_name_ptr).to_str().ok().map(str::to_string)}
        }
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
