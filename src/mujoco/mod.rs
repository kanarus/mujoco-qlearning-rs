mod bindgen;

pub use bindgen::{
    mjtDisableBit as DisableBit,
    mjtObj as ObjectType,
};

pub struct MjModel {
    mjmodel: bindgen::mjModel,
}

macro_rules! mjmodel_siezs {
    ($($n_name:ident: $description:literal;)*) => {
        impl MjModel {
            $(
                #[allow(non_snake_case)]
                #[doc = $description]
                pub fn $n_name(&self) -> usize {
                    self.mjmodel.$n_name as usize
                }
            )*  
        }
    };
}
mjmodel_siezs! {
    // sizes needed at mjModel construction
    nq:                         "number of generalized coordinates = dim(qpos)";
    nv:                         "number of degrees of freedom = dim(qvel)";
    nu:                         "number of actuators/controls = dim(ctrl)";
    na:                         "number of activation states = dim(act)";
    nbody:                      "number of bodies";
    nbvh:                       "number of total bounding volumes in all bodies";
    nbvhstatic:                 "number of static bounding volumes (aabb stored in mjModel)";
    nbvhdynamic:                "number of dynamic bounding volumes (aabb stored in mjData)";
    njnt:                       "number of joints";
    ngeom:                      "number of geoms";
    nsite:                      "number of sites";
    ncam:                       "number of cameras";
    nlight:                     "number of lights";
    nflex:                      "number of flexes";
    nflexnode:                  "number of dofs in all flexes";
    nflexvert:                  "number of vertices in all flexes";
    nflexedge:                  "number of edges in all flexes";
    nflexelem:                  "number of elements in all flexes";
    nflexelemdata:              "number of element vertex ids in all flexes";
    nflexelemedge:              "number of element edge ids in all flexes";
    nflexshelldata:             "number of shell fragment vertex ids in all flexes";
    nflexevpair:                "number of element-vertex pairs in all flexes";
    nflextexcoord:              "number of vertices with texture coordinates";
    nmesh:                      "number of meshes";
    nmeshvert:                  "number of vertices in all meshes";
    nmeshnormal:                "number of normals in all meshes";
    nmeshtexcoord:              "number of texcoords in all meshes";
    nmeshface:                  "number of triangular faces in all meshes";
    nmeshgraph:                 "number of ints in mesh auxiliary data";
    nmeshpoly:                  "number of polygons in all meshes";
    nmeshpolyvert:              "number of vertices in all polygons";
    nmeshpolymap:               "number of polygons in vertex map";
    nskin:                      "number of skins";
    nskinvert:                  "number of vertices in all skins";
    nskintexvert:               "number of vertiex with texcoords in all skins";
    nskinface:                  "number of triangular faces in all skins";
    nskinbone:                  "number of bones in all skins";
    nskinbonevert:              "number of vertices in all skin bones";
    nhfield:                    "number of heightfields";
    nhfielddata:                "number of data points in all heightfields";
    ntex:                       "number of textures";
    ntexdata:                   "number of bytes in texture rgb data";
    nmat:                       "number of materials";
    npair:                      "number of predefined geom pairs";
    nexclude:                   "number of excluded geom pairs";
    neq:                        "number of equality constraints";
    ntendon:                    "number of tendons";
    nwrap:                      "number of wrap objects in all tendon paths";
    nsensor:                    "number of sensors";
    nnumeric:                   "number of numeric custom fields";
    nnumericdata:               "number of mjtNums in all numeric fields";
    ntext:                      "number of text custom fields";
    ntextdata:                  "number of mjtBytes in all text fields";
    ntuple:                     "number of tuple custom fields";
    ntupledata:                 "number of objects in all tuple fields";
    nkey:                       "number of keyframes";
    nmocap:                     "number of mocap bodies";
    nplugin:                    "number of plugin instances";
    npluginattr:                "number of chars in all plugin config attributes";
    nuser_body:                 "number of mjtNums in body_user";
    nuser_jnt:                  "number of mjtNums in jnt_user";
    nuser_geom:                 "number of mjtNums in geom_user";
    nuser_site:                 "number of mjtNums in site_user";
    nuser_cam:                  "number of mjtNums in cam_user";
    nuser_tendon:               "number of mjtNums in tendon_user";
    nuser_actuator:             "number of mjtNums in actuator_user";
    nuser_sensor:               "number of mjtNums in sensor_user";
    nnames:                     "number of chars in all names";
    npaths:                     "number of chars in all paths";

    // sizes set after mjModel construction
    nnames_map:                 "number of slots in the names hash map";
    nM:                         "number of non-zeros in sparse inertia matrix";
    nB:                         "number of non-zeros in sparse body-dof matrix";
    nC:                         "number of non-zeros in sparse reduced dof-dof matrix";
    nD:                         "number of non-zeros in sparse dof-dof matrix";
    nJmom:                      "number of non-zeros in sparse actuator_moment matrix";
    ntree:                      "number of kinematic trees under world body";
    ngravcomp:                  "number of bodies with nonzero gravcomp";
    nemax:                      "number of potential equality-constraint rows";
    njmax:                      "number of available rows in constraint Jacobian (legacy)";
    nconmax:                    "number of potential contacts in contact list (legacy)";
    nuserdata:                  "number of mjtNums reserved for the user";
    nsensordata:                "number of mjtNums in sensor data vector";
    npluginstate:               "number of mjtNums in plugin state vector";

    narena:                     "number of bytes in the mjData arena (inclusive of stack)";
    nbuffer:                    "number of bytes in buffer";
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ObjectId {
    type_: ObjectType,
    index: usize,
}
impl PartialOrd for ObjectId {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        (self.type_ == other.type_).then_some(self.index.cmp(&other.index))
    }
}

impl MjModel {
    pub fn object_id_of(&self, objtype: ObjectType, name: &str) -> Option<ObjectId> {
        let c_name = std::ffi::CString::new(name).ok()?;
        let index = unsafe {
            bindgen::mj_name2id(
                &self.mjmodel,
                objtype as i32,
                c_name.as_ptr()
            )
        };
        (index >= 0).then_some(ObjectId {
            type_: objtype,
            index: index as usize,
        })
    }

    pub fn object_name_of(&self, id: ObjectId) -> String {
        let c_name_ptr = unsafe {
            bindgen::mj_id2name(
                &self.mjmodel,
                id.type_ as i32,
                id.index as i32
            )
        };
        #[cfg(debug_assertions)] {
            assert!(!c_name_ptr.is_null(), "ObjectId {:?} is invalid", id);
        }
        (unsafe {std::ffi::CStr::from_ptr(c_name_ptr)})
            .to_str()
            .expect("Object name is not valid UTF-8")
            .to_string()
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
    pub fn qpos_index(&self, id: ObjectId) -> Option<usize> {
        match id.type_ {
            ObjectType::mjOBJ_JOINT => Some(unsafe {self.mjmodel.jnt_qposadr.add(id.index).read() as usize}),
            _ => unimplemented!(),
        }
    }

    pub fn qpos_size(&self, id: ObjectId) -> Option<usize> {
        if id.index < self.njnt() - 1 {
            let qpos_index = self.qpos_index(id)?;
            let next_qpos_index = self.qpos_index(ObjectId {
                type_: id.type_,
                index: id.index + 1,
            })?;
            Some(next_qpos_index - qpos_index)
        } else {
            // last joint
            let qpos_index = self.qpos_index(id)?;
            Some(self.nq() - qpos_index)
        }
    }
}

pub struct MjOption {
    mjopt: bindgen::mjOption,
}
impl MjModel {
    pub fn opt(&self) -> MjOption {
        MjOption { mjopt: self.mjmodel.opt }
    }
}
impl MjOption {
    pub fn timestamp(&self) -> f64 {
        self.mjopt.timestep
    }
}

pub struct MjVisual {
    mjvis: bindgen::mjVisual,
}
impl MjModel {
    pub fn vis(&self) -> MjVisual {
        MjVisual { mjvis: self.mjmodel.vis }
    }
}

pub struct MjStatistic {
    mjstat: bindgen::mjStatistic,
}
impl MjModel {
    pub fn stat(&self) -> MjStatistic {
        MjStatistic { mjstat: self.mjmodel.stat }
    }
}

pub struct Vector {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

pub struct Matrix {
    pub xx: f64, pub xy: f64, pub xz: f64,
    pub yx: f64, pub yy: f64, pub yz: f64,
    pub zx: f64, pub zy: f64, pub zz: f64,
}

pub struct MjData {
    mjdata: bindgen::mjData,
}

impl MjData {
    pub fn time(&self) -> f64 {
        self.mjdata.time
    }
}

impl MjData {
    /// get rotation matrix of the body by its `ObjectId`
    /// 
    /// returns `None` if `body_id` is not an `ObjectId` of `mjOBJ_BODY`
    pub fn xmat(&self, body_id: ObjectId) -> Option<Matrix> {
        if body_id.type_ != ObjectType::mjOBJ_BODY {
            return None;
        }

        // ```c
        // data->xmat (image):
        // 
        // [
        //     b0_xx, b0_xy, b0_xz, b0_yx, b0_yy, b0_yz, b0_zx, b0_zy, b0_zz,
        // 
        //     b1_xx, b1_xy, b1_xz, b1_yx, b1_yy, b1_yz, b1_zx, b1_zy, b1_zz,
        // 
        //     ...
        // 
        //     bN_xx, bN_xy, bN_xz, bN_yx, bN_yy, bN_yz, bN_zx, bN_zy, bN_zz
        // ] // where N := nbody - 1
        // ```
        let offset = body_id.index * 9;
        // SAFETY: `offset` < corresponded model's `nbody * 9`
        macro_rules! read {
            ($plus:expr) => {unsafe {self.mjdata.xmat.add(offset + $plus).read()}};
        }
        Some(Matrix {
            xx: read!(0), xy: read!(1), xz: read!(2),
            yx: read!(3), yy: read!(4), yz: read!(5),
            zx: read!(6), zy: read!(7), zz: read!(8),
        })
    }

    /// get position of the body by its `ObjectId`
    ///
    /// returns `None` if `body_id` is not an `ObjectId` of `mjOBJ_BODY`
    pub fn xpos(&self, body_id: ObjectId) -> Option<Vector> {
        if body_id.type_ != ObjectType::mjOBJ_BODY {
            return None;
        }

        // ```c
        // data->xpos (image):
        // 
        // [
        //     b0_xpos_x, b0_xpos_y, b0_xpos_z,
        //     b1_xpos_x, b1_xpos_y, b1_xpos_z,
        //     ...
        //     bN_xpos_x, bN_xpos_y, bN_xpos_z
        // ] // where N := nbody - 1
        // ```
        let offset = body_id.index * 3;
        // SAFETY: `offset` < corresponded model's `nbody * 3`
        Some(Vector {
            x: unsafe {self.mjdata.xpos.add(offset).read()},
            y: unsafe {self.mjdata.xpos.add(offset + 1).read()},
            z: unsafe {self.mjdata.xpos.add(offset + 2).read()},
        })
    }

    /// get position of the site by its `ObjectId`
    ///
    /// returns `None` if `site_id` is not an `ObjectId` of `mjOBJ_SITE`
    pub fn site_xpos(&self, site_id: ObjectId) -> Option<Vector> {
        if site_id.type_ != ObjectType::mjOBJ_SITE {
            return None;
        }

        // SAFETY: `offset` < corresponded model's `nsite * 3`
        let offset = site_id.index * 3;
        Some(Vector {
            x: unsafe {self.mjdata.site_xpos.add(offset).read()},
            y: unsafe {self.mjdata.site_xpos.add(offset + 1).read()},
            z: unsafe {self.mjdata.site_xpos.add(offset + 2).read()},
        })
    }
}

impl MjData {
    /// sets the control signal for the actuators
    ///
    /// SAFETY: `ctrl` must have length equal to the model's `nu`
    pub unsafe fn set_ctrl(&mut self, ctrl: impl IntoIterator<Item = f64>) {
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
    /// SAFETY: `index` < corresponded model's `nu`
    pub unsafe fn get_ctrl(&self, index: usize) -> f64 {
        unsafe {self.mjdata.ctrl.add(index).read()}
    }

    /// get one of actuators of the MjData by the index
    /// 
    /// SAFETY: `index` < corresponded model's `na`
    pub unsafe fn get_act(&self, index: usize) -> f64 {
        unsafe {self.mjdata.act.add(index).read()}
    }

    /// get one element of velocity-vector of the MjData by the index
    /// 
    /// SAFETY: `index` < corresponded model's `nv`
    pub unsafe fn get_qvel(&self, index: usize) -> f64 {
        unsafe {self.mjdata.qvel.add(index).read()}
    }

    /// get one element of position-vector of the MjData by the index
    /// 
    /// SAFETY: `index` < corresponded model's `nv`
    pub unsafe fn get_qpos(&self, index: usize) -> f64 {
        unsafe {self.mjdata.qpos.add(index).read()}
    }
    pub unsafe fn set_qpos(&mut self, index: usize, value: f64) {
        unsafe {self.mjdata.qpos.add(index).write(value)};
    }

    /// get one element of the sensor data vector of the MjData by the index
    ///
    /// SAFETY: `index` < corresponded model's `nsensordata`
    pub unsafe fn get_sensor_data(&self, index: usize) -> f64 {
        unsafe {self.mjdata.sensordata.add(index).read()}
    }

    /// get one element of the plugin state vector of the MjData by the index
    ///
    /// SAFETY: `index` < corresponded model's `npluginstate`
    pub unsafe fn get_plugin_state(&self, index: usize) -> f64 {
        unsafe {self.mjdata.plugin_state.add(index).read()}
    }

    /// get one element of the user data vector of the MjData by the index
    ///
    /// SAFETY: `index` < corresponded model's `nuserdata`
    pub unsafe fn get_user_data(&self, index: usize) -> f64 {
        unsafe {self.mjdata.userdata.add(index).read()}
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
