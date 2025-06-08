#[allow(non_camel_case_types, non_snake_case, non_upper_case_globals)]
mod bindgen;

pub struct MjModel {
    mjmodel: bindgen::mjModel,
}

impl MjModel {
    pub fn nu(&self) -> usize {
        self.mjmodel.nu as usize
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

pub fn action_spec(physics: &MjData) -> () {

}
