//! based on
//! 
//! - <https://github.com/TheButlah/mujoco-rs/blob/f7d110aa3e9c909c351f64bee0564a5d6d1abe4b/mujoco-sys/scripts/gen-bindings.rs>
//! - <https://github.com/TheButlah/mujoco-rs/blob/f7d110aa3e9c909c351f64bee0564a5d6d1abe4b/mujoco-sys/build.rs>

use std::{env, path::Path};
use bindgen::{EnumVariation};//, callbacks::ParseCallbacks};

// #[derive(Debug)]
// struct EnumPrefixStripperCallbacks;
// impl ParseCallbacks for EnumPrefixStripperCallbacks {
//     fn enum_variant_name(
//         &self,
//         enum_name: Option<&str>,
//         original_variant_name: &str,
//         _variant_value: bindgen::callbacks::EnumVariantValue,
//     ) -> Option<String> {
//         let _mjt = enum_name?.strip_prefix("enum _mjt")?;
// 
// 
// 
//         todo!()
//     }
// }

fn main() {
    if option_env!("DOCS_RS").is_some() {
        return;
    }

    let mujoco_home = env::var("MUJOCO_HOME").expect("MUJOCO_HOME not set");
    let mujoco_lib = Path::new(&mujoco_home).join("lib").to_str().unwrap().to_owned();
    let mujoco_include = Path::new(&mujoco_home).join("include").to_str().unwrap().to_owned();

    let wrapper_home = Path::new(&env!("CARGO_MANIFEST_DIR")).join("src").join("mujoco");
    let wrapper_h = wrapper_home.join("wrapper.h").to_str().unwrap().to_owned();
    let generated = wrapper_home.join("wrapper.rs").to_str().unwrap().to_owned();

    println!("cargo:rerun-if-changed={wrapper_h}");
    println!("cargo:rustc-link-search={mujoco_lib}");
    println!("cargo:rustc-link-lib=dylib=libmujoco");

    bindgen::builder()
        .header(wrapper_h)
        .clang_arg(format!("-I{mujoco_include}"))
        .rustified_enum("_?mjt.+")
        .bitfield_enum("_?mjt.+Bit")
        .allowlist_type("_?mj.*")
        .allowlist_function("_?mj.*")
        .allowlist_var("_?mj.*")
        .default_enum_style(EnumVariation::NewType { is_bitfield: false, is_global: false })
        .derive_default(true)
        .size_t_is_usize(true)
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate().expect("Failed to generate bindings")
        .write_to_file(generated).expect("Failed to write bindings to file");
}
