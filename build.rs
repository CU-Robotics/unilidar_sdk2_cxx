use std::env;
use std::path::PathBuf;

fn main() {
    cxx_build::bridge("src/lib.rs")
        .file("include/lidar_wrapper.cpp")
        .flag("-std=c++17")
        .compile("cxxbridge-demo");

    let arch = env::var("CARGO_CFG_TARGET_ARCH").unwrap();
    let lib_dir = PathBuf::from("unitree_lidar_sdk/lib").join(&arch);
    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    println!("cargo:rustc-link-lib=static=unilidar_sdk2");

    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=include/lidar_wrapper.h");
    println!("cargo:rerun-if-changed=include/lidar_wrapper.cpp");
}
