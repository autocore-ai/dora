use std::{process::Command, fs, env, path::Path};
use dunce;

fn main() {
    let mut _build = cxx_build::bridge("src/ffi.rs"); // returns a cc::Build

    fs::copy("../../../../target/cxxbridge/voxel_grid_filter/src/ffi.rs.h", "include/voxel_grid_filter/ffi.rs.h").unwrap();
    fs::copy("../../../../target/cxxbridge/voxel_grid_filter/src/ffi.rs.cc", "src/ffi.rs.cc").unwrap();
    fs::copy("../../../../target/cxxbridge/rust/cxx.h", "include/voxel_grid_filter/cxx.h").unwrap();

    // let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let workspace = Path::new("../../");
    colcon_build(workspace);

    let lib_dir = dunce::canonicalize(workspace.join("install/voxel_grid_filter/lib")).unwrap();
    println!("cargo:rustc-link-lib=dylib=voxel_grid_filter_native");
    println!("cargo:rustc-link-search=native={}", env::join_paths(&[lib_dir]).unwrap().to_str().unwrap());
    println!("cargo:rerun-if-changed=src/ffi.rs");
    println!("cargo:rerun-if-changed=src/voxel_grid_filter.cpp");
    println!("cargo:rerun-if-changed=include/voxel_grid_filter/voxel_grid_filter.hpp");
    println!("cargo:rerun-if-changed=CMakeLists.txt");
}

fn colcon_build(work_dir: &Path){
    let mut cmd = Command::new("colcon");
    cmd.current_dir(work_dir);
    cmd.arg("build")
    .arg("--symlink-install")
    .arg("--cmake-args").arg("-DCMAKE_BUILD_TYPE=Release")
    .arg("--packages-up-to").arg("voxel_grid_filter")
    .output().expect("failed to execute process");
}