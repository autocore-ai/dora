use std::{process::Command, fs, env, path::Path};
use dunce;

fn main() {
    let mut _build = cxx_build::bridge("src/ffi.rs"); // returns a cc::Build

    fs::copy("../../../../../../target/cxxbridge/imu_corrector/src/ffi.rs.h", "include/imu_corrector/ffi.rs.h").unwrap();
    fs::copy("../../../../../../target/cxxbridge/imu_corrector/src/ffi.rs.cc", "src/ffi.rs.cc").unwrap();
    fs::copy("../../../../../../target/cxxbridge/rust/cxx.h", "include/imu_corrector/cxx.h").unwrap();

    // let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let workspace = Path::new("../../../");
    colcon_build(workspace);
    
    let lib_dir = dunce::canonicalize(workspace.join("install/imu_corrector/lib")).unwrap();
    println!("cargo:rustc-link-lib=dylib=imu_corrector_native");
    println!("cargo:rustc-link-search=native={}", env::join_paths(&[lib_dir]).unwrap().to_str().unwrap());
    println!("cargo:rerun-if-changed=src/ffi.rs");
    println!("cargo:rerun-if-changed=src/imu_corrector.cpp");
    println!("cargo:rerun-if-changed=include/imu_corrector/imu_corrector.hpp");
    println!("cargo:rerun-if-changed=CMakeLists.txt");
}

fn colcon_build(work_dir: &Path){
    let mut cmd = Command::new("colcon");
    cmd.current_dir(work_dir);
    cmd.arg("build")
    .arg("--symlink-install")
    .arg("--cmake-args").arg("-DCMAKE_BUILD_TYPE=Release")
    .arg("--packages-up-to").arg("imu_corrector")
    .output().expect("failed to execute process");
}