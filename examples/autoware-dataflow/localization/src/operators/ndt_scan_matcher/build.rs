use std::{process::Command, fs, env, path::Path};
use dunce;

fn main() {
    let mut _build = cxx_build::bridge("src/ffi.rs"); // returns a cc::Build

    fs::copy("../../../../../../target/cxxbridge/ndt_scan_matcher/src/ffi.rs.h", "include/ndt_scan_matcher/ffi.rs.h").unwrap();
    fs::copy("../../../../../../target/cxxbridge/ndt_scan_matcher/src/ffi.rs.cc", "src/ffi.rs.cc").unwrap();
    fs::copy("../../../../../../target/cxxbridge/rust/cxx.h", "include/ndt_scan_matcher/cxx.h").unwrap();

    // let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let workspace = Path::new("../../../");
    colcon_build(workspace);
    
    let lib_dir = dunce::canonicalize(workspace.join("install/ndt_scan_matcher/lib")).unwrap();
    println!("cargo:rustc-link-lib=dylib=ndt_scan_matcher_native");
    println!("cargo:rustc-link-search=native={}", env::join_paths(&[lib_dir]).unwrap().to_str().unwrap());
    println!("cargo:rerun-if-changed=src/ffi.rs");
    println!("cargo:rerun-if-changed=src/ndt_scan_matcher.cpp");
    println!("cargo:rerun-if-changed=include/ndt_scan_matcher/ndt_scan_matcher.hpp");
    println!("cargo:rerun-if-changed=CMakeLists.txt");
}

fn colcon_build(work_dir: &Path){
    let mut cmd = Command::new("colcon");
    cmd.current_dir(work_dir);
    cmd.arg("build")
    .arg("--symlink-install")
    .arg("--cmake-args").arg("-DCMAKE_BUILD_TYPE=Release")
    .arg("--packages-up-to").arg("ndt_scan_matcher")
    .output().expect("failed to execute process");
}