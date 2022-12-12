use eyre::{bail, Context};
use std::{
    // env::consts::{DLL_PREFIX, DLL_SUFFIX, EXE_SUFFIX},
    // ffi::{OsStr, OsString},
    path::Path,
    // process::Command,
};

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let target = root.join("target");

    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;
    let ros_src = Path::new("src");

    build_package("dora-node-api-cxx").await?;
    let cxxbridge = target.join("cxxbridge").join("dora-node-api-cxx").join("src");
    tokio::fs::copy(
        cxxbridge.join("lib.rs.cc"), 
        ros_src.join("map_loader").join("src").join("lib.rs.cc")
    ).await?;
    tokio::fs::copy(
        cxxbridge.join("lib.rs.h"), 
        ros_src.join("map_loader").join("include").join("map_loader").join("lib.rs.h")
    ).await?;

    tokio::fs::copy(
        cxxbridge.join("lib.rs.cc"), 
        ros_src.join("rosbag_dummy").join("src").join("lib.rs.cc")
    ).await?;
    tokio::fs::copy(
        cxxbridge.join("lib.rs.h"), 
        ros_src.join("rosbag_dummy").join("include").join("rosbag_dummy").join("lib.rs.h")
    ).await?;

    tokio::fs::copy(
        cxxbridge.join("lib.rs.cc"), 
        ros_src.join("tf_publisher").join("src").join("lib.rs.cc")
    ).await?;
    tokio::fs::copy(
        cxxbridge.join("lib.rs.h"), 
        ros_src.join("tf_publisher").join("include").join("tf_publisher").join("lib.rs.h")
    ).await?;

    tokio::fs::copy(
        cxxbridge.join("lib.rs.cc"), 
        ros_src.join("pose_initializer").join("src").join("lib.rs.cc")
    ).await?;
    tokio::fs::copy(
        cxxbridge.join("lib.rs.h"), 
        ros_src.join("pose_initializer").join("include").join("pose_initializer").join("lib.rs.h")
    ).await?;

    let ros_ws = dunce::canonicalize(".")?;
    // println!("ros workspace: {}", ros_ws.to_str().unwrap());
    
    build_ros_package(&ros_ws, "map_loader").await?;
    build_ros_package(&ros_ws, "rosbag_dummy").await?;
    build_ros_package(&ros_ws, "tf_publisher").await?;
    build_ros_package(&ros_ws, "pose_initializer").await?;

    build_package("voxel_grid_filter").await?;
    build_package("ndt_localizer").await?;

    build_package("dora-runtime").await?;

    // source_lib(&ros_ws);

    dora_coordinator::run(dora_coordinator::Args {
        run_dataflow: Path::new("dataflow.yml").to_owned().into(),
        runtime: Some(root.join("target").join("debug").join("dora-runtime")),
    })
    .await?;

    Ok(())
}

async fn build_package(package: &str) -> eyre::Result<()> {
    let cargo = std::env::var("CARGO").unwrap();
    let mut cmd = tokio::process::Command::new(&cargo);
    cmd.arg("build");
    cmd.arg("--package").arg(package);
    if !cmd.status().await?.success() {
        bail!("failed to build {package}");
    };
    Ok(())
}

async fn build_ros_package(work_dir: &Path, package: &str) -> eyre::Result<()> {
    let mut cmd = tokio::process::Command::new("colcon");
    cmd.current_dir(work_dir);
    cmd.arg("build")
    .arg("--symlink-install")
    .arg("--cmake-args").arg("-DCMAKE_BUILD_TYPE=Release")
    .arg("--packages-up-to").arg(package);
    if !cmd.status().await?.success() {
        bail!("failed to build {package}");
    };
    Ok(())
}

// fn source_lib(work_dir: &Path){
//     let mut source_cmd = Command::new("bash");
//     source_cmd.current_dir(work_dir);
//     source_cmd.arg("-c").arg("source install/setup.bash".to_string())
//     .output().expect("failed to execute source process for ros2 side dynamic lib");
// }
