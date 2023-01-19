

# Brief description

Taking [a clean and simple normal distribution transform (NDT) localizer](https://github.com/AbangLZU/ndt_localizer) as an example, ndt_localizer removes the blocks in the node migration and compilation process and uses necessary auxiliary software packages to form a minimum closed loop for testing.

The changes made to implement node migration mainly cover the following aspects:

1. The source code in CMakeLists.txt, .hpp files, and .cpp files is modified. 

2. The dora framework is used and ffi.rs is defined based on the source code in the launch and config folders.

3. Message serialization and deserialization are implemented in C++.

4. Code compilation in C++ and Rust is performed, and blocks in compilation are removed. Note that the dependencies on some Robot Operating System (ROS) dynamic libraries are retained, which will be addressed in a later version.

5. Auxiliary nodes are added to ensure the minimum closed loop, including manual sending of coordinate transformation information.

You can compare the code in the original project with that in the current directory for details of changes. The original project uses ROS 1 code, and the type of shared_ptr in use is boost::shared_ptr. However, member variables of this pointer type are private and cannot be serialized by using cereal. Therefore, ROS 1 code needs to be changed to ROS 2 code.

# Usage

Use the following syntax to compile code and run ndt_localizer:

```bash
# Compile code
cd dora/examples/autoware-dataflow/ndt_localizer
source /opt/ros/galactic/setup.bash
cargo run --example ndt-dataflow
# Run ndt_localizer
source install/setup.bash
../../../target/debug/dora-coordinator --run-dataflow dataflow.yml --runtime ../../../target/debug/dora-runtime
```

It is originally designed that ndt_localizer is started by directly running `cargo run --example ndt-dataflow`. However, `source install/setup.bash` cannot be effectively invoked in *examples/autoware-dataflow/ndt_localizer/run.rs*. Therefore, we need to manually run the `source` command on the ROS 2 dynamic library and executable program installed under the *install* directory, and then use dora-coordinator and dora-runtime to start ndt_localizer.

# Details about code rewriting

The code of the original project can be divided into the following four main subsets from the perspective of features: *map_loader*, *robot_description*, *points_downsample*, and *ndt_localizer*. To implement node migration, we can perform the following code rewriting operations based on the I/O relationships between the four features: rewrite *map_loader* and *robot_description* into dora nodes and rewrite *points_downsample* and *ndt_localizer* into dora operators. 

In addition, to form a minimum closed loop for testing, we need to add at least the following auxiliary nodes: *pose_initializer* for setting the initial position, *rosbag_dummy* for simulating point cloud data play by using rosbag, and *time_utils* for timestamp converting.

## Rewriting into dora nodes

The following sections use *robot_description* as an example to describe how to generate a dora node to implement the *robot_description* feature.

The *launch/lexus.launch* file in the original project shows that the original project is designed to first read the *lexus.urdf* file and then use the *joint_state_publisher* and *robot_state_publisher* ROS nodes to publish the connections and coordinate transformation data between different components of a vehicle. In actual scenarios, NDT uses only the coordinate transformation data between a vehicle and its lidar. Therefore, we need to only add a node named *tf_publisher* to publish the needed static coordinate transformation data. 

Objectives: Based on the foreign function interface (FFI) provided by [cxx](https://cxx.rs/), write the code of *tf_publisher* and compile the code to generate an executable file. 

Migration process: Confirm the definition and usage of the FFI --> Write C++ code and design the compilation logic --> Compile C++ code into a ROS 2 executable file named *tf_publisher_native* 

### Confirm the definition and usage of the FFI

**Definition of the FFI**

The dora framework provides the *dora-node-api-cxx* API that is used for data subscription and publishing. 

**Usage of the FFI in C++**

To use the defined FFI in C++, compile code to generate the *dora-node-api-cxx* static library first, and then copy the generated FFI files (i.e., *lib.rs.cc* and *lib.rs.h*) to the target folder, as shown in the following code in *run.rs*:

```rust
// dora/examples/autoware-dataflow/ndt_localizer/run.rs

#[tokio::main]
async fn main() -> eyre::Result<()> {
    // ...
    build_package("dora-node-api-cxx").await?;
    let cxxbridge = target.join("cxxbridge").join("dora-node-api-cxx").join("src");
    
    tokio::fs::copy(
        cxxbridge.join("lib.rs.cc"), 
        ros_src.join("tf_publisher").join("src").join("lib.rs.cc")
    ).await?;
    tokio::fs::copy(
        cxxbridge.join("lib.rs.h"), 
        ros_src.join("tf_publisher").join("include").join("tf_publisher").join("lib.rs.h")
    ).await?;
}
```

To enable the use of the FFI, configure settings in the *CMakeLists.txt* and *tf_publisher.cpp* files as follows.

Settings in the *CMakeLists.txt* file: Link the *dora_node_api_cxx* static library to the *tf_publisher_native* executable file to make and add a dependency on the *lib.rs.cc* FFI file when making the executable file.

```cmake
# dora/examples/autoware-dataflow/ndt_localizer/src/tf_publisher/CMakeLists.txt

# add the directory where the static library is located
string(REGEX REPLACE "(.+)/\\examples/autoware-dataflow/ndt_localizer.*" "\\1" lib_path ${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${lib_path}/target/debug)

ament_auto_add_executable(${PROJECT_NAME}_native
    src/tf_publisher.cpp
    src/lib.rs.cc
)

target_link_libraries(${PROJECT_NAME}_native
    dora_node_api_cxx -lrt
)
```

 Settings in the *tf_publisher.cpp* file: Include the *lib.rs.h* FFI header file.

```c++
// dora/examples/autoware-dataflow/ndt_localizer/src/tf_publisher/src/tf_publisher.cpp

#include "tf_publisher/lib.rs.h"
```

### Write C++ code and design the compilation logic

**Write code to implement the node feature**:  Implement the node feature in the *main* function in *tf_publisher.cpp*.

**Add ROS dependencies**: Add necessary ROS dependencies to *package.xml* and *CMakeLists.txt*.

```xml
<!-- dora/examples/autoware-dataflow/ndt_localizer/src/tf_publisher/package.xml -->

    <buildtool_depend>ament_cmake_auto</buildtool_depend>

    <depend>geometry_msgs</depend>
    <depend>tf2</depend>
    <depend>time_utils</depend>
```

```cmake
# dora/examples/autoware-dataflow/ndt_localizer/src/tf_publisher/CMakeLists.txt

find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
```

**Message serialization and deserialization**: For more information, see the following section.

### Implement message serialization and deserialization in C++

Message transmission under the dora framework uses middleware such as Zenoh, whereas ROS 2 nodes in the original project use Data Distribution Service (DDS). Therefore, during the migration from ROS 2 to dora, we need to manually implement message serialization and deserialization. [cereal](https://github.com/USCiLab/cereal) is an ideal tool to use, which requires only the relevant header files and no installation operations. Note that the type of shared pointer (*shared_ptr*) used in ROS 1 is `boost::shared_ptr`, but the member variables of this pointer type are private and cannot be serialized by using cereal. Therefore, code writing and compilation in C++ must use ROS 2.

To use cereal to implement message serialization and deserialization, copy the [relevant cereal header files](https://github.com/USCiLab/cereal/tree/master/include) to the *dora/examples/autoware-dataflow/ndt_localizer* directory first, and then include the header files in later coding.

The *tf_publisher* node uses the `geometry_msgs::msg::TransformStamped` message type in ROS 2, and the following manual operations are required.

**Add support for serialization and deserialization of the target message type**

In the *cereal_ros_msgs* file, add support for serialization and deserialization of the `geometry_msgs::msg::TransformStamped` message type.

```c++
// dora/examples/autoware-dataflow/ndt_localizer/src/cereal_ros_msgs/include/cereal_ros_msgs/cereal_geometry_msgs.hpp

#include "cereal/types/string.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/array.hpp"

#include "cereal_ros_msgs/cereal_std_msgs.hpp"

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace geometry_msgs::msg {
template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::Vector3& vector3)
{
    arch(vector3.x, vector3.y, vector3.z);
}
template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::Quaternion& quaternion)
{
    arch(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
}   
template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::Transform& trans)
{
    arch(trans.translation, trans.rotation);
}
template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::TransformStamped& trans_stamped)
{
    arch(trans_stamped.header, trans_stamped.child_frame_id, trans_stamped.transform);
}
}
```

**Configure the required settings in *tf_publisher***

- In *package.xml*, add a dependency on *cereal_ros_msgs*.

  ```xml
  <!-- dora/examples/autoware-dataflow/ndt_localizer/src/tf_publisher/package.xml -->
      <build_depend>cereal_ros_msgs</build_depend>
  ```

- Specify the path to the relevant cereal header files in *cmake_modules/find_cereal.cmake*, and then add the path to *CMakeLists.txt*.

  ```cmake
  # dora/examples/autoware-dataflow/ndt_localizer/src/tf_publisher/CMakeLists.txt
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/find_cereal.cmake)
  
  include_directories(
      ${CEREAL_DIRS}
  )
  ```

- In *tf_publisher.cpp*, include the relevant cereal header files and implement message serialization and deserialization.

  ```c++
  // dora/examples/autoware-dataflow/ndt_localizer/src/tf_publisher/src/tf_publisher.cpp
  
  #include <memory>
  #include <vector>
  
  // #include <cereal/archives/binary.hpp>
  #include "cereal/archives/portable_binary.hpp"
  #include "cereal/types/memory.hpp"
  #include <sstream>
  #include <cereal_ros_msgs/cereal_geometry_msgs.hpp>
  
  int main()
  {
      auto transform_ptr = std::make_shared<const geometry_msgs::msg::TransformStamped>(transform_msg);
      // seralize the msg to raw data (e.g. [u8] or Vec<u8>) as following
      std::stringstream ss; // any(in/out) stream can be used
      {
          cereal::PortableBinaryOutputArchive oarchive(ss); // Create an output archive
          oarchive(transform_ptr);                          // Write the data to the archive
      } // archive goes out of scope, ensuring all contents are flushed
  }
  ```

### Make other changes

During code rewriting, some custom settings are supported as needed, such as the links to system libraries and settings of compilation options. The following code provides an example:

```cmake
# dora/examples/autoware-dataflow/ndt_localizer/src/tf_publisher/CMakeLists.txt

set(CMAKE_CXX_FLAGS "-std=c++17 -pthread")

target_link_libraries(${PROJECT_NAME}_native
    dora_node_api_cxx -lrt
    dl
)
```

## Rewriting into dora operators

The following sections use *points_downsample* as an example to describe how to generate a dora operator to implement the *points_downsample* feature. The rewriting operations needed to generate a dora operator are more complex than that to generate a dora node.  

The original project code uses the following logic: After the raw point cloud data is imported, data cropping by distance is performed, and then the cropped data is published to NDT.

Objectives: Based on the FFI provided by [cxx](https://cxx.rs/), write the code of *voxel_grid_filter* and compile the code to generate a dynamic library. 

Migration process: Confirm the definition and usage of the FFI --> Write C++ code and design the compilation logic --> Write Rust code and design the compilation logic --> Compile Rust code into a Rust dynamic library named *voxel_grid_filter*

### Confirm the definition and usage of the FFI

This operation involves three aspects: definition of the FFI, usage of the FFI in C++, and usage of the FFI in Rust.

- Definition of the FFI

  The definition of the FFI is stored in the `ndt_localizer/src/voxel_grid_filter/src/ffi.rs` file. The file defines not only the *on_input* and *send_output* functions that are used for message subscription and publishing, but also the *VoxelGridFilterConfig* struct that is used for parameter passing. The parameters nested under *VoxelGridFilterConfig* are configured mainly based on the launch and config folders in the original ROS 2 project package.	

  ```rust
  // dora/examples/autoware-dataflow/ndt_localizer/src/voxel_grid_filter/src/ffi.rs
  
  #[cxx::bridge]
  #[allow(unsafe_op_in_unsafe_fn)]
  pub mod ffi {
      struct OnInputResult {
          error: String,
          stop: bool,
      }
      struct SendOutputResult {
          error: String,
      }
      struct VoxelGridFilterConfig {
          leaf_size: f32,
          output_log: bool,
      }
  
      extern "Rust" {
          type OutputSender<'a, 'b>;
          fn send_output(sender: &mut OutputSender, id: &str, data: &[u8]) -> SendOutputResult;
      }
  
      unsafe extern "C++" {
          include!("voxel_grid_filter/voxel_grid_filter.hpp");
  
          type VoxelGridFilter;
          fn new_operator(cfg: &VoxelGridFilterConfig) -> UniquePtr<VoxelGridFilter>;
  
          fn on_input(
              op: Pin<&mut VoxelGridFilter>,
              id: &str,
              data: &[u8],
              output_sender: &mut OutputSender,
          ) -> OnInputResult;
      }
  }
  ```

- Usage of the FFI in C++

  To use the defined FFI in C++, copy the generated FFI files (i.e., *ffi.rs.cc* and *ffi.rs.h*) and the cxxbridge header file (i.e., *cxx.h*) to the target folder, as shown in the following code in *build.rs*: 

  ```rust
  // dora/examples/autoware-dataflow/ndt_localizer/src/voxel_grid_filter/build.rs
  
  use std::{process::Command, fs, env, path::Path};
  
  fn main() {
      let mut _build = cxx_build::bridge("src/ffi.rs"); // returns a cc::Build
  
      fs::copy("../../../../../target/cxxbridge/voxel_grid_filter/src/ffi.rs.h", "include/voxel_grid_filter/ffi.rs.h").unwrap();
      fs::copy("../../../../../target/cxxbridge/voxel_grid_filter/src/ffi.rs.cc", "src/ffi.rs.cc").unwrap();
      fs::copy("../../../../../target/cxxbridge/rust/cxx.h", "include/voxel_grid_filter/cxx.h").unwrap();
  ```

  To enable the use of the FFI, configure settings in the *voxel_grid_fliter.cpp* and *CMakeLists.txt* files as follows.

  Settings in the *voxel_grid_filter.hpp* file: Include the *cxx.h* header file and declare the FFI type and interface.

  ```c++
  // dora/examples/autoware-dataflow/ndt_localizer/src/voxel_grid_filter/include/voxel_grid_filter/voxel_grid_filter.hpp
  
  #include "cxx.h"
  
  struct VoxelGridFilterConfig;
  
  class VoxelGridFilter
  {
  public:
      VoxelGridFilter(const VoxelGridFilterConfig &cfg);
  };
  
  std::unique_ptr<VoxelGridFilter> new_operator(const VoxelGridFilterConfig &cfg);
  
  struct OnInputResult;
  struct OutputSender;
  
  OnInputResult on_input(VoxelGridFilter &op, rust::Str, rust::Slice<const uint8_t>, OutputSender &output_sender);
  ```

  Settings in the *voxel_grid_fliter.cpp* file: Include the *ffi.rs.h* header file and implement *new_operator* and *on_input*.

  ```c++
  // dora/examples/autoware-dataflow/ndt_localizer/src/voxel_grid_filter/src/voxel_grid_filter.cpp
  
  #include "voxel_grid_filter/ffi.rs.h"
  
  std::unique_ptr<VoxelGridFilter> new_operator(const VoxelGridFilterConfig &cfg)
  {
      return std::make_unique<VoxelGridFilter>(cfg);
  }
  
  OnInputResult on_input(VoxelGridFilter &op, rust::Str id, rust::Slice<const uint8_t> data, OutputSender &output_sender)
  {
  }
  ```

  Settings in the *CMakeLists.txt* file: Link the *cxxbridge1* static library to the *voxel_grid_filter_native* ROS 2 dynamic library to make and add a dependency on the *ffi.rs.cc* FFI file when making the dynamic library.

  In this example, although the `cxxbridge1` static library is actually sourced from `dora/target/debug/build/cxx-99beedd1e9e35379/out/libcxxbridge1.a`, the path specified for the static library in *CMakeLists.txt* is *ndt_localizer/cxx_lib*. This is because the actual path contains a folder name that is randomly generated and cannot be specified. We need to copy the static library to *ndt_localizer/cxx_lib* and then link it to the ROS 2 dynamic library.

  ```cmake
  # dora/examples/autoware-dataflow/ndt_localizer/src/voxel_grid_filter/CMakeLists.txt
  
  # add the directory where the cxx static library is located
  string(REGEX REPLACE "(.+)/\\src/voxel_grid_filter" "\\1" cxx_lib_path_prefix ${CMAKE_CURRENT_SOURCE_DIR})
  link_directories(${cxx_lib_path_prefix}/cxx_lib)
  
  ament_auto_add_library(${PROJECT_NAME}_native SHARED
      src/voxel_grid_filter.cpp
      src/ffi.rs.cc
  )
  
  target_link_libraries(${PROJECT_NAME}_native
      cxxbridge1
  )
  ```

- Usage of the FFI in Rust

  The *lib.rs* file defines the usage of the FFI. Specifically, it encapsulates the VoxelGridFilter class in C++ and calls *on_input* defined in *ffi.rs* to process input data.

### Write C++ code and design the compilation logic

**Migration of the node feature**

The following information describes the code rewriting principle adopted for feature migration. You can compare the code in the original project with that in the current project to obtain more details. 

Retain the dependencies on ROS 2 message types (e.g., *sensor_msgs*) and auxiliary packages (e.g., *pcl_conversions*) and discard ROS 2 nodes, mechanism of parameter initial value passing, ROS2 launch files, and implementation of message subscription/publishing. Key points:

- The VoxelGridFilter class is no longer derived from rclcpp::Node. Parameter initial values are passed by using the `VoxelGridFilterConfig` struct defined in the FFI.

- In the original ROS 2 node design, input messages are processed by the callback function associated with each subscriber. In the current project, input messages are still processed by those callback functions. The only difference is that the callback functions are now explicitly invoked. We need to define the callback functions as public functions. Otherwise, they cannot be invoked by using `op.callback_scan(pc_ptr)` in *on_input*.
- In the original ROS 2 node design, messages are published by defining a publisher pointer and then calling `xxx_pub.publish(msg)` in a callback function. In the current project, only messages that need to be published are stored in variables, and message publishing is implemented in *on_input*.

**Message serialization and deserialization**

The process is the same as that for **rewriting into dora nodes**.

### Write Rust code and design the compilation logic

The *on_input* function in the *lib.rs* file actually invokes the *on_input* function implemented in C++. When building the target dynamic library *voxel_grid_filter* in Rust, we need to first compile the relevant C++ code into the *voxel_gird_filter_native* dynamic library and then link it to the *voxel_gird_filter* dynamic library, as shown in the following code in *build.rs*:

```rust
// dora/examples/autoware-dataflow/ndt_localizer/src/voxel_grid_filter/build.rs

use std::{process::Command, fs, env, path::Path};
use dunce;

fn main() {
    let mut _build = cxx_build::bridge("src/ffi.rs"); // returns a cc::Build

    let workspace = Path::new("../../");
    colcon_build(workspace);

    let lib_dir = dunce::canonicalize(workspace.join("install/voxel_grid_filter/lib")).unwrap();
    println!("cargo:rustc-link-lib=dylib=voxel_grid_filter_native");
    println!("cargo:rustc-link-search=native={}", env::join_paths(&[lib_dir]).unwrap().to_str().unwrap());
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
```

Then, write code in the `dora/examples/autoware-dataflow/ndt_localizer/src/voxel_grid_filter/cargo.toml` file.

### Compile code to generate the target dynamic library

The following information shows the complete process to compile code to generate the `voxel_gird_filter.so` dynamic library:

First, add the target package to the `dora/cargo.toml` file.

```rust
[workspace]
members = [
    "examples/autoware-dataflow/ndt_localizer/src/voxel_grid_filter",
]
```

Then, run the following commands to compile code. If the target dynamic library (*voxel_grid_filter*) fails to be generated by directly running `cargo build --package voxel_grid_filter`, run `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to voxel_gird_filter` to check whether an error occurs during the C++ code compilation process for generating *voxel_grid_filter_native*.

```bash
cd dora/examples/autoware-dataflow/ndt_localizer
source /opt/ros/galactic/setup.bash

# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to voxel_gird_filter

cargo build --package voxel_grid_filter
```





