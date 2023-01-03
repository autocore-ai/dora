

# 简要说明

ndt_localizer 以[简化版的ndt算法](https://github.com/AbangLZU/ndt_localizer)为例, 走通节点迁移和编译流程. 并添加必要的辅助软件包, 构成最小测试闭环.

节点迁移的改动主要涉及如下方面:

1. CMakeLists.txt, xxx.hpp, xxx.cpp 等源码改写; 

2. Dora接口使用, 根据源码中的 launch 和 config 文件定义ffi.rs;

3. C++ 侧消息的序列化和反序列化;

4. C++ 和 Rust 混合编译流程编写; (遗留对ros动态库的依赖, 留待后续解决)

5. 添加辅助节点, 保证构成最小闭环, 包括手动发送坐标转换信息.

具体改动内容可以对比原始工程和本目录下代码. 需要特别说明的是原始工程是ros1代码, 共享指针(shared_ptr)使用的是 boost::shared_ptr, 但 boost 指针的成员变量都是私有的, 没法用 cereal 进行序列化, 所以要改成ros2的代码.

# 使用方式

编译和使用方式如下

```bash
# 编译
cd dora/examples/autoware-dataflow/ndt_localizer
source /opt/ros/galactic/setup.bash
cargo run --example ntd-dataflow
# 运行
source install/setup.bash
../../../target/debug/dora-coordinator --run-dataflow dataflow.yml --runtime ../../../target/debug/dora-runtime
```

本来通过 `cargo run --example ntd-dataflow` 就可以直接运行程序了, 但由于没法在脚本 *examples/autoware-dataflow/ndt_localizer/run.rs* 里有效地调用 `source install/setup.bash` 命令, 所有才需要手动 source 位于 *install* 目录下安装好的 ROS2 动态库和可执行程序, 然后才能通过 dora-coordinator 和 dora-runtime 将程序运行起来.

# 详细改写说明

通过对原仓库的代码进行分析可知, 其代码按功能可划分为 *map_loader*,  *robot_description*, *points_downsample* 和 *ndt_localizer* 四个主要的子功能. 根据它们之间的输入输出关系, 可以将  *map_loader*,  *robot_description* 改写成 dora node; 将 *points_downsample*, *ndt_localizer* 改写成 dora operator. 

同时为了构成最小闭环来测试代码功能, 还需要添加: 初始位置设置节点 (*pose_initializer*), 模拟用rosbag播放点云数据的节点 (rosbag_dummy), 时间戳类型转换节点 (time_utils)  等辅助节点.

## 改写成dora node

以 *robot_description* 为例, 介绍如何用 dora node 实现其功能.

由 *launch/lexus.launch* 文件可知, 原来的代码逻辑是读取 *lexus.urdf* 文件, 然后通过ROS的 *joint_state_publisher* 和 *robot_state_publisher* 节点发布车辆各组件之间的连接和坐标变换关系. 实际上 ndt 功能真正需要用到的只有车辆和激光雷达之间的坐标变换关系, 所以添加 *tf_publisher* 节点, 来发送所需的静态坐标关系即可. 

任务目标: 基于[cxx](https://cxx.rs/) 提供的 FFI 接口, 编写 *tf_publisher* 功能代码, 并将其编译编译成可执行文件. 

移植思路: FFI接口的定义和使用 --> 编写C++侧代码和编译逻辑 --> 将C++侧代码编译成名为 *tf_publisher_native* 的 ROS2 可执行文件. 

### FFI 接口的定义和使用

**FFI 接口的定义**

Dora 框架的 *dora-node-api-cxx* 已经提供了数据订阅和发布的 api 接口, 直接使用即可. 

**C++ 侧 FFI 接口的使用**

为了在 C++ 侧使用所定义的 FFI 接口, 需要先编译生成 *dora-node-api-cxx* 静态库, 然后拷贝生成的 FFI 文件( *lib.rs.cc*,  *lib.rs.h*) 到目标文件夹, 参见 *run.rs* 代码.

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

具体的接口使用, 需要在 *CMakeLists.txt* 和 *tf_publisher.cpp* 进行相关设置.

*CMakeLists.txt* 中的设置包括: 将静态库 (*dora_node_api_cxx*) 链接到要构建的可执行文件 (*tf_publisher_native*) ; 构建可执行文件时添加关于FFI (*lib.rs.cc*) 的依赖.

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

 *tf_publisher.cpp* 中的设置主要是包含FFI头文件(lib.rs.h).

```c++
// dora/examples/autoware-dataflow/ndt_localizer/src/tf_publisher/src/tf_publisher.cpp

#include "tf_publisher/lib.rs.h"
```

### 编写C++侧代码和编译逻辑

**编写代码实现节点功能**:  在 *tf_publisher.cpp* 的 main函数中实现节点功能.

**添加ROS依赖**: 在 *package.xml* 和 *CMakeLists.txt* 中添加必要的 ROS 依赖.

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

**消息序列化和反序列化**: 参见下节完整说明.

### C++侧消息的序列化和反序列化

在 dora 框架消息传输使用的时 zenoh 等中间件, 而原来ROS2节点使用的是DDS, 所以在将ROS2节点往dora移植的过程中, 需要手动进行消息的序列化和反序列化, 而 [cereal](https://github.com/USCiLab/cereal) 是个不错的工具且使用比较方便 (免安装, 包含相应的头文件即可使用), 需要特别说明的是 ROS1里共享指针 (*shared_ptr*) 使用的是 `boost::shared_ptr`, 但 boost 指针的成员变量都是私有的, 没法用 cereal 进行序列化, 所以C++侧的代码和编译方式都得是 ROS2 方式才行.

首先将 [cereal 相关头文件](https://github.com/USCiLab/cereal/tree/master/include), 拷贝到 *dora/examples/autoware-dataflow/ndt_localizer* 目录, 后续用 cereal 进行消息序列化和反序列化, 只需要包含相应头文件即可.

这里 *tf_publisher* 节点需要使用 ROS2中的 `geometry_msgs::msg::TransformStamped` 消息类型, 主要涉及如下几方面工作

**目标消息类型序列化支持**

在 *cereal_ros_msgs* 软件包中添加对于 `geometry_msgs::msg::TransformStamped`消息类型序列化和反序列化的支持

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

**在 *tf_publisher* 进行必要的设置**

- 在 *package.xml* 中添加关于 *cereal_ros_msgs* 的依赖;

  ```xml
  <!-- dora/examples/autoware-dataflow/ndt_localizer/src/tf_publisher/package.xml -->
      <build_depend>cereal_ros_msgs</build_depend>
  ```

- 在*cmake_modules/find_cereal.cmake* 中指明 cereal 头文件路径, 并在 *CMakeLists.txt* 将路径添加进来.

  ```cmake
  # dora/examples/autoware-dataflow/ndt_localizer/src/tf_publisher/CMakeLists.txt
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/find_cereal.cmake)
  
  include_directories(
      ${CEREAL_DIRS}
  )
  ```

- 在 *tf_publisher.cpp* 引入cereal, 并对消息进行序列化和反序列化

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

### 其他改动

改写过程中还涉及的一些细节问题: 比如系统库的链接, 编译选项的一些设置等, 如有需要, 可按需设置和添加.

```cmake
# dora/examples/autoware-dataflow/ndt_localizer/src/tf_publisher/CMakeLists.txt

set(CMAKE_CXX_FLAGS "-std=c++17 -pthread")

target_link_libraries(${PROJECT_NAME}_native
    dora_node_api_cxx -lrt
    dl
)
```



## 改写成 dora operator

以 *points_downsample* 为例, 介绍如何用 dora operator 实现其功能, 这里改写的逻辑比 dora node 要复杂.  

原来的代码逻辑是输入原始点云, 按距离进行裁减后, 发布裁减后的点云数据给 ndt 使用.

任务目标: 基于[cxx](https://cxx.rs/) 提供的 FFI 接口, 编写 *voxel_grid_filter* 功能代码 , 并将其编译编译成动态库文件. 

移植思路: 定义FFI接口 --> 编写C++侧代码和编译逻辑 --> 编写 Rust 侧代码和编译逻辑 --> 将Rust侧代码编译成名为 *voxel_grid_filter* 的 Rust 动态库.

### FFI接口的定义与使用

分为三部分内容: FFI 接口的定义, FFI 接口在 C++ 侧的使用, FFI 接口在 Rust 侧的使用.

- FFI 接口的定义

  位于`ndt_localizer/src/voxel_grid_filter/src/ffi.rs`, 除了 *on_input* 和 *send_output* 这两个用于消息订阅和发送的接口之外, 还定义了用于传参的 *VoxelGridFilterConfig* 结构体, 该结构体的字段主要根据原 ROS2 软件包 launch 和 config 子文件夹中的内容进行设置.	

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

-  FFI 接口在 C++ 侧的使用

  为了在 C++ 侧使用所定义的 FFI 接口, 需要拷贝生成的 FFI 文件( *ffi.rs.cc*,  *ffi.rs.h*) 和 cxxbridge的头文件 (*cxx.h*) 到目标文件夹, 参见 *build.rs* 中的代码内容. 

  ```rust
  // dora/examples/autoware-dataflow/ndt_localizer/src/voxel_grid_filter/build.rs
  
  use std::{process::Command, fs, env, path::Path};
  
  fn main() {
      let mut _build = cxx_build::bridge("src/ffi.rs"); // returns a cc::Build
  
      fs::copy("../../../../../target/cxxbridge/voxel_grid_filter/src/ffi.rs.h", "include/voxel_grid_filter/ffi.rs.h").unwrap();
      fs::copy("../../../../../target/cxxbridge/voxel_grid_filter/src/ffi.rs.cc", "src/ffi.rs.cc").unwrap();
      fs::copy("../../../../../target/cxxbridge/rust/cxx.h", "include/voxel_grid_filter/cxx.h").unwrap();
  ```

  具体接口的使用, 需要在 *voxel_grid_filter.hpp*, *voxel_grid_fliter.cpp* 和 *CMakeLists.txt* 进行相关设置.

  在 *voxel_grid_filter.hpp* 中的设置包括: 包含*cxx.h* 头文件, FFI 类型和接口的声明.

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

  *voxel_grid_fliter.cpp* 中的设置: 包含 *ffi.rs.h* 头文件, 以及 *new_operator* 和 *on_input* 的实现.

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

  *CMakeLists.txt* 中的设置包括: 将静态库 (*cxxbridge1*) 链接到要构建的ROS2动态库 (*voxel_grid_filter_native*) ; 构建动态库时添加关于FFI (*ffi.rs.cc*) 的依赖.

  需要说明的是, 这里静态库`cxxbridge1` 路径设置的是 *ndt_localizer/cxx_lib*, 但源文件实际来自于`dora/target/debug/build/cxx-99beedd1e9e35379/out/libcxxbridge1.a`, 因为该目录不方便指定, 所以将才将其拷贝到 *ndt_localizer/cxx_lib* 目录以便将其链接到ROS2动态库.

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

- FFI 接口在 Rust 侧的使用

  *lib.rs* 的主要工作即为 对C++侧 VoxelGridFilter 类进行了封装, 并负责调用 *ffi.rs* 中所定义*on_input* 接口来处理输入数据.

### 编写C++侧代码和编译逻辑

**节点功能的迁移**

可通过与原始仓库代码进行对比, 获得具体改动内容. 

这里只对代码改写的原则进行必要说明: 除了保留关于ROS2消息类型 (比如 *sensor_msgs*) 和辅助功能包 (比如 *pcl_conversions*) 的依赖之外, 取消ROS2 node, 参数初值传递机制, ROS2 launch, Pub/Sub等相关的功能的使用. 要点如下

- class VoxelGridFilter 不再从rclcpp::Node派生, 参数初值通过 FFI 中定义的 `VoxelGridFilterConfig` 结构体来传递.

- 原来的ROS2节点, 输入消息的处理是由与subscriber绑定的回调函数负责, 现在输入消息的具体处理依然由各回调函数负责, 只不过要显式调用而已, 这部分内容没有大的变化. 但需要将原来的回调函数定义为公有 (public) 函数, 否则无法在 *on_input* 函数中通过 `op.callback_scan(pc_ptr)`方式进行调用.
- 原来的ROS2节点, 发布消息是通过定义publisher指针并在回到函数中调用`xxx_pub.publish(msg)`进行发送, 现在在回调函数中只将需要发送的消息保存在变量中, 而发布消息的具体过程在 *on_input* 函数中完成.

**消息的序列化和反序列化**

与 `改写成 dora node` 中的设置过程相同.

### 编写Rust侧代码和编译逻辑

从 *lib.rs* 代码具体内容可知, *on_input* 函数实际调用的是 C++侧所实现的 *on_input* 函数. 所以在构建 Rust 侧的目标动态库 (*voxel_grid_filter*) 时, 需要先将c++侧的代码编译成 *voxel_gird_filter_native*动态库, 然后将该动态库链接到 *voxel_gird_filter* 动态库中, 参见 *build.rs* 中的内容.

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

然后就是编写 `dora/examples/autoware-dataflow/ndt_localizer/src/voxel_grid_filter/cargo.toml`的内容.

### 编译生成目标动态库文件

完整编译生成 `voxel_gird_filter.so`动态库的 过程如下:

先在 `dora/cargo.toml` 添加目标package:

```rust
[workspace]
members = [
    "examples/autoware-dataflow/ndt_localizer/src/voxel_grid_filter",
]
```

执行下面的编译过程: 如果运行 `cargo build --package voxel_grid_filter` 直接生成目标动态库 (*voxel_grid_filter*) 失败, 那么可以执行命令 `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to voxel_gird_filter` 排查将C++侧的代码编译成 *voxel_grid_filter_native* 动态库的过程中是否存在错误.

```bash
cd dora/examples/autoware-dataflow/ndt_localizer
source /opt/ros/galactic/setup.bash

# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to voxel_gird_filter

cargo build --package voxel_grid_filter
```





