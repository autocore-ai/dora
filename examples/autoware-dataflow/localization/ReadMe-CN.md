# 简要说明
将 [autoware.universe](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f) 中定位模块完整迁移过来, 构建完整测试闭环. universe的代码基线是基于2022.09.16的版本, 有一定滞后, 但autoware定位模块的代码功能基本改动不大.

在以ntd为例走通流程基础上, 迁移universe版本定位模块代码额外涉及如下工作:

1. 定位模块所涉及辅助节点的分析, 剥离非必要节点;

2. ROS 中的 Service 机制在 dora 中尚未实现, 目前临时采用 pub/sub 的方式来替换 Sevice 的方案. 但这破坏了原来 Service 的异步调用机制, 可能存在性能瓶颈;

3. 原本 ROS 中提供的 坐标变换(TF)查询和管理机制在 dora 尚未实现, 目前采用的替代办法是, 专门新增一个`tf_publisher`节点, 用于对外发布定位模块运行所需的静态坐标变换信息.

4. 点云预处理模块的非必要功能的剥离和迁移;

5. ROS中存在将多个节点放在一个Container 的机制, 如何对其进行拆分为多个Dora Operator

6. 尽可能剥离 autoware 自定义消息类型.

7. *ndt_scan_matcher*里使用了多线程机制 (`rclcpp::executors::MultiThreadedExecutor` 和 `callback_group`), 这在dora中如何实现? 另外为加快运行速度 *ndt_scan_matcher* 采用的是多核版本的ndt算法实现, 往dora框架迁移要作哪些改动呢? 

8. `dataflow.yml`里描述的 `node/operator`启动顺序是否有讲究(应该没有)? 进一步的问题是, ROS2的Qos可以设置缓存队列的大小, 而dora中目前缺乏相应机制, 这会导致缓存很多已经失效的消息.

**关于如何将 ROS2 软件包改写成 dora node 或者 dora operator 的步骤, 请参考 `examples/autoware-dataflow/ndt_localizer/ReadMe.md` 中的说明.**

# 使用方式

编译和使用方式如下

```bash
# 编译
cd dora/examples/autoware-dataflow/localization
source /opt/ros/galactic/setup.bash
cargo run --example localizaton-dataflow
# 运行
source install/setup.bash
../../../target/debug/dora-coordinator --run-dataflow dataflow.yml --runtime ../../../target/debug/dora-runtime
```

本来通过 `cargo run --example localization-dataflow` 就可以直接运行程序了, 但由于没法在脚本 *examples/autoware-dataflow/localization/run.rs* 里有效地调用 `source install/setup.bash` 命令, 所有才需要手动 source 位于 *install* 目录下安装好的 ROS2 动态库和可执行程序, 然后才能通过 dora-coordinator 和 dora-runtime 将程序运行起来.

# 软件包改写说明

**auxiliary中的包 (纯ros节点): **

- `cereal_ros_msgs`,  新增, 用于对涉及的ROS消息类型进行序列化和反序列化.

- `autoware_cmake` 来自 [autoware_common/autoware_cmake](https://github.com/autowarefoundation/autoware_common/tree/main/autoware_cmake), 未改动, 作用是提供公共的CMake脚本.

- `autoware_lint_common` 来自 [autoware_common/autoware_lint_common](https://github.com/autowarefoundation/autoware_common/tree/main/autoware_lint_common), 未改动, 作为`ament_lint_common`包的定制化版本, 一般在测试ROS包的功能时使用, 在本工程中实际不需要.

- `time_utils` 来自 [autoware.universe/common/time_utils](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/common/time_utils), 未改动, 提供`std::chrono::time_point`和`builtin_interfaces::msg `类型之间的转换接口.

- `tier4_debug_msgs` 来自 [external/tier4_autoware_msgs/tier4_debug_msgs](https://github.com/tier4/tier4_autoware_msgs/tree/tier4/universe/tier4_debug_msgs), 未改动, 用于调试信息的输出, 比std信息多了时间戳信息.

- `tier4_autoware_utils` 来自  [autoware.universe/common/tier4_autoware_utils](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/common/tier4_autoware_utils), 做了简化, 只保留定位模块需要用到的功能. 简化后有两点好处, 一是避免了对 *autoware_auto_perception_msgs*, *autoware_auto_planning_msgs* 的依赖, 二是删除非必要功能后, 无需将本包改写为 dora node.

  > 删除 test 子目录. 
  >
  > 删除 *geometry* 子目录的 `boost_polygon_utils.hpp` 和 `boost_polygon_utils.cpp` 两个文件, 它们只在感知部分需要使用.
  >
  > 删除 *geometry* 子目录的 `path_with_lane_id_geometry.hpp` 文件, 删除`geometry.hpp` 中涉及 `autoware_auto_planning_msgs` 的内容, 它们只在规划部分需要使用.
  >
  > 删各子目录中涉及pub/sub的hpp文件, 既然定位模块用不到它们, 删除后就无需将该包改为dora node了.
  >
  > 相应修改`package.xml`, `CMakeLists.txt`, hpp和cpp等源文件.

- `geo_pos_conv` 来自 [autoware.universe/sensing/geo_pos_conv](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/sensing/geo_pos_conv), 删除对`rclcpp`的依赖, 用于实现gnss原始经纬高坐标和笛卡尔(东北天)坐标之间的转换.
- `autoware_sensing_msgs` 来自 [autoware_msgs/autoware_sensing_msgs](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_sensing_msgs), 未改动. 引入原因是`gnss_poser` 要用到, 通过删除 `gnss_poser` 中用于处理 "gnss orientation"话题的相关代码, 可取消对该包的依赖和引入.
- `ndt_pcl_modified` 来自 [autoware.universe/localization/ndt_pcl_modified](https://github.com/autowarefoundation/autoware.universe/tree/main/localization/ndt_pcl_modified), 未改动, 在*ndt_pcl* 实现基础上, 略作修改的 ndt 实现版本.
- `ndt_omp` 来自 [external/ndt_omp](https://github.com/tier4/ndt_omp), 未改动, ndt算法的多核实现版本.

- `ndt` 来自 [autoware.universe/localization/ndt](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/localization/ndt), 未改动, 提供不同 ndt 实现版本的通用接口.
- `kalman_filter` 来自 [autoware.universe/common/kalman_filter](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/common/kalman_filter), 未改动, kalman滤波器的实现.

**nodes中的包 (改写和新增的dora node):**

下面所说的**必要改动**是指将一个ROS2软件包改写成 dora node 所需的改动.

- `point_map_loader` 来自 [autoware.universe/map/pointcloud_map_loader](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/map/map_loader), 必要改动, 用于点云地图加载.
- `gnss_imu_dummy`, 新增, 用于模拟组合导航驱动程序, 对外发布 "gnss"+"imu"+"orientation"数据.
- `rosbag_dummy`, 新增, 模拟用 rosbag 播放激光雷达点云数据的功能. 
- `manual_pose`, 新增, 用于手动指定车辆初始位置. 初始位置与`rosbag_dummy`中所播放的点云数据要匹配.
- `tf_publisher`, 新增, 用于发布其他节点工作所需的静态坐标变换信息. 
- `vehicle_twist`, 新增, 用于模拟车辆线控程序, 对外发布车辆速度信息.

**operators 中的包 (改写的dora operator):**

下面所说的**必要改动**是指将一个ROS2软件包改写成 dora operator 所需的改动.

- `gnss_poser` 来自 [autoware.universe/sensing/gnss_poser](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/sensing/gnss_poser), 改动较大. 作为原始gnss信号的前处理节点, 作用是将 gnss 信号转换成 map 坐标系下的坐标. "必要改动"之外的修改主要有: 

  > 对输出 topic 进行简化: 只输出`gnss/pose_with_cov`, 取消其它不必要的输出.
  >
  > 取消 `RCLCPP_COMPONENTS_REGISTER_NODE` 操作和对`rclcpp_components`的依赖, 因为在 dora 框架中不再使用 ROS2 的component 机制, 相应地就无需定义 "gnss_poser"等命名空间.
  >
  > 删除`convert.hpp`中关于 `rclcpp::Logger`的使用, 打印信息改用std提供的标准打印, 这样可取消对`rclcpp`的依赖.
  >
  > 可通过删除用于处理 "gnss orientation"的相关代码, 来取消对 `autoware_sensing_msgs` 的依赖.

- `imu_corrector` 来自 [autoware.universe/sensing/imu_corrector](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/sensing/imu_corrector), 必要改动. 原始 imu 信号的前处理节点, 作用是为 imu 信号添加协方差信息.  "必要改动" 之外的修改主要在于, 取消用于向ROS2注册节点`RCLCPP_COMPONENTS_REGISTER_NODE`操作.

- `pose_initializer` 来自 [autoware.universe/localization/pose_initializer](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/localization/pose_initializer), 改动很大. 根据`pose_initializer_core.cpp` 可知原来的代码逻辑是: 获取手拖的初始位置或者gnss给出的初始位置 (gnss所给的初始位置需要经过`map_height_fitter`处理以便补充高度信息), 然后经过`ndt_align`服务 (如果启用的话) 进行处理, 得到较为准确的初始位置估计, 最后将该位置估计和初始化状态`state_`一同发布. `pose_initializer` 的调用由 `automatic_pose_initializer` 包触发. 所以根据上述逻辑, 同时考虑到 dora 目前没有提供类似 ROS Service的机制, 需要将`pose_initializer`的功能拆分为如下几个部分: 发送手拖初始位置的node; 对 gnss 消息 (由`gnss_poser`对原始 gnss 信息进行处理后发出) 进行`map_height_fliter`处理的operator; 接收手拖初始位置和经过`map_height_fitter`处理后gnss信号, 并向`ntd_align`发送服务请求的operator; 最后在`ndt_align`的 服务(service) 端将较为准确的初始位置估计对返回给请求 (client) 端. 除了"必要改动", 具体代码修改还涉及如下内容

  > 将`gnss_module.cpp`和`ndt_module.cpp`中原来抛出`ServiceException`异常的地方, 统一改为抛出 `std::runtime_error`异常, 如此可避免对 `component_interface_utils`的依赖.
  >
  > 删除`stop_check`模块, 使用该模块的目的是只对处于停止状态的车辆进行位置初始化. 删除此模块可避免对`motion_utils`及其依赖包的依赖.
  >
  > 取消发布 `state_`变量, 初始化部分不再调用`automatic_pose_initializer`节点. 主要原因有两点, 一是调用该节点额外涉及很多依赖包; 二是替换`automatic_pose_initializer`节点中涉及的 Service 调用和 Timer调用等功能比较麻烦. 重新定义枚举变量 `InitializationState` 用于记录位置初始化状态.

- `map_height_fitter` 来自 [autoware.universe/localization/pose_initializer](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/localization/pose_initializer), 改动较大. 作用是订阅 map 和 gnss 消息以便给 gnss 消息的添加高度信息. 改动后的内容整合了`map_height_fitter`子目录和`gnss_module.cpp`中的逻辑.

- `gyro_odometer` 来自 [autoware.universe/localization/gyro_odometer](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/localization/gyro_odometer), 必要改动. 作用是综合来自 imu 角速度信息和车辆线控给出的线速度信息, 然后发布车辆速度的观测值.

- `pointcloud_downsample` 来自 [autoware.universe/sensing/pointcloud_preprocessor](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/sensing/pointcloud_preprocessor), 原始点云输入ndt进行配准前的降采样模块, 改动较大. 原代码中`pointcloud_preprocessor`有很多功能, 涉及C++类的继承, 基类本身就比较复杂且原有架构是按ROS2 Component 设计的. 既然定位模块只用到`crop_box_filter`, `voxel_grid_downsample_filter`, `random_downsample_filter`三个功能, 这里保留原来的基本框架 , 删繁就简, 构建面向定位模块的点云降采样operator.

  > 这里取消了基类`filter`和不同子 filter 之间之间的继承关系, 主要原因有两方面. 
  >
  > 其一, 在dora中不再需要使用ROS2 Component机制;
  >
  > 其二, 与Rust混合编程时, 需要从外部调用`crop_box_filter`等的"filter"函数, 所以没法再在基类中定义纯虚函数"filter", 且也不能将"filter"函数设置为protected属性. 
  >
  > 构建面向定位模块的点云降采样节点, `crop_box_filter`, `voxel_grid_downsample_filter`, `random_downsample_filter`三个滤波器是顺序执行的关系, 正好可取消对原来基类中`mutex_`互斥变量的使用. 

- `ndt_scan_matcher` 来自 [autoware.universe/localization/ndt_scan_matcher](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/localization/ndt_scan_matcher), 执行ndt配准算法同时向`pose_initializer`提供`ndt_align`服务, 改动较大. 

  > 删除定时诊断功能函数 timerDiagnostic(), 它虽然是独立线程, 但其依赖`ndt_scan_matcher`中定义的状态, 无法剥离成独立dora node, 故删除此功能, 还可相应地取消对`diagnostic_msgs`的依赖.
  >
  > 修改`ndt_align`服务相关的代码, 即修改 serviceNDTAlign() 和 alignUsingMonteCarlo() 两个函数, 取消对`tier4_localization_msgs`的依赖, 并整合原来`pose_initializer`中`ndt_module.cpp`中的逻辑.
  >
  > 对输出topic进行简化: 只输出`pose_with_cov`和返回给`pose_initalizer`的`ndt_align`响应信息, 取消其他不必要的输出信息.
  >
  > 取消多线程 (`rclcpp::executors::MultiThreadedExecutor` 和 `callback_group`) 机制的使用.

- `ekf_localizer` 来自 [autoware.universe/localization/ekf_localizer](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/localization/ekf_localizer), 改动较大. 主要作用是对`ndt_scan_matcher`输出的位置进行滤波, 获得想对平滑的定位结果.

  > ekf_localizer 需要定时的发送滤波结果和map2baselink的坐标变换. 原来的定时机制是通过rclcpp::create_timer()根据传递进来的`ekf_rate_`和`tf_rate_`参数进行设置. 在dora框架中需要通过`dora/timer/millis/20` 来触发, 但初始参数还是要传, 这就要求初始参数`ekf_rate_`的设置与定时触发机制的间隔保持一致.
  >
  > 删除 `warning.hpp` 以及调用它的代码, 直接改用 std 输出打印信息. 同时调整其他打印信息的格式, 不再使用RCLCPP宏以便取消对rclcpp的依赖, 但`RCLCPP_WARN_THROTTLE`设置一段时间内只打印一次信息的功能没有好的替代办法, 只能也用std来近似替代.
  >
  > 将原来使用`rclcpp::time`的地方全部改用`std::chrono::system_clock::time_point`, 从而避免使用rclcpp::node->get_clock()->now() 等操作.
  >
  > 对输出topic进行简化, 取消不必要的输出信息.

- stop_filter 来自 [autoware.universe/localization/stop_filter](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/localization/stop_filter), 必要改动.
