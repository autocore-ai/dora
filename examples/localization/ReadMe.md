Notice: At present, the document is written in Chinese, and will be changed to English later

## 简要说明
将[autoware.universe](8e92d176721e71943d7dff0722365cb9ee97379f) 中定位模块完整迁移过来, 构建完整测试闭环. universe的代码基线是基于2022.09.16的版本, 有一定滞后, 但autoware定位模块的代码功能基本改动不大.

在以ntd为例走通流程基础上, 迁移universe版本定位模块代码额外涉及如下工作:

1. 完整定位模块所涉及辅助节点的分析, 剥离非必要节点;

2. ROS 中涉及的 SRV 机制如何在 Dora 中实现, 目前方案请求和回复都用topic来替代;

3. 点云预处理模块的非必要功能的剥离和迁移;

4. ROS中存在将多个节点放在一个Container 的机制, 如何对其进行拆分为多个Dora Operator

5. 尽可能剥离 autoware 自定义消息类型.

目前还有ekf_localizer及其依赖包没有迁移, 已经完成迁移的包也未测试. 所以本目录下具体内容将在两天后上传.

## 软件包改写说明

**auxiliary中的包 (纯ros节点): **

- cereal_ros_msgs 新增, 用于对涉及的消息进行序列化.

- autoware_cmake 来自 "src/core/autoware_common/autoware_cmake", 未改动.

- autoware_lint_common 来自 "src/core/autoware_common/autoware_lint_common", 未改动

- ndt_pcl_modified 来自 "src/universe/autoware.universe/localization/ndt_pcl_modified", 未改动

- ndt_omp 来自 src/universe/external/ndt_omp", 未改动

- ndt 来自  "src/universe/autoware.universe/localization/ndt", 未改动

- time_utils 来自 "src/universe/autoware.universe/common/time_utils", 未改动

- tier4_debug_msgs 来自 "src/universe/external/tier4_autoware_msgs/tier4_debug_msgs", 未改动, 用于调试信息的输出, 比std信息多了时间戳信息 --> cereal序列化待做

- tier4_autoware_utils 来自  "src/universe/autoware.universe/common/tier4_autoware_utils", 做了简化, 简化目的有两方面, 一方面避免引入对 autoware_auto_perception_msgs, autoware_auto_planning_msgs 的依赖, 另外一方面为了避免将 tier4_autoware_utils 改写为 dora node, , 具体如下(实际相当于只保留定位模块中需要用到的功能)

  > 删除 test 子目录. 
  >
  > 删除 geometry 子目录的 boost_polygon_utils.hpp 和 boost_polygon_utils.cpp 两个文件, 它们只在感知部分需要使用
  >
  > 删除 geometry 子目录的 path_with_lane_id_geometry.hpp 文件, 以及 geometry.hpp 中涉及 autoware_auto_planning_msgs 的内容, 它们只在规划部分使用
  >
  > 删各子目录中涉及pub/sub的hpp文件, 因为既然定位模块用不到它们, 删除后就无需将该包改为dora node了.
  >
  > 相应修改package.xml, CMakeLists.txt, hpp和cpp等源文件.

**nodes中的包 (迁移改造为dora node):**

- point_map_loader 来自 "src/universe/autoware.universe/map/pointcloud_map_loader", 必要改动. 

**operators 中的包 (迁移改造为dora operators):**

- stop_filter 来自 "src/universe/autoware.universe/localization/stop_filter", 必要改动

- gyro_odometer 来自 "src/universe/autoware.universe/localization/gyro_meter", 必要改动

- ndt_scan_matcher 来自 "src/universe/autoware.universe/localization/ndt_scan_matcher", 改动较大, 主要目的: 取消srv相关代码并改写为独立dora节点以备用, 删除timerDiagnostic定时诊断器.

  > 删除定时诊断功能函数 timerDiagnostic(), 因其依赖ndt_scan_matcher中定义的状态, 无法剥离成独立dora node, 故删除此功能, 相应地取消对diagnostic_msgs的依赖.
  >
  > 取消srv相关代码, 即删除 serviceNDTAlign() 和 alignUsingMonteCarlo()两个函数, 剥离此功能改成独立dora node以备用, 相应地可取消对 tier4_localization_msgs 的依赖.
