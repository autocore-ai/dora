# Brief description

The entire location module of [autoware.universe](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f) will be migrated here to build a closed test loop. The code baseline of autoware.universe is based on the version of 2022.09.16, which may lag behind the mainstream. However, the code functionality of the location module is largely unchanged.

In addition to streamlining the workflow based on Normal Distributions Transform (NDT), migrating the code of the location module of autoware.universe involves the following work:

1. Analyzing the auxiliary nodes in the location module and stripping out unnecessary nodes.

2. Implementing the pub/sub communication mechanism. This is a temporary substitute for ROS 2 Service, which has not yet been implemented in dora. Note that this solution does not support the asynchronous invocation mechanism of ROS 2 Service and may have a performance bottleneck.

3. Adding the `tf_publisher` node to externally publish the static coordinate transform information needed for the running of the location module. This is a temporary substitute for the coordinate transform (TF) query and management functions of ROS 2, which have not yet been implemented in dora.

4. Stripping out and migrating unnecessary functions of the point cloud preprocessing module.

5. Separating the nodes in one container into multiple dora operators (A container in ROS 2 may contain multiple nodes).

6. Stripping out the message types customized by Autoware as many as possible.

7. Figuring out how to implement the multithread mechanism (`rclcpp::executors::MultiThreadedExecutor` and `callback_group`) of *ndt_scan_matcher* in dora, and how to migrate the multi-core ndt algorithm used by *ndt_scan_matcher* for accelerating running to the dora framework. 

8. Figuring out what is the order for starting `node/operator` in `dataflow.yml`, and how to specify the length of the buffer queue. ROS 2 provides QoS for specifying the buffer queue length. Currently, dora does not limit the buffer queue length, which may lead to many invalid messages in the buffer.

**For more information about how to rewrite ROS 2 packages as dora nodes or dora operators, see `examples/autoware-dataflow/ndt_localizer/ReadMe.md`.**

# Usage

Use the following syntax to compile code and run localization:

```bash
# Compile code
cd dora/examples/autoware-dataflow/localization
source /opt/ros/galactic/setup.bash
cargo run --example localization-dataflow
# Run localization
source install/setup.bash
../../../target/debug/dora-coordinator --run-dataflow dataflow.yml --runtime ../../../target/debug/dora-runtime
```

The `source install/setup.bash`command cannot be invoked in the *examples/autoware-dataflow/localization/run.rs* script. As a result, after running `cargo run --example localization-dataflow`, you need to run the `source` command on the ROS 2 dynamic library and executables installed in the *install* directory, and then use dora-coordinator and dora-runtime to start localization.

# Package rewriting instructions

**Packages in the auxiliary directory (pure ROS 2 nodes): **

- `cereal_ros_msgs`: newly added. This package serves to serialize and deserialize ROS 2 message types.

- `autoware_cmake` from [autoware_common/autoware_cmake](https://github.com/autowarefoundation/autoware_common/tree/main/autoware_cmake): unchanged. This package serves to provide public CMake scripts.

- `autoware_lint_common` from [autoware_common/autoware_lint_common](https://github.com/autowarefoundation/autoware_common/tree/main/autoware_lint_common): unchanged. As a custom version of the `ament_lint_common` package, this package is used to test the functionality of the ROS 2 package, and not for actual use in this project.

- `time_utils` from [autoware.universe/common/time_utils](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/common/time_utils): unchanged. This package provides a conversion interface between the `std::chrono::time_point` and `builtin_interfaces::msg ` types.

- tier4_debug_msgs` from [external/tier4_autoware_msgs/tier4_debug_msgs](https://github.com/tier4/tier4_autoware_msgs/tree/tier4/universe/tier4_debug_msgs): unchanged. This package is used for the output of debugging information. Compared with the std info, the debugging information contains timestamps.

- `tier4_autoware_utils` from [autoware.universe/common/tier4_autoware_utils](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/common/tier4_autoware_utils): simplified to keep only the functions required by the location module. This simplification brings two advantages: removing the dependency on *autoware_auto_perception_msgs* and *autoware_auto_planning_msgs*; and removing the need to rewrite this package as a dora node after pruning unnecessary functions.

  > The *test* subdirectory is deleted. 
  >
  > The `boost_polygon_utils.hpp` and `boost_polygon_utils.cpp` files in the *geometry* subdirectory are deleted. These two files are needed only for perception.
  >
  > The `path_with_lane_id_geometry.hpp` file from the *geometry* subdirectory is deleted. The content related to `autoware_auto_planning_msgs` is also removed from `geometry.hpp`. They are needed only for planning.
  >
  > The .hpp files related to pub/sub in each subdirectory are deleted. They are not required by the location module, so, there is no need to convert the corresponding package to a dora node after the file deletion.
  >
  > The `package.xml`, `CMakeLists.txt`, .hpp, and .cpp source files are modified accordingly.

- `geo_pos_conv` from [autoware.universe/sensing/geo_pos_conv](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/sensing/geo_pos_conv): dependency on `rclcpp` removed. This package serves to the conversion between GNSS raw latitude/longitude coordinates and Cartesian (east-north-up, ENU) coordinates.

- `autoware_sensing_msgs` from [autoware_msgs/autoware_sensing_msgs](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_sensing_msgs): unchanged. This package is required by `gnss_poser`. If you want to remove the dependency on this package, delete the code used for processing the "gnss orientation" topic from `gnss_poser`.

- `ndt_pcl_modified` from [autoware.universe/localization/ndt_pcl_modified](https://github.com/autowarefoundation/autoware.universe/tree/main/localization/ndt_pcl_modified): unchanged. This package is a version of the ndt implementation slightly modified based on the *ndt_pcl* implementation.

- `ndt_omp` from [external/ndt_omp](https://github.com/tier4/ndt_omp): unchanged. This package is the multicore implementation of the ndt algorithm.

- `ndt` from [autoware.universe/localization/ndt](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/localization/ndt): unchanged. This package provides common interfaces for different ndt implementation versions.

- `kalman_filter` from [autoware.universe/common/kalman_filter](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/common/kalman_filter): unchanged. This package is the implementation of Kalman filter.

**Packages in the nodes directory (rewritten and new dora node):**

The **required changes** below are those required to rewrite a ROS 2 package to a dora node.

- `point_map_loader` from [autoware.universe/map/pointcloud_map_loader](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/map/map_loader): modified as necessary. This package is used to load the point cloud map.
- `gnss_imu_dummy`: newly added. This package is used to simulate the integrated navigation driver and publish "gnss "+"imu "+"orientation" data to the external.
- `rosbag_dummy`: newly added. This package is used to simulate the feature that uses rosbag to play lidar point cloud data. 
- `manual_pose`: newly added. This package is used to manually specify the initial position of vehicles. The initial position must match the point cloud data played by `rosbag_dummy`.
- `tf_publisher`: newly added. This package is used to publish static coordinate transformation information needed for other nodes to work. 
- `vehicle_twist`: newly added. This package is used to simulate the vehicle wire control program and publish the vehicle speed information to the external.

**Package in the operators package (rewritten dora operators):**

The **required changes** below are those required to rewrite a ROS 2 package to a dora operator.

- `gnss_poser` from [autoware.universe/sensing/gnss_poser](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/sensing/gnss_poser): changed a lot. As a pre-processing node of raw GNSS signals, this package is used to convert GNSS signals to coordinates in the map coordinate system. The modifications in addition to "required changes" are: 

  > The output topic is simplified: The output contains only `gnss/pose_with_cov`. Other unnecessary content will not be contained in the output.
  >
  > The `RCLCPP_COMPONENTS_REGISTER_NODE` operation and the dependency on `rclcpp_components` are removed. This is because the ROS 2 component mechanism is no longer used in the dora framework, and accordingly, namespaces such as "gnss_poser" do not need to be defined.
  >
  > `rclcpp::Logger`is removed from `convert.hpp`. stdout is used to print messages, thus removing the dependency on `rclcpp`.
  >
  > You are allowed to remove the dependency on`autoware_sensing_msgs` by deleting the code used for processing the "gnss orientation" topic from `gnss_poser`.

- `imu_corrector` from [autoware.universe/sensing/imu_corrector](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/sensing/imu_corrector): modified as necessary. As a pre-processing node of raw IMU signals, this package is used to add covariation information to IMU signals.  The modification in addition to "required changes" is removing the `RCLCPP_COMPONENTS_REGISTER_NODE` operation, which registers nodes to ROS 2.

- `pose_initializer` from [autoware.universe/localization/pose_initializer](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/localization/pose_initializer): changed a lot. According to `pose_initializer_core.cpp`, the original code logic is: Obtain the initial position manually specified or provided by GNSS (`map_height_fitter` will attach the height information to the initial position provided GNSS), use `ndt_align` (if enabled) to enhance the precision of the initial position, and then publish the precise position and `state` (initialization state). The invocation of `pose_initializer` is triggered by the `automatic_pose_initializer` package. Considering the code logic above and that dora does not provide a mechanism similar to ROS 2 Service, `pose_initializer` must contain the following parts: A node for sending the initial position that is manually specified; an operator for using `map_height_fliter` to attach the height information to GNSS messages that are processed by `gnss_poser`; an operator for receiving the manually-specified initial position information or GNSS signals processed by `map_height_fitter` and sending service requests to `ntd_align`; and a feature for returning processed precise initial position from the server of `ndt_align` to the client. The modifications in addition to the "required changes" are:

  > `ServiceException` thrown by `gnss_module.cpp` and `ndt_module.cpp` are changed to `std::runtime_error`. This removes the dependency on `component_interface_utils`.
  >
  > The `stop_check` module is removed. This module was used to initialize only the position of vehicles that are stopped. Removing this module avoids dependencies on `motion_utils` and its dependency packages.
  >
  > The publish of the `state_` variable is canceled. The `automatic_pose_initializer` node is no longer invoked during initialization. The two main reasons are: many additional dependencies are required for calling this node, and replacing the Service calls and Timer calls involved in the `automatic_pose_initializer` node causes much additional work. The enum variable `InitializationState` is redefined to record the position initialization state.

- `map_height_fitter` from [autoware.universe/localization/pose_initializer](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/localization/pose_initializer): changed a lot. This package is used to subscribe map and GNSS messages to attach the height information to GNSS messages. For this package, the logic of the files in the `map_height_fitter` subfolder and the logic of `gnss_module.cpp` are organized for adapting to each other.

- `gyro_odometer` from [autoware.universe/localization/gyro_odometer](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/localization/gyro_odometer): modified as necessary. This package is used to integrate the angular velocity from IMU and the linear velocity from the vehicle wire control program, and then publish the observed vehicle velocity.

- `pointcloud_downsample` from [autoware.universe/sensing/pointcloud_preprocessor](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/sensing/pointcloud_preprocessor): changed a lot. This package is the module for downsampling before the raw point cloud data is imported to ndt for alignment. The original `pointcloud_preprocessor` provides a lot of features and requires C++ class inheritance. The base class is more complex, and the original architecture is designed according to ROS 2 Component. Considering that the location module uses only `crop_box_filter`, `voxel_grid_downsample_filter`, and `random_downsample_filter`, here we keep only the original basic framework and build a point cloud downsampling operator for the location module.

  > The inheritance between the base class `filter` and different sub-filters is removed for two main reasons: 
  >
  > For one thing, ROS 2 Component is no longer needed in dora;
  >
  > For another, during hybrid programming with Rust, you need to call `crop_box_filter` and other `filter` functions externally. As a result, you can neither define a pure virtual `filter` function in the base class, nor set the `filter` function as a protected property. 
  >
  > In the point cloud downsampling node built for the location-oriented module, `crop_box_filter`, `voxel_grid_downsample_filter`, and `random_downsample_filter` are executed sequentially, which eliminates the use of the `mutex_` variable in the original base class. 

- `ndt_scan_matcher` from [autoware.universe/localization/ndt_scan_matcher](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/localization/ndt_scan_matcher): changed a lot. This package executes the ndt alignment algorithm and provides the `ndt_align` service to `pose_initializer`. 

  > The timerDiagnostic() function is deleted. Although it is an independent thread, it cannot be converted to a separate dora node because it depends on the states defined in `ndt_scan_matcher`. Accordingly, the dependency on `diagnostic_msgs` is removed.
  >
  > For the code related to the `ndt_align` service: the serviceNDTAlign() and alignUsingMonteCarlo() functions are modified, the dependency on `tier4_localization_msgs` is removed, and the logic of `ndt_module.cpp` in the original `pose_initializer` is integrated.
  >
  > The output topic is simplified: The output contains only `pose_with_cov` and the `ndt_align` response information that needs to be returned to `pose_initalizer`. Other unnecessary content will not be contained in the output.
  >
  > The multithread (`rclcpp::executors::MultiThreadedExecutor` and `callback_group`) mechanism is aborted.

- `ekf_localizer` from [autoware.universe/localization/ekf_localizer](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/localization/ekf_localizer): changed a lot. This package filters the output position information from `ndt_scan_matcher` to obtain a smooth positioning result.

  > ekf_localizer needs to send filter results and map2baselink coordinate transformations at a regular interval. In ROS 2, you can set rclcpp::create_timer() based on the values of the initial parameters `ekf_rate_` and `tf_rate_` parameters to specify the interval. dora uses `dora/timer/millis/20` to trigger result sending, but the initial parameters still are required. For this reason, the interval specified by the initial parameter `ekf_rate_` must be the same as that specified by `dora/timer/millis/20`.
  >
  > `warning.hpp` and the code for calling this file are removed. stdout is used to print messages instead. Also, the format of messages to be printed is adjusted. The RCLCPP macro is aborted to remove the dependency on rclcpp. Note that std is also used as a temporary solution to allow only one time of message print during a specified period, which is implemented by `RCLCPP_WARN_THROTTLE` in ROS 2. A better solution will be provided in the future.
  >
  > `rclcpp::time` is replaced by `std::chrono::system_clock::time_point` to avoid the use of operations like rclcpp::node->get_clock()->now().
  >
  > The output topic is simplified by eliminating unnecessary output content.

- stop_filter from [autoware.universe/localization/stop_filter](https://github.com/autowarefoundation/autoware.universe/tree/8e92d176721e71943d7dff0722365cb9ee97379f/localization/stop_filter): modified as necessary.