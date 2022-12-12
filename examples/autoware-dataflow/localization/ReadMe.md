## Brief description

The entire location module of [autoware.universe](8e92d176721e71943d7dff0722365cb9ee97379f) will be migrated here to build a closed test loop. The code baseline of autoware.universe is based on the version of 2022.09.16, which may lag behind the mainstream. However, the code functionality of the location module is largely unchanged in the mainstream.

In addition to streamlining the workflow based on Normal Distributions Transform (NDT), migrating the code of the location module of autoware.universe involves the following work:

1. Analyzing the auxiliary nodes in the location module and stripping out unnecessary nodes.

2. Figuring out how to implement the srv mechanism of ROS in Dora. In the current scheme, topics are used in both requests and responses as replacement.

3. Stripping out or migrating unnecessary functions of the point cloud preprocessing module.

4. Converting the nodes in one container into Dora operators (A container in ROS may contain multiple nodes).

5. Stripping out the message types customized by Autoware as many as possible.

Currently, ekf_localizer and its dependencies have not been migrated, and the migrated packages have not been tested. For this reason, the detailed content of this directory will be uploaded as soon as possible.

## Package rewriting

**Package in auxiliary (pure ROS nodes): **

- **cereal_ros_msgs**: newly added for serializing the messages involved.

- **autoware_cmake** from "src/core/autoware_common/autoware_cmake": unchanged.

- **autoware_lint_common** from "src/core/autoware_common/autoware_lint_common": unchanged.

- **ndt_pcl_modified** from "src/universe/autoware.universe/localization/ndt_pcl_modified": unchanged.

- **ndt_omp** from "src/universe/external/ndt_omp": unchanged.

- **ndt** from "src/universe/autoware.universe/localization/ndt": unchanged.

- **time_utils** from "src/universe/autoware.universe/common/time_utils": unchanged.

- **tier4_debug_msgs** from "src/universe/external/tier4_autoware_msgs/tier4_debug_msgs": unchanged. This package is used for the output of debugging information. Compared with the std info, the debugging information contains timestamps. (Note that cereal serialization will be implemented in the future.)

- **tier4_autoware_utils** from "src/universe/autoware.universe/common/tier4_autoware_utils": This package is simplified as follows to keep only the functions required by the location module. The simplification aims to avoid introducing dependencies on autoware_auto_perception_msgs and autoware_auto_planning_msgs, and to avoid converting tier4_autoware_utils as Dora nodes.

  > The test subdirectory is deleted. 
  >
  > The boost_polygon_utils.hpp and boost_polygon_utils.cpp files from the geometry subdirectory are deleted. These two files are needed only for perception.
  >
  > The path_with_lane_id_geometry.hpp file from the geometry subdirectory is deleted. The content related to autoware_auto_planning_msgs is also removed from geometry.hpp. They are needed only for planning.
  >
  > The .hpp files related to pub/sub in each subdirectory are deleted. They are not required by the location module, so, there is no need to convert the corresponding package to a Dora node after the deletion.
  >
  > The package.xml, CMakeLists.txt, .hpp and .cpp source files are modified accordingly.

**Packages in nodes (converted to Dora nodes after migration):**

- **point_map_loader** from "src/universe/autoware.universe/map/pointcloud_map_loader": modified as necessary. 

**Packages in operators (converted to Dora operators after migration):**

- **stop_filter** from "src/universe/autoware.universe/localization/stop_filter": modified as necessary.

- **gyro_odometer** from "src/universe/autoware.universe/localization/gyro_meter": modified as necessary.

- **ndt_scan_matcher** from "src/universe/autoware.universe/localization/ndt_scan_matcher": modified a lot. To be specific, the timerDiagnostic timed diagnotor is deleted, and the code related to srv is removed (the srv mechanism is converted to a separate Dora node for future use).

  > The timerDiagnostic() function is deleted. It cannot be converted to a separate Dora node because it depends on the states defined in ndt_scan_matcher. Accordingly, the dependency on diagnostic_msgs is removed.
  >
  > The srv-related code, i.e. the serviceNDTAlign() and alignUsingMonteCarlo() functions, is deleted. This mechanism is converted to a separate Dora node for future use. Accordingly, the dependency on tier4_localization_msgs is removed.