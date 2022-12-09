# Try to find cereal's hpp files
find_path(CEREAL_DIRS 
    NAMES cereal 
    PATHS ../../
)

# find_path(CEREAL_ROS_MSGS_INCLUDE_DIRS 
#     NAMES cereal_ros_msgs 
#     PATHS ../cereal_ros_msgs/include/
# )

# find_library(CEREAL_ROS_MSGS_LIBRARIES 
#     NAMES cereal_ros_msgs 
#     PATHS ../../devel/lib/
# )