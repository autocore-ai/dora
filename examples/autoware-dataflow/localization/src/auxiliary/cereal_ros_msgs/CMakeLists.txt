cmake_minimum_required(VERSION 3.5)
project(cereal_ros_msgs)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/FindCereal.cmake)

find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# find_package(PCL 1.10 REQUIRED)
# find_package(Eigen3 REQUIRED)

include_directories(
    # include
    ${CEREAL_DIRS}
    # ${PCL_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME}
    src/cereal_ros_msgs.cpp
)

# definition the dependency among differnt packages
# add_dependencies(${PROJECT_NAME} <target-dependency>)

# target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

# install the library or executable
ament_auto_package()
