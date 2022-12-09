cmake_minimum_required(VERSION 3.5)
project(voxel_grid_filter)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
# set(CMAKE_CXX_FLAGS "-std=c++17 -fPIC")   # for build static library

# set(CMAKE_EXPORT_COMPILE_COMMANDS true)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/find_cereal.cmake)

find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED)
# find_package(Eigen3 REQUIRED)

include_directories(
    ${CEREAL_DIRS}
    ${PCL_INCLUDE_DIRS}
    # ${EIGEN3_INCLUDE_DIR}
)

# add the directory where the cxx static library is located
string(REGEX REPLACE "(.+)/\\src/voxel_grid_filter" "\\1" cxx_lib_path_prefix ${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${cxx_lib_path_prefix}/cxx_lib)

ament_auto_add_library(${PROJECT_NAME}_native SHARED
    src/voxel_grid_filter.cpp
    src/ffi.rs.cc
)

# target_include_directories(${PROJECT_NAME}_native PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

target_link_libraries(${PROJECT_NAME}_native
    ${PCL_LIBRARIES}
    cxxbridge1
)

ament_export_dependencies(sensor_msgs pcl_conversions cereal_ros_msgs PCL)

# install the library or executable
ament_auto_package()
