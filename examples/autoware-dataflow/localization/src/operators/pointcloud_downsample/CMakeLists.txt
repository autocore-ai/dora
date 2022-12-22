cmake_minimum_required(VERSION 3.14)
project(pointcloud_downsample)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/find_cereal.cmake)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

include_directories(SYSTEM
  ${CEREAL_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# add the directory where the cxx static library is located
string(REGEX REPLACE "(.+)/\\src/operators/pointcloud_downsample" "\\1" cxx_lib_path_prefix ${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${cxx_lib_path_prefix}/cxx_lib)

ament_auto_add_library(${PROJECT_NAME}_native SHARED
  src/pointcloud_downsample.cpp
  src/filters/crop_box_filter.cpp
  src/filters/voxel_grid_downsample_filter.cpp
  src/filters/random_downsample_filter.cpp
  src/ffi.rs.cc
)

target_link_libraries(${PROJECT_NAME}_native
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  cxxbridge1
)

ament_auto_package()
