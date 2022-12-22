cmake_minimum_required(VERSION 3.14)
project(vehicle_twist)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -pthread")

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/find_cereal.cmake)

find_package(autoware_cmake REQUIRED)
autoware_package()

# find_package(PCL REQUIRED)
# find_package(Eigen3 REQUIRED)

include_directories(
    ${CEREAL_DIRS}
    # ${PCL_INCLUDE_DIRS}
    # ${EIGEN3_INCLUDE_DIR}
)

# add the directory where the static library is located
string(REGEX REPLACE "(.+)/\\examples/autoware-dataflow/localization.*" "\\1" lib_path ${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${lib_path}/target/debug)

ament_auto_add_executable(${PROJECT_NAME}_native
    src/vehicle_twist.cpp
    src/lib.rs.cc
)

target_link_libraries(${PROJECT_NAME}_native
    # ${PCL_LIBRARIES}
    dora_node_api_cxx -lrt
    dl
)

# install the library or executable
ament_auto_package()
