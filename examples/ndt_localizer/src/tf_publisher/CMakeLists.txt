cmake_minimum_required(VERSION 3.5)
project(tf_publisher)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -pthread")

# set(CMAKE_EXPORT_COMPILE_COMMANDS true)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/find_cereal.cmake)

find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${CEREAL_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# add the directory where the static library is located
string(REGEX REPLACE "(.+)/\\examples/ndt_localizer.*" "\\1" lib_path ${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${lib_path}/target/debug)

# ament_auto_add_library(${PROJECT_NAME}_native SHARED 
#     src/tf_publisher.cpp
#     src/lib.rs.cc
# )
ament_auto_add_executable(${PROJECT_NAME}_native
    src/tf_publisher.cpp
    src/lib.rs.cc
)

# definition the dependency among differnt targets
# add_dependencies(${PROJECT_NAME} xxx_lib)

target_link_libraries(${PROJECT_NAME}_native
    ${PCL_LIBRARIES}
    dora_node_api_cxx -lrt
    dl
)

# install the library or executable
ament_auto_package()
