cmake_minimum_required(VERSION 3.14)
project(ekf_localizer)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/find_cereal.cmake)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

include_directories(SYSTEM
  ${CEREAL_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

ament_auto_find_build_dependencies()

# add the directory where the cxx static library is located
string(REGEX REPLACE "(.+)/\\src/operators/ekf_localizer" "\\1" cxx_lib_path_prefix ${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${cxx_lib_path_prefix}/cxx_lib)

ament_auto_add_library(${PROJECT_NAME}_native SHARED
  src/ekf_localizer.cpp
  src/mahalanobis.cpp
  src/measurement.cpp
  src/state_transition.cpp
  src/ffi.rs.cc
)

target_link_libraries(${PROJECT_NAME}_native
  Eigen3::Eigen
  cxxbridge1
)

ament_auto_package()
