cmake_minimum_required(VERSION 3.14)
project(stop_filter)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/find_cereal.cmake)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  ${CEREAL_DIRS}
)

# add the directory where the cxx static library is located
string(REGEX REPLACE "(.+)/\\src/operators/stop_filter" "\\1" cxx_lib_path_prefix ${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${cxx_lib_path_prefix}/cxx_lib)

ament_auto_add_library(${PROJECT_NAME}_native SHARED
  src/stop_filter.cpp
  src/ffi.rs.cc
)

target_link_libraries(${PROJECT_NAME}_native
    cxxbridge1
)

ament_auto_package()
