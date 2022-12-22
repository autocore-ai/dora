cmake_minimum_required(VERSION 3.14)
project(gnss_poser)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/find_cereal.cmake)

find_package(autoware_cmake REQUIRED)
autoware_package()

## Find non-ROS library
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)

set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

include_directories(
  ${CEREAL_DIRS}
)

# add the directory where the cxx static library is located
string(REGEX REPLACE "(.+)/\\src/operators/gnss_poser" "\\1" cxx_lib_path_prefix ${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${cxx_lib_path_prefix}/cxx_lib)

ament_auto_add_library(${PROJECT_NAME}_native SHARED
  src/gnss_poser.cpp
  src/ffi.rs.cc
)

target_link_libraries(${PROJECT_NAME}_native
  Geographic
  cxxbridge1
)

ament_auto_package()
