cmake_minimum_required(VERSION 3.14)
project(tier4_autoware_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)

ament_auto_add_library(tier4_autoware_utils SHARED
  src/tier4_autoware_utils.cpp
)

ament_auto_package()
