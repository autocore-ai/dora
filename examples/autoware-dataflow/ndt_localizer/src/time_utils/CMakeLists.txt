cmake_minimum_required(VERSION 3.5)
project(time_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -pthread")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Ignore PCL errors in Clang
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wno-gnu-anonymous-struct -Wno-nested-anon-types)
endif()

# Ignore Boost deprecated messages
add_compile_definitions(BOOST_ALLOW_DEPRECATED_HEADERS)

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED 
  src/time_utils/time_utils.cpp
)

# install the library or executable
ament_auto_package()
