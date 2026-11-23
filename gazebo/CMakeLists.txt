cmake_minimum_required(VERSION 3.16)
project(asw_gazebo CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Gazebo Harmonic (gz-sim 8, gz-physics 7, DART 6), from conda-forge
# (environment/gazebo.yml) or ROS 2 Jazzy's vendored Gazebo.
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-physics7 REQUIRED)
find_package(gz-common5 REQUIRED COMPONENTS geospatial graphics profiler)
find_package(DART 6 REQUIRED COMPONENTS dart)

# The gz-sim8 CMake target also links GUI, rendering and FFmpeg libraries these
# server-side plugins never use. --as-needed drops them, so a headless process
# does not load media libraries that may need a newer C library than the host.
set(ASW_LINK_OPTIONS -Wl,--as-needed)

# Bridge between the gz-sim server and the Python controller.
add_library(SphericalWheelBridge SHARED plugin/SphericalWheelBridge.cc)
target_link_libraries(SphericalWheelBridge PRIVATE
  gz-sim8::gz-sim8 gz-plugin2::register gz-physics7::gz-physics7)
target_link_options(SphericalWheelBridge PRIVATE ${ASW_LINK_OPTIONS})

# gz-sim 8.10.0's physics system with exact per-step link state (physics/README.md).
add_library(AswExactPhysics SHARED physics/Physics.cc)
target_include_directories(AswExactPhysics PRIVATE physics)
target_link_libraries(AswExactPhysics PRIVATE
  gz-sim8::gz-sim8 gz-plugin2::register gz-physics7::gz-physics7
  gz-common5::gz-common5 gz-common5::geospatial gz-common5::graphics gz-common5::profiler
  dart)
target_link_options(AswExactPhysics PRIVATE ${ASW_LINK_OPTIONS})

install(TARGETS SphericalWheelBridge AswExactPhysics LIBRARY DESTINATION lib)
