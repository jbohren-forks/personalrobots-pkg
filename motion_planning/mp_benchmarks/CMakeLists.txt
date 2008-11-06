cmake_minimum_required (VERSION 2.6)

set (ROS_BUILD_TYPE Release)
include (rosbuild)
add_definitions (-Wall)
rospack (mp_benchmarks)

rospack_add_library (mp_benchmarks src/setup.cpp)

rospack_add_executable (mpbench src/main.cpp)
target_link_libraries (mpbench mp_benchmarks costmap_2d sunflower)
