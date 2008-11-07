cmake_minimum_required (VERSION 2.6)

### If you're trying to debug include directives or so, this here
### might help:
set (CMAKE_VERBOSE_MAKEFILE ON)

set (ROS_BUILD_TYPE Release)
include (rosbuild)
add_definitions (-Wall)
rospack (mp_benchmarks)

rospack_add_library (mp_benchmarks src/setup.cpp)

rospack_add_executable (mpbench src/main.cpp)
target_link_libraries (mpbench mp_benchmarks costmap_2d sunflower sbpl_util)
