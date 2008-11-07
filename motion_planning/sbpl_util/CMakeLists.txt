cmake_minimum_required (VERSION 2.6)

### If you're trying to debug include directives or so, this here
### might help:
set (CMAKE_VERBOSE_MAKEFILE ON)

set (ROS_BUILD_TYPE Release)
include (rosbuild)
add_definitions (-Wall)
rospack (sbpl_util)

rospack_add_library (sbpl_util src/sbpl_util.cpp)
