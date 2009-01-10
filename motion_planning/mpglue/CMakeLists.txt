cmake_minimum_required (VERSION 2.6)

### If you're trying to debug include directives or so, this here
### might help:
#set (CMAKE_VERBOSE_MAKEFILE ON)

set (ROS_BUILD_TYPE Release)
#set (ROS_BUILD_TYPE Debug)
include (rosbuild)
add_definitions (-Wall)
rospack (mpglue)

rospack_add_library (mpglue
		     src/sbpl_util.cpp
		     src/costmap.cpp
		     src/planner.cpp
		     src/sbpl_planner.cpp
		     src/navfn_planner.cpp
		     src/footprint.cpp
		     src/environment.cpp
		     src/plan.cpp)
target_link_libraries (mpglue costmap_2d costmap_2d sunflower navfn)

rospack_add_executable (t3dobst src/t3dobst.cpp)
target_link_libraries (t3dobst mpglue)
