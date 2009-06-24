cmake_minimum_required(VERSION 2.4.6)

### If you're trying to debug include directives or so, this here
### might help:
#SET (CMAKE_VERBOSE_MAKEFILE ON)

set(ROS_BUILD_TYPE Debug)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions(-Wall)
rospack(robot_actions)

genmsg()

#Add library
rospack_add_library(robot_actions src/action_runner.cpp)
rospack_link_boost(robot_actions thread)

