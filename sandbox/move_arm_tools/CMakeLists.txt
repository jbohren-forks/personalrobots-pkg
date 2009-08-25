cmake_minimum_required(VERSION 2.4.6)
set(ROS_BUILD_TYPE debug)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(move_arm_tools)

genmsg()
gensrv()

# Build executables in the bin directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(arm_cmd_unsafe src/arm_cmd_line_unsafe.cpp)
rospack_link_boost(arm_cmd_unsafe thread)

rospack_add_executable(auto_arm_cmd src/auto_arm.cpp)
rospack_link_boost(auto_arm_cmd thread)

