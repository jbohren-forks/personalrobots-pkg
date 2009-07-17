cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(teleop_anti_collision)
genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_boost_directories()
rospack_add_executable(teleop_goal_projection src/teleop_goal_projection.cpp)
rospack_add_executable(anti_collision_base_controller src/anti_collision_base_controller.cpp)
