cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(pr2_gripper_controller)
genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_boost_directories()

rospack_add_library( pr2_gripper_controller
  #src/pr2_gripper_controller.cpp
  src/teleop_gripper_keyboard.cpp
  )
rospack_add_executable(teleop_gripper_keyboard src/teleop_gripper_keyboard.cpp)
#rospack_link_boost(pr2_griper_controllers thread)
#rospack_link_boost(pr2_griper_controllers)

