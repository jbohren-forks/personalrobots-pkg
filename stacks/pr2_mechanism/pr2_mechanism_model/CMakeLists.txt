cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)
rospack(pr2_mechanism_model)

rospack_add_library( pr2_mechanism_model
  src/joint.cpp
  src/robot.cpp
  src/chain.cpp
  src/simple_transmission.cpp
  src/gripper_transmission.cpp
  src/pr2_gripper_transmission.cpp
  src/wrist_transmission.cpp)


rospack_add_gtest(test/test_chain test/test_chain.cpp)
target_link_libraries(test/test_chain pr2_mechanism_model)

