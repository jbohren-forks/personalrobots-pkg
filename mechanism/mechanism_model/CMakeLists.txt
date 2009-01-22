cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(mechanism_model)
rospack_add_library( mechanism_model
  src/simple_transmission.cpp
  src/gripper_transmission.cpp
  src/joint.cpp
  src/robot.cpp
  src/link.cpp
  src/chain.cpp
  src/wrist_transmission.cpp)

rospack_add_gtest(test/test_FK test/test_FK.cpp)
target_link_libraries(test/test_FK mechanism_model)

rospack_add_gtest(test/test_chain test/test_chain.cpp)
target_link_libraries(test/test_chain mechanism_model)
