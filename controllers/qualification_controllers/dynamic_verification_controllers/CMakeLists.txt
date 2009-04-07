cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(dynamic_verification_controllers)
genmsg()
gensrv()
rospack_add_library(dynamic_verification_controllers 
src/constant_torque_controller.cpp
src/freq_resp_controller.cpp)

rospack_add_library(dynamic_verification_controllers src/constant_torque_controller.cpp src/freq_resp_controller.cpp)

