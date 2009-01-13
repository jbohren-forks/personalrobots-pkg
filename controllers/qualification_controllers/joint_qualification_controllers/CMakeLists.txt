cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(joint_qualification_controllers)
genmsg()
gensrv()
rospack_add_library(joint_qualification_controllers 
src/hysteresis_controller.cpp
src/sine_sweep_controller.cpp
src/backlash_controller.cpp)
