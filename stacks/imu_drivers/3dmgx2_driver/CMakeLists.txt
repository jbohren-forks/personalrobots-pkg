cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(3dmgx2_driver)
rospack_add_library(3dmgx2 3dmgx2.cc)
