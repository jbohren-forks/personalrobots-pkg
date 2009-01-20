cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(cv_mech_turk)
genmsg()

rospack_add_executable(cv_mturk src/cv_mturk.cpp)

