cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(plug_in)

#rospack_add_boost_directories()
rospack_add_executable(servo src/servo.cpp)
