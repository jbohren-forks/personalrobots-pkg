cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(pr2_robot_actions)
genmsg()

rospack_add_executable(bin/set_laser_tilt src/set_laser_tilt.cpp)
