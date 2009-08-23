cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(wheel_odometry_calibration)
genmsg()

rospack_add_executable(wheel_odometry_calibration src/odom_calib.cpp)
rospack_add_boost_directories()
rospack_link_boost(wheel_odometry_calibration thread)
