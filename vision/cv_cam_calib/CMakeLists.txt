cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(cv_cam_calib)
rospack_add_executable(cv_cam_calib cv_cam_calib.cpp)

