cmake_minimum_required(VERSION 2.6)
include(rosbuild)

rospack(cv_cam_calib)
rospack_add_executable(cv_cam_calib cv_cam_calib.cpp)

