cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(plan_kinematic_path)
rospack_add_executable(plan_kinematic_path src/plan_kinematic_path.cpp)
