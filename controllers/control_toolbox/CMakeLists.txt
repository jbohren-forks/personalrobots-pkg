cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include_directories(include/control_toolbox/eigen2)
rospack(control_toolbox)
genmsg()
gensrv()
include_directories(include/control_toolbox/eigen2)
rospack_add_library(control_toolbox
 src/pid.cpp
 src/serialchain_model.cpp
 src/sine_sweep.cpp
 src/ramp_output.cpp
 src/base_position_pid.cpp
 )
 
rospack_add_gtest(test/lqr_utils test/lqr_utils.cpp)

rospack_add_gtest(test/lqr test/lqr.cpp)
# rospack_add_executable(test_linear test/linear.cpp)
