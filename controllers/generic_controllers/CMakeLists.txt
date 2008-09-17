cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(generic_controllers)
genmsg()
gensrv()
add_definitions(-DBT_USE_DOUBLE_PRECISION)
rospack_add_library(generic_controllers
 src/joint_effort_controller.cpp
 src/joint_position_controller.cpp
 src/joint_velocity_controller.cpp
 src/cartesian_effort_controller.cpp
 src/ramp_effort_controller.cpp
 src/sine_sweep_controller.cpp
 src/joint_autotuner.cpp
 src/joint_pd_controller.cpp
 src/joint_calibration_controller.cpp
 src/joint_blind_calibration_controller.cpp
 src/joint_manual_calibration_controller.cpp)
