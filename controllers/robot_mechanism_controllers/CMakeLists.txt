cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(robot_mechanism_controllers)
genmsg()
gensrv()
rospack_add_library(robot_mechanism_controllers
 src/endeffector_constraint_controller.cpp
 src/endeffector_pose_controller.cpp
 src/endeffector_twist_controller.cpp

 src/endeffector_wrench_controller.cpp
 src/joint_effort_controller.cpp
 src/joint_position_controller.cpp
 src/joint_position_smoothing_controller.cpp
 src/joint_velocity_controller.cpp
 src/cartesian_effort_controller.cpp
 src/cartesian_velocity_controller.cpp
 src/cartesian_position_controller.cpp
 src/cartesian_torque_controller.cpp
 src/cartesian_orientation_controller.cpp
 src/joint_autotuner.cpp
 src/joint_pd_controller.cpp
 src/joint_calibration_controller.cpp
 src/joint_blind_calibration_controller.cpp
 src/lqr_controller.cpp
 src/ros_serialchain_model.cpp
 )
