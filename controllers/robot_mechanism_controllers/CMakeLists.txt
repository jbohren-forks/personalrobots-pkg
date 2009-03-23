cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(robot_mechanism_controllers)
genmsg()
gensrv()
set(_srcs
    src/plug_controller.cpp
    src/cartesian_tff_controller.cpp
    src/cartesian_trajectory_controller.cpp
    src/cartesian_pose_controller.cpp
    src/cartesian_twist_controller.cpp
    src/cartesian_twist_controller_ik.cpp
    src/cartesian_wrench_controller.cpp
    src/dynamic_loader_controller.cpp
    src/joint_effort_controller.cpp
    src/joint_position_controller.cpp
    src/joint_position_smoothing_controller.cpp
    src/joint_velocity_controller.cpp
    src/cartesian_effort_controller.cpp
    src/cartesian_velocity_controller.cpp
    src/cartesian_position_controller.cpp
    src/cartesian_torque_controller.cpp
    src/cartesian_orientation_controller.cpp
    src/cartesian_hybrid_controller.cpp
    src/joint_autotuner.cpp
    src/joint_pd_controller.cpp
    src/joint_calibration_controller.cpp
    src/joint_ud_calibration_controller.cpp
    src/joint_limit_calibration_controller.cpp
    src/joint_blind_calibration_controller.cpp
    src/lqr_controller.cpp
    src/ros_serialchain_model.cpp
    src/joint_chain_constraint_controller.cpp)

# For some reason, the endeffector_constraint_controller won't build on OSX
if(NOT APPLE)
 set(_srcs ${_srcs} src/endeffector_constraint_controller.cpp)
endif(NOT APPLE)
rospack_add_library(robot_mechanism_controllers ${_srcs})
target_link_libraries(robot_mechanism_controllers ltdl)
