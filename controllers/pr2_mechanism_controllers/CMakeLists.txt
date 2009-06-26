cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)
rospack(pr2_mechanism_controllers)
genmsg()
gensrv()
#rospack_add_library(pr2_controllers src/base_controller.cpp src/laser_scanner_controller.cpp  src/arm_position_controller.cpp src/arm_velocity_controller.cpp src/arm_pd_controller.cpp src/head_pan_tilt_controller.cpp src/base_kinematics.cpp src/pr2_base_controller.cpp src/pr2_odometry.cpp)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_boost_directories()

rospack_add_library( pr2_mechanism_controllers
  src/base_controller.cpp
  src/base_controller_pos.cpp
  src/base_position_controller.cpp
  src/laser_scanner_controller.cpp
  src/laser_scanner_traj_controller.cpp 
  src/head_pan_tilt_controller.cpp
  src/head_servoing_controller.cpp
  src/caster_controller.cpp
  src/caster_controller_effort.cpp
  src/caster_calibration_controller.cpp
  src/caster_calibration_controller_effort.cpp
  src/wrist_calibration_controller.cpp
  src/gripper_calibration_controller.cpp
  src/arm_trajectory_controller.cpp
  src/whole_body_trajectory_controller.cpp
  src/joint_trajectory_controller.cpp
  src/base_kinematics.cpp
  src/pr2_gripper_controller.cpp
  src/pr2_odometry.cpp
  src/pr2_base_controller.cpp
  src/Pose3D.cpp
  )
rospack_link_boost(pr2_mechanism_controllers thread)


rospack_add_executable(base_trajectory_controller src/base_trajectory_controller.cpp)
rospack_declare_test(base_trajectory)
