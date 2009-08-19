cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)
rospack(robot_mechanism_controllers)
genmsg()
gensrv()
set(_srcs
    src/controller_manifest.cpp
    src/cartesian_trajectory_controller.cpp
    src/cartesian_pose_controller.cpp
    src/cartesian_twist_controller.cpp
    src/cartesian_wrench_controller.cpp
    src/joint_effort_controller.cpp
    src/joint_position_controller.cpp
    src/joint_velocity_controller.cpp
    src/joint_pd_controller.cpp
    src/joint_ud_calibration_controller.cpp
    src/trigger_controller.cpp
)

# For some reason, the endeffector_constraint_controller won't build on OSX
rospack_add_library(robot_mechanism_controllers ${_srcs})
rospack_remove_compile_flags(robot_mechanism_controllers -W)
target_link_libraries(robot_mechanism_controllers ltdl)
