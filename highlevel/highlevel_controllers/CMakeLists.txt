cmake_minimum_required(VERSION 2.4.6)

### If you're trying to debug include directives or so, this here
### might help:
#SET (CMAKE_VERBOSE_MAKEFILE ON)
set(ROS_BUILD_TYPE Release)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions(-Wall)
rospack(highlevel_controllers)

genmsg()

# Library
rospack_add_library(highlevel_controllers src/MoveBase.cpp src/VelocityControllers.cpp)
target_link_libraries(highlevel_controllers trajectory_rollout costmap_2d)

# MoveArm
rospack_add_executable(move_arm src/MoveArm.cpp)

# move_base_sbpl
rospack_add_executable(move_base_sbpl src/move_base_sbpl.cpp)
target_link_libraries(move_base_sbpl highlevel_controllers sbpl_util)

# move_base_navfn
rospack_add_executable(move_base_navfn src/move_base_navfn.cpp)
target_link_libraries(move_base_navfn highlevel_controllers navfn)

# move_base_topological
rospack_add_executable(move_base_topological src/move_base_topological.cpp)
target_link_libraries(move_base_topological highlevel_controllers topological_map)

# MoveArmEndEffector
rospack_add_executable(move_end_effector src/MoveEndEffector.cpp)

#Joystick Sleep Button
rospack_add_executable(joy_batt_sender src/joy_batt_sender.cpp)

# PlugIn and Unplug
rospack_add_executable(recharge_controller src/RechargeController.cpp)

#CLI
rospack_add_executable(cli src/ControlCli.cpp)
