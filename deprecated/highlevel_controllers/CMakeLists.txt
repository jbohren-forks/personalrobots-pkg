cmake_minimum_required(VERSION 2.4.6)

### If you're trying to debug include directives or so, this here
### might help:
#SET (CMAKE_VERBOSE_MAKEFILE ON)
set(ROS_BUILD_TYPE Debug)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions(-Wall)
rospack(highlevel_controllers)

rospack_add_boost_directories()

genmsg()

# Library
rospack_add_library(highlevel_controllers src/move_base.cpp)
rospack_link_boost(highlevel_controllers thread)

# MoveArm
rospack_add_executable(move_arm src/move_arm.cpp)
rospack_link_boost(move_arm thread)

# move_base_sbpl
rospack_add_executable(move_base_sbpl src/move_base_sbpl.cpp)
target_link_libraries(move_base_sbpl highlevel_controllers)
rospack_link_boost(move_base_sbpl thread)

# move_base_navfn
rospack_add_executable(move_base_navfn src/move_base_navfn.cpp)
target_link_libraries(move_base_navfn highlevel_controllers)
rospack_link_boost(move_base_navfn thread)

# move_base_navfn_constrained
rospack_add_executable(move_base_navfn_constrained src/move_base_navfn_constrained.cpp)
target_link_libraries(move_base_navfn_constrained highlevel_controllers)
rospack_link_boost(move_base_navfn_constrained thread)


# move_base_follow
rospack_add_executable(move_base_follow src/move_base_follow.cpp)
target_link_libraries(move_base_follow highlevel_controllers)
rospack_link_boost(move_base_follow thread)

# MoveArmEndEffector
rospack_add_executable(move_end_effector src/move_end_effector.cpp)
rospack_link_boost(move_end_effector thread)

#Joystick Sleep Button
rospack_add_executable(joy_batt_sender src/joy_batt_sender.cpp)

# PlugIn and Unplug
rospack_add_executable(recharge_controller src/recharge_controller.cpp)

# General container of little actions
rospack_add_executable(action_container src/action_container.cpp)
target_link_libraries(action_container robot_actions)

#CLI
# Disabled, pending update to new message formats
rospack_add_executable(cli src/control_cli.cpp)
rospack_link_boost(cli thread)
