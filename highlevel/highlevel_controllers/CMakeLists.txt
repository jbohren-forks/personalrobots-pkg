cmake_minimum_required(VERSION 2.6)
include(rosbuild)
add_definitions(-Wall)
rospack(highlevel_controllers)

genmsg()

# Library
#link_directories(/wg/stor1/mcgann/pr/trajectory_rollout/lib)
#link_directories(/wg/stor1/eitan/ros/ros-pkg/trajectory_rollout/lib)
rospack_add_library(highlevel_controllers src/MoveBase.cpp src/VelocityControllers.cc)
target_link_libraries(highlevel_controllers trajectory_rollout costmap_2d)

# MoveArm
rospack_add_executable(move_arm src/MoveArm.cpp)

# MoveBase
rospack_add_executable(move_base_sbpl src/move_base_sbpl.cpp)
target_link_libraries(move_base_sbpl highlevel_controllers)

# MoveArmEndEffector
rospack_add_executable(move_end_effector src/MoveEndEffector.cpp)

# PlugIn and Unplug
rospack_add_executable(plug_in src/PlugInController.cpp)
rospack_add_executable(recharge_controller src/RechargeController.cpp)

#CLI
rospack_add_executable(cli src/ControlCli.cpp)

