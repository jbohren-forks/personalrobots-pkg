cmake_minimum_required(VERSION 2.6)
include(rosbuild)
add_definitions(-Wall)

rospack(highlevel_controllers)

# MoveArm
rospack_add_executable(move_arm src/MoveArm.cpp)

# MoveBase
rospack_add_library(highlevel_controllers src/MoveBase.cpp)
rospack_add_executable(move_base_sbpl src/move_base_sbpl.cpp)
target_link_libraries(move_base_sbpl highlevel_controllers)
target_link_libraries(highlevel_controllers costmap_2d)
