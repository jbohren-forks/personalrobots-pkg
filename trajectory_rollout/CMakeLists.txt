cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(trajectory_rollout)

rospack_add_library(trajectory_rollout src/map_cell.cpp src/map_grid.cpp src/trajectory_controller.cpp src/trajectory.cpp)
rospack_add_executable(tests src/tests.cpp src/map_cell.cpp src/map_grid.cpp src/trajectory_controller.cpp src/trajectory.cpp)
