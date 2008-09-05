cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(trajectory_rollout)

rospack_add_library(trajectory_rollout src/map_cell.cpp src/map_grid.cpp src/trajectory_controller.cpp src/trajectory.cpp)
rospack_add_gtest(utest test/utest.cpp src/map_cell.cpp src/map_grid.cpp src/trajectory.cpp src/trajectory_controller.cpp)
set_target_properties(utest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
