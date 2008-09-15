cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(trajectory_rollout)
genmsg()

#uncomment for profiling
set(ROS_COMPILE_FLAGS "-pg" ${ROS_COMPILE_FLAGS})
set(ROS_LINK_FLAGS "-pg" ${ROS_LINK_FLAGS})

rospack_add_library(trajectory_rollout src/map_cell.cpp src/map_grid.cpp src/trajectory_controller.cpp src/trajectory.cpp src/governor_node.cpp)
rospack_add_executable(governor_node src/map_cell.cpp src/map_grid.cpp src/trajectory_controller.cpp src/trajectory.cpp src/governor_node.cpp)
#rospack_add_gtest(utest test/utest.cpp src/map_cell.cpp src/map_grid.cpp src/trajectory.cpp src/trajectory_controller.cpp)
#set_target_properties(utest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
