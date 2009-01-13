cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(trajectory_rollout)
genmsg()

#uncomment for profiling
#set(ROS_COMPILE_FLAGS "-g -pg" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g -pg" ${ROS_LINK_FLAGS})
#set(ROS_COMPILE_FLAGS "-g" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g" ${ROS_LINK_FLAGS})

rospack_add_library(trajectory_rollout src/map_cell.cpp src/map_grid.cpp src/trajectory_controller.cpp 
    src/trajectory.cpp src/governor_node.cpp)
target_link_libraries(trajectory_rollout costmap_2d)

rospack_add_executable(governor_node src/map_cell.cpp src/map_grid.cpp src/trajectory_controller.cpp 
    src/trajectory.cpp src/governor_node.cpp)

target_link_libraries(governor_node costmap_2d)

rospack_add_gtest(utest test/utest.cpp src/map_cell.cpp src/map_grid.cpp src/trajectory_controller.cpp src/trajectory.cpp)
set_target_properties(utest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(utest costmap_2d)
