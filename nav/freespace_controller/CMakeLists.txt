cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(freespace_controller)
genmsg()

#uncomment for profiling
#set(ROS_COMPILE_FLAGS "-g -pg" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g -pg" ${ROS_LINK_FLAGS})
#set(ROS_COMPILE_FLAGS "-g" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g" ${ROS_LINK_FLAGS})

rospack_add_library(freespace_controller src/point_grid.cpp src/costmap_model.cpp src/freespace_controller.cpp src/trajectory_controller.cpp)
target_link_libraries(freespace_controller costmap_2d)

rospack_add_executable(point_grid src/point_grid.cpp)
target_link_libraries(point_grid costmap_2d)

#rospack_add_gtest(utest test/utest.cpp src/map_cell.cpp src/map_grid.cpp src/trajectory_controller.cpp src/trajectory.cpp)
#set_target_properties(utest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
#target_link_libraries(utest costmap_2d)
