cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set (ROS_BUILD_TYPE Release)
rospack(robot_voxelizer)

# Build executables in the bin directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_library(robot_voxelizer
																		src/robot_voxelizer.cpp
									 )

rospack_add_executable(robot_voxelizer_node src/robot_voxelizer.cpp)