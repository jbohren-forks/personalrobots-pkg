cmake_minimum_required(VERSION 2.6)
include(rosbuild)
add_definitions(-Wall)

rospack(highlevel_controllers)

rospack_add_executable(move_arm
	src/MoveArm.cpp)
