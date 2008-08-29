cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(imu_node)
rospack_add_executable(imu_node imu_node.cc)
