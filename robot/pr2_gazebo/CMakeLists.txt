cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(pr2_gazebo)
rospack_add_executable(pr2_gazebo src/pr2_gazebo.cpp)
