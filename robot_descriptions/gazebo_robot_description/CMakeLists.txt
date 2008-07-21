cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(gazebo_robot_description)
rospack_add_executable(urdf2gazebo src/urdf2gazebo.cpp)
