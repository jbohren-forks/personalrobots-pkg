cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(gazebo_plugin)
rospack_add_library(Pr2_ActArray src/Pr2_Actarray.cc)
rospack_add_library(Pr2_Gripper src/Pr2_Gripper.cc)
rospack_add_library(P3D src/P3D.cc)
# TODO: remove this hardcoded dependency
include_directories(/usr/include/libxml2)

