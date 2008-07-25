cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(collision_space)
rospack_add_library(CollisionSpace src/collision_space/environment.cpp src/collision_space/environmentODE.cpp)
