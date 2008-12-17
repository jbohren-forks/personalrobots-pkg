cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(old_test_collision_space)

rospack_add_executable(test_kinematic_ode src/test_kinematic_ode.cpp)
