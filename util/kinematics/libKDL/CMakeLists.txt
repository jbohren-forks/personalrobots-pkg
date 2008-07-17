cmake_minimum_required(VERSION 2.6)
include(rosbuild)
add_definitions(-Wall)
rospack(libKDL)
rospack_add_library(KDL src/kdl_kinematics.cpp)

rospack_add_executable(bin/pr2_kin_test test/pr2_kin_test.cpp)
target_link_libraries(bin/pr2_kin_test KDL orocos-kdl)

#rospack_add_executable(bin/dynamics test/dynamics.cpp)
#target_link_libraries(bin/dynamics KDL orocos-kdl)

