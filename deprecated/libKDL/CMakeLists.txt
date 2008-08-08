cmake_minimum_required(VERSION 2.6)
include(rosbuild)
add_definitions(-Wall)
rospack(libKDL)
rospack_add_library(KDL src/kdl_kinematics.cpp)

#find_library(HAVE_OROCOS_KDL orocos-kdl)
#if(NOT HAVE_OROCOS_KDL)
#  message(FATAL_ERROR "Can't proceed without liborocos-kdl;  on Ubuntu, you should install the liborocos-kdl-dev package")
#endif(NOT HAVE_OROCOS_KDL)
rospack_add_executable(bin/pr2_kin_test test/pr2_kin_test.cpp)
target_link_libraries(bin/pr2_kin_test KDL)

rospack_add_executable(bin/dynamics test/dynamics.cpp)
target_link_libraries(bin/dynamics KDL)

