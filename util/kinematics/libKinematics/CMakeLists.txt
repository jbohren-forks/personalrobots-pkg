cmake_minimum_required(VERSION 2.6)
include(rosbuild)
add_definitions(-Wall)
rospack(libKinematics)
rospack_add_library(libKinematics src/pr2_ik.cpp src/kinematics.cpp)

rospack_add_executable(libKinematicsTest src/test/test_PR2ik.cpp)
target_link_libraries(libKinematicsTest libKinematics)


