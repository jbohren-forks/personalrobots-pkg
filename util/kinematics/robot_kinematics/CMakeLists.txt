cmake_minimum_required(VERSION 2.6)
include(rosbuild)
add_definitions(-Wall)
rospack(robot_kinematics)
rospack_add_library(robot_kinematics src/robot_kinematics.cpp src/serial_chain.cpp)

find_library(HAVE_OROCOS_KDL orocos-kdl)
if(NOT HAVE_OROCOS_KDL)
  message(FATAL_ERROR "Can't proceed without liborocos-kdl;  on Ubuntu, you should install the liborocos-kdl-dev package")
endif(NOT HAVE_OROCOS_KDL)

rospack_add_executable(bin/robot_kinematics_test test/robot_kinematics_test.cpp)
target_link_libraries(bin/robot_kinematics_test robot_kinematics orocos-kdl)


