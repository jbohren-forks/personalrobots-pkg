cmake_minimum_required(VERSION 2.6)
include(rosbuild)
add_definitions(-Wall)
rospack(libpr2API)
#set(ROS_BUILD_STATIC_EXES true)
rospack_add_library(pr2API src/pr2API.cpp)
rospack_add_executable(test_libpr2API src/test/test_pr2API.cpp)
rospack_add_executable(torque_test src/test/torque_test.cpp)
rospack_add_executable(pick_object src/test/pick_object.cpp)
rospack_add_executable(test_kin_controllers src/test/test_kin_controllers.cpp src/kinematic_controllers.cpp)
rospack_add_executable(pr2_kin_kdl src/test/pr2_kin_kdl.cpp)
rospack_add_executable(test_FK src/test/test_FK.cpp)


