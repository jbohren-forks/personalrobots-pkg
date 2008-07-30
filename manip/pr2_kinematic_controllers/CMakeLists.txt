cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(pr2_kinematic_controllers)

gensrv()
add_definitions(-Wall)
include_directories(srv/cpp)

rospack_add_executable(bin/pr2_kinematic_controllers src/pr2_kinematic_controllers.cc)
rospack_add_executable(bin/test_pr2_kin_con test/test_pr2_kin_con.cc)


