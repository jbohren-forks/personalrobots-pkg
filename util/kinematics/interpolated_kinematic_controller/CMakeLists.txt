cmake_minimum_required(VERSION 2.6)
include(rosbuild)
add_definitions(-Wall)
rospack(interpolated_kinematic_controller)

rospack_add_executable(bin/interpolated_kinematic_controller
src/interpolated_kinematic_controller.cc)
rospack_add_executable(bin/test_inter_kin_con test/test_inter_kin_con.cc)


