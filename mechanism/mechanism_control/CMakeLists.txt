cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(mechanism_control)
#rospack_add_executable(mechanismControl src/base_control.cpp)
#target_link_libraries(mechanism_control etherdrive_hardware pr2Controllers mechanism_model)
rospack_add_library(mechanism_control src/base_control.cpp)

