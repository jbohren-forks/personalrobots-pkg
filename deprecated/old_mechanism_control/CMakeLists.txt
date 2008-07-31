cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(old_mechanism_control)
#rospack_add_executable(mechanismControl src/base_control.cpp)
#target_link_libraries(mechanism_control etherdrive_hardware pr2Controllers mechanism_model)
rospack_add_library(mechanism_control src/mechanism_control.cpp src/base_control.cpp)
rospack_add_library(single_control src/single_control.cpp)
rospack_add_library(gazebo_mechanism_control src/gazebo_control.cpp) # src/controllerFactory.cpp)

