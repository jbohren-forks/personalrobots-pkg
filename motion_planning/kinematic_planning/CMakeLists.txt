cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(kinematic_planning)
rospack_add_executable(kinematic_planning src/kinematic_planning.cpp)
rospack_add_executable(motion_validator src/motion_validator.cpp)
rospack_add_link_flags(kinematic_planning)
rospack_add_link_flags(motion_validator)

add_subdirectory(test)
