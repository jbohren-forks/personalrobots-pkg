cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(kinematic_planning)

rospack_add_boost_directories()

rospack_add_executable(kinematic_planning src/kinematic_planning.cpp)
rospack_add_executable(motion_validator src/motion_validator.cpp)
rospack_add_executable(state_validity_monitor src/state_validity_monitor.cpp)
rospack_add_link_flags(kinematic_planning)
rospack_add_link_flags(motion_validator)
rospack_add_link_flags(state_validity_monitor)

rospack_link_boost(kinematic_planning thread)
rospack_link_boost(motion_validator thread)
rospack_link_boost(state_validity_monitor thread)

add_subdirectory(test)
