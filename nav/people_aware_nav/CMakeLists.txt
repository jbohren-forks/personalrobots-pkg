cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(people_aware_nav)
rospack_add_executable(bin/person_pos_sender src/person_node.cpp)
rospack_add_lisp_executable(bin/fake_person src/fake-person.lisp)

