cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(wg_robot_description)

find_ros_package(wg_robot_description)

add_custom_target(ALL DEPENDS)
