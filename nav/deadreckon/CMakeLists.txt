cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(deadreckon)
gensrv()
# the service provider
rospack_add_executable(deadreckon deadreckon.cpp)
# a dumb test client
rospack_add_executable(test_deadreckon_service test_deadreckon_service.cpp)
