cmake_minimum_required(VERSION 2.6)
set(CMAKE_MODULE_PATH $ENV{ROS_ROOT}/core/rosbuild)
include(rosbuild)
project(deadreckon)
rospack()
gensrv()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
# the service provider
rospack_add_executable(deadreckon deadreckon.cpp)
# a dumb test client
rospack_add_executable(test_deadreckon_service test_deadreckon_service.cpp)
