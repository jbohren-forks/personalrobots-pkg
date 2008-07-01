cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(deadreckon)
gensrv()
# the service provider
rospack_add_executable(deadreckon deadreckon.cpp)
# a dumb test client
rospack_add_executable(test_deadreckon_service test_deadreckon_service.cpp)
