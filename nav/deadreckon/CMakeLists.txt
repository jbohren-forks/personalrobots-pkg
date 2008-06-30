cmake_minimum_required(VERSION 2.6)
set(CMAKE_MODULE_PATH $ENV{ROS_ROOT}/core/rosbuild)
include(rosbuild)
project(deadreckon)
rospack()
gensrv()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
# the service provider
add_executable(deadreckon deadreckon.cpp)
target_link_libraries(deadreckon ${deadreckon_LIBRARIES})
# a dumb test client
add_executable(test_deadreckon_service test_deadreckon_service.cpp)
target_link_libraries(test_deadreckon_service ${deadreckon_LIBRARIES})
