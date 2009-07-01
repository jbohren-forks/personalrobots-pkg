cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(message_filters)

rospack_add_executable(filter_example src/filter_example.cpp)
rospack_add_executable(tf_example src/tf_example.cpp)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(test EXCLUDE_FROM_ALL)
