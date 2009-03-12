cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(visual_nav)
gensrv()

rospack_add_boost_directories()

set(ROS_COMPILE_FLAGS "-O3 " ${ROS_COMPILE_FLAGS})
#uncomment for profiling
set(ROS_COMPILE_FLAGS "-g -pg " ${ROS_COMPILE_FLAGS})
set(ROS_LINK_FLAGS "-g -pg " ${ROS_LINK_FLAGS})

rospack_add_library(visual_nav src/visual_nav.cpp src/transform.cpp)

rospack_add_gtest(test/test_visual_nav test/test_visual_nav.cpp)
target_link_libraries(test/test_visual_nav visual_nav)

rospack_add_executable(bin/ros_visual_nav src/ros_visual_nav.cpp)
target_link_libraries(bin/ros_visual_nav visual_nav)
rospack_link_boost(bin/ros_visual_nav filesystem)
rospack_link_boost(bin/ros_visual_nav program_options)

