cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(costmap_2d)
genmsg()

#rospack_add_boost_directories()

rospack_add_library(costmap_2d src/costmap_2d.cpp src/observation_buffer.cpp src/costmap_2d_ros.cpp src/costmap_2d_publisher.cpp src/rate.cpp)
#rospack_link_boost(costmap_2d thread)

rospack_add_executable(costmap_test src/costmap_2d_ros.cpp)
target_link_libraries(costmap_test costmap_2d)
