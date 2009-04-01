cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(new_costmap)
genmsg()

#rospack_add_boost_directories()

rospack_add_library(new_costmap_2d src/costmap_2d.cpp)
#rospack_link_boost(costmap_2d thread)

rospack_add_executable(costmap_test src/costmap_test.cpp)
target_link_libraries(costmap_test new_costmap_2d)
