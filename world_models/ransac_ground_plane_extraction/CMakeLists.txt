cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(ransac_ground_plane_extraction)
genmsg()

rospack_add_library(ransac_ground_plane_extraction src/ransac_ground_plane_extraction.cpp src/ransac_ground_plane_extraction_node.cpp)

# Test target for module tests to be included in gtest regression test harness
#rospack_add_gtest(utest src/test/module-tests.cc)
#target_link_libraries(utest costmap_2d)

# Target for benchmarking the costmap
rospack_add_executable(ransacnode src/ransac_ground_plane_extraction_launch_node.cpp )
target_link_libraries(ransacnode ransac_ground_plane_extraction)
