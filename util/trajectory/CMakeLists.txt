cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(trajectory)

genmsg()

rospack_add_library(trajectory src/trajectory.cpp)

# Unit tests
rospack_add_gtest(test/utest test/utest.cpp)
target_link_libraries(test/utest trajectory)


rospack_add_gtest(test/utestBlended test/utestBlended.cpp)
target_link_libraries(test/utestBlended trajectory)

rospack_add_gtest(test/utestCubic test/utestCubic.cpp)
target_link_libraries(test/utestCubic trajectory)
