cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(filters)
rospack_add_library(standard_filter src/mean.cpp )
rospack_add_gtest(median_test test/test_median.cpp)
target_link_libraries(median_test standard_filter)

#rospack_add_gtest(mean_test test/test_mean.cpp)
#target_link_libraries(mean_test standard_filter)