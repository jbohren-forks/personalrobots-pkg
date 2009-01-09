cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(filters)

rospack_add_gtest(median_test test/test_median.cpp)
rospack_add_gtest(transfer_function_test test/test_transfer_function.cpp)

target_link_libraries(median_test)
target_link_libraries(transfer_function_test)

#rospack_add_gtest(mean_test test/test_mean.cpp)
#target_link_libraries(mean_test)
