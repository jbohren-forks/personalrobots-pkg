cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(filters)

rospack_add_boost_directories()

#Plugins
rospack_add_library(mean src/mean.cpp)
rospack_add_library(median src/median.cpp)
rospack_add_library(transfer_function src/transfer_function.cpp)

# Test median filter
rospack_add_gtest(median_test test/test_median.cpp)
target_link_libraries(median_test median) 

#test transfer function filter
rospack_add_gtest(transfer_function_test test/test_transfer_function.cpp)
target_link_libraries(transfer_function_test transfer_function)

#test mean filter
rospack_add_gtest(mean_test test/test_mean.cpp)
target_link_libraries(mean_test mean)

#Test plugin loading into filter chain
rospack_add_gtest(chain_test test/test_chain.cpp)

# Test reltime safe buffer class
rospack_add_gtest(realtime_buffer_test test/test_realtime_circular_buffer.cpp)
