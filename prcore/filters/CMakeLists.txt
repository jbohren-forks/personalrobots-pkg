cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(filters)

rospack_add_boost_directories()

rospack_add_gtest(median_test test/test_median.cpp)

# broken, needs to go to the new form factor
rospack_add_gtest(transfer_function_test test/test_transfer_function.cpp)


rospack_add_gtest(mean_test test/test_mean.cpp)

rospack_add_gtest(factory_test test/test_factory.cpp)
rospack_add_gtest(chain_test test/test_chain.cpp)
rospack_add_gtest(realtime_buffer_test test/test_realtime_circular_buffer.cpp)
