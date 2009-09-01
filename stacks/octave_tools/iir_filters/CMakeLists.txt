cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(iir_filters)
genmsg()
gensrv()

add_subdirectory(test)

rospack_add_boost_directories()

rospack_add_library(iir_filters
                    src/filter_coeff_client.cpp)

rospack_add_executable(filter_coeff_client
                       src/filter_coeff_client.cpp)



