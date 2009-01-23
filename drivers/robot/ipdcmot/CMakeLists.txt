cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(ipdcmot)

rospack_add_boost_directories()

rospack_add_library(ipdcmot src/libipdcmot/ipdcmot.cpp)
rospack_link_boost(ipdcmot thread)

rospack_add_executable(test/test_patrol/test_patrol test/test_patrol/test_patrol.cpp)
target_link_libraries(test/test_patrol/test_patrol ipdcmot)
rospack_add_executable(test/test_position/test_position test/test_position/test_position.cpp)
target_link_libraries(test/test_position/test_position ipdcmot)
