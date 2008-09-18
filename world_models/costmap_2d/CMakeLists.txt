cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(costmap_2d)
include_directories(${PROJECT_SOURCE_DIR}/	include)
rospack_add_library(costmap_2d src/costmap_2d.cpp)
#rospack_add_executable(bin/costmap_2d_ros src/costmap_2d_ros.cpp)
#target_link_libraries(bin/costmap_2d_ros costmap_2d)

# Test target for module tests to be included in gtest regression test harness
rospack_add_gtest(utest src/test/module-tests.cc)
target_link_libraries(utest costmap_2d)
