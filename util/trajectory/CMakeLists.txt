cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(trajectory)

genmsg()

rospack_add_library(trajectory src/trajectory.cpp)

# Unit tests
rospack_add_gtest(utest test/utest.cpp)
set_target_properties(utest PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(utest trajectory)


rospack_add_gtest(utestBlended test/utestBlended.cpp)
set_target_properties(utestBlended PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(utestBlended trajectory)

rospack_add_gtest(utestCubic test/utestCubic.cpp)
set_target_properties(utestCubic PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(utestCubic trajectory)
