cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(tf)

genmsg()

rospack_add_library(trajectory src/trajectory.cpp)

# Unit tests
rospack_add_gtest(utest test/utest.cpp)
set_target_properties(utest PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(utest trajectory)
