cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(angles)

# Unit tests
rospack_add_gtest(utest test/utest.cpp)
set_target_properties(utest PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(utest)
