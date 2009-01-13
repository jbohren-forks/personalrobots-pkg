cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(math_expr)
rospack_add_library(math_expr src/MathExpression.cpp)

# Unit tests
rospack_add_gtest(test/utest test/utest.cpp)
target_link_libraries(test/utest math_expr)
