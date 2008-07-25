cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(math_utils)
rospack_add_library(math_utils src/MathExpression.cpp)

# Unit tests
rospack_add_gtest(test/math_utils_unittest test/math_utils_unittest.cpp)
target_link_libraries(test/math_utils_unittest math_utils)
