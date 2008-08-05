cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(math_utils)
rospack_add_library(math_utils src/MathExpression.cpp)

# Unit tests
rospack_add_gtest(math_utils_unittest test/math_utils_unittest.cpp)
set_target_properties(math_utils_unittest PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(math_utils_unittest math_utils)
