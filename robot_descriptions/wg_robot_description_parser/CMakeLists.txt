cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(wg_robot_description_parser)
rospack_add_library(URDF src/urdf/URDF.cpp)
rospack_add_executable(parse src/urdf/parse.cpp)
target_link_libraries(parse URDF)
rospack_add_executable(merge src/urdf/merge.cpp)

# Tests
rospack_add_gtest(test_cpp_parser test/test_cpp_parser.cpp)
target_link_libraries(test_cpp_parser URDF)
