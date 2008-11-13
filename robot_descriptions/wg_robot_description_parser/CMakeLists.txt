cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(wg_robot_description_parser)
set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

rospack_add_library(URDF src/URDF.cpp src/parser.cpp)
rospack_add_compile_flags(URDF -Wextra)

rospack_add_executable(parse src/parse.cpp)
target_link_libraries(parse URDF)

rospack_add_executable(merge src/merge.cpp)

# Tests
rospack_add_gtest(test_cpp_parser test/test_cpp_parser.cpp)
target_link_libraries(test_cpp_parser URDF)
