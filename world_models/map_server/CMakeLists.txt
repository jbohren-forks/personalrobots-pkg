cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
include(rosbuild)
rospack(map_server)
rospack_add_executable(map_server src/map_server.cpp src/main.cpp)
rospack_add_gtest(utest test/test.cpp src/map_server.cpp)
set_target_properties(utest PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
