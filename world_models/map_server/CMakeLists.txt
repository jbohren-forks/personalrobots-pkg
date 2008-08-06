cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
include(rosbuild)
rospack(map_server)
rospack_add_executable(map_server src/image_loader.cpp src/main.cpp)
rospack_add_gtest(utest test/utest.cpp test/test_constants.cpp src/image_loader.cpp)
set_target_properties(utest PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

rospack_add_executable(rtest test/rtest.cpp test/test_constants.cpp)
set_target_properties(rtest PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
