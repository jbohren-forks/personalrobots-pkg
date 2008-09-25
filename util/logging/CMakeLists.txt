cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(logging)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(checklog src/checklog/checklog.cpp)
rospack_add_executable(demo src/demo/demo.cpp)