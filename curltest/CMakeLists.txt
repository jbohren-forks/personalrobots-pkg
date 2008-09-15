cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(curltest)

find_library(curl REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(curltest curlnode.cpp curltest.cpp)
target_link_libraries(curltest curl)



