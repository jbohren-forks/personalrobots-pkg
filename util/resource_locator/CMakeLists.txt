cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(resource_locator)
rospack_add_library(resource_locator src/reslocator.cpp)
set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

target_link_libraries(resource_locator -lboost_regex-mt)
