cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(outlet_detection)

rospack_add_boost_directories()

add_definitions(-Wall -O3 -DNDEBUG)
#add_definitions(-Wall -O0 -g)
include_directories(cvblobslib)

rospack_add_library(blob cvblobslib/Blob.cpp cvblobslib/BlobResult.cpp cvblobslib/BlobExtraction.cpp)
rospack_add_executable(bin/orange_detect src/orange_detect.cpp)
target_link_libraries(bin/orange_detect blob)
