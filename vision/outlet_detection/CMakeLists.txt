cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(outlet_detection)

rospack_add_boost_directories()

add_definitions(-Wall -O3 -DNDEBUG)
#add_definitions(-Wall -O0 -g)
include_directories(cvblobslib)
#include_directories(src/keypoint)

rospack_add_library(outlet src/outlet_model.cpp src/planar.cpp src/outlet_tuple.cpp src/outlet_detector.cpp src/learning.cpp)
rospack_add_executable(outlet_model src/main.cpp)
target_link_libraries(outlet_model outlet)
rospack_add_executable(outlet_sample src/sample.cpp)
target_link_libraries(outlet_sample outlet)
add_definitions(-D_OUTLET_HR -D_TUNING -D_VERBOSE -D_USE_OUTLET_TUPLE)

rospack_add_executable(bin/useChessBoard src/useChessBoard.cpp)
rospack_add_executable(bin/templateMatching src/templateMatching.cpp)
rospack_add_library(blob cvblobslib/Blob.cpp cvblobslib/BlobResult.cpp cvblobslib/BlobExtraction.cpp)
rospack_add_executable(bin/orange_detect src/orange_detect.cpp)
target_link_libraries(bin/orange_detect blob)
