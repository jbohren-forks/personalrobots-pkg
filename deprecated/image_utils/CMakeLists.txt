cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(image_utils)
rospack_add_library(image_utils
                    src/libimage_utils/image_utils.cpp)
rospack_add_executable(src/create_chessboard/create_chessboard
                       src/create_chessboard/create_chessboard.cpp)
target_link_libraries(src/create_chessboard/create_chessboard image_utils)
