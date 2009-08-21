cmake_minimum_required(VERSION 2.6)
#set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rospack_add_boost_directories()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


rosbuild_genmsg()

add_subdirectory(src)
