cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rospack(descriptors_2d_gpl)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#genmsg()
#uncomment if you have defined services
#gensrv()

#common commands for building c++ executables and libraries
#rospack_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)
#rospack_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


rospack_add_library(descriptors_2d_gpl descriptors_2d_gpl.cpp)


rospack_download_test_data(http://pr.willowgarage.com/data/descriptors_2d/frame0000.jpg test/frame0000.jpg)
rospack_download_test_data(http://pr.willowgarage.com/data/descriptors_2d/correct-output/daisy.results test/correct-output/daisy.results)

rospack_add_gtest(test/test_descriptors_2d_gpl test/test_descriptors_2d_gpl.cpp)
target_link_libraries(test/test_descriptors_2d_gpl descriptors_2d_gpl)
