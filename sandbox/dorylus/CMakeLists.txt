cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rospack(dorylus)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rospack_add_library(dorylus dorylus.cpp)
rospack_add_executable(eigentest eigentest.cpp)

rospack_download_test_data(http://pr.willowgarage.com/data/dorylus/sanity.d test/sanity.d)
rospack_download_test_data(http://pr.willowgarage.com/data/dorylus/sanity.dd test/sanity.dd)
rospack_download_test_data(http://pr.willowgarage.com/data/dorylus/test.dd test/test.dd)
rospack_download_test_data(http://pr.willowgarage.com/data/dorylus/test.d test/test.d)
rospack_add_gtest(test/test_dorylus test/test_dorylus.cpp)
target_link_libraries(test/test_dorylus dorylus)
