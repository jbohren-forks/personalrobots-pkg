cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rospack(descriptors_2d)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rospack_download_test_data(http://pr.willowgarage.com/data/descriptors_2d/frame0000.jpg test/frame0000.jpg)
rospack_download_test_data(http://pr.willowgarage.com/data/descriptors_2d/correct-output/hog.results test/correct-output/hog.results)
rospack_download_test_data(http://pr.willowgarage.com/data/descriptors_2d/correct-output/surf.results test/correct-output/surf.results)
rospack_download_test_data(http://pr.willowgarage.com/data/descriptors_2d/correct-output/sch.results test/correct-output/sch.results)

rospack_add_library(descriptors_2d src/descriptors_2d.cpp)
rospack_add_executable(example src/example.cpp)
target_link_libraries(example descriptors_2d)
rospack_add_executable(hist_test src/hist_test.cpp)
rospack_add_executable(surf_test src/surf_test.cpp)

# This unit test fails on 64-bit, #2141
include(CMakeDetermineSystem)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "i686" OR
   CMAKE_SYSTEM_PROCESSOR MATCHES "i386" OR
   CMAKE_SYSTEM_PROCESSOR MATCHES "unknown")
rospack_add_gtest(test/test_descriptors_2d test/test_descriptors_2d.cpp)
rosbuild_add_gtest_build_flags(test/test_descriptors_2d)
target_link_libraries(test/test_descriptors_2d descriptors_2d)
endif(CMAKE_SYSTEM_PROCESSOR MATCHES "i686" OR
      CMAKE_SYSTEM_PROCESSOR MATCHES "i386" OR
      CMAKE_SYSTEM_PROCESSOR MATCHES "unknown")
