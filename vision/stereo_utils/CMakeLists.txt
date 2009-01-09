cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(stereo_utils)

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

rospack_add_library(stereo_utils src/CvStereoCamModel.cpp)


# Unit tests
rospack_add_gtest(test/utest test/utest.cpp)
target_link_libraries(test/utest stereo_utils)
