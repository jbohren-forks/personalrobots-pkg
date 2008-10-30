cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(stereo_utils)

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

rospack_add_library(stereo_utils src/CvStereoCamModel.cpp)


