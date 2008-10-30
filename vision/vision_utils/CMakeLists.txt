cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(vision_utils)

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

rospack_add_executable(vision_bag2tiff.py src/vision_bag2tiff.py)
