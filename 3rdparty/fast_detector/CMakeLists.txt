cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(fast_detector)
rospack_add_library(fast_detector src/fast_10.c src/fast_11.c src/fast_12.c src/fast_9.c src/nonmax.c)

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

rospack_add_library(pyfast_detector src/pyfast.c src/fast_10.c src/fast_11.c src/fast_12.c src/fast_9.c src/nonmax.c)
set_target_properties(pyfast_detector PROPERTIES OUTPUT_NAME fast PREFIX "")

rospack_add_pyunit(test/directed.py)
