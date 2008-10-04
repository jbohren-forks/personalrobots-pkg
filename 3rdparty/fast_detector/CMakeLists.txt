cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(fast_detector)
rospack_add_library(fast_detector src/fast_10.c src/fast_11.c src/fast_12.c src/fast_9.c src/nonmax.c)

find_package(PythonLibs)
if(NOT PYTHONLIBS_FOUND)
  message(FATAL "Cannot build without Python development libs")
endif(NOT PYTHONLIBS_FOUND)

include_directories(${PYTHON_INCLUDE_PATH})
link_libraries(${PYTHON_LIBRARIES})

rospack_add_library(pyfast_detector src/pyfast.c src/fast_10.c src/fast_11.c src/fast_12.c src/fast_9.c src/nonmax.c)
set_target_properties(pyfast_detector PROPERTIES OUTPUT_NAME fast PREFIX "")

rospack_add_pyunit(test/directed.py)
