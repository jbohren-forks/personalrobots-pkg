cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(pytoro)
genmsg()

rospack_add_boost_directories()

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

rospack_add_library(pytoro src/py.cpp)

set_target_properties(pytoro PROPERTIES OUTPUT_NAME pytoro PREFIX "")
rospack_add_compile_flags(pytoro -g -O3 -Wno-missing-field-initializers )

# unit tests
rospack_add_pyunit(test/directed.py)
