cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rospack(dcam)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(src)

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})
rospack_add_library(dcam_py src/python/dcam.cpp)
set_target_properties(dcam_py PROPERTIES OUTPUT_NAME dcam PREFIX "")
rospack_add_compile_flags(dcam_py -g -O3 -Wno-missing-field-initializers -msse3)
target_link_libraries(dcam_py dcam)
