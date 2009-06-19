cmake_minimum_required(VERSION 2.4.6)
# Unit test fails when build type changes to RelWithDebInfo (-O2)
set(ROS_BUILD_TYPE RelWithDebInfo)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(stereo_utils)

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

rospack_add_library(stereo_utils src/CvStereoCamModel.cpp)

rospack_add_library(stereo_utils_lowlevel_py src/lowlevel.cpp src/stereolib.cpp src/ost_stereolib.cpp)

set_target_properties(stereo_utils_lowlevel_py PROPERTIES OUTPUT_NAME stereo_utils_lowlevel PREFIX "_")
#rospack_add_compile_flags(stereo_utils_lowlevel_py -g -O3 -Wno-missing-field-initializers -msse3)
rospack_add_compile_flags(stereo_utils_lowlevel_py -g -Wno-missing-field-initializers -msse3)

# Unit tests
rospack_add_gtest(test/utest test/utest.cpp)
target_link_libraries(test/utest stereo_utils)
rospack_add_pyunit(test/directed.py)
