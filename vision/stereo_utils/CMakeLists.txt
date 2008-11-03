cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(stereo_utils)

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

rospack_add_library(stereo_utils src/CvStereoCamModel.cpp)


# Unit tests
rospack_add_gtest(utest test/utest.cpp)
set_target_properties(utest PROPERTIES
			    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(utest stereo_utils)
