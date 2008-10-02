cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(star_detector)
add_definitions(-Wall -O3 -save-temps -march=pentium3 -msse3 -DNDEBUG)
rospack_add_library(starfeature src/detector.cpp src/integral.cpp src/keypoint.cpp)

# Obsoleted by directed test below
#rospack_add_gtest(utest test/utest.cpp )
#set_target_properties(utest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
#target_link_libraries(utest starfeature)

FIND_PACKAGE(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

rospack_add_library(pystarfeature src/pydetector.cpp src/detector.cpp src/integral.cpp src/keypoint.cpp)
set_target_properties(pystarfeature PROPERTIES OUTPUT_NAME starfeature PREFIX "")

rospack_add_pyunit(test/directed.py)
