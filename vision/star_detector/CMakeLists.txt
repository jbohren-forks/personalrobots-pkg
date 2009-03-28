cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include(CMakeDetermineSystem)
#set(ROS_BUILD_TYPE Release)
rospack(star_detector)
rospack_add_library(starfeature src/detector.cpp src/integral.cpp src/keypoint.cpp)
rospack_add_compile_flags(starfeature -save-temps)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "i686")
  rospack_add_compile_flags(starfeature -march=pentium3 -msse3)
endif(CMAKE_SYSTEM_PROCESSOR MATCHES "i686")

# Obsoleted by directed test below
#rospack_add_gtest(utest test/utest.cpp )
#set_target_properties(utest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
#target_link_libraries(utest starfeature)

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

rospack_add_library(pystarfeature src/pydetector.cpp)
target_link_libraries(pystarfeature starfeature)
set_target_properties(pystarfeature PROPERTIES OUTPUT_NAME starfeature PREFIX "")
rospack_add_compile_flags(pystarfeature -Wno-missing-field-initializers -save-temps)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "i686")
  rospack_add_compile_flags(pystarfeature -march=pentium3 -msse3)
endif(CMAKE_SYSTEM_PROCESSOR MATCHES "i686")

rospack_add_pyunit(test/directed.py)
