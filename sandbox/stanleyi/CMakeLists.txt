cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

set(ROS_BUILD_TYPE Debug)
rospack(stanleyi)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rospack_add_executable(stanleyi stanleyi.cpp)

include(CheckIncludeFiles)
check_include_files(google/profiler.h HAVE_GOOGLE_PROFILER_H)
if(HAVE_GOOGLE_PROFILER_H)
  message("Found libgoogle-perftools")	
#  rospack_add_link_flags(stanleyi "-lprofiler")	
else(HAVE_GOOGLE_PROFILER_H)
  message("No libgoogle-perftools; can't profile.")	
endif(HAVE_GOOGLE_PROFILER_H)

#rospack_add_executable(test_cv test_cv.cpp)
#rospack_add_executable(demhist demhist.c)
#rospack_add_executable(ex7-1 ex7-1.cpp)
