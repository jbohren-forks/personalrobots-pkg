cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(phase_space)
include(CMakeDetermineSystem)

# For now, we only have a 32-bit Linux version of libowlsock
# TODO: more thoroughly test and wrap up this 32-bit check
if(CMAKE_SYSTEM_PROCESSOR MATCHES "i686" OR
   CMAKE_SYSTEM_PROCESSOR MATCHES "i386" OR
   CMAKE_SYSTEM_PROCESSOR MATCHES "unknown")
  set(_is32bit TRUE)
else(CMAKE_SYSTEM_PROCESSOR MATCHES "i686" OR
     CMAKE_SYSTEM_PROCESSOR MATCHES "i386" OR
     CMAKE_SYSTEM_PROCESSOR MATCHES "unknown")
  set(_is32bit FALSE)
endif(CMAKE_SYSTEM_PROCESSOR MATCHES "i686" OR
      CMAKE_SYSTEM_PROCESSOR MATCHES "i386" OR
      CMAKE_SYSTEM_PROCESSOR MATCHES "unknown")

if(CMAKE_SYSTEM_NAME MATCHES "Linux" AND _is32bit)

  link_directories(${PROJECT_SOURCE_DIR}/lib)
  rospack_add_executable(phase_space_node phase_space_node.cpp)
  #rospack_add_executable(phase_space_benchmark phase_space_benchmark.cpp)

  target_link_libraries(phase_space_node owlsock)
  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
else(CMAKE_SYSTEM_NAME MATCHES "Linux" AND _is32bit)
  message("WARNING: not building phase_space because this is not a 32-bit Linux system")
endif(CMAKE_SYSTEM_NAME MATCHES "Linux" AND _is32bit)
