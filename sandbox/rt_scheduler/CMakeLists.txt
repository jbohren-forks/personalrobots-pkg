cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(rt_scheduler)

#rospack_add_library(rt_scheduler                     )

rospack_add_executable(run_test src/run_test.cpp)
#target_link_libraries(run_test rt_scheduler)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)