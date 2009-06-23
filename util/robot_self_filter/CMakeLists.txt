cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

set(ROS_BUILD_TYPE Release)

rospack(robot_self_filter)

rospack_add_library(new_robot_self_filter src/self_mask.cpp)
rospack_add_openmp_flags(new_robot_self_filter)

rospack_add_executable(test_filter src/test_filter.cpp)
rospack_add_openmp_flags(test_filter)
target_link_libraries(test_filter new_robot_self_filter)

rospack_add_executable(self_filter src/self_filter.cpp)
rospack_add_openmp_flags(self_filter)
target_link_libraries(self_filter new_robot_self_filter)
