cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

set(ROS_BUILD_TYPE Release)

rospack(find_bottles)

rospack_add_boost_directories()

rospack_add_executable(find_bottle src/find_bottle.cpp)
rospack_add_openmp_flags(find_bottle)
#target_link_libraries(find_bottle find_bottles)

#rospack_add_executable(find_trash src/find_trash.cpp)
#rospack_add_openmp_flags(find_trash)
#target_link_libraries(find_trash find_bottles)

gensrv()
