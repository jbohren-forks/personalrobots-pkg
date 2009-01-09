cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions(-Wall)
rospack(openraveros)
genmsg()
gensrv()
rospack_add_executable(openraveros src/openraveros.cpp)
target_link_libraries (openraveros openrave-core boost_thread)

# add sessions dependency
find_ros_package(roscpp_sessions)
set_source_files_properties(src/openraveros.cpp PROPERTIES OBJECT_DEPENDS ${roscpp_sessions_PACKAGE_PATH}/include/ros/session.h)
