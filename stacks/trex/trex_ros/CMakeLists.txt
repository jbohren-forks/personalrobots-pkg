cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE Release)
rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()

rospack_add_boost_directories()

find_ros_package(trex_ros)
include(${trex_ros_PACKAGE_PATH}/trex_build.cmake)
create_nddl_config()



set(TREX_FILES src/adapters.cpp
               src/main.cpp
               src/ros_reactor.cpp
               src/ros_db_listener.cpp
	       src/executive.cpp
	       src/monitor.cpp
	       src/ros_adapter.cpp 
	       src/logger.cpp 
               src/logclock.cpp 
	       src/components.cpp
	       src/adapter_utilities.cpp)

create_trex_lib(trex_ros TREX_FILES)

rosbuild_add_gtest_build_flags(trex_ros_o)
if($ENV{ROS_TREX_DEBUG} MATCHES 1)
   rosbuild_add_gtest_build_flags(trex_ros_g)
endif($ENV{ROS_TREX_DEBUG} MATCHES 1)

# Implementation of actions 
rosbuild_add_executable(bin/actions src/actions.cpp)
