cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE Release)
rospack(trex_ros)

genmsg()
gensrv()

rospack_add_boost_directories()

set(TREX_FILES src/main.cpp
               src/ros_reactor.cpp
	       src/executive.cpp
	       src/monitor.cpp
	       src/ros_adapter.cpp 
	       src/logger.cpp 
               src/logclock.cpp 
	       src/calc_angle_diff_constraint.cpp 
	       src/calc_distance_constraint.cpp 
	       src/components.cpp
	       src/adapter_utilities.cpp)

# trexdebug builds with a large number of run-time error checking running which is expensive
# but gives good feedback in discovering problems.
rospack_add_library(trex_ros ${TREX_FILES})

# trexwatchdog is a safety process that will monitor pings from the executive
# and dispense shutdown commands to highlevel controllers (deactivation) if pings are
# not received in time
rospack_add_executable(bin/trexwatchdog src/watch_dog.cpp)
