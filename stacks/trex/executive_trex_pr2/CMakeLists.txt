cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE Release)
rospack(executive_trex_pr2)

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
rospack_add_library(executive_trex_ros_g ${TREX_FILES})
target_link_libraries(executive_trex_ros_g Utils_g TREX_g)

# trexfast is about an order of magnitude faster than trexdebug
rospack_add_library(executive_trex_ros_o ${TREX_FILES})
target_link_libraries(executive_trex_ros_o Utils_o TREX_o)
rospack_add_compile_flags(executive_trex_ros_o "-O3 -DEUROPA_FAST")

# trexwatchdog is a safety process that will monitor pings from the executive
# and dispense shutdown commands to highlevel controllers (deactivation) if pings are
# not received in time
rospack_add_executable(bin/trexwatchdog src/watch_dog.cpp)
