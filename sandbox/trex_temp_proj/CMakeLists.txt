cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE Release)
rospack(trex_temp_proj)

genmsg()
gensrv()

rospack_add_boost_directories()

###############################################################################
# NDDL Include Macro
###############################################################################

# This generates the NDDL inclue configuration file
find_ros_package(trex_ros)
include(${trex_ros_PACKAGE_PATH}/trex_build.cmake)
create_nddl_config()

###############################################################################
# TREX Hooks into Local ROS Code
###############################################################################

set(TREX_TEMP_PROJ_FILES src/adapter_utilities.cpp
			 src/components.cpp
			 src/adapters.cpp
			 src/master_reactor.cpp)

###############################################################################
# Source library
###############################################################################

# build extensions into exported library
create_trex_lib(trex_temp_proj ${TREX_TEMP_PROJ_FILES})

###############################################################################
# Executables & Tests
###############################################################################

create_trex_executables(bin/trexfast bin/trexdebug src/main.cpp)

# stub_ros__container implements stub state publishers and actions to allow easy testing of the ROS adapters for the model
rospack_add_executable(bin/stub_ros_container src/stub_ros_container.cpp)

# Try the API for running the service calls to the executive
rospack_add_executable(test/ros_reactor_test test/ros_reactor_test.cpp)

