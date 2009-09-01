cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rospack(orocos_ros_integration)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#genmsg()
#uncomment if you have defined services
#gensrv()

rospack_add_library(${PROJECT_NAME} src/Orocos2Ros.cpp src/RosToolkit.cpp src/RosDeploymentComponent.cpp)

#common commands for building c++ executables and libraries
rospack_add_library(example src/Components.cpp)
ADD_DEFINITIONS(-DOCL_DLL_EXPORT)
target_link_libraries(example ${PROJECT_NAME})

rospack_add_executable(rosdeployer src/ros-deployer.cpp src/deployer-funcs.cpp)
target_link_libraries(rosdeployer ${PROJECT_NAME} orocos-deployment-gnulinux orocos-taskbrowser-gnulinux orocos-rtt-gnulinux)
rospack_add_boost_directories()
rospack_link_boost(rosdeployer program_options)

#rospack_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
