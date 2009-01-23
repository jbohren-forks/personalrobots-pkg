cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(topological_map)
gensrv()

#uncomment for profiling
set(ROS_COMPILE_FLAGS "-g -pg -O3" ${ROS_COMPILE_FLAGS})
set(ROS_LINK_FLAGS "-g -pg -O3" ${ROS_LINK_FLAGS})

rospack_add_library(topological_map src/bottleneck_graph.cpp src/roadmap.cpp src/gridcell.cpp)
target_link_libraries(topological_map navfn)

#rospack_add_executable(ros_bottleneck_graph src/ros_bottleneck_graph.cpp)
#target_link_libraries(ros_bottleneck_graph topological_map)

rospack_add_executable(bg_driver src/bg_driver.cpp)
target_link_libraries(bg_driver topological_map)

