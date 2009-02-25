cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(topological_map)
gensrv()

rospack_add_boost_directories()

set(ROS_COMPILE_FLAGS "-O3 " ${ROS_COMPILE_FLAGS})

#uncomment for profiling/gdb
#set(ROS_COMPILE_FLAGS "-g -pg " ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g -pg " ${ROS_LINK_FLAGS})


## Old version
rospack_add_library(topological_map src/bottleneck_graph.cpp src/roadmap.cpp src/gridcell.cpp)
target_link_libraries(topological_map navfn)

rospack_add_executable(ros_topological_planner src/ros_topological_planner.cpp)
target_link_libraries(ros_topological_planner topological_map)
# rospack_link_boost(ros_bottleneck_graph graph)

rospack_add_executable(bg_driver src/bg_driver.cpp)
target_link_libraries(bg_driver topological_map)





## New version
## Rename once done
rospack_add_library(topological_graph src/region.cpp src/topological_map.cpp src/creation.cpp src/region_graph.cpp)

rospack_add_executable(ros_topological_map src/ros_topological_map.cpp)
target_link_libraries(ros_topological_map topological_graph)

rospack_add_gtest(test/test_topological_map test/test_topological_map.cpp)
target_link_libraries(test/test_topological_map topological_graph)

rospack_add_executable(tm_driver src/tm_driver)
target_link_libraries(tm_driver topological_graph)
