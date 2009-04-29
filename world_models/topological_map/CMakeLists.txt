cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(topological_map)
gensrv()

rospack_add_boost_directories()

set(ROS_COMPILE_FLAGS "-O3 " ${ROS_COMPILE_FLAGS})

#uncomment for profiling/gdb
set(ROS_COMPILE_FLAGS "-g -pg " ${ROS_COMPILE_FLAGS})
set(ROS_LINK_FLAGS "-g -pg " ${ROS_LINK_FLAGS})


rospack_add_library(topological_graph src/region.cpp src/topological_map.cpp src/creation.cpp src/region_graph.cpp src/connector_roadmap.cpp 
				      src/grid_graph.cpp src/grid_utils.cpp src/door_info.cpp src/outlet_info.cpp)

#rospack_add_executable(bin/ros_topological_map src/ros_topological_map.cpp)
#target_link_libraries(bin/ros_topological_map topological_graph)

rospack_add_executable(bin/generate_ground_truth_map src/generate_from_ground_truth.cpp)
target_link_libraries(bin/generate_ground_truth_map topological_graph)
rospack_link_boost(bin/generate_ground_truth_map program_options)						     

rospack_add_gtest(test/test_topological_map test/test_topological_map.cpp)
target_link_libraries(test/test_topological_map topological_graph)

rospack_add_executable(bin/tm_driver src/tm_driver)
target_link_libraries(bin/tm_driver topological_graph)

rospack_add_executable(bin/door_sensor_fusion src/door_sensor_fusion.cpp)
