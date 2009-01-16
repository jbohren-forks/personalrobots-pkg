cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(phase_space)
link_directories(${PROJECT_SOURCE_DIR}/lib)
rospack_add_executable(phase_space_node phase_space_node.cpp)
#rospack_add_executable(phase_space_benchmark phase_space_benchmark.cpp)

target_link_libraries(phase_space_node owlsock)
