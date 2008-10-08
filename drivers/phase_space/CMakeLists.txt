cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(phase_space)
genmsg()
link_directories(${PROJECT_SOURCE_DIR}/lib)
rospack_add_executable(phase_space_node phase_space_node.cpp)

target_link_libraries(phase_space_node owlsock)
