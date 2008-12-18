cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(filter_generator)
genmsg()
gensrv()

rospack_add_library( filter_generator
  src/gen_filter_client.cpp)
  
rospack_add_executable(filter_generator_client src/gen_filter_client.cpp)
target_link_libraries(filter_generator_client filter_generator)
