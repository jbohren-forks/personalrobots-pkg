cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(filter_generator)
genmsg()
gensrv()

rospack_add_library( filter_generator
  test/gen_filter_client.cpp)
  
rospack_add_executable(test_gen_filter test/gen_filter_client.cpp)
target_link_libraries(test_gen_filter filter_generator)
