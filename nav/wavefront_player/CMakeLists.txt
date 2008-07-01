cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(wavefront_player)
gensrv()
# the service provider
rospack_add_executable(wavefront_player wavefront_player.cc)
target_link_libraries(wavefront_player wavefront_standalone)

rospack_add_executable(send_goal send_goal.cc)

rospack_add_executable(query_wavefront query_wavefront.cpp)
