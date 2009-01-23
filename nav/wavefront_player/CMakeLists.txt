cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(wavefront_player)

rospack_add_boost_directories()

rospack_add_executable(wavefront_player wavefront_player.cc)
target_link_libraries(wavefront_player wavefront_standalone)
rospack_link_boost(wavefront_player thread)

# this little program just lets you send and block on goals from the shell
rospack_add_executable(cli cli.cpp)
rospack_link_boost(cli thread)
