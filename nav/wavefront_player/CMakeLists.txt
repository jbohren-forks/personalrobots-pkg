cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(wavefront_player)

find_package(Boost 0 REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

rospack_add_executable(wavefront_player wavefront_player.cc)
target_link_libraries(wavefront_player wavefront_standalone ${Boost_LIBRARIES})

# this little program just lets you send and block on goals from the shell
rospack_add_executable(cli cli.cpp)
