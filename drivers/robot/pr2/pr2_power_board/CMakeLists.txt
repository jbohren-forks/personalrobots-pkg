cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(pr2_power_board)

gensrv()

set(EXECUTABLE_OUTPUT_PATH ${pr2_power_board_SOURCE_DIR}/bin)
add_subdirectory(src)
