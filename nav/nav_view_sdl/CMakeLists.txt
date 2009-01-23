cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(nav_view_sdl)

rospack_add_boost_directories()

rospack_add_executable(nav_view nav_view.cpp)
target_link_libraries(nav_view SDL_image)

rospack_link_boost(nav_view thread)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

