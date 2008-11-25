cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(borg)
#add_definitions(-Wall -O3 -g)
rospack_add_library(borg borg.cpp cam_dc1394.cpp cam.cpp)

include(FindPkgConfig)
pkg_check_modules(SDL REQUIRED sdl)
include_directories(${SDL_INCLUDE_DIRS})
link_directories(${SDL_LIBRARY_DIRS})

rospack_add_executable(scan scan.cpp)
target_link_libraries(scan borg)

rospack_add_executable(view view.cpp)
target_link_libraries(view borg)
target_link_libraries(view ${SDL_LIBRARIES})
