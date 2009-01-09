cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Release)

rospack(dc1394_cam)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include(FindPkgConfig)
pkg_check_modules(SDL REQUIRED sdl)
include_directories(${SDL_INCLUDE_DIRS})
link_directories(${SDL_LIBRARY_DIRS})

rospack_add_library(dc1394cam src/dc1394_cam/dc1394_cam.cpp)

rospack_add_executable(cam_dump src/util/cam_dump/cam_dump.cpp)
target_link_libraries(cam_dump dc1394cam)

rospack_add_executable(view_cam src/util/view_cam/view_cam.cpp)
target_link_libraries(view_cam dc1394cam)
target_link_libraries(view_cam ${SDL_LIBRARIES})
