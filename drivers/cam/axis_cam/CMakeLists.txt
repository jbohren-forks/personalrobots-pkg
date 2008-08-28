cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(axis_cam)
genmsg()
find_library(curl REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_library(axis src/libaxis_cam/axis_cam.cpp)
rospack_add_executable(axis_cam src/axis_cam/axis_cam.cpp)
target_link_libraries(axis_cam axis curl)
rospack_add_executable(axis_ptz src/axis_ptz/axis_ptz.cpp)
target_link_libraries(axis_ptz axis curl)

