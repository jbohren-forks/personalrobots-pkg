cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(axis_cam)
genmsg()
find_library(curl REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_boost_directories()

rospack_add_library(axis src/libaxis_cam/axis_cam.cpp)

rospack_add_executable(axis_cam src/axis_cam/axis_cam.cpp)
target_link_libraries(axis_cam axis curl)
rospack_link_boost(axis_cam thread)

rospack_add_executable(axis_ptz src/axis_ptz/axis_ptz.cpp)
target_link_libraries(axis_ptz axis curl)
rospack_link_boost(axis_ptz thread)

rospack_add_executable(axis_demo src/axis_demo/axis_demo.cpp)
target_link_libraries(axis_demo axis curl)
rospack_link_boost(axis_demo thread)

add_subdirectory(src/libaxis_cam/demo)



