cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(dc1394_cam)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_library(dc1394cam src/dc1394_cam/dc1394_cam.cpp)

rospack_add_executable(cam_dump src/util/cam_dump/cam_dump.cpp)
target_link_libraries(cam_dump dc1394cam)

rospack_add_executable(view_cam src/util/view_cam/view_cam.cpp)
target_link_libraries(view_cam dc1394cam)

rospack_add_executable(dc1394_node src/dc1394_node/dc1394_node.cpp)
target_link_libraries(dc1394_node dc1394cam)

