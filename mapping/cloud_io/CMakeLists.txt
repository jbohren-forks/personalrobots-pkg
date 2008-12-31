cmake_minimum_required(VERSION 2.6)

include(rosbuild)
set(ROS_BUILD_TYPE Release)
rospack(cloud_io)
add_definitions(-Wall)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_library(cloud_io src/read.cpp src/write.cpp src/misc.cpp)

rospack_add_gtest(test_io test/test_io.cpp)
target_link_libraries(test_io cloud_io)

rospack_add_executable(pcd_to_cloudmsg src/tools/pcd_to_cloudmsg.cpp)
target_link_libraries(pcd_to_cloudmsg cloud_io)

