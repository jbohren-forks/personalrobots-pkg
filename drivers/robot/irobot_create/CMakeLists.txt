cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(irobot_create)

rospack_add_library(irobot_create irobot_create.cpp)

rospack_add_executable(print_encoders print_encoders.cpp)
target_link_libraries(print_encoders irobot_create)

rospack_add_executable(drive_in_circle drive_in_circle.cpp)
target_link_libraries(drive_in_circle irobot_create)
