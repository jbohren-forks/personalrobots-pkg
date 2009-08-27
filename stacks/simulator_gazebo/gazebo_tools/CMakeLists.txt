cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rospack(gazebo_tools)
genmsg()
include_directories(msg/cpp)
gensrv()
include_directories(srv/cpp)

add_definitions(-fPIC)

rospack_add_boost_directories()

rospack_add_library(urdf2gazebo src/urdf2gazebo.cpp)

rospack_add_executable(bin/remove_model src/remove_model.cpp)
target_link_libraries(bin/remove_model)

rospack_add_executable(bin/xml2factory src/xml2factory.cpp)
target_link_libraries(bin/xml2factory)

rospack_add_executable(bin/urdf2factory src/urdf2factory.cpp)
target_link_libraries(bin/urdf2factory urdf2gazebo)

rospack_add_executable(bin/urdf2file src/urdf2file.cpp)
target_link_libraries(bin/urdf2file urdf2gazebo)

rospack_add_executable(bin/spawn_urdf src/spawn_urdf.cpp)
target_link_libraries(bin/spawn_urdf urdf2gazebo)
rospack_link_boost(bin/spawn_urdf program_options)
