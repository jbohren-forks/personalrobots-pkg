cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(rosgazebo)
genmsg()
include_directories(msg/cpp)
gensrv()
include_directories(srv/cpp)
add_definitions(-Wall)
rospack_add_library(RosGazeboNode src/RosGazeboNode.cpp)

rospack_add_executable(rosgazebo src/rosgazebo.cc)
target_link_libraries(rosgazebo RosGazeboNode)

rospack_add_executable(rosgazebowriter src/rosgazebowriter.cc)
target_link_libraries(rosgazebowriter RosGazeboNode)


rospack_add_executable(poshack test/poshack.cpp)
