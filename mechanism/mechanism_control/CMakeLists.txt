cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(rosgazebo)
add_definitions(-Wall)
rospack_add_library(RosGazeboNode src/RosGazeboNode.cpp)
rospack_add_executable(rosgazebo src/rosgazebo.cc)
target_link_libraries(rosgazebo RosGazeboNode)
rospack_add_executable(rosgazebowriter src/rosgazebowriter.cc)
target_link_libraries(rosgazebowriter RosGazeboNode)

#add_executable(map_to_model src/map_to_model.cpp)
## Search for gdk-pixbuf using pkg-config (we don't redistribute gdk-pixbuf,
## because that's a deep deep hole of dependencies).
## pkg_check_modules is defined in the FindPkgConfig module
#include(FindPkgConfig)
#pkg_check_modules(GDKPIXBUF REQUIRED gdk-pixbuf-2.0)
#include_directories(${GDKPIXBUF_INCLUDE_DIRS})
#link_directories(${GDKPIXBUF_LIBRARY_DIRS})
#target_link_libraries(map_to_model ${GDKPIXBUF_LIBRARIES})
