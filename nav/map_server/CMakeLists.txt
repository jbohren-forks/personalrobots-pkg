cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
set(CMAKE_MODULE_PATH $ENV{ROS_ROOT}/core/rosbuild)
set(EXECUTABLE_OUTPUT_PATH ..)
include(rosbuild)
project(map_server)
rospack()

add_executable(map_server map_server.cc)

# Search for gdk-pixbuf using pkg-config (we don't redistribute gdk-pixbuf,
# because that's a deep deep hole of dependencies).
# pkg_check_modules is defined in the FindPkgConfig module
include(FindPkgConfig)
pkg_check_modules(GDKPIXBUF REQUIRED gdk-pixbuf-2.0)
include_directories(${GDKPIXBUF_INCLUDE_DIRS})
link_directories(${GDKPIXBUF_LIBRARY_DIRS})
target_link_libraries(map_server ${GDKPIXBUF_LIBRARIES} ${map_server_LIBRARIES})

