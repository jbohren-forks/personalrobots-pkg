cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
include(rosbuild)
rospack(map_server)

rospack_add_executable(map_server map_server.cc)

# Search for gdk-pixbuf using pkg-config (we don't redistribute gdk-pixbuf,
# because that's a deep deep hole of dependencies).
# pkg_check_modules is defined in the FindPkgConfig module
include(FindPkgConfig)
pkg_check_modules(GDKPIXBUF REQUIRED gdk-pixbuf-2.0)
include_directories(${GDKPIXBUF_INCLUDE_DIRS})
link_directories(${GDKPIXBUF_LIBRARY_DIRS})
target_link_libraries(map_server ${GDKPIXBUF_LIBRARIES})
