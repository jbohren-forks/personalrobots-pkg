cmake_minimum_required (VERSION 2.6)

### If you're trying to debug include directives or so, this here
### might help:
#set (CMAKE_VERBOSE_MAKEFILE ON)

### Debug mode implies slower runtime, "of course"
#set (ROS_BUILD_TYPE Debug)
set (ROS_BUILD_TYPE Release)

include (rosbuild)
add_definitions (-Wall)
rospack (navfn)

#include (CheckIncludeFile)
#check_include_file (pgm.h NAVFN_HAVE_NETPGM)
#if (${NAVFN_HAVE_NETPGM})
  # should probably also check for libnetpbm existence...
 # message (STATUS "OK found pgm.h")
#else (${NAVFN_HAVE_NETPGM})
 # message (FATAL_ERROR "pgm.h not found (this could probably be made optional)")
#endif (${NAVFN_HAVE_NETPGM})

rospack_add_library (navfn src/navfn.cpp)

rospack_add_executable (navtest src/navtest.cpp src/navwin.cpp)
target_link_libraries (navtest navfn -lfltk -lnetpbm)
