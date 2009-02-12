cmake_minimum_required (VERSION 2.4.6)

### If you're trying to debug include directives or so, this here
### might help:
#set (CMAKE_VERBOSE_MAKEFILE ON)

#set (ROS_BUILD_TYPE Release)
set (ROS_BUILD_TYPE Debug)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions (-Wall)
rospack (mpglue)

list (APPEND MPGLUE_SRCS
      src/costmap.cpp
      src/costmapper.cpp
      src/planner.cpp
      src/sbpl_planner.cpp
      src/navfn_planner.cpp
      src/footprint.cpp
      src/sbpl_environment.cpp
      src/plan.cpp)

list (APPEND MPGLUE_LIBS costmap_2d costmap_2d sunflower navfn)

set (ESTAR_DIR $ENV{ESTAR_DIR})
if (ESTAR_DIR)
  message (STATUS "ESTAR_DIR is ${ESTAR_DIR}")
  set (CMAKE_REQUIRED_INCLUDES ${ESTAR_DIR}/include)
  include_directories (${ESTAR_DIR}/include)
  link_directories (${ESTAR_DIR}/lib)
else (ESTAR_DIR)
  message (STATUS "ESTAR_DIR is not set")
endif (ESTAR_DIR)

include (CheckIncludeFileCXX)
check_include_file_cxx (estar/Facade.hpp MPGLUE_HAVE_ESTAR)
if (MPGLUE_HAVE_ESTAR)
  message (STATUS "found E*, adding src/estar_planner.cpp to build")
  add_definitions (-DMPGLUE_HAVE_ESTAR)
  list (APPEND MPGLUE_SRCS src/estar_planner.cpp)
  list (APPEND MPGLUE_LIBS estar)
else (MPGLUE_HAVE_ESTAR)
  message (STATUS "E* not found (optional)")
endif (MPGLUE_HAVE_ESTAR)

rospack_add_library (mpglue ${MPGLUE_SRCS})
target_link_libraries (mpglue ${MPGLUE_LIBS})
