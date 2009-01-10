cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(laser_scan)
rospack_add_library(laser_scan src/laser_scan.cc )
rospack_add_library(laser_median_filter src/median_filter.cpp )

rospack_add_executable(median_node src/median_filter_node.cpp)
target_link_libraries(median_node laser_median_filter)

rospack_add_executable(robotlinks_filter_node src/robotlinks_filter_node.cpp)
target_link_libraries (robotlinks_filter_node openrave-core qhull)

# check for OpenMP
include(CheckIncludeFile)
include(CheckCXXCompilerFlag)
include(CheckLibraryExists)

if( WIN32 )
  CHECK_INCLUDE_FILE(omp.h HAVE_OMP_H)
  if( HAVE_OMP_H )
    message(STATUS "Using OpenMP")
    check_cxx_compiler_flag(/openmp HAVE_OPENMP)

    if( HAVE_OPENMP )
      add_definitions("/openmp")
    endif()
  endif()
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)

  # check if compilers supports -fopenmp
  check_cxx_compiler_flag(-fopenmp HAVE_OPENMP)
  check_library_exists(gomp omp_get_num_threads "" HAS_GOMP_LIB)

  if( HAVE_OPENMP AND HAS_GOMP_LIB )
    add_definitions("-fopenmp")
    target_link_libraries(robotlinks_filter_node gomp)
    set(OPENMP_LFLAGS "-lgomp")
  endif()
endif()

rospack_add_executable(scan_shadows_filter_node src/scan_shadows_filter.cpp)
target_link_libraries (scan_shadows_filter_node laser_scan)

rospack_add_executable(scan_intensity_filter src/scan_intensity_filter.cpp)
target_link_libraries (scan_intensity_filter laser_scan)
