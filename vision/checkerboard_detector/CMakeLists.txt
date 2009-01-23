cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(checkerboard_detector)
rospack_add_executable(checkerboard_detector checkerboard_detector.cpp)

rospack_add_boost_directories()
rospack_link_boost(checkerboard_detector thread)

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
    endif(HAVE_OPENMP)
  endif(HAVE_OMP_H)
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)

  # check if compilers supports -fopenmp
  check_cxx_compiler_flag(-fopenmp HAVE_OPENMP)
  check_library_exists(gomp omp_get_num_threads "" HAS_GOMP_LIB)

  if( HAVE_OPENMP AND HAS_GOMP_LIB )
    add_definitions("-fopenmp")
    target_link_libraries(checkerboard_detector gomp)
    set(OPENMP_LFLAGS "-lgomp")
  endif(HAVE_OPENMP AND HAS_GOMP_LIB)
endif(WIN32)

genmsg()

# if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
#   add_definitions(" -msse2 -O2 ")
# endif()
