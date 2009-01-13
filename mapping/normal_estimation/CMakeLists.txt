cmake_minimum_required(VERSION 2.6)

include(rosbuild)
set(ROS_BUILD_TYPE Release)
rospack(normal_estimation)
add_definitions(-Wall -O3 -DNDEBUG -march=pentium4 -mfpmath=sse -mmmx -msse -msse2 -msse3)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#Deprecated for now, use the omp (OpenMP version)
#rospack_add_executable(normal_estimation_node src/normal_estimation.cpp)
#target_link_libraries(normal_estimation_node lapack)

rospack_add_executable(normal_estimation_omp_node src/normal_estimation_omp.cpp)
target_link_libraries(normal_estimation_omp_node lapack)

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
    target_link_libraries(normal_estimation_omp_node gomp)
    set(OPENMP_LFLAGS "-lgomp")
  endif()
endif()
