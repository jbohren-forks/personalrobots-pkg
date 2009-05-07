cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(door_handle_detector)
gensrv()
genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# door detector
rospack_add_executable (doors_detector_laser
                        src/geometric_helper.cpp
                        src/doors_detector.cpp
                       )

# handle detector
rospack_add_executable (handle_detector_laser
                        src/geometric_helper.cpp
                        src/handle_detector.cpp
                       )

# handle detector
rospack_add_executable(handle_detector_vision
                       src/handle_detector_vision.cpp 
                      )

rospack_add_executable(handle_detector_vision src/handle_detector_vision.cpp)
rospack_add_executable(test_detect_service test/test_detect_service.cpp)
rospack_add_executable(test_preempt_service test/test_preempt_service.cpp)


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
    endif( HAVE_OPENMP )
  endif( HAVE_OMP_H )
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  # check if compilers supports -fopenmp
  check_cxx_compiler_flag(-fopenmp HAVE_OPENMP)
  check_library_exists(gomp omp_get_num_threads "" HAS_GOMP_LIB)

  if( HAVE_OPENMP AND HAS_GOMP_LIB )
    add_definitions("-fopenmp")
    target_link_libraries(doors_detector_laser gomp)
    target_link_libraries(handle_detector_laser gomp)
    set(OPENMP_LFLAGS "-lgomp")
  endif( HAVE_OPENMP AND HAS_GOMP_LIB )
endif( WIN32 )
