cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(doors_core)
gensrv()
genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# executive interface
rospack_add_executable(action_runner 
                       src/action_detect_door.cpp 
		       src/action_detect_handle.cpp 	     
		       src/action_grasp_handle.cpp 
		       src/action_open_door.cpp 
		       src/action_release_handle.cpp 
		       src/action_runner.cpp
                      )


# test
rospack_add_executable(trigger_doors_detector 
                       test/trigger_doors_detector.cpp
		       src/action_detect_door.cpp
		       src/action_detect_handle.cpp
                      )

rospack_add_executable(trigger_doors_opener
                       test/trigger_doors_opener.cpp
		       src/action_open_door.cpp
                      )

rospack_add_executable(door_reactive_planner_action
                       src/move_base_door_action.cpp
		       src/door_reactive_planner.cpp
                      )

rospack_add_library(executive_functions src/executive_functions.cpp)


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
    set(OPENMP_LFLAGS "-lgomp")
  endif( HAVE_OPENMP AND HAS_GOMP_LIB )
endif( WIN32 )
