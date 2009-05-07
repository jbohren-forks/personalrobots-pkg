cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Debug)
rospack(doors_core)
gensrv()
genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# executive interface
rospack_add_executable(action_runner_check_path
                       src/action_check_path.cpp 
		       src/action_runner_check_path.cpp
                      )

rospack_add_executable(action_runner_detect_door
                       src/action_detect_door.cpp 
		       src/action_runner_detect_door.cpp
                      )

rospack_add_executable(action_runner_detect_handle
		       src/action_detect_handle.cpp 	     
		       src/action_runner_detect_handle.cpp
                      )

rospack_add_executable(action_runner_grasp_handle
		       src/action_grasp_handle.cpp 
		       src/action_runner_grasp_handle.cpp
                      )

rospack_add_executable(action_runner_touch_door
		       src/action_touch_door.cpp 
		       src/action_runner_touch_door.cpp
                      )

rospack_add_executable(action_runner_push_door
		       src/action_push_door.cpp 
		       src/action_runner_push_door.cpp
                      )

rospack_add_executable(action_runner_open_door
		       src/action_open_door.cpp
		       src/action_runner_open_door.cpp
                      )

rospack_add_executable(action_runner_unlatch_handle
       		       src/action_unlatch_handle.cpp		
		       src/action_runner_unlatch_handle.cpp
                      )

rospack_add_executable(action_runner_release_handle
		       src/action_release_handle.cpp 
		       src/action_runner_release_handle.cpp
                      )

# test
rospack_add_executable(trigger_check_path
                       test/trigger_check_path.cpp
		       src/action_check_path.cpp
                      )

rospack_add_executable(trigger_doors_detector 
                       test/trigger_doors_detector.cpp
		       src/action_detect_door.cpp
		       src/action_detect_handle.cpp
                      )

rospack_add_executable(trigger_push_door
                       test/trigger_push_door.cpp
		       src/action_push_door.cpp
                      )

rospack_add_executable(trigger_doors_opener
                       test/trigger_doors_opener.cpp
		       src/action_open_door.cpp
                      )

rospack_add_executable(trigger_handle_release
                       test/trigger_handle_release.cpp
		       src/action_release_handle.cpp
                      )

rospack_add_executable(test_executive
		       test/test_executive.cpp
                      )

rospack_add_executable(move_base_door
                       src/move_base_door_action.cpp
		       src/door_reactive_planner.cpp
                      )


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
