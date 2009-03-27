cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(door_handle_detector)
gensrv()
genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable (doors_detector_node
                        src/geometric_helper.cpp
                        src/doors_detector.cpp
                       )

rospack_add_executable (handle_detector_node
                        src/geometric_helper.cpp
                        src/handle_detector.cpp
                       )

rospack_add_library(executive_functions src/executive_functions.cpp)

rospack_add_executable(action_runner src/action_detect_door.cpp 
				     src/action_grasp_door.cpp 
				     src/action_open_door.cpp 
				     test/action_runner.cpp)



rospack_add_executable(trigger_doors test/trigger_doors.cpp)
rospack_add_executable(trigger_doors_scan test/trigger_doors_scan.cpp)
rospack_add_executable(trigger_handle test/trigger_handle.cpp)

#rospack_add_executable(trigger test/trigger.cpp)
#rospack_add_executable(open_door_executive_test test/open_door_executive_test.cpp)
#rospack_add_executable(door_checkerboard_detector test/door_checkerboard_detector.cpp)
#rospack_add_executable(test_door_detection test/test_door_detection.cpp)
#rospack_add_executable(test_door_detection_joystick test/test_door_detection_joystick.cpp)

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
    target_link_libraries(doors_detector_node gomp)
    target_link_libraries(handle_detector_node gomp)
    set(OPENMP_LFLAGS "-lgomp")
  endif( HAVE_OPENMP AND HAS_GOMP_LIB )
endif( WIN32 )
