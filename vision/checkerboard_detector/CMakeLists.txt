cmake_minimum_required(VERSION 2.4.6)
set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(checkerboard_detector)
rospack_add_boost_directories()

rospack_add_executable(checkerboard_detector checkerboard_detector.cpp)
rospack_link_boost(checkerboard_detector thread)

rospack_add_executable(checkerboard_calibration checkerboard_calibration.cpp)
rospack_link_boost(checkerboard_calibration thread)

include(CheckIncludeFile)
include(CheckCXXCompilerFlag)
include(CheckLibraryExists)
include(CheckFunctionExists)

# check for newer versions of opencv that support cvInitUndistortRectifyMap
# extract include dirs, libraries, and library dirs
_rospack_invoke(checkerboard_detector checkerboard_detector cflags export --lang=cpp --attrib=cflags)
_rospack_invoke(checkerboard_detector checkerboard_detector lflags export --lang=cpp --attrib=lflags)
foreach(arg ${checkerboard_detector_cflags})
  set(CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS} ${arg}")
endforeach()
foreach(arg ${checkerboard_detector_lflags})
  set(CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS} ${arg}")
  set(_used_lflags "${_used_lflags} ${arg}")
endforeach()
string(REGEX MATCHALL "(^| )-l([./+-_\\a-zA-Z]*)" _used_libs "${_used_lflags}")
string(REGEX REPLACE "(^| )-l" "" _used_libs "${_used_libs}")
set(CMAKE_REQUIRED_LIBRARIES ${_used_libs})
check_function_exists(cvInitUndistortRectifyMap HAVE_CV_UNDISTORT_RECTIFY_MAP)
if( HAVE_CV_UNDISTORT_RECTIFY_MAP )
  add_definitions("-DHAVE_CV_UNDISTORT_RECTIFY_MAP")
endif()

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
