cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#include(FindPkgConfig)
rospack(bullet)

rospack_add_boost_directories()

MAKE_DIRECTORY(${PROJECT_SOURCE_DIR}/src/)

# Build Bullet
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.bullet
                RESULT_VARIABLE _make_failed)
if(_make_failed)
  message(FATAL_ERROR "Build of Bullet failed")
endif(_make_failed)

### local includes
include_directories(build/include)
link_directories(lib)
add_definitions(-DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX)

### bullet unit tests for items i've patched
rospack_add_gtest(bullet_unittest test/bullet_unittest.cpp)
target_link_libraries(bullet_unittest LinearMath)

# If on Darwin, create a symlink _foo.so -> _foo.dylib, because the
# MacPorts version of Python won't find _foo.dylib for 'import _foo'
include(CMakeDetermineSystem)
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  add_custom_command(OUTPUT ${LIBRARY_OUTPUT_PATH}/_${PROJECT_NAME}.so
                     COMMAND cmake -E create_symlink  ${LIBRARY_OUTPUT_PATH}/_${PROJECT_NAME}.dylib ${LIBRARY_OUTPUT_PATH}/_${PROJECT_NAME}.so
                     DEPENDS python_${PROJECT_NAME})
  add_custom_target(symlink_darwin_lib ALL
                    DEPENDS ${LIBRARY_OUTPUT_PATH}/_${PROJECT_NAME}.so)
endif(CMAKE_SYSTEM_NAME MATCHES "Darwin")
