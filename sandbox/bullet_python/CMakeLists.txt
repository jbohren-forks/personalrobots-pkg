cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#include(FindPkgConfig)
rospack(bullet_python)

rospack_add_boost_directories()

MAKE_DIRECTORY(${PROJECT_SOURCE_DIR}/src/)

### python header for swig
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})


rospack_add_pyunit(test/bullet_unittest.py)

# Find the combined swig flags for this project
_rospack_invoke(${PROJECT_NAME} ${PROJECT_NAME} SWIG_FLAGS "export" "--lang=swig" "--attrib=flags")
set(SWIG_FLAGS ${${PROJECT_NAME}_SWIG_FLAGS})


# Add a custom command for generating the swig output files
set(SWIG_INTERFACE_FILE ${PROJECT_SOURCE_DIR}/swig/bullet_swig.i)
set(SWIG_OUTPUT_PYTHON_FILE ${PROJECT_SOURCE_DIR}/src/bullet.py)

# Deal with CMake 2.4 vs. 2.6             
if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 2.4)   # CMake 2.6
  set(SWIG_OUTPUT_CPP_FILE ${PROJECT_SOURCE_DIR}/swig/bullet_swig_wrap.cpp)
  set(SWIG_COMMAND swig ${SWIG_FLAGS} -o ${SWIG_OUTPUT_CPP_FILE} -outdir ${PROJECT_SOURCE_DIR}/src -I${PROJECT_SOURCE_DIR}/build/include ${SWIG_INTERFACE_FILE})
else("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 2.4)    # CMake 2.4
  set(SWIG_OUTPUT_CPP_FILE swig/bullet_swig_wrap.cpp)
  set(SWIG_COMMAND swig ${SWIG_FLAGS} -o ../${SWIG_OUTPUT_CPP_FILE} -outdir ../src -I../build/include ${SWIG_INTERFACE_FILE})
endif("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 2.4)

set_source_files_properties(${SWIG_OUTPUT_CPP_FILE} PROPERTIES GENERATED true)
add_custom_command(OUTPUT   ${SWIG_OUTPUT_CPP_FILE}
                            ${SWIG_OUTPUT_PYTHON_FILE}
                   COMMAND  ${SWIG_COMMAND}
                   DEPENDS  ${SWIG_INTERFACE_FILE}
                            swig/pybtQuaternion.h
			    swig/pybtVector3.h
			    swig/pybtMatrix3x3.h
			    swig/pybtTransform.h)

# We add src/wx_rosout to the include list so that
# ogre_tools_swig_generated.cpp can find its headers.  This is part of a
# workaround for a deficiency in CMake 2.4's handling of generated sources.
#####include_directories(${PROJECT_SOURCE_DIR}/src/rxtools)
# shared library (with SWIG)    
rospack_add_library(python_${PROJECT_NAME} ${SWIG_OUTPUT_CPP_FILE})
target_link_libraries(python_${PROJECT_NAME} LinearMath ${PYTHON_LIBRARIES})

# swig python needs a shared library named _<modulename>.[so|dll|...]
# this renames the output file to conform to that by prepending an underscore and removing the "lib" prefix
#set_target_properties(python_${PROJECT_NAME}
#                      PROPERTIES OUTPUT_NAME _${PROJECT_NAME}
#                      PREFIX "")
### overriding above to make things easier to type
set(BULLET_PYTHON_LIBRARY_NAME bullet)
set_target_properties(python_${PROJECT_NAME}
                      PROPERTIES OUTPUT_NAME _${BULLET_PYTHON_LIBRARY_NAME}
                      PREFIX "")

# If on Darwin, create a symlink _foo.so -> _foo.dylib, because the
# MacPorts version of Python won't find _foo.dylib for 'import _foo'
include(CMakeDetermineSystem)
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  add_custom_command(OUTPUT ${LIBRARY_OUTPUT_PATH}/_${BULLET_PYTHON_LIBRARY_NAME}.so
                     COMMAND cmake -E create_symlink  ${LIBRARY_OUTPUT_PATH}/_${BULLET_PYTHON_LIBRARY_NAME}.dylib ${LIBRARY_OUTPUT_PATH}/_${BULLET_PYTHON_LIBRARY_NAME}.so
                     DEPENDS python_${PROJECT_NAME})
  add_custom_target(symlink_darwin_lib ALL
                    DEPENDS ${LIBRARY_OUTPUT_PATH}/_${BULLET_PYTHON_LIBRARY_NAME}.so)
endif(CMAKE_SYSTEM_NAME MATCHES "Darwin")
