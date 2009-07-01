cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(tf)

rospack_add_boost_directories()

genmsg()
gensrv()

rospack_add_library(tf src/tf.cpp src/transform_listener.cpp src/cache.cpp src/message_notifier.cpp src/transform_broadcaster.cpp)
rospack_link_boost(tf thread signals)

rospack_add_executable(bin/tf_echo src/tf_echo.cpp)
target_link_libraries(bin/tf_echo tf)

rospack_add_executable(bin/change_notifier src/change_notifier.cpp)
target_link_libraries(bin/change_notifier tf)

rospack_add_executable(bin/tf_monitor src/tf_monitor.cpp)
target_link_libraries(bin/tf_monitor tf)

rospack_add_gtest(test/tf_unittest test/tf_unittest.cpp)
target_link_libraries(test/tf_unittest tf)
rospack_add_gtest_future(test/tf_unittest_future test/tf_unittest_future.cpp)
target_link_libraries(test/tf_unittest_future tf)
rospack_add_gtest(test/cache_unittest test/cache_unittest.cpp)
target_link_libraries(test/cache_unittest tf)


### Benchmarking
rospack_add_gtest_future(test/tf_benchmark test/tf_benchmark.cpp)
target_link_libraries(test/tf_benchmark tf)

rospack_add_executable(test/btTest test/quaternion.cpp)
rospack_add_executable(test/testListener test/testListener.cpp)
target_link_libraries(test/testListener tf)
                                 
rospack_add_executable(test/testBroadcaster test/testBroadcaster.cpp)
target_link_libraries(test/testBroadcaster tf)
rospack_add_executable(bin/transform_sender src/transform_sender.cpp)
target_link_libraries(bin/transform_sender tf)

rospack_add_executable(test/test_notifier EXCLUDE_FROM_ALL test/test_notifier.cpp)
rospack_declare_test(test/test_notifier)
rospack_add_gtest_build_flags(test/test_notifier)
target_link_libraries(test/test_notifier tf ${Boost_LIBRARIES})
rospack_add_rostest(test/test_notifier.xml)

### python header for swig
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})

# Find the combined swig flags for this project
_rospack_invoke(${PROJECT_NAME} ${PROJECT_NAME} SWIG_FLAGS "export" "--lang=swig" "--attrib=flags")
set(SWIG_FLAGS ${${PROJECT_NAME}_SWIG_FLAGS})


# Add a custom command for generating the swig output files
set(SWIG_INTERFACE_FILE ${PROJECT_SOURCE_DIR}/swig/tf_swig.i)
set(SWIG_OUTPUT_PYTHON_FILE ${PROJECT_SOURCE_DIR}/src/tf/tf_swig.py)

# Deal with CMake 2.4 vs. 2.6             
if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 2.4)   # CMake 2.6
  set(SWIG_OUTPUT_CPP_FILE ${PROJECT_SOURCE_DIR}/swig/tf_swig_wrap.cpp)
  set(SWIG_COMMAND swig ${SWIG_FLAGS} -o ${SWIG_OUTPUT_CPP_FILE} -outdir ${PROJECT_SOURCE_DIR}/src/tf -I${PROJECT_SOURCE_DIR}/build/include -module ${PROJECT_NAME}_swig ${SWIG_INTERFACE_FILE})
else("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 2.4)    # CMake 2.4
  set(SWIG_OUTPUT_CPP_FILE swig/tf_swig_wrap.cpp)
  set(SWIG_COMMAND swig ${SWIG_FLAGS} -o ../${SWIG_OUTPUT_CPP_FILE} -outdir ../src/tf -I../build/include -module ${PROJECT_NAME}_swig ${SWIG_INTERFACE_FILE})
endif("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 2.4)

set_source_files_properties(${SWIG_OUTPUT_CPP_FILE} PROPERTIES GENERATED true)
add_custom_command(OUTPUT   ${SWIG_OUTPUT_CPP_FILE}
                            ${SWIG_OUTPUT_PYTHON_FILE}
                   COMMAND  ${SWIG_COMMAND}
                   DEPENDS  ${SWIG_INTERFACE_FILE}
			    swig/tf/tf_swig.h)

# We add src/wx_rosout to the include list so that
# ogre_tools_swig_generated.cpp can find its headers.  This is part of a
# workaround for a deficiency in CMake 2.4's handling of generated sources.
#####include_directories(${PROJECT_SOURCE_DIR}/src/rxtools)
# shared library (with SWIG)    
rospack_add_library(python_${PROJECT_NAME} ${SWIG_OUTPUT_CPP_FILE})
target_link_libraries(python_${PROJECT_NAME} tf ${PYTHON_LIBRARIES})

# swig python needs a shared library named _<modulename>.[so|dll|...]
# this renames the output file to conform to that by prepending an underscore and removing the "lib" prefix
set_target_properties(python_${PROJECT_NAME}
                      PROPERTIES OUTPUT_NAME _${PROJECT_NAME}_swig
                      PREFIX "")


# Tests for swig
rospack_add_pyunit_future(test/test_datatype_conversion.py)

