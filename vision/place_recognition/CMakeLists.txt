cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(place_recognition)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_boost_directories()

set(ROS_BUILD_TYPE Release)
rospack_add_library(place_recognition src/kmeans.cpp src/vocabulary_tree.cpp)
rospack_add_compile_flags(place_recognition -g -Wno-missing-field-initializers -save-temps -msse3)

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

# This usually causes compile- or link-time failures of the C++ lib!
#rospack_add_library(pyplace_recognition src/kmeans.cpp src/vocabulary_tree.cpp src/py.cpp)

# These lines instead work fine
rospack_add_library(pyplace_recognition src/py.cpp)
target_link_libraries(pyplace_recognition place_recognition)

set_target_properties(pyplace_recognition PROPERTIES OUTPUT_NAME place_recognition PREFIX "")
rospack_add_compile_flags(pyplace_recognition -g -Wno-missing-field-initializers -save-temps -msse3)

# Trivial program to test whether libplace_recognition has all symbols defined correctly
rospack_add_executable(link_lib test/link_lib.cpp)
target_link_libraries(link_lib place_recognition)

# Graphical test
#rospack_add_pyunit(test/directed.py)

# thrash.py must be declared as a graphical test, because it loads the 
# pylab module which requires a display.
#
# thrash.py isn't actually a unit test, so I'm commenting it out for
# now.
#
# Also, thrash.py requires access to a fairly large data set that isn't
# checked in.
#rospack_add_pyunit_graphical(test/thrash.py)
