cmake_minimum_required(VERSION 2.6)
include(rosbuild)
#set(ROS_BUILD_TYPE Release)
rospack(place_recognition)
rospack_add_library(place_recognition src/kmeans.cpp src/vocabulary_tree.cpp src/place_recognizer.cpp)
rospack_add_compile_flags(place_recognition -g -Wno-missing-field-initializers -save-temps -march=pentium3 -msse3)

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

rospack_add_library(pyplace_recognition src/kmeans.cpp src/vocabulary_tree.cpp src/place_recognizer.cpp src/py.cpp)
set_target_properties(pyplace_recognition PROPERTIES OUTPUT_NAME place_recognition PREFIX "")
rospack_add_compile_flags(pyplace_recognition -g -Wno-missing-field-initializers -save-temps -msse3)

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
