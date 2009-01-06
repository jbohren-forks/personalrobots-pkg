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
rospack_add_compile_flags(pyplace_recognition -g -Wno-missing-field-initializers -save-temps )

#rospack_add_pyunit(test/directed.py)
rospack_add_pyunit(test/thrash.py)
