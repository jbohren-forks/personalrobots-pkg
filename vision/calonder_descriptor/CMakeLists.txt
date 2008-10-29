cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(calonder_descriptor)
add_definitions(-Wall -O3 -DNDEBUG)
#add_definitions(-Wall -O0 -g)
rospack_add_library(calonder src/randomized_tree.cpp src/rtree_classifier.cpp src/patch_generator.cpp)

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

rospack_add_library(pycalonder src/py.cpp src/randomized_tree.cpp src/rtree_classifier.cpp src/patch_generator.cpp)
set_target_properties(pycalonder PROPERTIES OUTPUT_NAME calonder PREFIX "")
rospack_add_compile_flags(pycalonder -Wno-missing-field-initializers -save-temps -march=pentium3 -msse3)
#rospack_add_compile_flags(pycalonder -Wno-missing-field-initializers)

rospack_add_pyunit(test/directed.py)
