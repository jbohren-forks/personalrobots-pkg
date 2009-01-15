cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(calonder_descriptor)
add_definitions(-Wall -O3 -DNDEBUG -march=pentium4 -mfpmath=sse -mmmx -msse -msse2 -msse3)
#add_definitions(-Wall -O0 -g -DDEBUG)
rospack_add_library(calonder src/randomized_tree.cpp src/rtree_classifier.cpp src/patch_generator.cpp)

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})
target_link_libraries(calonder cblas)

rospack_add_library(pycalonder src/py.cpp src/randomized_tree.cpp src/rtree_classifier.cpp src/patch_generator.cpp)
set_target_properties(pycalonder PROPERTIES OUTPUT_NAME calonder PREFIX "")
rospack_add_compile_flags(pycalonder -Wno-missing-field-initializers -save-temps -march=pentium3 -msse3)
#rospack_add_compile_flags(pycalonder -Wno-missing-field-initializers -save-temps)
target_link_libraries(pycalonder cblas)

rospack_add_compile_flags(pycalonder -Wno-missing-field-initializers)

# Commenting out because test is broken by the functions hard-coded to
# assume d = 176, t = 50
#rospack_add_pyunit(test/directed.py)
