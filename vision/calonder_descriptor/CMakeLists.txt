cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(calonder_descriptor)

#rospack_add_boost_directories()

add_definitions(-Wall -O3 -march=pentium4 -mfpmath=sse -mmmx -msse -msse2 -msse3)
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

# Download needed data file
# This code is wrapped up in a new macro, which hasn't yet been merged to
# the stable branch of ROS.
#rospack_download_test_data(http://pr.willowgarage.com/data/calonder_descriptor/current.rtc current.rtc)

set(_url http://pr.willowgarage.com/data/calonder_descriptor/current.rtc)
set(_filename current.rtc)
find_package(Wget REQUIRED)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/${_filename}
                   COMMAND ${WGET_EXECUTABLE} -r ${_url} -O ${PROJECT_SOURCE_DIR}/${_filename}
                   VERBATIM)
# Create a legal target name, in case the target name has slashes in it
string(REPLACE "/" "_" _testname download_data_${_filename})
add_custom_target(${_testname}
                  DEPENDS ${PROJECT_SOURCE_DIR}/current.rtc)
add_dependencies(tests ${_testname})
rospack_add_pyunit(test/directed.py)
