cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(videre_face_detection)
#genmsg()
#find_library(curl REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(videre_face_detection src/videre_face_detection.cpp src/people.cpp)

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

rospack_add_library(pyvidere_face_detection src/py.cpp src/videre_face_detection.cpp src/people.cpp)
set_target_properties(pyvidere_face_detection PROPERTIES OUTPUT_NAME videre_face_detection PREFIX "")

rospack_add_pyunit(test/directed.py)
