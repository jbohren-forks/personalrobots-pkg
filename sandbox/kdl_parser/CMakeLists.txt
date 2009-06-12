cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rospack(kdl_parser)

rospack_add_boost_directories()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#genmsg()
#uncomment if you have defined services
#gensrv()

rospack_add_library(${PROJECT_NAME} src/tree_parser.cpp)

rospack_add_executable(example test/example.cpp )
target_link_libraries(example ${PROJECT_NAME})

rospack_add_executable(test_parser test/test_kdl_parser.cpp )
target_link_libraries(test_parser ${PROJECT_NAME})

rospack_add_rostest(${PROJECT_SOURCE_DIR}/test/test_kdl_parser.launch)
