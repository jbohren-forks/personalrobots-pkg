cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rospack(pcd_novelty)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#genmsg()
#uncomment if you have defined services
#gensrv()

#common commands for building c++ executables and libraries
rospack_add_library(${PROJECT_NAME} src/novelty_estimator.cpp)

#target_link_libraries(${PROJECT_NAME} another_library)
#rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)
#target_link_libraries(example ${PROJECT_NAME})

rospack_add_executable(novelty_estimator_node src/novelty_estimator_node.cpp)
target_link_libraries(novelty_estimator_node ${PROJECT_NAME})
			       
rospack_add_executable (test_novelty_estimator test/test_novelty_estimator.cpp)
target_link_libraries(test_novelty_estimator ${PROJECT_NAME})
rospack_add_gtest_build_flags(test_novelty_estimator)

rospack_add_rostest(test/test_novelty_estimator.launch)
