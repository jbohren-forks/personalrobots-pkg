cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rospack(planar_objects)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
genmsg()
#uncomment if you have defined services
#gensrv()

#common commands for building c++ executables and libraries
#rospack_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)
#rospack_add_executable(rectangular_fit src/rectangular_fit.cpp)

rospack_add_executable(box_detector src/box_detector.cpp src/find_planes.cpp src/vis_utils.cpp src/cornercandidate.cpp)

rospack_add_executable(track_publisher src/track_publisher.cpp)

rospack_add_executable(track_visualizer src/track_visualizer.cpp src/vis_utils.cpp  src/track_utils.cpp)

rospack_add_executable(mocap_eval src/mocap_eval.cpp src/vis_utils.cpp  src/track_utils.cpp)

rospack_add_executable(eval_saver src/eval_saver.cpp)

#rospack_add_executable(stereo_throttle src/stereo_throttle)

rospack_add_executable(box_tracker src/box_tracker.cpp src/vis_utils.cpp src/track_utils.cpp)

rospack_add_executable(articulation_learner src/articulation_learner.cpp src/vis_utils.cpp src/track_utils.cpp src/articulation_models.cpp)

#target_link_libraries(example ${PROJECT_NAME})
