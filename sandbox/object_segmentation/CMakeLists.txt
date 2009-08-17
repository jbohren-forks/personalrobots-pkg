cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rospack(object_segmentation)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#genmsg()
#uncomment if you have defined services
#gensrv()

# Library to generate RandomFields from 3-D point clouds
rospack_add_library(rf_creator_3d
                        src/util/rf_creator_3d.cpp)
rospack_add_openmp_flags (rf_creator_3d)

# Library to create RandomFields from Stereo point cloud + image data
rospack_add_library(table_object_rf
			src/image_stereo/table_object_rf.cpp)
rospack_add_openmp_flags (table_object_rf)
target_link_libraries(table_object_rf rf_creator_3d)

# random field creator for table object segmenter
rospack_add_executable(rf_table_object_segmenter 
			src/image_stereo/rf_table_object_segmenter.cpp)
target_link_libraries(rf_table_object_segmenter table_object_rf)

# example program for training m3n
rospack_add_executable(ex_train_m3n
                        examples/train_m3n.cpp 
			examples/common.cpp)
target_link_libraries(ex_train_m3n rf_creator_3d)

# example program for training m3n
rospack_add_executable(ex_classify_m3n
                        examples/classify_m3n.cpp 
                        examples/common.cpp)
target_link_libraries(ex_classify_m3n rf_creator_3d)


#target_link_libraries(${PROJECT_NAME} another_library)
rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)
#rospack_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
