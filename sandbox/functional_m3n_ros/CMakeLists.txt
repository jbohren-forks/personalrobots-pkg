cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rospack(functional_m3n_ros)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#genmsg()
#uncomment if you have defined services
gensrv()

#common commands for building c++ executables and libraries
#rospack_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)
#rospack_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rospack_add_executable (m3n_prediction_node src/m3n_prediction_node.cpp)
target_link_libraries (m3n_prediction_node functional_m3n)

rospack_add_executable (m3n_learning_node src/m3n_learning_node.cpp)
target_link_libraries (m3n_learning_node functional_m3n)
rospack_link_boost(m3n_learning_node filesystem)


rospack_download_test_data(http://pr.willowgarage.com/data/${PROJECT_NAME}/pcd_all_1.bag test_data/pcd_all_1.bag)
rospack_download_test_data(http://pr.willowgarage.com/data/${PROJECT_NAME}/pcd_all_1.index test_data/pcd_all_1.index)
rospack_download_test_data(http://pr.willowgarage.com/data/${PROJECT_NAME}/pcd_test_1.bag test_data/pcd_test_1.bag)
rospack_download_test_data(http://pr.willowgarage.com/data/${PROJECT_NAME}/pcd_train_1.bag test_data/pcd_train_1.bag)

rospack_add_rostest(test/test_full_training.xml)


