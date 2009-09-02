cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rospack(rf_detector)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#genmsg()
#uncomment if you have defined services
#gensrv()

#common commands for building c++ executables and libraries
#rospack_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)
#rospack_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#include_directories(${PROJECT_SOURCE_DIR}/include)

macro(rosbuild_download_data _url _filename)
 find_package(Wget REQUIRED)
 add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/${_filename}
                    COMMAND cmake -E echo "[rosbuild] Downloading ${_url} to ${_filename}..."
                    COMMAND ${WGET_EXECUTABLE} -q ${_url} -O ${PROJECT_SOURCE_DIR}/${_filename}
                    COMMAND cmake -E echo "[rosbuild] Done."
                    VERBATIM)
 # Create a legal target name, in case the target name has slashes in it
 string(REPLACE "/" "_" _testname download_data_${_filename})
 add_custom_target(${_testname} ALL
                   DEPENDS ${PROJECT_SOURCE_DIR}/${_filename})
endmacro(rosbuild_download_data)

rosbuild_download_data(http://pr.willowgarage.com/data/rf_detector/model/Pcd_test_hogshape_RFV05/ClassId2ViewObjClassMap.txt model/ClassId2ViewObjClassMap.txt)
rosbuild_download_data(http://pr.willowgarage.com/data/rf_detector/model/Pcd_test_hogshape_RFV05/ClassIdsBinary model/ClassIdsBinary)
rosbuild_download_data(http://pr.willowgarage.com/data/rf_detector/model/Pcd_test_hogshape_RFV05/FgBgProb.txt model/FgBgProb.txt)
rosbuild_download_data(http://pr.willowgarage.com/data/rf_detector/model/Pcd_test_hogshape_RFV05/NormVotesBinary model/NormVotesBinary)
rosbuild_download_data(http://pr.willowgarage.com/data/rf_detector/model/Pcd_test_hogshape_RFV05/RF.model model/RF.model)

rospack_add_executable (rf_pcd_hogshape_detector src/RFPcdHogShapeDetector.cpp src/write.cpp src/shape2.cpp src/Exehog.cpp src/RandomForest.cpp src/stdafx.cpp src/librf/discrete_dist.cpp src/librf/random_forest.cpp src/librf/tree_node.cpp src/librf/instance_set.cpp src/librf/tree.cpp)
target_link_libraries (rf_pcd_hogshape_detector t2obj)
target_link_libraries (rf_pcd_hogshape_detector descriptors_3d)


