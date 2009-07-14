cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#set(ROS_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(outlet_detection)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

gensrv()
rospack_add_boost_directories()

# Outlet detection library
rospack_add_library(outlet src/affine_transform.cpp src/chamfer_match.cpp src/constellation.cpp src/features.cpp src/learning.cpp src/one_way_descriptor.cpp src/one_way_descriptor_base.cpp src/one_way_outlets.cpp src/outlet_detector.cpp src/outlet_model.cpp src/outlet_tuple.cpp src/pca_features.cpp src/planar.cpp src/tracker_base.cpp src/outlet_tracker.cpp src/generalized_hough.cpp)
add_definitions(-D_OUTLET_HR -D_TUNING -D_USE_OUTLET_TUPLE)
#add_definitions(-D_VERBOSE)
rospack_link_boost(outlet thread)

# Stand-alone outlet detection executables
#rospack_add_executable(outlet_model src/main.cpp)
#target_link_libraries(outlet_model outlet)
rospack_add_executable(outlet_sample src/sample.cpp)
target_link_libraries(outlet_sample outlet)
rospack_add_executable(outlet_measure src/CalculateRealCoordinates.cpp src/OutletsMeasurement.cpp)
target_link_libraries(outlet_measure outlet)

# Fine outlet detection node
rospack_add_executable(outlet_node src/outlet_node.cpp)
target_link_libraries(outlet_node outlet)

# Far outlet spotting node
#rospack_add_executable(outlet_spotting src/outlet_spotting.cpp src/outlet_tuple_coarse.cpp)
rospack_add_executable(outlet_spotting2 src/outlet_spotting2.cpp src/outlet_util.cpp)
rospack_add_executable(test_spotting_service src/test_spotting_service.cpp)

# Plug tracking
rospack_add_library(plug src/tracker_base.cpp src/plug_tracker.cpp src/cvcalibinit.cpp)
rospack_link_boost(plug thread)

rospack_add_executable(plug_node src/plug_node.cpp)
target_link_libraries(plug_node plug)

rospack_add_executable(calibrate_plug src/calibrate_plug.cpp)

##############################################
# Regression testing

###########################################
# Outlet coarse detector
###########################################
macro(rostest_outlet_coarse _test_name)
    rospack_download_test_data(http://pr.willowgarage.com/data/outlet_detection/outlet_test_${_test_name}.bag test/outlet_test_${_test_name}.bag)
    rospack_download_test_data(http://pr.willowgarage.com/data/outlet_detection/outlet_test_${_test_name}.pose test/outlet_test_${_test_name}.pose)
    rospack_add_rostest_future(test/outlet_test_${_test_name}.launch)
endmacro(rostest_outlet_coarse)

rostest_outlet_coarse(1)
rostest_outlet_coarse(4)
rostest_outlet_coarse(4_2)
rostest_outlet_coarse(4_3)
rostest_outlet_coarse(6)
rostest_outlet_coarse(16_light_off)
rostest_outlet_coarse(16_light_on)
rostest_outlet_coarse(20)
rostest_outlet_coarse(21)
rostest_outlet_coarse(21b)
rostest_outlet_coarse(24)
rostest_outlet_coarse(25)
rostest_outlet_coarse(26)
rostest_outlet_coarse(26_false)
rostest_outlet_coarse(27)
rostest_outlet_coarse(39)
rostest_outlet_coarse(39_far)
rostest_outlet_coarse(40_no_detection)
rostest_outlet_coarse(40_no_detection2)
rostest_outlet_coarse(40_no_detection3)
rostest_outlet_coarse(40_no_detection4)



###########################################
# Outlet fine detector
###########################################
# TODO: move the tarball-unpacking logic into rosbuild
#################
# Test 1
# Download the tarball of images
rospack_download_test_data(http://pr.willowgarage.com/data/outlet_detection/outlets_2009_05_20_part1.tgz test/outlets_2009_05_20_part1.tgz)
# Unpack the tarball.
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/test/outlets_2009_05_20_part1
                   COMMAND tar zxvf outlets_2009_05_20_part1.tgz
                   COMMAND touch outlets_2009_05_20_part1
                   DEPENDS ${PROJECT_SOURCE_DIR}/test/outlets_2009_05_20_part1.tgz
                   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
                   VERBATIM)
# Run a test on that data
rospack_add_rostest_future(test/test_outlets_2009_05_20_part1.xml)

#################
# Test 2
# Download the tarball of images
rospack_download_test_data(http://pr.willowgarage.com/data/outlet_detection/outlets_2009_05_20_part2.tgz test/outlets_2009_05_20_part2.tgz)
# Unpack the tarball.
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/test/outlets_2009_05_20_part2
                   COMMAND tar zxvf outlets_2009_05_20_part2.tgz
                   COMMAND touch outlets_2009_05_20_part2
                   DEPENDS ${PROJECT_SOURCE_DIR}/test/outlets_2009_05_20_part2.tgz
                   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
                   VERBATIM)
# Run a test on that data

# Set up dependencies so that we unpack the data before running tests
add_custom_target(download_and_unpack_test_data
                  DEPENDS ${PROJECT_SOURCE_DIR}/test/outlets_2009_05_20_part1
                          ${PROJECT_SOURCE_DIR}/test/outlets_2009_05_20_part2)
add_dependencies(tests download_and_unpack_test_data)
