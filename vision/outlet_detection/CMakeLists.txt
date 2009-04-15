cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(outlet_detection)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

gensrv()
rospack_add_boost_directories()

add_definitions(-Wall -O3 -DNDEBUG)
#add_definitions(-Wall -O0 -g)

rospack_add_library(outlet src/outlet_model.cpp src/planar.cpp src/outlet_tuple.cpp src/outlet_detector.cpp src/learning.cpp)
rospack_add_executable(outlet_model src/main.cpp)
target_link_libraries(outlet_model outlet)
rospack_add_executable(outlet_sample src/sample.cpp)
target_link_libraries(outlet_sample outlet)
rospack_add_executable(outlet_measure src/CalculateRealCoordinates.cpp src/OutletsMeasurement.cpp)
target_link_libraries(outlet_measure outlet)

add_definitions(-D_OUTLET_HR -D_TUNING -D_USE_OUTLET_TUPLE)
#add_definitions(-D_VERBOSE)

rospack_add_executable(outlet_node src/outlet_node.cpp)
target_link_libraries(outlet_node outlet)

rospack_add_library(plugtracker src/plug_tracker.cpp src/cvcalibinit.cpp)
rospack_add_executable(plug_node src/plug_node.cpp)
target_link_libraries(plug_node plugtracker)

rospack_add_executable(calibrate_plug src/calibrate_plug.cpp)

rospack_add_executable(test_outlet_service src/test_outlet_service.cpp)
