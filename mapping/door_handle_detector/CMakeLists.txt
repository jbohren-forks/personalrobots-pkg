cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(door_handle_detector)
gensrv()
genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# door detector
rospack_add_executable (doors_detector_laser
                        src/geometric_helper.cpp
                        src/doors_detector.cpp
                       )

# handle detector
rospack_add_executable (handle_detector_laser
                        src/geometric_helper.cpp
                        src/handle_detector.cpp
                       )

# handle detector
rospack_add_executable(handle_detector_vision
                       src/handle_detector_vision.cpp 
                      )

rospack_add_executable(handle_detector_vision src/handle_detector_vision.cpp)
rospack_add_executable(test_detect_service test/test_detect_service.cpp)
rospack_add_executable(test_preempt_service test/test_preempt_service.cpp)

##############################################
# Regression testing

# Helper macro
macro(rostest_door_handle _test_name)
    rospack_download_test_data(http://pr.willowgarage.com/data/door_handle_detector/${_test_name}.bag test/${_test_name}.bag)
    rospack_add_rostest(test/${_test_name}.launch)
endmacro(rostest_door_handle)

# We use the point_cloud_cropper in the tests, to simulate a narrow scan of
# the handle
rospack_add_executable(point_cloud_cropper test/point_cloud_cropper.cpp)

rostest_door_handle(door_closed_39)
rostest_door_handle(door_closed_conference_1)
rostest_door_handle(door_closed_josh)
rostest_door_handle(door_closed_kurt)
rostest_door_handle(door_closed_sachin)
rostest_door_handle(door_closed_tully)

# Currently failing tests below
#rostest_door_handle(door_closed_melonee)
#rostest_door_handle(door_closed_40)
#rostest_door_handle(door_closed_alex)
#rostest_door_handle(door_closed_blaise)
#rostest_door_handle(door_closed_conor)
#rostest_door_handle(door_closed_curt)
#rostest_door_handle(door_closed_daniel)
#rostest_door_handle(door_closed_ethan)
#rostest_door_handle(door_closed_john)
#rostest_door_handle(door_closed_melonee_2)
#rostest_door_handle(door_closed_melonee_3)
#rostest_door_handle(door_closed_patrick)
#rostest_door_handle(door_closed_victoria)
#rostest_door_handle(door_closed_vijay)
