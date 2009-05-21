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

# Testing
rospack_add_executable(point_cloud_cropper test/point_cloud_cropper.cpp)
rospack_download_test_data(http://pr.willowgarage.com/data/door_handle_detector/door_closed_sachin.bag test/door_closed_sachin.bag)
rospack_add_rostest(test/door_closed_sachin.launch)
