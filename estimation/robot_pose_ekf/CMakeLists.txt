cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(robot_pose_ekf)
genmsg()

rospack_add_boost_directories()

rospack_add_executable(robot_pose_ekf 
                       src/odom_estimation.cpp 
                       src/nonlinearanalyticconditionalgaussianodo.cpp 
                       src/odom_estimation_node.cpp)
rospack_link_boost(robot_pose_ekf thread)

rospack_add_executable(test/test_ekf 
                       test/test_ekf.cpp)
rospack_link_boost(test/test_ekf thread)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_ekf.launch)

# Download needed data file
# This code is wrapped up in a new macro, which hasn't yet been merged to
# the stable branch of ROS.
#rospack_download_test_data(http://pr.willowgarage.com/data/calonder_descriptor/current.rtc current.rtc)
set(_url http://pr.willowgarage.com/data/robot_pose_ekf/ekf_test.bag)
set(_filename test/ekf_test.bag)
find_package(Wget REQUIRED)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/${_filename}
                   COMMAND ${WGET_EXECUTABLE} -r ${_url} -O ${PROJECT_SOURCE_DIR}/${_filename}
                   VERBATIM)
# Create a legal target name, in case the target name has slashes in it
string(REPLACE "/" "_" _testname download_data_${_filename})
add_custom_target(${_testname}
                  DEPENDS ${PROJECT_SOURCE_DIR}/${_filename})
add_dependencies(tests ${_testname})
