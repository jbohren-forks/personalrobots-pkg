cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(robot_pose_ekf)
genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_boost_directories()

rospack_add_executable(robot_pose_ekf 
                       src/odom_estimation.cpp 
                       src/nonlinearanalyticconditionalgaussianodo.cpp 
                       src/odom_estimation_node.cpp)
rospack_link_boost(robot_pose_ekf thread)

rospack_add_executable(test_robot_pose_ekf 
                       test/test_robot_pose_ekf.cpp)
rospack_link_boost(test_robot_pose_ekf thread)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_robot_pose_ekf.launch)

# Download needed data file
rospack_download_test_data(http://pr.willowgarage.com/data/robot_pose_ekf/ekf_test.bag test/ekf_test.bag)
