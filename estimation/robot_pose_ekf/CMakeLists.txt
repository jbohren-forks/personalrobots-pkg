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
