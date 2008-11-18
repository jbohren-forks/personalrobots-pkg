cmake_minimum_required(VERSION 2.4)
include(rosbuild)
rospack(robot_pose_ekf)
genmsg()
rospack_add_executable(robot_pose_ekf 
                       src/odom_estimation.cpp 
                       src/nonlinearanalyticconditionalgaussianodo.cpp 
                       src/odom_estimation_node.cpp)
