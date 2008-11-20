cmake_minimum_required(VERSION 2.4)

find_package(Boost 0 REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include(rosbuild)
rospack(robot_pose_ekf)
genmsg()
rospack_add_executable(robot_pose_ekf 
                       src/odom_estimation.cpp 
                       src/nonlinearanalyticconditionalgaussianodo.cpp 
                       src/odom_estimation_node.cpp)

target_link_libraries(robot_pose_ekf ${Boost_LIBRARIES})
