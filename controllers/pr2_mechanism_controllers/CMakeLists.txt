cmake_minimum_required(VERSION 2.6)
include(rosbuild)
set(ROS_BUILD_STATIC_LIBS true)
set(ROS_BUILD_SHARED_LIBS false)
rospack(pr2_mechanism_controllers)
genmsg()
gensrv()
#rospack_add_library(pr2_controllers src/base_controller.cpp src/laser_scanner_controller.cpp  src/arm_position_controller.cpp src/arm_velocity_controller.cpp src/arm_pd_controller.cpp src/head_pan_tilt_controller.cpp)

rospack_add_library( pr2_mechanism_controllers
  src/base_controller.cpp
  src/base_controller_pos.cpp
  src/base_position_controller.cpp
  src/laser_scanner_controller.cpp
  src/laser_scanner_traj_controller.cpp
  src/laser_scanner_velocity_controller.cpp
  src/arm_position_controller.cpp
  src/arm_velocity_controller.cpp
  src/arm_dynamics_controller.cpp
  src/head_pan_tilt_controller.cpp
  src/head_servoing_controller.cpp
  src/caster_controller.cpp
  src/caster_calibration_controller.cpp
  src/wrist_calibration_controller.cpp
  src/gripper_calibration_controller.cpp
  src/arm_trajectory_controller.cpp
  )

#rospack_add_executable(testBaseController test/test_base_controller.cpp)
#target_link_libraries(testBaseController pr2_mechanism_controllers)

#rospack_add_executable(testRunBaseController test/test_run_base_controller.cpp)
#target_link_libraries(testRunBaseController pr2_mechanism_controllers)

#rospack_add_executable(testArmDynamicsController test/test_arm_dynamics_controller.cpp)
#target_link_libraries(testRunBaseController pr2_mechanism_controllers)

rospack_add_executable(testOdom test/test_odom.cpp)
target_link_libraries(testOdom pr2_mechanism_controllers)

rospack_add_executable(testTraj test/test_arm_trajectory_controller.cpp)
target_link_libraries(testTraj pr2_mechanism_controllers)
