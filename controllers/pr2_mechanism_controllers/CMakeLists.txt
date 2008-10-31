cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(pr2_mechanism_controllers)
genmsg()
gensrv()
#rospack_add_library(pr2_controllers src/base_controller.cpp src/laser_scanner_controller.cpp  src/arm_position_controller.cpp src/arm_velocity_controller.cpp src/arm_pd_controller.cpp src/head_pan_tilt_controller.cpp)

rospack_add_library( pr2_mechanism_controllers
  src/base_controller.cpp
  src/laser_scanner_controller.cpp
  src/laser_scanner_velocity_controller.cpp
  src/arm_position_controller.cpp
  src/arm_velocity_controller.cpp
  src/arm_dynamics_controller.cpp
  src/head_pan_tilt_controller.cpp
  src/caster_controller.cpp
  src/caster_calibration_controller.cpp
  src/wrist_calibration_controller.cpp
  )

rospack_add_executable(testBaseController test/test_base_controller.cpp)
target_link_libraries(testBaseController pr2_mechanism_controllers)

rospack_add_executable(testRunBaseController test/test_run_base_controller.cpp)
target_link_libraries(testRunBaseController pr2_mechanism_controllers)

rospack_add_executable(testArmDynamicsController test/test_arm_dynamics_controller.cpp)
target_link_libraries(testRunBaseController pr2_mechanism_controllers)

rospack_add_executable(testOdom test/test_odom.cpp)
target_link_libraries(testOdom pr2_mechanism_controllers)
