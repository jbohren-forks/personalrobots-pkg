cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(kinematic_calibration)
genmsg()
gensrv()
rospack_add_library(kinematic_calibration src/link_param_jacobian.cpp
                                          src/link_param_jacobian_solver.cpp
                                          src/verify_jacobian.cpp
                                          src/jac_newmat_bridge.cpp
                                          src/chain_modifier.cpp
                                          )

rospack_add_executable(arm_phasespace_grabber src/arm_phasespace_grabber.cpp)
target_link_libraries(arm_phasespace_grabber kinematic_calibration)

rospack_add_executable(sensor_kinematics_grabber src/sensor_kinematics_grabber.cpp)
target_link_libraries(sensor_kinematics_grabber kinematic_calibration)

# ********** Tests **********
rospack_add_gtest(jacobian_unittest test/jacobian_unittest.cpp)
target_link_libraries(jacobian_unittest kinematic_calibration)
set_target_properties(jacobian_unittest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

rospack_add_gtest(bridge_unittest test/bridge_unittest.cpp)
target_link_libraries(bridge_unittest kinematic_calibration)

rospack_add_gtest(chain_modifier_unittest test/chain_modifier_unittest.cpp)
target_link_libraries(chain_modifier_unittest kinematic_calibration)
set_target_properties(chain_modifier_unittest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
