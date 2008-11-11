cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(kinematic_calibration)
genmsg()
gensrv()
rospack_add_library(kinematic_calibration src/link_param_jacobian.cpp
                                          src/link_param_jacobian_solver.cpp
                                          src/verify_jacobian.cpp                                          
                                          )

rospack_add_executable(arm_phasespace_grabber src/arm_phasespace_grabber.cpp)
target_link_libraries(arm_phasespace_grabber kinematic_calibration)

rospack_add_gtest(jacobian_unittest test/jacobian_unittest.cpp)
target_link_libraries(jacobian_unittest kinematic_calibration)
set_target_properties(jacobian_unittest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

