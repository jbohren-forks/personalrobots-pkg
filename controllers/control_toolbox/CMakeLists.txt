cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(control_toolbox)
genmsg()
gensrv()
include_directories(include/control_toolbox/eigen2)
rospack_add_library(control_toolbox
 src/pid.cpp
 src/serialchain_model.cpp
 src/sine_sweep.cpp
 src/ramp_output.cpp
 )
 
rospack_add_gtest(lqr_utils test/lqr_utils.cpp)
set_target_properties(lqr_utils PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

rospack_add_gtest(lqr test/lqr.cpp)
set_target_properties(lqr PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
# rospack_add_executable(test_linear test/linear.cpp)
