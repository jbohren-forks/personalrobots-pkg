cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions(-Wall)
rospack(libKinematics)
rospack_add_library(libKinematics src/pr2_ik.cpp src/kinematics.cpp)
rospack_add_compile_flags(libKinematics "-O2")

rospack_add_executable(bin/libKinematicsTest src/test/test_PR2ik.cpp)
target_link_libraries(bin/libKinematicsTest libKinematics)

rospack_add_executable(bin/test_ik src/test/test_ik.cpp)
#rospack_add_compile_flags(test_ik "-pg")
#rospack_add_link_flags(test_ik "-pg")
target_link_libraries(bin/test_ik libKinematics)

rospack_add_executable(bin/test_ik_fast src/test/test_ik_fast.cpp)
target_link_libraries(bin/test_ik_fast libKinematics)

rospack_add_executable(bin/test_ik_fast_single src/test/test_ik_fast_single.cpp)
target_link_libraries(bin/test_ik_fast_single libKinematics)
