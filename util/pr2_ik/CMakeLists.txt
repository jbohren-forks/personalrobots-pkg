cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE Release)
add_definitions(-Wall)
rospack(pr2_ik)

rospack_add_library(pr2_ik
 src/pr2_ik.cpp
 src/pr2_ik_solver.cpp)


#rospack_add_executable(pr2_ik_single_test test/test_ik_single.cpp)
#target_link_libraries(pr2_ik_single_test pr2_ik)

#rospack_add_executable(pr2_ik_single_search_test test/test_ik_search_single.cpp)
#target_link_libraries(pr2_ik_single_search_test pr2_ik)

rospack_add_executable(test_ik_simple test/test_ik_simple.cpp)
target_link_libraries(test_ik_simple pr2_ik)

#rospack_add_gtest(pr2_ik_regression_test test/test_ik_regress.cpp)
#target_link_libraries(pr2_ik_regression_test pr2_ik)
#rospack_declare_test(bin/pr2_ik_regression_test)
