cmake_minimum_required(VERSION 2.6)
include(rosbuild)
add_definitions(-Wall)
rospack(issue_kinematic_plan)

rospack_add_executable(bin/issue_kinematic_plan
src/issue_kinematic_plan.cpp)
rospack_add_executable(bin/test_issue_kinematic_plan test/test_issue_kinematic_plan.cpp)


