cmake_minimum_required(VERSION 2.4.6)
set(ROS_BUILD_TYPE Release)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(sbpl)
include_directories(${PROJECT_SOURCE_DIR}/code)
rospack_add_library(sbpl 
                         src/discrete_space_information/nav2d/environment_nav2D.cpp
                         src/discrete_space_information/nav3dkin/environment_nav3Dkin.cpp
                         src/discrete_space_information/navxythetalat/environment_navxythetalat.cpp
                         src/discrete_space_information/nav2d_uu/environment_nav2Duu.cpp
                         src/discrete_space_information/template/environment_XXX.cpp
                         src/discrete_space_information/robarm/environment_robarm.cpp
                         src/planners/ARAStar/araplanner.cpp
                         src/planners/ADStar/adplanner.cpp
                         src/planners/VI/viplanner.cpp
                         src/planners/PPCP/ppcpplanner.cpp
                         src/utils/heap.cpp
                         src/utils/mdp.cpp
                         src/utils/utils.cpp
						 src/utils/2Dgridsearch.cpp
			 )
# Build executables in the bin directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(test_sbpl src/test/main.cpp)
target_link_libraries(test_sbpl sbpl)

rospack_add_executable(test_adjacency_list src/test/test_adjacency_list.cpp)
target_link_libraries(test_adjacency_list sbpl)

## Test target for module tests to be included in gtest regression test harness
#rospack_add_gtest_future(utest src/test/module-tests.cpp)
#target_link_libraries(utest sbpl)
## Send output of tests to the test directory
#set_target_properties(utest PROPERTIES
#                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/src/test)

# Download needed data file
rospack_download_test_data(http://pr.willowgarage.com/data/sbpl/env_examples/nav2d/willow-25mm-inflated-env.cfg env_examples/nav2d/willow-25mm-inflated-env.cfg)
