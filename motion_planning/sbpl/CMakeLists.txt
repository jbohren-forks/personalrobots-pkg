cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(sbpl)
include_directories(${PROJECT_SOURCE_DIR}/code)
rospack_add_library(sbpl 
                         src/discrete_space_information/nav2d/environment_nav2D.cpp
                         src/discrete_space_information/nav3dkin/environment_nav3Dkin.cpp
                         src/discrete_space_information/template/environment_XXX.cpp
                         src/discrete_space_information/robarm/environment_robarm.cpp
                         src/planners/ARAStar/araplanner.cpp
                         src/planners/ADStar/adplanner.cpp
                         src/planners/VI/viplanner.cpp
                         src/utils/heap.cpp
                         src/utils/mdp.cpp
                         src/utils/utils.cpp
			 )
# Build executables in the bin directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(test_sbpl src/test/main.cpp)
target_link_libraries(test_sbpl sbpl)

# Test target for module tests to be included in gtest regression test harness
rospack_add_gtest_future(utest src/test/module-tests.cpp)
target_link_libraries(utest sbpl)

# Send output of tests to the test directory
set_target_properties(utest PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/src/test)

