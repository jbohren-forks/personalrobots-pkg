cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(pm_wrapper)
gensrv()

# Build executables in the bin directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_library(pm_wrapper
				                    src/pm_wrapper.cpp
		   			       )

rospack_add_executable(test_pm_wrapper  src/test.cpp)
target_link_libraries(test_pm_wrapper pm_wrapper)