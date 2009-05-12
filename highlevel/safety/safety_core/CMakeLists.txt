cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(safety_core)
gensrv()
genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(action_runner src/action_detect_plug_on_base.cpp
                       src/action_tuck_arms.cpp
                       src/action_doors_tuck_arms.cpp
		       src/action_runner.cpp)

rospack_add_executable(run_detect_plug_on_base
  src/run_detect_plug_on_base.cpp
  src/action_detect_plug_on_base.cpp
  )
rospack_add_executable(run_doors_tuck_arms
  src/run_doors_tuck_arms.cpp
  src/action_doors_tuck_arms.cpp
  )
rospack_add_executable(run_tuck_arms
  src/run_tuck_arms.cpp
  src/action_tuck_arms.cpp
  )
