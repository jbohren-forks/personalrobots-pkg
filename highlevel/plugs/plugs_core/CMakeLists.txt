cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(plugs_core)
gensrv()
genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(action_runner src/action_untuck_arms.cpp 
                       src/action_move_and_grasp_plug.cpp 
											 src/action_detect_outlet_fine.cpp
                       src/action_localize_plug_in_gripper.cpp
                       src/action_stow_plug.cpp 
				               src/action_runner.cpp)
