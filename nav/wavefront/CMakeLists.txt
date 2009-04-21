cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(wavefront)

rospack_add_boost_directories()

rospack_add_library(wavefront_planner
                    src/heap.c
                    src/plan.c
                    src/plan_control.c
                    src/plan_plan.c
                    src/plan_waypoint.c)

rospack_add_executable(bin/wavefront src/wavefront_node.cpp)
target_link_libraries(bin/wavefront wavefront_planner)
rospack_link_boost(bin/wavefront thread)

# this little program just lets you send and block on goals from the shell
rospack_add_executable(bin/cli src/cli.cpp)
rospack_link_boost(bin/cli thread)

# TODO: Make this program into a real unit test, and wean it off GDK
#rospack_add_executable(bin/test src/test.c)
#target_link_libraries(bin/test wavefront_planner)
