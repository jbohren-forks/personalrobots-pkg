cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(chamfer_matching)

rospack_add_library(chamfer_matching src/chamfer_matching.cpp)

#rospack_add_executable(test_chamfer src/test_chamfer.cpp)
#target_link_libraries(test_chamfer chamfer_matching)

#rospack_add_executable(test_chamfer2 src/test_chamfer2.cpp)
#target_link_libraries(test_chamfer2 chamfer_matching)


#rospack_add_executable(test_iterator src/test_iterator.cpp)
#target_link_libraries(test_iterator chamfer_matching)
