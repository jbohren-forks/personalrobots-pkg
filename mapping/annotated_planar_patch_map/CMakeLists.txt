cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(annotated_planar_patch_map)

gensrv()

rospack_add_boost_directories()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(annotation2d_lifter_to_planar_patch_map_node src/annotation2d_lifter_to_planar_patch_map.cpp)

rospack_add_executable(annotated_map_assembler_srv src/annotated_map_assembler_srv.cpp)
rospack_add_executable(annotated_map_snapshotter src/annotated_map_snapshotter.cpp)
rospack_add_executable(annotated_map_query_snapshotter src/annotated_map_query_snapshotter.cpp)




