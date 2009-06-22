cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE RelWithDebInfo)

rospack(annotated_planar_patch_map)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#uncomment if you have defined messages
#genmsg()
#uncomment if you have defined services
gensrv()

#common commands for building c++ executables and libraries
#rospack_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)
#rospack_add_executable(pf_learn src/learn.cc)
#target_link_libraries(example ${PROJECT_NAME})

rospack_add_library(annotated_map_lib src/annotated_map_lib.cpp src/projection.cpp src/query.cpp src/binding.cpp)


rospack_add_executable(annotation2d_lifter_to_planar_patch_map_node src/annotation2d_lifter_to_planar_patch_map.cpp)
target_link_libraries(annotation2d_lifter_to_planar_patch_map_node annotated_map_lib)

rospack_add_executable(annotation2d_lifter_to_tagged_patch_map_node src/annotation2d_lifter_to_tagged_patch_map.cpp)
target_link_libraries(annotation2d_lifter_to_tagged_patch_map_node annotated_map_lib)

rospack_add_executable(annotation_lifter_node src/annotation2d_lifter_to_tagged_patch_map_via_service.cpp)
target_link_libraries(annotation_lifter_node annotated_map_lib)

rospack_add_executable(annotated_map_info src/annotated_map_info.cpp)
target_link_libraries(annotated_map_info annotated_map_lib)

#rospack_add_executable(image_value_node src/image_value_node.cpp)
#target_link_libraries(image_value_node annotated_map_lib)

rospack_add_executable(annotated_map_assembler_srv src/annotated_map_assembler_srv.cpp)
target_link_libraries(annotated_map_assembler_srv annotated_map_lib)

rospack_add_executable(annotated_map_snapshotter src/annotated_map_snapshotter.cpp)
rospack_add_executable(annotated_map_query_snapshotter src/annotated_map_query_snapshotter.cpp)


rospack_add_executable(empty_annotated_map src/empty_annotated_map.cpp)
target_link_libraries(empty_annotated_map annotated_map_lib)
