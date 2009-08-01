cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rospack(new_descriptors_3d)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#genmsg()
#uncomment if you have defined services
#gensrv()

#common commands for building c++ executables and libraries
rospack_add_library(${PROJECT_NAME} 
			src/descriptor_3d.cpp
			src/neighborhood_feature.cpp
			src/bounding_box_raw.cpp
			src/spectral_analysis.cpp
			src/bounding_box_spectral.cpp
			src/shape_spectral.cpp
			src/orientation_generic.cpp
			src/orientation_normal.cpp
			src/orientation_tangent.cpp
			src/position.cpp
			src/spin_image_generic.cpp
			src/spin_image_normal.cpp
			src/spin_image_tangent.cpp
			src/spin_image_custom.cpp)
rospack_add_openmp_flags (${PROJECT_NAME})
			

#target_link_libraries(${PROJECT_NAME} another_library)
#rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)

rospack_add_executable(interest_point_example examples/interest_point_example.cpp)
target_link_libraries(interest_point_example ${PROJECT_NAME})
rospack_add_openmp_flags (interest_point_example)


