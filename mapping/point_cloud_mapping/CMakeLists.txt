cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(point_cloud_mapping)
rospack_add_boost_directories()
#add_definitions(-Wall -O3 -DNDEBUG -march=pentium4 -mfpmath=sse -mmmx -msse -msse2 -msse3)

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable (bin/cloud_downsampler_node src/cloud_downsampler.cpp)
target_link_libraries (bin/cloud_downsampler_node cloud_geometry cloud_kdtree)

rospack_add_executable (bin/normal_estimation_node src/normal_estimation.cpp)
target_link_libraries (bin/normal_estimation_node cloud_geometry cloud_kdtree)

rospack_add_executable (bin/planar_fit_node src/planar_fit.cpp)
target_link_libraries (bin/planar_fit_node cloud_geometry cloud_kdtree)

# ---[ Cloud Geometry library
  rospack_add_library (cloud_geometry
                       src/cloud_geometry/angles.cpp
                       src/cloud_geometry/areas.cpp
                       src/cloud_geometry/distances.cpp
                       src/cloud_geometry/intersections.cpp
                       src/cloud_geometry/nearest.cpp
                       src/cloud_geometry/point.cpp
                       src/cloud_geometry/projections.cpp
                       src/cloud_geometry/statistics.cpp
                       src/cloud_geometry/transforms.cpp
                      )
  rospack_add_gtest (bin/test_geometry_statistics test/cloud_geometry/test_geometry_statistics.cpp)
  rospack_add_gtest (bin/test_geometry_angles     test/cloud_geometry/test_geometry_angles.cpp)

# ---[ Cloud I/O library
  rospack_add_library (cloud_io
                       src/cloud_io/read.cpp
                       src/cloud_io/write.cpp
                       src/cloud_io/misc.cpp
                      )
  rospack_add_executable (bin/cloudmsg_to_screen src/cloud_io/tools/cloudmsg_to_screen.cpp)
  target_link_libraries (bin/cloudmsg_to_screen cloud_io)

  rospack_add_executable (bin/bag_to_pcd_node src/cloud_io/tools/bag_to_pcd.cpp)
  target_link_libraries (bin/bag_to_pcd_node cloud_io cloud_geometry cloud_kdtree)

  rospack_add_executable (bin/pcd_to_msg_node src/cloud_io/tools/pcd_to_msg.cpp)
  target_link_libraries (bin/pcd_to_msg_node cloud_io)

  rospack_add_gtest (bin/test_cloud_io test/cloud_io/test_io.cpp)
  target_link_libraries (bin/test_cloud_io cloud_io)

# ---[ Cloud Kd-Tree library
  rospack_add_library (cloud_kdtree
                       src/cloud_kdtree/kdtree_ann.cpp
                       src/cloud_kdtree/kdtree_flann.cpp
                      )

  rospack_add_gtest (bin/test_cloud_kdtree test/cloud_kdtree/test_kdtree.cpp)
  target_link_libraries (bin/test_cloud_kdtree cloud_kdtree)

# ---[ Cloud Octree library
  rospack_add_library (cloud_octree
                       src/cloud_octree/octree.cpp
                       src/cloud_octree/octree_node.cpp
                      )

# ---[ Sample-Consensus library
  rospack_add_library (sample_consensus
                       src/sample_consensus/sac.cpp
                       src/sample_consensus/sac_model.cpp
                       src/sample_consensus/lmeds.cpp
                       src/sample_consensus/ransac.cpp
                       src/sample_consensus/msac.cpp
                       src/sample_consensus/mlesac.cpp
                       src/sample_consensus/rransac.cpp
                       src/sample_consensus/rmsac.cpp
                       src/sample_consensus/sac_model_plane.cpp
                       src/sample_consensus/sac_model_oriented_plane.cpp
                       src/sample_consensus/sac_model_line.cpp
                       src/sample_consensus/sac_model_oriented_line.cpp
                       src/sample_consensus/sac_model_circle.cpp
                       src/sample_consensus/sac_model_sphere.cpp
                       src/sample_consensus/sac_model_cylinder.cpp
                       src/sample_consensus/sac_model_parallel_lines.cpp
                      )
  target_link_libraries(sample_consensus cloud_geometry)
  rospack_add_gtest (bin/test_sample_consensus_line_fit test/sample_consensus/test_line_fit.cpp)
  rospack_add_gtest (bin/test_sample_consensus_parallel_lines_fit test/sample_consensus/test_parallel_lines_fit.cpp)
  rospack_add_gtest (bin/test_sample_consensus_plane_fit test/sample_consensus/test_plane_fit.cpp)
  rospack_add_gtest (bin/test_sample_consensus_circle_fit test/sample_consensus/test_circle_fit.cpp)
  rospack_add_gtest (bin/test_sample_consensus_sphere_fit test/sample_consensus/test_sphere_fit.cpp)
  rospack_add_gtest (bin/test_sample_consensus_cylinder_fit test/sample_consensus/test_cylinder_fit.cpp)

  target_link_libraries (bin/test_sample_consensus_line_fit sample_consensus cloud_geometry cloud_kdtree)
  target_link_libraries (bin/test_sample_consensus_parallel_lines_fit sample_consensus cloud_geometry cloud_kdtree)
  target_link_libraries (bin/test_sample_consensus_plane_fit sample_consensus cloud_geometry cloud_kdtree)
  target_link_libraries (bin/test_sample_consensus_circle_fit sample_consensus cloud_geometry cloud_kdtree)
  target_link_libraries (bin/test_sample_consensus_sphere_fit sample_consensus cloud_geometry cloud_kdtree)
  target_link_libraries (bin/test_sample_consensus_cylinder_fit sample_consensus cloud_geometry cloud_kdtree)


# check for OpenMP
include(CheckIncludeFile)
include(CheckCXXCompilerFlag)
include(CheckLibraryExists)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  # check if compilers supports -fopenmp
  check_cxx_compiler_flag (-fopenmp HAVE_OPENMP)
  check_library_exists (gomp omp_get_num_threads "" HAS_GOMP_LIB)

  if(HAVE_OPENMP AND HAS_GOMP_LIB)
    add_definitions ("-fopenmp")
    target_link_libraries (bin/normal_estimation_node gomp)
    target_link_libraries (bin/planar_fit_node gomp)
    set (OPENMP_LFLAGS "-lgomp")
  endif(HAVE_OPENMP AND HAS_GOMP_LIB)
endif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
