cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rospack(sample_consensus)
add_definitions(-Wall)

rospack_add_library(sample_consensus src/sac.cpp src/sac_model.cpp
                    src/lmeds.cpp src/ransac.cpp src/msac.cpp src/mlesac.cpp src/rransac.cpp src/rmsac.cpp
                    src/sac_model_plane.cpp src/sac_model_line.cpp
                    src/sac_model_circle.cpp src/sac_model_sphere.cpp
                    src/sac_model_cylinder.cpp)

rospack_add_gtest(bin/test_line_fit test/test_line_fit.cpp)
rospack_add_gtest(bin/test_plane_fit test/test_plane_fit.cpp)
rospack_add_gtest(bin/test_circle_fit test/test_circle_fit.cpp)
rospack_add_gtest(bin/test_sphere_fit test/test_sphere_fit.cpp)
rospack_add_gtest(bin/test_cylinder_fit test/test_cylinder_fit.cpp)

target_link_libraries(bin/test_line_fit sample_consensus lapack)
target_link_libraries(bin/test_plane_fit sample_consensus lapack)
target_link_libraries(bin/test_circle_fit sample_consensus lapack)
target_link_libraries(bin/test_sphere_fit sample_consensus lapack)
target_link_libraries(bin/test_cylinder_fit sample_consensus lapack cloud_geometry)
