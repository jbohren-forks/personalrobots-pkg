cmake_minimum_required(VERSION 2.6)

include(rosbuild)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(sample_consensus)
add_definitions(-Wall)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_library(sample_consensus src/sac.cpp src/sac_model.cpp
                    src/lmeds.cpp src/ransac.cpp src/msac.cpp src/mlesac.cpp src/rransac.cpp src/rmsac.cpp
                    src/sac_model_plane.cpp src/sac_model_line.cpp)

rospack_add_gtest(test_plane_fit test/test_plane_fit.cpp)
target_link_libraries(test_plane_fit sample_consensus lapack)

rospack_add_gtest(test_line_fit test/test_line_fit.cpp)
target_link_libraries(test_line_fit sample_consensus lapack)
