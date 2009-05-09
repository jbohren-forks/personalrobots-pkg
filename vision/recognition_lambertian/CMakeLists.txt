cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(recognition_lambertian)
rospack_add_boost_directories()

#gensrv()
#genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


rospack_add_executable(recognition_lambertian src/recognition_lambertian.cpp src/chamfer_matching.cpp)
rospack_add_executable (plane_fit src/plane_fit.cpp)
rospack_add_executable (publish_scene src/publish_scene.cpp)
rospack_add_executable (prosilica_capture src/prosilica_capture.cpp)
rospack_add_executable(rec_lam_normal_features src/rec_lam_normal_features.cpp src/visualization.cpp)

rospack_add_executable (rectify_image src/rectify_image.cpp)
