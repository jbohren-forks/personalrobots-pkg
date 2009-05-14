set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

# ********** Tests **********
rospack_add_gtest(test/mono_checkerboard_unittest mono_checkerboard_unittest.cpp)
target_link_libraries(test/mono_checkerboard_unittest stereo_checkerboard_detector)

rospack_add_gtest(test/reprojection_unittest reprojection_unittest.cpp)
target_link_libraries(test/reprojection_unittest stereo_checkerboard_detector)

rospack_add_gtest(test/pose_unittest pose_unittest.cpp)
target_link_libraries(test/pose_unittest stereo_checkerboard_detector)
