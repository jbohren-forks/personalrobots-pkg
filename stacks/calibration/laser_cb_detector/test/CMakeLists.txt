set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

# ********** Tests **********

rospack_add_gtest(test/cv_laser_bridge_unittest     cv_laser_bridge_unittest.cpp)
target_link_libraries(test/cv_laser_bridge_unittest laser_cb_detector)
