set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

# ********** Tests **********
rospack_add_gtest(test/jacobian_unittest jacobian_unittest.cpp)
target_link_libraries(test/jacobian_unittest kinematic_calibration)

rospack_add_gtest(test/bridge_unittest bridge_unittest.cpp)
target_link_libraries(test/bridge_unittest kinematic_calibration)

rospack_add_gtest(test/chain_modifier_unittest chain_modifier_unittest.cpp)
target_link_libraries(test/chain_modifier_unittest kinematic_calibration)

rospack_add_gtest(test/error_vec_marker_3d_unittest error_vec_marker_3d_unittest.cpp)
target_link_libraries(test/error_vec_marker_3d_unittest kinematic_calibration)

rospack_add_gtest(test/estimator_marker_3d_unittest estimator_marker_3d_unittest.cpp)
target_link_libraries(test/estimator_marker_3d_unittest kinematic_calibration)

