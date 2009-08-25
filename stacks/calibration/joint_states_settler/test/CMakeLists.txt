set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

# ********** Tests **********
rospack_add_gtest(test/joint_states_deflater_unittest joint_states_deflater_unittest.cpp)
target_link_libraries(test/joint_states_deflater_unittest joint_states_settler)

