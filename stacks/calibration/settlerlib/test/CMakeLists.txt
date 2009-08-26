set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

# ********** Tests **********

rospack_add_gtest(test/sorted_deque_unittest sorted_deque_unittest.cpp)

rospack_add_gtest(test/interval_calc_unittest interval_calc_unittest.cpp)
target_link_libraries(test/interval_calc_unittest settlerlib)