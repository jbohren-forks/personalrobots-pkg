set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

# ********** Tests **********

rospack_add_gtest(test/sorted_deque_unittest sorted_deque_unittest.cpp)
