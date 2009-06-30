set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

# ********** Tests **********
rospack_add_gtest(test/msg_cache_unittest msg_cache_unittest.cpp)
