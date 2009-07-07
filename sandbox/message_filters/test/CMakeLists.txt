set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

# ********** Tests **********
rospack_add_gtest(test/msg_cache_unittest msg_cache_unittest.cpp)
target_link_libraries(test/msg_cache_unittest message_filters)

rospack_add_gtest(test/time_synchronizer_unittest time_synchronizer_unittest.cpp)
target_link_libraries(test/time_synchronizer_unittest message_filters)

rospack_add_gtest(test/time_sequencer_unittest time_sequencer_unittest.cpp)
target_link_libraries(test/time_sequencer_unittest message_filters)
