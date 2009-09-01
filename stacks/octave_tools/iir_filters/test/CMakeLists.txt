rospack_add_executable(check_function_calls check_function_calls.cpp)
rospack_add_gtest_build_flags(check_function_calls)
set_target_properties(check_function_calls PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(check_function_calls ros)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/check_function_calls.xml)  


rospack_add_executable(test_iirfilter test_iirfilter.cpp)
rospack_add_gtest_build_flags(test_iirfilter)
set_target_properties(test_iirfilter PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_iirfilter ros)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test_iirfilter.xml)

                

