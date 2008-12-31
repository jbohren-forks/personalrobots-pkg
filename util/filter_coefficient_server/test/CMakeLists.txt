rospack_add_executable(check_function_calls check_function_calls.cpp)
rospack_add_gtest_build_flags(check_function_calls)
set_target_properties(check_function_calls PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(check_function_calls ros)
rospack_add_rostest_future(${CMAKE_CURRENT_SOURCE_DIR}/check_function_calls.xml)                    

