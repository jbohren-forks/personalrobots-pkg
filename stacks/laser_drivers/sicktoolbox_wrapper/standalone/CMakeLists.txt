rospack_add_executable(print_scans print_scans.cpp)
target_link_libraries(pthread)

rospack_add_executable(time_scans time_scans.cpp)
target_link_libraries(pthread)
