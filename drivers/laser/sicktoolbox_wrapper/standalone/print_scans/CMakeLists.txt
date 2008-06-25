add_executable(print_scans print_scans.cpp)
target_link_libraries(print_scans ${sicktoolbox_wrapper_LIBRARIES} pthread)
