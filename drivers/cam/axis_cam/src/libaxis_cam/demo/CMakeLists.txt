set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/libaxis_demos/)

rospack_add_executable(test_get test_get.cpp)
target_link_libraries(test_get axis curl)

rospack_add_executable(test_ptz test_ptz.cpp)
target_link_libraries(test_ptz axis curl)

rospack_add_executable(test_camcontrol test_camcontrol.cpp kbhit.cpp)
target_link_libraries(test_camcontrol axis curl)