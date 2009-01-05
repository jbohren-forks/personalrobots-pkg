rospack_add_executable(dcam_exe dcam.cpp)
target_link_libraries(dcam_exe dcam)
SET_TARGET_PROPERTIES(dcam_exe PROPERTIES OUTPUT_NAME dcam)

rospack_add_executable(stereodcam stereodcam.cpp)
target_link_libraries(stereodcam dcam)

rospack_add_executable(check_params check_params.cpp)
