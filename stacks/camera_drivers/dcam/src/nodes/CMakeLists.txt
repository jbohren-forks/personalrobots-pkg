rospack_add_executable(dcam_exe dcam.cpp)
rospack_add_compile_flags(dcam_exe -msse3)
target_link_libraries(dcam_exe dcam)
SET_TARGET_PROPERTIES(dcam_exe PROPERTIES OUTPUT_NAME dcam)

rospack_add_executable(stereodcam stereodcam.cpp)
rospack_add_compile_flags(stereodcam -msse3)
target_link_libraries(stereodcam dcam)

rospack_add_executable(videre_no_STOC videre_no_STOC.cpp)
target_link_libraries(videre_no_STOC dcam)

rospack_add_executable(check_params check_params.cpp)
