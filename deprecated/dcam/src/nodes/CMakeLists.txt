rospack_add_executable(dcam_exe dcam.cpp)
rospack_add_compile_flags(dcam_exe -msse3)
target_link_libraries(dcam_exe dcam imageproc stereoproc)
SET_TARGET_PROPERTIES(dcam_exe PROPERTIES OUTPUT_NAME dcam)

rospack_add_executable(stereodcam stereodcam.cpp)
rospack_add_compile_flags(stereodcam -msse3)
target_link_libraries(stereodcam dcam imageproc stereoproc)

rospack_add_executable(videre_no_STOC videre_no_STOC.cpp)
rospack_add_compile_flags(videre_no_STOC -msse3)
target_link_libraries(videre_no_STOC dcam imageproc stereoproc)

rospack_add_executable(check_params check_params.cpp)
