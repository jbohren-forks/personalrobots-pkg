rospack_add_executable(access_register access_register.cpp)
target_link_libraries(access_register forearmcam)

rospack_add_executable(set_name set_name.cpp)
target_link_libraries(set_name forearmcam)

rospack_add_executable(reconfigure_cam reconfigure_cam.cpp)
target_link_libraries(reconfigure_cam forearmcam)

rospack_add_executable(reset_cam reset_cam.cpp)
target_link_libraries(reset_cam forearmcam)

rospack_add_executable(upload_mcs upload_mcs.cpp)
target_link_libraries(upload_mcs forearmcam)

rospack_add_executable(write_memory write_memory.cpp)
target_link_libraries(write_memory forearmcam)

rospack_add_executable(fcam_sim fcam_sim.cc)
target_link_libraries(fcam_sim forearmcam)

rospack_add_executable(discover discover.cpp)
target_link_libraries(discover forearmcam)

rospack_add_executable(killcam killcam.cpp)
target_link_libraries(killcam forearmcam)

rospack_add_executable(check_flash check_flash.cpp)
target_link_libraries(check_flash forearmcam)

rospack_add_executable(read_all_flash read_all_flash.cpp)
target_link_libraries(read_all_flash forearmcam)
