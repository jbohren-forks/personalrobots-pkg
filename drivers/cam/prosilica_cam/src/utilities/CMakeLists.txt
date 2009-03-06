rospack_add_executable(write_memory write_memory.cpp)
target_link_libraries(write_memory prosilica)

rospack_add_executable(calibration calibration.cpp ../cvcalibinit.cpp)

rospack_add_executable(yml2ini yml2ini.cpp)
