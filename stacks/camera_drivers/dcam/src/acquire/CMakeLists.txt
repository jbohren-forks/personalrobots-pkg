rospack_add_executable(acquire acquire.cpp)
rospack_add_compile_flags(acquire -msse3)
target_link_libraries(acquire dcam fltk)

rospack_add_executable(stacquire stacquire.cpp)
rospack_add_compile_flags(stacquire -msse3)
target_link_libraries(stacquire dcam imwin fltk fltk_gl glut)
