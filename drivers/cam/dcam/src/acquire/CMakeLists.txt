rospack_add_executable(acquire acquire.cpp)
target_link_libraries(acquire dcam fltk)

rospack_add_executable(stacquire stacquire.cpp)
target_link_libraries(stacquire dcam imwin fltk fltk_gl glut)
