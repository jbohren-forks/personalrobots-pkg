rospack_add_executable(ost ost.cpp stereogui.cpp calwin.cpp)
rospack_add_compile_flags(ost -msse3)
target_link_libraries(ost dcam imwin fltk fltk_gl glut)
