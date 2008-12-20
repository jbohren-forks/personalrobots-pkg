rospack_add_executable(ost ost.cpp stereogui.cpp calwin.cpp)
target_link_libraries(ost dcam imwin fltk fltk_gl glut)
