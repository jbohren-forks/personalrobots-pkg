FIND_PACKAGE(FLTK REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)

rospack_add_executable(ost ost.cpp stereogui.cpp calwin.cpp lodepng.cpp ostpng.cpp)
rospack_add_compile_flags(ost -msse3)
target_link_libraries(ost dcam imwin ${FLTK_LIBRARIES} ${OPENGL_LIBRARIES})
