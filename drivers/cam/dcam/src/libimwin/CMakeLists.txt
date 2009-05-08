FIND_PACKAGE(FLTK REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)

rospack_add_library(imwin imwin.cpp im3Dwin.cpp)
rospack_add_compile_flags(imwin -msse3)
target_link_libraries(imwin ${FLTK_LIBRARIES} ${OPENGL_LIBRARIES})
