FIND_PACKAGE(FLTK REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)

rospack_add_executable(acquire acquire.cpp)
rospack_add_compile_flags(acquire -msse3)
target_link_libraries(acquire dcam ${FLTK_LIBRARIES})

rospack_add_executable(stacquire stacquire.cpp)
rospack_add_compile_flags(stacquire -msse3)
target_link_libraries(stacquire dcam imwin ${FLTK_LIBRARIES} ${OPENGL_LIBRARIES})
