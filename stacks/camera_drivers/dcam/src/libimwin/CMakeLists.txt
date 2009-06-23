rospack_add_library(imwin imwin.cpp im3Dwin.cpp)
rospack_add_compile_flags(imwin -msse3)
