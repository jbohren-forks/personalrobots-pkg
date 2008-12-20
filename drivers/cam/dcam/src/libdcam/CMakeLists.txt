rospack_add_library(dcam dcam.cpp stereocam.cpp)
rospack_add_compile_flags(dcam "-msse2 -mpreferred-stack-boundary=4")
