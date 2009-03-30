rospack_add_library(forearmcam host_netutil.c list.c pr2lib.c)
rospack_add_compile_flags(forearmcam --std=gnu99)
