rospack_add_library(wge100camera host_netutil.c list.c wge100lib.c)
rospack_add_compile_flags(wge100camera --std=gnu99)
