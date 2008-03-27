SRC = print_scans.cpp
OUT = print_scans
PKG = hokuyo_top_urg
CFLAGS = -I$(shell $(ROS_ROOT)/rospack find serial_port)/include -I../../include
LFLAGS = -L$(shell $(ROS_ROOT)/rospack find serial_port)/lib -L../../lib -lhokuyo_top_urg -lserial_port
include $(shell $(ROS_ROOT)/rospack find roscpp)/make_include/only_rules.mk
