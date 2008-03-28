SRC = test_camcontrol.cpp kbhit.cpp
OUT = test_camcontrol
LFLAGS = -L../../lib -laxis_cam -lcurl -lstring_utils
LIBDEPS = ../../lib/libaxis_cam.a
PKG = axis_cam
include $(shell $(ROS_ROOT)/rospack find roscpp)/make_include/node.mk
