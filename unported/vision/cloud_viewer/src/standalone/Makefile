SRC = cloud.cpp
OUT = cloud
PKG = cloud_viewer
CFLAGS = -I../../include
LFLAGS = -L../../lib -lcloud_viewer -lGL -lGLU
#CFLAGS = -I$(shell $(ROS_ROOT)/rospack find sdl)/include -I../../include
#LFLAGS = -L$(shell $(ROS_ROOT)/rospack find sdl)/lib -lSDL -L../../lib -lcloud_viewer -lGL -lGLU
#include $(shell $(ROS_ROOT)/rospack find roscpp)/make_include/only_rules.mk
include $(shell rospack find mk)/node.mk
