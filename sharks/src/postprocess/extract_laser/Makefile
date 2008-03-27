SRC = extract_laser.cpp
OUT = extract_laser
PKG = stair__sharks
CFLAGS = -I$(shell $(ROS_ROOT)/rospack find stair__sdl)/include -I$(shell $(ROS_ROOT)/rospack find stair__sdl_image)/include
LFLAGS = -L$(shell $(ROS_ROOT)/rospack find stair__sdl)/lib -L$(shell $(ROS_ROOT)/rospack find stair__sdl_image)/lib -lSDL -lSDL_image
include $(shell $(ROS_ROOT)/rospack find roscpp)/make_include/only_rules.mk
