SRC = wavefront_player.cc
OUT = wavefront_player
PKG = wavefront_player
CFLAGS = -Wall -Werror `pkg-config --cflags gdk-pixbuf-2.0`
LFLAGS = `pkg-config --libs gdk-pixbuf-2.0` -lwavefront_standalone
include $(shell rospack find mk)/node.mk
