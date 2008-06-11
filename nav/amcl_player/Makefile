SRC = amcl_player.cc
OUT = amcl_player
PKG = amcl_player
CFLAGS = -Wall -Werror `pkg-config --cflags gdk-pixbuf-2.0`
LFLAGS = `pkg-config --libs gdk-pixbuf-2.0` -lplayerdrivers
include $(shell rospack find mk)/node.mk
