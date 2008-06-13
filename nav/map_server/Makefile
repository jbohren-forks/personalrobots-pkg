#SRC = map_server.cc
#OUT = map_server
#PKG = map_server
#CFLAGS = -Wall -Werror `pkg-config --cflags gdk-pixbuf-2.0`
#LFLAGS = `pkg-config --libs gdk-pixbuf-2.0`
#include $(shell rospack find mk)/node.mk

all: map_server

map_server:
	mkdir -p build
	cd build && cmake .. && make

clean:
	rm -rf build map_server
