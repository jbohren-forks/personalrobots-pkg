#SRC = map_server.cc
#OUT = map_server
#PKG = map_server
#CFLAGS = -Wall -Werror `pkg-config --cflags gdk-pixbuf-2.0`
#LFLAGS = `pkg-config --libs gdk-pixbuf-2.0`
#include $(shell rospack find mk)/node.mk

all: build/Makefile
	cd build && make -s
build/Makefile:
	mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=Debug ..
clean:
	if [ -f build/Makefile ]; then cd build && make clean && cd ..; fi
	rm -rf build
