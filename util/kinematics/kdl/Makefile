PKG = kdl

CFLAGS = -g -Wall -Iinclude $(shell rospack export/cpp/cflags $(PKG))

LFLAGS  = $(shell rospack export/cpp/lflags $(PKG))

all: pr2

pr2: pr2.cpp
	g++ $(CFLAGS) -o $@ $^ $(LFLAGS)

clean:
	-rm -f pr2

