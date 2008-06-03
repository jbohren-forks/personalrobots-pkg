SRC = rosgazebo.cc
OUT = rosgazebo
PKG = rosgazebo
CXX = g++
all: $(PKG)

CFLAGS = -g -Wall $(shell rospack export/cpp/cflags $(PKG)) $(shell pkg-config --cflags libpr2API) $(shell pkg-config --cflags libgazebo) $(shell pkg-config --cflags libgazeboServer)
LFLAGS = $(shell rospack export/cpp/lflags $(PKG)) $(shell pkg-config --libs libpr2API) $(shell pkg-config --libs libgazebo) $(shell pkg-config --libs libgazeboServer)

rosgazebo: rosgazebo.cc
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

clean:
	rm -rf *.o $(PKG) $(PKG).dSYM
