SRC = rosgazebo.cc
OUT = rosgazebo
PKG = rosgazebo
CXX = g++
all: $(PKG)


CFLAGS = -g -Wall $(shell rospack export/cpp/cflags $(PKG))

LFLAGS = $(shell rospack export/cpp/lflags $(PKG))

rosgazebo: rosgazebo.cc
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

clean:
	rm -rf *.o $(PKG) $(PKG).dSYM
