SRC = rosgazebo.cc
OUT = rosgazebo
PKG = rosgazebo
CXX = g++
all: $(PKG)


CFLAGS = -g -Wall $(shell rospack export/cpp/cflags $(PKG))

LFLAGS = $(shell rospack export/cpp/lflags $(PKG))

rosgazebo.o: rosgazebo.cc
	$(CXX) $(CFLAGS) -c -o $@ $<

rosgazebo: rosgazebo.o
	$(CXX) -o $@ $< $(LFLAGS)

clean:
	rm -rf *.o $(PKG) $(PKG).dSYM
