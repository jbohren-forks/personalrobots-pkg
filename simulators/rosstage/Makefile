PKG = rosstage
CXX = g++
all: $(PKG)

CFLAGS = -g -Wall $(shell rospack export/cpp/cflags $(PKG))
LFLAGS = $(shell rospack export/cpp/lflags $(PKG))

rosstage: rosstage.cc
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

clean:
	rm -rf *.o $(PKG) $(PKG).dSYM
