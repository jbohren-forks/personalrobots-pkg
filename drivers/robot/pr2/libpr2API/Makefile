PKG = libpr2API
CXX = g++

CFLAGS = -g -Wall $(shell rospack export/cpp/cflags $(PKG))

LFLAGS = $(shell rospack export/cpp/lflags $(PKG))

LIB = lib/libpr2API.a

all: $(LIB) test_libpr2API

pr2API.o: src/pr2API.cpp
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

$(LIB): pr2API.o
	ar -rs $@ $^

test_libpr2API: src/test/test_pr2API.cpp
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

clean:
	rm -f test_pr2API $(LIB)
	rm -rf *.o $(PKG) $(PKG).dSYM

