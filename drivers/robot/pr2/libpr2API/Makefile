PKG = libpr2API
CXX = g++

CFLAGS = -g -Wall $(shell rospack export/cpp/cflags $(PKG))

LFLAGS = $(shell rospack export/cpp/lflags $(PKG))

LIBP = lib/libpr2API.a

all: $(LIBP) #test_libpr2API

pr2API.o: src/pr2API.cpp
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

$(LIBP): pr2API.o
	ar -rsv $@ $^

test_pr2API.o: src/test/test_pr2API.cpp
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

test_libpr2API: src/test/test_pr2API.cpp  test_pr2API.o
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

clean:
	rm -f test_pr2API $(LIBP)
	rm -rf *.o $(PKG) $(PKG).dSYM

