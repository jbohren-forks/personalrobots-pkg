PKG  = libpr2API
CXX = g++

CFLAGS = -g -Wall $(shell rospack export/cpp/cflags $(PKG))

LDFLAGS = $(shell rospack export/cpp/lflags gazebo) \
          $(shell rospack export/cpp/lflags pr2Core) \
          $(shell rospack export/cpp/lflags libKinematics)

LFLAGS = $(shell rospack export/cpp/lflags $(PKG))

LIBP = lib/libpr2API.a
LIBS = lib/libpr2API.so

all: $(LIBP) $(LIBS) test_libpr2API

pr2API.o: src/pr2API.cpp
	$(CXX) $(CFLAGS) -o $@ $< $(LDFLAGS)

$(LIBP): pr2API.o
	ar -rv $@ $^

$(LIBS): src/pr2API.cpp
	$(CXX) $(CFLAGS) -shared -o $@ $^ $(LDFLAGS)

test_pr2API.o: src/test/test_pr2API.cpp
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

test_libpr2API: src/test/test_pr2API.cpp
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

clean:
	rm -f test_pr2API $(LIBP) $(LIBS)
	rm -rf *.o $(PKG) $(PKG).dSYM

