PKG  = libpr2API
CXX = g++

CFLAGS = -ggdb -g -Wall $(shell rospack export/cpp/cflags $(PKG))

CDFLAGS = $(shell rospack export/cpp/cflags gazebo) \
          $(shell rospack export/cpp/cflags pr2Core) \
          $(shell rospack export/cpp/cflags libKinematics) \
          $(shell rospack export/cpp/cflags newmat10)

LDFLAGS = $(shell rospack export/cpp/lflags gazebo) \
          $(shell rospack export/cpp/lflags pr2Core) \
          $(shell rospack export/cpp/lflags libKinematics) \
          $(shell rospack export/cpp/lflags newmat10)

LFLAGS = $(shell rospack export/cpp/lflags $(PKG))

LIBP = lib/libpr2API.a
LIBS = lib/libpr2API.so

all: $(LIBP) test_libpr2API

pr2API.o: src/pr2API.cpp
	$(CXX) $(CFLAGS) -c -o $@ $?

$(LIBP): pr2API.o
	ar -rv $@ $^

#src/pr2API.cpp
$(LIBS): pr2API.o
	$(CXX) $(CDFLAGS) -shared -o $@ $^ $(LDFLAGS)

test_libpr2API: src/test/test_pr2API.cpp
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

clean:
	rm -f test_pr2API $(LIBP) $(LIBS)
	rm -rf *.o $(PKG) $(PKG).dSYM

