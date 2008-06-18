PKG  = libpr2HW
CXX = g++

CFLAGS = -g -Wall $(shell rospack export/cpp/cflags $(PKG))

CDFLAGS = $(shell rospack export/cpp/cflags gazebo) \
          $(shell rospack export/cpp/cflags pr2Core) \
          $(shell rospack export/cpp/cflags libKinematics) \
          $(shell rospack export/cpp/cflags newmat10)

LDFLAGS = $(shell rospack export/cpp/lflags gazebo) \
          $(shell rospack export/cpp/lflags pr2Core) \
          $(shell rospack export/cpp/lflags libKinematics) \
          $(shell rospack export/cpp/lflags newmat10)

LFLAGS = $(shell rospack export/cpp/lflags $(PKG))

LIBP = lib/libpr2HW.a
LIBS = lib/libpr2HW.so

all: $(LIBP) bin/test_libpr2HW 

pr2HW.o: src/pr2HW.cpp include/libpr2HW/pr2HW.h
	$(CXX) $(CFLAGS) -c -o $@ $<

$(LIBP): pr2HW.o
	ar -rv $@ $^

$(LIBS): pr2HW.o
	$(CXX) -shared -o $@ $^ $(LDFLAGS)

bin/test_libpr2HW: src/test/test_pr2HW.cpp
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

clean:
	-rm -f test_libpr2HW $(LIBP) $(LIBS) bin/*
	-rm -rf *.o $(PKG) $(PKG).dSYM

