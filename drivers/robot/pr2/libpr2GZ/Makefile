PKG  = libpr2GZ
CXX = g++

CFLAGS = -g -Wall $(shell rospack export/cpp/cflags $(PKG))

LFLAGS = $(shell rospack export/cpp/lflags $(PKG))

LDFLAGS = $(shell rospack export/cpp/lflags gazebo) \
          $(shell rospack export/cpp/lflags pr2Core) \
          $(shell rospack export/cpp/lflags libKinematics) \
          $(shell rospack export/cpp/lflags newmat10) \
          $(shell rospack export/cpp/lflags genericControllers)

LIBS = lib/libpr2GZ.so

all: $(LIBS) bin/test_libpr2GZ 

pr2GZ.o: src/pr2GZ.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<

$(LIBS): pr2GZ.o
	$(CXX) -shared -o $@ $^ $(LDFLAGS)

test_pr2GZ.o: src/test/test_pr2GZ.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<

bin/test_libpr2GZ: test_pr2GZ.o
	$(CXX) -o $@ $< $(LFLAGS)

clean:
	-rm -f test_libpr2GZ $(LIBS) bin/*
	-rm -rf *.o $(PKG) $(PKG).dSYM

