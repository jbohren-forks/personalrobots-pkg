PKG  = libpr2HW
CXX = g++

CFLAGS = -g -Wall $(shell rospack export/cpp/cflags $(PKG))

LFLAGS = $(shell rospack export/cpp/lflags $(PKG))

LDFLAGS = $(shell rospack export/cpp/lflags gazebo) \
          $(shell rospack export/cpp/lflags pr2Core) \
          $(shell rospack export/cpp/lflags libKinematics) \
          $(shell rospack export/cpp/lflags newmat10) \
          $(shell rospack export/cpp/lflags genericControllers)

LIBS = lib/libpr2HW.so

all: $(LIBS) bin/test_libpr2HW 

pr2HW.o: src/pr2HW.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<

$(LIBS): pr2HW.o
	$(CXX) -shared -o $@ $^ $(LDFLAGS)

test_pr2HW.o: src/test/test_pr2HW.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<

bin/test_libpr2HW: test_pr2HW.o
	$(CXX) -o $@ $< $(LFLAGS)

clean:
	-rm -f test_libpr2HW $(LIBS) bin/*
	-rm -rf *.o $(PKG) $(PKG).dSYM

