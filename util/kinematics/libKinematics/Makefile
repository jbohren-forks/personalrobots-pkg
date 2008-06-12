PKG = libKinematics

CFLAGS = -g -Wall -Iinclude $(shell rospack export/cpp/cflags $(PKG))

LDFLAGS = $(shell rospack export/cpp/lflags newmat10)

LFLAGS  = $(shell rospack export/cpp/lflags $(PKG))

LIBK  = lib/libKinematics.a
LIBIK = lib/libIk.a

all: $(LIBK) $(LIBIK) test_PR2ik


ik.o: src/ik.cpp
	g++  $(CFLAGS) -o $@ -c $< 

kinematics.o: src/kinematics.cpp
	g++  $(CFLAGS) -o $@ -c $<

$(LIBK): kinematics.o
	ar -rs $@ $^

$(LIBIK): ik.o kinematics.o
	ar -rs $@ $^

test_PR2ik: src/test/test_PR2ik.cpp
	g++ $(CFLAGS) -o $@ $^ $(LFLAGS)

clean:
	rm -rf test_PR2ik
	rm -f *.o *.a lib/*.a

