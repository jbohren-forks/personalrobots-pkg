PKG = libKinematics

CFLAGS = -g -Wall -Iinclude `rospack export/cpp/cflags $(PKG)`

LDFLAGS = `rospack export/cpp/lflags $(PKG)`

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

test_PR2ik: src/test/test_PR2ik.cpp $(LIBK) $(LIBIK)
	g++ $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -rf test_PR2ik
	rm -f *.o *.a lib/*.a

