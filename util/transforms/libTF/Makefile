CFLAGS = -g -Wall -Iinclude

LDFLAGS = -lnewmat

all: install test


Quaternion3D.o: src/simple/Quaternion3D.cpp include/libTF/Quaternion3D.h
	g++ $(CFLAGS) -o $@ -c $< 

libTF.o: src/simple/libTF.cpp include/libTF/libTF.h
	g++  $(CFLAGS) -o $@ -c $< 

libTF.a: libTF.o Quaternion3D.o
	ar -rs $@ $^

test: src/test/main.cpp libTF.a
	g++ $(CFLAGS) -o $@ $^ $(LDFLAGS)

install: libTF.a 
	cp libTF.a lib/

clean:
	rm -f *.o test *.a lib/*.a



