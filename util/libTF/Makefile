
CFLAGS = -g -Wall -Iinclude

LDFLAGS = -lnewmat

all:test


Quaternion3D.o: src/simple/Quaternion3D.cpp include/libTF/Quaternion3D.h
	g++ $(CFLAGS) -o $@ -c $< 

libTF.o: src/simple/libTF.cpp include/libTF/libTF.h
	g++  $(CFLAGS) -o $@ -c $< 

test: src/test/main.cpp libTF.o Quaternion3D.o
	g++ $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f *.o test