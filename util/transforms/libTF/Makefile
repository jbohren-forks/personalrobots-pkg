PKG = libTF

CFLAGS = -g -Wall -Iinclude `rospack export/cpp/cflags $(PKG)`

LDFLAGS = `rospack export/cpp/lflags $(PKG)`

LIB = lib/libTF.a

all: $(LIB) test


Quaternion3D.o: src/simple/Quaternion3D.cpp include/libTF/Quaternion3D.h
	g++ $(CFLAGS) -o $@ -c $< 

libTF.o: src/simple/libTF.cpp include/libTF/libTF.h
	g++  $(CFLAGS) -o $@ -c $< 

$(LIB): libTF.o Quaternion3D.o
	ar -rs $@ $^

test: src/test/main.cpp $(LIB)
	g++ $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -rf test testtf
	rm -f *.o *.a lib/*.a

testtf: testtf.cc $(LIB)
	g++ $(CFLAGS) -o $@ $^ $(LDFLAGS)