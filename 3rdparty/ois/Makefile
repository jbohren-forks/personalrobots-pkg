all: ois

#REVISION = -r 6641

ois-checkout:
	@if [ ! -d ois-cvs ]; then cvs -z3 -d:pserver:anonymous@wgois.cvs.sourceforge.net:/cvsroot/wgois co -P ois ; mv ois ois-cvs; fi
	-cd ois-cvs && cvs up $(REVISION)

CONFIGURE_FLAGS = --with-arch=nocona --enable-release CXXFLAGS='-O3   -I$(ROOT)/include' LDFLAGS='-L$(ROOT)/lib' CFLAGS='-O3   -I$(ROOT)/include -I$(ROOT)/usr/include'

ROOT = $(shell rospack find ois)/ois

ois: ois-checkout
	cd ois-cvs && ./bootstrap
	cd ois-cvs && ./configure --prefix=$(ROOT) --with-trimesh=opcode $(CONFIGURE_FLAGS)
	cd ois-cvs && make
	cd ois-cvs && make install
	touch ois

clean:
	cd ois-cvs && make clean
	rm -rf ois
wipe:
	rm -rf ois-cvs ois

download: ois-checkout

