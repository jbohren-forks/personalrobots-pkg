all: opende

#REVISION = -r 6641

opende-checkout:
	@if [ ! -d opende-svn ]; then svn co $(REVISION) https://opende.svn.sourceforge.net/svnroot/opende/trunk opende-svn; fi
	-cd opende-svn && svn up $(REVISION)

CONFIGURE_FLAGS = --with-arch=nocona --enable-release CXXFLAGS='-O3   -I$(ROOT)/include' LDFLAGS='-L$(ROOT)/lib' CFLAGS='-O3   -I$(ROOT)/include -I$(ROOT)/usr/include'

ROOT = $(shell rospack find opende)/opende

opende: opende-checkout
	cd opende-svn && sh ./autogen.sh
	cd opende-svn && ./configure --prefix=$(ROOT) --with-trimesh=opcode $(CONFIGURE_FLAGS)
	cd opende-svn && make
	cd opende-svn && make install
	touch opende

clean:
	cd opende-svn && make clean
	rm -rf opende
wipe:
	rm -rf opende-svn opende

download: opende-checkout

