all: libdc1394v2-build

libdc1394v2:
	svn co https://libdc1394.svn.sourceforge.net/svnroot/libdc1394/trunk/libdc1394/ libdc1394v2
	cd libdc1394v2 && patch -p0 < ../conversions_fix.patch
	cd libdc1394v2 && autoreconf -i -s
	cd libdc1394v2 && ./configure --prefix=`rospack find libdc1394v2` --enable-shared=no

download: libdc1394v2

libdc1394v2-build: libdc1394v2
	cd libdc1394v2 && make
	cd libdc1394v2 && make install

clean: 
	rm -rf include
	rm -rf lib
	rm -rf share
	rm -rf bin

wipe: clean
	rm -rf libdc1394v2

.PHONY : clean wipe
