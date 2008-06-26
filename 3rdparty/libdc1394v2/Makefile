all: libdc1394v2

libdc1394-2.0.2.tar.gz:
	wget http://downloads.sourceforge.net/libdc1394/libdc1394-2.0.2.tar.gz

download: libdc1394-2.0.2.tar.gz

libdc1394v2: libdc1394-2.0.2.tar.gz
	tar xzf libdc1394-2.0.2.tar.gz
	cd libdc1394-2.0.2 && ./configure --prefix=`rospack find libdc1394v2` --enable-shared=no
	cd libdc1394-2.0.2 && make install

clean: 
	rm -rf libdc1394-2.0.2
	rm -rf include
	rm -rf lib
	rm -rf share
	rm -rf bin

wipe: clean
	rm -rf libdc1394-2.0.2.tar.gz

.PHONY : clean wipe
