all: libdc1394

libdc1394-1.2.2:
	tar xzf libdc1394-1.2.2.tar.gz

libdc1394: libdc1394-1.2.2
	@if [ ! -f /usr/lib/libraw1394.a ]; then echo "you don't appear to have the libraw1394 driver installed. On Ubuntu, this is fixed with 'sudo apt-get install libraw1394-dev'"; false; fi
	@if [ ! -f /usr/include/X11/extensions/Xvlib.h ]; then echo "you don't appear to have the X11 Xv extensions installed. On Ubuntu, this is fixed with 'sudo apt-get install libxv-dev'"; false; fi
	cd libdc1394-1.2.2 && ./configure --prefix=$(PWD)/libdc1394
	cd libdc1394-1.2.2 && make && make install

clean:
	-rm -rf libdc1394 libdc1394-1.2.2

.PHONY : clean
