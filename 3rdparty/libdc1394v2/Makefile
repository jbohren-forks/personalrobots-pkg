all: libdc1394v2-build

SVN_DIR = libdc1394v2
SVN_URL = https://libdc1394.svn.sourceforge.net/svnroot/libdc1394/trunk/libdc1394
SVN_REVISION =
SVN_PATCH = conversions_fix.patch
include $(shell rospack find mk)/svn_checkout.mk

libdc1394v2-build: SVN_UP
	cd $(SVN_DIR) && autoreconf -i -s
	cd $(SVN_DIR) && ./configure --prefix=$(PWD) --enable-shared=no
	cd $(SVN_DIR) && make
	cd $(SVN_DIR) && make install

clean: 
	rm -rf include
	rm -rf lib
	rm -rf share
	rm -rf bin

wipe: clean
	rm -rf $(SVN_DIR)

.PHONY : clean wipe
