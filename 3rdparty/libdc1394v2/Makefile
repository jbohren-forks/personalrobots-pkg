all: build

SVN_DIR = libdc1394v2-svn
SVN_URL = https://libdc1394.svn.sourceforge.net/svnroot/libdc1394/trunk/libdc1394
SVN_REVISION = -r 565
SVN_PATCH = conversions_fix.patch videre_fix.patch configure.in.patch
include $(shell rospack find mk)/svn_checkout.mk

build: SVN_UP_REVERT_PATCH libdc1394v2

configured: $(SVN_DIR) Makefile
	cd $(SVN_DIR) && autoreconf -i -s
	cd $(SVN_DIR) && CFLAGS="-g -O2 -fPIC" ./configure --prefix=$(PWD)/libdc1394v2 --enable-shared=no
	touch configured

.PHONY: libdc1394v2
libdc1394v2: configured
	cd $(SVN_DIR) && ln -fs /usr/bin/libtool
	cd $(SVN_DIR) && make
	cd $(SVN_DIR) && make install
	touch libdc1394v2

clean: 
	rm -rf libdc1394v2 configured

wipe: clean
	rm -rf $(SVN_DIR)

.PHONY : clean wipe
