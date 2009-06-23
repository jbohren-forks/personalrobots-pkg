all: installed

SVN_DIR = libdc1394v2-svn
SVN_URL = https://libdc1394.svn.sourceforge.net/svnroot/libdc1394/trunk/libdc1394
SVN_REVISION = -r 580
SVN_PATCH = videre_fix.patch
include $(shell rospack find mk)/svn_checkout.mk

installed: wiped $(SVN_DIR) patched
	cd $(SVN_DIR) && autoreconf -i -s
	cd $(SVN_DIR) && CFLAGS="-g -O2 -fPIC" ./configure --prefix=$(PWD)/libdc1394v2 --enable-shared=no
	cd $(SVN_DIR) && ln -fs /usr/bin/libtool
	cd $(SVN_DIR) && make
	cd $(SVN_DIR) && make install
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean: 
	rm -rf libdc1394v2 installed wiped

wipe: clean
	rm -rf $(SVN_DIR)

.PHONY : clean wipe
