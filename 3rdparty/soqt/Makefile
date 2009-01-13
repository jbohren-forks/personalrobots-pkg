all: installed

TARBALL = build/SoQt-1.4.1.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/SoQt-1.4.1.tar.gz
SOURCE_DIR = build/SoQt-1.4.1
MD5SUM_FILE = SoQt-1.4.1.tar.gz.md5sum
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

PATCH=soqt_qt4.patch

installed: wiped $(SOURCE_DIR)
	cd $(SOURCE_DIR) && patch -s -N -p1 < ../../$(PATCH) || echo
	cd $(SOURCE_DIR) && autoconf && ./configure --prefix=$(PWD)
	cd $(SOURCE_DIR) && make && make install
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	-make -C $(SOURCE_DIR) clean
	rm -rf $(SOURCE_DIR)
	rm -rf installed

wipe: clean
	rm -rf build
