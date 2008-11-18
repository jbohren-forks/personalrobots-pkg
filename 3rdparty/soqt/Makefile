all: soqt

TARBALL = build/SoQt-1.4.1.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/SoQt-1.4.1.tar.gz
SOURCE_DIR = build/SoQt-1.4.1
MD5SUM_FILE = SoQt-1.4.1.tar.gz.md5sum
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

PATCH=soqt_qt4.patch

configured: $(SOURCE_DIR)
	cd $(SOURCE_DIR) && patch -s -N -p1 < ../../$(PATCH) || echo
	cd $(SOURCE_DIR) && autoconf && ./configure --prefix=$(PWD)
	touch configured

soqt: configured
	cd $(SOURCE_DIR) && make && make install
	touch configured

clean:
	rm -rf $(SOURCE_DIR)
	rm -rf configured

wipe: clean
	rm -rf $(TARBALL)
