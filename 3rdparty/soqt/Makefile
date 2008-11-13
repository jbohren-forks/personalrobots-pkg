all: soqt

TARBALL = SoQt-1.4.1.tar.gz
TARBALL_URL = http://ftp.coin3d.org/coin/src/all/SoQt-1.4.1.tar.gz
SOURCE_DIR = SoQt-1.4.1
#MD5SUM_FILE = opencv-1.0.0.tar.gz.md5sum
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack.mk

configured: $(SOURCE_DIR)
	cd $(SOURCE_DIR) && ./configure --prefix=$(PWD)
	touch configured

soqt: configured
	cd $(SOURCE_DIR) && make && make install
	touch configured

clean:
	rm -rf $(SOURCE_DIR)
	rm -rf configured

wipe: clean
	rm -rf $(TARBALL)
