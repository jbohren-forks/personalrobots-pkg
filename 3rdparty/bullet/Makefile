all: bullet

TARBALL = bullet-2.71-SDK.tgz
TARBALL_URL = http://bullet.googlecode.com/files/bullet-2.71-SDK.tgz
SOURCE_DIR = bullet-2.71
MD5SUM_FILE = bullet-2.71-SDK.tgz.md5sum
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack.mk

bullet: $(SOURCE_DIR) quaternion.patch
	patch -p0 < quaternion.patch
	cd $(SOURCE_DIR) && cmake .
	make -C $(SOURCE_DIR) LibLinearMath
	touch bullet
clean:
	rm -rf $(SOURCE_DIR)
wipe: clean
	rm -rf $(TARBALL)
