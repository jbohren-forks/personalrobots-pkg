all: boost

TARBALL = build/boost_1_36_0.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/boost_1_36_0.tar.gz
SOURCE_DIR = build/boost_1_36_0
LOCAL_DIR = boost_1_36_0
UNPACK_CMD = tar zxf
MD5SUM_FILE = boost_1_36_0.tar.gz.md5sum
include $(shell rospack find mk)/download_unpack_build.mk

boost: $(SOURCE_DIR)
	rm -rf $(LOCAL_DIR)
	mv $(SOURCE_DIR) $(LOCAL_DIR)
clean:
	rm -rf boost_1_36_0

wipe: clean
	rm -rf build

.PHONY : clean distclean
