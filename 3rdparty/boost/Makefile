all: boost

TARBALL = boost_1_36_0.tar.gz
TARBALL_URL = http://voxel.dl.sourceforge.net/sourceforge/boost/boost_1_36_0.tar.gz
SOURCE_DIR = boost_1_36_0
UNPACK_CMD = tar zxf
MD5SUM_FILE = boost_1_36_0.tar.gz.md5sum
include $(shell rospack find mk)/download_unpack.mk

boost: $(SOURCE_DIR)

clean:
	rm -rf boost_1_36_0

wipe: clean
	rm -rf boost_1_36_0.tar.gz

.PHONY : clean distclean
