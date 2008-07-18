all: boost

TARBALL = boost_1_35_0.tar.gz
TARBALL_URL = http://voxel.dl.sourceforge.net/sourceforge/boost/boost_1_35_0.tar.gz
SOURCE_DIR = boost_1_35_0
UNPACK_CMD = tar zxf
include $(shell rospack find mk)/download_unpack.mk

download: boost_1_35_0.tar.gz

boost: $(SOURCE_DIR)

clean:
	-rm -rf boost_1_35_0

distclean: clean
	-rm -rf boost_1_35_0.tar.gz

.PHONY : clean distclean
