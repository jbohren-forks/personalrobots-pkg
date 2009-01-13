all: installed

TARBALL     = build/ann_1.1.1.tar.gz
TARBALL_URL = http://www.cs.umd.edu/~mount/ANN/Files/1.1.1/ann_1.1.1.tar.gz
UNPACK_CMD  = tar xfz
SOURCE_DIR  = build/ann_1.1.1
TARBALL_PATCH = gcc43_shared.patch

include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR) Makefile
	echo "Building FL-ANN..."
	cd build/ann_1.1.1 && make linux-shared-g++
	touch installed
wipe:
	rm -rf build installed
