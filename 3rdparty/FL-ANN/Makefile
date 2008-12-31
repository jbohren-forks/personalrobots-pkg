all: fl-ann

TARBALL     = build/flann-1.11-src.zip
TARBALL_URL = http://pr.willowgarage.com/downloads/flann-1.11-src.zip
UNPACK_CMD  = unzip -x
SOURCE_DIR  = build/flann-1.11-src
TARBALL_PATCH = cmakelists.patch

include $(shell rospack find mk)/download_unpack_build.mk

fl-ann: $(SOURCE_DIR)
	echo "Building FL-ANN..."
	mkdir -p build/build && cd build/build && cmake ../flann-1.11-src/src && make

wipe:
	rm -rf build
