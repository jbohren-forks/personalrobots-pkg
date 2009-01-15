all: installed

TARBALL = build/glc-0.5.7.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/glc-0.5.7.tar.gz
SOURCE_DIR = build/glc
SOURCE_DIR2 = glc-0.5.7
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked Makefile
	-mv $(SOURCE_DIR) $(SOURCE_DIR2)
	cd $(SOURCE_DIR2) && cmake . -DCMAKE_INSTALL_PREFIX:PATH="`rospack find glc`" -DCMAKE_BUILD_TYPE:STRING="Release" -DCMAKE_C_FLAGS_RELEASE:STRING="-O2 -msse -mmmx -fomit-frame-pointer" -DCMAKE_INSTALL_RPATH:STRING="`rospack find glc`/lib" -DELFHACKS_LIBRARY=$(PWD)/../lib/libelfhacks.so -DELFHACKS_INCLUDE_DIR=$(PWD)/../include -DPACKETSTREAM_LIBRARY=$(PWD)/../lib/libpacketstream.so -DPACKETSTREAM_INCLUDE_DIR=$(PWD)/../include && make && make install
	touch installed

clean:
	rm -rf $(SOURCE_DIR) $(SOURCE_DIR2) installed
wipe: clean
	rm -rf $(TARBALL) build
