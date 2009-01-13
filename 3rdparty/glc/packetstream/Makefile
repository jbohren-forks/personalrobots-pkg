all: installed

TARBALL = build/packetstream-0.1.4.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/packetstream-0.1.4.tar.gz
SOURCE_DIR = packetstream-0.1.4
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR) Makefile
	-mv build/packetstream packetstream-0.1.4
	cd $(SOURCE_DIR) && cmake . -DCMAKE_INSTALL_PREFIX:PATH="`rospack find glc`" -DCMAKE_BUILD_TYPE:STRING="Release" -DCMAKE_C_FLAGS_RELEASE:STRING="-O2 -msse -mmmx -fomit-frame-pointer" -DCMAKE_INSTALL_RPATH:STRING="`rospack find glc`/lib" && make && make install
	touch installed

clean:
	rm -rf $(SOURCE_DIR) installed
wipe: clean
	rm -rf $(TARBALL) build
