all: elfhacks

TARBALL = elfhacks-0.4.1.tar.gz
TARBALL_URL = http://nullkey.ath.cx/elfhacks/archive/elfhacks-0.4.1.tar.gz
SOURCE_DIR = elfhacks-0.4.1
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack.mk

elfhacks: $(SOURCE_DIR)
	-mv elfhacks elfhacks-0.4.1
	cd $(SOURCE_DIR) && cmake . -DCMAKE_INSTALL_PREFIX:PATH="`rospack find glc`" -DCMAKE_BUILD_TYPE:STRING="Release" -DCMAKE_C_FLAGS_RELEASE:STRING="-O2 -msse -mmmx -fomit-frame-pointer" -DCMAKE_INSTALL_RPATH:STRING="`rospack find glc`/lib" && make && make install

clean:
	rm -rf $(SOURCE_DIR)
wipe: clean
	rm -rf $(TARBALL)
