all: build

TARBALL = build/eml-r36-patched.tar.bz2
TARBALL_URL = http://pr.willowgarage.com/downloads/eml-r36-patched.tar.bz2
SOURCE_DIR = build/eml-svn
UNPACK_CMD = tar xjf
include $(shell rospack find mk)/download_unpack_build.mk

build: wiped eml

configured: $(SOURCE_DIR) Makefile
	cd $(SOURCE_DIR) && \
	mkdir -p build && \
	cd build && \
	cmake -D RTNET_INSTALL_PATH=`rospack find rtnet`/rtnet -D XENOMAI_INSTALL_PATH=`rospack find xenomai`/xenomai -D CMAKE_INSTALL_PREFIX=`rospack find eml`/eml .. && \
	touch configured

wiped: Makefile
	make wipe
	touch wiped

.PHONY: eml
eml: configured
	make -C $(SOURCE_DIR)/build install

clean:
	-make -C $(SOURCE_DIR)/build clean
	rm -rf eml configured

wipe: clean
	rm -rf build



