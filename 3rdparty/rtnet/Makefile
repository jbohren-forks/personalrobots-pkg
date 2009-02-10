all: installed

RTNET_VERSION = 0.9.10
TARBALL = rtnet-$(RTNET_VERSION).tar.bz2

TARBALL_URL = http://pr.willowgarage.com/downloads/$(TARBALL)
SOURCE_DIR = build/rtnet-0.9.10
UNPACK_CMD = tar xjf
TARBALL_PATCH = rtnet-$(RTNET_VERSION).patch
include $(shell rospack find mk)/download_unpack_build.mk

installed: wiped $(SOURCE_DIR)/unpacked Makefile
	cd $(SOURCE_DIR) && \
	cp ../../rtnet_config .rtnet_config && \
    make oldconfig && \
	make install
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	-make -C $(SOURCE_DIR) clean
	
wipe:
	rm -rf rtnet build $(TARBALL)

