all: build

TARBALL = gmapping_r39.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/gmapping_r39.tar.gz
SOURCE_DIR = gmapping_export
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack.mk

PATCH = gmapping-r39.patch

build: wiped gmapping

configured: $(SOURCE_DIR) Makefile
	cd $(SOURCE_DIR) && patch -p0 < ../$(PATCH) && ./configure
	touch configured

wiped: Makefile $(PATCH)
	make wipe
	touch wiped

.PHONY: gmapping
gmapping: configured
	cd $(SOURCE_DIR) && make

clean:
	-cd $(SOURCE_DIR) && make clean
	rm -rf gmapping configured
wipe: clean
	rm -rf $(SOURCE_DIR)

