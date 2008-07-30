# If all goes according to plan, all you have to touch for a
# libsunflower update is the VERSION. This also allows you to build a
# specific version without touching the Makefile, for example by
# typing:
#
#  $ make VERSION=0.2
#
# Or, to replace an existing build, do e.g.:
#
#  $ make VERSION=0.2 rebuild

all: really-all

VERSION=     r900

TARBALL=     libsunflower-$(VERSION).tar.gz
TARBALL_URL= http://downloads.sourceforge.net/libsunflower/$(TARBALL)
SOURCE_DIR=  $(PWD)/libsunflower-$(VERSION)
UNPACK_CMD=  tar xfz
BUILD_DIR=   $(PWD)/build
INST_DIR=    $(PWD)/local
ESTAR_DIR=   `rospack find estar`/local

include $(shell rospack find mk)/download_unpack.mk

.PHONY: reall-all
really-all: $(SOURCE_DIR) $(SOURCE_DIR)/configure $(BUILD_DIR)/config.h
	$(MAKE) -C $(BUILD_DIR) install

rebuild: wipe really-all

$(SOURCE_DIR)/configure: $(SOURCE_DIR)/configure.ac $(SOURCE_DIR)/bootstrap-buildsystem.sh
	cd $(SOURCE_DIR) && ./bootstrap-buildsystem.sh

$(BUILD_DIR)/config.h: $(SOURCE_DIR)/configure $(SOURCE_DIR)/config.h.in
	test -d $(BUILD_DIR) || mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && $(SOURCE_DIR)/configure --prefix=$(INST_DIR) --with-estar=$(ESTAR_DIR)

clean:
	$(MAKE) -C $(BUILD_DIR) clean

distclean:
	$(MAKE) -C $(BUILD_DIR) distclean

wipe:
	rm -rf $(BUILD_DIR) $(INST_DIR)
