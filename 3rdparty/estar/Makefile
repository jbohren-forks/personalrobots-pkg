# If all goes according to plan, all you have to touch for an E*
# update is the VERSION. This also allows you to build a specific
# version without touching the Makefile, for example by typing:
#
#  $ make VERSION=0.2
#
# Or, to replace an existing build, do e.g.:
#
#  $ make VERSION=0.2 rebuild

all: really-all

VERSION=     r267

TARBALL=     estar-$(VERSION).tar.gz
TARBALL_URL= http://downloads.sourceforge.net/estar/$(TARBALL)
SOURCE_DIR=  $(PWD)/estar-$(VERSION)
UNPACK_CMD=  tar xfz
BUILD_DIR=   $(PWD)/build
INST_DIR=    $(PWD)/local

include $(shell rospack find mk)/download_unpack.mk

.PHONY: reall-all
really-all: $(SOURCE_DIR) $(SOURCE_DIR)/configure $(BUILD_DIR)/config.h
	$(MAKE) -C $(BUILD_DIR) install

rebuild: wipe really-all

$(SOURCE_DIR)/configure: $(SOURCE_DIR)/configure.ac $(SOURCE_DIR)/bootstrap-buildsystem.sh
	cd $(SOURCE_DIR) && ./bootstrap-buildsystem.sh

$(BUILD_DIR)/config.h: $(SOURCE_DIR)/configure $(SOURCE_DIR)/config.h.in
	test -d $(BUILD_DIR) || mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && $(SOURCE_DIR)/configure --prefix=$(INST_DIR)

clean:
	$(MAKE) -C $(BUILD_DIR) clean

distclean:
	$(MAKE) -C $(BUILD_DIR) distclean

wipe:
	rm -rf $(BUILD_DIR) $(INST_DIR)
