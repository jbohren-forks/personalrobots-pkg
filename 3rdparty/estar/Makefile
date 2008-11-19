# If all goes according to plan, all you have to touch for an E*
# update is the VERSION. This also allows you to build a specific
# version without touching the Makefile, for example by typing:
#
#  $ make VERSION=0.2
#
# Or, to replace an existing build, do e.g.:
#
#  $ make VERSION=0.2 rebuild
#
# It is also possible to use svn checkout to get the sources, in case
# you need the bleeding edge. This is done using:
#
#  $ make svn-all

##all: tarball-all
all: svn-all

VERSION=     r267

TARBALL=     build/estar-$(VERSION).tar.gz
TARBALL_URL= http://pr.willowgarage.com/downloads/estar-$(VERSION).tar.gz
SOURCE_DIR=  $(PWD)/build/estar-$(VERSION)
UNPACK_CMD=  tar xfz
BUILD_DIR=   $(PWD)/build/build
INST_DIR=    $(PWD)/local
BOOST_DIR=   `rospack cflags-only-I boost`

SVN_DIR=     $(PWD)/build/estar-svn
SVN_REV=     HEAD
SVN_URL=     https://estar.svn.sourceforge.net/svnroot/estar/trunk/estar

include $(shell rospack find mk)/download_unpack_build.mk

.PHONY: reall-all
really-all: $(SOURCE_DIR)/configure $(BUILD_DIR)/config.h
	$(MAKE) -C $(BUILD_DIR) install

.PHONY: svn-all
svn-all: checkout_or_update
	$(MAKE) SOURCE_DIR=$(SVN_DIR) really-all

checkout_or_update:
	- test -d $(SVN_DIR) && svn up $(SVN_DIR)
	test -d $(SVN_DIR) || svn co -r$(SVN_REV) $(SVN_URL) $(SVN_DIR)

.PHONY: tarball-all
tarball-all: $(SOURCE_DIR)
	$(MAKE) really-all

rebuild: wipe really-all

$(SOURCE_DIR)/configure: $(SOURCE_DIR)/configure.ac $(SOURCE_DIR)/bootstrap-buildsystem.sh
	cd $(SOURCE_DIR) && ./bootstrap-buildsystem.sh

$(BUILD_DIR)/config.h: $(SOURCE_DIR)/configure $(SOURCE_DIR)/config.h.in
	test -d $(BUILD_DIR) || mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && $(SOURCE_DIR)/configure --prefix=$(INST_DIR) \
                                                   --with-boost=$(BOOST_DIR)

clean:
	$(MAKE) -C $(BUILD_DIR) clean

distclean:
	$(MAKE) -C $(BUILD_DIR) distclean

wipe:
	rm -rf $(BUILD_DIR) $(INST_DIR)
