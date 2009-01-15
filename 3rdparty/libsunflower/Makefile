# If all goes according to plan, all you have to touch for a
# libsunflower update is the VERSION. This also allows you to build a
# specific version without touching the Makefile, for example by
# typing:
#
#  $ make VERSION=1.1
#
# Or, to replace an existing build, do e.g.:
#
#  $ make VERSION=1.1 rebuild
#
# It is also possible to use svn checkout to get the sources, in case
# you need the bleeding edge. This is done using:
#
#  $ make svn-all

##all: tarball-all
#all: svn-all
all: installed

TARBALL_VERSION=   r910
SVN_REVISION=    -r 963

TARBALL=     build/libsunflower-$(TARBALL_VERSION).tar.gz
TARBALL_URL= http://pr.willowgarage.com/downloads/libsunflower-$(TARBALL_VERSION).tar.gz
SOURCE_DIR=  $(PWD)/build/libsunflower-$(TARBALL_VERSION)
UNPACK_CMD=  tar xfz
BUILD_DIR=   $(PWD)/build
INST_DIR=    $(PWD)/local
ESTAR_DIR=   `rospack find estar`/local
BOOST_DIR=   `rospack cflags-only-I boost`

SVN_DIR=     $(PWD)/build/sunflower-svn
SVN_URL=     https://libsunflower.svn.sourceforge.net/svnroot/libsunflower/trunk/sunflower

#include $(shell rospack find mk)/download_unpack_build.mk
include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) patched
	cd $(SVN_DIR) && ./bootstrap-buildsystem.sh
	test -d $(BUILD_DIR) || mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && $(SVN_DIR)/configure --with-pic \
	                                        --prefix=$(INST_DIR) \
                                                --with-estar=$(ESTAR_DIR) \
                                                --with-boost=$(BOOST_DIR)
	$(MAKE) -C $(BUILD_DIR) install
	touch installed

.PHONY: really-all
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

$(SOURCE_DIR)/configure $(SOURCE_DIR)/config.h.in: $(SOURCE_DIR)/configure.ac $(SOURCE_DIR)/bootstrap-buildsystem.sh
	cd $(SOURCE_DIR) && ./bootstrap-buildsystem.sh

$(BUILD_DIR)/config.h: $(SOURCE_DIR)/configure $(SOURCE_DIR)/config.h.in
	test -d $(BUILD_DIR) || mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && \
	$(SOURCE_DIR)/configure --with-pic --prefix=$(INST_DIR) \
                                                   --with-estar=$(ESTAR_DIR) \
                                                   --with-boost=$(BOOST_DIR)

clean:
	-$(MAKE) -C $(BUILD_DIR) clean
	rm -f installed

distclean:
	$(MAKE) -C $(BUILD_DIR) distclean

wipe: clean
	rm -rf build $(INST_DIR)
