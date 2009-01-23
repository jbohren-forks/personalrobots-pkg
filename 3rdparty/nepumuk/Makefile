# Note that the version of 3rdparty/libsunflower needs to closely
# match the nepumuk build.
#
# If all goes according to plan, all you have to touch for a
# nepumuk update is the VERSION. This also allows you to build a
# specific version without touching the Makefile, for example by
# typing:
#
#  $ make VERSION=r900
#
# Or, to replace an existing build, do e.g.:
#
#  $ make VERSION=r900 rebuild
#
# It is also possible to use svn checkout to get the sources, in case
# you need the bleeding edge. This is done using:
#
#  $ make svn-all

##all: tarball-all
#all: svn-all
all: installed

TARBALL_VERSION=   r912
SVN_REVISION=    -r 964

TARBALL=     build/nepumuk-$(TARBALL_VERSION).tar.gz
TARBALL_URL= http://downloads.sourceforge.net/libsunflower/nepumuk-$(TARBALL_VERSION).tar.gz
SOURCE_DIR=  $(PWD)/build/nepumuk-$(TARBALL_VERSION)
UNPACK_CMD=  tar xfz
BUILD_DIR=   $(PWD)/build
INST_DIR=    $(PWD)/local
ESTAR_DIR=   `rospack find estar`/local
SFL_DIR=     `rospack find libsunflower`/local
BOOST_DIR=   `rosboost-cfg --include_dirs`

SVN_DIR=     $(PWD)/build/nepumuk-svn
SVN_URL=     https://libsunflower.svn.sourceforge.net/svnroot/libsunflower/trunk/nepumuk

SIMULINKS=   nepumuk rostest ros0.sh robots.ros0 layout.ros0 expodemo.sh robots.expo layout.expo


#include $(shell rospack find mk)/download_unpack_build.mk
include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) patched
	cd $(SVN_DIR) && ./bootstrap-buildsystem.sh
	test -d $(BUILD_DIR) || mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && $(SVN_DIR)/configure --prefix=$(INST_DIR) \
                                                --with-sfl=$(SFL_DIR) \
                                                --with-boost=$(BOOST_DIR)
	$(MAKE) -C $(BUILD_DIR) install
	for foo in $(SIMULINKS); do \
	  test -L $$foo || ln -s $(INST_DIR)/bin/$$foo || exit 42; done
	touch installed

.PHONY: svn-all
svn-all: checkout_or_update
	$(MAKE) SOURCE_DIR=$(SVN_DIR) really-all

.PHONY: tarball-all
tarball-all: $(SOURCE_DIR)
	$(MAKE) really-all

rebuild: wipe really-all

.PHONY: really-all
really-all: $(SOURCE_DIR)/configure $(BUILD_DIR)/config.h
	$(MAKE) -C $(BUILD_DIR) install
	$(MAKE) symlinks

checkout_or_update:
	- test -d $(SVN_DIR) && svn up $(SVN_DIR)
	test -d $(SVN_DIR) || svn co -r$(SVN_REV) $(SVN_URL) $(SVN_DIR)

symlinks:
	for foo in $(SIMULINKS); do \
	  test -L $$foo || ln -s $(INST_DIR)/bin/$$foo || exit 42; done

$(SOURCE_DIR)/configure $(SOURCE_DIR)/config.h.in: $(SOURCE_DIR)/Makefile.am $(SOURCE_DIR)/configure.ac $(SOURCE_DIR)/bootstrap-buildsystem.sh
	cd $(SOURCE_DIR) && ./bootstrap-buildsystem.sh

## --with-estar=$(ESTAR_DIR)
## --enable-ros
$(BUILD_DIR)/config.h: $(SOURCE_DIR)/configure $(SOURCE_DIR)/config.h.in
	test -d $(BUILD_DIR) || mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && $(SOURCE_DIR)/configure --prefix=$(INST_DIR) \
                                                   --with-sfl=$(SFL_DIR) \
                                                   --with-boost=$(BOOST_DIR)

clean:
	- rm $(SIMULINKS)
	-$(MAKE) -C $(BUILD_DIR) clean
	rm -f installed patched

distclean:
	- rm $(SIMULINKS)
	$(MAKE) -C $(BUILD_DIR) distclean

wipe: clean
	rm -rf build $(INST_DIR)
