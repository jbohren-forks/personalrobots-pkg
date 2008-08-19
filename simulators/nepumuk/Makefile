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

all: tarball-all

SVN_DIR=     $(PWD)/nepumuk-svn
SVN_REV=     HEAD
SVN_URL=     https://libsunflower.svn.sourceforge.net/svnroot/libsunflower/trunk/nepumuk

VERSION=     r903

TARBALL=     nepumuk-$(VERSION).tar.gz
TARBALL_URL= http://downloads.sourceforge.net/libsunflower/$(TARBALL)
SOURCE_DIR=  $(PWD)/nepumuk-$(VERSION)
UNPACK_CMD=  tar xfz

SYMLINK=   npm
SYMTARGET= $(SOURCE_DIR)

include $(shell rospack find mk)/download_unpack.mk

.PHONY: really-all tarball-all svn-all checkout

svn-all: checkout
	$(MAKE) SOURCE_DIR=$(SVN_DIR) really-all

tarball-all: $(SOURCE_DIR)
	$(MAKE) really-all

rebuild: clean really-all

really-all: symlinks build/Makefile
	test -d build || mkdir build
	$(MAKE) -C build

checkout:
	test -d $(SVN_DIR) || svn co -r$(SVN_REV) $(SVN_URL) $(SVN_DIR)

symlinks:
	$(MAKE) SYMLINK=src         SYMTARGET=$(SOURCE_DIR)                   symlink
	$(MAKE) SYMLINK=npm         SYMTARGET=$(SOURCE_DIR)                   symlink
	$(MAKE) SYMLINK=ros0.sh     SYMTARGET=$(SOURCE_DIR)/simul/ros0.sh     symlink
	$(MAKE) SYMLINK=robots.ros0 SYMTARGET=$(SOURCE_DIR)/simul/robots.ros0 symlink
	$(MAKE) SYMLINK=layout.ros0 SYMTARGET=$(SOURCE_DIR)/simul/layout.ros0 symlink

symlink:
	test -L $(SYMLINK) || ln -s $(SYMTARGET) $(SYMLINK)

build/Makefile: CMakeLists.txt manifest.xml
	mkdir -p build
	cd build && `rospack find cmake`/cmake/bin/cmake -DCMAKE_TOOLCHAIN_FILE=`rospack find rosbuild`/rostoolchain.cmake ..

clean:
	rm -rf lib build npm ros0.sh robots.ros0 layout.ros0
