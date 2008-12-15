
VERSION = player_svn-r7125
TARBALL = build/$(VERSION).tar.bz2
TARBALL_URL = http://pr.willowgarage.com/downloads/$(VERSION).tar.bz2
SOURCE_DIR = build/player-svn
UNPACK_CMD = tar xf

SVN_DIR = player-svn
SVN_URL = https://playerstage.svn.sourceforge.net/svnroot/playerstage/code/player/trunk
SVN_REVISION = -r 7125
#include $(shell rospack find mk)/svn_checkout.mk

all: player

include $(shell rospack find mk)/download_unpack_build.mk


CMAKE = `rospack find cmake`/cmake/bin/cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE=RELEASE \
             -D CMAKE_INSTALL_PREFIX=$(PWD)/player \
	     -D INCLUDE_RTK=OFF \
	     -D BUILD_PLAYERCC=OFF \
	     -D BUILD_UTILS=OFF \
	     -D BUILD_EXAMPLES=OFF \
	     -D PLAYER_BUILD_TESTS=OFF \
	     -D ENABLE_DRIVER_CAMERA1394=OFF \
	     -D BUILD_PYTHON_BINDINGS=OFF

configured: $(SOURCE_DIR) Makefile
	mkdir -p $(SOURCE_DIR)/build
	cd $(SOURCE_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	touch configured

.PHONY: player
player: configured
	cd $(SOURCE_DIR)/build && make $(PARALLEL_JOBS) install

clean:
	rm -rf $(SOURCE_DIR)/build
	rm -rf player configured

wipe: clean
	rm -rf build

