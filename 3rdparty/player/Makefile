all: check_svn

BUILD_TARGET = player
SVN_TARGET = player-checkout
SVN_DIR = player-svn
include $(shell rospack find mk)/check_svn.mk

REVISION = -r 6833

$(SVN_TARGET):
	@if [ ! -d $(SVN_DIR) ]; then svn co $(REVISION) https://playerstage.svn.sf.net/svnroot/playerstage/code/player/trunk $(SVN_DIR); fi
	-cd $(SVN_DIR) && svn up $(REVISION) 

CMAKE = `rospack find cmake`/cmake/bin/cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE=RELEASE \
             -D CMAKE_INSTALL_PREFIX=$(PWD)/player \
	     -D BUILD_PLAYERCC_BOOST=OFF \
	     -D ENABLE_DRIVER_CAMERA1394=OFF

.PHONY: $(BUILD_TARGET)
$(BUILD_TARGET):
	mkdir -p player-svn/build
	cd player-svn/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd player-svn/build && make install

clean: check_svn_clean
	rm -rf player-svn/build
	rm -rf player
wipe:
	rm -rf $(SVN_DIR) player

download: player-checkout

