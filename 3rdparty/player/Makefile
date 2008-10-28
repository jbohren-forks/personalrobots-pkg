all: build

SVN_DIR = player-svn
SVN_URL = https://playerstage.svn.sourceforge.net/svnroot/playerstage/code/player/trunk
SVN_REVISION = -r 7108
include $(shell rospack find mk)/svn_checkout.mk


CMAKE = `rospack find cmake`/cmake/bin/cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE=RELEASE \
             -D CMAKE_INSTALL_PREFIX=$(PWD)/player \
	     -D INCLUDE_RTKGUI=ON \
	     -D BUILD_PLAYERCC_BOOST=OFF \
	     -D BUILD_UTILS_PMAP=OFF \
	     -D ENABLE_DRIVER_CAMERA1394=OFF \
	     -D BUILD_PYTHON_BINDINGS=OFF

build: SVN_UP player

configured: $(SVN_DIR) Makefile
	mkdir -p $(SVN_DIR)/build
	cd $(SVN_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	touch configured

.PHONY: player
player: configured
	cd $(SVN_DIR)/build && make $(PARALLEL_JOBS) install

clean:
	rm -rf $(SVN_DIR)/build
	rm -rf player configured
wipe: clean
	rm -rf $(SVN_DIR)

