all: player

REVISION = -r 6833

player-checkout:
	@if [ ! -d player-svn ]; then svn co $(REVISION) https://playerstage.svn.sf.net/svnroot/playerstage/code/player/trunk player-svn; fi
	-cd player-svn && svn up $(REVISION) 

CMAKE = `rospack find cmake`/cmake/bin/cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE=RELEASE \
             -D CMAKE_INSTALL_PREFIX=$(PWD)/player \
	     -D BUILD_PLAYERCC_BOOST=OFF \
	     -D ENABLE_DRIVER_CAMERA1394=OFF

player: player-checkout
	cd player-svn && mkdir -p build
	cd player-svn/build &&  $(CMAKE) $(CMAKE_ARGS) ..
	cd player-svn/build && make install
	touch player

clean:
	rm -rf player-svn/build
	rm -rf player
wipe:
	rm -rf player-svn player

download: player-checkout

