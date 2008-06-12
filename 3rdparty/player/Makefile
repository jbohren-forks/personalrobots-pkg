all: player

REVISION = -r 6497

player-svn:
	@if [ ! -d player-svn ]; then svn co $(REVISION) https://playerstage.svn.sf.net/svnroot/playerstage/code/player/trunk player-svn; fi
	-cd player-svn && svn up $(REVISION) 

player: player-svn
	cd player-svn && mkdir -p build
	cd player-svn/build && cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$(PWD)/player -D BUILD_PLAYERCC_BOOST=OFF -D ENABLE_DRIVER_CAMERA1394=OFF ..
	cd player-svn/build && make install
	touch player

clean:
	cd player-svn/build && make clean
	rm -rf player
wipe:
	rm -rf player-svn player

