all: player

player-svn:
	@if [ ! -d player-svn ]; then svn co https://playerstage.svn.sf.net/svnroot/playerstage/code/player/trunk player-svn; fi
	-cd player-svn && svn up

player: player-svn
	cd player-svn && mkdir -p build
	cd player-svn/build && cmake -D CMAKE_INSTALL_PREFIX=$(PWD)/player -D BUILD_PLAYERCC_BOOST=OFF ..
	cd player-svn/build && make install
	touch player

clean:
	cd player-svn/build && make clean
	rm -rf player
wipe:
	rm -rf player-svn player

