all: player-build

player-checkout:
	@if [ ! -d player-svn ]; then svn co https://playerstage.svn.sf.net/svnroot/playerstage/code/player/trunk player-svn; fi
	-cd player-svn && svn up

player-build: player-checkout
	cd player-svn && mkdir -p build
	cd player-svn/build && cmake -D CMAKE_INSTALL_PREFIX=$(PWD)/player -D BUILD_PLAYERCC_BOOST=OFF ..
	cd player-svn/build && make install

clean:
	rm -rf player-svn player
