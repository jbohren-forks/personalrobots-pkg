all: stage-build

#REVISION = -r 6785

#	@if [ ! -d stage-svn ]; then svn co $(REVISION) https://playerstage.svn.sf.net/svnroot/playerstage/code/stage/trunk stage-svn; fi
stage-checkout:
	@if [ ! -d stage-svn ]; then svn co $(REVISION) https://playerstage.svn.sourceforge.net/svnroot/playerstage/code/stage/branches/stage-ros stage-svn; fi
	-cd stage-svn && svn up $(REVISION) 

CMAKE = `rospack find cmake`/cmake/bin/cmake 
CMAKE_ARGS = -Wno-dev -D CMAKE_BUILD_TYPE=RELEASE \
             -D CMAKE_INSTALL_PREFIX=$(PWD)/stage

stage-build: stage-checkout
	#cd stage-svn && mkdir -p build
	#cd stage-svn/build &&  $(CMAKE) $(CMAKE_ARGS) ..
	#cd stage-svn/build && make install
	cd stage-svn && autoreconf -i -s
	cd stage-svn && ./configure --prefix=$(PWD)/stage
	cd stage-svn && make install
	touch stage

clean:
	#rm -rf stage-svn/build stage
	cd stage-svn && make clean
	rm -rf stage

wipe:
	rm -rf stage stage-svn
