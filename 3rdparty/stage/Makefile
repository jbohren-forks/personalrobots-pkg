all: stage-build

stage-checkout:
	@if [ ! -d stage-svn ]; then svn co https://playerstage.svn.sf.net/svnroot/playerstage/code/stage/trunk stage-svn; fi
	-cd stage-svn && svn up

stage-build: stage-checkout
	cd stage-svn && autoreconf -i -s
	cd stage-svn && ./configure --prefix=$(PWD)/stage
	cd stage-svn && make install

clean:
	cd stage-svn && make clean
	rm -rf stage

wipe: clean
	rm -rf stage-svn