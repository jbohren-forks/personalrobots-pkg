all: stage-build

REVISION = -r {2008-05-26}

stage-checkout:
	@if [ ! -d stage-svn ]; then svn co $(REVISION) https://playerstage.svn.sf.net/svnroot/playerstage/code/stage/trunk stage-svn; fi
	-cd stage-svn && svn up $(REVISION) 

stage-build: stage-checkout
	cd stage-svn && autoreconf -i -s
	cd stage-svn && ./configure --prefix=$(PWD)/stage
	cd stage-svn && make install

clean:
	cd stage-svn && make clean
	rm -rf stage

wipe: clean
	rm -rf stage-svn
