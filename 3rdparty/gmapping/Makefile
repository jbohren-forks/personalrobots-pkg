all: gmapping

REVISION = -r 35

gmapping-svn:
	@if [ ! -d gmapping-svn ]; then svn co $(REVISION) https://svn.openslam.org/data/svn/gmapping/trunk gmapping-svn && cd gmapping-svn && patch -p0 < ../gmapping-r35.patch; fi

gmapping: gmapping-svn
	cd gmapping-svn && ./configure && make

clean:
	cd gmapping-svn && make clean
	rm -rf gmapping
wipe:
	rm -rf gmapping-svn gmapping

