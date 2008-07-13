all: plplot

plplot-checkout:
	@if [ ! -d plplot-svn ]; then svn co https://plplot.svn.sf.net/svnroot/plplot/trunk plplot-svn; fi
	-cd plplot-svn && svn up $(REVISION) 

CMAKE = `rospack find cmake`/cmake/bin/cmake 
CMAKE_ARGS = -DCMAKE_INSTALL_PREFIX=$(PWD)/plplot \
	-DPLD_wxwidgets:BOOL=ON

plplot: plplot-checkout
	cd plplot-svn && mkdir -p build
	cd plplot-svn/build &&  $(CMAKE) $(CMAKE_ARGS) ..
	cd plplot-svn/build && make install
	touch plplot

clean:
	rm -rf plplot-svn/build
	rm -rf plplot
wipe:
	rm -rf plplot-svn plplot

download: plplot-checkout

