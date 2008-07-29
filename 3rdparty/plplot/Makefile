all: build

SVN_DIR = plplot-svn
SVN_URL = https://plplot.svn.sf.net/svnroot/plplot/trunk 
include $(shell rospack find mk)/svn_checkout.mk

CMAKE = `rospack find cmake`/cmake/bin/cmake 
CMAKE_ARGS = -DCMAKE_INSTALL_PREFIX=$(PWD)/plplot \
	-DPLD_wxwidgets:BOOL=ON

build: SVN_UP plplot

plplot: $(SVN_DIR)
	cd $(SVN_DIR) && mkdir -p build
	cd $(SVN_DIR)/build &&  $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SVN_DIR)/build && make install
	touch plplot

clean:
	rm -rf $(SVN_DIR)/build
	rm -rf plplot
wipe: clean
	rm -rf plplot-svn
