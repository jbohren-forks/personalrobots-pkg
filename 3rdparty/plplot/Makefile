all: installed

SVN_DIR = plplot-svn
SVN_URL = https://plplot.svn.sf.net/svnroot/plplot/trunk 
SVN_REVISION = -r 8746
include $(shell rospack find mk)/svn_checkout.mk

CMAKE = cmake 
CMAKE_ARGS = -DCMAKE_INSTALL_PREFIX=$(PWD)/plplot \
	-DPLD_wxwidgets:BOOL=ON

installed: $(SVN_DIR) patched
	cd $(SVN_DIR) && mkdir -p build
	cd $(SVN_DIR)/build &&  $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SVN_DIR)/build && make install
	touch installed

clean:
	rm -rf $(SVN_DIR)/build
	rm -rf plplot installed
wipe: clean
	rm -rf plplot-svn
