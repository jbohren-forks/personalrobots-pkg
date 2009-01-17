all: installed

SVN_DIR = openrave_svn
# Should really specify a revision
SVN_REVISION = -r 610
SVN_URL = https://openrave.svn.sourceforge.net/svnroot/openrave
include $(shell rospack find mk)/svn_checkout.mk

.PHONY: openrave

installed: $(SVN_DIR) patched
	@cd $(SVN_DIR) && export PATH="$(shell rospack find soqt)/bin:$(shell rospack find opende)/opende/bin:$(PATH)" && export CPATH="$(shell rospack find bullet)/include" && export LD_LIBRARY_PATH=$(shell rospack find bullet)/lib:$(LD_LIBRARY_PATH) && export BOOST_ROOT=$(shell rospack find boost)/boost && mkdir -p build && cd build && cmake -DCMAKE_INSTALL_PREFIX=$(PWD) -DCMAKE_BUILD_TYPE=Release .. && make $(PARALLEL_JOBS) install
	touch installed

clean:
	make -C $(SVN_DIR) clean
	rm -f installed

wipe: clean
	rm -rf $(SVN_DIR) include lib share bin
