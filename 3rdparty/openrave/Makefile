all: installed

SVN_DIR = openrave_svn
# Should really specify a revision
SVN_REVISION = -r 618
SVN_URL = https://openrave.svn.sourceforge.net/svnroot/openrave
include $(shell rospack find mk)/svn_checkout.mk

.PHONY: openrave

BOOST_INCLUDE_DIRS=$(shell rosboost-cfg --include_dirs)
BOOST_LIBRARY_DIRS=$(shell rosboost-cfg --lib_dirs)

installed: $(SVN_DIR) patched
	@cd $(SVN_DIR) && export PATH="$(shell rospack find soqt)/bin:$(shell rospack find opende)/opende/bin:$(PATH)" && export CPATH="$(shell rospack find bullet)/include" && export LD_LIBRARY_PATH=$(shell rospack find bullet)/lib:$(LD_LIBRARY_PATH) && mkdir -p build && cd build && cmake -DCMAKE_INSTALL_PREFIX=$(PWD) -DCMAKE_BUILD_TYPE=Release -DBoost_INCLUDE_DIR=$(BOOST_INCLUDE_DIRS) -DBoost_LIBRARY_DIR=$(BOOST_LIBRARY_DIRS) .. && make $(PARALLEL_JOBS) install
	touch installed

clean:
	make -C $(SVN_DIR) clean
	rm -f installed

wipe: clean
	rm -rf $(SVN_DIR) include lib share bin
