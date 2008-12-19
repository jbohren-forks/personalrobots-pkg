all: openrave

SVN_DIR = openrave_svn
# Should really specify a revision
SVN_REVISION = -r 574
SVN_URL = https://openrave.svn.sourceforge.net/svnroot/openrave
include $(shell rospack find mk)/svn_checkout.mk

.PHONY: openrave

build: SVN_UP openrave

openrave: SVN_UP
	@cd $(SVN_DIR) && export PATH="$(shell rospack find soqt)/bin:$(shell rospack find opende)/opende/bin:$(PATH)" && export CPATH="$(shell rospack find bullet)/include" && export LD_LIBRARY_PATH=$(shell rospack find bullet)/lib:$(LD_LIBRARY_PATH) && export BOOST_ROOT=$(shell rospack find boost)/boost && mkdir -p build && cd build && $(shell rospack find cmake)/cmake/bin/cmake -DCMAKE_INSTALL_PREFIX=$(PWD) -DCMAKE_BUILD_TYPE=Release .. && make $(PARALLEL_JOBS) install

clean:
	make -C $(SVN_DIR) clean

wipe: clean
	rm -rf $(SVN_DIR) include lib share bin
