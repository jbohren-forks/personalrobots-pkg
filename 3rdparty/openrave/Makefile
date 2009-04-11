all: installed

SVN_DIR = openrave_svn
# Should really specify a revision
SVN_REVISION = -r 757
SVN_URL = https://openrave.svn.sourceforge.net/svnroot/openrave
#SVN_PATCH = fini_patch.patch
include $(shell rospack find mk)/svn_checkout.mk

.PHONY: openrave

BOOST_INCLUDE_DIRS=$(shell rosboost-cfg --include_dirs)
BOOST_LIBRARY_DIRS=$(shell rosboost-cfg --lib_dirs)

installed: $(SVN_DIR) patched
	cd $(SVN_DIR) && export PATH="$(shell rospack find soqt)/bin:$(shell rospack find opende)/opende/bin:$(PATH)" && mkdir -p build && cd build && export CPATH="$(shell rospack find bullet)/include:$(CPATH)" && export LD_LIBRARY_PATH=$(shell rospack find bullet)/lib:$(LD_LIBRARY_PATH) && BOOST_INCLUDEDIR=$(BOOST_INCLUDE_DIRS) BOOST_LIBRARYDIR=$(BOOST_LIBRARY_DIRS) cmake -DCMAKE_INSTALL_PREFIX=$(PWD) -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBoost_INCLUDE_DIR=$(BOOST_INCLUDE_DIRS) -DBoost_LIBRARY_DIR=$(BOOST_LIBRARY_DIRS) .. && export PARALLEL_JOBS=ROS_PARALLEL_JOBS && make $(ROS_PARALLEL_JOBS) install
	touch installed

clean:
	make -C $(SVN_DIR) clean
	rm -f installed

wipe: clean
	rm -rf $(SVN_DIR) include lib share bin
