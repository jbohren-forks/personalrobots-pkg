all: installed

SVN_DIR = openrave_svn
# Should really specify a revision
SVN_REVISION = -r 848
SVN_URL = https://openrave.svn.sourceforge.net/svnroot/openrave
#SVN_PATCH = fini_patch.patch
include $(shell rospack find mk)/svn_checkout.mk

.PHONY: openrave

BOOST_INCLUDE_DIRS=$(shell rosboost-cfg --include_dirs)
BOOST_LIBRARY_DIRS=$(shell rosboost-cfg --lib_dirs)

BULLET_DIR=$(shell rospack find bullet)
BULLET_INCLUDE_DIR=$(shell rospack find bullet)/build/include
BULLET_LIB_DIR=$(shell rospack find bullet)/lib
BULLET_CFLAGS=$(shell rospack export --lang=cpp --attrib=cflags bullet)
BULLET_LFLAGS=$(shell rospack export --lang=cpp --attrib=lflags bullet)

# create 
installed: $(SVN_DIR) patched
	echo "prefix=${BULLET_DIR}\n\
libdir=${BULLET_INCLUDE_DIR}\n\
includedir=${BULLET_LIB_DIR}\n\
\n\
Name: Bullet\n\
Description: Bullet as built by ROS\n\
Version: 2.74\n\
Libs: ${BULLET_LFLAGS}\n\
CFlags: ${BULLET_CFLAGS}" > bullet.pc
	cd $(SVN_DIR) && export PKG_CONFIG_PATH="$(PKG_CONFIG_PATH):$(PWD)" && export PATH="$(shell rospack find soqt)/bin:$(shell rospack find opende)/opende/bin:$(PATH)" && mkdir -p build && cd build && BOOST_INCLUDEDIR=$(BOOST_INCLUDE_DIRS) BOOST_LIBRARYDIR=$(BOOST_LIBRARY_DIRS) cmake -DCMAKE_INSTALL_PREFIX=$(PWD) -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBoost_INCLUDE_DIR=$(BOOST_INCLUDE_DIRS) -DBoost_LIBRARY_DIR=$(BOOST_LIBRARY_DIRS) .. && export PARALLEL_JOBS=ROS_PARALLEL_JOBS && make $(ROS_PARALLEL_JOBS) install
	touch installed

clean:
	make -C $(SVN_DIR) clean
	rm -f installed

wipe: clean
	rm -rf $(SVN_DIR) include lib share bin
