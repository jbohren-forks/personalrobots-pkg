all: bullet_svn

VERSION = bullet_svn-r1497
TARBALL = build/$(VERSION).tar.bz2
TARBALL_URL = http://pr.willowgarage.com/downloads/$(VERSION).tar.bz2
SOURCE_DIR = build/bullet_svn
UNPACK_CMD = tar xf

SVN_DIR = bullet_svn
SVN_URL = http://bullet.googlecode.com/svn/trunk/
SVN_PATCH = quaternion.patch
SVN_REVISION=-r1497

#include $(shell rospack find mk)/svn_checkout.mk
include $(shell rospack find mk)/download_unpack_build.mk


bullet_svn: $(SOURCE_DIR)
	ln -sf $(SOURCE_DIR) bullet_svn
	cd $(SOURCE_DIR) && cmake .
	make $(PARALLEL_JOBS) -C $(SOURCE_DIR) LinearMath
	touch bullet

clean:
	-make -C $(SOURCE_DIR) clean

wipe: 
	-rm -f bullet patched
	-rm -rf bullet_svn
	-rm -rf build
