all: build

SVN_DIR = bullet_svn
SVN_URL = http://bullet.googlecode.com/svn/trunk/
SVN_PATCH = quaternion.patch demos.patch
SVN_REVISION=-r1555

include $(shell rospack find mk)/svn_checkout.mk

.PHONY: build


build: SVN_UP_REVERT_PATCH
	cd $(SVN_DIR) && cmake -DCMAKE_INSTALL_PREFIX=$(PWD) .
	cd $(SVN_DIR) && make $(PARALLEL_JOBS) && make $(PARALLEL_JOBS) install
	touch bullet

clean:
	-make -C $(SVN_DIR) clean

wipe: 
	-rm -f bullet patched
	-rm -rf bullet_svn
	-rm -rf build
