all: build

SVN_DIR = bullet_svn
SVN_URL = http://bullet.googlecode.com/svn/trunk/
SVN_PATCH = quaternion.patch
include $(shell rospack find mk)/svn_checkout.mk

.PHONY: build


build: SVN_UP_REVERT_PATCH
	cd $(SVN_DIR) && cmake .
	make $(PR2_JOBS) -C $(SVN_DIR) LibLinearMath
	touch bullet

clean:
	-make -C $(SVN_DIR) clean

wipe: clean
	rm -rf $(SVN_DIR)
