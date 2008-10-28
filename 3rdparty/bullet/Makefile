all: build

SVN_DIR = bullet_svn
SVN_URL = http://bullet.googlecode.com/svn/trunk/
SVN_PATCH = quaternion.patch
SVN_REVISION=-r1422

include $(shell rospack find mk)/svn_checkout.mk

.PHONY: build


build: SVN_UP_REVERT_PATCH
	cd $(SVN_DIR) && cmake .
	make $(PARALLEL_JOBS) -C $(SVN_DIR) LibLinearMath
	touch bullet

clean:
	-make -C $(SVN_DIR) clean

wipe: clean
	rm -rf $(SVN_DIR)
