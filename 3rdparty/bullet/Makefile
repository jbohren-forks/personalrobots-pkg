all: bullet

SVN_DIR = bullet_svn
SVN_URL = http://bullet.googlecode.com/svn/trunk/
SVN_PATCH = quaternion.patch
include $(shell rospack find mk)/svn_checkout.mk

.PHONY: bullet

build: SVN_UP_REVERT_PATCH bullet

bullet: $(SVN_DIR) quaternion.patch aCos_overflow.diff
	cd $(SVN_DIR) && cmake .
	make -C $(SVN_DIR) LibLinearMath
	touch bullet
clean:
	make -C $(SVN_DIR) clean

wipe: clean
	rm -rf $(SVN_DIR)
