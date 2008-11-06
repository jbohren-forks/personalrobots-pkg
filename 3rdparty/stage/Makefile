all: build

SVN_DIR = stage-svn
SVN_URL = https://playerstage.svn.sourceforge.net/svnroot/playerstage/code/stage/branches/stage-ros
SVN_REVISION = -r 7126
include $(shell rospack find mk)/svn_checkout.mk

build: SVN_UP stage

configured: $(SVN_DIR) Makefile
	cd $(SVN_DIR) && autoreconf -i -s
	cd $(SVN_DIR) && ./configure --prefix=$(PWD)/stage
	touch configured

.PHONY: stage
stage: configured
	cd $(SVN_DIR) && make install

clean:
	-cd $(SVN_DIR) && make clean
	rm -rf stage configured

wipe: clean
	rm -rf $(SVN_DIR)
