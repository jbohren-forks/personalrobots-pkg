all: build

SVN_DIR = stage-svn
SVN_URL = https://playerstage.svn.sourceforge.net/svnroot/playerstage/code/stage/branches/stage-ros
SVN_REVISION =
include $(shell rospack find mk)/svn_checkout.mk

build: SVN_UP stage

stage: $(SVN_DIR)
	cd $(SVN_DIR) && autoreconf -i -s
	cd $(SVN_DIR) && ./configure --prefix=$(PWD)/stage
	cd $(SVN_DIR) && make install
	touch stage

clean:
	-cd $(SVN_DIR) && make clean
	rm -rf stage

wipe: clean
	rm -rf $(SVN_DIR)
