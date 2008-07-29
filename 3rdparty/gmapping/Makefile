all: gmapping

SVN_DIR = gmapping-svn
SVN_URL = https://svn.openslam.org/data/svn/gmapping/trunk 
SVN_REVISION = -r 35
SVN_PATCH = gmapping-r35.patch
include $(shell rospack find mk)/svn_checkout.mk

gmapping: SVN_UP
	cd $(SVN_DIR) && ./configure && make

clean:
	cd $(SVN_DIR) && make clean
	rm -rf gmapping
wipe: clean
	rm -rf gmapping-svn

