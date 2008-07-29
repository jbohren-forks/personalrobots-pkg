all: build

SVN_DIR = gmapping-svn
SVN_URL = https://svn.openslam.org/data/svn/gmapping/trunk 
SVN_REVISION = -r 35
SVN_PATCH = gmapping-r35.patch
include $(shell rospack find mk)/svn_checkout.mk

build: SVN_UP gmapping

configured: $(SVN_DIR)
	cd $(SVN_DIR) && ./configure
	touch configured

.PHONY: gmapping
gmapping: configured
	cd $(SVN_DIR) && make

clean:
	-cd $(SVN_DIR) && make clean
	rm -rf gmapping configured
wipe: clean
	rm -rf gmapping-svn

