all: build

SVN_DIR = gmapping-svn
SVN_URL = https://svn.openslam.org/data/svn/gmapping/trunk 
SVN_REVISION = -r 39
SVN_PATCH = gmapping-r39.patch
include $(shell rospack find mk)/svn_checkout.mk

build: wiped SVN_UP gmapping

configured: $(SVN_DIR) Makefile
	cd $(SVN_DIR) && ./configure
	touch configured

wiped: Makefile
	make wipe
	touch wiped

.PHONY: gmapping
gmapping: configured
	cd $(SVN_DIR) && make

clean:
	-cd $(SVN_DIR) && make clean
	rm -rf gmapping configured
wipe: clean
	rm -rf gmapping-svn

