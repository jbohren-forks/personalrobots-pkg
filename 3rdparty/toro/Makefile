all: build

SVN_DIR = toro_svn
SVN_URL = https://svn.openslam.org/data/svn/toro
include $(shell rospack find mk)/svn_checkout.mk

build: SVN_UP toro

configured: $(SVN_DIR) Makefile
	touch configured

.PHONY: toro
toro: configured
	cd $(SVN_DIR)/trunk && make 

clean:
	-cd $(SVN_DIR) && make clean
	rm -rf toro configured

wipe: clean
	rm -rf $(SVN_DIR)
