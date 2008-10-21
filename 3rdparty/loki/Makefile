all: build

SVN_DIR = loki-svn
SVN_URL = https://loki-lib.svn.sourceforge.net/svnroot/loki-lib/tags/Release_0_1_5
SVN_REVISION =
include $(shell rospack find mk)/svn_checkout.mk

build: SVN_UP loki

.PHONY: loki
loki:
	cd $(SVN_DIR) && make $(PR2_JOBS)

clean:
	-cd $(SVN_DIR) && make clean
wipe: 
	rm -rf $(SVN_DIR)

