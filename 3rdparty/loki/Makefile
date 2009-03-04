all: build

SVN_DIR = loki-svn
SVN_URL = https://loki-lib.svn.sourceforge.net/svnroot/loki-lib/tags/Release_0_1_5
SVN_PATCH = gcc4.3.patch
SVN_REVISION =
include $(shell rospack find mk)/svn_checkout.mk

build: SVN_UP_REVERT_PATCH loki

.PHONY: loki
loki: 
	cd $(SVN_DIR) && make $(ROS_PARALLEL_JOBS)
	if [ `uname` = Darwin ]; then \
		install_name_tool -id `rospack find loki`/loki-svn/lib/libloki.0.1.5.dylib loki-svn/lib/libloki.0.1.5.dylib; \
	fi

clean:
	-cd $(SVN_DIR) && make clean
wipe: 
	rm -rf $(SVN_DIR)

