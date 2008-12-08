all: openrave

SVN_DIR = openrave_svn
# Should really specify a revision
SVN_REVISION = -r 542
SVN_URL = https://openrave.svn.sourceforge.net/svnroot/openrave
include $(shell rospack find mk)/svn_checkout.mk

.PHONY: openrave

build: SVN_UP openrave

openrave: SVN_UP
	@cd $(SVN_DIR) && PATH="$(shell rospack find soqt)/bin:$(shell rospack find opende)/opende/bin:$(PATH)" && CPATH="$(shell rospack find bullet)/include" && LD_LIBRARY_PATH=$(shell rospack find bullet)/lib:$(LD_LIBRARY_PATH) && svn up && make prefix=$(PWD) && make install

clean:
	make -C $(SVN_DIR) clean

wipe: clean
	rm -rf $(SVN_DIR) include lib share bin
