all: build

SVN_DIR = lapackpp-svn
SVN_URL = https://lapackpp.svn.sourceforge.net/svnroot/lapackpp/lapackpp/trunk/
SVN_REVISION = -r 414
SVN_PATCH = dgev.patch
include $(shell rospack find mk)/svn_checkout.mk


build: SVN_UP_REVERT_PATCH lapackpp

.PHONY: lapackpp
lapackpp: $(SVN_DIR)
	if [ ! -d lapackpp ]; then mkdir -p lapackpp; mkdir -p lapackpp/lib -p lapackpp/include -p lapackpp/bin; fi
	-cd $(SVN_DIR) && ./autogen.sh
	-cd $(SVN_DIR) && ./configure --prefix=$(PWD)/lapackpp
	-cd $(SVN_DIR) && make
	-cd $(SVN_DIR) && make install
	touch lapackpp

clean:
	-cd $(SVN_DIR) && make clean
	rm -rf lapackpp
wipe: clean
	rm -rf $(SVN_DIR) lapackpp
