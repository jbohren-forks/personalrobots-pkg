all: build

SVN_DIR = octave-forge-svn
SVN_URL = https://octave.svn.sourceforge.net/svnroot/octave/trunk/octave-forge
SVN_REVISION = -r 5505
SVN_PATCH = 
include $(shell rospack find mk)/svn_checkout.mk

#OCT_PATH = $(shell rospack find octave)/build/octave

build: SVN_UP_REVERT_PATCH octave-forge

.PHONY: octave-forge
octave-forge: $(SVN_DIR)
	if [ ! -d octave-forge ]; then mkdir -p octave-forge; mkdir -p octave-forge/lib -p octave-forge/include -p octave-forge/bin; fi
	#-cd $(SVN_DIR) && ./autogen.sh
	#-cd $(SVN_DIR) && PATH=$(PATH):$(OCT_PATH) ./configure --prefix=$(PWD)/octave-forge
	#-cd $(SVN_DIR) && ./configure --prefix=$(PWD)/octave-forge
	#-cd $(SVN_DIR) && make all
	touch octave-forge

clean:
	-cd $(SVN_DIR) && make clean
	rm -rf octave-forge
wipe: clean
	rm -rf $(SVN_DIR) octave-forge
