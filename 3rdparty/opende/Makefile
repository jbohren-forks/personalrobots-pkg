# change the next flag to 'no' if drawstuff is not to be built
WITH_DRAWSTUFF = yes

CFG_OPTIONS = --with-arch=nocona --enable-release --disable-debug --enable-double-precision
#REVISION = -r 1520
SVN_REVISION = -r 1534 #this seems to have the bNormalize error removed.

ifeq ($(WITH_DRAWSTUFF), yes)
CFG_OPTIONS += --with-drawstuff=X11
endif

all: build

SVN_DIR = opende-svn
SVN_URL = https://opende.svn.sourceforge.net/svnroot/opende/trunk 
include $(shell rospack find mk)/svn_checkout.mk

CONFIGURE_FLAGS = $(CFG_OPTIONS) CXXFLAGS='-O3 -fPIC -I$(ROOT)/include' LDFLAGS='-L$(ROOT)/lib' CFLAGS='-O3 -fPIC -I$(ROOT)/include -I$(ROOT)/usr/include'

ROOT = $(shell rospack find opende)/opende

build: SVN_UP opende

configured: $(SVN_DIR)
	cd $(SVN_DIR) && sh ./autogen.sh
	cd $(SVN_DIR) && ./configure --prefix=$(ROOT) --with-trimesh=opcode --enable-shared $(CONFIGURE_FLAGS)
	touch configured

.PHONY: opende
opende: configured
	cd $(SVN_DIR) && make
	cd $(SVN_DIR) && make install

clean:
	-cd $(SVN_DIR) && make clean
	rm -rf opende configured
wipe: clean
	rm -rf $(SVN_DIR)
