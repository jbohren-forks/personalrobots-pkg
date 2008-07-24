# change the next flag to 'no' if drawstuff is not to be built
WITH_DRAWSTUFF = yes

CFG_OPTIONS = --with-arch=nocona --enable-release --disable-debug --enable-double-precision
#REVISION = -r 1520
REVISION = -r 1534 #this seems to have the bNormalize error removed.

ifeq ($(WITH_DRAWSTUFF), yes)
CFG_OPTIONS += --with-drawstuff=X11
endif

#all: opende
all: check_svn

BUILD_TARGET = opende
SVN_TARGET = opende-checkout
SVN_DIR = opende-svn
include $(shell rospack find mk)/check_svn.mk

$(SVN_TARGET):
	@if [ ! -d $(SVN_DIR) ]; then svn co $(REVISION) https://opende.svn.sourceforge.net/svnroot/opende/trunk $(SVN_DIR); fi
	-cd $(SVN_DIR) && svn up $(REVISION)

CONFIGURE_FLAGS = $(CFG_OPTIONS) CXXFLAGS='-O3 -fPIC -I$(ROOT)/include' LDFLAGS='-L$(ROOT)/lib' CFLAGS='-O3 -fPIC -I$(ROOT)/include -I$(ROOT)/usr/include'

ROOT = $(shell rospack find opende)/opende

.PHONY: $(BUILD_TARGET)
$(BUILD_TARGET):
	cd $(SVN_DIR) && sh ./autogen.sh
	cd $(SVN_DIR) && ./configure --prefix=$(ROOT) --with-trimesh=opcode --enable-shared $(CONFIGURE_FLAGS)
	cd $(SVN_DIR) && make
	cd $(SVN_DIR) && make install

clean: check_svn_clean
	cd $(SVN_DIR) && make clean
	rm -rf opende
wipe: clean
	rm -rf $(SVN_DIR) opende

download: opende-checkout

