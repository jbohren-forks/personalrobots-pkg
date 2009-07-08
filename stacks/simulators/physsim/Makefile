all: installed

INSTALL_DIR = $(shell rospack find physsim)/physsim

SVN_DIR = physsim-svn
SVN_URL = https://physsim.svn.sourceforge.net/svnroot/physsim/Physsim
SVN_REVISION = -r 311
SVN_PATCH =  physsim_patch.diff

include $(shell rospack find mk)/svn_checkout.mk

#build: SVN_UP_REVERT_PATCH physsim
installed: $(SVN_DIR) patched
	cd $(SVN_DIR) && scons PREFIX_PATH=$(INSTALL_DIR)
	cd $(SVN_DIR) && scons PREFIX_PATH=$(INSTALL_DIR) install

clean:
	-cd $(SVN_DIR) && scons --clean
	rm -rf physsim installed
wipe: clean
	rm -rf $(SVN_DIR) physsim


