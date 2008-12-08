all: spacenavd libspnav

SVN_DIR = spacenav_svn
SVN_URL = https://spacenav.svn.sourceforge.net/svnroot/spacenav/trunk
SVN_REVISION = -r 47
include $(shell rospack find mk)/svn_checkout.mk

.PHONY: spacenavd libspnav

spacenavd: spacenavd_configured
	cd $(SVN_DIR)/spacenavd && make && make install

spacenavd_configured: spacenav_svn
	cd $(SVN_DIR)/spacenavd && ./configure --prefix=$(PWD)/spacenavd
	touch $@

libspnav: libspnav_configured
	mkdir -p libspnav/lib libspnav/lib64 libspnav/include
	cd $(SVN_DIR)/libspnav && make && make install

libspnav_configured: spacenav_svn
	cd $(SVN_DIR)/libspnav && ./configure --prefix=$(PWD)/libspnav
	touch $@

clean:
	-make -C $(SVN_DIR)/spacenavd clean
	-make -C $(SVN_DIR)/libspnav clean
	-$(RM) -r libspnav spacenavd libspnav_configured spacenavd_configured

wipe: clean
	-rm -rf $(SVN_DIR)
