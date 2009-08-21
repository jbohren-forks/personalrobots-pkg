all: installed

SVN_DIR = libtheora_svn
SVN_URL = http://svn.xiph.org/trunk/theora

#move this revision number forward as appropriate
SVN_REVISION = -r 16480

include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR)
	cd $(SVN_DIR) && ./autogen.sh --prefix=$(PWD)/installed
	cd $(SVN_DIR) && make && make install
	touch installed

clean:
	-make -C $(SVN_DIR) clean
	-$(RM) -r installed

wipe: clean
	-rm -rf $(SVN_DIR)
