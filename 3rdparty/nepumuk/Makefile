all: installed

SVN_REVISION=    -r 968
BUILD_DIR=   $(PWD)/build
INST_DIR=    $(PWD)/local
SVN_DIR=     $(PWD)/build/nepumuk-svn
SVN_URL=     https://libsunflower.svn.sourceforge.net/svnroot/libsunflower/trunk/nepumuk

##later## ESTAR_DIR=   $(shell rospack find estar)/local
SFL_DIR=     $(shell rospack find libsunflower)/local
BOOST_DIR=   $(shell rosboost-cfg --include_dirs)

SIMULINKS=   nepumuk rostest ros0.sh robots.ros0 layout.ros0 expodemo.sh robots.expo layout.expo


include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) patched
	cd $(SVN_DIR) && ./bootstrap-buildsystem.sh
	test -d $(BUILD_DIR) || mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && $(SVN_DIR)/configure --prefix=$(INST_DIR) \
                                                --with-sfl=$(SFL_DIR) \
                                                --with-boost=$(BOOST_DIR)
	$(MAKE) -C $(BUILD_DIR) install
	for foo in $(SIMULINKS); do \
	  test -L $$foo || ln -s $(INST_DIR)/bin/$$foo || exit 42; done
	touch installed

clean:
	- rm $(SIMULINKS)
	-$(MAKE) -C $(BUILD_DIR) clean
	rm -f installed patched

distclean:
	- rm $(SIMULINKS)
	$(MAKE) -C $(BUILD_DIR) distclean

wipe:
	rm -rf $(BUILD_DIR) $(INST_DIR)
