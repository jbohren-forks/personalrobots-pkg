all: installed

SVN_REVISION= -r 29808
SVN_DIR = bfl-svn
SVN_URL = http://svn.mech.kuleuven.be:/repos/orocos/trunk/bfl
SVN_PATCH = mcpdf.patch warning.patch
INSTALL_DIR = bfl-boost
CMAKE = cmake
BOOST_INCLUDE =$(shell rosboost-cfg --include_dirs)
CMAKE_ARGS = -DCMAKE_INSTALL_PREFIX=$(PWD)/$(INSTALL_DIR)/ \
	-DMATRIX_LIB=boost -DRNG_LIB=boost -DBOOST_FOUND=$(BOOST_INCLUDE)

include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) patched
	mkdir -p $(SVN_DIR)/build-boost
	cd $(SVN_DIR)/build-boost && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SVN_DIR)/build-boost && make $(PARALLEL_JOBS) && make install
	touch installed

clean:
	rm -rf $(INSTALL_DIR)
	rm -rf $(SVN_DIR)
	rm -rf installed

