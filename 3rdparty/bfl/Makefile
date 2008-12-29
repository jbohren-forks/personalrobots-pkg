.PHONY: $(SVN_DIR) bfl

all: bfl

SVN_REVISION= -r 29808
SVN_DIR = bfl-svn
SVN_URL = http://svn.mech.kuleuven.be:/repos/orocos/trunk/bfl
INSTALL_DIR = bfl-boost
CMAKE = `rospack find cmake`/cmake/bin/cmake
BOOST_ROOT =$(shell rospack find boost)/boost 
CMAKE_ARGS = -DCMAKE_INSTALL_PREFIX=$(PWD)/$(INSTALL_DIR)/ \
	-DMATRIX_LIB=boost -DRNG_LIB=boost -DBOOST_FOUND=`rospack find boost`/boost/include/

include $(shell rospack find mk)/svn_checkout.mk



build: wiped bfl


configured: SVN_UP Makefile
	mkdir -p $(SVN_DIR)/build-boost
	cd $(SVN_DIR)/build-boost && $(CMAKE) $(CMAKE_ARGS) ..
	touch configured


bfl:	configured
	cd $(SVN_DIR)/build-boost && make $(PARALLEL_JOBS) && make install

clean:
	rm -rf $(INSTALL_DIR)
	rm -rf $(SVN_DIR)
	rm -rf configured

#test:
#	cd $(SVN_DIR)/build-boost && make check