all: installed

SVN_REVISION=-r 30170
SVN_DIR = rtt-svn
SVN_URL = http://svn.mech.kuleuven.be:/repos/orocos/trunk/rtt
INSTALL_DIR = rtt
CMAKE = cmake
BOOST_INCLUDE =$(shell rosboost-cfg --include_dirs)
CMAKE_ARGS = -DCMAKE_INSTALL_PREFIX=$(PWD)/$(INSTALL_DIR)/ \
		-DOROCOS_TARGET=gnulinux \
		-DBOOST_DIR=$(BOOST_INCLUDE)\
		-DHAS_BOOST_SPIRIT=$(BOOST_INCLUDE)\
		-DHAS_BOOST_GRAPH=$(BOOST_INCLUDE)\
		-DCMAKE_BUILD_TYPE="RTT"
include $(shell rospack find mk)/svn_checkout.mk


installed: $(SVN_DIR) Makefile
	-mkdir -p $(SVN_DIR)/build
	cd $(SVN_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SVN_DIR)/build && make $(ROS_PARALLEL_JOBS) && make install
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	rm -rf $(INSTALL_DIR)
	rm -rf $(SVN_DIR)
	rm -rf installed

wipe: 	clean
	rm -rf $(SVN_DIR)/build
