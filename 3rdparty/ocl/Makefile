all: installed

SVN_REVISION=-r 30170
SVN_DIR = ocl-svn
SVN_URL = http://svn.mech.kuleuven.be:/repos/orocos/trunk/ocl
SVN_PATCH=total.patch
INSTALL_DIR = ocl
CMAKE = cmake
BOOST_INCLUDE =$(shell rosboost-cfg --include_dirs)
BOOST_LIB_DIR = $(shell rosboost-cfg --lib_dirs)
BOOST_OPTIONS_LIB = ${BOOST_LIB_DIR}/lib$(shell echo `rosboost-cfg --libs program_options` | sed s/-l//).so 
OROCOS_INSTALL=`rospack find rtt`/rtt
KDL_INSTALL=`rospack find kdl`/kdl
BFL_INSTALL=`rospack find bfl`/bfl-boost

CMAKE_ARGS = -DCMAKE_INSTALL_PREFIX=$(PWD)/$(INSTALL_DIR)/ \
		-DOROCOS_TARGET=gnulinux \
		-DCMAKE_BUILD_TYPE="RTT" \
		-DOROCOS_INSTALL=$(OROCOS_INSTALL) \
		-DKDL_INSTALL=$(KDL_INSTALL) \
		-DBFL_INSTALL=$(BFL_INSTALL) \
		-DBUILD_HELLOWORLD=OFF \
		-DBUILD_REPORTING=OFF \
		-DBUILD_TIMER=OFF \
		-DBUILD_TASKBROWSER=ON \
		-DBUILD_DEPLOYMENT=ON \
		-DBUILD_MOTION=ON \
		-DBUILD_NAXES=ON \
		-DBUILD_HARDWARE=ON \
		-DBUILD_SIM_ROB=ON \
		-DCMAKE_LD_FLAGS="-Wl,-rpath,-L$(OROCOS_INSTALL)/lib" \
		-DBOOST=$(BOOST_INCLUDE)\
		-DBOOST_OPTIONS=$(BOOST_INCLUDE)\
		-DBOOST_OPTIONS_LIB_DIR=$(BOOST_LIB_DIR)\
		-DBOOST_OPTIONS_LIBRARY=$(BOOST_OPTIONS_LIB)

include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) patched
	-mkdir -p $(SVN_DIR)/build
	cd $(SVN_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SVN_DIR)/build && make $(ROS_PARALLEL_JOBS) && make install
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	rm -rf $(INSTALL_DIR)/bin
	rm -rf $(INSTALL_DIR)/lib
	rm -rf $(INSTALL_DIR)/include
	rm -rf $(SVN_DIR)
	rm -rf installed

wipe: 	clean
	rm -rf $(SVN_DIR)/build
