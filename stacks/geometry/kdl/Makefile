all: installed

SVN_REVISION=-r 30215
SVN_DIR = kdl-svn-trunk
OLD_SVN_DIR = kdl-svn
SVN_URL = http://svn.mech.kuleuven.be/repos/orocos/trunk/kdl
SVN_PATCH=total.patch
INSTALL_DIR = kdl
CMAKE = cmake
BOOST_INCLUDE =$(shell rosboost-cfg --include_dirs)
EIGEN2_INCLUDE_DIR=`rospack find eigen`/build/eigen2/
TINYXML_INCLUDE_DIR=`rospack find tinyxml`/inlucde/
CMAKE_ARGS = -DCMAKE_INSTALL_PREFIX=$(PWD)/$(INSTALL_DIR)/ \
		-DPYTHON_BINDINGS=OFF \
		-DBUILD_MODELS=OFF \
		-DENABLE_TESTS=ON \
		-DBOOST:STRING=$(BOOST_INCLUDE)\
		-DEIGEN2_INCLUDE_DIR=$(EIGEN2_INCLUDE_DIR)\
		-DTINYXML_INCLUDE_DIR=$(TINYXML_INCLUDE_DIR)\
		-DCMAKE_BUILD_TYPE="Release"

include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) patched Makefile
	-mkdir -p $(SVN_DIR)/build
	cd $(SVN_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SVN_DIR)/build && make $(ROS_PARALLEL_JOBS) && make install
	if [ `uname` = Darwin ]; then \
		install_name_tool -id `rospack find kdl`/kdl/lib/liborocos-kdl.1.0.dylib kdl/lib/liborocos-kdl.1.0.dylib; \
	fi
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	rm -rf $(INSTALL_DIR)
	rm -rf $(SVN_DIR)/build
	rm -rf installed

wipe: 	clean
	rm -rf $(SVN_DIR)
	rm -rf $(OLD_SVN_DIR)