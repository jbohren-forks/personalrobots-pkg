all: build

SVN_DIR = kdl-svn
SVN_URL = http://svn.mech.kuleuven.be/repos/orocos/trunk/kdl
SVN_REVISION =  -r 29562
SVN_PATCH = kdl_patch29562.diff
include $(shell rospack find mk)/svn_checkout.mk

CMAKE = `rospack find cmake`/cmake/bin/cmake 
CMAKE_ARGS = -DCMAKE_INSTALL_PREFIX=$(PWD)/kdl/ \
		-DPYTHON_BINDINGS=ON \
		-DKDL_PYTHON_INSTALL_PATH=$(PWD)/kdl/lib/

build: wiped SVN_UP kdl

configured: $(SVN_DIR) Makefile
	mkdir -p $(SVN_DIR)/build
	cd $(SVN_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	touch configured

wiped: Makefile
	make wipe
	touch wiped
	

.PHONY: kdl
kdl:	configured
	cd $(SVN_DIR)/build && make $(PR2_JOBS) && make python && make install

clean:
	rm -rf $(SVN_DIR)/build
	rm -rf kdl configured

wipe: 	clean
	rm -rf $(SVN_DIR) 
