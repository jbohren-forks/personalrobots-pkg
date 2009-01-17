all: installed

SOURCE_DIR = kdl-willow
CMAKE = cmake 
BOOST_ROOT =$(shell rospack find boost)/boost
CMAKE_ARGS = -DCMAKE_INSTALL_PREFIX=$(PWD)/kdl/ \
		-DPYTHON_BINDINGS=OFF \
		-DKDL_PYTHON_INSTALL_PATH=$(PWD)/kdl/lib/ \
		-DBOOST_INCLUDEDIR:STRING=$(BOOST_ROOT)/include \
		-DBOOST_LIBRARYDIR:STRING=$(BOOST_ROOT)/lib

installed: wiped 
	-mkdir -p $(SOURCE_DIR)/build
	cd $(SOURCE_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SOURCE_DIR)/build && make $(PARALLEL_JOBS) && make install
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	rm -rf installed

wipe: 	clean
	rm -rf $(SOURCE_DIR)/build
