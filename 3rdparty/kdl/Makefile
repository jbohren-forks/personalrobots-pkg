all: installed

SOURCE_DIR = kdl-willow
CMAKE = cmake 
BOOST_INCLUDE_DIR = $(shell rosboost-cfg --include_dirs)
BOOST_LIBRARY_DIR = $(shell rosboost-cfg --library_dirs)
CMAKE_ARGS = -DCMAKE_INSTALL_PREFIX=$(PWD)/kdl/ \
		-DPYTHON_BINDINGS=OFF \
		-DKDL_PYTHON_INSTALL_PATH=$(PWD)/kdl/lib/ \
		-DBOOST_INCLUDEDIR:STRING=$(BOOST_INCLUDE_DIR) \
		-DBOOST_LIBRARYDIR:STRING=$(BOOST_LIBRARY_DIR)

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
