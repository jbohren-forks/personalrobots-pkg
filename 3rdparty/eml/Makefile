all: eml

EML_REVISION = 36

SVN_DIR = eml-svn
SVN_URL = svn://svn.berlios.de/ethercatmaster/trunk 
SVN_REVISION = -r $(EML_REVISION)
SVN_PATCH = eml-r$(EML_REVISION).patch
include $(shell rospack find mk)/svn_checkout.mk

eml: SVN_UP
	cd eml-svn && \
	mkdir -p build && \
	cd build && \
	`rospack find cmake`/cmake/bin/cmake -D RTNET_INSTALL_PATH=`rospack find rtnet`/rtnet -D XENOMAI_INSTALL_PATH=`rospack find xenomai`/xenomai -D CMAKE_INSTALL_PREFIX=`rospack find eml`/eml .. && \
	make install

clean:
	cd eml-svn/build && make clean
	
wipe:
	rm -rf eml eml-svn

