all: eml

EML_REVISION = -r 36

download: eml-checkout

eml-checkout:
	@if [ ! -d eml-svn ]; then svn co $(EML_REVISION) svn://svn.berlios.de/ethercatmaster/trunk eml-svn; fi
	-cd eml-svn && svn up $(EML_REVISION)

eml: eml-checkout
	cd eml-svn && \
	mkdir -p build && \
	cd build && \
	`rospack find cmake`/cmake/bin/cmake -D RTNET_INSTALL_PATH=`rospack find rtnet`/rtnet -D XENOMAI_INSTALL_PATH=`rospack find xenomai`/xenomai -D CMAKE_INSTALL_PREFIX=`rospack find eml`/eml .. && \
	make install

clean:
	cd eml-svn/build && make clean
	
wipe:
	rm -rf eml eml-svn

