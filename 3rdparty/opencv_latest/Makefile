INSTALL_DIR = opencv
all: $(INSTALL_DIR)

SVN_DIR = build/opencv-svn
SVN_URL = https://opencvlibrary.svn.sourceforge.net/svnroot/opencvlibrary/tags/latest_tested_snapshot/opencv
include $(shell rospack find mk)/svn_checkout.mk

CMAKE = cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE=RELEASE \
             -D CMAKE_INSTALL_PREFIX=$(PWD)/$(INSTALL_DIR) \
             -D BUILD_PYTHON_SUPPORT=OFF

configured: Makefile
	#@if [ ! -d /usr/include/gtk-2.0 ]; then echo "you don't appear to have the GTK+2.0 headers. On Ubuntu, this is fixed with 'sudo apt-get install libgtk2.0-dev'"; false; fi
	mkdir -p $(SVN_DIR)/build
	cd $(SVN_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	touch configured

.PHONY: $(INSTALL_DIR)
$(INSTALL_DIR): SVN_UP configured
	cd $(SVN_DIR)/build && make $(PARALLEL_JOBS) && make install
	@echo "patch opencv.pc to pass -Wl,-rpath,-L{libdir}"
	-mv $(PWD)/$(INSTALL_DIR)/lib/pkgconfig/opencv.pc $(PWD)/$(INSTALL_DIR)/lib/pkgconfig/opencv.bak
	sed 's%Libs: -L$${libdir} -lcxcore%Libs: -Wl,-rpath,$${libdir} -L$${libdir} -lcxcore%g' $(PWD)/$(INSTALL_DIR)/lib/pkgconfig/opencv.bak > $(PWD)/$(INSTALL_DIR)/lib/pkgconfig/opencv.pc

clean:
	-cd $(SVN_DIR)/build && make clean
	rm -rf $(INSTALL_DIR) configured

wipe: clean
	rm -rf build

.PHONY : clean download
